"""Run with python -m unittest discover -s sim -p test_lock_recovery.py."""
import unittest
from statistics import median
from unittest.mock import patch

from antenna_model import Axes, DriveTest, FlightTest, MIN_HEIGHT_GAP_M, MODES, Pose, Simulation, beam_direction, off_axis, wrap_diff


class LockRecoveryTest(unittest.TestCase):
    def setUp(self):
        self.sim = Simulation()
        self.addCleanup(self.sim.close)

    def advance(self, ticks):
        for _ in range(ticks):
            self.sim.step()

    def test_call_snapshot_preserves_init_and_actual_lora_inputs(self):
        s = self.sim
        first = s.call_snapshot()
        self.assertEqual(first.now, 0)
        self.assertFalse(first.initial_history.local.fix.valid)
        self.assertFalse(first.initial_config.fast_acquire)
        self.assertTrue(first.startup_config.fast_acquire)
        old_lat = first.ground.link.peer.lat_deg
        s.air.lat += 0.001
        self.advance(10)
        tick = s.call_snapshot()
        self.assertEqual(tick.now, 100)
        self.assertEqual(tick.air.link.local.lat_deg, s.air.lat)
        self.assertEqual((tick.ground.link.peer.lat_deg, tick.ground.link.peer.stamp_ms), (old_lat, 0))
        self.assertEqual(tick.ground.link.actual.az_deg, s.axes[1].az)
        self.assertEqual(tick.ground.rssi, s.level)
        self.assertEqual(tick.tick_config.acquire_level, s.out.acquire)
        self.assertEqual(bytes(tick.air_target), bytes(s.out.air))
        self.assertEqual(bytes(tick.ground_target), bytes(s.out.ground))
        self.advance(89)
        previous = s.call_snapshot()
        s.step()
        current = s.call_snapshot()
        self.assertEqual((current.ground.link.peer.lat_deg, current.ground.link.peer.stamp_ms), (s.air.lat, 1000))
        self.assertEqual(bytes(current.air.request), bytes(previous.request))
        self.assertEqual(bytes(current.ground.air), bytes(previous.reply))
        self.assertEqual(current.phase_before, previous.phase_after)
        self.assertEqual(current.phase_after, s.out.phase)
        self.assertEqual(bytes(current.initial_config), bytes(first.initial_config))

    def wait_for(self, predicate, seconds):
        for _ in range(seconds * 100):
            self.sim.step()
            if predicate():
                return
        self.fail("condition timed out at simulation time %.2f s" % (self.sim.ms / 1000))

    def test_detectable_screenshot_sidelobe_cannot_hold_or_relock(self):
        s = self.sim
        s.air = Pose(31.233600, 121.481880, 155, 215)
        s.ground = Pose(31.229135, 121.474160, 30, 22)
        self.wait_for(lambda: s.first_lock_ms is not None, 180)
        for axis, az, el in zip(s.axes, (21.06, 53.57), (8.03, 9.97)):
            axis.az, axis.el = az, el
            axis.az_speed = axis.el_speed = 0
        s.step()
        self.assertAlmostEqual(s.errors[1], 19.37, places=2)
        self.assertLess(abs(s.level + 86.14), 0.17)
        self.assertTrue(s.detected)
        self.assertFalse(s.aligned_lock)
        self.assertLess(s.level, s.out.release)
        self.wait_for(lambda: s.loss_start_ms is not None, 1)
        self.advance(1000)
        self.assertFalse(s.out.holding)
        self.assertIsNotNone(s.loss_start_ms)
        self.assertFalse(s.relocks)
        for axis in s.axes:
            axis.az_speed = axis.el_speed = 5
        self.wait_for(lambda: len(s.relocks) == 1, 240)
        self.assertTrue(s.aligned_lock)

    def test_strong_first_peak_and_stable_tracking(self):
        s = self.sim
        self.wait_for(lambda: s.first_lock_ms is not None, 25)
        self.assertIn("CAL_AIR", s.state_paths["phase"])
        self.assertNotIn("VERIFY_PEAK", s.state_paths["phase"])
        path = tuple(s.state_paths["phase"])
        self.advance(18000)
        self.assertEqual(tuple(s.state_paths["phase"]), path)
        self.assertTrue(s.aligned_lock)

    def test_tracking_stats_include_acquisition_and_restart_from_current_state(self):
        s = self.sim
        self.assertEqual(s.tracking_stats_seconds, (0, 0))
        self.advance(100)
        self.assertEqual(s.tracking_stats_seconds, (0, 1))
        self.wait_for(lambda: s.first_lock_ms is not None, 30)
        self.assertGreater(s.tracking_stats_seconds[0], 0)
        clock_before_clear = s.ms
        output_before_clear = bytes(s.out)
        s.clear_tracking_stats()
        self.assertEqual(s.tracking_stats_seconds, (0, 0))
        self.assertEqual((s.ms, bytes(s.out)), (clock_before_clear, output_before_clear))
        self.advance(100)
        self.assertEqual(s.tracking_stats_seconds, (1, 1))
        s.blocked = True
        self.advance(100)
        self.assertEqual(s.tracking_stats_seconds, (1, 2))

    def test_continuous_transverse_motion_without_reacquisition(self):
        s=self.sim
        s.flight=f=FlightTest(10,10,90,15,42,True)
        self.wait_for(lambda: f.ticks==1,30)
        start=s.out.center_az,s.out.center_el,s.out.air.az
        self.advance(8999)
        self.assertEqual((f.ticks,f.track_ticks,f.losses,f.near_returns),(9000,9000,0,0))
        self.assertGreater(wrap_diff(s.out.center_az-start[0]),30)
        self.assertLess(s.out.center_el-start[1],-6)
        self.assertGreater(wrap_diff(s.out.air.az-start[2]),30)
        f.active=False
        last=bytes(s.air)
        self.advance(100)
        self.assertEqual(bytes(s.air),last)

    def test_seeded_random_flight_tracks_through_turns(self):
        s=self.sim
        s.flight=f=FlightTest(2,3,None,15,42,True)
        self.wait_for(lambda: f.ticks==1,30)
        courses=set()
        for _ in range(6):
            courses.add(round(f.course,1))
            self.advance(1500)
        self.assertGreater(len(courses),2)
        self.assertEqual(f.ticks,f.track_ticks)
        self.assertEqual((f.losses,f.near_returns),(0,0))

    def test_tracking_improves_already_qualified_center(self):
        s = self.sim
        s.air = Pose(31.229008, 121.518574, 155, 215)
        s.ground = Pose(31.221155, 121.460413, 30, 22)
        self.wait_for(lambda: s.first_lock_ms is not None, 60)
        s.ground.heading += 1.7
        self.advance(3000)
        scores = []
        for _ in range(6000):
            s.step()
            if MODES[s.out.phase]=="TRACK" and s.out.point==0 and s.out.settling and s.out.samples==4:
                scores.append(s.level)
        self.assertGreater(len(scores), 20)
        self.assertGreater(median(scores), s.nominal-0.08)

    def test_road_tilt_rotates_the_beam_in_body_coordinates(self):
        p = Pose(0, 0, 0, 90)
        axes = Axes(body_pitch=8)
        az, el = beam_direction(p, axes)
        self.assertAlmostEqual(az, 90)
        self.assertAlmostEqual(el, 8)
        axes.az, axes.body_roll, axes.body_pitch = 90, 4, 0
        az, el = beam_direction(p, axes)
        self.assertAlmostEqual(az, 180)
        self.assertAlmostEqual(el, -4)
        self.assertAlmostEqual(off_axis(p, axes, 180, 0), 4)

    def test_dual_motion_over_uneven_road_and_independent_stop(self):
        s = self.sim
        s.flight = FlightTest(2, 3, None, 15, 42, True)
        s.drive = d = DriveTest(2, 4, None, 15, 73, True, .25, .08, 12)
        initial = bytes(s.ground)
        self.advance(100)
        self.assertEqual((bytes(s.ground), d.ticks), (initial, 0))
        self.wait_for(lambda: d.ticks == 1, 30)
        self.advance(8999)
        self.assertEqual(d.ticks, s.flight.ticks)
        self.assertGreater(d.track_ticks, 8800)
        self.assertEqual((d.losses, d.near_returns), (0, 0))
        self.assertGreater(d.distance, 180)
        self.assertGreater(abs(s.axes[1].body_pitch) + abs(s.axes[1].body_roll), .05)
        self.assertGreater(max(p.height for p in d.trail)-min(p.height for p in d.trail), .03)
        d.active = False
        stopped = bytes(s.ground), s.axes[1].body_pitch, s.axes[1].body_roll
        self.advance(100)
        self.assertEqual((bytes(s.ground), s.axes[1].body_pitch, s.axes[1].body_roll), stopped)
        self.assertEqual((d.ticks, s.flight.ticks), (9000, 9100))

    def test_rough_road_exposes_loss_and_flat_road_has_zero_tilt(self):
        s = self.sim
        flat = DriveTest(2, 4, None, 15, 73, True, 1, 0, 12)
        self.assertEqual(flat.road_pose(17.0), (0, 0, 0))
        s.flight = FlightTest(2, 3, None, 15, 42, True)
        s.drive = d = DriveTest(2, 4, None, 15, 73, True, 1, .35, 12)
        self.wait_for(lambda: d.losses > 0, 90)
        self.assertIsNotNone(s.loss_start_ms)
        self.assertLess(d.track_ticks, d.ticks)

    def test_beacon_tracking_during_vehicle_turns_on_uneven_road(self):
        s = self.sim
        s.flight = FlightTest(2, 3, None, 15, 42, True)
        s.drive = DriveTest(2, 4, None, 15, 73, True, 1, .08, 12)
        self.wait_for(lambda: s.drive.ticks == 1, 30)
        self.advance(8999)
        self.assertGreater(s.drive.track_ticks, 8500)
        self.assertEqual((s.drive.losses, s.drive.near_returns), (0, 0))

    def test_simultaneous_climb_turn_and_grade_keep_the_link(self):
        s = self.sim
        self.wait_for(lambda: s.ms == 30000, 31)
        self.assertTrue(s.aligned_lock)
        s.flight = FlightTest(2, 3, None, 15, 42, False, 20, 120)
        s.drive = DriveTest(2, 4, None, 15, 73, False, 1, .08, 12, 6, 400)
        self.advance(18000)
        self.assertEqual((s.flight.losses, s.flight.near_returns), (0, 0))
        self.assertGreater(s.flight.track_ticks, 17700)

    def test_opposing_routes_turn_back_and_change_height(self):
        s = self.sim
        s.reset_motion_area(1000)
        s.flight = FlightTest(35, 35, 45, 15, 42, False, 20, 120)
        s.drive = DriveTest(15, 15, 225, 15, 73, False, 8, .08, 12, 6, 400)
        routes = (s.flight, s.drive)
        completed_turns = [0, 0]
        air_heights, ground_heights = [], []
        for tick in range(18000):
            returning = [r.returning for r in routes]
            s.step()
            self.assertLessEqual(s.distance, 1000)
            for i, run in enumerate(routes):
                completed_turns[i] += int(returning[i] and not run.returning)
                self.assertTrue(run.active)
            if tick % 100 == 0:
                air_heights.append(s.air.height)
                ground_heights.append(s.ground.height)
        self.assertGreater(min(completed_turns), 0)
        self.assertGreater(max(air_heights)-min(air_heights), 39)
        self.assertGreater(max(ground_heights)-min(ground_heights), 11)

    def test_invalid_range_and_manual_move_do_not_teleport_stations(self):
        s = self.sim
        with self.assertRaises(ValueError):
            s.reset_motion_area(100)
        s.flight = FlightTest(3, 3, None, 15, 42, False, 20, 120)
        s.step()
        s.ground.lon += 1
        before = bytes(s.air), bytes(s.ground)
        s.step()
        self.assertFalse(s.flight.active)
        self.assertTrue(s.flight.stop_reason)
        self.assertEqual((bytes(s.air), bytes(s.ground)), before)

    def test_descending_aircraft_and_uphill_vehicle_keep_positive_height_gap(self):
        s = self.sim
        s.air.height = s.ground.height + 0.05
        s.reset_motion_area(1000)
        s.flight = FlightTest(0, 0, 0, 15, 42, False, 2, 4)
        s.drive = DriveTest(4, 4, s.ground.heading, 15, 73, False, 1, .08, 12, 8, 160)
        reached_floor = False
        for _ in range(1200):
            s.step()
            self.assertGreaterEqual(s.air.height, s.ground.height + MIN_HEIGHT_GAP_M)
            self.assertLessEqual(s.distance, 1000)
            reached_floor |= s.air.height == s.ground.height + MIN_HEIGHT_GAP_M
        self.assertTrue(reached_floor)
        self.assertGreater(s.ground.height, 35)
        self.assertGreater(s.drive.distance, 47)

    def test_uphill_vehicle_waits_below_stationary_aircraft(self):
        s = self.sim
        s.air.height = s.ground.height + .25
        parked = bytes(s.air)
        s.drive = DriveTest(4, 4, 22, 15, 73, False, 1, 0, 12, 8, 160)
        self.advance(300)
        self.assertEqual(bytes(s.air), parked)
        self.assertGreater(s.air.height, s.ground.height)
        self.assertEqual(s.drive.actual_speed, 0)
        self.assertGreater(s.drive.distance, 0)
        s.air.height += 3
        self.advance(10)
        self.assertGreater(s.drive.actual_speed, 0)

    def test_nearby_moving_aircraft_relocks_after_one_second_blackout(self):
        s = self.sim
        s.air = Pose(31.233159, 121.476924, 155, 215)
        s.ground = Pose(31.234376, 121.477592, 30, 29.06)
        air, ground = s.axes
        air.az_speed = ground.az_speed = 25
        air.az, air.el = 170.28, 39.97
        ground.az, ground.el = 160.30, 35.74
        ground.body_pitch, ground.body_roll = -.96, .59
        s.flight = FlightTest(4.9, 4.9, 249, 15, 42, True)
        self.wait_for(lambda: s.flight.ticks == 1000, 45)
        s.blocked = True
        self.advance(100)
        self.assertFalse(s.aligned_lock)
        self.assertIsNotNone(s.loss_start_ms)
        s.blocked = False
        self.wait_for(lambda: len(s.relocks) == 1, 8)
        self.assertTrue(s.aligned_lock)
        self.assertGreaterEqual(s.level, s.out.acquire)
        self.advance(9000)
        self.assertEqual(s.flight.losses, 1)
        self.assertTrue(s.aligned_lock)

    def test_loss_of_peak_triggers_tx_probe_while_above_lock_gate(self):
        s = self.sim
        self.wait_for(lambda: s.first_lock_ms is not None, 25)
        def biased_beam(pose, axis, bearing, elevation):
            return off_axis(pose, axis, bearing + (3.5 if pose is s.air else 0), elevation)
        with patch("antenna_model.off_axis", biased_beam):
            s.step()
            self.assertGreater(s.level, s.out.acquire)
            self.wait_for(lambda: s.out.reason==17, 25)
            self.wait_for(lambda: MODES[s.out.phase]=="TRACK", 80)
            self.assertLess(s.errors[0], 1)
            self.assertFalse(s.relocks)

    def test_weak_detected_beacon_preempts_air_calibration(self):
        s = self.sim
        self.wait_for(lambda: MODES[s.out.phase] == "CAL_AIR", 25)
        s.air = Pose(31.231624, 121.472407, 208.09, 215)
        s.ground = Pose(31.231729, 121.473190, -7.50, 22)
        for axis, az, el in zip(s.axes, (226.06, 49.27), (68.69, 13.76)):
            axis.az, axis.el = az, el
            axis.az_speed = axis.el_speed = 0
        onset = s.ms
        self.wait_for(lambda: s.out.reason == 14, 1)
        self.assertTrue(s.detected)
        self.assertAlmostEqual(s.errors[1], 95.27, places=2)
        self.assertEqual((MODES[s.out.phase], s.ms-10-onset), ("NEAR", 250))

    def test_continuous_blackout_allows_complete_sweep(self):
        s = self.sim
        self.wait_for(lambda: s.first_lock_ms is not None, 25)
        s.blocked = True
        self.wait_for(lambda: MODES[s.out.phase] == "SWEEP", 10)
        for _ in range(11000):
            s.step()
            self.assertEqual(MODES[s.out.phase], "SWEEP")
            if s.out.point == 35:
                break
        self.assertEqual(s.out.point, 35)

    def test_geometric_reacquisition_after_large_vehicle_turn(self):
        s = self.sim
        while s.ms < 100000:
            s.step()
        s.ground.heading += 60
        self.wait_for(lambda: len(s.relocks) == 1, 35)
        self.assertTrue(s.aligned_lock)
        self.assertIn("NEAR", s.state_paths["phase"])
        self.assertNotIn("SWEEP", s.state_paths["phase"])

    def test_outage_timer_and_short_dips(self):
        s = self.sim
        self.wait_for(lambda: s.first_lock_ms is not None, 180)
        self.advance(9000)
        self.assertFalse(s.relocks)
        self.assertIsNone(s.loss_start_ms)
        s.blocked = True
        self.advance(10)
        s.blocked = False
        self.advance(150)
        self.assertIsNone(s.loss_start_ms)
        for number, ticks in enumerate((100, 500), 1):
            onset = s.ms
            s.blocked = True
            self.advance(ticks)
            self.assertEqual(s.loss_start_ms, onset + 250)
            s.blocked = False
            self.wait_for(lambda: len(s.relocks) == number, 40)
            start, end = s.relocks[-1]
            self.assertEqual(start, onset + 250)
            self.assertEqual(end, s.ms - 10)
            self.assertTrue(s.aligned_lock)
            self.assertIsNone(s.loss_start_ms)


if __name__ == "__main__":
    unittest.main()

"""Run with python -m unittest discover -s sim -p test_lock_recovery.py."""
import unittest
from statistics import median
from unittest.mock import patch

from antenna_model import Axes, DriveTest, FlightTest, MODES, Pose, Simulation, beam_direction, off_axis, wrap_diff


class LockRecoveryTest(unittest.TestCase):
    def setUp(self):
        self.sim = Simulation()
        self.addCleanup(self.sim.close)

    def advance(self, ticks):
        for _ in range(ticks):
            self.sim.step()

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

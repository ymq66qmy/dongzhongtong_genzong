"""10 ms antenna plant and ctypes binding for the STM32 pointing controllers."""
import ctypes as C
from collections import deque
from dataclasses import dataclass
import hashlib
import math
import random
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
SOURCES = ["common/link.c", "airborne/airborne.c", "vehicle/vehicle.c", "sim/control_bridge.c"]
HEADERS = ["common/link.h", "airborne/airborne.h", "vehicle/vehicle.h"]
MODES = ("IDLE", "WAIT_DATA", "FAULT", "POINT", "SWEEP", "REFINE", "TRACK",
         "RECOVER", "CAL_AIR", "NEAR", "VERIFY_PEAK")
DT = 0.01
SERVO_SPEED = 5.0
BEAM_HALF_DEG = 6.0


class Pose(C.Structure):
    _fields_ = [(name, C.c_double) for name in ("lat", "lon", "height", "heading")]


class Target(C.Structure):
    _fields_ = [("az", C.c_double), ("el", C.c_double), ("valid", C.c_int), ("mode", C.c_int)]


class Readout(C.Structure):
    _fields_ = [("air", Target), ("ground", Target)] + [
        (name, C.c_int) for name in ("phase", "point", "samples", "ack", "settling",
                                      "confirm", "rx_probe", "reason", "holding")] + [
        ("acquire", C.c_double), ("release", C.c_double), ("severe", C.c_double),
        ("severe_ms", C.c_uint32), ("ack_timeout_ms", C.c_uint32),
        ("track_center", C.c_double), ("track_best", C.c_double),
        ("center_az", C.c_double), ("center_el", C.c_double)]


def controller_library():
    digest = hashlib.sha256((platform.system()+platform.machine()+str(C.sizeof(C.c_void_p))).encode())
    for name in SOURCES + HEADERS:
        digest.update((ROOT / name).read_bytes())
    cache = Path(os.getenv("LOCALAPPDATA") or tempfile.gettempdir()) / "AntennaSimulator"
    cache.mkdir(parents=True, exist_ok=True)
    suffix = ".dll" if os.name == "nt" else ".so"
    library = cache / ("controller-" + digest.hexdigest()[:20] + suffix)
    if not library.exists():
        candidates = [os.environ.get("CC"), shutil.which("gcc"), shutil.which("clang"),
                      str(Path(sys.prefix).parent / "CLion/bin/mingw/bin/gcc.exe")]
        cc = next((shutil.which(x) for x in candidates if x and shutil.which(x)), None)
        if not cc:
            raise RuntimeError("首次启动需要主机 GCC 或 Clang。请将编译器加入 PATH，或设置 CC 为其完整路径。")
        # Build beside the final path so an interrupted or concurrent build cannot
        # publish a partial library. Do not change the user's working directory.
        with tempfile.TemporaryDirectory(dir=cache) as build:
            output = Path(build) / library.name
            flags = ["-shared", "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror"]
            flags += ["-static-libgcc"] if os.name == "nt" else ["-fPIC"]
            env = os.environ.copy()
            env["PATH"] = str(Path(cc).parent) + os.pathsep + env.get("PATH", "")
            result = subprocess.run([cc, *flags, *SOURCES, "-lm", "-o", str(output)],
                                    cwd=ROOT, env=env, capture_output=True)
            if result.returncode:
                raise RuntimeError(result.stderr.decode(errors="replace"))
            if not library.exists():
                try:
                    os.replace(output, library)
                except PermissionError:
                    if not library.exists():
                        raise
    lib = C.CDLL(str(library))
    lib.control_new.argtypes = []
    lib.control_new.restype = C.c_void_p
    lib.control_free.argtypes = [C.c_void_p]
    lib.control_free.restype = None
    lib.control_thresholds.argtypes = [C.c_void_p, C.c_int, C.c_double, C.c_double,
                                      C.c_double, C.c_double, C.POINTER(Readout)]
    lib.control_thresholds.restype = C.c_int
    lib.control_step.argtypes = [C.c_void_p, C.c_uint32, C.POINTER(Pose), C.POINTER(Pose),
                                 C.POINTER(C.c_double), C.c_double, C.c_int, C.c_double, C.c_int, C.POINTER(Readout)]
    lib.control_step.restype = None
    return lib


def wrap_diff(angle):
    return (angle + 180.0) % 360.0 - 180.0


@dataclass
class Axes:
    az: float = 0.0
    el: float = 0.0
    az_speed: float = SERVO_SPEED
    el_speed: float = SERVO_SPEED
    down_positive: bool = False
    body_pitch: float = 0.0  # simulator truth, nose up
    body_roll: float = 0.0   # simulator truth, right side down

    @property
    def beam_elevation(self):
        """Signed geometric elevation of the beam; encoder pitch stays in [0, 90]."""
        return -self.el if self.down_positive else self.el

    def follow(self, target):
        if target.valid:
            az_step = self.az_speed * DT
            el_step = self.el_speed * DT
            self.az = (self.az + max(-az_step, min(az_step, wrap_diff(target.az-self.az)))) % 360
            self.el += max(-el_step, min(el_step, target.el-self.el))


def ecef(p):
    lat, lon = map(math.radians, (p.lat, p.lon))
    ecc = 0.0066943799901413165
    n = 6378137.0 / math.sqrt(1-ecc*math.sin(lat)**2)
    r = (n+p.height)*math.cos(lat)
    return r*math.cos(lon), r*math.sin(lon), (n*(1-ecc)+p.height)*math.sin(lat)


def view(p, delta):
    lat, lon = map(math.radians, (p.lat, p.lon))
    x, y, z = delta
    radial = math.cos(lon)*x + math.sin(lon)*y
    east = -math.sin(lon)*x + math.cos(lon)*y
    north = -math.sin(lat)*radial + math.cos(lat)*z
    up = math.cos(lat)*radial + math.sin(lat)*z
    return math.degrees(math.atan2(east, north)) % 360, math.degrees(math.atan2(up, math.hypot(east, north)))


def beam_direction(pose, axes):
    if not (axes.body_pitch or axes.body_roll):
        return (pose.heading + axes.az) % 360, axes.beam_elevation
    az, el, pitch, roll = map(math.radians, (axes.az, axes.beam_elevation,
                                             axes.body_pitch, axes.body_roll))
    forward, right, up = math.cos(el)*math.cos(az), math.cos(el)*math.sin(az), math.sin(el)
    right, up = right*math.cos(roll)+up*math.sin(roll), up*math.cos(roll)-right*math.sin(roll)
    forward, up = forward*math.cos(pitch)-up*math.sin(pitch), forward*math.sin(pitch)+up*math.cos(pitch)
    return ((pose.heading+math.degrees(math.atan2(right, forward))) % 360,
            math.degrees(math.atan2(up, math.hypot(forward, right))))


def off_axis(pose, axes, bearing, elevation):
    az, el = beam_direction(pose, axes)
    a, b = math.radians(el), math.radians(elevation)
    cos_error = math.sin(a)*math.sin(b) + math.cos(a)*math.cos(b)*math.cos(
        math.radians(az-bearing))
    return math.degrees(math.acos(max(-1, min(1, cos_error))))


def pattern_gain(off):
    return max(-60.0, -3*(off/BEAM_HALF_DEG)**2, -19.0-0.6*(off-19)**2)


def change_reason(previous, current):
    a, b = MODES[previous.phase], MODES[current.phase]
    if not current.ground.valid:
        return "导航数据或几何视线无效" if current.ground.mode == 1 else "设备未就绪"
    special = {1:"最细步长稳定，扩大范围复核候选峰", 2:"精扫已达轮数上限，扩大范围复核",
               3:"网格发现更强点，移心重新精扫", 4:"双端回峰复测未满足锁定条件",
               5:"机端最细档峰值未达锁定门限", 6:"车端精扫尚未收敛，再次精扫",
               7:"机端校正达到轮数上限，重新复核", 8:"车端小十字确认强峰，双端回峰复测",
               9:"信标恢复，回到上次峰位重新测量一次", 10:"连续 250 ms 低于保持门限或未检出，进入局部重捕",
               11:"车端强峰已稳定，开始机端双轴探测", 12:"局部网格未找到可用候选，按最新位置重新对准",
               13:"机端探测未得到合格候选，进入局部重捕",
               14:f"连续 {current.severe_ms} ms 信标缺失或强度 ≤ {current.severe:.1f}，重新几何对准",
               15:f"连续 {current.ack_timeout_ms/1000:g} s 未匹配到当前点的机端回执，重新几何对准",
               16:"收到重新对准请求，按最新位置重新互指",
               17:"车端连续两轮峰心强度低于已验证峰值，复测机端方向"}
    if current.reason in special:
        return special[current.reason]
    if a != b:
        if b == "REFINE":
            return "局部网格重新检出信标" if a == "RECOVER" else "双方到位，在当前方向检出信标"
        if b == "CAL_AIR":
            return "达到复校周期且信号合格，复校机端" if a == "TRACK" else "网格未发现更强点，开始机端校正"
        if b == "RECOVER":
            return "连续弱信号轮数达到失锁判据" if a == "TRACK" else "本轮无有效信标，进入局部重捕"
        return {"NEAR":"重捕网格无信标，重新按位置对准", "SWEEP":"几何方向未检出信标",
                "TRACK":"双端回峰到位，完整复测窗口均分达到锁定门限"}.get(b, "控制阶段改变")
    if not previous.ground.valid:
        return "导航数据恢复，重新对准"
    return "本点机端回执已匹配" if current.ack else "等待本次机端修正的到位回执"


class FlightTest:
    """Generate physical positions on the simulation clock. No velocity enters the controllers."""
    def __init__(self, low, high, course, interval, seed, wait_lock):
        if not (0<=low<=high<=200 and 0.1<=interval<=3600 and
                (course is None or math.isfinite(course))):
            raise ValueError("速度范围 0～200 m/s，最小值须小于等于最大值，航段为 0.1～3600 s。")
        self.low,self.high,self.fixed_course=low,high,course
        self.segment_ticks=round(interval/DT)
        self.rng=random.Random(seed)
        self.waiting=wait_lock
        self.active=True
        self.ticks=self.track_ticks=self.losses=self.near_returns=0
        self.speed=self.course=0.0
        self.trail=deque(maxlen=600)

    def platform(self, sim):
        return sim.air

    def initial_course(self, pose):
        return self.target_course

    def turn_rate(self):
        return 10.0

    def move(self, sim):
        if self.waiting:
            self.waiting=not sim.aligned_lock or MODES[sim.out.phase]!="TRACK"
            if self.waiting:return
        p=self.platform(sim)
        if self.ticks%self.segment_ticks==0:
            self.target_course=self.rng.uniform(0,360) if self.fixed_course is None else self.fixed_course%360
            self.target_speed=self.rng.uniform(self.low,self.high)
            if self.ticks==0:
                self.course,self.speed=self.initial_course(p),self.target_speed
        turn=self.turn_rate()*DT
        self.course=(self.course+max(-turn,min(turn,wrap_diff(self.target_course-self.course))))%360
        self.speed+=max(-2*DT,min(2*DT,self.target_speed-self.speed))
        lat,brg=map(math.radians,(p.lat,self.course))
        arc=self.speed*DT/6371000
        sinlat=math.sin(lat)*math.cos(arc)+math.cos(lat)*math.sin(arc)*math.cos(brg)
        newlat=math.asin(max(-1,min(1,sinlat)))
        dlon=math.atan2(math.sin(brg)*math.sin(arc)*math.cos(lat),math.cos(arc)-math.sin(lat)*sinlat)
        if self.ticks%50==0:self.trail.append(Pose(p.lat,p.lon,p.height,p.heading))
        p.lat,p.lon=math.degrees(newlat),wrap_diff(p.lon+math.degrees(dlon))
        self.ticks+=1


class DriveTest(FlightTest):
    """Rigid chassis following the average grades under four wheel contact points."""
    WHEELBASE = 2.8
    TRACK_WIDTH = 1.6

    def __init__(self, low, high, course, interval, seed, wait_lock, yaw_rate, amplitude, wavelength):
        super().__init__(low, high, course, interval, seed, wait_lock)
        if not (0 < yaw_rate <= 90 and 0 <= amplitude <= 2 and 1 <= wavelength <= 500):
            raise ValueError("转向限速须在 (0, 90] °/s，路面幅值 0～2 m，波长 1～500 m。")
        self.yaw_rate, self.amplitude, self.wavelength = yaw_rate, amplitude, wavelength
        self.distance = 0.0
        self.heave = 0.0
        self.start_pitch = self.start_roll = None
        # Keep the road independent of segment selection and speed settings.
        road_rng = random.Random(seed+1)
        self.phases = [road_rng.uniform(0, math.tau) for _ in range(3)]
        self.initial_height, _, _ = self.road_pose(0.0)

    def road_pose(self, distance):
        def axle(s):
            phase = math.tau*s/self.wavelength
            a,b,c = self.phases
            common = .58*math.sin(phase+a) + .20*math.sin(2.3*phase+b)
            cross = .22*math.sin(1.7*phase+c)
            return self.amplitude*common, self.amplitude*cross
        front, front_cross = axle(distance+self.WHEELBASE/2)
        rear, rear_cross = axle(distance-self.WHEELBASE/2)
        return ((front+rear)/2, math.degrees(math.atan2(front-rear, self.WHEELBASE)),
                math.degrees(math.atan2(front_cross+rear_cross, self.TRACK_WIDTH)))

    def platform(self, sim):
        return sim.ground

    def initial_course(self, pose):
        return pose.heading

    def turn_rate(self):
        return min(self.yaw_rate, math.degrees(self.speed/5.0))  # minimum turn radius 5 m

    def move(self, sim):
        prior = self.ticks
        super().move(sim)
        if self.ticks == prior or self.speed == 0:
            return
        ground, axes = sim.ground, sim.axes[1]
        if self.start_pitch is None:
            self.start_pitch, self.start_roll = axes.body_pitch, axes.body_roll
        self.distance += self.speed*DT
        height, pitch, roll = self.road_pose(self.distance)
        # Enter the selected road from the current chassis attitude.
        blend = min(1.0, self.distance/self.WHEELBASE)
        heave = (height-self.initial_height)*blend
        ground.height += heave-self.heave
        self.heave = heave
        axes.body_pitch = self.start_pitch+(pitch-self.start_pitch)*blend
        axes.body_roll = self.start_roll+(roll-self.start_roll)*blend
        ground.heading = self.course


class Simulation:
    def __init__(self):
        self.lib = controller_library()
        self.handle = self.lib.control_new()
        if not self.handle:
            raise MemoryError("无法创建仿真实例")
        self.air = Pose(31.2330, 121.4780, 155, 215)
        self.ground = Pose(31.2294, 121.4748, 30, 22)
        self.axes = [Axes(down_positive=True), Axes()]
        self.out = Readout()
        self.ms = 0
        self.blocked = False
        self.fixed_thresholds = None
        self.realign_pending = False
        self.events = []
        self.flight = None
        self.drive = None
        self.first_lock_ms = None
        self.loss_start_ms = None
        self.relocks = []  # (loss confirmed, lock confirmed), simulation milliseconds
        self.state_paths = {key: deque(maxlen=8) for key in ("phase", "air", "ground")}
        self.step()

    def close(self):
        if self.handle:
            self.lib.control_free(self.handle)
            self.handle = None

    def set_thresholds(self, levels):
        """None selects the range budget; fixed levels are detect, severe, hold, lock."""
        if levels == self.fixed_thresholds:
            return
        args = levels if levels is not None else (0, 0, 0, 0)
        if not self.lib.control_thresholds(self.handle, levels is not None, *args, self.out):
            raise ValueError("范围为 −120～0，须满足极弱 < 保持 < 锁定，且检出 ≤ 保持。")
        self.fixed_thresholds = levels
        self.measure_link()
        self.realign_pending = True

    def measure_link(self):
        delta = tuple(a-b for a, b in zip(ecef(self.air), ecef(self.ground)))
        self.distance = math.hypot(*delta)
        gview = view(self.ground, delta)
        aview = view(self.air, tuple(-d for d in delta))
        self.errors = (off_axis(self.air, self.axes[0], *aview),
                       off_axis(self.ground, self.axes[1], *gview))
        self.in_beams = self.distance >= 1 and max(self.errors) <= BEAM_HALF_DEG
        self.nominal = min(-40.0, -62.0 - 20*math.log10(max(self.distance, 1)/500.0))
        level = (self.nominal + sum(pattern_gain(e) for e in self.errors)
                 + 0.15*math.sin(self.ms*0.047))
        self.level = -120.0 if self.blocked else max(-120.0, min(0.0, level))
        detect = self.fixed_thresholds[0] if self.fixed_thresholds is not None else -116
        self.detected = not self.blocked and self.distance >= 1 and self.level >= detect

    @property
    def aligned_lock(self):
        return (self.out.holding and self.out.air.valid and self.out.ground.valid and
                self.in_beams and self.detected and self.level >= self.out.release)

    def step(self):
        prev = self.out
        runs = [r for r in (self.flight, self.drive) if r and r.active]
        for run in runs:
            run.move(self)
        for axis, target in zip(self.axes, (prev.air, prev.ground)):
            axis.follow(target)
        self.measure_link()
        readings = (C.c_double*4)(self.axes[0].az, self.axes[0].el, self.axes[1].az, self.axes[1].el)
        self.out = Readout()
        self.lib.control_step(self.handle, self.ms, self.air, self.ground, readings,
                              self.level, self.detected, self.nominal, self.realign_pending, self.out)
        self.realign_pending = False
        for run in runs:
            if not run.waiting:
                run.track_ticks+=int(self.out.phase==MODES.index("TRACK") and self.aligned_lock)
                run.near_returns+=int(self.out.phase==MODES.index("NEAR") and self.out.phase!=prev.phase)
                run.losses+=int(bool(prev.holding) and not self.out.holding)
        # Timing follows controller loss declarations, including loss during a
        # periodic calibration. Only a verified return to TRACK closes an outage.
        if self.first_lock_ms is not None and not self.out.holding and self.loss_start_ms is None:
            self.loss_start_ms = self.ms
        if self.out.phase == MODES.index("TRACK") and self.aligned_lock:
            if self.first_lock_ms is None:
                self.first_lock_ms = self.ms
            elif self.loss_start_ms is not None:
                self.relocks.append((self.loss_start_ms, self.ms))
                self.loss_start_ms = None
        # Capture transitions on controller ticks, even between two GUI refreshes.
        modes = (self.out.phase if self.out.ground.valid else self.out.ground.mode,
                 self.out.air.mode, self.out.ground.mode)
        for key, mode in zip(("phase", "air", "ground"), modes):
            path = self.state_paths[key]
            if not path or path[-1] != MODES[mode]:
                path.append(MODES[mode])
        if self.ms == 0 or self.out.reason in (14,15,16) or (prev.phase, prev.ground.mode) != (self.out.phase, self.out.ground.mode):
            reason = "已使能，几何对准" if self.ms == 0 else change_reason(prev,self.out)
            self.events.append((self.ms/1000, MODES[self.out.phase], MODES[self.out.ground.mode], reason))
            self.events = self.events[-100:]
        self.ms += 10


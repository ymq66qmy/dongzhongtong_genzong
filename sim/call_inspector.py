"""Native host types used by the read-only algorithm call inspector."""
import ctypes as C

D = C.c_double
U = C.c_uint32
B = C.c_bool


def fields(*groups):
    return [(name, kind) for kind, names in groups for name in names.split()]


class Fix(C.Structure):
    _fields_ = fields(
        (D, "lat_deg lon_deg height_m"),
        (U, "stamp_ms"),
        (B, "valid"),
    )


class Velocity(C.Structure):
    _fields_ = fields(
        (D, "east north up"),
        (B, "valid"),
    )


class GimbalAngles(C.Structure):
    _fields_ = fields(
        (D, "az_deg el_deg"),
        (U, "stamp_ms"),
        (B, "valid"),
    )


class GimbalFrame(C.Structure):
    _fields_ = fields(
        (D, "az_zero_deg el_zero_deg"),
        (B, "reverse_az reverse_el"),
    )


class AirRequest(C.Structure):
    _fields_ = fields(
        (U, "session point"),
        (D, "az_trim el_trim"),
    )


class AirReply(C.Structure):
    _fields_ = fields(
        (U, "session point"),
        (B, "arrived"),
    )


class LinkInput(C.Structure):
    _fields_ = fields(
        (Fix, "local peer"),
        (B, "enabled local_ok peer_ok"),
        (U, "peer_status_ms"),
        (GimbalAngles, "actual"),
        (GimbalFrame, "frame"),
        (U, "lead_ms"),
    )


class MotionSample(C.Structure):
    _fields_ = fields(
        (Fix, "fix"),
        (Velocity, "estimate"),
    )


class AirHistory(C.Structure):
    _fields_ = fields(
        (MotionSample, "local peer"),
    )


class AirInput(C.Structure):
    _fields_ = fields(
        (LinkInput, "link"),
        (D, "true_heading_deg"),
        (U, "heading_ms"),
        (B, "heading_valid"),
        (D, "pitch_deg roll_deg"),
        (B, "rates_valid"),
        (D, "heading_rate pitch_rate roll_rate"),
        (AirRequest, "request"),
        (U, "request_rx_ms"),
    )


class VehicleConfig(C.Structure):
    _fields_ = fields(
        (D, "sweep_step_deg row_step_deg"),
        (D, "acquire_level release_level peak_margin"),
        (D, "refine_start_deg refine_min_deg refine_factor"),
        (C.c_uint, "refine_max_rounds weak_rounds"),
        (D, "probe_deg recover_step_deg arrival_tolerance_deg"),
        (C.c_uint, "recover_radius_points sample_count"),
        (U, "settle_ms sample_window_ms move_timeout_ms"),
        (D, "air_search_deg air_refine_deg air_min_deg"),
        (C.c_uint, "air_max_rounds"),
        (U, "air_recheck_ms"),
        (D, "near_step_deg"),
        (D, "detect_level"),
        (B, "fast_relock"),
        (U, "track_weak_ms"),
        (B, "fast_acquire"),
        (D, "realign_drop_db"),
        (U, "realign_weak_ms realign_ack_ms"),
        (D, "pair_margin"),
        (D, "rx_half_power_deg"),
    )


class VehicleInput(C.Structure):
    _fields_ = fields(
        (LinkInput, "link"),
        (B, "rssi_valid"),
        (U, "sample_ms"),
        (D, "rssi"),
        (U, "session"),
        (AirReply, "air"),
        (U, "air_rx_ms"),
        (D, "rough_heading_deg"),
        (U, "heading_ms"),
        (B, "heading_valid"),
        (B, "has_beacon_flag beacon_present"),
        (B, "request_realign"),
    )


class Target(C.Structure):
    _fields_ = fields(
        (D, "az el"),
        (C.c_int, "valid mode"),
    )


class CallSnapshot(C.Structure):
    _fields_ = fields(
        (AirHistory, "initial_history history_before"),
        (VehicleConfig, "initial_config startup_config tick_config"),
        (C.c_int, "initial_phase phase_before"),
        (AirRequest, "initial_request request_before"),
        (U, "now"),
        (AirInput, "air"),
        (VehicleInput, "ground"),
        (Target, "air_target ground_target"),
        (AirReply, "reply"),
        (AirRequest, "request"),
        (C.c_int, "phase_after holding"),
    )


def field_lines(structure, prefix):
    for name, _ in structure._fields_:
        value = getattr(structure, name)
        path = f"{prefix}.{name}"
        if isinstance(value, C.Structure):
            yield from field_lines(value, path)
        else:
            if isinstance(value, bool):
                value = "true" if value else "false"
            elif isinstance(value, float):
                value = f"{value:.10g}"
            yield f"{path} = {value}"


def call_text(snapshot, airborne, initialization, modes):
    s = snapshot
    if initialization:
        if airborne:
            lines = ["airborne_track_init(&history)", "", "输入    history（待初始化的 AirHistory 对象）",
                     "返回    void", "", "初始化后的定位历史", *field_lines(s.initial_history, "history")]
        else:
            lines = ["vehicle_init(&scanner)", "", "输入    scanner（待初始化的 Vehicle 对象）",
                     "返回    void", "", "初始化后的状态", f"scanner.mode = {modes[s.initial_phase]}",
                     *field_lines(s.initial_request, "scanner.request"), "", "仿真启动时覆盖的配置"]
            for name, _ in s.initial_config._fields_:
                before, after = getattr(s.initial_config, name), getattr(s.startup_config, name)
                if before != after:
                    lines.append(f"cfg.{name} = {before} → {after}")
            lines.extend(["", "vehicle_init 写入的默认配置", *field_lines(s.initial_config, "scanner.cfg")])
    else:
        target = s.air_target if airborne else s.ground_target
        incoming = s.air if airborne else s.ground
        link = incoming.link
        signature = ("airborne_track_step(&history, now, &input, &reply)" if airborne else
                     "vehicle_step(&scanner, now, &input)")
        lines = ["cmd = " + signature, f"now = {s.now} ms", "", "本拍输入"]
        for title, fix in (("本端定位", link.local), ("对端 LoRa 定位", link.peer)):
            lines.append(f"{title}    纬 {fix.lat_deg:.7f}°   经 {fix.lon_deg:.7f}°   高 {fix.height_m:.2f} m"
                         f"    采样 {fix.stamp_ms} ms" + ("" if fix.valid else "（无效）"))
        lines.append(f"云台实测    方位 {link.actual.az_deg:.3f}°   俯仰 {link.actual.el_deg:.3f}°"
                     + ("" if link.actual.valid else "（无效）"))
        if airborne:
            lines.append(f"机端航向    {incoming.true_heading_deg:.3f}°"
                         + ("" if incoming.heading_valid else "（无效）"))
        else:
            lines.append(f"信标强度    {incoming.rssi:.2f}    检出标志 {bool(incoming.beacon_present)}"
                         + ("" if incoming.rssi_valid else "（强度无效）"))
        lines.extend(["", "返回    Command（云台目标角与状态）",
                 f"cmd.mode = {modes[target.mode]} ({target.mode})", f"cmd.valid = {bool(target.valid)}",
                 f"cmd.az_deg = {target.az:.4f}°", f"cmd.el_deg = {target.el:.4f}°"])
        if not target.valid:
            lines.append("本拍角度无效")
        if airborne:
            lines.extend(["", "输出    机端到位回执", *field_lines(s.reply, "reply"),
                          "", "输入    AirInput", *field_lines(s.air, "input"),
                          "", "调用前    定位历史", *field_lines(s.history_before, "history")])
        else:
            lines.extend(["", "输出    车端状态和待发修正请求", f"scanner.mode = {modes[s.phase_after]}",
                          f"scanner.monitoring_lock = {bool(s.holding)}", *field_lines(s.request, "scanner.request"),
                          "", "输入    VehicleInput", *field_lines(s.ground, "input"),
                          "", "调用前    车端阶段、请求和生效配置", f"scanner.mode = {modes[s.phase_before]}",
                          *field_lines(s.request_before, "scanner.request"), *field_lines(s.tick_config, "scanner.cfg")])
    return "\n".join(lines)

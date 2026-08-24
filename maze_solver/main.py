"""Main entry point for RoboMaster Trémaux / DFS maze explorer V9."""

import statistics
import time
from robomaster import robot

import config
from controller import MotionController
from exploration import DecisionPointDetector, TremauxExplorer
from navigation import decision_from_relative, execute_turn, print_exploration_decision
from pose_tracker import PoseTracker
from sensors import SensorManager


def fmt(value):
    return "---" if value is None else f"{value:4.1f}"


def stop_chassis(chassis):
    if not config.ENABLE_MOTION or chassis is None:
        return
    chassis.drive_speed(x=0, y=0, z=0, timeout=0.1)


def print_startup_info():
    print()
    print("==========================================================")
    print(" MAZE SOLVER V11 - TRÉMAUX / DFS + SHARP-IR FUSION")
    print("==========================================================")
    print()
    print(f"Forward speed       : {config.FORWARD_SPEED:.2f} m/s")
    print(f"Front Slow          : {config.SLOW_FRONT_CM:.1f} cm")
    print(f"Front Stop          : {config.STOP_FRONT_CM:.1f} cm")
    print(f"Side BLOCKED        : <= {config.SIDE_BLOCKED_MAX_CM:.1f} cm")
    print(
        f"Side BORDERLINE     : {config.SIDE_BLOCKED_MAX_CM:.1f}-"
        f"{config.SIDE_OPEN_MIN_CM:.1f} cm (IR decides / hold if unknown)"
    )
    print(f"Side OPEN           : >= {config.SIDE_OPEN_MIN_CM:.1f} cm")
    print(f"Front Traversable   : >= {config.EXPLORATION_FRONT_OPEN_CM:.1f} cm")
    print(
        f"IR Fusion           : {config.ENABLE_IR_SIDE_FUSION} | "
        f"L=Hub{config.IR_LEFT_ID}/P{config.IR_PORT} "
        f"R=Hub{config.IR_RIGHT_ID}/P{config.IR_PORT}"
    )
    print(
        f"IR WALL level       : L={config.IR_LEFT_WALL_LEVEL} "
        f"R={config.IR_RIGHT_WALL_LEVEL} | "
        f"vote={config.IR_MIN_SAMPLES}/{config.IR_FILTER_SIZE} samples"
    )
    print()
    print(f"Side Target         : {config.TARGET_LEFT_CM:.1f} cm")
    print(
        f"Wall Hysteresis     : enter<{config.SIDE_WALL_ENTER_CM:.1f} "
        f"leave>{config.SIDE_WALL_EXIT_CM:.1f} cm"
    )
    print(f"Side Danger         : {config.SIDE_TOO_CLOSE_CM:.1f} cm")
    print()
    print(f"Node Match Radius   : {config.NODE_MATCH_RADIUS_M:.2f} m")
    print(f"Rearm Distance      : {config.JUNCTION_REARM_DISTANCE_M:.2f} m")
    print(f"Edge Max Visits     : {config.MAX_EDGE_VISITS}")
    print(f"DFS Preference      : {config.EXPLORATION_PREFERENCE}")
    print(
        f"Junction Creep      : {config.ENABLE_JUNCTION_CREEP} "
        f"({config.JUNCTION_CREEP_DISTANCE_M:.2f} m)"
    )
    print(
        f"Corner Turn Setup   : {config.ENABLE_CORNER_TURN_SETUP} "
        f"(ToF->{config.CORNER_TURN_FRONT_TARGET_CM:.1f} cm, "
        f"{config.CORNER_TURN_SETUP_DISTANCE_M:.2f} m max)"
    )
    print(
        f"Post-turn Clearance : {config.ENABLE_POST_TURN_CLEARANCE} "
        f"(release>{config.POST_TURN_CLEARANCE_RELEASE_CM:.1f} cm)"
    )
    print(f"Feedback Turn       : {config.ENABLE_FEEDBACK_TURN}")
    print(f"Heading Hold        : {config.ENABLE_HEADING_HOLD}")
    print(f"Move Z -> Yaw Sign  : {config.DEFAULT_MOVE_TO_YAW_SIGN:+d}")
    print(f"Drive Z -> Yaw Sign : {config.DEFAULT_DRIVE_TO_YAW_SIGN:+d}")
    print(f"Heading Recover     : >={config.HEADING_RECOVER_TRIGGER_DEG:.1f} deg")
    print()


def wait_for_pose(pose_tracker):
    deadline = time.time() + config.POSE_WAIT_SEC
    while time.time() < deadline:
        if pose_tracker.has_pose():
            x, y, _ = pose_tracker.get_position()
            return x, y
        time.sleep(0.05)
    print("WARNING: chassis position not ready; using (0, 0).")
    return 0.0, 0.0


def wait_for_yaw(pose_tracker):
    deadline = time.time() + config.POSE_WAIT_SEC
    while time.time() < deadline:
        yaw = pose_tracker.get_yaw()
        if yaw is not None:
            return yaw
        time.sleep(0.05)
    print("WARNING: attitude yaw not ready; heading hold will wait.")
    return None


def median_or_none(values):
    return statistics.median(values) if values else None


def majority_bool_or_none(values):
    values = [value for value in values if value is not None]
    if not values:
        return None
    true_count = sum(1 for value in values if value)
    false_count = len(values) - true_count
    if true_count == false_count:
        return None
    return true_count > false_count


def ir_text(wall_state):
    if wall_state is True:
        return "W"
    if wall_state is False:
        return "-"
    return "?"


def _pose_xy(pose_tracker):
    x, y, _ = pose_tracker.get_pose()
    return x, y


def _travelled_m(start_x, start_y, pose_tracker):
    x, y = _pose_xy(pose_tracker)
    if start_x is None or start_y is None or x is None or y is None:
        return None
    return ((x - start_x) ** 2 + (y - start_y) ** 2) ** 0.5


def align_heading_in_place(chassis, controller, pose_tracker):
    if not config.ENABLE_ABSOLUTE_HEADING_ALIGN or not config.ENABLE_MOTION:
        return
    target = controller.heading_target_yaw
    if target is None:
        return

    deadline = time.monotonic() + config.HEADING_ALIGN_TIMEOUT_SEC
    while time.monotonic() < deadline:
        yaw = pose_tracker.get_yaw()
        error = controller.heading_error(yaw)
        if error is None or abs(error) <= config.HEADING_ALIGN_TOLERANCE_DEG:
            break
        z_cmd, _ = controller.calculate_heading_hold(
            yaw, pose_tracker, recover=True
        )
        chassis.drive_speed(
            x=0.0,
            y=0.0,
            z=z_cmd,
            timeout=config.DRIVE_TIMEOUT_SEC,
        )
        time.sleep(config.HEADING_ALIGN_LOOP_SEC)

    stop_chassis(chassis)
    yaw = pose_tracker.get_yaw()
    error = controller.heading_error(yaw)
    if yaw is not None and error is not None:
        print(
            f">>> ABS HEADING ALIGN target={target:+.1f} "
            f"yaw={yaw:+.1f} error={error:+.1f}"
        )


def scan_decision_point(detector, sensors):
    """Stopped multi-sample Sharp + IR fusion re-scan."""
    time.sleep(config.JUNCTION_SETTLE_SEC)

    left_samples = []
    right_samples = []
    front_samples = []
    left_ir_samples = []
    right_ir_samples = []

    def collect_one_sample():
        _, left_cm = sensors.read_left_sharp()
        _, right_cm = sensors.read_right_sharp()
        _, left_ir_wall = sensors.read_left_ir()
        _, right_ir_wall = sensors.read_right_ir()
        front_cm = sensors.get_front_cm()

        if left_cm is not None:
            left_samples.append(left_cm)
        if right_cm is not None:
            right_samples.append(right_cm)
        if front_cm is not None:
            front_samples.append(front_cm)
        if left_ir_wall is not None:
            left_ir_samples.append(left_ir_wall)
        if right_ir_wall is not None:
            right_ir_samples.append(right_ir_wall)

    for index in range(config.DECISION_SCAN_SAMPLES):
        collect_one_sample()
        if index + 1 < config.DECISION_SCAN_SAMPLES:
            time.sleep(config.DECISION_SCAN_INTERVAL_SEC)

    def classify_current():
        left_cm = median_or_none(left_samples)
        right_cm = median_or_none(right_samples)
        front_cm = median_or_none(front_samples)
        left_ir_wall = majority_bool_or_none(left_ir_samples)
        right_ir_wall = majority_bool_or_none(right_ir_samples)

        result = detector.classify_openings(
            front_cm,
            left_cm,
            right_cm,
            left_ir_wall=left_ir_wall,
            right_ir_wall=right_ir_wall,
        )
        return (
            front_cm,
            left_cm,
            right_cm,
            left_ir_wall,
            right_ir_wall,
            result,
        )

    (
        front_cm,
        left_cm,
        right_cm,
        left_ir_wall,
        right_ir_wall,
        result,
    ) = classify_current()

    fusion = detector.get_side_fusion()
    conflict = (
        "CONFLICT" in fusion["left"]["reason"]
        or "CONFLICT" in fusion["right"]["reason"]
    )

    # Strong Sharp vs IR disagreement is not allowed to instantly flip the
    # result.  Re-scan a few extra samples, then Sharp strong-zone still wins.
    if config.ENABLE_IR_SIDE_FUSION and conflict:
        print(">>> IR/SHARP CONFLICT: extra decision re-scan")
        for index in range(config.IR_CONFLICT_RESCAN_SAMPLES):
            collect_one_sample()
            if index + 1 < config.IR_CONFLICT_RESCAN_SAMPLES:
                time.sleep(config.DECISION_SCAN_INTERVAL_SEC)
        (
            front_cm,
            left_cm,
            right_cm,
            left_ir_wall,
            right_ir_wall,
            result,
        ) = classify_current()
        fusion = detector.get_side_fusion()

    front_open, front_blocked, left_open, right_open = result
    left_zone, right_zone = detector.get_side_zones()

    print(
        f"Decision Scan -> Front:{fmt(front_cm)} "
        f"({'OPEN' if front_open else 'BLOCK'}, "
        f"need>={config.EXPLORATION_FRONT_OPEN_CM:.1f}) | "
        f"L:{fmt(left_cm)} [{left_zone}] IR:{ir_text(left_ir_wall)} "
        f"=>{'OPEN' if left_open else 'BLOCK'} "
        f"({fusion['left']['reason']}/{fusion['left']['confidence']}) | "
        f"R:{fmt(right_cm)} [{right_zone}] IR:{ir_text(right_ir_wall)} "
        f"=>{'OPEN' if right_open else 'BLOCK'} "
        f"({fusion['right']['reason']}/{fusion['right']['confidence']})"
    )

    return {
        "front_cm": front_cm,
        "left_cm": left_cm,
        "right_cm": right_cm,
        "front_open": front_open,
        "front_blocked": front_blocked,
        "left_open": left_open,
        "right_open": right_open,
        "left_zone": left_zone,
        "right_zone": right_zone,
        "left_ir_wall": left_ir_wall,
        "right_ir_wall": right_ir_wall,
        "left_fusion": fusion["left"],
        "right_fusion": fusion["right"],
    }


def creep_to_junction_center(
    chassis,
    sensors,
    controller,
    pose_tracker,
    front_open,
    left_open,
    right_open,
):
    if not config.ENABLE_JUNCTION_CREEP:
        return
    if not front_open or not (left_open or right_open):
        return
    if not config.ENABLE_MOTION:
        return

    start_x, start_y = _pose_xy(pose_tracker)
    start_time = time.monotonic()
    print(
        f">>> JUNCTION_CREEP speed={config.JUNCTION_CREEP_SPEED:.2f} m/s "
        f"target={config.JUNCTION_CREEP_DISTANCE_M:.2f}m"
    )

    while time.monotonic() - start_time < config.JUNCTION_CREEP_MAX_SEC:
        front_cm = sensors.get_front_cm()
        if front_cm is None:
            print("JUNCTION_CREEP abort: ToF unavailable")
            break
        if front_cm <= config.JUNCTION_CREEP_ABORT_FRONT_CM:
            print(f"JUNCTION_CREEP abort: front={front_cm:.1f} cm")
            break
        travelled = _travelled_m(start_x, start_y, pose_tracker)
        if travelled is not None and travelled >= config.JUNCTION_CREEP_DISTANCE_M:
            print(f"JUNCTION_CREEP done: travelled={travelled:.3f} m")
            break

        x_cmd, y_cmd, z_cmd, _, _ = controller.apply_heading_hold(
            config.JUNCTION_CREEP_SPEED,
            0.0,
            pose_tracker.get_yaw(),
            pose_tracker,
            "JUNCTION_CREEP",
        )
        chassis.drive_speed(
            x=x_cmd,
            y=y_cmd,
            z=z_cmd,
            timeout=config.DRIVE_TIMEOUT_SEC,
        )
        time.sleep(config.JUNCTION_CREEP_LOOP_SEC)
    else:
        print("JUNCTION_CREEP timeout")

    stop_chassis(chassis)


def corner_turn_setup(
    chassis,
    sensors,
    controller,
    pose_tracker,
    relative_direction,
    front_open,
):
    if not config.ENABLE_CORNER_TURN_SETUP:
        return
    if relative_direction not in ("LEFT", "RIGHT"):
        return
    if front_open:
        return
    if not config.ENABLE_MOTION:
        return

    start_x, start_y = _pose_xy(pose_tracker)
    start_time = time.monotonic()
    start_front = sensors.get_front_cm()

    print(
        f">>> TURN_SETUP {relative_direction} "
        f"speed={config.CORNER_TURN_SETUP_SPEED:.2f} m/s "
        f"target_move={config.CORNER_TURN_SETUP_DISTANCE_M:.2f}m "
        f"front_target={config.CORNER_TURN_FRONT_TARGET_CM:.1f}cm "
        f"start_front={start_front if start_front is not None else 'None'}"
    )

    reason = "timeout"
    while time.monotonic() - start_time < config.CORNER_TURN_SETUP_MAX_SEC:
        front_cm = sensors.get_front_cm()
        if front_cm is None:
            reason = "ToF unavailable"
            break
        if front_cm <= config.CORNER_TURN_FRONT_HARD_STOP_CM:
            reason = f"HARD STOP front={front_cm:.1f}cm"
            break
        if front_cm <= config.CORNER_TURN_FRONT_TARGET_CM:
            reason = f"front target={front_cm:.1f}cm"
            break
        travelled = _travelled_m(start_x, start_y, pose_tracker)
        if travelled is not None and travelled >= config.CORNER_TURN_SETUP_DISTANCE_M:
            reason = f"travelled={travelled:.3f}m"
            break

        x_cmd, y_cmd, z_cmd, _, _ = controller.apply_heading_hold(
            config.CORNER_TURN_SETUP_SPEED,
            0.0,
            pose_tracker.get_yaw(),
            pose_tracker,
            "TURN_SETUP",
        )
        chassis.drive_speed(
            x=x_cmd,
            y=y_cmd,
            z=z_cmd,
            timeout=config.DRIVE_TIMEOUT_SEC,
        )
        time.sleep(config.CORNER_TURN_SETUP_LOOP_SEC)

    stop_chassis(chassis)
    print(f"TURN_SETUP done: {reason}")


def post_turn_clearance(
    chassis,
    sensors,
    controller,
    pose_tracker,
    relative_direction,
):
    if not config.ENABLE_POST_TURN_CLEARANCE:
        return
    if relative_direction not in ("LEFT", "RIGHT"):
        return
    if not config.ENABLE_MOTION:
        return

    sensors.reset_filters()
    read_inner = (
        sensors.read_left_sharp
        if relative_direction == "LEFT"
        else sensors.read_right_sharp
    )
    y_out = (
        +config.POST_TURN_CLEARANCE_Y_SPEED * config.Y_DIR_SIGN
        if relative_direction == "LEFT"
        else -config.POST_TURN_CLEARANCE_Y_SPEED * config.Y_DIR_SIGN
    )

    inner_cm = None
    for _ in range(2):
        _, inner_cm = read_inner()
        time.sleep(config.POST_TURN_CLEARANCE_LOOP_SEC)

    if inner_cm is None:
        print("POST_TURN_CLEARANCE skipped: inner Sharp unavailable")
        return
    if inner_cm > config.POST_TURN_CLEARANCE_TRIGGER_CM:
        print(
            f"POST_TURN_CLEARANCE not needed: {relative_direction} "
            f"inner={inner_cm:.1f} cm"
        )
        return

    start_x, start_y = _pose_xy(pose_tracker)
    start_time = time.monotonic()
    print(
        f">>> POST_TURN_CLEARANCE {relative_direction} "
        f"inner={inner_cm:.1f}cm "
        f"release={config.POST_TURN_CLEARANCE_RELEASE_CM:.1f}cm"
    )

    while time.monotonic() - start_time < config.POST_TURN_CLEARANCE_MAX_SEC:
        front_cm = sensors.get_front_cm()
        if front_cm is None:
            stop_chassis(chassis)
            time.sleep(config.POST_TURN_CLEARANCE_LOOP_SEC)
            continue
        if front_cm <= config.POST_TURN_CLEARANCE_FRONT_STOP_CM:
            print(f"POST_TURN_CLEARANCE stop: front={front_cm:.1f} cm")
            break

        _, inner_cm = read_inner()
        if inner_cm is None:
            print("POST_TURN_CLEARANCE abort: inner Sharp unavailable")
            break
        if inner_cm >= config.POST_TURN_CLEARANCE_RELEASE_CM:
            print(f"POST_TURN_CLEARANCE done: inner={inner_cm:.1f} cm")
            break

        travelled = _travelled_m(start_x, start_y, pose_tracker)
        if travelled is not None and travelled >= config.POST_TURN_CLEARANCE_MAX_DISTANCE_M:
            print(
                f"POST_TURN_CLEARANCE done: travelled={travelled:.3f} m, "
                f"inner={inner_cm:.1f} cm"
            )
            break

        x_cmd, y_cmd, z_cmd, _, _ = controller.apply_heading_hold(
            config.POST_TURN_CLEARANCE_FORWARD_SPEED,
            y_out,
            pose_tracker.get_yaw(),
            pose_tracker,
            "POST_TURN_CLEARANCE",
        )
        chassis.drive_speed(
            x=x_cmd,
            y=y_cmd,
            z=z_cmd,
            timeout=config.DRIVE_TIMEOUT_SEC,
        )
        time.sleep(config.POST_TURN_CLEARANCE_LOOP_SEC)

    stop_chassis(chassis)


def apply_motion_safety(x, y, z, mode):
    if mode == "BOTH_TOO_CLOSE":
        return 0.0, y, z, mode + "_STOP_X"
    if "ESCAPE_" in mode:
        return min(x, config.ESCAPE_FORWARD_SPEED), y, z, mode + "_SLOW_X"
    if mode == "NO_SENSOR":
        return 0.0, y, z, mode + "_STOP_X"
    return x, y, z, mode


def main():
    ep_robot = robot.Robot()
    chassis = None
    tof_sensor = None
    pose_subscribed = False
    attitude_subscribed = False
    tof_subscribed = False

    try:
        print("Connecting RoboMaster...")
        ep_robot.initialize(conn_type="ap")
        chassis = ep_robot.chassis
        sensor_adapter = ep_robot.sensor_adaptor
        tof_sensor = ep_robot.sensor

        sensors = SensorManager(sensor_adapter)
        controller = MotionController()
        pose_tracker = PoseTracker()
        detector = DecisionPointDetector()
        explorer = TremauxExplorer()

        tof_subscribed = tof_sensor.sub_distance(freq=20, callback=sensors.tof_callback)
        pose_subscribed = chassis.sub_position(
            cs=1,
            freq=config.POSE_FREQ_HZ,
            callback=pose_tracker.position_callback,
        )
        attitude_subscribed = chassis.sub_attitude(
            freq=config.ATTITUDE_FREQ_HZ,
            callback=pose_tracker.attitude_callback,
        )

        print_startup_info()

        start_x, start_y = wait_for_pose(pose_tracker)
        start_yaw = wait_for_yaw(pose_tracker)
        controller.initialize_heading(start_yaw, pose_tracker)

        start_node = explorer.initialize_start(start_x, start_y)
        explorer.commit_initial_forward()
        print(f"START NODE: {start_node} at ({start_x:+.2f}, {start_y:+.2f}) m")
        print("Initial action: explore FRONT")
        if controller.heading_target_yaw is not None:
            print(f"Heading grid N      : {controller.heading_target_yaw:+.1f} deg")
        print()

        if config.SAVE_MAZE_MEMORY:
            explorer.save_memory()

        while True:
            raw_adc_l, sharp_left_cm = sensors.read_left_sharp()
            raw_adc_r, sharp_right_cm = sensors.read_right_sharp()
            ir_left_level, ir_left_wall = sensors.read_left_ir()
            ir_right_level, ir_right_wall = sensors.read_right_ir()
            front_cm = sensors.get_front_cm()
            pose_x, pose_y, _ = pose_tracker.get_pose()

            x = y = z = 0.0
            mode = "STOP"

            decision_event = detector.update(
                front_cm,
                sharp_left_cm,
                sharp_right_cm,
                left_ir_wall=ir_left_wall,
                right_ir_wall=ir_right_wall,
                pose_x=pose_x,
                pose_y=pose_y,
            )

            front_blocked_now = (
                front_cm is not None and 0.0 < front_cm <= config.STOP_FRONT_CM
            )

            if decision_event:
                controller.reset_side_owner()
                stop_chassis(chassis)

                pre_front_open, _, pre_left_open, pre_right_open = detector.classify_openings(
                    front_cm,
                    sharp_left_cm,
                    sharp_right_cm,
                    left_ir_wall=ir_left_wall,
                    right_ir_wall=ir_right_wall,
                )

                creep_to_junction_center(
                    chassis,
                    sensors,
                    controller,
                    pose_tracker,
                    pre_front_open,
                    pre_left_open,
                    pre_right_open,
                )

                scan = scan_decision_point(detector, sensors)

                if (
                    scan["front_cm"] is None
                    or scan["left_cm"] is None
                    or scan["right_cm"] is None
                ):
                    print("Decision rejected: incomplete sensor data.")
                    detector.cancel_event()
                    stop_chassis(chassis)
                    time.sleep(config.LOOP_DELAY_SEC)
                    continue

                if scan["front_open"] and not scan["left_open"] and not scan["right_open"]:
                    print("Decision rejected: normal corridor after re-scan.")
                    detector.cancel_event()
                    time.sleep(config.LOOP_DELAY_SEC)
                    continue

                pose_x, pose_y, _ = pose_tracker.get_pose()
                if pose_x is None or pose_y is None:
                    current = explorer.nodes.get(explorer.current_node_id)
                    pose_x = current.x if current is not None else 0.0
                    pose_y = current.y if current is not None else 0.0

                node_id, is_new = explorer.arrive_at_decision_point(pose_x, pose_y)
                print()

                match = explorer.last_node_match or {}
                if match.get("mode") in ("EXPECTED", "EXPECTED_RELAXED"):
                    label = (
                        "EXPECTED NODE MATCH"
                        if match.get("mode") == "EXPECTED"
                        else "EXPECTED NODE MATCH RELAXED"
                    )
                    print(
                        f"[{label}] {node_id} "
                        f"distance={match['distance_m']:.3f} m "
                        f"from={match.get('from_node')}"
                    )
                elif (
                    match.get("mode") == "NEARBY"
                    and match.get("expected_node")
                    and match.get("expected_node") != node_id
                ):
                    print(
                        f"[EXPECTED NODE CONFLICT] expected={match['expected_node']} "
                        f"but nearby={node_id}; graph-link overwrite is blocked"
                    )
                elif match.get("mode") == "NEW" and match.get("expected_node"):
                    print(
                        f"[EXPECTED NODE MISSED] expected={match['expected_node']} "
                        f"distance={match.get('expected_distance_m')} -> created {node_id}; "
                        f"graph-link overwrite is blocked"
                    )

                print(
                    f"[{'NEW' if is_new else 'KNOWN'} NODE] {node_id} "
                    f"at ({pose_x:+.2f}, {pose_y:+.2f}) m"
                )
                print("Memory:", explorer.describe_node(node_id))
                print("DFS Stack:", " -> ".join(explorer.dfs_stack))

                exploration_decision = explorer.plan_direction(
                    front_open=scan["front_open"],
                    left_open=scan["left_open"],
                    right_open=scan["right_open"],
                )
                print_exploration_decision(exploration_decision)

                if exploration_decision.direction == "COMPLETE":
                    explorer.commit_decision(exploration_decision)
                    if config.SAVE_MAZE_MEMORY:
                        explorer.save_memory()
                    print("EXPLORATION COMPLETE")
                    if config.STOP_WHEN_EXPLORATION_COMPLETE:
                        break
                    detector.cancel_event()
                    continue

                turn_decision = decision_from_relative(exploration_decision.direction)

                corner_turn_setup(
                    chassis,
                    sensors,
                    controller,
                    pose_tracker,
                    exploration_decision.direction,
                    scan["front_open"],
                )

                turn_ok = execute_turn(
                    chassis,
                    turn_decision,
                    pose_tracker=pose_tracker,
                )
                if not turn_ok:
                    stop_chassis(chassis)
                    print("TURN FAILED SAFELY - MAP EDGE NOT COMMITTED")
                    if config.SAVE_MAZE_MEMORY:
                        explorer.save_memory()
                    break

                explorer.commit_decision(exploration_decision)
                controller.set_heading_index(explorer.heading_index, pose_tracker)
                align_heading_in_place(chassis, controller, pose_tracker)

                controller.reset_after_turn()
                post_turn_clearance(
                    chassis,
                    sensors,
                    controller,
                    pose_tracker,
                    exploration_decision.direction,
                )

                print("Updated Memory:", explorer.describe_node(node_id))
                print(f"New heading: {explorer.heading_name()}")

                if config.SAVE_MAZE_MEMORY:
                    explorer.save_memory()

                sensors.reset_filters()
                latch_x, latch_y = _pose_xy(pose_tracker)
                detector.force_latched(
                    latch_x if latch_x is not None else pose_x,
                    latch_y if latch_y is not None else pose_y,
                    reset_side_memory=True,
                )
                stop_chassis(chassis)
                time.sleep(config.AFTER_TURN_DELAY_SEC)
                continue

            if front_blocked_now:
                controller.reset_side_owner()
                x = y = z = 0.0
                mode = "FRONT_CONFIRM"
            else:
                x = controller.calculate_forward_speed(front_cm)
                y, z, mode = controller.calculate_motion_control(
                    raw_adc_l,
                    sharp_left_cm,
                    raw_adc_r,
                    sharp_right_cm,
                    ir_left_wall,
                )
                if (
                    front_cm is not None
                    and config.STOP_FRONT_CM < front_cm < config.SLOW_FRONT_CM
                ):
                    mode = "SLOW_" + mode

                x, y, z, mode = apply_motion_safety(x, y, z, mode)
                x, y, z, mode, _ = controller.apply_heading_hold(
                    x,
                    y,
                    pose_tracker.get_yaw(),
                    pose_tracker,
                    mode,
                )

            if config.ENABLE_MOTION:
                chassis.drive_speed(
                    x=x,
                    y=y,
                    z=z,
                    timeout=config.DRIVE_TIMEOUT_SEC,
                )

            delta = (
                sharp_left_cm - sharp_right_cm
                if sharp_left_cm is not None and sharp_right_cm is not None
                else 0.0
            )
            pose_x, pose_y, yaw_deg = pose_tracker.get_pose()
            pose_text = (
                f"({pose_x:+.2f},{pose_y:+.2f})"
                if pose_x is not None and pose_y is not None
                else "(---,---)"
            )
            yaw_text = f"{yaw_deg:+6.1f}" if yaw_deg is not None else "  --- "
            target_yaw = controller.heading_target_yaw
            target_text = f"{target_yaw:+6.1f}" if target_yaw is not None else "  --- "
            heading_error = controller.heading_error(yaw_deg)
            heading_error_text = (
                f"{heading_error:+5.1f}" if heading_error is not None else "  ---"
            )
            left_zone = detector.classify_side_zone(sharp_left_cm)
            right_zone = detector.classify_side_zone(sharp_right_cm)
            fusion = detector.get_side_fusion()
            ir_l_text = ir_text(ir_left_wall)
            ir_r_text = ir_text(ir_right_wall)

            print(
                f"ToF:{fmt(front_cm)}cm | "
                f"L:{fmt(sharp_left_cm)}[{left_zone[:3]}] ADC:{raw_adc_l:4d} | "
                f"R:{fmt(sharp_right_cm)}[{right_zone[:3]}] ADC:{raw_adc_r:4d} | "
                f"IR L:{ir_left_level if ir_left_level is not None else '?'}:{ir_l_text} "
                f"R:{ir_right_level if ir_right_level is not None else '?'}:{ir_r_text} | "
                f"D:{delta:+5.1f} | "
                f"POSE:{pose_text} | "
                f"YAW:{yaw_text}/{target_text} E:{heading_error_text} | "
                f"H:{explorer.heading_name()} | "
                f"OWNER:{controller.side_owner:5s} | "
                f"LATCH:{int(detector.latched)} | "
                f"{mode:24s} | x={x:.3f} y={y:+.3f} z={z:+.1f}"
            )

            time.sleep(config.LOOP_DELAY_SEC)

    except KeyboardInterrupt:
        print("\nSTOP REQUESTED BY USER")
    except Exception as exc:
        print("\nERROR:", exc)
        raise
    finally:
        try:
            stop_chassis(chassis)
        except Exception:
            pass
        try:
            if tof_sensor is not None and tof_subscribed:
                tof_sensor.unsub_distance()
        except Exception:
            pass
        try:
            if chassis is not None and pose_subscribed:
                chassis.unsub_position()
        except Exception:
            pass
        try:
            if chassis is not None and attitude_subscribed:
                chassis.unsub_attitude()
        except Exception:
            pass
        ep_robot.close()
        print("Robot stopped and disconnected.")


if __name__ == "__main__":
    main()

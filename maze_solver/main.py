"""Main entry point for the RoboMaster Trémaux / DFS maze explorer."""

import statistics
import time

from robomaster import robot

import config
from controller import MotionController
from exploration import DecisionPointDetector, TremauxExplorer
from navigation import (
    decision_from_relative,
    execute_turn,
    print_exploration_decision,
)
from pose_tracker import PoseTracker
from sensors import SensorManager


def fmt(value):
    if value is None:
        return "---"
    return f"{value:4.1f}"


def stop_chassis(chassis):
    if not config.ENABLE_MOTION or chassis is None:
        return
    chassis.drive_speed(x=0, y=0, z=0, timeout=0.1)


def print_startup_info():
    print()
    print("==========================================================")
    print(" MAZE SOLVER - ROBUST TRÉMAUX / DFS + WALL FOLLOWING")
    print("==========================================================")
    print()
    print(f"Forward speed       : {config.FORWARD_SPEED:.2f} m/s")
    print(f"Front Slow          : {config.SLOW_FRONT_CM:.1f} cm")
    print(f"Front Stop          : {config.STOP_FRONT_CM:.1f} cm")
    print(f"Side Opening        : >= {config.EXPLORATION_SIDE_OPEN_CM:.1f} cm")
    print(f"Front Traversable   : >= {config.EXPLORATION_FRONT_OPEN_CM:.1f} cm")
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
    print(f"Junction Creep      : {config.ENABLE_JUNCTION_CREEP} ({config.JUNCTION_CREEP_DISTANCE_M:.2f} m)")
    print(f"Corner Turn Setup   : {config.ENABLE_CORNER_TURN_SETUP} (ToF->{config.CORNER_TURN_FRONT_TARGET_CM:.1f} cm, {config.CORNER_TURN_SETUP_DISTANCE_M:.2f} m max)")
    print(f"Post-turn Clearance : {config.ENABLE_POST_TURN_CLEARANCE} (release>{config.POST_TURN_CLEARANCE_RELEASE_CM:.1f} cm)")
    print(f"Yaw Correction      : {config.ENABLE_YAW_CORRECTION}")
    print(f"Feedback Turn       : {config.ENABLE_FEEDBACK_TURN} (no unbounded SDK wait)")
    print(f"Heading Hold        : {config.ENABLE_HEADING_HOLD}")
    print(f"Move Z -> Yaw Sign  : {config.DEFAULT_MOVE_TO_YAW_SIGN:+d}")
    print(f"Drive Z -> Yaw Sign : {config.DEFAULT_DRIVE_TO_YAW_SIGN:+d}")
    print(f"Heading Recover     : >={config.HEADING_RECOVER_TRIGGER_DEG:.1f} deg")
    print()
    print("Sharp controls Y; attitude yaw holds Z while driving corridors.")
    print("Trémaux chooses FRONT / LEFT / RIGHT / BACK at junctions.")
    print("Unvisited exits are always preferred over visited exits.")
    print()


def wait_for_pose(pose_tracker):
    deadline = time.time() + config.POSE_WAIT_SEC

    while time.time() < deadline:
        if pose_tracker.has_pose():
            x, y, _ = pose_tracker.get_position()
            return x, y
        time.sleep(0.05)

    print("WARNING: chassis position not ready; using start pose (0, 0).")
    return 0.0, 0.0


def wait_for_yaw(pose_tracker):
    deadline = time.time() + config.POSE_WAIT_SEC

    while time.time() < deadline:
        yaw = pose_tracker.get_yaw()
        if yaw is not None:
            return yaw
        time.sleep(0.05)

    print("WARNING: attitude yaw not ready; heading hold will wait for data.")
    return None


def align_heading_in_place(chassis, controller, pose_tracker):
    """Rotate gently in place to the absolute cardinal target."""
    if not config.ENABLE_ABSOLUTE_HEADING_ALIGN or not config.ENABLE_MOTION:
        return

    target = controller.heading_target_yaw
    if target is None:
        return

    deadline = time.monotonic() + config.HEADING_ALIGN_TIMEOUT_SEC

    while time.monotonic() < deadline:
        yaw = pose_tracker.get_yaw()
        error = controller.heading_error(yaw)
        if error is None:
            break

        if abs(error) <= config.HEADING_ALIGN_TOLERANCE_DEG:
            break

        z_cmd, _ = controller.calculate_heading_hold(
            yaw,
            pose_tracker,
            recover=True,
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


def median_or_none(values):
    return statistics.median(values) if values else None


def scan_decision_point(detector, sensors):
    """Stop-time re-scan using several filtered Sharp samples."""
    time.sleep(config.JUNCTION_SETTLE_SEC)

    left_samples = []
    right_samples = []
    front_samples = []

    for index in range(config.DECISION_SCAN_SAMPLES):
        _, left_cm = sensors.read_left_sharp()
        _, right_cm = sensors.read_right_sharp()
        front_cm = sensors.get_front_cm()

        if left_cm is not None:
            left_samples.append(left_cm)
        if right_cm is not None:
            right_samples.append(right_cm)
        if front_cm is not None:
            front_samples.append(front_cm)

        if index + 1 < config.DECISION_SCAN_SAMPLES:
            time.sleep(config.DECISION_SCAN_INTERVAL_SEC)

    left_cm = median_or_none(left_samples)
    right_cm = median_or_none(right_samples)
    front_cm = median_or_none(front_samples)

    front_open, front_blocked, left_open, right_open = (
        detector.classify_openings(
            front_cm,
            left_cm,
            right_cm,
        )
    )

    print(
        f"Decision Scan -> Front:{fmt(front_cm)} "
        f"({'OPEN' if front_open else 'BLOCK'}, "
        f"need>={config.EXPLORATION_FRONT_OPEN_CM:.1f}) | "
        f"L:{fmt(left_cm)} ({'OPEN' if left_open else 'BLOCK'}, "
        f"need>={config.EXPLORATION_SIDE_OPEN_CM:.1f}) | "
        f"R:{fmt(right_cm)} ({'OPEN' if right_open else 'BLOCK'}, "
        f"need>={config.EXPLORATION_SIDE_OPEN_CM:.1f})"
    )

    return {
        "front_cm": front_cm,
        "left_cm": left_cm,
        "right_cm": right_cm,
        "front_open": front_open,
        "front_blocked": front_blocked,
        "left_open": left_open,
        "right_open": right_open,
    }


def _pose_xy(pose_tracker):
    x, y, _ = pose_tracker.get_pose()
    return x, y


def _travelled_m(start_x, start_y, pose_tracker):
    x, y = _pose_xy(pose_tracker)
    if start_x is None or start_y is None or x is None or y is None:
        return None
    return ((x - start_x) ** 2 + (y - start_y) ** 2) ** 0.5


def creep_to_junction_center(
    chassis,
    sensors,
    controller,
    pose_tracker,
    front_open,
    left_open,
    right_open,
):
    """Move into the centre of a front-open side junction.

    V6 uses travelled distance rather than a fixed 0.50 s. This makes the
    physical offset much more repeatable when battery/load/traction changes.
    Corners with a non-open front are handled later by corner_turn_setup().
    """
    if not config.ENABLE_JUNCTION_CREEP:
        return

    if not front_open or not (left_open or right_open):
        return

    if not config.ENABLE_MOTION:
        print("JUNCTION_CREEP skipped: motion disabled")
        return

    start_x, start_y = _pose_xy(pose_tracker)
    start_time = time.monotonic()

    print(
        f">>> JUNCTION_CREEP speed={config.JUNCTION_CREEP_SPEED:.2f} m/s "
        f"target={config.JUNCTION_CREEP_DISTANCE_M:.2f}m "
        f"max={config.JUNCTION_CREEP_MAX_SEC:.2f}s"
    )

    while time.monotonic() - start_time < config.JUNCTION_CREEP_MAX_SEC:
        front_cm = sensors.get_front_cm()

        if front_cm is None:
            print("JUNCTION_CREEP abort: ToF unavailable")
            break

        if front_cm <= config.JUNCTION_CREEP_ABORT_FRONT_CM:
            print(
                f"JUNCTION_CREEP abort: front={front_cm:.1f} cm "
                f"<= {config.JUNCTION_CREEP_ABORT_FRONT_CM:.1f} cm"
            )
            break

        travelled = _travelled_m(start_x, start_y, pose_tracker)
        if (
            travelled is not None
            and travelled >= config.JUNCTION_CREEP_DISTANCE_M
        ):
            print(f"JUNCTION_CREEP done: travelled={travelled:.3f} m")
            break

        creep_x, creep_y, creep_z, _, _ = controller.apply_heading_hold(
            config.JUNCTION_CREEP_SPEED,
            0.0,
            pose_tracker.get_yaw(),
            pose_tracker,
            "JUNCTION_CREEP",
        )

        chassis.drive_speed(
            x=creep_x,
            y=creep_y,
            z=creep_z,
            timeout=config.DRIVE_TIMEOUT_SEC,
        )
        time.sleep(config.JUNCTION_CREEP_LOOP_SEC)

    stop_chassis(chassis)


def corner_turn_setup(
    chassis,
    sensors,
    controller,
    pose_tracker,
    relative_direction,
    front_open,
):
    """Advance a little farther before a LEFT/RIGHT corner turn.

    This fixes the V5 failure mode where a side opening is found while the
    front is not traversable. V5 skipped junction creep in that situation and
    rotated immediately, so the chassis could pivot before reaching the corner
    centre and clip the inside wall.

    Motion stops on whichever occurs first:
      * odometry reaches CORNER_TURN_SETUP_DISTANCE_M,
      * front ToF reaches CORNER_TURN_FRONT_TARGET_CM,
      * hard-stop distance is reached,
      * timeout / missing ToF.
    """
    if not config.ENABLE_CORNER_TURN_SETUP:
        return

    if relative_direction not in ("LEFT", "RIGHT"):
        return

    # A front-open junction has already been centred by JUNCTION_CREEP.
    if front_open:
        return

    if not config.ENABLE_MOTION:
        print("TURN_SETUP skipped: motion disabled")
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

    while time.monotonic() - start_time < config.CORNER_TURN_SETUP_MAX_SEC:
        front_cm = sensors.get_front_cm()

        if front_cm is None:
            print("TURN_SETUP abort: ToF unavailable")
            break

        if front_cm <= config.CORNER_TURN_FRONT_HARD_STOP_CM:
            print(
                f"TURN_SETUP HARD STOP: front={front_cm:.1f} cm "
                f"<= {config.CORNER_TURN_FRONT_HARD_STOP_CM:.1f} cm"
            )
            break

        if front_cm <= config.CORNER_TURN_FRONT_TARGET_CM:
            print(f"TURN_SETUP done: front target reached ({front_cm:.1f} cm)")
            break

        travelled = _travelled_m(start_x, start_y, pose_tracker)
        if (
            travelled is not None
            and travelled >= config.CORNER_TURN_SETUP_DISTANCE_M
        ):
            print(f"TURN_SETUP done: travelled={travelled:.3f} m")
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


def post_turn_clearance(
    chassis,
    sensors,
    controller,
    pose_tracker,
    relative_direction,
):
    """Crawl clear of the inside corner after a LEFT/RIGHT turn.

    If the inner-side Sharp sensor still sees the old corner wall very close,
    move forward slowly while adding a small outward strafe.  This prevents
    resuming 0.15 m/s while the rear/side of the chassis is still beside the
    corner edge.
    """
    if not config.ENABLE_POST_TURN_CLEARANCE:
        return
    if relative_direction not in ("LEFT", "RIGHT"):
        return
    if not config.ENABLE_MOTION:
        return

    # Flush pre-turn Sharp history so the first post-turn values are not mixed
    # with the geometry before rotation.
    sensors.reset_filters()

    read_inner = (
        sensors.read_left_sharp
        if relative_direction == "LEFT"
        else sensors.read_right_sharp
    )

    # Same outward directions used by ESCAPE_LEFT / ESCAPE_RIGHT.
    y_out = (
        +config.POST_TURN_CLEARANCE_Y_SPEED * config.Y_DIR_SIGN
        if relative_direction == "LEFT"
        else -config.POST_TURN_CLEARANCE_Y_SPEED * config.Y_DIR_SIGN
    )

    # Let the Sharp filter get a couple of fresh samples after the turn.
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
            # reset_filters() also clears ToF; wait for a fresh callback before
            # allowing any post-turn translation.
            stop_chassis(chassis)
            time.sleep(config.POST_TURN_CLEARANCE_LOOP_SEC)
            continue

        if front_cm <= config.POST_TURN_CLEARANCE_FRONT_STOP_CM:
            print(
                f"POST_TURN_CLEARANCE stop: front={front_cm:.1f} cm"
            )
            break

        _, inner_cm = read_inner()
        if inner_cm is None:
            print("POST_TURN_CLEARANCE abort: inner Sharp unavailable")
            break

        if inner_cm >= config.POST_TURN_CLEARANCE_RELEASE_CM:
            print(
                f"POST_TURN_CLEARANCE done: inner={inner_cm:.1f} cm"
            )
            break

        travelled = _travelled_m(start_x, start_y, pose_tracker)
        if (
            travelled is not None
            and travelled >= config.POST_TURN_CLEARANCE_MAX_DISTANCE_M
        ):
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
    """Final safety layer before sending chassis.drive_speed()."""
    if mode == "BOTH_TOO_CLOSE":
        return 0.0, y, z, mode + "_STOP_X"

    if "ESCAPE_" in mode:
        x = min(x, config.ESCAPE_FORWARD_SPEED)
        return x, y, z, mode + "_SLOW_X"

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
        # ====================================================
        # CONNECT
        # ====================================================

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

        # ====================================================
        # SUBSCRIPTIONS
        # ====================================================

        tof_subscribed = tof_sensor.sub_distance(
            freq=20,
            callback=sensors.tof_callback,
        )

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
        controller.initialize_heading(start_yaw, pose_tracker=pose_tracker)

        start_node = explorer.initialize_start(start_x, start_y)
        explorer.commit_initial_forward()

        print(
            f"START NODE: {start_node} "
            f"at ({start_x:+.2f}, {start_y:+.2f}) m"
        )
        print("Initial action: explore FRONT")
        if controller.heading_target_yaw is not None:
            print(f"Heading grid N      : {controller.heading_target_yaw:+.1f} deg")
        print()

        if config.SAVE_MAZE_MEMORY:
            explorer.save_memory()

        # ====================================================
        # MAIN LOOP
        # ====================================================

        while True:
            raw_adc_l, sharp_left_cm = sensors.read_left_sharp()
            raw_adc_r, sharp_right_cm = sensors.read_right_sharp()
            ir_left_wall = sensors.read_ir_digital_io()
            front_cm = sensors.get_front_cm()

            pose_x, pose_y, _ = pose_tracker.get_pose()

            x = 0.0
            y = 0.0
            z = 0.0
            mode = "STOP"
            heading_error = controller.heading_error(pose_tracker.get_yaw())

            decision_event = detector.update(
                front_cm,
                sharp_left_cm,
                sharp_right_cm,
                pose_x=pose_x,
                pose_y=pose_y,
            )

            front_blocked_now = (
                front_cm is not None
                and 0.0 < front_cm <= config.STOP_FRONT_CM
            )

            # =================================================
            # DECISION POINT
            # =================================================

            if decision_event:
                controller.reset_side_owner()
                stop_chassis(chassis)
                mode = "DFS_DECISION"

                # Classify the triggering sample first so a side opening can
                # creep a few cm toward the physical centre before final scan.
                (
                    pre_front_open,
                    _,
                    pre_left_open,
                    pre_right_open,
                ) = detector.classify_openings(
                    front_cm,
                    sharp_left_cm,
                    sharp_right_cm,
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

                # Do not create graph nodes from an invalid sensor snapshot.
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

                # Side-opening noise can trigger before the stopped re-scan.
                # If it is now a plain corridor, reject it instead of U-turning.
                if (
                    scan["front_open"]
                    and not scan["left_open"]
                    and not scan["right_open"]
                ):
                    print("Decision rejected: normal corridor after re-scan.")
                    detector.cancel_event()
                    time.sleep(config.LOOP_DELAY_SEC)
                    continue

                pose_x, pose_y, _ = pose_tracker.get_pose()

                if pose_x is None or pose_y is None:
                    current = explorer.nodes.get(explorer.current_node_id)
                    if current is not None:
                        pose_x, pose_y = current.x, current.y
                    else:
                        pose_x, pose_y = 0.0, 0.0

                node_id, is_new = explorer.arrive_at_decision_point(
                    pose_x,
                    pose_y,
                )

                print()
                print(
                    f"[{'NEW' if is_new else 'KNOWN'} NODE] "
                    f"{node_id} at ({pose_x:+.2f}, {pose_y:+.2f}) m"
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

                    print()
                    print("============================================")
                    print(" EXPLORATION COMPLETE - NO NEW PATH TO TAKE")
                    print("============================================")

                    if config.STOP_WHEN_EXPLORATION_COMPLETE:
                        break

                    detector.cancel_event()
                    continue

                turn_decision = decision_from_relative(
                    exploration_decision.direction
                )

                # V6: for a real corner (front not traversable + side turn),
                # advance the chassis a few centimetres before rotating so the
                # pivot is closer to the physical centre of the corner.
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
                    print()
                    print("============================================")
                    print(" TURN FAILED SAFELY - MAP EDGE NOT COMMITTED")
                    print(" Check yaw / chassis communication, then retry.")
                    print("============================================")
                    if config.SAVE_MAZE_MEMORY:
                        explorer.save_memory()
                    break

                # Mark the edge only after physical turn succeeds.
                explorer.commit_decision(exploration_decision)

                # Internal N/E/S/W now maps to an absolute attitude-yaw grid.
                # This removes accumulated corridor drift instead of accepting
                # the current (already-skewed) yaw as the next turn reference.
                controller.set_heading_index(
                    explorer.heading_index,
                    pose_tracker=pose_tracker,
                )
                align_heading_in_place(chassis, controller, pose_tracker)

                # V8: the 90-degree rotation can finish while the inside side
                # of the chassis is still beside the old corner wall.  Crawl
                # clear before resuming normal corridor speed.
                controller.reset_after_turn()
                post_turn_clearance(
                    chassis,
                    sensors,
                    controller,
                    pose_tracker,
                    exploration_decision.direction,
                )

                print(
                    "Updated Memory:",
                    explorer.describe_node(node_id),
                )
                print(f"New heading: {explorer.heading_name()}")

                if config.SAVE_MAZE_MEMORY:
                    explorer.save_memory()

                # Clear stale ranges. Junction detector remains locked to this
                # node but can now re-arm by distance/corridor/timeout/emergency.
                sensors.reset_filters()
                latch_x, latch_y = _pose_xy(pose_tracker)
                detector.force_latched(
                    latch_x if latch_x is not None else pose_x,
                    latch_y if latch_y is not None else pose_y,
                )

                stop_chassis(chassis)
                time.sleep(config.AFTER_TURN_DELAY_SEC)
                continue

            # =================================================
            # FRONT TOO CLOSE
            # Stop immediately while detector collects enough
            # confirmation samples. Emergency re-arm prevents deadlock.
            # =================================================

            if front_blocked_now:
                controller.reset_side_owner()
                x = 0.0
                y = 0.0
                z = 0.0
                mode = "FRONT_CONFIRM"

            # =================================================
            # NORMAL CORRIDOR MOVEMENT
            # =================================================

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
                    and config.STOP_FRONT_CM
                    < front_cm
                    < config.SLOW_FRONT_CM
                ):
                    mode = "SLOW_" + mode

                x, y, z, mode = apply_motion_safety(
                    x,
                    y,
                    z,
                    mode,
                )

                # Sharp fixes lateral position (Y); attitude yaw independently
                # keeps the chassis square to the maze (Z).
                x, y, z, mode, heading_error = controller.apply_heading_hold(
                    x,
                    y,
                    pose_tracker.get_yaw(),
                    pose_tracker,
                    mode,
                )

            # =================================================
            # SEND COMMAND
            # =================================================

            if config.ENABLE_MOTION:
                chassis.drive_speed(
                    x=x,
                    y=y,
                    z=z,
                    timeout=config.DRIVE_TIMEOUT_SEC,
                )

            # =================================================
            # DEBUG
            # =================================================

            if sharp_left_cm is not None and sharp_right_cm is not None:
                delta = sharp_left_cm - sharp_right_cm
            else:
                delta = 0.0

            pose_x, pose_y, yaw_deg = pose_tracker.get_pose()
            pose_text = (
                f"({pose_x:+.2f},{pose_y:+.2f})"
                if pose_x is not None and pose_y is not None
                else "(---,---)"
            )
            yaw_text = f"{yaw_deg:+6.1f}" if yaw_deg is not None else "  --- "
            target_yaw = controller.heading_target_yaw
            target_text = f"{target_yaw:+6.1f}" if target_yaw is not None else "  --- "
            current_heading_error = controller.heading_error(yaw_deg)
            heading_error_text = (
                f"{current_heading_error:+5.1f}"
                if current_heading_error is not None
                else "  ---"
            )
            ir_text = str(ir_left_wall) if ir_left_wall is not None else "-"

            print(
                f"ToF:{fmt(front_cm)}cm | "
                f"L:{fmt(sharp_left_cm)} ADC:{raw_adc_l:4d} | "
                f"R:{fmt(sharp_right_cm)} ADC:{raw_adc_r:4d} | "
                f"IR:{ir_text} | "
                f"D:{delta:+5.1f} | "
                f"POSE:{pose_text} | "
                f"YAW:{yaw_text}/{target_text} "
                f"E:{heading_error_text} | "
                f"H:{explorer.heading_name()} | "
                f"OWNER:{controller.side_owner:5s} | "
                f"LATCH:{int(detector.latched)} | "
                f"{mode:24s} | "
                f"x={x:.3f} "
                f"y={y:+.3f} "
                f"z={z:+.1f}"
            )

            time.sleep(config.LOOP_DELAY_SEC)

    except KeyboardInterrupt:
        print()
        print("STOP REQUESTED BY USER")

    except Exception as exc:
        print()
        print("ERROR:", exc)
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

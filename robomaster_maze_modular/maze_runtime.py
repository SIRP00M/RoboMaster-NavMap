"""Runtime helpers for scanning junctions, safety, gate guard, and exit logic."""

import math
import config

# ==================== MAIN WALK TEST ====================
"""Main entry point for the RoboMaster Trémaux / DFS maze explorer."""

import statistics
import time




def fmt(value):
    if value is None:
        return "---"
    return f"{value:4.1f}"


def fmt_adc(value):
    if value is None:
        return " ---"
    try:
        return f"{int(value):4d}"
    except (TypeError, ValueError):
        return " ---"


def stop_chassis(chassis):
    if not config.ENABLE_MOTION or chassis is None:
        return
    chassis.drive_speed(x=0, y=0, z=0, timeout=0.1)


def print_startup_info():
    print()
    print("==========================================================")
    print(" MAZE SOLVER V10.1 - SENSOR TIMEOUT GUARD + V10 CONTROL")
    print("==========================================================")
    print()
    print(f"Program version     : {config.PROGRAM_VERSION}")
    print(f"Forward speed       : {config.FORWARD_SPEED:.2f} m/s")
    print(f"Sharp stale hold    : {config.SHARP_STALE_HOLD_SEC:.2f} s")
    print(f"Front Slow          : {config.SLOW_FRONT_CM:.1f} cm")
    print(f"Front Stop          : {config.STOP_FRONT_CM:.1f} cm")
    print(f"Side Opening        : ENTER >= {config.SIDE_OPEN_ENTER_CM:.1f} cm / EXIT < {config.SIDE_OPEN_EXIT_CM:.1f} cm")
    print(f"Opening Zone        : min {config.OPENING_ZONE_MIN_LENGTH_M:.2f} m, centre-backtrack enabled={config.ENABLE_OPENING_ZONE_CENTERING}")
    print(f"Intersection Window : lookahead {config.INTERSECTION_WINDOW_LOOKAHEAD_M:.2f} m / max {config.INTERSECTION_WINDOW_MAX_M:.2f} m / evidence {config.INTERSECTION_MIN_OPEN_SAMPLES} samples")
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
    print(f"Edge Split          : {config.ENABLE_INTERMEDIATE_NODE_EDGE_SPLIT}")
    print(f"Route Loop Break    : {config.ENABLE_ROUTE_LOOP_BREAK} (repeat={config.ROUTE_REPEAT_LIMIT})")
    print(f"Unresolved Recovery : {config.ENABLE_UNRESOLVED_EDGE_RECOVERY} (max visits={config.UNRESOLVED_EDGE_MAX_VISITS})")
    print(f"Junction Creep      : {config.ENABLE_JUNCTION_CREEP} ({config.JUNCTION_CREEP_DISTANCE_M:.2f} m)")
    print(f"Corner Turn Setup   : {config.ENABLE_CORNER_TURN_SETUP} (ToF->{config.CORNER_TURN_FRONT_TARGET_CM:.1f} cm, {config.CORNER_TURN_SETUP_DISTANCE_M:.2f} m max)")
    print(f"Post-turn Clearance : {config.ENABLE_POST_TURN_CLEARANCE} (release>{config.POST_TURN_CLEARANCE_RELEASE_CM:.1f} cm)")
    print(f"Yaw Correction      : {config.ENABLE_YAW_CORRECTION}")
    print(f"Feedback Turn       : {config.ENABLE_FEEDBACK_TURN} (tol=±{config.TURN_FEEDBACK_TOLERANCE_DEG:.1f}°, stable={config.TURN_FEEDBACK_STABLE_SAMPLES}, attempts={config.TURN_MAX_ATTEMPTS})")
    print(f"Turn Watchdog       : 90={config.TURN_FEEDBACK_TIMEOUT_90_SEC:.1f}s / 180={config.TURN_FEEDBACK_TIMEOUT_180_SEC:.1f}s / timeout-accept=±{config.TURN_TIMEOUT_ACCEPT_TOLERANCE_DEG:.1f}°")
    print(f"Heading Hold        : {config.ENABLE_HEADING_HOLD}")
    print(f"Move Z -> Yaw Sign  : {config.DEFAULT_MOVE_TO_YAW_SIGN:+d}")
    print(f"Drive Z -> Yaw Sign : {config.DEFAULT_DRIVE_TO_YAW_SIGN:+d}")
    print(f"Heading Recover     : >={config.HEADING_RECOVER_TRIGGER_DEG:.1f} deg")
    print()
    print("--- Open area / exit ---")
    print(f"Open Area           : {config.ENABLE_OPEN_AREA_HEADING_HOLD} (side>={config.OPEN_AREA_SIDE_ENTER_CM:.0f} cm, front>={config.OPEN_AREA_FRONT_MIN_CM:.0f} cm)")
    print(f"Exit Detection      : {config.ENABLE_EXIT_DETECTION} (front>={config.EXIT_FRONT_START_CM:.0f} cm, sides>={config.EXIT_SIDE_START_CM:.0f} cm)")
    print(f"Start Gate Guard    : {config.ENABLE_START_GATE_GUARD} (block outside J0 + geometric guard)")
    print(f"Exit Confirmation   : {config.EXIT_CONFIRM_DISTANCE_M:.2f} m + {config.EXIT_CONFIRM_MIN_SEC:.1f} s")
    print(f"Stop on Exit        : {config.STOP_WHEN_EXIT_FOUND}")
    print()
    print("Sharp controls Y; attitude yaw holds Z while driving corridors.")
    print("Trémaux chooses FRONT / LEFT / RIGHT / BACK at junctions.")
    print("Unvisited exits are always preferred over visited exits.")
    if config.SIDE_OPEN_ENTER_CM < 15.0:
        print("*** WARNING: SIDE OPEN threshold is suspiciously low (<15 cm). ***")
    print()
    if getattr(config, "ENABLE_MAPPING", False):
        print("--- SLAM-style mapping ---")
        print(f"Map resolution       : {config.MAP_RESOLUTION_M*100:.1f} cm/cell")
        print(f"ToF map range        : free<={config.MAP_TOF_FREE_MAX_CM:.0f} cm, wall<={config.MAP_TOF_OCCUPIED_MAX_CM:.0f} cm")
        print(f"Sharp wall range     : wall<={config.MAP_SHARP_OCCUPIED_MAX_CM:.0f} cm")
        print(f"IR wall level        : {config.MAP_IR_WALL_LEVEL} (flip to 1 if your sensor is active-high)")
        print(f"Map output           : {config.MAP_OUTPUT_DIR}/maze_map.png + .svg")
        print("Mapper is passive: it never changes robot movement or DFS decisions.")
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


def scan_decision_point(detector, sensors, intersection_event=None):
    """Stopped re-scan merged with V6 intersection-window observations.

    A direction is considered physically open if either the stable stopped scan
    sees it OR the moving intersection window saw it open for enough samples.
    This is what preserves FRONT in the real-field case where a side opening is
    encountered first and the final stopped ToF snapshot points at a nearby
    wall edge.
    """
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

    raw_front_open, front_blocked, raw_left_open, raw_right_open = (
        detector.classify_openings(front_cm, left_cm, right_cm)
    )

    front_open = raw_front_open
    left_open = raw_left_open
    right_open = raw_right_open

    print(
        f"Decision Scan RAW -> Front:{fmt(front_cm)} "
        f"({'OPEN' if raw_front_open else 'BLOCK'}, "
        f"need>={config.EXPLORATION_FRONT_OPEN_CM:.1f}) | "
        f"L:{fmt(left_cm)} ({'OPEN' if raw_left_open else 'BLOCK'}, "
        f"enter>={config.SIDE_OPEN_ENTER_CM:.1f}) | "
        f"R:{fmt(right_cm)} ({'OPEN' if raw_right_open else 'BLOCK'}, "
        f"enter>={config.SIDE_OPEN_ENTER_CM:.1f})"
    )

    if intersection_event is not None and intersection_event.get("type") == "INTERSECTION_WINDOW":
        observed = intersection_event.get("observed_open", {})
        counts = intersection_event.get("open_samples", {})

        # Moving-window FRONT evidence may recover a straight corridor that a
        # stopped snapshot under-ranges because ToF hits a wall edge.  A true
        # hard-stop reading still wins for safety.
        front_open = (
            front_open or bool(observed.get("FRONT", False))
        ) and not front_blocked
        left_open = left_open or bool(observed.get("LEFT", False))
        right_open = right_open or bool(observed.get("RIGHT", False))

        print(
            ">>> INTERSECTION MEMORY -> "
            f"F={'OPEN' if observed.get('FRONT') else '---'}({counts.get('FRONT', 0)}) "
            f"L={'OPEN' if observed.get('LEFT') else '---'}({counts.get('LEFT', 0)}) "
            f"R={'OPEN' if observed.get('RIGHT') else '---'}({counts.get('RIGHT', 0)})"
        )
        print(
            ">>> DECISION MERGED     -> "
            f"F={'OPEN' if front_open else 'BLOCK'} "
            f"L={'OPEN' if left_open else 'BLOCK'} "
            f"R={'OPEN' if right_open else 'BLOCK'}"
        )

    return {
        "front_cm": front_cm,
        "left_cm": left_cm,
        "right_cm": right_cm,
        "front_open": front_open,
        "front_blocked": front_blocked,
        "left_open": left_open,
        "right_open": right_open,
        "raw_front_open": raw_front_open,
        "raw_left_open": raw_left_open,
        "raw_right_open": raw_right_open,
    }


def _pose_xy(pose_tracker):
    x, y, _ = pose_tracker.get_pose()
    return x, y


def _travelled_m(start_x, start_y, pose_tracker):
    x, y = _pose_xy(pose_tracker)
    if start_x is None or start_y is None or x is None or y is None:
        return None
    return ((x - start_x) ** 2 + (y - start_y) ** 2) ** 0.5



def center_on_opening_zone(
    chassis,
    controller,
    pose_tracker,
    zone_event,
):
    """Reverse from the far edge of a measured side opening to its midpoint.

    The robot has just traversed this corridor segment, so the path directly
    behind it is known free. We nevertheless cap the backtrack distance and
    timeout. No rear range sensor is assumed.
    """
    if not zone_event:
        return
    if not getattr(config, "ENABLE_OPENING_ZONE_CENTERING", True):
        return
    if not config.ENABLE_MOTION:
        print("OPENING_ZONE_CENTER skipped: motion disabled")
        return

    length = max(0.0, float(zone_event.get("length_m", 0.0)))
    requested_backtrack = zone_event.get("backtrack_m")
    if requested_backtrack is None:
        requested_backtrack = 0.5 * length
    target = min(
        max(0.0, float(requested_backtrack))
        + float(getattr(config, "OPENING_ZONE_CENTER_REVERSE_BIAS_M", 0.0)),
        float(config.OPENING_ZONE_CENTERING_MAX_BACKTRACK_M),
    )
    if target <= 0.005:
        return

    start_x, start_y = _pose_xy(pose_tracker)
    start_time = time.monotonic()
    print(
        f">>> INTERSECTION_CENTER type={zone_event.get('type')} "
        f"span={zone_event.get('opening_span_m', length):.3f}m "
        f"window={length:.3f}m backtrack={target:.3f}m"
    )

    while time.monotonic() - start_time < config.OPENING_ZONE_CENTERING_MAX_SEC:
        travelled = _travelled_m(start_x, start_y, pose_tracker)
        if travelled is not None and travelled >= target:
            print(f"OPENING_ZONE_CENTER done: travelled={travelled:.3f} m")
            break

        back_x, back_y, back_z, _, _ = controller.apply_heading_hold(
            -config.OPENING_ZONE_CENTERING_SPEED,
            0.0,
            pose_tracker.get_yaw(),
            pose_tracker,
            "OPENING_ZONE_CENTER",
        )
        chassis.drive_speed(
            x=back_x,
            y=back_y,
            z=back_z,
            timeout=config.DRIVE_TIMEOUT_SEC,
        )
        time.sleep(config.OPENING_ZONE_CENTERING_LOOP_SEC)

    stop_chassis(chassis)

def align_to_selected_side_opening(
    chassis,
    sensors,
    controller,
    pose_tracker,
    direction,
    raw_side_open,
):
    """Ensure the pivot is physically inside the chosen side opening.

    Intersection Window intentionally remembers openings seen while moving.
    That is good for topology, but the final stopped position can be a few cm
    beyond a mouth.  Never rotate into a remembered LEFT/RIGHT branch unless
    Sharp at that side confirms usable clearance.  Search backward first,
    matching the field failure observed on the real maze.
    """
    if direction not in ("LEFT", "RIGHT"):
        return True
    if not getattr(config, "ENABLE_TURN_ENTRY_REALIGN", True):
        return True
    if raw_side_open:
        return True
    if not config.ENABLE_MOTION:
        return False

    print(
        f">>> TURN_ENTRY_REALIGN {direction}: accumulated opening exists "
        "but stopped Sharp is not open; searching backward"
    )
    sx, sy = _pose_xy(pose_tracker)
    t0 = time.monotonic()
    good = 0

    while time.monotonic() - t0 < config.TURN_ENTRY_MAX_SEC:
        travelled = _travelled_m(sx, sy, pose_tracker)
        if travelled is not None and travelled >= config.TURN_ENTRY_MAX_BACKTRACK_M:
            break

        _, left_cm = sensors.read_left_sharp()
        _, right_cm = sensors.read_right_sharp()
        side_cm = left_cm if direction == "LEFT" else right_cm
        if side_cm is not None and side_cm >= config.TURN_ENTRY_OPEN_CM:
            good += 1
            if good >= config.TURN_ENTRY_CONFIRM_SAMPLES:
                stop_chassis(chassis)
                print(
                    f">>> TURN_ENTRY_REALIGN OK side={side_cm:.1f}cm "
                    f"backtracked={float(travelled or 0.0):.3f}m"
                )
                return True
        else:
            good = 0

        bx, by, bz, _, _ = controller.apply_heading_hold(
            -config.TURN_ENTRY_SEARCH_SPEED,
            0.0,
            pose_tracker.get_yaw(),
            pose_tracker,
            "TURN_ENTRY_REALIGN",
        )
        chassis.drive_speed(x=bx, y=by, z=bz, timeout=config.DRIVE_TIMEOUT_SEC)
        time.sleep(config.TURN_ENTRY_LOOP_SEC)

    stop_chassis(chassis)
    _, left_cm = sensors.read_left_sharp()
    _, right_cm = sensors.read_right_sharp()
    side_cm = left_cm if direction == "LEFT" else right_cm
    print(
        f">>> TURN_ENTRY_REALIGN FAILED {direction} "
        f"side={side_cm if side_cm is not None else 'None'}; turn cancelled safely"
    )
    return False


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


class StartGateGuard:
    """Geometric + topological protection for the physical entrance.

    The first robot heading is defined as the INWARD maze direction.  Instead of
    assuming how RoboMaster raw x/y axes are oriented, the guard learns an inward
    unit vector from the first ~12 cm of real odometry.  The line through the
    initial pose perpendicular to that vector is the START GATE.

    The planner already blocks the outside absolute direction at J0.  This class
    is a second safety layer for cases where open-area/exit logic or imperfect
    junction recognition lets the chassis approach the entrance without a normal
    J0 decision event.
    """

    def __init__(self, start_x, start_y, inside_abs_dir=0):
        self.start_x = float(start_x)
        self.start_y = float(start_y)
        self.inside_abs_dir = int(inside_abs_dir) % 4
        self.outside_abs_dir = (self.inside_abs_dir + 2) % 4
        self.inward_unit = None
        self.last_recovery_time = -1e9
        self._learn_announced = False
        self._reject_announced = False

    def observe(self, x, y):
        if x is None or y is None:
            return self.metrics(x, y)
        if self.inward_unit is None:
            dx = float(x) - self.start_x
            dy = float(y) - self.start_y
            distance = math.hypot(dx, dy)
            if distance >= float(config.START_GATE_LEARN_DISTANCE_M):
                self.inward_unit = (dx / distance, dy / distance)
                if not self._learn_announced:
                    print(
                        ">>> START_GATE LEARNED "
                        f"inward=({self.inward_unit[0]:+.3f},"
                        f"{self.inward_unit[1]:+.3f}) from {distance:.3f}m"
                    )
                    self._learn_announced = True
        return self.metrics(x, y)

    def metrics(self, x, y):
        if x is None or y is None:
            return {
                "learned": self.inward_unit is not None,
                "distance_m": None,
                "progress_m": None,
                "lateral_m": None,
            }
        dx = float(x) - self.start_x
        dy = float(y) - self.start_y
        distance = math.hypot(dx, dy)
        if self.inward_unit is None:
            return {
                "learned": False,
                "distance_m": distance,
                "progress_m": None,
                "lateral_m": None,
            }
        ux, uy = self.inward_unit
        progress = dx * ux + dy * uy
        lateral = abs(-uy * dx + ux * dy)
        return {
            "learned": True,
            "distance_m": distance,
            "progress_m": progress,
            "lateral_m": lateral,
        }

    def is_outward_heading(self, heading_index):
        return int(heading_index) % 4 == self.outside_abs_dir

    def should_reject_exit(self, x, y, heading_index):
        if not bool(getattr(config, "ENABLE_START_GATE_GUARD", True)):
            return False
        m = self.metrics(x, y)
        distance = m["distance_m"]
        if distance is not None and distance <= float(config.START_EXIT_REJECT_RADIUS_M):
            return True
        if (
            m["learned"]
            and self.is_outward_heading(heading_index)
            and m["progress_m"] is not None
            and m["progress_m"] <= float(config.START_EXIT_REJECT_INNER_PROGRESS_M)
            and m["lateral_m"] is not None
            and m["lateral_m"] <= float(config.START_EXIT_REJECT_LATERAL_M)
        ):
            return True
        return False

    def should_force_return(self, x, y, heading_index):
        if not bool(getattr(config, "ENABLE_START_GATE_GUARD", True)):
            return False
        if not self.is_outward_heading(heading_index):
            return False
        m = self.metrics(x, y)
        if not m["learned"]:
            return False
        if m["progress_m"] is None or m["lateral_m"] is None:
            return False
        in_gate = (
            m["progress_m"] <= float(config.START_GATE_BLOCK_INNER_M)
            and m["lateral_m"] <= float(config.START_GATE_HALF_WIDTH_M)
        )
        cooldown_ok = (
            time.monotonic() - self.last_recovery_time
            >= float(config.START_GATE_RECOVERY_COOLDOWN_SEC)
        )
        return in_gate and cooldown_ok

    def mark_recovery(self):
        self.last_recovery_time = time.monotonic()


class OpenAreaExitManager:
    """Hysteretic open-area driving state + conservative exit detector.

    OPEN_AREA is a motion-mode hint only: while active, main() disables normal
    Sharp wall-centering and lets the existing attitude heading-hold keep the
    chassis straight. Dangerously-close Sharp readings still override it.

    EXIT uses a second, much stricter state. It requires a very long front
    range, both sides very open, enough explored graph nodes/runtime, and a
    sustained forward displacement. This prevents a brief 4-way intersection
    from being labelled as the maze exit.
    """

    def __init__(self):
        self.started_at = time.monotonic()
        self.open_area_active = False
        self.open_enter_count = 0
        self.open_exit_count = 0

        self.exit_start_count = 0
        self.exit_candidate = None
        self.exit_found = False
        self.exit_event = None

    @staticmethod
    def _distance_xy(x1, y1, x2, y2):
        if None in (x1, y1, x2, y2):
            return None
        return math.hypot(float(x2) - float(x1), float(y2) - float(y1))

    @staticmethod
    def _at_least(value, threshold):
        return value is not None and float(value) >= float(threshold)

    @staticmethod
    def _at_most(value, threshold):
        return value is not None and 0.0 < float(value) <= float(threshold)

    def _cancel_exit_candidate(self, reason=None):
        had = self.exit_candidate is not None or self.exit_start_count > 0
        if had and reason:
            print(f">>> EXIT_CANDIDATE CANCEL reason={reason}")
        self.exit_candidate = None
        self.exit_start_count = 0

    def cancel_exit_candidate(self, reason=None):
        """Public cancellation hook used by V7 junction/start-gate safety."""
        self._cancel_exit_candidate(reason)

    def update(
        self, front_cm, left_cm, right_cm, pose_x, pose_y,
        node_count=0, heading_error=None, start_gate_block_exit=False,
    ):
        now = time.monotonic()
        runtime = now - self.started_at
        candidate_started = False
        candidate_cancelled = False
        open_area_entered = False
        open_area_left = False

        front_blocked = self._at_most(front_cm, config.STOP_FRONT_CM)
        broad_open_sample = (
            self._at_least(front_cm, config.OPEN_AREA_FRONT_MIN_CM)
            and self._at_least(left_cm, config.OPEN_AREA_SIDE_ENTER_CM)
            and self._at_least(right_cm, config.OPEN_AREA_SIDE_ENTER_CM)
        )
        wall_reacquired = (
            self._at_most(left_cm, config.OPEN_AREA_SIDE_EXIT_CM)
            or self._at_most(right_cm, config.OPEN_AREA_SIDE_EXIT_CM)
        )

        # Open-area hysteresis. A front hard-stop exits immediately for safety.
        if front_blocked:
            if self.open_area_active:
                open_area_left = True
                print(">>> OPEN_AREA EXIT reason=FRONT_BLOCKED")
            self.open_area_active = False
            self.open_enter_count = 0
            self.open_exit_count = 0
        elif not self.open_area_active:
            self.open_enter_count = self.open_enter_count + 1 if broad_open_sample else 0
            if self.open_enter_count >= int(config.OPEN_AREA_ENTER_SAMPLES):
                self.open_area_active = True
                self.open_enter_count = 0
                self.open_exit_count = 0
                open_area_entered = True
                print(
                    ">>> OPEN_AREA ENTER "
                    f"F={fmt(front_cm)} L={fmt(left_cm)} R={fmt(right_cm)}"
                )
        else:
            self.open_exit_count = self.open_exit_count + 1 if wall_reacquired else 0
            if self.open_exit_count >= int(config.OPEN_AREA_EXIT_SAMPLES):
                self.open_area_active = False
                self.open_exit_count = 0
                self.open_enter_count = 0
                open_area_left = True
                print(
                    ">>> OPEN_AREA EXIT reason=WALL_REACQUIRED "
                    f"L={fmt(left_cm)} R={fmt(right_cm)}"
                )

        # V7: the entrance can look exactly like a true wide-open exit.  If the
        # geometric START-GATE guard says we are near/facing the entrance, an
        # exit candidate is forbidden and any existing candidate is cancelled.
        if start_gate_block_exit:
            if self.exit_candidate is not None or self.exit_start_count > 0:
                self._cancel_exit_candidate("START_GATE")
            self.exit_start_count = 0

        heading_ok = (
            heading_error is None
            or abs(float(heading_error)) <= float(config.EXIT_MAX_HEADING_ERROR_DEG)
        )
        enough_history = (
            runtime >= float(config.EXIT_MIN_RUNTIME_SEC)
            and int(node_count) >= int(config.EXIT_MIN_NODE_COUNT)
        )
        exit_strong = (
            bool(config.ENABLE_EXIT_DETECTION)
            and not start_gate_block_exit
            and self.open_area_active
            and enough_history
            and heading_ok
            and self._at_least(front_cm, config.EXIT_FRONT_START_CM)
            and self._at_least(left_cm, config.EXIT_SIDE_START_CM)
            and self._at_least(right_cm, config.EXIT_SIDE_START_CM)
        )
        exit_keep = (
            not start_gate_block_exit
            and self.open_area_active
            and heading_ok
            and self._at_least(front_cm, config.EXIT_FRONT_KEEP_CM)
            and self._at_least(left_cm, config.EXIT_SIDE_KEEP_CM)
            and self._at_least(right_cm, config.EXIT_SIDE_KEEP_CM)
        )

        if self.exit_found:
            return {
                "open_area_active": self.open_area_active,
                "open_area_entered": open_area_entered,
                "open_area_left": open_area_left,
                "exit_candidate_active": False,
                "exit_candidate_started": False,
                "exit_candidate_cancelled": False,
                "exit_found": True,
                "exit_event": self.exit_event,
            }

        if self.exit_candidate is None:
            self.exit_start_count = self.exit_start_count + 1 if exit_strong else 0
            if self.exit_start_count >= int(config.EXIT_START_SAMPLES):
                self.exit_candidate = {
                    "start_x": pose_x,
                    "start_y": pose_y,
                    "start_time": now,
                    "strong_samples": int(self.exit_start_count),
                    "min_front_cm": float(front_cm),
                    "min_left_cm": float(left_cm),
                    "min_right_cm": float(right_cm),
                }
                candidate_started = True
                print(
                    ">>> EXIT_CANDIDATE START "
                    f"F={front_cm:.1f} L={left_cm:.1f} R={right_cm:.1f} "
                    f"confirm={config.EXIT_CONFIRM_DISTANCE_M:.2f}m"
                )
        else:
            c = self.exit_candidate
            if not exit_keep:
                candidate_cancelled = True
                self._cancel_exit_candidate("OPENNESS_LOST")
            else:
                if exit_strong:
                    c["strong_samples"] += 1
                c["min_front_cm"] = min(c["min_front_cm"], float(front_cm))
                c["min_left_cm"] = min(c["min_left_cm"], float(left_cm))
                c["min_right_cm"] = min(c["min_right_cm"], float(right_cm))

                travelled = self._distance_xy(
                    c["start_x"], c["start_y"], pose_x, pose_y
                )
                elapsed = now - c["start_time"]
                distance_ok = (
                    travelled is not None
                    and travelled >= float(config.EXIT_CONFIRM_DISTANCE_M)
                )
                time_ok = elapsed >= float(config.EXIT_CONFIRM_MIN_SEC)
                samples_ok = (
                    c["strong_samples"] >= int(config.EXIT_CONFIRM_STRONG_SAMPLES)
                )
                if distance_ok and time_ok and samples_ok:
                    self.exit_found = True
                    self.exit_event = {
                        "raw_x": None if pose_x is None else float(pose_x),
                        "raw_y": None if pose_y is None else float(pose_y),
                        "travelled_m": float(travelled),
                        "confirm_sec": float(elapsed),
                        "strong_samples": int(c["strong_samples"]),
                        "front_cm": None if front_cm is None else float(front_cm),
                        "left_cm": None if left_cm is None else float(left_cm),
                        "right_cm": None if right_cm is None else float(right_cm),
                        "node_count": int(node_count),
                        "runtime_sec": float(runtime),
                    }
                    print(
                        ">>> EXIT FOUND "
                        f"travelled={travelled:.3f}m time={elapsed:.2f}s "
                        f"F={front_cm:.1f} L={left_cm:.1f} R={right_cm:.1f}"
                    )

        return {
            "open_area_active": self.open_area_active,
            "open_area_entered": open_area_entered,
            "open_area_left": open_area_left,
            "exit_candidate_active": self.exit_candidate is not None and not self.exit_found,
            "exit_candidate_started": candidate_started,
            "exit_candidate_cancelled": candidate_cancelled,
            "exit_found": self.exit_found,
            "exit_event": self.exit_event,
        }

"""RoboMaster Maze Explorer V10.1 modular entry point."""

try:
    from robomaster import robot
except ModuleNotFoundError as exc:
    raise SystemExit(
        "RoboMaster SDK is not installed for this Python. "
        "Install it first, then run this file again."
    ) from exc

import time
import config
from pose_tracker import PoseTracker
from sensors import SensorManager
from motion_controller import MotionController
from exploration import DecisionPointDetector, ExplorationDecision, TremauxExplorer
from navigation import decision_from_relative, execute_turn, print_exploration_decision
from maze_runtime import (
    OpenAreaExitManager,
    StartGateGuard,
    _pose_xy,
    align_heading_in_place,
    align_to_selected_side_opening,
    apply_motion_safety,
    center_on_opening_zone,
    corner_turn_setup,
    creep_to_junction_center,
    fmt,
    fmt_adc,
    post_turn_clearance,
    print_startup_info,
    scan_decision_point,
    stop_chassis,
    wait_for_pose,
    wait_for_yaw,
)
from mapping import SLAMStyleMazeMapper

def main():
    ep_robot = robot.Robot()
    chassis = None
    tof_sensor = None

    pose_subscribed = False
    attitude_subscribed = False
    tof_subscribed = False
    mapper = None

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
        open_area_exit = OpenAreaExitManager()
        mapper = SLAMStyleMazeMapper()

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
        start_gate = StartGateGuard(
            start_x, start_y,
            inside_abs_dir=(
                explorer.start_inside_abs_dir
                if explorer.start_inside_abs_dir is not None
                else explorer.heading_index
            ),
        )
        print(
            f">>> START_GATE armed: inside={explorer.heading_name(start_gate.inside_abs_dir)} "
            f"outside={explorer.heading_name(start_gate.outside_abs_dir)}"
        )

        if mapper is not None and config.ENABLE_MAPPING:
            mapper.initialize(
                start_x, start_y, start_yaw,
                heading_index=explorer.heading_index,
            )
            mapper.observe_junction(
                start_node, True, start_x, start_y, start_yaw,
                heading_index=explorer.heading_index,
            )

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

            # V10.1: if a Sharp stream is unavailable longer than the short
            # cache window, stop and keep polling instead of driving blind or
            # letting None reach arithmetic/median code.
            if sharp_left_cm is None or sharp_right_cm is None:
                stop_chassis(chassis)
                controller.reset_side_owner()
                detector.cancel_event()
                open_area_exit.cancel_exit_candidate("SHARP_SENSOR_MISSING")
                missing = []
                if sharp_left_cm is None:
                    missing.append("LEFT")
                if sharp_right_cm is None:
                    missing.append("RIGHT")
                print(
                    ">>> SHARP SENSOR HOLD: missing=" + ",".join(missing)
                    + " | robot stopped; waiting for sensor recovery"
                )
                time.sleep(config.SHARP_SENSOR_RECOVERY_DELAY_SEC)
                continue

            pose_x, pose_y, _ = pose_tracker.get_pose()

            x = 0.0
            y = 0.0
            z = 0.0
            mode = "STOP"
            heading_error = controller.heading_error(pose_tracker.get_yaw())

            front_blocked_now = (
                front_cm is not None
                and 0.0 < front_cm <= config.STOP_FRONT_CM
            )

            # -------------------------------------------------
            # V7 START-GATE geometric safety layer
            # -------------------------------------------------
            start_gate.observe(pose_x, pose_y)
            start_gate_block_exit = start_gate.should_reject_exit(
                pose_x, pose_y, explorer.heading_index
            )

            if start_gate.should_force_return(
                pose_x, pose_y, explorer.heading_index
            ):
                metrics = start_gate.metrics(pose_x, pose_y)
                print()
                print("============================================")
                print(" START GATE GUARD - RETURNING INTO MAZE")
                print(
                    f" progress={metrics.get('progress_m', 0.0):+.3f}m "
                    f"lateral={metrics.get('lateral_m', 0.0):.3f}m "
                    f"heading={explorer.heading_name()}"
                )
                print("============================================")

                stop_chassis(chassis)
                detector.cancel_event()
                open_area_exit.cancel_exit_candidate("START_GATE_FORCE_RETURN")
                controller.reset_side_owner()

                # Snap the topological arrival to the known start node.  This
                # closes the return corridor before forcing the only legal
                # departure: back INTO the maze.
                node_id, _ = explorer.arrive_at_decision_point(start_x, start_y)
                inside_abs = (
                    explorer.start_inside_abs_dir
                    if explorer.start_inside_abs_dir is not None
                    else start_gate.inside_abs_dir
                )
                relative = explorer.relative_for_absolute(inside_abs)
                inside_state = explorer._exit(node_id, inside_abs)
                guard_decision = ExplorationDecision(
                    direction=relative,
                    node_id=node_id,
                    reason="START_GATE_RETURN_TO_MAZE",
                    visits_before=inside_state.visits,
                    absolute_heading=explorer.heading_name(inside_abs),
                )
                print_exploration_decision(guard_decision)

                turn_ok = execute_turn(
                    chassis, decision_from_relative(relative),
                    pose_tracker=pose_tracker,
                )
                if not turn_ok:
                    stop_chassis(chassis)
                    print("START_GATE recovery turn failed safely; stopping.")
                    break

                explorer.commit_decision(guard_decision)
                controller.set_heading_index(
                    explorer.heading_index, pose_tracker=pose_tracker
                )
                align_heading_in_place(chassis, controller, pose_tracker)
                controller.reset_after_turn()
                sensors.reset_filters()
                start_gate.mark_recovery()

                rx, ry, _ = pose_tracker.get_pose()
                detector.force_latched(
                    rx if rx is not None else start_x,
                    ry if ry is not None else start_y,
                )

                if mapper is not None and config.ENABLE_MAPPING:
                    mx, my, myaw = pose_tracker.get_pose()
                    mapper.update(
                        mx, my, myaw,
                        front_cm=None, left_cm=None, right_cm=None,
                        ir_value=ir_left_wall,
                        heading_index=explorer.heading_index,
                        mode="START_GATE_RETURN", map_ranges=False, force=True,
                    )
                if config.SAVE_MAZE_MEMORY:
                    explorer.save_memory()
                time.sleep(config.AFTER_TURN_DELAY_SEC)
                continue

            open_state = open_area_exit.update(
                front_cm, sharp_left_cm, sharp_right_cm,
                pose_x, pose_y,
                node_count=len(explorer.nodes),
                heading_error=heading_error,
                start_gate_block_exit=start_gate_block_exit,
            )

            # V7 deliberately keeps the decision detector alive while an exit
            # candidate is being verified. A real junction must interrupt EXIT
            # confirmation rather than being driven through blindly.

            if open_state["exit_found"]:
                stop_chassis(chassis)
                if mapper is not None and config.ENABLE_MAPPING:
                    mx, my, myaw = pose_tracker.get_pose()
                    mapper.update(
                        mx, my, myaw,
                        front_cm=front_cm, left_cm=sharp_left_cm, right_cm=sharp_right_cm,
                        ir_value=ir_left_wall, heading_index=explorer.heading_index,
                        mode="EXIT_FOUND", map_ranges=True, force=True,
                    )
                    mapper.mark_exit(
                        mx, my, myaw,
                        heading_index=explorer.heading_index,
                        details=open_state.get("exit_event"),
                    )
                    mapper.save_all(rebuild=True, quiet=False)

                if config.SAVE_MAZE_MEMORY:
                    explorer.save_memory()

                print()
                print("============================================")
                print(" MAZE EXIT FOUND - OPEN AREA CONFIRMED")
                print("============================================")
                if config.STOP_WHEN_EXIT_FOUND:
                    break

            decision_event = detector.update(
                front_cm,
                sharp_left_cm,
                sharp_right_cm,
                pose_x=pose_x,
                pose_y=pose_y,
            )

            if decision_event and open_state["exit_candidate_active"]:
                open_area_exit.cancel_exit_candidate("JUNCTION_DETECTED")
                open_state["exit_candidate_active"] = False

            # =================================================
            # DECISION POINT
            # =================================================

            if decision_event:
                if mapper is not None and config.ENABLE_MAPPING:
                    mapper.update(
                        pose_x, pose_y, pose_tracker.get_yaw(),
                        front_cm=front_cm, left_cm=sharp_left_cm, right_cm=sharp_right_cm,
                        ir_value=ir_left_wall, heading_index=explorer.heading_index,
                        mode="DECISION_TRIGGER", map_ranges=True, force=True,
                    )
                controller.reset_side_owner()
                stop_chassis(chassis)
                mode = "DFS_DECISION"

                # V6 side-junction events carry an accumulated intersection window.
                # Move back to the estimated centre of the union of observed
                # side openings, then merge the stopped scan with the directions
                # seen during the moving window. Front-only dead-end events have
                # no window metadata and keep the older creep/corner path.
                zone_event = detector.consume_pending_zone()

                if zone_event is not None:
                    center_on_opening_zone(
                        chassis,
                        controller,
                        pose_tracker,
                        zone_event,
                    )
                else:
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

                scan = scan_decision_point(detector, sensors, zone_event)

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

                if mapper is not None and config.ENABLE_MAPPING:
                    mapper.update(
                        pose_x, pose_y, pose_tracker.get_yaw(),
                        front_cm=scan["front_cm"], left_cm=scan["left_cm"], right_cm=scan["right_cm"],
                        ir_value=sensors.read_ir_digital_io(), heading_index=explorer.heading_index,
                        mode="JUNCTION_SCAN", map_ranges=True, force=True,
                    )

                node_id, is_new = explorer.arrive_at_decision_point(
                    pose_x,
                    pose_y,
                )

                if mapper is not None and config.ENABLE_MAPPING:
                    map_event = mapper.observe_junction(
                        node_id, is_new, pose_x, pose_y, pose_tracker.get_yaw(),
                        heading_index=explorer.heading_index,
                    )
                    if map_event and map_event.get("corrected"):
                        print(
                            f"MAP LOOP CLOSURE: {node_id} "
                            f"error={map_event.get('error_m', 0.0):.3f} m"
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

                # V10: accumulated intersection memory can remember a branch
                # that is no longer beside the chassis after centering.  Before
                # rotating, require the selected side to be physically open at
                # the current pivot; otherwise backtrack a few centimetres to
                # re-find the mouth.
                raw_side_open = True
                if exploration_decision.direction == "LEFT":
                    raw_side_open = bool(scan.get("raw_left_open", False))
                elif exploration_decision.direction == "RIGHT":
                    raw_side_open = bool(scan.get("raw_right_open", False))

                entry_ok = align_to_selected_side_opening(
                    chassis,
                    sensors,
                    controller,
                    pose_tracker,
                    exploration_decision.direction,
                    raw_side_open,
                )
                if not entry_ok:
                    # Do not commit the graph edge and, most importantly, do
                    # not rotate into a wall.  Rearm detector and approach the
                    # junction again so the branch can be retried safely.
                    print(">>> TURN CANCELLED: selected opening not aligned with chassis")
                    detector.cancel_event()
                    controller.reset_corridor_heading_calibration()
                    stop_chassis(chassis)
                    time.sleep(config.AFTER_TURN_DELAY_SEC)
                    continue

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

                if mapper is not None and config.ENABLE_MAPPING:
                    tx, ty, tyaw = pose_tracker.get_pose()
                    mapper.record_pose(
                        tx, ty, tyaw,
                        heading_index=explorer.heading_index,
                        mode="AFTER_TURN", force=True,
                    )

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

                side_danger = (
                    (sharp_left_cm is not None and sharp_left_cm <= config.SIDE_TOO_CLOSE_CM)
                    or (sharp_right_cm is not None and sharp_right_cm <= config.SIDE_TOO_CLOSE_CM)
                )

                if (
                    config.ENABLE_OPEN_AREA_HEADING_HOLD
                    and open_state["open_area_active"]
                    and not side_danger
                ):
                    # Do not let a distant/noisy Sharp reading pull the chassis
                    # sideways in a plaza/open room. Existing yaw heading-hold
                    # below still keeps the robot square to its chosen grid direction.
                    controller.reset_side_owner()
                    y = 0.0
                    z = 0.0
                    if open_state["exit_candidate_active"]:
                        x = min(x, config.EXIT_CANDIDATE_SPEED)
                        mode = "EXIT_CANDIDATE_HEADING_HOLD"
                    else:
                        mode = "OPEN_AREA_HEADING_HOLD"
                else:
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

                # V10: use stable corridor walls as a very slow external yaw
                # reference.  Freeze this adaptation near junctions/open areas
                # and while any side is dangerously close.
                corridor_cal_allowed = (
                    not side_danger
                    and not open_state["open_area_active"]
                    and not detector.intersection_window.get("active", False)
                    and not detector.left_zone.get("active", False)
                    and not detector.right_zone.get("active", False)
                    and mode not in ("HEADING_RECOVER",)
                )
                controller.update_corridor_heading_reference(
                    sharp_left_cm,
                    sharp_right_cm,
                    front_cm,
                    pose_tracker,
                    explorer.heading_index,
                    allow=corridor_cal_allowed,
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
            # PASSIVE OCCUPANCY MAPPING
            # =================================================

            if mapper is not None and config.ENABLE_MAPPING:
                map_x_raw, map_y_raw, map_yaw = pose_tracker.get_pose()
                mapper.update(
                    map_x_raw, map_y_raw, map_yaw,
                    front_cm=front_cm,
                    left_cm=sharp_left_cm,
                    right_cm=sharp_right_cm,
                    ir_value=ir_left_wall,
                    heading_index=explorer.heading_index,
                    mode=mode,
                    map_ranges=True,
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
                f"L:{fmt(sharp_left_cm)} ADC:{fmt_adc(raw_adc_l)} | "
                f"R:{fmt(sharp_right_cm)} ADC:{fmt_adc(raw_adc_r)} | "
                f"IR:{ir_text} | "
                f"D:{delta:+5.1f} | "
                f"POSE:{pose_text} | "
                f"YAW:{yaw_text}/{target_text} "
                f"E:{heading_error_text} | "
                f"H:{explorer.heading_name()} | "
                f"OWNER:{controller.side_owner:5s} | "
                f"OA:{int(open_state['open_area_active'])} | "
                f"EXITC:{int(open_state['exit_candidate_active'])} | "
                f"LATCH:{int(detector.latched)} | "
                f"{mode:28s} | "
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
            if mapper is not None and getattr(config, "ENABLE_MAPPING", False):
                mapper.save_all(rebuild=True, quiet=False)
        except Exception as map_exc:
            print("MAPPER SAVE ERROR:", map_exc)

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

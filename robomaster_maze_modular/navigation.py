"""Turn decisions and feedback-controlled RoboMaster rotation."""

import config
from pose_tracker import normalize_angle_deg, shortest_angle_error_deg

# ==================== NAVIGATION / TURN ====================
"""Translate exploration decisions into RoboMaster turn commands."""

from dataclasses import dataclass
import math
import time



@dataclass(frozen=True)
class TurnDecision:
    name: str
    angle_deg: float


RELATIVE_TO_TURN = {
    "FRONT": TurnDecision("FRONT", 0.0),
    "FORWARD": TurnDecision("FRONT", 0.0),
    "LEFT": TurnDecision("LEFT_90", config.TURN_LEFT_DEG),
    "RIGHT": TurnDecision("RIGHT_90", config.TURN_RIGHT_DEG),
    "BACK": TurnDecision("BACK_180", config.TURN_AROUND_DEG),
    "COMPLETE": TurnDecision("COMPLETE", 0.0),
}


def decision_from_relative(relative_direction):
    try:
        return RELATIVE_TO_TURN[relative_direction]
    except KeyError as exc:
        raise ValueError(f"Unknown relative direction: {relative_direction}") from exc


def print_exploration_decision(exploration_decision):
    print()
    print("========== TRÉMAUX / DFS DECISION ==========")
    print(f"NODE       : {exploration_decision.node_id}")
    print(f"DIRECTION  : {exploration_decision.direction}")
    print(f"ABS HEADING: {exploration_decision.absolute_heading}")
    print(f"MARK BEFORE: {exploration_decision.visits_before}")
    print(f"REASON     : {exploration_decision.reason}")
    print("============================================")


def _safe_stop(chassis):
    try:
        chassis.drive_speed(x=0.0, y=0.0, z=0.0, timeout=0.2)
    except Exception:
        pass


def _clamp(value, low, high):
    return max(low, min(high, value))


def _feedback_turn(chassis, decision, pose_tracker):
    """Closed-loop turn using attitude yaw + drive_speed().

    V8 keeps one absolute target yaw for all retry attempts. This is critical:
    if the first attempt reaches most of a 90-degree turn and telemetry stalls,
    a retry must finish the remaining error, not command another full 90 deg.
    """
    command_deg = decision.angle_deg * config.Z_DIR_SIGN
    if abs(command_deg) < 0.001:
        return True

    start_yaw = pose_tracker.get_yaw()
    if start_yaw is None:
        return None

    move_sign = pose_tracker.get_move_to_yaw_sign()
    if move_sign not in (-1, 1):
        move_sign = config.DEFAULT_MOVE_TO_YAW_SIGN
        pose_tracker.set_move_to_yaw_sign(move_sign)

    drive_sign = pose_tracker.get_drive_to_yaw_sign()
    if drive_sign not in (-1, 1):
        drive_sign = config.DEFAULT_DRIVE_TO_YAW_SIGN
        pose_tracker.set_drive_to_yaw_sign(drive_sign)

    # Preserve the field-verified logical turn convention.
    target_yaw = normalize_angle_deg(start_yaw + command_deg * move_sign)
    timeout_sec = (
        config.TURN_FEEDBACK_TIMEOUT_180_SEC
        if abs(command_deg) > 135.0
        else config.TURN_FEEDBACK_TIMEOUT_90_SEC
    )
    max_attempts = max(1, int(getattr(config, "TURN_MAX_ATTEMPTS", 1)))
    timeout_accept = float(
        getattr(
            config,
            "TURN_TIMEOUT_ACCEPT_TOLERANCE_DEG",
            config.TURN_FEEDBACK_TOLERANCE_DEG,
        )
    )

    print(
        f">>> TURN {decision.name} [FEEDBACK]: command={command_deg:+.1f} deg "
        f"start_yaw={start_yaw:+.1f} target={target_yaw:+.1f} "
        f"max_attempts={max_attempts}"
    )

    _safe_stop(chassis)
    time.sleep(config.TURN_PRE_SETTLE_SEC)

    try:
        for attempt in range(1, max_attempts + 1):
            if attempt > 1:
                current = pose_tracker.get_yaw()
                remaining = (
                    shortest_angle_error_deg(target_yaw, current)
                    if current is not None
                    else None
                )
                print(
                    f">>> TURN RETRY {attempt}/{max_attempts} SAME TARGET "
                    + (
                        f"yaw={current:+.1f} remaining={remaining:+.1f} deg"
                        if current is not None
                        else "yaw unavailable"
                    )
                )
                _safe_stop(chassis)
                time.sleep(getattr(config, "TURN_RETRY_SETTLE_SEC", 0.25))

            started = time.monotonic()
            stable_samples = 0
            last_print = 0.0

            while True:
                now = time.monotonic()
                current_yaw = pose_tracker.get_yaw()

                if current_yaw is None:
                    if now - started >= timeout_sec:
                        _safe_stop(chassis)
                        print(
                            f"TURN ATTEMPT {attempt}/{max_attempts} TIMEOUT: "
                            "attitude yaw unavailable."
                        )
                        break
                    time.sleep(config.TURN_FEEDBACK_LOOP_SEC)
                    continue

                error = shortest_angle_error_deg(target_yaw, current_yaw)
                abs_error = abs(error)

                if abs_error <= config.TURN_FEEDBACK_TOLERANCE_DEG:
                    stable_samples += 1
                    _safe_stop(chassis)

                    if stable_samples >= config.TURN_FEEDBACK_STABLE_SAMPLES:
                        time.sleep(config.YAW_SETTLE_SEC)
                        final_yaw = pose_tracker.get_yaw()
                        final_error = (
                            shortest_angle_error_deg(target_yaw, final_yaw)
                            if final_yaw is not None
                            else error
                        )
                        print(
                            f"TURN OK: yaw={final_yaw:+.1f} target={target_yaw:+.1f} "
                            f"error={final_error:+.1f} deg"
                            if final_yaw is not None
                            else "TURN OK"
                        )
                        return True
                else:
                    stable_samples = 0

                    speed = abs_error * config.TURN_FEEDBACK_KP
                    speed = _clamp(
                        speed,
                        config.TURN_FEEDBACK_MIN_Z_SPEED,
                        config.TURN_FEEDBACK_MAX_Z_SPEED,
                    )

                    # yaw_rate = drive_speed_z * drive_to_yaw_sign
                    z_cmd = math.copysign(speed, error) / drive_sign

                    chassis.drive_speed(
                        x=0.0,
                        y=0.0,
                        z=z_cmd,
                        timeout=config.TURN_FEEDBACK_DRIVE_TIMEOUT_SEC,
                    )

                    if now - last_print >= config.TURN_FEEDBACK_PRINT_SEC:
                        print(
                            f"    turn yaw={current_yaw:+.1f} target={target_yaw:+.1f} "
                            f"err={error:+.1f} z={z_cmd:+.1f} "
                            f"attempt={attempt}/{max_attempts}"
                        )
                        last_print = now

                if now - started >= timeout_sec:
                    _safe_stop(chassis)
                    final_yaw = pose_tracker.get_yaw()
                    final_error = (
                        shortest_angle_error_deg(target_yaw, final_yaw)
                        if final_yaw is not None
                        else None
                    )

                    # RoboMaster attitude telemetry can freeze briefly. If the
                    # chassis is already close enough at watchdog expiry, accept
                    # the turn rather than aborting a physically correct turn.
                    if final_error is not None and abs(final_error) <= timeout_accept:
                        print(
                            f"TURN OK AT TIMEOUT: yaw={final_yaw:+.1f} "
                            f"target={target_yaw:+.1f} error={final_error:+.1f} deg "
                            f"(accept<={timeout_accept:.1f})"
                        )
                        return True

                    print(
                        f"TURN ATTEMPT {attempt}/{max_attempts} WATCHDOG TIMEOUT: "
                        + (
                            f"yaw={final_yaw:+.1f} target={target_yaw:+.1f} "
                            f"error={final_error:+.1f} deg"
                            if final_yaw is not None
                            else "yaw unavailable"
                        )
                    )
                    break

                time.sleep(config.TURN_FEEDBACK_LOOP_SEC)

        _safe_stop(chassis)
        final_yaw = pose_tracker.get_yaw()
        final_error = (
            shortest_angle_error_deg(target_yaw, final_yaw)
            if final_yaw is not None
            else None
        )
        print(
            "TURN FAILED AFTER RETRIES: "
            + (
                f"yaw={final_yaw:+.1f} target={target_yaw:+.1f} "
                f"error={final_error:+.1f} deg"
                if final_yaw is not None
                else "yaw unavailable"
            )
        )
        return False

    except KeyboardInterrupt:
        _safe_stop(chassis)
        raise
    except Exception as exc:
        _safe_stop(chassis)
        print(f"TURN FEEDBACK ERROR: {exc}")
        return False

def _action_turn_with_timeout(chassis, decision, pose_tracker=None):
    """Finite-time fallback only when attitude feedback is unavailable."""
    command_deg = decision.angle_deg * config.Z_DIR_SIGN
    if abs(command_deg) < 0.001:
        return True

    start_yaw = pose_tracker.get_yaw() if pose_tracker is not None else None
    print(
        f">>> TURN {decision.name} [ACTION FALLBACK]: command={command_deg:+.1f} deg"
        + (f" start_yaw={start_yaw:+.1f}" if start_yaw is not None else "")
    )

    _safe_stop(chassis)
    time.sleep(config.TURN_PRE_SETTLE_SEC)

    timeout_sec = (
        config.TURN_ACTION_TIMEOUT_180_SEC
        if abs(command_deg) > 135.0
        else config.TURN_ACTION_TIMEOUT_90_SEC
    )

    try:
        action = chassis.move(
            x=0,
            y=0,
            z=command_deg,
            z_speed=config.TURN_SPEED,
        )
        completed = action.wait_for_completed(timeout=timeout_sec)
    except KeyboardInterrupt:
        _safe_stop(chassis)
        raise
    except Exception as exc:
        _safe_stop(chassis)
        print(f"TURN ACTION ERROR: {exc}")
        return False

    if not completed:
        _safe_stop(chassis)
        print(f"TURN ACTION TIMEOUT after {timeout_sec:.1f}s - stopped safely.")
        return False

    return True


def execute_turn(chassis, decision, pose_tracker=None):
    """Execute a turn without any unbounded SDK wait.

    Feedback-mode retries are handled inside _feedback_turn() while preserving
    one absolute target yaw. Returns True on success, False after bounded retry.
    """
    if not config.ENABLE_MOTION:
        return True

    command_deg = decision.angle_deg * config.Z_DIR_SIGN
    if abs(command_deg) < 0.001:
        return True

    if (
        config.ENABLE_FEEDBACK_TURN
        and pose_tracker is not None
        and pose_tracker.get_yaw() is not None
    ):
        result = _feedback_turn(chassis, decision, pose_tracker)
        if result is not None:
            return result

    # Never call wait_for_completed() without a timeout.
    return _action_turn_with_timeout(chassis, decision, pose_tracker)

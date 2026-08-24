"""Translate exploration decisions into RoboMaster turn commands."""

from dataclasses import dataclass
import math
import time

import config
from pose_tracker import normalize_angle_deg, shortest_angle_error_deg


@dataclass(frozen=True)
class TurnDecision:
    name: str
    angle_deg: float


RELATIVE_TO_TURN = {
    "FRONT": TurnDecision("FRONT", 0.0),
    # Backward-compatible alias in case an older caller still sends FORWARD.
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
        raise ValueError(
            f"Unknown relative direction: {relative_direction}"
        ) from exc


def print_exploration_decision(exploration_decision):
    print()
    print("========== TRÉMAUX / DFS DECISION ==========")
    print(f"NODE       : {exploration_decision.node_id}")
    print(f"DIRECTION  : {exploration_decision.direction}")
    print(f"ABS HEADING: {exploration_decision.absolute_heading}")
    print(f"MARK BEFORE: {exploration_decision.visits_before}")
    print(f"REASON     : {exploration_decision.reason}")
    print("============================================")


def _infer_move_to_yaw_sign(command_deg, actual_yaw_delta_deg):
    """Infer whether chassis.move(z) and attitude yaw use same/opposite signs."""
    if abs(command_deg) < 20.0 or abs(actual_yaw_delta_deg) < 20.0:
        return None

    command_sign = 1 if command_deg > 0 else -1
    yaw_sign = 1 if actual_yaw_delta_deg > 0 else -1
    return 1 if command_sign == yaw_sign else -1


def execute_turn(chassis, decision, pose_tracker=None):
    """Execute rotation and optionally verify/correct it using attitude yaw.

    The primary rotation always uses chassis.move(). After a normal 90-degree
    turn we learn the sign relation between move(z) and attitude yaw, so future
    90/180 turns can be checked without hard-coding the IMU sign convention.
    """
    if not config.ENABLE_MOTION:
        return

    command_deg = decision.angle_deg * config.Z_DIR_SIGN
    if abs(command_deg) < 0.001:
        return

    start_yaw = pose_tracker.get_yaw() if pose_tracker is not None else None

    print(
        f">>> TURN {decision.name}: command={command_deg:+.1f} deg"
        + (f" start_yaw={start_yaw:+.1f}" if start_yaw is not None else "")
    )

    chassis.move(
        x=0,
        y=0,
        z=command_deg,
        z_speed=config.TURN_SPEED,
    ).wait_for_completed()

    if not config.ENABLE_YAW_CORRECTION or pose_tracker is None:
        return

    time.sleep(config.YAW_SETTLE_SEC)
    end_yaw = pose_tracker.get_yaw()

    if start_yaw is None or end_yaw is None:
        print("Yaw verify skipped: attitude data unavailable.")
        return

    actual_delta = shortest_angle_error_deg(end_yaw, start_yaw)

    # For a 90-degree turn the sign is unambiguous, so learn mapping.
    if abs(command_deg) <= 135.0:
        learned_sign = _infer_move_to_yaw_sign(command_deg, actual_delta)
        if learned_sign is not None:
            pose_tracker.set_move_to_yaw_sign(learned_sign)

    sign_map = pose_tracker.get_move_to_yaw_sign()

    print(
        f"Yaw verify: start={start_yaw:+.1f} end={end_yaw:+.1f} "
        f"delta={actual_delta:+.1f} sign_map={sign_map}"
    )

    if sign_map is None:
        # Usually happens if the first turn is 180 degrees. Primary move() is
        # still valid; we simply avoid guessing the attitude sign for correction.
        print("Yaw correction skipped: sign mapping not learned yet.")
        return

    target_yaw = normalize_angle_deg(start_yaw + command_deg * sign_map)
    error_yaw = shortest_angle_error_deg(target_yaw, end_yaw)

    if abs(error_yaw) <= config.YAW_TOLERANCE_DEG:
        print(f"Yaw OK: error={error_yaw:+.1f} deg")
        return

    if abs(error_yaw) > config.YAW_MAX_CORRECTION_DEG:
        print(
            f"WARNING: yaw error {error_yaw:+.1f} deg exceeds correction limit; "
            "not applying automatic correction."
        )
        return

    correction_command = error_yaw / sign_map

    # Avoid tiny float noise and enforce the configured safety limit.
    correction_command = math.copysign(
        min(abs(correction_command), config.YAW_MAX_CORRECTION_DEG),
        correction_command,
    )

    print(
        f">>> YAW CORRECTION: target={target_yaw:+.1f} "
        f"error={error_yaw:+.1f} command={correction_command:+.1f}"
    )

    chassis.move(
        x=0,
        y=0,
        z=correction_command,
        z_speed=config.TURN_CORRECTION_SPEED,
    ).wait_for_completed()

    time.sleep(config.YAW_SETTLE_SEC)
    corrected_yaw = pose_tracker.get_yaw()
    if corrected_yaw is not None:
        final_error = shortest_angle_error_deg(target_yaw, corrected_yaw)
        print(
            f"Yaw after correction: {corrected_yaw:+.1f} "
            f"final_error={final_error:+.1f} deg"
        )

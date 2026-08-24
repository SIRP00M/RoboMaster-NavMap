"""Maze turn decision and turn execution."""

from dataclasses import dataclass

import config


@dataclass(frozen=True)
class TurnDecision:
    name: str
    angle_deg: float


def decide_turn(left_cm, right_cm):
    """
    เลือกทิศทางเมื่อเจอกำแพงด้านหน้า

    Priority:
    1. ซ้าย + ขวา ตัน -> กลับหลัง 180
    2. ซ้าย + ขวา เปิด -> เลี้ยวขวาก่อน (T-Junction)
    3. ซ้ายเปิดกว่าขวาชัดเจน -> ซ้าย
    4. ขวาเปิดกว่าซ้ายชัดเจน -> ขวา
    5. Fallback -> เลือกด้านที่กว้างกว่า
    """

    left_open = left_cm >= config.SIDE_DEAD_END_CM
    right_open = right_cm >= config.SIDE_DEAD_END_CM

    # ========================================================
    # CASE 1: DEAD END
    # ซ้ายตัน + ขวาตัน
    # ========================================================
    if not left_open and not right_open:
        return TurnDecision(
            "DEAD_END_180",
            config.TURN_AROUND_DEG
        )

    # ========================================================
    # CASE 2: T-JUNCTION / 3-WAY
    # ซ้ายเปิด + ขวาเปิด
    #
    # RIGHT PRIORITY:
    # ถ้าเลือกได้ทั้งสองทาง ให้ไปขวาก่อน
    # ========================================================
    if left_open and right_open:
        return TurnDecision(
            "T_JUNCTION_RIGHT",
            config.TURN_RIGHT_DEG
        )

    # ========================================================
    # CASE 3: LEFT clearly more open
    # ========================================================
    if (left_cm - right_cm) > config.SIDE_OPEN_DIFFERENCE_CM:
        return TurnDecision(
            "LEFT_90",
            config.TURN_LEFT_DEG
        )

    # ========================================================
    # CASE 4: RIGHT clearly more open
    # ========================================================
    if (right_cm - left_cm) > config.SIDE_OPEN_DIFFERENCE_CM:
        return TurnDecision(
            "RIGHT_90",
            config.TURN_RIGHT_DEG
        )

    # ========================================================
    # CASE 5: FALLBACK
    # ========================================================

    # มีแค่ซ้ายเปิด
    if left_open:
        return TurnDecision(
            "FALLBACK_LEFT",
            config.TURN_LEFT_DEG
        )

    # มีแค่ขวาเปิด
    if right_open:
        return TurnDecision(
            "FALLBACK_RIGHT",
            config.TURN_RIGHT_DEG
        )

    # Safety fallback
    if left_cm >= right_cm:
        return TurnDecision(
            "FALLBACK_LEFT",
            config.TURN_LEFT_DEG
        )

    return TurnDecision(
        "FALLBACK_RIGHT",
        config.TURN_RIGHT_DEG
    )


def print_turn_decision(decision):
    labels = {
        "DEAD_END_180": (
            "--> DEAD END",
            "--> TURN 180",
        ),

        "T_JUNCTION_RIGHT": (
            "--> T-JUNCTION",
            "--> LEFT OPEN + RIGHT OPEN",
            "--> RIGHT PRIORITY",
            "--> TURN RIGHT 90",
        ),

        "LEFT_90": (
            "--> LEFT OPEN",
            "--> TURN LEFT 90",
        ),

        "RIGHT_90": (
            "--> RIGHT OPEN",
            "--> TURN RIGHT 90",
        ),

        "FALLBACK_LEFT": (
            "--> FALLBACK LEFT",
        ),

        "FALLBACK_RIGHT": (
            "--> FALLBACK RIGHT",
        ),
    }

    for line in labels.get(
        decision.name,
        (f"--> {decision.name}",)
    ):
        print(line)


def execute_turn(chassis, decision):
    if not config.ENABLE_MOTION:
        return

    chassis.move(
        x=0,
        y=0,
        z=decision.angle_deg * config.Z_DIR_SIGN,
        z_speed=config.TURN_SPEED,
    ).wait_for_completed()
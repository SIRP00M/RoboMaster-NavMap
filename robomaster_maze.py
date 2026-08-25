"""
RoboMaster Wall-Maze Solver - CLEAN GRAPH DFS / TREMaux
========================================================
Architecture
    Wall sensing
      - Front ToF
      - Left/Right Sharp IR distance
      - Optional digital IR sensors on front-left/front-right chassis corners

    Localization
      - RoboMaster odometry (sub_position cs=1)
      - Discrete N/E/S/W heading from attitude yaw
      - Small metric grid quantization for junction memory

    Exploration
      - Persistent topological junction graph
      - Unvisited exits are frontiers
      - Trémaux-style edge traversal marks
      - Local DFS preference; BFS is used only to transit through already-known
        corridors to the nearest node that still owns an unexplored frontier
      - Loop closure uses position + expected graph target + incoming heading

    Junction handling
      - Side opening is tracked as a physical opening window
      - Robot passes the opening, then reverses to the estimated centre
      - Front blocked corners/dead-ends creep to a repeatable front distance
      - Junction node is registered ONLY after centering

    Exit handling
      - Optional sustained open-space detector. It requires FRONT + LEFT + RIGHT
        to remain open for a configurable travelled distance, so a normal short
        intersection does not immediately count as the maze exit.

IMPORTANT HARDWARE NOTE
    The old code only provided one digital IR adapter ID (1). This clean version
    therefore enables the known front-left IR and leaves IR_FRONT_RIGHT_ID=None.
    If you really have a second digital IR on the right front edge, set its
    adapter ID below. Also verify IR_BLOCKED_LEVEL on your hardware.

Requires RoboMaster Python SDK.
"""

from __future__ import annotations

import json
import math
import statistics
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

try:
    from robomaster import robot
except ModuleNotFoundError as exc:
    raise SystemExit(
        "RoboMaster SDK is not installed for this Python. Install it first."
    ) from exc


# ============================================================
# CONFIG - mostly preserved from the supplied working values
# ============================================================

ENABLE_MOTION = True
PROGRAM_VERSION = "CLEAN_GRAPH_DFS_V3_CONFIRMED_EDGE_SAFE"

# ---------------- Sensor IDs ----------------
IR_FRONT_LEFT_ID = 1
IR_FRONT_RIGHT_ID = 4

IR_FRONT_LEFT_PORT = 1
IR_FRONT_RIGHT_PORT = 1

SHARP_LEFT_ID = 2
SHARP_RIGHT_ID = 3
SENSOR_PORT = 1

# Digital IR polarity
# 0 = obstacle detected
# 1 = clear
IR_BLOCKED_LEVEL = 0
IR_CONFIRM_SAMPLES = 2

SHARP_FILTER_SIZE = 3
TOF_FILTER_SIZE = 3
SHARP_EMA_NEW_WEIGHT = 0.6
SHARP_EMA_OLD_WEIGHT = 0.4
TOF_STALE_SEC = 0.40

# ---------------- Distances ----------------
TARGET_LEFT_CM = 5.0
TARGET_RIGHT_CM = 5.0

SLOW_FRONT_CM = 18.0
STOP_FRONT_CM = 15.0
SIDE_TOO_CLOSE_CM = 4

SIDE_WALL_ENTER_CM = 28.0
SIDE_WALL_EXIT_CM = 32.0

SIDE_OPEN_ENTER_CM = 18.0
SIDE_OPEN_EXIT_CM = 15.0
EXPLORATION_FRONT_OPEN_CM = 45.0

# ---------------- Speed ----------------
FORWARD_SPEED = 0.25
MIN_FORWARD_SPEED = 0.05
UNKNOWN_FRONT_SPEED = 0.0
ESCAPE_FORWARD_SPEED = 0.00
ESCAPE_Y_SPEED = 0.08

# Proactive side safety: start moving away BEFORE reaching SIDE_TOO_CLOSE_CM.
SIDE_WARNING_CM = 7.0
SIDE_WARNING_FORWARD_SPEED = 0.08
SIDE_WARNING_Y_SPEED = 0.05

# Front-corner IR has priority over normal wall following.
IR_ESCAPE_Y_SPEED = 0.08
IR_ESCAPE_FORWARD_SPEED = 0.00

Y_DIR_SIGN = 1

# ---------------- Wall controller ----------------
SIDE_KP_STRAFE = 0.012
SIDE_MAX_Y = 0.05
SIDE_DEADBAND_CM = 1.0

CENTER_TRIGGER_CM = 2.0
CENTER_RELEASE_CM = 0.7
CENTER_HOLD_SEC = 0.30
CENTER_KP_STRAFE = 0.009
CENTER_MAX_Y = 0.04

# ---------------- Heading / turn ----------------
DEFAULT_MOVE_TO_YAW_SIGN = -1
DEFAULT_DRIVE_TO_YAW_SIGN = +1

ENABLE_HEADING_HOLD = True
HEADING_KP_Z = 1.5
HEADING_MAX_Z_SPEED = 12.0
HEADING_DEADBAND_DEG = 0.8
HEADING_RECOVER_TRIGGER_DEG = 8.0
HEADING_RECOVER_RELEASE_DEG = 2.0
HEADING_RECOVER_MAX_Z_SPEED = 18.0

TURN_FEEDBACK_KP = 1.20
TURN_FEEDBACK_MIN_Z_SPEED = 10.0
TURN_FEEDBACK_MAX_Z_SPEED = 55.0
TURN_FEEDBACK_TOLERANCE_DEG = 2.0
TURN_FEEDBACK_STABLE_SAMPLES = 3
TURN_FEEDBACK_LOOP_SEC = 0.03
TURN_FEEDBACK_DRIVE_TIMEOUT_SEC = 0.20
TURN_FEEDBACK_TIMEOUT_90_SEC = 3.50
TURN_FEEDBACK_TIMEOUT_180_SEC = 6.00
TURN_PRE_SETTLE_SEC = 0.10
YAW_SETTLE_SEC = 0.08

ATTITUDE_FREQ_HZ = 20
POSE_FREQ_HZ = 20
POSE_WAIT_SEC = 1.0

HEADING_ALIGN_TOLERANCE_DEG = 2.0
HEADING_ALIGN_TIMEOUT_SEC = 1.20
HEADING_ALIGN_LOOP_SEC = 0.04

# ---------------- Junction detector ----------------
OPENING_ENTER_SAMPLES = 3
OPENING_EXIT_SAMPLES = 3
OPENING_MIN_LENGTH_M = 0.10
OPENING_MAX_LENGTH_M = 0.70
OPENING_LOOKAHEAD_M = 0.12
OPENING_MIN_EVIDENCE_SAMPLES = 2

OPENING_CENTER_SPEED = 0.07
OPENING_CENTER_MAX_BACKTRACK_M = 0.40
OPENING_CENTER_TIMEOUT_SEC = 6.5
OPENING_CENTER_LOOP_SEC = 0.04

# For front-blocked corner/dead-end: creep to a repeatable turn centre.
CORNER_CENTER_SPEED = 0.05
CORNER_CENTER_FRONT_TARGET_CM = 11.0
CORNER_CENTER_FRONT_HARD_STOP_CM = 10.5
CORNER_CENTER_MAX_DISTANCE_M = 0.14
CORNER_CENTER_TIMEOUT_SEC = 3.0
CORNER_CENTER_LOOP_SEC = 0.04

DECISION_SCAN_SAMPLES = 5
DECISION_SCAN_INTERVAL_SEC = 0.04
JUNCTION_SETTLE_SEC = 0.15
JUNCTION_CONFIRM_SAMPLES = 3

# Lock the same physical junction after a decision until the robot leaves it.
JUNCTION_REARM_MIN_DISTANCE_M = 0.14
JUNCTION_REARM_DISTANCE_M = 0.25
JUNCTION_REARM_TIMEOUT_SEC = 2.50
JUNCTION_REARM_SAMPLES = 4
JUNCTION_REARM_EMERGENCY_SEC = 0.25

# ---------------- Junction memory / loop closure ----------------
NODE_MATCH_RADIUS_M = 0.18
EXPECTED_TARGET_MATCH_RADIUS_M = 0.26

# Extended loop-closure radius is only used when incoming topology agrees.
# It is deliberately NOT used as a blind distance-only merge radius.
LOOP_MATCH_RADIUS_M = 0.30

# If a new junction event fires only a few centimetres after leaving a node,
# it is usually the SAME wide intersection being detected again rather than a
# real neighbouring junction.  The old V1 deliberately excluded pending_from
# from matching, which created fake nodes such as J10 in the field log.
SAME_NODE_RETRIGGER_RADIUS_M = 0.32
# A departure that re-triggers the same node after only a short distance and
# has a hard front block is a false/blocked exit, not a completed traversal.
FAILED_EDGE_MAX_PROGRESS_M = 0.30
NODE_POSITION_UPDATE_ALPHA = 0.15

# Quantization resolution for the local metric grid (not maze-cell size).
GRID_RESOLUTION_M = 0.05

EXPLORATION_PREFERENCE = ("FRONT", "LEFT", "RIGHT", "BACK")

# ---------------- Post-turn corner clearance ----------------
ENABLE_POST_TURN_CLEARANCE = True
POST_TURN_TRIGGER_CM = 7.0
POST_TURN_RELEASE_CM = 8.5
POST_TURN_FORWARD_SPEED = 0.04
POST_TURN_Y_SPEED = 0.045
POST_TURN_MAX_DISTANCE_M = 0.10
POST_TURN_MAX_SEC = 1.50
POST_TURN_FRONT_STOP_CM = 12.0
POST_TURN_LOOP_SEC = 0.04

# ---------------- Exit detector ----------------
# Outside the maze should remain open longer than an ordinary intersection.
ENABLE_EXIT_DETECTION = True
EXIT_FRONT_OPEN_CM = 50.0
EXIT_SIDE_OPEN_CM = 35.0
EXIT_CONFIRM_DISTANCE_M = 0.60
EXIT_CONFIRM_MIN_SEC = 1.5
EXIT_RESET_SAMPLES = 3

# ---------------- Main loop / logging ----------------
LOOP_DELAY_SEC = 0.05
DRIVE_TIMEOUT_SEC = 0.15
AFTER_TURN_DELAY_SEC = 0.12
SAVE_MAZE_MEMORY = True
MAZE_MEMORY_FILE = "maze_memory_clean.json"
PRINT_EVERY_SEC = 0.20

# ---------------- Sharp calibration: ADC -> cm ----------------
CALIBRATION_SHARP_LEFT = [
    (675, 5.0),
    (343, 10.0),
    (236, 15.0),
    (166, 20.0),
    (126, 25.0),
    (105, 30.0),
    (50, 80.0),
]
CALIBRATION_SHARP_RIGHT = list(CALIBRATION_SHARP_LEFT)


# ============================================================
# Utility
# ============================================================

HEADINGS = ("N", "E", "S", "W")
RELATIVE_OFFSET = {"FRONT": 0, "RIGHT": 1, "BACK": 2, "LEFT": -1}
RELATIVE_TURN_DEG = {"FRONT": 0.0, "LEFT": 90.0, "RIGHT": -90.0, "BACK": -180.0}


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def normalize_angle_deg(angle: float) -> float:
    return (float(angle) + 180.0) % 360.0 - 180.0


def shortest_angle_error_deg(target: float, current: float) -> float:
    return normalize_angle_deg(float(target) - float(current))


def distance_xy(a: Tuple[Optional[float], Optional[float]],
                b: Tuple[Optional[float], Optional[float]]) -> Optional[float]:
    if None in (a[0], a[1], b[0], b[1]):
        return None
    return math.hypot(float(b[0]) - float(a[0]), float(b[1]) - float(a[1]))


def median_or_none(values: List[float]) -> Optional[float]:
    return statistics.median(values) if values else None


def fmt(v: Optional[float]) -> str:
    return "---" if v is None else f"{v:5.1f}"


# ============================================================
# Pose tracker
# ============================================================

class PoseTracker:
    def __init__(self):
        self._lock = threading.Lock()
        self.x: Optional[float] = None
        self.y: Optional[float] = None
        self.z: Optional[float] = None
        self.yaw: Optional[float] = None
        self.pitch: Optional[float] = None
        self.roll: Optional[float] = None

    def position_callback(self, data):
        try:
            if data is None or len(data) < 3:
                return
            x, y, z = data[:3]
            with self._lock:
                self.x = float(x)
                self.y = float(y)
                self.z = float(z)
        except Exception as exc:
            print("Position callback error:", exc)

    def attitude_callback(self, data):
        try:
            if data is None or len(data) < 3:
                return
            yaw, pitch, roll = data[:3]
            with self._lock:
                self.yaw = normalize_angle_deg(yaw)
                self.pitch = float(pitch)
                self.roll = float(roll)
        except Exception as exc:
            print("Attitude callback error:", exc)

    def get_xy(self) -> Tuple[Optional[float], Optional[float]]:
        with self._lock:
            return self.x, self.y

    def get_yaw(self) -> Optional[float]:
        with self._lock:
            return self.yaw

    def has_position(self) -> bool:
        x, y = self.get_xy()
        return x is not None and y is not None


# ============================================================
# Sensor manager
# ============================================================

class SensorManager:
    def __init__(self, sensor_adapter):
        self.sensor_adapter = sensor_adapter

        self.left_adc_buf = deque(maxlen=SHARP_FILTER_SIZE)
        self.right_adc_buf = deque(maxlen=SHARP_FILTER_SIZE)
        self.tof_buf = deque(maxlen=TOF_FILTER_SIZE)

        self.left_ema: Optional[float] = None
        self.right_ema: Optional[float] = None
        self.front_cm: Optional[float] = None
        self.tof_last_update: Optional[float] = None

        self.ir_left_hist = deque(maxlen=IR_CONFIRM_SAMPLES)
        self.ir_right_hist = deque(maxlen=IR_CONFIRM_SAMPLES)

    def tof_callback(self, data):
        try:
            if not data or data[0] is None:
                return
            mm = float(data[0])
            if mm < 20.0 or mm > 4000.0:
                return
            cm = mm / 10.0
            self.tof_buf.append(cm)
            self.front_cm = statistics.median(self.tof_buf)
            self.tof_last_update = time.monotonic()
        except Exception as exc:
            print("ToF callback error:", exc)

    def get_front_cm(self) -> Optional[float]:
        if self.front_cm is None or self.tof_last_update is None:
            return None
        if time.monotonic() - self.tof_last_update > TOF_STALE_SEC:
            return None
        return self.front_cm

    @staticmethod
    def adc_to_cm(adc: float, table) -> float:
        if adc >= table[0][0]:
            return float(table[0][1])
        if adc <= table[-1][0]:
            return float(table[-1][1])

        for i in range(len(table) - 1):
            adc1, cm1 = table[i]
            adc2, cm2 = table[i + 1]
            if adc1 >= adc >= adc2:
                ratio = (adc1 - adc) / (adc1 - adc2)
                return cm1 + ratio * (cm2 - cm1)
        return float(table[-1][1])

    def _read_sharp(self, sensor_id: int, left: bool):
        try:
            raw = self.sensor_adapter.get_adc(id=sensor_id, port=SENSOR_PORT)
            raw = float(raw)
        except Exception as exc:
            print(f"Sharp {sensor_id} read error: {exc}")
            return 0, None

        buf = self.left_adc_buf if left else self.right_adc_buf
        buf.append(raw)
        med = statistics.median(buf)

        if left:
            self.left_ema = med if self.left_ema is None else (
                SHARP_EMA_NEW_WEIGHT * med + SHARP_EMA_OLD_WEIGHT * self.left_ema
            )
            ema = self.left_ema
            table = CALIBRATION_SHARP_LEFT
        else:
            self.right_ema = med if self.right_ema is None else (
                SHARP_EMA_NEW_WEIGHT * med + SHARP_EMA_OLD_WEIGHT * self.right_ema
            )
            ema = self.right_ema
            table = CALIBRATION_SHARP_RIGHT

        return int(round(raw)), self.adc_to_cm(ema, table)

    def read_left(self):
        return self._read_sharp(SHARP_LEFT_ID, left=True)

    def read_right(self):
        return self._read_sharp(SHARP_RIGHT_ID, left=False)

    def _read_digital_ir(self, sensor_id: Optional[int], history: deque) -> Optional[bool]:
        if sensor_id is None:
            return None

        level = None

        try:
            if sensor_id == IR_FRONT_LEFT_ID:
                level = self.sensor_adapter.get_io(
                    id=sensor_id,
                    port=IR_FRONT_LEFT_PORT,
                )
            elif sensor_id == IR_FRONT_RIGHT_ID:
                level = self.sensor_adapter.get_io(
                    id=sensor_id,
                    port=IR_FRONT_RIGHT_PORT,
                )
        except Exception as exc:
            print(f"Digital IR {sensor_id} read error: {exc}")
            return None

        if level is None:
            return None

        # Hardware convention:
        # raw 0 -> obstacle
        # raw 1 -> clear
        blocked = int(level) == int(IR_BLOCKED_LEVEL)
        history.append(bool(blocked))
        if len(history) < history.maxlen:
            return None
        return all(history)

    def read_front_corner_ir(self) -> Tuple[Optional[bool], Optional[bool]]:
        left = self._read_digital_ir(IR_FRONT_LEFT_ID, self.ir_left_hist)
        right = self._read_digital_ir(IR_FRONT_RIGHT_ID, self.ir_right_hist)
        return left, right

    def reset_filters(self):
        self.left_adc_buf.clear()
        self.right_adc_buf.clear()
        self.tof_buf.clear()
        self.ir_left_hist.clear()
        self.ir_right_hist.clear()
        self.left_ema = None
        self.right_ema = None
        self.front_cm = None
        self.tof_last_update = None


# ============================================================
# Heading manager and motion control
# ============================================================

class HeadingManager:
    """Maps logical N/E/S/W to absolute RoboMaster attitude yaw."""

    def __init__(self):
        self.base_yaw: Optional[float] = None
        self.target_yaw: Optional[float] = None
        self.heading_index = 0
        # With supplied robot logs: logical RIGHT causes +90 attitude yaw.
        right_command_sign = 1 if (-90.0) > 0 else -1
        self.right_yaw_step_sign = right_command_sign * DEFAULT_MOVE_TO_YAW_SIGN
        self.recovering = False

    def initialize(self, yaw: Optional[float]):
        if yaw is None:
            return False
        self.base_yaw = normalize_angle_deg(yaw)
        self.target_yaw = self.base_yaw
        self.heading_index = 0
        return True

    def set_heading_index(self, index: int):
        self.heading_index = index % 4
        if self.base_yaw is None:
            return
        step = {0: 0, 1: 1, 2: 2, 3: -1}[self.heading_index]
        self.target_yaw = normalize_angle_deg(
            self.base_yaw + self.right_yaw_step_sign * 90.0 * step
        )

    def error(self, current_yaw: Optional[float]) -> Optional[float]:
        if self.target_yaw is None or current_yaw is None:
            return None
        return shortest_angle_error_deg(self.target_yaw, current_yaw)

    def correction_z(self, current_yaw: Optional[float], recover=False) -> Tuple[float, Optional[float]]:
        if not ENABLE_HEADING_HOLD:
            return 0.0, None
        error = self.error(current_yaw)
        if error is None:
            return 0.0, None
        if abs(error) <= HEADING_DEADBAND_DEG:
            return 0.0, error

        max_z = HEADING_RECOVER_MAX_Z_SPEED if recover else HEADING_MAX_Z_SPEED
        desired_yaw_rate = clamp(error * HEADING_KP_Z, -max_z, max_z)
        z_cmd = desired_yaw_rate / DEFAULT_DRIVE_TO_YAW_SIGN
        return z_cmd, error

    def apply(self, x: float, y: float, current_yaw: Optional[float], mode: str):
        error = self.error(current_yaw)
        if error is None:
            return x, y, 0.0, mode, None

        if self.recovering:
            if abs(error) <= HEADING_RECOVER_RELEASE_DEG:
                self.recovering = False
        elif abs(error) >= HEADING_RECOVER_TRIGGER_DEG:
            self.recovering = True

        if self.recovering:
            z, error = self.correction_z(current_yaw, recover=True)
            return 0.0, 0.0, z, "HEADING_RECOVER", error

        z, error = self.correction_z(current_yaw, recover=False)
        return x, y, z, mode, error


class WallController:
    def __init__(self):
        self.left_wall = False
        self.right_wall = False
        self.center_owner = "NONE"
        self.center_owner_since = 0.0

    @staticmethod
    def forward_speed(front_cm: Optional[float]) -> float:
        if front_cm is None:
            return UNKNOWN_FRONT_SPEED
        if front_cm >= SLOW_FRONT_CM:
            return FORWARD_SPEED
        if front_cm <= STOP_FRONT_CM:
            return 0.0
        ratio = (front_cm - STOP_FRONT_CM) / (SLOW_FRONT_CM - STOP_FRONT_CM)
        return MIN_FORWARD_SPEED + ratio * (FORWARD_SPEED - MIN_FORWARD_SPEED)

    @staticmethod
    def _wall_state(distance_cm: Optional[float], current: bool) -> bool:
        if distance_cm is None:
            return False
        # A physically open side is never simultaneously treated as a wall.
        if distance_cm >= SIDE_OPEN_ENTER_CM:
            return False
        if current:
            return distance_cm < SIDE_WALL_EXIT_CM
        return distance_cm < SIDE_WALL_ENTER_CM

    def reset(self):
        self.left_wall = False
        self.right_wall = False
        self.center_owner = "NONE"
        self.center_owner_since = 0.0

    def _center_between_walls(self, left_cm: float, right_cm: float):
        now = time.monotonic()
        delta = left_cm - right_cm
        abs_delta = abs(delta)

        if self.center_owner == "NONE":
            if abs_delta < CENTER_TRIGGER_CM:
                return 0.0, "CENTER_STABLE"
            self.center_owner = "LEFT" if delta < 0 else "RIGHT"
            self.center_owner_since = now

        if now - self.center_owner_since >= CENTER_HOLD_SEC:
            if abs_delta <= CENTER_RELEASE_CM:
                self.center_owner = "NONE"
                return 0.0, "CENTER_RELEASE"
            if self.center_owner == "LEFT" and delta >= CENTER_TRIGGER_CM:
                self.center_owner = "RIGHT"
                self.center_owner_since = now
            elif self.center_owner == "RIGHT" and delta <= -CENTER_TRIGGER_CM:
                self.center_owner = "LEFT"
                self.center_owner_since = now

        correction = clamp(abs_delta * CENTER_KP_STRAFE, 0.0, CENTER_MAX_Y)
        if self.center_owner == "LEFT":
            return +correction * Y_DIR_SIGN, "CENTER_LEFT_OWNER"
        return -correction * Y_DIR_SIGN, "CENTER_RIGHT_OWNER"

    def lateral(self, left_cm: Optional[float], right_cm: Optional[float]):
        if left_cm is None or right_cm is None:
            self.reset()
            return 0.0, "NO_SIDE_SENSOR"

        if left_cm <= SIDE_TOO_CLOSE_CM and right_cm <= SIDE_TOO_CLOSE_CM:
            self.center_owner = "NONE"
            return 0.0, "BOTH_TOO_CLOSE"
        if left_cm <= SIDE_TOO_CLOSE_CM:
            self.center_owner = "NONE"
            return +ESCAPE_Y_SPEED * Y_DIR_SIGN, "ESCAPE_LEFT"
        if right_cm <= SIDE_TOO_CLOSE_CM:
            self.center_owner = "NONE"
            return -ESCAPE_Y_SPEED * Y_DIR_SIGN, "ESCAPE_RIGHT"

        # Start avoidance earlier. The V2 field log repeatedly reached the
        # Sharp calibration floor (~5 cm) before ESCAPE engaged, which is late
        # enough for a front chassis corner to scrape the wall.
        if left_cm < SIDE_WARNING_CM and right_cm >= SIDE_WARNING_CM:
            self.center_owner = "NONE"
            return +SIDE_WARNING_Y_SPEED * Y_DIR_SIGN, "AVOID_LEFT"
        if right_cm < SIDE_WARNING_CM and left_cm >= SIDE_WARNING_CM:
            self.center_owner = "NONE"
            return -SIDE_WARNING_Y_SPEED * Y_DIR_SIGN, "AVOID_RIGHT"

        self.left_wall = self._wall_state(left_cm, self.left_wall)
        self.right_wall = self._wall_state(right_cm, self.right_wall)

        if self.left_wall and self.right_wall:
            return self._center_between_walls(left_cm, right_cm)

        self.center_owner = "NONE"

        if self.left_wall:
            error = left_cm - TARGET_LEFT_CM
            if abs(error) <= SIDE_DEADBAND_CM:
                return 0.0, "FOLLOW_LEFT"
            y = clamp(-error * SIDE_KP_STRAFE * Y_DIR_SIGN, -SIDE_MAX_Y, SIDE_MAX_Y)
            return y, "FOLLOW_LEFT"

        if self.right_wall:
            error = right_cm - TARGET_RIGHT_CM
            if abs(error) <= SIDE_DEADBAND_CM:
                return 0.0, "FOLLOW_RIGHT"
            y = clamp(error * SIDE_KP_STRAFE * Y_DIR_SIGN, -SIDE_MAX_Y, SIDE_MAX_Y)
            return y, "FOLLOW_RIGHT"

        return 0.0, "OPEN_SPACE"


def apply_front_corner_ir_guard(
    x: float,
    y: float,
    ir_left_blocked: Optional[bool],
    ir_right_blocked: Optional[bool],
) -> Tuple[float, float, str]:
    """Use the front-edge digital IRs only as collision guards, not topology."""
    left = ir_left_blocked is True
    right = ir_right_blocked is True

    if left and right:
        return 0.0, 0.0, "IR_BOTH_FRONT_STOP"
    if left:
        return min(x, IR_ESCAPE_FORWARD_SPEED), +IR_ESCAPE_Y_SPEED * Y_DIR_SIGN, "IR_LEFT_ESCAPE"
    if right:
        return min(x, IR_ESCAPE_FORWARD_SPEED), -IR_ESCAPE_Y_SPEED * Y_DIR_SIGN, "IR_RIGHT_ESCAPE"
    return x, y, ""


# ============================================================
# Junction detector
# ============================================================

@dataclass
class JunctionEvent:
    kind: str  # SIDE_WINDOW or FRONT_BLOCKED
    observed_front: bool = False
    observed_left: bool = False
    observed_right: bool = False
    start_xy: Optional[Tuple[float, float]] = None
    last_open_xy: Optional[Tuple[float, float]] = None
    end_xy: Optional[Tuple[float, float]] = None
    backtrack_m: float = 0.0


class JunctionDetector:
    def __init__(self):
        self.locked = False
        self.lock_xy: Optional[Tuple[float, float]] = None
        self.lock_time: Optional[float] = None
        self.clear_samples = 0

        self.front_block_count = 0
        self.left_enter_count = 0
        self.right_enter_count = 0
        self.side_close_count = 0

        self.window_active = False
        self.window_start_xy: Optional[Tuple[float, float]] = None
        self.last_open_xy: Optional[Tuple[float, float]] = None
        self.lookahead_start_xy: Optional[Tuple[float, float]] = None
        self.front_evidence = 0
        self.left_evidence = 0
        self.right_evidence = 0

    def reset_window(self):
        self.front_block_count = 0
        self.left_enter_count = 0
        self.right_enter_count = 0
        self.side_close_count = 0
        self.window_active = False
        self.window_start_xy = None
        self.last_open_xy = None
        self.lookahead_start_xy = None
        self.front_evidence = 0
        self.left_evidence = 0
        self.right_evidence = 0

    @staticmethod
    def side_still_open(cm: Optional[float]) -> bool:
        return cm is not None and cm >= SIDE_OPEN_EXIT_CM

    @staticmethod
    def side_enter_open(cm: Optional[float]) -> bool:
        return cm is not None and cm >= SIDE_OPEN_ENTER_CM

    @staticmethod
    def front_is_open(cm: Optional[float]) -> bool:
        return cm is not None and cm >= EXPLORATION_FRONT_OPEN_CM

    @staticmethod
    def front_is_blocked(cm: Optional[float]) -> bool:
        return cm is not None and 0.0 < cm <= STOP_FRONT_CM

    def lock_here(self, pose_xy: Tuple[Optional[float], Optional[float]]):
        self.locked = True
        if None not in pose_xy:
            self.lock_xy = (float(pose_xy[0]), float(pose_xy[1]))
        else:
            self.lock_xy = None
        self.lock_time = time.monotonic()
        self.clear_samples = 0
        self.reset_window()

    def _maybe_unlock(self, pose_xy, front_cm, left_cm, right_cm) -> bool:
        if not self.locked:
            return True

        now = time.monotonic()
        elapsed = now - self.lock_time if self.lock_time is not None else 0.0
        moved = distance_xy(self.lock_xy or (None, None), pose_xy)

        side_clear = not self.side_still_open(left_cm) and not self.side_still_open(right_cm)
        normal_corridor = side_clear and not self.front_is_blocked(front_cm)
        self.clear_samples = self.clear_samples + 1 if normal_corridor else 0

        by_corridor = (
            self.clear_samples >= JUNCTION_REARM_SAMPLES
            and (moved is None or moved >= JUNCTION_REARM_MIN_DISTANCE_M)
        )
        by_distance = (
            moved is not None and moved >= JUNCTION_REARM_DISTANCE_M and side_clear
        )
        by_timeout = (
            elapsed >= JUNCTION_REARM_TIMEOUT_SEC
            and moved is not None
            and moved >= JUNCTION_REARM_MIN_DISTANCE_M
            and side_clear
        )
        by_emergency_front = (
            self.front_is_blocked(front_cm)
            and elapsed >= JUNCTION_REARM_EMERGENCY_SEC
        )

        if by_corridor or by_distance or by_timeout or by_emergency_front:
            self.locked = False
            self.lock_xy = None
            self.lock_time = None
            self.clear_samples = 0
            self.reset_window()
            return True
        return False

    def _start_window(self, pose_xy):
        self.window_active = True
        self.window_start_xy = pose_xy if None not in pose_xy else None
        self.last_open_xy = self.window_start_xy
        self.lookahead_start_xy = None
        self.side_close_count = 0
        print(">>> JUNCTION WINDOW START")

    def update(self, front_cm, left_cm, right_cm, pose_xy) -> Optional[JunctionEvent]:
        if not self._maybe_unlock(pose_xy, front_cm, left_cm, right_cm):
            return None

        front_blocked = self.front_is_blocked(front_cm)
        self.front_block_count = self.front_block_count + 1 if front_blocked else 0

        left_enter = self.side_enter_open(left_cm)
        right_enter = self.side_enter_open(right_cm)
        self.left_enter_count = self.left_enter_count + 1 if left_enter else 0
        self.right_enter_count = self.right_enter_count + 1 if right_enter else 0

        if not self.window_active:
            if (
                self.left_enter_count >= OPENING_ENTER_SAMPLES
                or self.right_enter_count >= OPENING_ENTER_SAMPLES
            ):
                self._start_window(pose_xy)
                self.left_evidence = self.left_enter_count
                self.right_evidence = self.right_enter_count

            elif self.front_block_count >= JUNCTION_CONFIRM_SAMPLES:
                event = JunctionEvent(kind="FRONT_BLOCKED")
                self.lock_here(pose_xy)
                return event
            else:
                return None

        # Window is active.
        if self.front_is_open(front_cm):
            self.front_evidence += 1
        if left_enter:
            self.left_evidence += 1
        if right_enter:
            self.right_evidence += 1

        left_still = self.side_still_open(left_cm)
        right_still = self.side_still_open(right_cm)

        if left_still or right_still:
            if None not in pose_xy:
                self.last_open_xy = (float(pose_xy[0]), float(pose_xy[1]))
            self.lookahead_start_xy = None
            self.side_close_count = 0
        else:
            self.side_close_count += 1
            if self.side_close_count >= OPENING_EXIT_SAMPLES and self.lookahead_start_xy is None:
                self.lookahead_start_xy = pose_xy if None not in pose_xy else None

        # A real front stop while inside the opening window is a valid decision.
        if self.front_block_count >= JUNCTION_CONFIRM_SAMPLES:
            return self._finalize_window(pose_xy, "FRONT_BLOCKED_IN_WINDOW")

        # Finalize after a short look-ahead past the far edge.
        if self.lookahead_start_xy is not None:
            lookahead = distance_xy(self.lookahead_start_xy, pose_xy)
            if lookahead is not None and lookahead >= OPENING_LOOKAHEAD_M:
                return self._finalize_window(pose_xy, "LOOKAHEAD_COMPLETE")

        # Hard maximum prevents a never-ending opening window.
        total = distance_xy(self.window_start_xy or (None, None), pose_xy)
        if total is not None and total >= OPENING_MAX_LENGTH_M:
            return self._finalize_window(pose_xy, "MAX_WINDOW")

        return None

    def _finalize_window(self, pose_xy, reason: str) -> JunctionEvent:
        total = distance_xy(self.window_start_xy or (None, None), pose_xy) or 0.0
        span = distance_xy(
            self.window_start_xy or (None, None),
            self.last_open_xy or pose_xy,
        ) or total

        # Current position is after the opening. Reverse to its estimated centre.
        backtrack = max(0.0, total - 0.5 * span)
        backtrack = min(backtrack, OPENING_CENTER_MAX_BACKTRACK_M)

        event = JunctionEvent(
            kind="SIDE_WINDOW",
            observed_front=self.front_evidence >= OPENING_MIN_EVIDENCE_SAMPLES,
            observed_left=self.left_evidence >= OPENING_MIN_EVIDENCE_SAMPLES,
            observed_right=self.right_evidence >= OPENING_MIN_EVIDENCE_SAMPLES,
            start_xy=self.window_start_xy,
            last_open_xy=self.last_open_xy,
            end_xy=pose_xy if None not in pose_xy else None,
            backtrack_m=backtrack,
        )

        print(
            f">>> JUNCTION WINDOW END reason={reason} total={total:.3f}m "
            f"span={span:.3f}m backtrack={backtrack:.3f}m "
            f"evidence F/L/R={self.front_evidence}/{self.left_evidence}/{self.right_evidence}"
        )
        self.lock_here(pose_xy)
        return event


# ============================================================
# Open-space exit detector
# ============================================================

class ExitDetector:
    def __init__(self):
        self.active = False
        self.start_xy: Optional[Tuple[float, float]] = None
        self.start_time: Optional[float] = None
        self.reset_count = 0

    def reset(self):
        self.active = False
        self.start_xy = None
        self.start_time = None
        self.reset_count = 0

    def update(
        self,
        front_cm: Optional[float],
        left_cm: Optional[float],
        right_cm: Optional[float],
        pose_xy,
        ir_left_blocked: Optional[bool],
        ir_right_blocked: Optional[bool],
    ) -> bool:
        if not ENABLE_EXIT_DETECTION:
            return False

        open_space = (
            front_cm is not None
            and left_cm is not None
            and right_cm is not None
            and front_cm >= EXIT_FRONT_OPEN_CM
            and left_cm >= EXIT_SIDE_OPEN_CM
            and right_cm >= EXIT_SIDE_OPEN_CM
            and ir_left_blocked is not True
            and ir_right_blocked is not True
        )

        if open_space:
            self.reset_count = 0
            if not self.active:
                if None in pose_xy:
                    return False
                self.active = True
                self.start_xy = (float(pose_xy[0]), float(pose_xy[1]))
                self.start_time = time.monotonic()
                print(">>> EXIT CANDIDATE: sustained open space started")
                return False

            moved = distance_xy(self.start_xy or (None, None), pose_xy)
            elapsed = time.monotonic() - self.start_time if self.start_time is not None else 0.0
            if (
                moved is not None
                and moved >= EXIT_CONFIRM_DISTANCE_M
                and elapsed >= EXIT_CONFIRM_MIN_SEC
            ):
                return True
            return False

        if self.active:
            self.reset_count += 1
            if self.reset_count >= EXIT_RESET_SAMPLES:
                self.reset()
        return False


# ============================================================
# Junction graph / Trémaux memory
# ============================================================

@dataclass
class EdgeState:
    # confirmed traversals only: incremented after arriving at a DIFFERENT node
    traversals: int = 0
    target: Optional[str] = None
    observed: bool = False
    attempts: int = 0
    blocked: bool = False


@dataclass
class JunctionNode:
    node_id: str
    x: float
    y: float
    gx: int
    gy: int
    exits: Dict[int, EdgeState] = field(default_factory=dict)
    seen_count: int = 1


@dataclass
class Plan:
    relative: str
    absolute: int
    reason: str


class MazeGraphExplorer:
    def __init__(self):
        self.nodes: Dict[str, JunctionNode] = {}
        self.next_index = 0
        self.current_node: Optional[str] = None
        self.start_node: Optional[str] = None
        self.first_junction_node: Optional[str] = None
        # The first corridor (start -> first junction) is the entrance corridor.
        # Once we have entered the maze, graph routing must never use this edge
        # as a transit route back outside.
        self.entry_edge_nodes: Optional[Tuple[str, str]] = None
        self.heading_index = 0

        # Corridor currently being traversed from a known node.
        self.pending_from: Optional[str] = None
        self.pending_abs: Optional[int] = None
        self.pending_start_xy: Optional[Tuple[float, float]] = None

        self.history: List[dict] = []
        self.graph_events: List[dict] = []

    # ---------------- headings ----------------
    @staticmethod
    def opposite(abs_dir: int) -> int:
        return (abs_dir + 2) % 4

    def absolute_for_relative(self, relative: str) -> int:
        return (self.heading_index + RELATIVE_OFFSET[relative]) % 4

    def relative_for_absolute(self, abs_dir: int) -> str:
        diff = (abs_dir - self.heading_index) % 4
        return {0: "FRONT", 1: "RIGHT", 2: "BACK", 3: "LEFT"}[diff]

    # ---------------- grid ----------------
    @staticmethod
    def grid_xy(x: float, y: float) -> Tuple[int, int]:
        return (
            int(round(x / GRID_RESOLUTION_M)),
            int(round(y / GRID_RESOLUTION_M)),
        )

    def _new_node(self, x: float, y: float) -> str:
        node_id = f"J{self.next_index}"
        self.next_index += 1
        gx, gy = self.grid_xy(x, y)
        self.nodes[node_id] = JunctionNode(node_id, float(x), float(y), gx, gy)
        return node_id

    def initialize_start(self, x: float, y: float) -> str:
        node_id = self._new_node(x, y)
        self.start_node = node_id
        self.current_node = node_id
        return node_id

    def _edge(self, node_id: str, abs_dir: int) -> EdgeState:
        node = self.nodes[node_id]
        abs_dir %= 4
        if abs_dir not in node.exits:
            node.exits[abs_dir] = EdgeState()
        return node.exits[abs_dir]

    def _touch_node(self, node_id: str, x: float, y: float):
        node = self.nodes[node_id]
        alpha = NODE_POSITION_UPDATE_ALPHA
        node.x = (1.0 - alpha) * node.x + alpha * x
        node.y = (1.0 - alpha) * node.y + alpha * y
        node.gx, node.gy = self.grid_xy(node.x, node.y)
        node.seen_count += 1

    def _expected_target(self, x: float, y: float, incoming_abs: Optional[int]) -> Optional[str]:
        if self.pending_from is None or self.pending_abs is None:
            return None
        if self.pending_from not in self.nodes:
            return None
        state = self.nodes[self.pending_from].exits.get(self.pending_abs)
        if state is None or state.target is None or state.target not in self.nodes:
            return None

        target = self.nodes[state.target]
        d = math.hypot(x - target.x, y - target.y)
        if d > EXPECTED_TARGET_MATCH_RADIUS_M:
            return None

        # The expected node should expose the reverse corridor.
        if incoming_abs is not None:
            incoming_state = target.exits.get(incoming_abs)
            if incoming_state is not None and incoming_state.target not in (None, self.pending_from):
                return None
        return target.node_id

    def _topology_compatible(self, node: JunctionNode, observed_abs: set, incoming_abs: Optional[int]) -> bool:
        # Incoming corridor is the strongest identity feature for a revisit.
        if incoming_abs is not None:
            incoming_state = node.exits.get(incoming_abs)
            if incoming_state is None:
                # For extended-distance loop closure, require that the candidate
                # at least already knows the corridor by which we are arriving.
                return False

        known_open = set(node.exits.keys())
        if not known_open:
            return True

        # Require at least one non-incoming overlap if the candidate has one.
        candidate_other = known_open - ({incoming_abs} if incoming_abs is not None else set())
        observed_other = observed_abs - ({incoming_abs} if incoming_abs is not None else set())
        if candidate_other and observed_other and not (candidate_other & observed_other):
            return False
        return True

    def _match_node(self, x: float, y: float, observed_abs: set, incoming_abs: Optional[int]):
        expected = self._expected_target(x, y, incoming_abs)
        if expected is not None:
            return expected, "EXPECTED_TARGET"

        candidates = []
        for node_id, node in self.nodes.items():
            d = math.hypot(x - node.x, y - node.y)

            # V2: allow a very-close event to collapse back into the node we
            # just departed.  This is intentionally a *special* match, not a
            # normal node merge.  It handles a wide junction/corner that
            # re-triggers the detector 10-30 cm after departure.
            if self.pending_from is not None and node_id == self.pending_from:
                if d <= SAME_NODE_RETRIGGER_RADIUS_M:
                    candidates.append((d, -1, node_id))
                continue

            if d <= NODE_MATCH_RADIUS_M:
                candidates.append((d, 0, node_id))
            elif d <= LOOP_MATCH_RADIUS_M and self._topology_compatible(node, observed_abs, incoming_abs):
                candidates.append((d, 1, node_id))

        if not candidates:
            return None, None

        candidates.sort(key=lambda item: (item[1], item[0]))
        d, match_class, node_id = candidates[0]
        if match_class == -1:
            return node_id, "SAME_NODE_RETRIGGER"
        if match_class == 1:
            return node_id, "TOPOLOGY_LOOP_MATCH"
        return node_id, "NEAR_POSITION"

    def _connect(self, a: str, abs_a: int, b: str):
        if a == b:
            return
        abs_a %= 4
        abs_b = self.opposite(abs_a)
        ea = self._edge(a, abs_a)
        eb = self._edge(b, abs_b)

        if ea.target not in (None, b):
            self.graph_events.append({
                "kind": "EDGE_CONFLICT",
                "from": a,
                "heading": HEADINGS[abs_a],
                "existing": ea.target,
                "attempted": b,
            })
            return
        if eb.target not in (None, a):
            self.graph_events.append({
                "kind": "EDGE_CONFLICT_REVERSE",
                "from": b,
                "heading": HEADINGS[abs_b],
                "existing": eb.target,
                "attempted": a,
            })
            return

        ea.target = b
        eb.target = a
        shared = max(ea.traversals, eb.traversals)
        ea.traversals = shared
        eb.traversals = shared
        ea.observed = True
        eb.observed = True
        ea.blocked = False
        eb.blocked = False

    def _confirm_arrival_traversal(self, from_node: str, abs_dir: int, to_node: str):
        """Count an edge only after the robot reaches a different node."""
        if from_node == to_node:
            return
        ea = self._edge(from_node, abs_dir)
        eb = self._edge(to_node, self.opposite(abs_dir))
        new_count = max(ea.traversals, eb.traversals) + 1
        ea.traversals = new_count
        eb.traversals = new_count
        ea.blocked = False
        eb.blocked = False

    def pending_progress_m(self, x: float, y: float) -> Optional[float]:
        if self.pending_start_xy is None:
            return None
        sx, sy = self.pending_start_xy
        return math.hypot(float(x) - sx, float(y) - sy)

    def cancel_pending_as_blocked(self, reason: str, x: float, y: float):
        """Cancel a short failed departure without pretending it was traversed."""
        if self.pending_from is None or self.pending_abs is None:
            return
        state = self._edge(self.pending_from, self.pending_abs)
        state.blocked = True
        progress = self.pending_progress_m(x, y)
        self.graph_events.append({
            "kind": "FAILED_DEPARTURE_BLOCKED",
            "node": self.pending_from,
            "heading": HEADINGS[self.pending_abs],
            "progress_m": progress,
            "reason": reason,
        })
        self.current_node = self.pending_from
        self.pending_from = None
        self.pending_abs = None
        self.pending_start_xy = None

    def arrive(self, x: float, y: float, observed_abs: set) -> Tuple[str, bool, str]:
        previous_from = self.pending_from
        previous_abs = self.pending_abs
        incoming_abs = self.opposite(previous_abs) if previous_abs is not None else None
        matched, match_reason = self._match_node(x, y, observed_abs, incoming_abs)
        is_new = matched is None
        node_id = self._new_node(x, y) if is_new else matched

        same_node_retrigger = (
            not is_new
            and previous_from is not None
            and node_id == previous_from
            and match_reason == "SAME_NODE_RETRIGGER"
        )

        # Do not drag the canonical node position toward the far edge of the
        # same physical intersection on a re-trigger.
        if not is_new and not same_node_retrigger:
            self._touch_node(node_id, x, y)

        if previous_from is not None and previous_abs is not None:
            if same_node_retrigger:
                progress = self.pending_progress_m(x, y)
                self.graph_events.append({
                    "kind": "SAME_NODE_RETRIGGER",
                    "node": node_id,
                    "heading": HEADINGS[previous_abs],
                    "distance_m": math.hypot(
                        x - self.nodes[node_id].x,
                        y - self.nodes[node_id].y,
                    ),
                    "departure_progress_m": progress,
                })
                # IMPORTANT V3: do NOT clear pending_from/pending_abs and do NOT
                # increment traversal here. This is not a successful corridor
                # traversal. Main decides whether to keep moving through a wide
                # junction or cancel a short hard-blocked false exit.
                self.current_node = node_id
                return node_id, False, "SAME_NODE_RETRIGGER"
            else:
                self._connect(previous_from, previous_abs, node_id)
                self._confirm_arrival_traversal(previous_from, previous_abs, node_id)

                # First real junction reached from the synthetic start node.
                if (
                    self.start_node is not None
                    and previous_from == self.start_node
                    and node_id != self.start_node
                    and self.entry_edge_nodes is None
                ):
                    self.first_junction_node = node_id
                    self.entry_edge_nodes = (self.start_node, node_id)
                    self.graph_events.append({
                        "kind": "ENTRY_EDGE_LOCKED",
                        "start": self.start_node,
                        "first_junction": node_id,
                    })

        self.current_node = node_id
        self.pending_from = None
        self.pending_abs = None
        self.pending_start_xy = None

        return node_id, is_new, ("NEW" if is_new else match_reason or "MATCH")

    def observe_openings(self, observed_abs: set):
        if self.current_node is None:
            return
        for abs_dir in observed_abs:
            state = self._edge(self.current_node, abs_dir)
            state.observed = True

    def frontiers(self, node_id: Optional[str] = None) -> List[int]:
        if node_id is None:
            node_id = self.current_node
        if node_id is None:
            return []
        result = []
        for abs_dir, state in self.nodes[node_id].exits.items():
            if (
                state.observed
                and state.traversals == 0
                and state.target is None
                and not state.blocked
            ):
                result.append(abs_dir)
        return sorted(result)

    def all_frontiers(self) -> List[Tuple[str, int]]:
        items = []
        for node_id in sorted(self.nodes, key=lambda n: int(n[1:]) if n[1:].isdigit() else 999999):
            for abs_dir in self.frontiers(node_id):
                items.append((node_id, abs_dir))
        return items

    def _is_entry_edge(self, a: str, b: str) -> bool:
        if self.entry_edge_nodes is None:
            return False
        s, j = self.entry_edge_nodes
        return (a == s and b == j) or (a == j and b == s)

    def _neighbors(self, node_id: str):
        for abs_dir, state in self.nodes[node_id].exits.items():
            if state.target is None or state.target not in self.nodes:
                continue
            # V2: the start corridor is one-way for exploration.  It was used
            # once to enter the maze, but BFS must not route back through it.
            if self._is_entry_edge(node_id, state.target):
                continue
            yield state.target, abs_dir

    def _path_to_nearest_frontier(self) -> Optional[List[str]]:
        if self.current_node is None:
            return None
        q = deque([[self.current_node]])
        visited = {self.current_node}

        while q:
            path = q.popleft()
            node_id = path[-1]
            if node_id != self.current_node and self.frontiers(node_id):
                return path
            for nxt, _ in self._neighbors(node_id):
                if nxt in visited:
                    continue
                visited.add(nxt)
                q.append(path + [nxt])
        return None

    def _abs_to_neighbor(self, node_id: str, neighbor_id: str) -> Optional[int]:
        for abs_dir, state in self.nodes[node_id].exits.items():
            if state.target == neighbor_id:
                return abs_dir
        return None

    def plan(self, current_observed_abs: Optional[set] = None) -> Optional[Plan]:
        if self.current_node is None:
            raise RuntimeError("No current node")

        # 1) DFS/Trémaux: prefer a frontier physically confirmed by the CURRENT
        # centred scan. V2 could remember FRONT as open from an older moving
        # window, then drive straight even while current ToF was ~10 cm.
        local_frontiers = self.frontiers(self.current_node)
        rank = {name: i for i, name in enumerate(EXPLORATION_PREFERENCE)}

        if local_frontiers and current_observed_abs is not None:
            physical = [d for d in local_frontiers if d in current_observed_abs]
            if physical:
                choices = []
                for abs_dir in physical:
                    rel = self.relative_for_absolute(abs_dir)
                    choices.append((rank.get(rel, 99), rel, abs_dir))
                choices.sort()
                _, rel, abs_dir = choices[0]
                return Plan(rel, abs_dir, "LOCAL_CONFIRMED_UNVISITED_EXIT")

            # A remembered side branch can still be useful if the stopped Sharp
            # misses the mouth by a few cm. Turning toward it is safe because
            # front ToF validates the branch after the turn. Never force a
            # remembered FRONT when current ToF says it is blocked.
            side_memory = []
            for abs_dir in local_frontiers:
                rel = self.relative_for_absolute(abs_dir)
                if rel in ("LEFT", "RIGHT"):
                    side_memory.append((rank.get(rel, 99), rel, abs_dir))
            if side_memory:
                side_memory.sort()
                _, rel, abs_dir = side_memory[0]
                return Plan(rel, abs_dir, "REMEMBERED_SIDE_FRONTIER")

        elif local_frontiers:
            choices = []
            for abs_dir in local_frontiers:
                rel = self.relative_for_absolute(abs_dir)
                choices.append((rank.get(rel, 99), rel, abs_dir))
            choices.sort()
            _, rel, abs_dir = choices[0]
            return Plan(rel, abs_dir, "LOCAL_UNVISITED_EXIT")

        # 2) If no frontier exists anywhere, graph exploration is complete.
        if not self.all_frontiers():
            return None

        # 3) Transit through known graph to nearest unexplored frontier.
        path = self._path_to_nearest_frontier()
        if path and len(path) >= 2:
            abs_dir = self._abs_to_neighbor(self.current_node, path[1])
            if abs_dir is not None:
                return Plan(
                    self.relative_for_absolute(abs_dir),
                    abs_dir,
                    "ROUTE_TO_NEAREST_FRONTIER",
                )

        raise RuntimeError(
            "Frontiers exist but none are reachable in the current graph. "
            "This indicates a node/edge matching error."
        )

    def commit_departure(self, plan: Plan, start_xy: Optional[Tuple[float, float]] = None):
        if self.current_node is None:
            raise RuntimeError("No current node")

        state = self._edge(self.current_node, plan.absolute)
        state.attempts += 1

        # V3: traversal count is NOT incremented here. A corridor is counted only
        # after arrive() reaches a different node and confirms the connection.
        self.pending_from = self.current_node
        self.pending_abs = plan.absolute
        self.pending_start_xy = start_xy
        self.heading_index = plan.absolute
        self.history.append({
            "time": time.time(),
            "node": self.current_node,
            "relative": plan.relative,
            "absolute": HEADINGS[plan.absolute],
            "reason": plan.reason,
            "edge_traversals_confirmed": state.traversals,
            "edge_attempts": state.attempts,
            "frontiers_remaining": len(self.all_frontiers()),
        })

    def describe_node(self, node_id: Optional[str] = None) -> str:
        node_id = node_id or self.current_node
        if node_id is None or node_id not in self.nodes:
            return "NO_NODE"
        node = self.nodes[node_id]
        parts = []
        for abs_dir in range(4):
            if abs_dir not in node.exits:
                continue
            e = node.exits[abs_dir]
            frontier = e.observed and e.traversals == 0 and e.target is None and not e.blocked
            tag = "*" if frontier else ("[X]" if e.blocked else "")
            parts.append(
                f"{HEADINGS[abs_dir]}:{e.traversals}->{e.target or '?'}"
                f"(a{e.attempts}){tag}"
            )
        return " | ".join(parts) if parts else "NO_EXITS"

    def save(self, filepath=MAZE_MEMORY_FILE):
        data = {
            "version": PROGRAM_VERSION,
            "current_node": self.current_node,
            "start_node": self.start_node,
            "first_junction_node": self.first_junction_node,
            "entry_edge_nodes": list(self.entry_edge_nodes) if self.entry_edge_nodes else None,
            "heading": HEADINGS[self.heading_index],
            "pending_from": self.pending_from,
            "pending_heading": HEADINGS[self.pending_abs] if self.pending_abs is not None else None,
            "frontiers": [
                {"node": n, "heading": HEADINGS[h]} for n, h in self.all_frontiers()
            ],
            "nodes": {},
            "history": self.history,
            "graph_events": self.graph_events,
        }
        for node_id, node in self.nodes.items():
            data["nodes"][node_id] = {
                "x": node.x,
                "y": node.y,
                "grid": [node.gx, node.gy],
                "seen_count": node.seen_count,
                "exits": {
                    HEADINGS[h]: {
                        "traversals": e.traversals,
                        "target": e.target,
                        "observed": e.observed,
                        "attempts": e.attempts,
                        "blocked": e.blocked,
                        "frontier": (
                            e.observed
                            and e.traversals == 0
                            and e.target is None
                            and not e.blocked
                        ),
                    }
                    for h, e in node.exits.items()
                },
            }
        with open(filepath, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)


# ============================================================
# Physical actions
# ============================================================

def stop_chassis(chassis):
    if chassis is None or not ENABLE_MOTION:
        return
    try:
        chassis.drive_speed(x=0.0, y=0.0, z=0.0, timeout=0.2)
    except Exception:
        pass


def wait_for_position(pose: PoseTracker) -> Tuple[float, float]:
    deadline = time.monotonic() + POSE_WAIT_SEC
    while time.monotonic() < deadline:
        if pose.has_position():
            x, y = pose.get_xy()
            return float(x), float(y)
        time.sleep(0.05)
    print("WARNING: odometry not ready; using (0,0)")
    return 0.0, 0.0


def wait_for_yaw(pose: PoseTracker) -> Optional[float]:
    deadline = time.monotonic() + POSE_WAIT_SEC
    while time.monotonic() < deadline:
        yaw = pose.get_yaw()
        if yaw is not None:
            return yaw
        time.sleep(0.05)
    print("WARNING: attitude yaw not ready")
    return None


def travelled_from(start_xy, pose: PoseTracker) -> Optional[float]:
    return distance_xy(start_xy, pose.get_xy())


def feedback_turn(chassis, pose: PoseTracker, relative: str) -> bool:
    angle = RELATIVE_TURN_DEG[relative]
    if abs(angle) < 0.001:
        return True
    if not ENABLE_MOTION:
        return True

    start_yaw = pose.get_yaw()
    if start_yaw is None:
        print("TURN ERROR: yaw unavailable")
        return False

    # Preserve supplied robot convention:
    # RIGHT logical command=-90 and move->yaw sign=-1 => attitude target +90.
    target_yaw = normalize_angle_deg(start_yaw + angle * DEFAULT_MOVE_TO_YAW_SIGN)
    timeout = TURN_FEEDBACK_TIMEOUT_180_SEC if abs(angle) > 135 else TURN_FEEDBACK_TIMEOUT_90_SEC

    stop_chassis(chassis)
    time.sleep(TURN_PRE_SETTLE_SEC)
    started = time.monotonic()
    stable = 0

    print(f">>> TURN {relative}: yaw {start_yaw:+.1f} -> {target_yaw:+.1f}")

    while time.monotonic() - started < timeout:
        yaw = pose.get_yaw()
        if yaw is None:
            time.sleep(TURN_FEEDBACK_LOOP_SEC)
            continue

        err = shortest_angle_error_deg(target_yaw, yaw)
        if abs(err) <= TURN_FEEDBACK_TOLERANCE_DEG:
            stable += 1
            stop_chassis(chassis)
            if stable >= TURN_FEEDBACK_STABLE_SAMPLES:
                time.sleep(YAW_SETTLE_SEC)
                return True
        else:
            stable = 0
            speed = clamp(
                abs(err) * TURN_FEEDBACK_KP,
                TURN_FEEDBACK_MIN_Z_SPEED,
                TURN_FEEDBACK_MAX_Z_SPEED,
            )
            z_cmd = math.copysign(speed, err) / DEFAULT_DRIVE_TO_YAW_SIGN
            chassis.drive_speed(
                x=0.0,
                y=0.0,
                z=z_cmd,
                timeout=TURN_FEEDBACK_DRIVE_TIMEOUT_SEC,
            )
        time.sleep(TURN_FEEDBACK_LOOP_SEC)

    stop_chassis(chassis)
    print("TURN TIMEOUT")
    return False


def align_heading(chassis, pose: PoseTracker, heading: HeadingManager):
    if not ENABLE_MOTION or heading.target_yaw is None:
        return
    deadline = time.monotonic() + HEADING_ALIGN_TIMEOUT_SEC
    while time.monotonic() < deadline:
        yaw = pose.get_yaw()
        err = heading.error(yaw)
        if err is None or abs(err) <= HEADING_ALIGN_TOLERANCE_DEG:
            break
        z, _ = heading.correction_z(yaw, recover=True)
        chassis.drive_speed(x=0.0, y=0.0, z=z, timeout=DRIVE_TIMEOUT_SEC)
        time.sleep(HEADING_ALIGN_LOOP_SEC)
    stop_chassis(chassis)


def backtrack_to_opening_center(chassis, pose: PoseTracker, heading: HeadingManager, meters: float):
    if not ENABLE_MOTION or meters <= 0.005:
        return

    target = min(max(0.0, meters), OPENING_CENTER_MAX_BACKTRACK_M)
    start = pose.get_xy()
    started = time.monotonic()
    print(f">>> CENTER SIDE OPENING: reverse {target:.3f}m")

    while time.monotonic() - started < OPENING_CENTER_TIMEOUT_SEC:
        moved = travelled_from(start, pose)
        if moved is not None and moved >= target:
            break
        x, y, z, _, _ = heading.apply(
            -OPENING_CENTER_SPEED,
            0.0,
            pose.get_yaw(),
            "OPENING_CENTER",
        )
        chassis.drive_speed(x=x, y=y, z=z, timeout=DRIVE_TIMEOUT_SEC)
        time.sleep(OPENING_CENTER_LOOP_SEC)
    stop_chassis(chassis)


def center_front_blocked(chassis, sensors: SensorManager, pose: PoseTracker, heading: HeadingManager):
    """Move slowly toward the front wall until a repeatable turn distance."""
    if not ENABLE_MOTION:
        return

    start = pose.get_xy()
    started = time.monotonic()
    print(f">>> CENTER FRONT BLOCKED: target ToF={CORNER_CENTER_FRONT_TARGET_CM:.1f}cm")

    while time.monotonic() - started < CORNER_CENTER_TIMEOUT_SEC:
        front = sensors.get_front_cm()
        if front is None:
            break

        ir_left, ir_right = sensors.read_front_corner_ir()
        if ir_left is True or ir_right is True:
            print(
                "FRONT_CENTER stop: front-corner IR blocked "
                f"(L={ir_left}, R={ir_right})"
            )
            break

        if front <= CORNER_CENTER_FRONT_HARD_STOP_CM:
            break
        if front <= CORNER_CENTER_FRONT_TARGET_CM:
            break
        moved = travelled_from(start, pose)
        if moved is not None and moved >= CORNER_CENTER_MAX_DISTANCE_M:
            break

        x, y, z, _, _ = heading.apply(
            CORNER_CENTER_SPEED,
            0.0,
            pose.get_yaw(),
            "FRONT_CENTER",
        )
        chassis.drive_speed(x=x, y=y, z=z, timeout=DRIVE_TIMEOUT_SEC)
        time.sleep(CORNER_CENTER_LOOP_SEC)
    stop_chassis(chassis)


def post_turn_clearance(
    chassis,
    sensors: SensorManager,
    pose: PoseTracker,
    heading: HeadingManager,
    relative: str,
):
    """Crawl out of an inside corner before returning to full corridor speed."""
    if not ENABLE_POST_TURN_CLEARANCE or relative not in ("LEFT", "RIGHT"):
        return
    if not ENABLE_MOTION:
        return

    sensors.reset_filters()
    start = pose.get_xy()
    started = time.monotonic()

    # After LEFT turn the old inside corner is on the left; vice versa.
    read_inner = sensors.read_left if relative == "LEFT" else sensors.read_right
    y_out = (
        +POST_TURN_Y_SPEED * Y_DIR_SIGN
        if relative == "LEFT"
        else -POST_TURN_Y_SPEED * Y_DIR_SIGN
    )

    inner = None
    for _ in range(3):
        _, inner = read_inner()
        time.sleep(POST_TURN_LOOP_SEC)

    ir_left, ir_right = sensors.read_front_corner_ir()
    inner_ir = ir_left if relative == "LEFT" else ir_right

    if (
        inner is not None
        and inner >= POST_TURN_TRIGGER_CM
        and inner_ir is not True
    ):
        return

    print(
        f">>> POST_TURN_CLEARANCE {relative} "
        f"inner={fmt(inner)} IR={inner_ir}"
    )

    while time.monotonic() - started < POST_TURN_MAX_SEC:
        front = sensors.get_front_cm()
        if front is None:
            stop_chassis(chassis)
            time.sleep(POST_TURN_LOOP_SEC)
            continue
        if front <= POST_TURN_FRONT_STOP_CM:
            break

        _, inner = read_inner()
        ir_left, ir_right = sensors.read_front_corner_ir()
        inner_ir = ir_left if relative == "LEFT" else ir_right

        if (
            inner is not None
            and inner >= POST_TURN_RELEASE_CM
            and inner_ir is not True
        ):
            break

        moved = travelled_from(start, pose)
        if moved is not None and moved >= POST_TURN_MAX_DISTANCE_M:
            break

        x, y, z, _, _ = heading.apply(
            POST_TURN_FORWARD_SPEED,
            y_out,
            pose.get_yaw(),
            "POST_TURN_CLEARANCE",
        )
        chassis.drive_speed(x=x, y=y, z=z, timeout=DRIVE_TIMEOUT_SEC)
        time.sleep(POST_TURN_LOOP_SEC)

    stop_chassis(chassis)


def scan_junction(sensors: SensorManager, event: JunctionEvent):
    time.sleep(JUNCTION_SETTLE_SEC)
    front_values, left_values, right_values = [], [], []

    for i in range(DECISION_SCAN_SAMPLES):
        _, left = sensors.read_left()
        _, right = sensors.read_right()
        front = sensors.get_front_cm()
        if front is not None:
            front_values.append(front)
        if left is not None:
            left_values.append(left)
        if right is not None:
            right_values.append(right)
        if i + 1 < DECISION_SCAN_SAMPLES:
            time.sleep(DECISION_SCAN_INTERVAL_SEC)

    front = median_or_none(front_values)
    left = median_or_none(left_values)
    right = median_or_none(right_values)

    raw_front_open = front is not None and front >= EXPLORATION_FRONT_OPEN_CM
    raw_left_open = left is not None and left >= SIDE_OPEN_ENTER_CM
    raw_right_open = right is not None and right >= SIDE_OPEN_ENTER_CM

    front_open = raw_front_open or event.observed_front
    left_open = raw_left_open or event.observed_left
    right_open = raw_right_open or event.observed_right

    # A true close front reading always wins for safety.
    if front is not None and front <= STOP_FRONT_CM:
        front_open = False

    print(
        f">>> SCAN F={fmt(front)} {'OPEN' if front_open else 'BLOCK'} | "
        f"L={fmt(left)} {'OPEN' if left_open else 'BLOCK'} | "
        f"R={fmt(right)} {'OPEN' if right_open else 'BLOCK'}"
    )

    return {
        "front_cm": front,
        "left_cm": left,
        "right_cm": right,
        "front_open": front_open,
        "left_open": left_open,
        "right_open": right_open,
    }


def observed_absolute_openings(explorer: MazeGraphExplorer, scan) -> set:
    result = set()
    if scan["front_open"]:
        result.add(explorer.absolute_for_relative("FRONT"))
    if scan["left_open"]:
        result.add(explorer.absolute_for_relative("LEFT"))
    if scan["right_open"]:
        result.add(explorer.absolute_for_relative("RIGHT"))

    # BACK is physically open because the robot just arrived through it.
    if explorer.pending_abs is not None:
        result.add(explorer.opposite(explorer.pending_abs))
    elif explorer.current_node is not None:
        # At the synthetic start node there may not be a meaningful BACK yet.
        pass
    return result


# ============================================================
# Main
# ============================================================

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

        pose = PoseTracker()
        sensors = SensorManager(sensor_adapter)
        heading = HeadingManager()
        walls = WallController()
        junctions = JunctionDetector()
        exit_detector = ExitDetector()
        explorer = MazeGraphExplorer()

        tof_subscribed = tof_sensor.sub_distance(freq=20, callback=sensors.tof_callback)
        pose_subscribed = chassis.sub_position(cs=1, freq=POSE_FREQ_HZ, callback=pose.position_callback)
        attitude_subscribed = chassis.sub_attitude(freq=ATTITUDE_FREQ_HZ, callback=pose.attitude_callback)

        start_x, start_y = wait_for_position(pose)
        start_yaw = wait_for_yaw(pose)
        heading.initialize(start_yaw)

        start_node = explorer.initialize_start(start_x, start_y)

        # Initial corridor departure: create a forward frontier at the synthetic start.
        start_forward = explorer.absolute_for_relative("FRONT")
        explorer._edge(start_node, start_forward).observed = True
        initial_plan = Plan("FRONT", start_forward, "INITIAL_FORWARD")
        explorer.commit_departure(initial_plan, start_xy=(start_x, start_y))

        print("============================================================")
        print(" RoboMaster CLEAN Wall-Maze Solver")
        print("============================================================")
        print(f"Version             : {PROGRAM_VERSION}")
        print(f"Forward speed       : {FORWARD_SPEED:.2f} m/s")
        print(f"Side open           : >= {SIDE_OPEN_ENTER_CM:.1f} cm")
        print(f"Front open          : >= {EXPLORATION_FRONT_OPEN_CM:.1f} cm")
        print(f"Node match          : {NODE_MATCH_RADIUS_M:.2f} m")
        print(f"Topology loop match : {LOOP_MATCH_RADIUS_M:.2f} m")
        print(f"Same-node retrigger : <= {SAME_NODE_RETRIGGER_RADIUS_M:.2f} m")
        print("Entry edge guard    : ON (start corridor is not a return route)")
        print(f"Grid resolution     : {GRID_RESOLUTION_M:.2f} m")
        print(f"Exit confirm        : {EXIT_CONFIRM_DISTANCE_M:.2f} m sustained open space")
        print(f"IR front-left ID    : {IR_FRONT_LEFT_ID}")
        print(f"IR front-right ID   : {IR_FRONT_RIGHT_ID}")
        print(f"IR blocked level    : {IR_BLOCKED_LEVEL}")
        print(f"Digital IR mode      : get_io() active-low")
        print(f"Side warning        : < {SIDE_WARNING_CM:.1f} cm")
        print("Confirmed-edge DFS  : ON (count edge only after arrival)")
        print("Post-turn clearance : ON" if ENABLE_POST_TURN_CLEARANCE else "Post-turn clearance : OFF")
        print("============================================================")

        last_print = 0.0

        while True:
            raw_l, left_cm = sensors.read_left()
            raw_r, right_cm = sensors.read_right()
            front_cm = sensors.get_front_cm()
            ir_left, ir_right = sensors.read_front_corner_ir()
            pose_xy = pose.get_xy()

            # ------------------------------------------------
            # 1) Maze exit has priority over creating endless outside nodes.
            # ------------------------------------------------
            if exit_detector.update(
                front_cm,
                left_cm,
                right_cm,
                pose_xy,
                ir_left,
                ir_right,
            ):
                stop_chassis(chassis)
                print()
                print("============================================")
                print(" MAZE EXIT CONFIRMED")
                print("============================================")
                if SAVE_MAZE_MEMORY:
                    explorer.save()
                break

            # ------------------------------------------------
            # 2) Junction detection
            # ------------------------------------------------
            event = junctions.update(front_cm, left_cm, right_cm, pose_xy)

            if event is not None:
                stop_chassis(chassis)
                walls.reset()

                # Establish a repeatable physical node position BEFORE graph registration.
                if event.kind == "SIDE_WINDOW":
                    backtrack_to_opening_center(chassis, pose, heading, event.backtrack_m)
                else:
                    center_front_blocked(chassis, sensors, pose, heading)

                # Stable stopped scan at the canonical junction centre.
                scan = scan_junction(sensors, event)
                if None in (scan["front_cm"], scan["left_cm"], scan["right_cm"]):
                    print("Decision rejected: incomplete sensor snapshot")
                    junctions.lock_here(pose.get_xy())
                    time.sleep(LOOP_DELAY_SEC)
                    continue

                # If a side event resolves into a plain corridor, ignore it.
                if (
                    event.kind == "SIDE_WINDOW"
                    and scan["front_open"]
                    and not scan["left_open"]
                    and not scan["right_open"]
                    and not (event.observed_left or event.observed_right)
                ):
                    print("Decision rejected: plain corridor after centred scan")
                    junctions.lock_here(pose.get_xy())
                    time.sleep(LOOP_DELAY_SEC)
                    continue

                x, y = pose.get_xy()
                if x is None or y is None:
                    print("No odometry at junction; stopping for safety")
                    break

                observed_abs = observed_absolute_openings(explorer, scan)
                node_id, is_new, match_reason = explorer.arrive(
                    float(x), float(y), observed_abs
                )
                explorer.observe_openings(observed_abs)

                print()
                print(
                    f"[{'NEW' if is_new else 'KNOWN'} NODE] {node_id} "
                    f"at ({x:+.3f},{y:+.3f}) match={match_reason}"
                )
                print("Memory:", explorer.describe_node(node_id))
                print(
                    "Frontiers:",
                    " | ".join(f"{n}.{HEADINGS[h]}" for n, h in explorer.all_frontiers()) or "NONE",
                )

                # V3: a same-node retrigger is NOT a successful edge traversal.
                # If the planned direction is still open, this was probably a
                # wide junction mouth: ignore the duplicate event and continue.
                # If it immediately ends in a hard front block after only a
                # short distance, cancel it as a false/blocked exit.
                if match_reason == "SAME_NODE_RETRIGGER":
                    pending_abs = explorer.pending_abs
                    progress = explorer.pending_progress_m(float(x), float(y))
                    pending_still_open = (
                        pending_abs is not None and pending_abs in observed_abs
                    )
                    pending_unknown = False
                    if explorer.pending_from is not None and pending_abs is not None:
                        pending_unknown = (
                            explorer._edge(explorer.pending_from, pending_abs).target is None
                        )
                    hard_front_blocked = (
                        scan["front_cm"] is not None
                        and scan["front_cm"] <= STOP_FRONT_CM
                    )

                    if (
                        pending_unknown
                        and not pending_still_open
                        and hard_front_blocked
                        and progress is not None
                        and progress <= FAILED_EDGE_MAX_PROGRESS_M
                    ):
                        print(
                            ">>> FALSE/SHORT EXIT: cancel pending edge "
                            f"progress={progress:.3f}m front={scan['front_cm']:.1f}cm"
                        )
                        explorer.cancel_pending_as_blocked(
                            "SAME_NODE_HARD_FRONT",
                            float(x),
                            float(y),
                        )
                        junctions.lock_here(pose.get_xy())
                    else:
                        print(
                            ">>> SAME NODE WINDOW: keep pending corridor, "
                            f"progress={0.0 if progress is None else progress:.3f}m"
                        )
                        if SAVE_MAZE_MEMORY:
                            explorer.save()
                        junctions.lock_here(pose.get_xy())
                        time.sleep(LOOP_DELAY_SEC)
                        continue

                if SAVE_MAZE_MEMORY:
                    explorer.save()

                plan = explorer.plan(current_observed_abs=observed_abs)
                if plan is None:
                    stop_chassis(chassis)
                    print()
                    print("============================================")
                    print(" GRAPH EXPLORATION COMPLETE")
                    print(" No unexplored junction exits remain.")
                    print("============================================")
                    if SAVE_MAZE_MEMORY:
                        explorer.save()
                    break

                print(
                    f">>> PLAN {plan.relative} ({HEADINGS[plan.absolute]}) "
                    f"reason={plan.reason}"
                )

                # Physical action first; graph edge is marked only if turn succeeds.
                if not feedback_turn(chassis, pose, plan.relative):
                    stop_chassis(chassis)
                    print("Turn failed safely; departure NOT committed")
                    break

                explorer.commit_departure(plan, start_xy=(float(x), float(y)))
                heading.set_heading_index(explorer.heading_index)
                align_heading(chassis, pose, heading)

                # Clear the inside corner before resuming normal speed.
                post_turn_clearance(
                    chassis,
                    sensors,
                    pose,
                    heading,
                    plan.relative,
                )

                sensors.reset_filters()
                walls.reset()
                junctions.lock_here(pose.get_xy())
                exit_detector.reset()

                if SAVE_MAZE_MEMORY:
                    explorer.save()

                stop_chassis(chassis)
                time.sleep(AFTER_TURN_DELAY_SEC)
                continue

            # ------------------------------------------------
            # 3) Normal corridor movement
            # ------------------------------------------------
            if front_cm is not None and front_cm <= STOP_FRONT_CM:
                # Stop while JunctionDetector gathers confirmation samples.
                x_cmd = 0.0
                y_cmd = 0.0
                z_cmd = 0.0
                mode = "FRONT_CONFIRM"
            else:
                x_cmd = walls.forward_speed(front_cm)
                y_cmd, mode = walls.lateral(left_cm, right_cm)

                # Side-too-close escape should not keep full forward speed.
                if mode.startswith("ESCAPE_"):
                    x_cmd = min(x_cmd, ESCAPE_FORWARD_SPEED)
                elif mode.startswith("AVOID_"):
                    x_cmd = min(x_cmd, SIDE_WARNING_FORWARD_SPEED)
                if mode in ("BOTH_TOO_CLOSE", "NO_SIDE_SENSOR"):
                    x_cmd = 0.0

                # Front-corner digital IR guard protects chassis corners.
                x_cmd, y_cmd, ir_mode = apply_front_corner_ir_guard(
                    x_cmd, y_cmd, ir_left, ir_right
                )
                if ir_mode:
                    mode = ir_mode

                x_cmd, y_cmd, z_cmd, mode, _ = heading.apply(
                    x_cmd,
                    y_cmd,
                    pose.get_yaw(),
                    mode,
                )

            if ENABLE_MOTION:
                chassis.drive_speed(
                    x=x_cmd,
                    y=y_cmd,
                    z=z_cmd,
                    timeout=DRIVE_TIMEOUT_SEC,
                )

            # ------------------------------------------------
            # Debug print, throttled
            # ------------------------------------------------
            now = time.monotonic()
            if now - last_print >= PRINT_EVERY_SEC:
                px, py = pose.get_xy()
                yaw = pose.get_yaw()
                yaw_err = heading.error(yaw)
                print(
                    f"ToF:{fmt(front_cm)} | "
                    f"L:{fmt(left_cm)} ADC:{raw_l:4d} | "
                    f"R:{fmt(right_cm)} ADC:{raw_r:4d} | "
                    f"IR-L:{ir_left} IR-R:{ir_right} | "
                    f"Pose:({px if px is not None else 0:+.2f},{py if py is not None else 0:+.2f}) | "
                    f"Yaw:{yaw if yaw is not None else 0:+.1f} "
                    f"Err:{yaw_err if yaw_err is not None else 0:+.1f} | "
                    f"H:{HEADINGS[explorer.heading_index]} | "
                    f"{mode:22s} | x={x_cmd:.3f} y={y_cmd:+.3f} z={z_cmd:+.1f}"
                )
                last_print = now

            time.sleep(LOOP_DELAY_SEC)

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

        try:
            ep_robot.close()
        except Exception:
            pass

        print("Robot stopped and disconnected.")


if __name__ == "__main__":
    main()

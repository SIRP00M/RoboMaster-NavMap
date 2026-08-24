"""
RoboMaster Maze Walking Test - SINGLE FILE
==========================================

Movement/exploration only. No occupancy-grid mapper.
Uses the V6 frontier-aware exploration logic with intersection-window junction detection, edge splitting, loop guard, wall following, ToF front safety,
Sharp left/right sensing, odometry and yaw heading hold.

Important test values:
    FORWARD_SPEED = 0.20 m/s
    SIDE_OPEN_ENTER_CM = 18.0 cm / SIDE_OPEN_EXIT_CM = 15.0 cm
    TARGET_LEFT_CM / TARGET_RIGHT_CM = 8.0 cm
    SIDE_TOO_CLOSE_CM = 5.5 cm

Run:
    py maze_walk_only_test_v6.py

Requires RoboMaster Python SDK.
"""

try:
    from robomaster import robot
except ModuleNotFoundError as exc:
    raise SystemExit(
        "RoboMaster SDK is not installed for this Python. "
        "Install it first, then run this file again."
    ) from exc



# ==================== CONFIG ====================
"""Configuration for the RoboMaster maze solver.

ปรับค่าจูนของหุ่นทั้งหมดจากไฟล์นี้ไฟล์เดียว
"""

# ============================================================
# GENERAL
# ============================================================

ENABLE_MOTION = True
PROGRAM_VERSION = "WALK_ONLY_V6_INTERSECTION_WINDOW"


# ============================================================
# SENSOR CONFIG
# ============================================================

IR_LEFT_FRONT_ID = 1
SHARP_LEFT_ID = 2
SHARP_RIGHT_ID = 3
SENSOR_PORT = 1

SHARP_FILTER_SIZE = 3
TOF_FILTER_SIZE = 3
SHARP_EMA_NEW_WEIGHT = 0.6
SHARP_EMA_OLD_WEIGHT = 0.4

# ถ้า ToF ไม่อัปเดตเกินเวลานี้ ให้ถือว่าไม่รู้ระยะหน้า
TOF_STALE_SEC = 0.40


# ============================================================
# DISTANCE CONFIG
# ============================================================

TARGET_LEFT_CM = 8.0
TARGET_RIGHT_CM = 8.0

SLOW_FRONT_CM = 18.0
STOP_FRONT_CM = 15.0

SIDE_DEAD_END_CM = 15.0
SIDE_TOO_CLOSE_CM = 5.5

# Wall-state hysteresis: เข้าโหมดมีกำแพงที่ค่าต่ำกว่า ENTER
# และจะปล่อยสถานะก็ต่อเมื่อสูงกว่า EXIT เพื่อลดการสลับ 29/30/29/30 cm
SIDE_WALL_ENTER_CM = 28.0
SIDE_WALL_EXIT_CM = 32.0

# เก็บชื่อเดิมไว้เผื่อโค้ด/debug ภายนอกยังอ้างอยู่
SIDE_WALL_DETECT_CM = 30.0

# ต้องต่างกันมากกว่านี้ถึงถือว่าฝั่งหนึ่งเปิดชัดเจน
SIDE_OPEN_DIFFERENCE_CM = 20.0


# ============================================================
# ROBOT SPEED
# ============================================================

FORWARD_SPEED = 0.20
MIN_FORWARD_SPEED = 0.05

# เมื่อ ToF ยังไม่มีข้อมูลสด จะไม่พุ่งไปข้างหน้า
UNKNOWN_FRONT_SPEED = 0.0

# ตอนหนีกำแพง ให้ลดความเร็วหน้าเพื่อให้แกน Y มีเวลาพาหุ่นออก
ESCAPE_FORWARD_SPEED = 0.04


# ============================================================
# DIRECTION
# ============================================================

# ถ้าสไลด์ซ้าย/ขวากลับทิศ เปลี่ยนเป็น -1
Y_DIR_SIGN = 1

# ใช้กับ chassis.move(z=...) สำหรับการเลี้ยว
# RoboMaster SDK example: +z = LEFT, -z = RIGHT สำหรับ move()
Z_DIR_SIGN = 1


# ============================================================
# SINGLE WALL CONTROLLER
# ============================================================

SIDE_KP_STRAFE = 0.012
SIDE_MAX_Y = 0.05
SIDE_DEADBAND_CM = 1.0


# ============================================================
# BOTH-WALL / OWNER CONTROLLER
# ============================================================

CENTER_TRIGGER_CM = 2.0
CENTER_RELEASE_CM = 0.7
CENTER_HOLD_SEC = 0.30
CENTER_KP_STRAFE = 0.009
CENTER_MAX_Y = 0.04


# ============================================================
# ESCAPE CONFIG
# ============================================================

ESCAPE_Y_SPEED = 0.07


# ============================================================
# TURN CONFIG
# ============================================================

TURN_SPEED = 60
TURN_LEFT_DEG = 90
TURN_RIGHT_DEG = -90
TURN_AROUND_DEG = -180

# ตรวจ yaw หลัง chassis.move() และแก้มุมเล็กน้อยถ้าคลาดมากเกิน tolerance
ENABLE_YAW_CORRECTION = True
ATTITUDE_FREQ_HZ = 20
YAW_SETTLE_SEC = 0.08
YAW_TOLERANCE_DEG = 3.0
YAW_MAX_CORRECTION_DEG = 15.0
TURN_CORRECTION_SPEED = 35


# ============================================================
# HEADING HOLD / STRAIGHT-LINE CONTROL
# ============================================================

# IMPORTANT: RoboMaster SDK ใช้คนละ command path สำหรับ turn กับ velocity control.
# จาก log ของหุ่นตัวนี้:
#   chassis.move(z=-90)  -> attitude yaw เพิ่มประมาณ +90  => sign = -1
#   drive_speed(z<0)     -> attitude yaw ลด             => sign = +1
# ห้ามใช้ sign เดียวกันกับสอง API นี้
DEFAULT_MOVE_TO_YAW_SIGN = -1
DEFAULT_DRIVE_TO_YAW_SIGN = +1

ENABLE_HEADING_HOLD = True
HEADING_KP_Z = 1.5              # ลด gain จาก V4 เพื่อไม่ให้แก้หัวแรงเกิน
HEADING_MAX_Z_SPEED = 12.0      # จำกัดการแก้หัวระหว่างวิ่ง
HEADING_DEADBAND_DEG = 0.8

# ถ้าหัวหลุดมาก ให้หยุด x/y แล้วตั้งหัวกลับก่อน ไม่ลากตัวรถไปชนกำแพง
HEADING_RECOVER_TRIGGER_DEG = 8.0
HEADING_RECOVER_RELEASE_DEG = 2.0
HEADING_RECOVER_MAX_Z_SPEED = 18.0

# หลังเลี้ยว/ตัดสินใจ หยุดตั้งหัวกลับเข้ากริดก่อนออกจาก junction
ENABLE_ABSOLUTE_HEADING_ALIGN = True
HEADING_ALIGN_TOLERANCE_DEG = 2.0
HEADING_ALIGN_TIMEOUT_SEC = 1.20
HEADING_ALIGN_LOOP_SEC = 0.04


# ============================================================
# JUNCTION CENTERING / LEAVING
# ============================================================

# เจอช่องด้านข้างขณะด้านหน้ายังโล่ง -> ค่อย ๆ เข้าไปกลาง junction ก่อน scan/turn
ENABLE_JUNCTION_CREEP = True
JUNCTION_CREEP_SPEED = 0.07
# V6: ใช้ระยะ odometry แทนเวลาคงที่ เพื่อให้เข้า junction ลึกสม่ำเสมอ
JUNCTION_CREEP_DISTANCE_M = 0.06
JUNCTION_CREEP_MAX_SEC = 1.20
JUNCTION_CREEP_ABORT_FRONT_CM = 16.0
JUNCTION_CREEP_LOOP_SEC = 0.05

# V6: กรณีโค้งจริง (ด้านหน้าไม่ใช่ทางตรง แต่ LEFT/RIGHT เปิด)
# V5 จะหมุนทันที ทำให้ pivot อยู่ก่อนถึงศูนย์กลางมุมและท้ายหุ่นกวาดชนมุมใน
ENABLE_CORNER_TURN_SETUP = True
CORNER_TURN_SETUP_SPEED = 0.05
CORNER_TURN_SETUP_DISTANCE_M = 0.14
CORNER_TURN_SETUP_MAX_SEC = 3.00
# หยุดดันหน้าเมื่อ ToF ถึงระยะนี้ แม้ odometry ยังไม่ครบ
CORNER_TURN_FRONT_TARGET_CM = 11.0
# safety hard stop: ห้ามเข้าใกล้กว่านี้ระหว่างจัดตำแหน่งก่อนหมุน
CORNER_TURN_FRONT_HARD_STOP_CM = 10.5
CORNER_TURN_SETUP_LOOP_SEC = 0.04

# V8: after a 90-degree turn, the inside side of the chassis may still be
# beside the old corner wall. Crawl out of the corner before resuming full speed.
ENABLE_POST_TURN_CLEARANCE = True
POST_TURN_CLEARANCE_TRIGGER_CM = 6.5
POST_TURN_CLEARANCE_RELEASE_CM = 7.5
POST_TURN_CLEARANCE_FORWARD_SPEED = 0.045
POST_TURN_CLEARANCE_Y_SPEED = 0.035
POST_TURN_CLEARANCE_MAX_DISTANCE_M = 0.07
POST_TURN_CLEARANCE_MAX_SEC = 1.50
POST_TURN_CLEARANCE_FRONT_STOP_CM = 12.0
POST_TURN_CLEARANCE_LOOP_SEC = 0.04

# หลังตัดสินใจแล้ว detector จะล็อก junction เดิมไว้ชั่วคราว
# แต่ปลดได้จากระยะทาง, corridor samples, timeout หรือ emergency front block
JUNCTION_REARM_MIN_DISTANCE_M = 0.14
JUNCTION_REARM_DISTANCE_M = 0.25
JUNCTION_REARM_TIMEOUT_SEC = 2.50
JUNCTION_REARM_EMERGENCY_SEC = 0.25


# ============================================================
# LOOP / CONFIRMATION
# ============================================================

AFTER_TURN_DELAY_SEC = 0.12
LOOP_DELAY_SEC = 0.05
DRIVE_TIMEOUT_SEC = 0.15

# จำนวน sample ตอนหยุด scan junction ซ้ำเพื่อให้ Sharp นิ่งขึ้น
DECISION_SCAN_SAMPLES = 5
DECISION_SCAN_INTERVAL_SEC = 0.04


# ============================================================
# TRÉMAUX / DFS EXPLORATION
# ============================================================

# V6: side-opening hysteresis + intersection-window detection.
# A wall must rise to >= 18 cm before a side opening can START. Once the
# opening is active, it remains open until Sharp falls below 15 cm. This avoids
# 14-17 cm edge/corner readings creating a fake junction.
SIDE_OPEN_ENTER_CM = 18.0
SIDE_OPEN_EXIT_CM = 15.0
# Compatibility alias used by stopped decision scans and older helper code.
EXPLORATION_SIDE_OPEN_CM = SIDE_OPEN_ENTER_CM

# Do not create a graph node at the first OPEN sample. Track the whole mouth of
# the side branch, require a minimum travelled width, then reverse to the
# estimated centre of that opening before the final stopped scan. This makes
# outbound and inbound passes associate with the same physical junction.
ENABLE_OPENING_ZONE_DETECTION = True
OPENING_ZONE_ENTER_SAMPLES = 3
OPENING_ZONE_EXIT_SAMPLES = 3
OPENING_ZONE_MIN_LENGTH_M = 0.10
OPENING_ZONE_MAX_LENGTH_M = 0.70
ENABLE_OPENING_ZONE_CENTERING = True
OPENING_ZONE_CENTERING_SPEED = 0.07
OPENING_ZONE_CENTERING_MAX_BACKTRACK_M = 0.40
OPENING_ZONE_CENTERING_MAX_SEC = 6.5
OPENING_ZONE_CENTERING_LOOP_SEC = 0.04

# V6: after the first confirmed side opening, keep observing a short
# intersection window instead of deciding from one stopped snapshot.
# This captures layouts where LEFT opens first, FRONT remains traversable, and
# the old RIGHT branch becomes visible a few centimetres later.
ENABLE_INTERSECTION_WINDOW = True
INTERSECTION_WINDOW_LOOKAHEAD_M = 0.18
INTERSECTION_WINDOW_MAX_M = 0.55
INTERSECTION_MIN_OPEN_SAMPLES = 2
INTERSECTION_FRONT_OPEN_CM = 35.0
INTERSECTION_SIDE_OPEN_CM = SIDE_OPEN_ENTER_CM


# ระยะหน้า 17-24 cm ยังใกล้กำแพงเกินไปที่จะถือว่าเป็นทางตรง
# ใช้ 35 cm จาก log สนามจริง: กำแพงหน้า ~27 cm ที่ T-junction ต้องเป็น BLOCK
EXPLORATION_FRONT_OPEN_CM = 35.0

# Number of consecutive samples needed before declaring a decision point.
JUNCTION_CONFIRM_SAMPLES = 3

# Samples of normal corridor needed before the same detector can trigger again.
JUNCTION_REARM_SAMPLES = 4

# Odometry radius used to decide that a junction is one already seen before.
NODE_MATCH_RADIUS_M = 0.18
# Target-specific revisit tolerance for a corridor whose graph target is already known.
# This does not enlarge generic node merging.
EXPECTED_TARGET_MATCH_RADIUS_M = 0.26

# Small position averaging when the same junction is re-observed.
NODE_POSITION_UPDATE_ALPHA = 0.20

# Compatibility value kept for old logs/code. Frontier-aware exploration no
# longer forbids a third traversal: known corridors may be reused as transit
# routes while the robot returns to an unexplored frontier.
MAX_EDGE_VISITS = 2

# Tie-breaker when multiple physically confirmed NEW exits exist at one node.
EXPLORATION_PREFERENCE = ("FRONT", "LEFT", "RIGHT", "BACK")

# ---------------- Frontier-aware DFS / graph routing ----------------
# A frontier = an observed opening with visits==0 and no known target node.
# Exploration is COMPLETE only when no active frontier exists anywhere.
ENABLE_FRONTIER_AWARE_DFS = True
# A remembered side frontier at the current known junction outranks routing BACK
# if one stopped Sharp scan narrowly misses the opening.
ENABLE_REMEMBERED_LOCAL_FRONTIER = True
REMEMBERED_FRONTIER_MIN_SEEN = 1

# A one-off Sharp/ToF false opening can otherwise keep the maze unfinished
# forever. Retire an untraversed frontier only after this many separate stopped
# revisits to the SAME node fail to observe that opening. Set 0 to disable.
ENABLE_STALE_FRONTIER_RETIRE = True
FRONTIER_STALE_MISS_LIMIT = 3

# V4 graph/route robustness.
# If a new junction is detected while travelling along an already-known direct
# edge A->B, insert it as A->X->B instead of rejecting X as a graph conflict.
ENABLE_INTERMEDIATE_NODE_EDGE_SPLIT = True

# Prevent ROUTE_TO_* from choosing the exact same directed edge again while the
# set of remaining frontiers has not changed. This breaks J3<->...<->J9 style
# oscillation without globally preferring FRONT over correct DFS backtracking.
ENABLE_ROUTE_LOOP_BREAK = True
ROUTE_REPEAT_LIMIT = 1

# Preserve the actual traversal path through loops. A revisit to an older node
# is appended to the stack unless it is a normal one-step backtrack.
DFS_STACK_MAX_LEN = 128

# RoboMaster chassis odometry subscription.
POSE_FREQ_HZ = 20
POSE_WAIT_SEC = 1.0

# Save graph/marks for debugging after every decision.
SAVE_MAZE_MEMORY = True
MAZE_MEMORY_FILE = "maze_memory.json"

# Stop automatically after DFS returns to its first decision root with all branches covered.
STOP_WHEN_EXPLORATION_COMPLETE = True

# Short pause before re-reading sensors at a detected junction.
JUNCTION_SETTLE_SEC = 0.15


# ============================================================
# SHARP CALIBRATION: ADC -> CM
# ============================================================

# ตอนนี้ใช้ calibration เดิมเป็นค่าเริ่มต้นให้ทั้งสองตัว
# ถ้าวัดจริงแล้ว Sensor ID 2/3 ให้ค่าไม่เหมือนกัน ให้แยกจูนสองตารางนี้
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

# backward compatibility
CALIBRATION_SHARP2 = CALIBRATION_SHARP_LEFT

# ============================================================
# V7 FEEDBACK TURN / ACTION WATCHDOG
# ============================================================
# Normal 90/180 turns use attitude yaw + drive_speed closed-loop so the program
# does not depend on an SDK action-completion ACK.
ENABLE_FEEDBACK_TURN = True
TURN_PRE_SETTLE_SEC = 0.10
TURN_FEEDBACK_KP = 1.20
TURN_FEEDBACK_MIN_Z_SPEED = 10.0
TURN_FEEDBACK_MAX_Z_SPEED = 55.0
TURN_FEEDBACK_TOLERANCE_DEG = 2.0
TURN_FEEDBACK_STABLE_SAMPLES = 3
TURN_FEEDBACK_LOOP_SEC = 0.03
TURN_FEEDBACK_DRIVE_TIMEOUT_SEC = 0.20
TURN_FEEDBACK_TIMEOUT_90_SEC = 3.50
TURN_FEEDBACK_TIMEOUT_180_SEC = 6.00
TURN_FEEDBACK_PRINT_SEC = 0.25

# Fallback only if attitude yaw is unavailable. Even this path is bounded.
TURN_ACTION_TIMEOUT_90_SEC = 3.50
TURN_ACTION_TIMEOUT_180_SEC = 6.00



# Existing modules were merged into one file.  This proxy lets the original
# tested code keep using config.NAME without importing a separate config.py.
class _ConfigProxy:
    def __getattr__(self, name):
        try:
            return globals()[name]
        except KeyError as exc:
            raise AttributeError(name) from exc

config = _ConfigProxy()


# ==================== POSE TRACKER ====================
"""Thread-safe chassis position + attitude tracker."""

import threading



def normalize_angle_deg(angle):
    """Normalize angle to [-180, 180)."""
    return (float(angle) + 180.0) % 360.0 - 180.0


def shortest_angle_error_deg(target, current):
    """Signed shortest error needed to rotate current -> target."""
    return normalize_angle_deg(float(target) - float(current))


class PoseTracker:
    """Hold odometry position and chassis attitude from separate subscriptions."""

    def __init__(self):
        self._lock = threading.Lock()

        self.x = None
        self.y = None
        self.position_z = None

        self.yaw_deg = None
        self.pitch_deg = None
        self.roll_deg = None

        # Keep separate sign mappings for the two RoboMaster APIs.
        # +1 = command z sign and attitude-yaw sign agree.
        # -1 = command z sign and attitude-yaw sign are opposite.
        #
        # IMPORTANT: on this robot chassis.move(z) and drive_speed(z) do NOT
        # behave with the same yaw-sign convention (confirmed by real logs).
        self.move_to_yaw_sign = (
            config.DEFAULT_MOVE_TO_YAW_SIGN
            if hasattr(config, "DEFAULT_MOVE_TO_YAW_SIGN")
            else None
        )
        self.drive_to_yaw_sign = (
            config.DEFAULT_DRIVE_TO_YAW_SIGN
            if hasattr(config, "DEFAULT_DRIVE_TO_YAW_SIGN")
            else None
        )

    def position_callback(self, data):
        """RoboMaster sub_position callback: data = (x, y, z)."""
        try:
            if data is None or len(data) < 3:
                return

            x, y, z = data[:3]

            with self._lock:
                self.x = float(x)
                self.y = float(y)
                self.position_z = float(z)

        except Exception as exc:
            print("Position callback error:", exc)

    def attitude_callback(self, data):
        """RoboMaster sub_attitude callback: data = (yaw, pitch, roll)."""
        try:
            if data is None or len(data) < 3:
                return

            yaw, pitch, roll = data[:3]

            with self._lock:
                self.yaw_deg = normalize_angle_deg(yaw)
                self.pitch_deg = float(pitch)
                self.roll_deg = float(roll)

        except Exception as exc:
            print("Attitude callback error:", exc)

    def get_pose(self):
        """Return (x, y, yaw_deg). yaw comes from sub_attitude()."""
        with self._lock:
            return self.x, self.y, self.yaw_deg

    def get_position(self):
        with self._lock:
            return self.x, self.y, self.position_z

    def get_yaw(self):
        with self._lock:
            return self.yaw_deg

    def has_pose(self):
        x, y, _ = self.get_position()
        return x is not None and y is not None

    def has_yaw(self):
        return self.get_yaw() is not None

    def set_move_to_yaw_sign(self, sign):
        if sign not in (-1, 1):
            return
        with self._lock:
            self.move_to_yaw_sign = sign

    def get_move_to_yaw_sign(self):
        with self._lock:
            return self.move_to_yaw_sign

    def set_drive_to_yaw_sign(self, sign):
        if sign not in (-1, 1):
            return
        with self._lock:
            self.drive_to_yaw_sign = sign

    def get_drive_to_yaw_sign(self):
        with self._lock:
            return self.drive_to_yaw_sign

# ==================== SENSOR MANAGER ====================
"""Sensor reading, calibration and filtering."""

import statistics
import time
from collections import deque



class SensorManager:
    def __init__(self, sensor_adapter):
        self.sensor_adapter = sensor_adapter

        self.sharp_left_buffer = deque(maxlen=config.SHARP_FILTER_SIZE)
        self.sharp_right_buffer = deque(maxlen=config.SHARP_FILTER_SIZE)
        self.tof_buffer = deque(maxlen=config.TOF_FILTER_SIZE)

        self.sharp_left_ema = None
        self.sharp_right_ema = None
        self.front_cm = None
        self.tof_last_update = None

    # ========================================================
    # TOF
    # ========================================================

    def tof_callback(self, data):
        try:
            if not data or data[0] is None:
                return

            mm = data[0]

            # Reject invalid values
            if mm < 20 or mm > 4000:
                return

            cm = mm / 10.0
            self.tof_buffer.append(cm)
            self.front_cm = statistics.median(self.tof_buffer)
            self.tof_last_update = time.monotonic()

        except Exception as exc:
            print("ToF callback error:", exc)

    def get_front_cm(self):
        """Return fresh ToF distance, or None if data is absent/stale."""
        if self.front_cm is None or self.tof_last_update is None:
            return None

        age = time.monotonic() - self.tof_last_update
        if age > config.TOF_STALE_SEC:
            return None

        return self.front_cm

    # ========================================================
    # SHARP CALIBRATION
    # ========================================================

    @staticmethod
    def adc_to_cm(adc, table):
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

    def calibration_for_sensor(self, sensor_id):
        if sensor_id == config.SHARP_LEFT_ID:
            return config.CALIBRATION_SHARP_LEFT
        if sensor_id == config.SHARP_RIGHT_ID:
            return config.CALIBRATION_SHARP_RIGHT
        raise ValueError(f"Unknown Sharp sensor id: {sensor_id}")

    # ========================================================
    # SHARP READ + FILTER
    # ========================================================

    def read_sharp_raw_and_cm(self, sensor_id):
        try:
            raw = self.sensor_adapter.get_adc(
                id=sensor_id,
                port=config.SENSOR_PORT,
            )
        except Exception as exc:
            print(f"Sharp {sensor_id} read error: {exc}")
            return 0, None

        if sensor_id == config.SHARP_LEFT_ID:
            self.sharp_left_buffer.append(raw)
            median_adc = statistics.median(self.sharp_left_buffer)

            if self.sharp_left_ema is None:
                self.sharp_left_ema = median_adc
            else:
                self.sharp_left_ema = (
                    config.SHARP_EMA_NEW_WEIGHT * median_adc
                    + config.SHARP_EMA_OLD_WEIGHT * self.sharp_left_ema
                )

            ema_val = self.sharp_left_ema

        elif sensor_id == config.SHARP_RIGHT_ID:
            self.sharp_right_buffer.append(raw)
            median_adc = statistics.median(self.sharp_right_buffer)

            if self.sharp_right_ema is None:
                self.sharp_right_ema = median_adc
            else:
                self.sharp_right_ema = (
                    config.SHARP_EMA_NEW_WEIGHT * median_adc
                    + config.SHARP_EMA_OLD_WEIGHT * self.sharp_right_ema
                )

            ema_val = self.sharp_right_ema

        else:
            raise ValueError(f"Unknown Sharp sensor id: {sensor_id}")

        table = self.calibration_for_sensor(sensor_id)
        return raw, self.adc_to_cm(ema_val, table)

    def read_left_sharp(self):
        return self.read_sharp_raw_and_cm(config.SHARP_LEFT_ID)

    def read_right_sharp(self):
        return self.read_sharp_raw_and_cm(config.SHARP_RIGHT_ID)

    # ========================================================
    # DIGITAL IR
    # ========================================================

    def read_ir_digital_io(self):
        try:
            return self.sensor_adapter.get_io_level(
                id=config.IR_LEFT_FRONT_ID,
                port=config.SENSOR_PORT,
            )

        except Exception:
            try:
                raw = self.sensor_adapter.get_adc(
                    id=config.IR_LEFT_FRONT_ID,
                    port=config.SENSOR_PORT,
                )
                return 1 if raw > 300 else 0
            except Exception:
                return None

    # ========================================================
    # RESET FILTERS AFTER TURN
    # ========================================================

    def reset_filters(self):
        self.tof_buffer.clear()
        self.sharp_left_buffer.clear()
        self.sharp_right_buffer.clear()

        self.sharp_left_ema = None
        self.sharp_right_ema = None
        self.front_cm = None
        self.tof_last_update = None

# ==================== MOTION CONTROLLER ====================
"""Forward speed, side-wall and heading-hold motion controller."""

import time



def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


class MotionController:
    def __init__(self):
        self.side_owner = "NONE"
        self.side_owner_since = 0.0

        self.left_wall_present = False
        self.right_wall_present = False

        # Absolute yaw grid. Initial physical yaw = internal N.
        self.heading_base_yaw = None
        self.heading_target_yaw = None
        self.heading_right_step_sign = None
        self.heading_recovering = False

    # ========================================================
    # RESET / STATE
    # ========================================================

    def reset_side_owner(self):
        self.side_owner = "NONE"
        self.side_owner_since = 0.0

    def reset_wall_states(self):
        self.left_wall_present = False
        self.right_wall_present = False

    def reset_after_turn(self):
        # Important: do NOT reset absolute heading target here.
        self.reset_side_owner()
        self.reset_wall_states()

    # ========================================================
    # HEADING HOLD
    # ========================================================

    def initialize_heading(self, yaw_deg, pose_tracker=None):
        if yaw_deg is None:
            return False

        self.heading_base_yaw = normalize_angle_deg(yaw_deg)
        self.heading_target_yaw = self.heading_base_yaw
        self.heading_recovering = False

        if pose_tracker is not None:
            if pose_tracker.get_move_to_yaw_sign() is None:
                pose_tracker.set_move_to_yaw_sign(config.DEFAULT_MOVE_TO_YAW_SIGN)
            self._learn_heading_axis_from_pose(pose_tracker)

        return True

    def _learn_heading_axis_from_pose(self, pose_tracker):
        """Learn attitude-yaw direction for one logical RIGHT grid step."""
        sign_map = pose_tracker.get_move_to_yaw_sign()
        if sign_map not in (-1, 1):
            return False

        # Logical RIGHT uses TURN_RIGHT_DEG (-90 by default).
        # actual yaw delta sign = command sign * move_to_yaw_sign.
        right_command_sign = 1 if (config.TURN_RIGHT_DEG * config.Z_DIR_SIGN) > 0 else -1
        self.heading_right_step_sign = right_command_sign * sign_map
        return True

    @staticmethod
    def _signed_cardinal_step(heading_index):
        # 0=N, 1=E/right, 2=S, 3=W/left
        return {0: 0, 1: 1, 2: 2, 3: -1}[heading_index % 4]

    def set_heading_index(self, heading_index, pose_tracker=None):
        if self.heading_base_yaw is None:
            current_yaw = pose_tracker.get_yaw() if pose_tracker is not None else None
            if current_yaw is None:
                return None
            self.initialize_heading(current_yaw, pose_tracker=pose_tracker)

        if pose_tracker is not None:
            self._learn_heading_axis_from_pose(pose_tracker)

        if self.heading_right_step_sign not in (-1, 1):
            return self.heading_target_yaw

        step = self._signed_cardinal_step(heading_index)
        self.heading_target_yaw = normalize_angle_deg(
            self.heading_base_yaw
            + self.heading_right_step_sign * 90.0 * step
        )
        return self.heading_target_yaw

    def heading_error(self, current_yaw):
        if self.heading_target_yaw is None or current_yaw is None:
            return None
        return shortest_angle_error_deg(self.heading_target_yaw, current_yaw)

    def calculate_heading_hold(self, current_yaw, pose_tracker, recover=False):
        """Return (z_command_deg_s, yaw_error_deg)."""
        if not config.ENABLE_HEADING_HOLD:
            return 0.0, None

        error = self.heading_error(current_yaw)
        if error is None:
            return 0.0, None

        if abs(error) <= config.HEADING_DEADBAND_DEG:
            return 0.0, error

        # Heading hold is sent through chassis.drive_speed(), NOT chassis.move().
        # These two RoboMaster APIs have separate yaw-sign mappings on this robot.
        sign_map = pose_tracker.get_drive_to_yaw_sign()
        if sign_map not in (-1, 1):
            return 0.0, error

        max_z = (
            config.HEADING_RECOVER_MAX_Z_SPEED
            if recover
            else config.HEADING_MAX_Z_SPEED
        )

        desired_yaw_rate = clamp(
            error * config.HEADING_KP_Z,
            -max_z,
            max_z,
        )

        # drive_speed z sign -> attitude yaw sign through DRIVE sign_map.
        # With DRIVE_TO_YAW_SIGN=+1, positive error correctly yields +z.
        z_cmd = desired_yaw_rate / sign_map
        return z_cmd, error

    def apply_heading_hold(self, x, y, current_yaw, pose_tracker, mode):
        """Add z correction and stop translation if yaw has drifted too far."""
        if not config.ENABLE_HEADING_HOLD:
            return x, y, 0.0, mode, None

        error = self.heading_error(current_yaw)
        if error is None:
            return x, y, 0.0, mode, None

        if self.heading_recovering:
            if abs(error) <= config.HEADING_RECOVER_RELEASE_DEG:
                self.heading_recovering = False
        elif abs(error) >= config.HEADING_RECOVER_TRIGGER_DEG:
            self.heading_recovering = True

        if self.heading_recovering:
            z_cmd, error = self.calculate_heading_hold(
                current_yaw,
                pose_tracker,
                recover=True,
            )
            return 0.0, 0.0, z_cmd, "HEADING_RECOVER", error

        z_cmd, error = self.calculate_heading_hold(
            current_yaw,
            pose_tracker,
            recover=False,
        )
        return x, y, z_cmd, mode, error

    # ========================================================
    # FRONT SPEED CONTROL
    # ========================================================

    @staticmethod
    def calculate_forward_speed(front_distance):
        if front_distance is None:
            return config.UNKNOWN_FRONT_SPEED

        if front_distance >= config.SLOW_FRONT_CM:
            return config.FORWARD_SPEED

        if front_distance <= config.STOP_FRONT_CM:
            return 0.0

        ratio = (
            (front_distance - config.STOP_FRONT_CM)
            / (config.SLOW_FRONT_CM - config.STOP_FRONT_CM)
        )

        return (
            config.MIN_FORWARD_SPEED
            + ratio * (config.FORWARD_SPEED - config.MIN_FORWARD_SPEED)
        )

    # ========================================================
    # WALL HYSTERESIS
    # ========================================================

    @staticmethod
    def _update_wall_state(distance_cm, current_state):
        if distance_cm is None:
            return False

        if current_state:
            return distance_cm < config.SIDE_WALL_EXIT_CM

        return distance_cm < config.SIDE_WALL_ENTER_CM

    def update_wall_states(self, sharp_left_cm, sharp_right_cm):
        # IMPORTANT: an observed maze opening must not simultaneously be treated
        # as a wall by the centering controller. The old thresholds overlapped
        # (opening >=20 cm while wall-enter <28 cm), which caused violent y kicks.
        left_is_opening = (
            sharp_left_cm is not None
            and sharp_left_cm >= config.EXPLORATION_SIDE_OPEN_CM
        )
        right_is_opening = (
            sharp_right_cm is not None
            and sharp_right_cm >= config.EXPLORATION_SIDE_OPEN_CM
        )

        if left_is_opening:
            self.left_wall_present = False
        else:
            self.left_wall_present = self._update_wall_state(
                sharp_left_cm,
                self.left_wall_present,
            )

        if right_is_opening:
            self.right_wall_present = False
        else:
            self.right_wall_present = self._update_wall_state(
                sharp_right_cm,
                self.right_wall_present,
            )

        return self.left_wall_present, self.right_wall_present

    # ========================================================
    # BOTH-WALL OWNER CONTROLLER
    # ========================================================

    def calculate_center_owner(self, sharp_left_cm, sharp_right_cm):
        now = time.time()
        delta = sharp_left_cm - sharp_right_cm
        abs_delta = abs(delta)

        if self.side_owner == "NONE":
            if abs_delta < config.CENTER_TRIGGER_CM:
                return 0.0, "CENTER_STABLE"

            self.side_owner = "LEFT" if delta < 0 else "RIGHT"
            self.side_owner_since = now

        owner_age = now - self.side_owner_since

        if owner_age >= config.CENTER_HOLD_SEC:
            if abs_delta <= config.CENTER_RELEASE_CM:
                self.reset_side_owner()
                return 0.0, "CENTER_RELEASE"

            if self.side_owner == "LEFT" and delta >= config.CENTER_TRIGGER_CM:
                self.side_owner = "RIGHT"
                self.side_owner_since = now
            elif self.side_owner == "RIGHT" and delta <= -config.CENTER_TRIGGER_CM:
                self.side_owner = "LEFT"
                self.side_owner_since = now

        correction = clamp(
            abs_delta * config.CENTER_KP_STRAFE,
            0.0,
            config.CENTER_MAX_Y,
        )

        if self.side_owner == "LEFT":
            return +correction * config.Y_DIR_SIGN, "CENTER_LEFT_OWNER"
        if self.side_owner == "RIGHT":
            return -correction * config.Y_DIR_SIGN, "CENTER_RIGHT_OWNER"

        return 0.0, "CENTER_STABLE"

    # ========================================================
    # SIDE MOTION CONTROL
    # ========================================================

    def calculate_motion_control(
        self,
        raw_adc_l,
        sharp_left_cm,
        raw_adc_r,
        sharp_right_cm,
        ir_left_wall,
    ):
        """Sharp controls Y; heading hold in main/controller controls Z."""
        _ = raw_adc_l, raw_adc_r, ir_left_wall

        if sharp_left_cm is None or sharp_right_cm is None:
            self.reset_side_owner()
            self.reset_wall_states()
            return 0.0, 0.0, "NO_SENSOR"

        if (
            sharp_left_cm <= config.SIDE_TOO_CLOSE_CM
            and sharp_right_cm <= config.SIDE_TOO_CLOSE_CM
        ):
            self.reset_side_owner()
            return 0.0, 0.0, "BOTH_TOO_CLOSE"

        # Use FILTERED distance only. A single raw ADC spike used to trigger
        # full ±0.10 m/s strafes even while filtered distance was ~8 cm.
        if sharp_left_cm <= config.SIDE_TOO_CLOSE_CM:
            self.reset_side_owner()
            return +config.ESCAPE_Y_SPEED * config.Y_DIR_SIGN, 0.0, "ESCAPE_LEFT"

        if sharp_right_cm <= config.SIDE_TOO_CLOSE_CM:
            self.reset_side_owner()
            return -config.ESCAPE_Y_SPEED * config.Y_DIR_SIGN, 0.0, "ESCAPE_RIGHT"

        left_wall, right_wall = self.update_wall_states(
            sharp_left_cm,
            sharp_right_cm,
        )

        if left_wall and right_wall:
            y_cmd, mode = self.calculate_center_owner(
                sharp_left_cm,
                sharp_right_cm,
            )
            return y_cmd, 0.0, mode

        if left_wall:
            self.reset_side_owner()
            error = sharp_left_cm - config.TARGET_LEFT_CM
            if abs(error) <= config.SIDE_DEADBAND_CM:
                y_cmd = 0.0
            else:
                y_cmd = clamp(
                    -error * config.SIDE_KP_STRAFE * config.Y_DIR_SIGN,
                    -config.SIDE_MAX_Y,
                    config.SIDE_MAX_Y,
                )
            return y_cmd, 0.0, "FOLLOW_LEFT"

        if right_wall:
            self.reset_side_owner()
            error = sharp_right_cm - config.TARGET_RIGHT_CM
            if abs(error) <= config.SIDE_DEADBAND_CM:
                y_cmd = 0.0
            else:
                y_cmd = clamp(
                    error * config.SIDE_KP_STRAFE * config.Y_DIR_SIGN,
                    -config.SIDE_MAX_Y,
                    config.SIDE_MAX_Y,
                )
            return y_cmd, 0.0, "FOLLOW_RIGHT"

        self.reset_side_owner()
        return 0.0, 0.0, "OPEN_SPACE"

# ==================== FRONTIER EXPLORATION ====================
"""Trémaux / DFS-style maze exploration with topological memory.

The explorer remembers decision points (junctions/corners/dead ends) using
RoboMaster chassis odometry.  Every exit has a visit count:

    0 = never used        -> highest priority
    1 = used once         -> backtracking / second choice
    2+ = already covered  -> avoid unless there is no alternative

This is deliberately topological rather than a full occupancy-grid mapper.
"""

from dataclasses import dataclass, field
from collections import deque
from typing import Optional
import json
import math
import time



HEADINGS = ("N", "E", "S", "W")
RELATIVE_ORDER = ("FRONT", "RIGHT", "BACK", "LEFT")
RELATIVE_OFFSET = {
    "FRONT": 0,
    "RIGHT": 1,
    "BACK": 2,
    "LEFT": -1,
}


@dataclass
class ExitState:
    visits: int = 0
    target: Optional[str] = None
    # A frontier is an observed physical opening that has never been traversed.
    # miss_count lets repeated stopped re-scans retire a one-off false opening.
    seen_open_count: int = 0
    miss_count: int = 0
    blocked: bool = False


@dataclass
class MazeNode:
    node_id: str
    x: float
    y: float
    exits: dict = field(default_factory=dict)
    seen_count: int = 0


@dataclass(frozen=True)
class ExplorationDecision:
    direction: str
    node_id: str
    reason: str
    visits_before: int
    absolute_heading: str


class DecisionPointDetector:
    """Detect physical decision points using a V6 intersection window.

    V5 waited for one side-opening zone to finish, moved back to its midpoint,
    then trusted a stopped snapshot.  On the real maze this can miss a straight
    continuation: LEFT may open first, the robot moves farther, RIGHT (the old
    branch) appears later, and the stopped ToF can point at a nearby wall edge.

    V6 therefore keeps an *intersection window* after the first confirmed side
    opening.  During that window it accumulates whether FRONT / LEFT / RIGHT
    were physically open for multiple samples.  A short look-ahead lets the
    detector merge opposite-side openings that are longitudinally offset by the
    robot/sensor geometry.  The final stopped scan is merged with these
    accumulated observations before the explorer chooses a branch.
    """

    def __init__(self):
        self.front_candidate_count = 0
        self.clear_count = 0
        self.latched = False
        self.latch_x = None
        self.latch_y = None
        self.latch_time = None
        self.pending_zone_event = None
        self.left_zone = self._new_zone()
        self.right_zone = self._new_zone()
        self.intersection_window = self._new_intersection_window()

    @staticmethod
    def _new_zone():
        return {
            "candidate_count": 0,
            "candidate_start_x": None,
            "candidate_start_y": None,
            "active": False,
            "start_x": None,
            "start_y": None,
            "exit_count": 0,
            "max_cm": 0.0,
        }

    @staticmethod
    def _new_intersection_window():
        return {
            "active": False,
            "start_x": None,
            "start_y": None,
            "last_side_open_x": None,
            "last_side_open_y": None,
            "lookahead_start_x": None,
            "lookahead_start_y": None,
            "front_open_samples": 0,
            "left_open_samples": 0,
            "right_open_samples": 0,
            "front_max_cm": 0.0,
            "left_max_cm": 0.0,
            "right_max_cm": 0.0,
            "completed_sides": set(),
        }

    @staticmethod
    def classify_openings(front_cm, left_cm, right_cm):
        """Stopped-scan classification using strict thresholds."""
        front_open = (
            front_cm is not None
            and front_cm >= config.EXPLORATION_FRONT_OPEN_CM
        )
        front_blocked = (
            front_cm is not None
            and 0.0 < front_cm <= config.STOP_FRONT_CM
        )
        left_open = (
            left_cm is not None
            and left_cm >= config.SIDE_OPEN_ENTER_CM
        )
        right_open = (
            right_cm is not None
            and right_cm >= config.SIDE_OPEN_ENTER_CM
        )
        return front_open, front_blocked, left_open, right_open

    @staticmethod
    def _distance_xy(x1, y1, x2, y2):
        if None in (x1, y1, x2, y2):
            return None
        return math.hypot(float(x2) - float(x1), float(y2) - float(y1))

    def _distance_from_latch(self, pose_x, pose_y):
        return self._distance_xy(self.latch_x, self.latch_y, pose_x, pose_y)

    def _reset_zones(self):
        self.left_zone = self._new_zone()
        self.right_zone = self._new_zone()
        self.intersection_window = self._new_intersection_window()
        self.pending_zone_event = None

    def _release_latch(self):
        self.latched = False
        self.front_candidate_count = 0
        self.clear_count = 0
        self.latch_x = None
        self.latch_y = None
        self.latch_time = None
        self._reset_zones()

    def _latch_now(self, now, pose_x, pose_y):
        self.latched = True
        self.front_candidate_count = 0
        self.clear_count = 0
        self.latch_x = float(pose_x) if pose_x is not None else None
        self.latch_y = float(pose_y) if pose_y is not None else None
        self.latch_time = now

    def _track_side_zone(self, zone, side_cm, pose_x, pose_y, side_name):
        """Track one physical side-opening mouth; return completion metadata."""
        if side_cm is None:
            return None

        enter = float(config.SIDE_OPEN_ENTER_CM)
        exit_threshold = float(config.SIDE_OPEN_EXIT_CM)

        if not zone["active"]:
            if side_cm >= enter:
                if zone["candidate_count"] == 0:
                    zone["candidate_start_x"] = pose_x
                    zone["candidate_start_y"] = pose_y
                zone["candidate_count"] += 1
                zone["max_cm"] = max(zone["max_cm"], float(side_cm))

                if zone["candidate_count"] >= config.OPENING_ZONE_ENTER_SAMPLES:
                    zone["active"] = True
                    zone["start_x"] = zone["candidate_start_x"]
                    zone["start_y"] = zone["candidate_start_y"]
                    zone["exit_count"] = 0
                    print(
                        f">>> OPENING_ZONE {side_name} START "
                        f"Sharp={side_cm:.1f}cm enter>={enter:.1f}"
                    )
            else:
                zone.update(self._new_zone())
            return None

        zone["max_cm"] = max(zone["max_cm"], float(side_cm))
        length = self._distance_xy(
            zone["start_x"], zone["start_y"], pose_x, pose_y
        )

        if side_cm < exit_threshold:
            zone["exit_count"] += 1
        else:
            zone["exit_count"] = 0

        completed_by_exit = (
            zone["exit_count"] >= config.OPENING_ZONE_EXIT_SAMPLES
        )
        completed_by_max = (
            length is not None
            and length >= config.OPENING_ZONE_MAX_LENGTH_M
        )
        if not (completed_by_exit or completed_by_max):
            return None

        if length is None or length < config.OPENING_ZONE_MIN_LENGTH_M:
            print(
                f">>> OPENING_ZONE {side_name} REJECT "
                f"length={0.0 if length is None else length:.3f}m "
                f"< {config.OPENING_ZONE_MIN_LENGTH_M:.3f}m"
            )
            zone.update(self._new_zone())
            return None

        event = {
            "type": "SIDE_OPENING_ZONE",
            "side": side_name,
            "length_m": float(length),
            "start_x": zone["start_x"],
            "start_y": zone["start_y"],
            "end_x": pose_x,
            "end_y": pose_y,
            "max_cm": float(zone["max_cm"]),
            "forced_by_max": bool(completed_by_max and not completed_by_exit),
        }
        print(
            f">>> OPENING_ZONE {side_name} END "
            f"length={event['length_m']:.3f}m maxSharp={event['max_cm']:.1f}cm"
        )
        zone.update(self._new_zone())
        return event

    def _start_intersection_window(self, pose_x, pose_y):
        if self.intersection_window["active"]:
            return

        starts = []
        for side_name, zone in (("LEFT", self.left_zone), ("RIGHT", self.right_zone)):
            if zone["active"]:
                starts.append((side_name, zone["start_x"], zone["start_y"]))
        if not starts:
            return

        # Both side sensors are on the same chassis and normally trigger within
        # a few samples.  Use the first confirmed candidate start as the window
        # start.  Exact ordering is not safety-critical because centering is
        # clipped later.
        side_name, start_x, start_y = starts[0]
        w = self.intersection_window
        w["active"] = True
        w["start_x"] = start_x if start_x is not None else pose_x
        w["start_y"] = start_y if start_y is not None else pose_y
        w["last_side_open_x"] = pose_x
        w["last_side_open_y"] = pose_y

        # The initiating zone has already satisfied ENTER_SAMPLES, so seed its
        # evidence rather than pretending we only saw the current sample.
        if self.left_zone["active"]:
            w["left_open_samples"] = max(
                w["left_open_samples"], config.OPENING_ZONE_ENTER_SAMPLES
            )
        if self.right_zone["active"]:
            w["right_open_samples"] = max(
                w["right_open_samples"], config.OPENING_ZONE_ENTER_SAMPLES
            )

        print(
            f">>> INTERSECTION_WINDOW START by={side_name} "
            f"lookahead={config.INTERSECTION_WINDOW_LOOKAHEAD_M:.2f}m"
        )

    def _accumulate_intersection(self, front_cm, left_cm, right_cm, pose_x, pose_y):
        w = self.intersection_window
        if not w["active"]:
            return

        if front_cm is not None:
            w["front_max_cm"] = max(w["front_max_cm"], float(front_cm))
            if front_cm >= config.INTERSECTION_FRONT_OPEN_CM:
                w["front_open_samples"] += 1

        left_phys_open = (
            left_cm is not None
            and left_cm >= config.INTERSECTION_SIDE_OPEN_CM
        )
        right_phys_open = (
            right_cm is not None
            and right_cm >= config.INTERSECTION_SIDE_OPEN_CM
        )

        if left_cm is not None:
            w["left_max_cm"] = max(w["left_max_cm"], float(left_cm))
        if right_cm is not None:
            w["right_max_cm"] = max(w["right_max_cm"], float(right_cm))

        if left_phys_open:
            w["left_open_samples"] += 1
        if right_phys_open:
            w["right_open_samples"] += 1

        # EXIT hysteresis is used here so the last-open location reaches the
        # physical far edge of the intersection instead of ending on one noisy
        # sample below ENTER.
        left_still_open = (
            left_cm is not None and left_cm >= config.SIDE_OPEN_EXIT_CM
        )
        right_still_open = (
            right_cm is not None and right_cm >= config.SIDE_OPEN_EXIT_CM
        )
        if left_still_open or right_still_open or self.left_zone["active"] or self.right_zone["active"]:
            w["last_side_open_x"] = pose_x
            w["last_side_open_y"] = pose_y
            w["lookahead_start_x"] = None
            w["lookahead_start_y"] = None
        elif w["lookahead_start_x"] is None:
            w["lookahead_start_x"] = pose_x
            w["lookahead_start_y"] = pose_y
            print(
                ">>> INTERSECTION_WINDOW LOOKAHEAD "
                f"target={config.INTERSECTION_WINDOW_LOOKAHEAD_M:.2f}m"
            )

    def _intersection_should_finalize(self, pose_x, pose_y):
        w = self.intersection_window
        if not w["active"]:
            return False, None

        total = self._distance_xy(w["start_x"], w["start_y"], pose_x, pose_y)
        if total is not None and total >= config.INTERSECTION_WINDOW_MAX_M:
            return True, "MAX_LENGTH"

        if w["lookahead_start_x"] is not None:
            lookahead = self._distance_xy(
                w["lookahead_start_x"], w["lookahead_start_y"], pose_x, pose_y
            )
            if (
                lookahead is not None
                and lookahead >= config.INTERSECTION_WINDOW_LOOKAHEAD_M
            ):
                return True, "LOOKAHEAD_COMPLETE"

        return False, None

    def _finalize_intersection_window(self, pose_x, pose_y, reason):
        w = self.intersection_window
        if not w["active"]:
            return None

        total_length = self._distance_xy(
            w["start_x"], w["start_y"], pose_x, pose_y
        )
        opening_span = self._distance_xy(
            w["start_x"], w["start_y"],
            w["last_side_open_x"], w["last_side_open_y"],
        )
        total_length = float(total_length or 0.0)
        opening_span = float(opening_span or total_length)

        # We are at the end of the look-ahead.  To return to the centre of the
        # actual intersection (not the centre of the look-ahead tail), back up
        # by: total travelled - half the observed side-opening span.
        backtrack_m = max(0.0, total_length - 0.5 * opening_span)

        min_samples = int(config.INTERSECTION_MIN_OPEN_SAMPLES)
        observed = {
            "FRONT": w["front_open_samples"] >= min_samples,
            "LEFT": w["left_open_samples"] >= min_samples,
            "RIGHT": w["right_open_samples"] >= min_samples,
        }
        counts = {
            "FRONT": int(w["front_open_samples"]),
            "LEFT": int(w["left_open_samples"]),
            "RIGHT": int(w["right_open_samples"]),
        }
        event = {
            "type": "INTERSECTION_WINDOW",
            "side": "MULTI",
            "length_m": total_length,
            "opening_span_m": opening_span,
            "backtrack_m": backtrack_m,
            "start_x": w["start_x"],
            "start_y": w["start_y"],
            "end_x": pose_x,
            "end_y": pose_y,
            "observed_open": observed,
            "open_samples": counts,
            "max_cm": {
                "FRONT": float(w["front_max_cm"]),
                "LEFT": float(w["left_max_cm"]),
                "RIGHT": float(w["right_max_cm"]),
            },
            "completed_sides": sorted(w["completed_sides"]),
            "finish_reason": reason,
        }
        print(
            ">>> INTERSECTION_WINDOW END "
            f"reason={reason} length={total_length:.3f}m "
            f"span={opening_span:.3f}m backtrack={backtrack_m:.3f}m"
        )
        print(
            ">>> INTERSECTION_ACCUM "
            f"F={int(observed['FRONT'])}({counts['FRONT']}) "
            f"L={int(observed['LEFT'])}({counts['LEFT']}) "
            f"R={int(observed['RIGHT'])}({counts['RIGHT']})"
        )
        self.intersection_window = self._new_intersection_window()
        return event

    def consume_pending_zone(self):
        event = self.pending_zone_event
        self.pending_zone_event = None
        return event

    def update(
        self,
        front_cm,
        left_cm,
        right_cm,
        pose_x=None,
        pose_y=None,
    ):
        _, front_blocked, _, _ = self.classify_openings(
            front_cm, left_cm, right_cm
        )
        now = time.monotonic()

        left_still_open = (
            left_cm is not None and left_cm >= config.SIDE_OPEN_EXIT_CM
        )
        right_still_open = (
            right_cm is not None and right_cm >= config.SIDE_OPEN_EXIT_CM
        )

        if self.latched:
            normal_corridor = (
                not left_still_open
                and not right_still_open
                and not front_blocked
            )
            self.clear_count = self.clear_count + 1 if normal_corridor else 0

            distance = self._distance_from_latch(pose_x, pose_y)
            elapsed = (
                now - self.latch_time if self.latch_time is not None else None
            )
            moved_minimum = (
                distance is None
                or distance >= config.JUNCTION_REARM_MIN_DISTANCE_M
            )
            released_by_corridor = (
                self.clear_count >= config.JUNCTION_REARM_SAMPLES
                and moved_minimum
            )
            released_by_distance = (
                distance is not None
                and distance >= config.JUNCTION_REARM_DISTANCE_M
                and not left_still_open
                and not right_still_open
            )
            released_by_timeout = (
                elapsed is not None
                and elapsed >= config.JUNCTION_REARM_TIMEOUT_SEC
                and distance is not None
                and distance >= config.JUNCTION_REARM_MIN_DISTANCE_M
                and not left_still_open
                and not right_still_open
            )
            released_by_emergency_front = (
                front_blocked
                and elapsed is not None
                and elapsed >= config.JUNCTION_REARM_EMERGENCY_SEC
            )
            if not (
                released_by_corridor
                or released_by_distance
                or released_by_timeout
                or released_by_emergency_front
            ):
                return False
            self._release_latch()

        if front_blocked:
            self.front_candidate_count += 1
        else:
            self.front_candidate_count = 0

        if not getattr(config, "ENABLE_OPENING_ZONE_DETECTION", True):
            left_open = left_cm is not None and left_cm >= config.SIDE_OPEN_ENTER_CM
            right_open = right_cm is not None and right_cm >= config.SIDE_OPEN_ENTER_CM
            if self.front_candidate_count >= config.JUNCTION_CONFIRM_SAMPLES or left_open or right_open:
                self.pending_zone_event = None
                self._latch_now(now, pose_x, pose_y)
                return True
            return False

        completed = []
        left_event = self._track_side_zone(
            self.left_zone, left_cm, pose_x, pose_y, "LEFT"
        )
        if left_event is not None:
            completed.append(left_event)
        right_event = self._track_side_zone(
            self.right_zone, right_cm, pose_x, pose_y, "RIGHT"
        )
        if right_event is not None:
            completed.append(right_event)

        # Start/continue the V6 window as soon as at least one side opening has
        # been confirmed.  Completed side mouths do not immediately trigger a
        # decision; the short look-ahead may reveal the opposite-side branch.
        if getattr(config, "ENABLE_INTERSECTION_WINDOW", True):
            self._start_intersection_window(pose_x, pose_y)
            if self.intersection_window["active"]:
                for item in completed:
                    self.intersection_window["completed_sides"].add(item["side"])
                self._accumulate_intersection(
                    front_cm, left_cm, right_cm, pose_x, pose_y
                )

                # If the front becomes a confirmed hard stop while we are
                # already inside an intersection, finalize now rather than
                # discarding the side-opening evidence.
                if self.front_candidate_count >= config.JUNCTION_CONFIRM_SAMPLES:
                    event = self._finalize_intersection_window(
                        pose_x, pose_y, "FRONT_BLOCKED"
                    )
                    self.pending_zone_event = event
                    self._latch_now(now, pose_x, pose_y)
                    return True

                should_finish, reason = self._intersection_should_finalize(
                    pose_x, pose_y
                )
                if should_finish:
                    event = self._finalize_intersection_window(
                        pose_x, pose_y, reason
                    )
                    self.pending_zone_event = event
                    self._latch_now(now, pose_x, pose_y)
                    return True

                return False

        # No side intersection is active: front dead-end/corner safety retains
        # priority and triggers the normal stopped scan immediately.
        if self.front_candidate_count >= config.JUNCTION_CONFIRM_SAMPLES:
            self.pending_zone_event = None
            self._latch_now(now, pose_x, pose_y)
            return True

        # Compatibility path if V6 intersection windows are disabled.
        if completed:
            event = max(completed, key=lambda item: item["length_m"])
            event["all_sides"] = [item["side"] for item in completed]
            self.pending_zone_event = event
            self._latch_now(now, pose_x, pose_y)
            return True

        return False

    def force_latched(self, pose_x=None, pose_y=None):
        self._reset_zones()
        self._latch_now(time.monotonic(), pose_x, pose_y)

    def cancel_event(self):
        self._release_latch()


class TremauxExplorer:
    """Frontier-aware DFS explorer with a persistent topological graph.

    Important change from the old local Trémaux rule:
    - visits == 0 is a *frontier* (observed but never traversed).
    - If the current node has no actionable frontier, the robot routes through
      already-known corridors back to a node that still has a frontier.
    - Exploration is COMPLETE only when there are no active frontiers anywhere
      in the graph, not merely because the current node has been crossed twice.

    Existing corridors may therefore be traversed more than twice while they
    are being used as transit routes.  visits is kept as a traversal counter,
    not as a hard two-pass limit.
    """

    def __init__(self):
        self.nodes = {}
        self.next_node_index = 0

        # Internal compass. Initial robot heading is defined as N.
        self.heading_index = 0

        self.start_node_id = None
        self.current_node_id = None
        self.root_decision_node_id = None
        self.root_entry_abs_dir = None

        # Pending edge = corridor currently being travelled.
        self.pending_from_node = None
        self.pending_abs_dir = None

        self.route_history = []
        self.dfs_stack = []
        self.completed = False

        # Diagnostics for graph-integrity protection / skipped-node recovery.
        self.graph_events = []

        # V4 anti-oscillation memory. Key = (frontier signature, node, abs dir).
        # It only blocks a repeated ROUTE_* choice while no frontier progress
        # has happened; discovering/consuming a frontier naturally changes the
        # signature and releases the edge again.
        self.route_attempt_counts = {}

    # ========================================================
    # HEADING HELPERS
    # ========================================================

    def heading_name(self, index=None):
        if index is None:
            index = self.heading_index
        return HEADINGS[index % 4]

    def absolute_index(self, relative_direction):
        return (
            self.heading_index
            + RELATIVE_OFFSET[relative_direction]
        ) % 4

    @staticmethod
    def opposite_index(abs_index):
        return (abs_index + 2) % 4

    def relative_for_absolute(self, abs_index):
        diff = (abs_index - self.heading_index) % 4
        return {
            0: "FRONT",
            1: "RIGHT",
            2: "BACK",
            3: "LEFT",
        }[diff]

    # ========================================================
    # NODE / EDGE MEMORY
    # ========================================================

    def _create_node(self, x, y):
        node_id = f"J{self.next_node_index}"
        self.next_node_index += 1
        self.nodes[node_id] = MazeNode(
            node_id=node_id,
            x=float(x),
            y=float(y),
        )
        return node_id

    def _find_nearby_node(self, x, y):
        best_id = None
        best_distance = None

        for node_id, node in self.nodes.items():
            distance = math.hypot(x - node.x, y - node.y)
            if distance <= config.NODE_MATCH_RADIUS_M:
                if best_distance is None or distance < best_distance:
                    best_id = node_id
                    best_distance = distance
        return best_id

    def _touch_node_position(self, node_id, x, y):
        node = self.nodes[node_id]
        node.seen_count += 1
        alpha = config.NODE_POSITION_UPDATE_ALPHA
        node.x = (1.0 - alpha) * node.x + alpha * float(x)
        node.y = (1.0 - alpha) * node.y + alpha * float(y)

    def _expected_arrival_node(self, x, y):
        """Prefer the known target of the corridor currently being traversed.

        On a revisit, odometry can drift enough that the normal generic radius
        creates a duplicate node at the same physical junction.  If the edge we
        are travelling already has a known target, allow a slightly larger
        target-specific radius.  This is safer than globally increasing the
        node-match radius because only the expected graph neighbor is eligible.
        """
        if self.pending_from_node is None or self.pending_abs_dir is None:
            return None
        if self.pending_from_node not in self.nodes:
            return None

        state = self.nodes[self.pending_from_node].exits.get(self.pending_abs_dir % 4)
        if state is None or state.target is None or state.target not in self.nodes:
            return None

        target = self.nodes[state.target]
        distance = math.hypot(float(x) - target.x, float(y) - target.y)
        radius = float(getattr(config, "EXPECTED_TARGET_MATCH_RADIUS_M", config.NODE_MATCH_RADIUS_M))
        if distance <= radius:
            return state.target
        return None

    def _get_or_create_node(self, x, y):
        expected_id = self._expected_arrival_node(x, y)
        if expected_id is not None:
            self._touch_node_position(expected_id, x, y)
            return expected_id, False

        node_id = self._find_nearby_node(x, y)
        is_new = node_id is None

        if is_new:
            node_id = self._create_node(x, y)

        self._touch_node_position(node_id, x, y)
        return node_id, is_new

    def _exit(self, node_id, abs_index):
        node = self.nodes[node_id]
        abs_index %= 4
        if abs_index not in node.exits:
            node.exits[abs_index] = ExitState()
        return node.exits[abs_index]

    def _record_graph_event(self, kind, **payload):
        event = {"time": time.time(), "kind": kind}
        event.update(payload)
        self.graph_events.append(event)

    def _connect_direct(self, from_node_id, abs_index, to_node_id):
        """Connect one corridor without overwriting an existing different edge."""
        if from_node_id == to_node_id:
            return False

        abs_index %= 4
        opposite = self.opposite_index(abs_index)
        source_exit = self._exit(from_node_id, abs_index)
        target_exit = self._exit(to_node_id, opposite)

        if source_exit.target not in (None, to_node_id):
            self._record_graph_event(
                "SOURCE_EDGE_CONFLICT_PROTECTED",
                from_node=from_node_id,
                heading=self.heading_name(abs_index),
                existing_target=source_exit.target,
                attempted_target=to_node_id,
            )
            return False

        if target_exit.target not in (None, from_node_id):
            self._record_graph_event(
                "TARGET_EDGE_CONFLICT_PROTECTED",
                to_node=to_node_id,
                heading=self.heading_name(opposite),
                existing_target=target_exit.target,
                attempted_target=from_node_id,
            )
            return False

        source_exit.target = to_node_id
        target_exit.target = from_node_id

        shared_visits = max(source_exit.visits, target_exit.visits)
        source_exit.visits = shared_visits
        target_exit.visits = shared_visits
        source_exit.blocked = False
        target_exit.blocked = False
        return True

    def _follow_same_heading_chain(self, start_node_id, abs_index, max_hops=64):
        """Follow already-known nodes in one absolute heading until chain ends."""
        chain = [start_node_id]
        current = start_node_id
        seen = {start_node_id}

        for _ in range(max_hops):
            state = self.nodes[current].exits.get(abs_index % 4)
            if state is None or state.target is None:
                break
            nxt = state.target
            if nxt in seen or nxt not in self.nodes:
                break
            chain.append(nxt)
            seen.add(nxt)
            current = nxt
        return chain

    def _increment_chain_after_first_edge(self, chain, abs_index):
        """A skipped intermediate decision was physically crossed this run.

        The first edge was already incremented by commit_decision() when the
        robot left pending_from_node.  Increment only later chain edges here.
        """
        if len(chain) <= 2:
            return
        for node_id in chain[1:-1]:
            self._increment_departure(node_id, abs_index)

    def _link_nodes(self, from_node_id, abs_index, to_node_id):
        """Link arrival while preserving known intermediate decision nodes.

        This fixes a failure mode where one pass detects J16 between J15/J17,
        but a later pass skips J16 and used to overwrite J15->J16 with J15->J17.
        """
        if from_node_id == to_node_id:
            return

        abs_index %= 4
        source_exit = self._exit(from_node_id, abs_index)

        # Normal first connection or already-direct connection.
        if source_exit.target in (None, to_node_id):
            self._connect_direct(from_node_id, abs_index, to_node_id)
            return

        # A known node already lies ahead in the same direction.  Follow the
        # chain rather than overwriting it.
        chain = self._follow_same_heading_chain(from_node_id, abs_index)
        if to_node_id in chain:
            cut = chain[:chain.index(to_node_id) + 1]
            self._increment_chain_after_first_edge(cut, abs_index)
            self._record_graph_event(
                "SKIPPED_NODE_CHAIN_REUSED",
                from_node=from_node_id,
                to_node=to_node_id,
                heading=self.heading_name(abs_index),
                chain=cut,
            )
            return

        # If the chain ends at an intermediate node with an empty continuation,
        # extend from that tail to the actual arrival node.
        tail = chain[-1]
        tail_exit = self._exit(tail, abs_index)
        if tail_exit.target is None and tail != from_node_id:
            # Existing edges after the first one were physically traversed.
            if len(chain) > 1:
                for node_id in chain[1:]:
                    # Do not increment an empty tail edge yet.
                    state = self._exit(node_id, abs_index)
                    if state.target is not None:
                        self._increment_departure(node_id, abs_index)
            if self._connect_direct(tail, abs_index, to_node_id):
                # Mark the newly extended tail->arrival edge as traversed once.
                self._increment_departure(tail, abs_index)
                self._record_graph_event(
                    "SKIPPED_NODE_CHAIN_EXTENDED",
                    from_node=from_node_id,
                    via_tail=tail,
                    to_node=to_node_id,
                    heading=self.heading_name(abs_index),
                    chain=chain + [to_node_id],
                )
                return

        # Protect the existing topology if the new observation is inconsistent.
        self._record_graph_event(
            "DIRECT_LINK_REJECTED_TO_PROTECT_GRAPH",
            from_node=from_node_id,
            to_node=to_node_id,
            heading=self.heading_name(abs_index),
            existing_target=source_exit.target,
            chain=chain,
        )

    def _split_known_edge_with_intermediate(self, from_node_id, abs_index, new_node_id):
        """Insert a newly detected junction into an already-known corridor.

        Example: an earlier pass stored A -- B directly because the side branch
        at X was missed. A later pass detects X before reaching B. The correct
        topology is A -- X -- B; rejecting X leaves the graph inconsistent and
        can make frontier routing oscillate forever.
        """
        if not bool(getattr(config, "ENABLE_INTERMEDIATE_NODE_EDGE_SPLIT", True)):
            return False
        if from_node_id not in self.nodes or new_node_id not in self.nodes:
            return False
        if from_node_id == new_node_id:
            return False

        abs_index %= 4
        opposite = self.opposite_index(abs_index)
        source_exit = self._exit(from_node_id, abs_index)
        old_target_id = source_exit.target
        if old_target_id is None or old_target_id == new_node_id:
            return False
        if old_target_id not in self.nodes:
            return False

        old_target_back = self._exit(old_target_id, opposite)
        # Split only a mutually confirmed direct edge A<->B.
        if old_target_back.target != from_node_id:
            return False

        new_back = self._exit(new_node_id, opposite)
        new_forward = self._exit(new_node_id, abs_index)
        # A fresh intermediate node must not already contradict this corridor.
        if new_back.target not in (None, from_node_id):
            return False
        if new_forward.target not in (None, old_target_id):
            return False

        inherited_visits = max(source_exit.visits, old_target_back.visits)

        # A <-> X
        source_exit.target = new_node_id
        new_back.target = from_node_id
        # X <-> B
        new_forward.target = old_target_id
        old_target_back.target = new_node_id

        for state in (source_exit, new_back, new_forward, old_target_back):
            state.visits = max(state.visits, inherited_visits)
            state.blocked = False
            state.seen_open_count = max(state.seen_open_count, 1)
            state.miss_count = 0

        self._record_graph_event(
            "INTERMEDIATE_NODE_EDGE_SPLIT",
            from_node=from_node_id,
            inserted_node=new_node_id,
            old_target=old_target_id,
            heading=self.heading_name(abs_index),
            inherited_visits=inherited_visits,
        )
        return True

    def _increment_departure(self, node_id, abs_index):
        exit_state = self._exit(node_id, abs_index)
        target_id = exit_state.target

        if target_id is None:
            exit_state.visits += 1
            exit_state.blocked = False
            return exit_state.visits

        opposite = self.opposite_index(abs_index)
        target_exit = self._exit(target_id, opposite)
        new_visits = max(exit_state.visits, target_exit.visits) + 1
        exit_state.visits = new_visits
        target_exit.visits = new_visits
        exit_state.blocked = False
        target_exit.blocked = False
        return new_visits

    # ========================================================
    # START / ARRIVAL
    # ========================================================

    def initialize_start(self, x, y):
        if self.start_node_id is not None:
            return self.start_node_id

        node_id = self._create_node(x, y)
        self.nodes[node_id].seen_count = 1
        self.start_node_id = node_id
        self.current_node_id = node_id
        self.dfs_stack = [node_id]
        return node_id

    def commit_initial_forward(self):
        if self.current_node_id is None:
            raise RuntimeError("initialize_start() must be called first")

        abs_index = self.heading_index
        self._increment_departure(self.current_node_id, abs_index)
        self.pending_from_node = self.current_node_id
        self.pending_abs_dir = abs_index
        self.route_history.append({
            "time": time.time(),
            "node": self.current_node_id,
            "direction": "FRONT",
            "heading": self.heading_name(abs_index),
            "kind": "initial_departure",
        })

    def arrive_at_decision_point(self, x, y):
        node_id, is_new = self._get_or_create_node(x, y)

        previous_node = self.pending_from_node
        incoming_abs_dir = self.pending_abs_dir

        if (
            previous_node is not None
            and incoming_abs_dir is not None
            and previous_node != node_id
        ):
            split_done = False
            if is_new:
                split_done = self._split_known_edge_with_intermediate(
                    previous_node, incoming_abs_dir, node_id
                )
            if not split_done:
                self._link_nodes(previous_node, incoming_abs_dir, node_id)

        if (
            self.root_decision_node_id is None
            and previous_node == self.start_node_id
            and incoming_abs_dir is not None
        ):
            self.root_decision_node_id = node_id
            self.root_entry_abs_dir = self.opposite_index(incoming_abs_dir)

        self.current_node_id = node_id

        # V4 traversal stack: preserve the ACTUAL route through graph loops.
        # The old code collapsed the stack back to an earlier occurrence of a
        # known node. In the J12 case that discarded J12 even though J12 still
        # owned an unexplored branch, forcing global BFS and causing oscillation.
        if not self.dfs_stack:
            self.dfs_stack.append(node_id)
        elif self.dfs_stack[-1] != node_id:
            if len(self.dfs_stack) >= 2 and self.dfs_stack[-2] == node_id:
                # Normal one-edge DFS backtrack.
                self.dfs_stack.pop()
            else:
                if node_id in self.dfs_stack:
                    self._record_graph_event(
                        "LOOP_REVISIT_STACK_PRESERVED",
                        node=node_id,
                        previous_node=previous_node,
                    )
                self.dfs_stack.append(node_id)
                max_len = int(getattr(config, "DFS_STACK_MAX_LEN", 128))
                if max_len > 0 and len(self.dfs_stack) > max_len:
                    self.dfs_stack = self.dfs_stack[-max_len:]

        self.pending_from_node = None
        self.pending_abs_dir = None
        return node_id, is_new

    # ========================================================
    # FRONTIERS / GRAPH ROUTING
    # ========================================================

    def _physical_candidates(self, front_open, left_open, right_open, allow_back=True):
        relative_candidates = []
        if front_open:
            relative_candidates.append("FRONT")
        if left_open:
            relative_candidates.append("LEFT")
        if right_open:
            relative_candidates.append("RIGHT")
        if allow_back:
            relative_candidates.append("BACK")

        result = []
        seen = set()
        for relative in relative_candidates:
            abs_index = self.absolute_index(relative)
            if abs_index in seen:
                continue
            seen.add(abs_index)
            result.append((relative, abs_index))
        return result

    def _update_frontier_observations(self, candidates):
        node = self.nodes[self.current_node_id]
        observed_abs = {abs_index for _, abs_index in candidates}

        # Register / refresh every physically observed opening.
        for _, abs_index in candidates:
            state = self._exit(self.current_node_id, abs_index)
            state.seen_open_count += 1
            state.miss_count = 0
            state.blocked = False

        # Only untraversed, unlinked exits can be stale frontiers.  Known graph
        # corridors are never retired because one stopped scan failed to see it.
        retire_enabled = bool(getattr(config, "ENABLE_STALE_FRONTIER_RETIRE", True))
        miss_limit = int(getattr(config, "FRONTIER_STALE_MISS_LIMIT", 3))
        if not retire_enabled or miss_limit <= 0:
            return

        for abs_index, state in list(node.exits.items()):
            if abs_index in observed_abs:
                continue
            if state.visits != 0 or state.target is not None or state.blocked:
                continue
            state.miss_count += 1
            if state.miss_count >= miss_limit:
                state.blocked = True
                self._record_graph_event(
                    "STALE_FRONTIER_RETIRED",
                    node=self.current_node_id,
                    heading=self.heading_name(abs_index),
                    misses=state.miss_count,
                )

    @staticmethod
    def _is_frontier_state(state):
        return (
            state.visits == 0
            and state.target is None
            and not state.blocked
        )

    def frontier_exits(self, node_id=None):
        if node_id is None:
            node_id = self.current_node_id
        if node_id is None or node_id not in self.nodes:
            return []
        result = []
        for abs_index, state in self.nodes[node_id].exits.items():
            if self._is_frontier_state(state):
                result.append(abs_index % 4)
        return sorted(result)

    def all_frontiers(self):
        result = []
        for node_id in sorted(self.nodes, key=lambda n: int(n[1:]) if n[1:].isdigit() else n):
            for abs_index in self.frontier_exits(node_id):
                result.append((node_id, abs_index))
        return result

    def describe_frontiers(self):
        items = self.all_frontiers()
        if not items:
            return "NONE"
        return " | ".join(
            f"{node_id}.{self.heading_name(abs_index)}"
            for node_id, abs_index in items
        )

    def _graph_neighbors(self, node_id):
        neighbors = []
        if node_id not in self.nodes:
            return neighbors
        for abs_index, state in self.nodes[node_id].exits.items():
            if state.target is None or state.target not in self.nodes:
                continue
            neighbors.append((state.target, abs_index % 4))
        return neighbors

    def _shortest_path(self, start_id, target_id, allowed_first_abs=None):
        if start_id == target_id:
            return [start_id]
        if start_id not in self.nodes or target_id not in self.nodes:
            return None

        allowed_first_abs = None if allowed_first_abs is None else set(allowed_first_abs)
        q = deque([[start_id]])
        visited = {start_id}

        while q:
            path = q.popleft()
            node_id = path[-1]
            for nxt, abs_index in self._graph_neighbors(node_id):
                if len(path) == 1 and allowed_first_abs is not None:
                    if abs_index not in allowed_first_abs:
                        continue
                if nxt in visited:
                    continue
                new_path = path + [nxt]
                if nxt == target_id:
                    return new_path
                visited.add(nxt)
                q.append(new_path)
        return None

    def _abs_to_target(self, from_node_id, to_node_id, allowed_abs=None):
        allowed_abs = None if allowed_abs is None else set(allowed_abs)
        for abs_index, state in self.nodes[from_node_id].exits.items():
            if state.target != to_node_id:
                continue
            if allowed_abs is not None and abs_index not in allowed_abs:
                continue
            return abs_index % 4
        return None

    def _preferred_stack_frontier_target(self):
        # Nearest ancestor with an unexplored branch.
        if not self.dfs_stack:
            return None
        for node_id in reversed(self.dfs_stack[:-1]):
            if self.frontier_exits(node_id):
                return node_id
        return None

    def _nearest_reachable_frontier_path(self, allowed_first_abs):
        """BFS until reaching any node that owns an active frontier."""
        start = self.current_node_id
        if start is None:
            return None

        q = deque([[start]])
        visited = {start}
        allowed_first_abs = set(allowed_first_abs)

        while q:
            path = q.popleft()
            node_id = path[-1]
            if node_id != start and self.frontier_exits(node_id):
                return path

            for nxt, abs_index in self._graph_neighbors(node_id):
                if len(path) == 1 and abs_index not in allowed_first_abs:
                    continue
                if nxt in visited:
                    continue
                visited.add(nxt)
                q.append(path + [nxt])
        return None

    def _frontier_signature(self):
        return tuple(self.all_frontiers())

    def _route_attempt_key(self, abs_index, frontier_signature=None):
        if frontier_signature is None:
            frontier_signature = self._frontier_signature()
        return (frontier_signature, self.current_node_id, abs_index % 4)

    def _repeated_route_abs(self, candidates, frontier_signature):
        if not bool(getattr(config, "ENABLE_ROUTE_LOOP_BREAK", True)):
            return set()
        limit = max(1, int(getattr(config, "ROUTE_REPEAT_LIMIT", 1)))
        blocked = set()
        for _, abs_index in candidates:
            key = self._route_attempt_key(abs_index, frontier_signature)
            if self.route_attempt_counts.get(key, 0) >= limit:
                blocked.add(abs_index % 4)
        return blocked

    # ========================================================
    # FRONTIER-AWARE DFS DECISION
    # ========================================================

    def plan_direction(self, front_open, left_open, right_open):
        if self.current_node_id is None:
            raise RuntimeError("No current node. Call arrive_at_decision_point()")

        allow_back = self.current_node_id != self.start_node_id or bool(
            self.nodes[self.current_node_id].exits
        )
        candidates = self._physical_candidates(
            front_open,
            left_open,
            right_open,
            allow_back=allow_back,
        )
        self._update_frontier_observations(candidates)

        preference_rank = {
            name: index
            for index, name in enumerate(config.EXPLORATION_PREFERENCE)
        }
        scored = []
        for relative, abs_index in candidates:
            state = self._exit(self.current_node_id, abs_index)
            scored.append((
                state.visits,
                preference_rank.get(relative, 99),
                relative,
                abs_index,
                state,
            ))
        scored.sort(key=lambda item: (item[0], item[1]))

        # 1) Always take a physically confirmed local frontier first.
        local_unvisited = [
            item for item in scored
            if self._is_frontier_state(item[4])
        ]
        if local_unvisited:
            visits, _, relative, abs_index, _ = local_unvisited[0]
            self.completed = False
            return ExplorationDecision(
                direction=relative,
                node_id=self.current_node_id,
                reason="UNVISITED_EXIT",
                visits_before=visits,
                absolute_heading=self.heading_name(abs_index),
            )

        # 1b) IMPORTANT: a known junction can remember an unexplored side branch
        # even when the current stopped Sharp scan misses that opening by a few
        # centimetres.  The V2 planner accidentally ignored a frontier owned by
        # the *current* node unless it appeared in the current sensor candidates,
        # then immediately routed BACK through a known corridor.  That produced
        # exactly the observed "came out of a dead end, then went back the same
        # way instead of taking the branch I had not tried" behaviour.
        #
        # A remembered frontier was already created from a multi-sample stopped
        # scan on an earlier visit.  For LEFT/RIGHT we can safely turn toward it;
        # after the turn the front ToF still prevents driving into a real wall.
        if bool(getattr(config, "ENABLE_REMEMBERED_LOCAL_FRONTIER", True)):
            observed_abs = {abs_index for _, abs_index in candidates}
            min_seen = int(getattr(config, "REMEMBERED_FRONTIER_MIN_SEEN", 1))
            remembered = []
            for abs_index in self.frontier_exits(self.current_node_id):
                if abs_index in observed_abs:
                    continue
                state = self._exit(self.current_node_id, abs_index)
                if state.seen_open_count < min_seen:
                    continue
                relative = self.relative_for_absolute(abs_index)
                # Never force straight ahead against the current front scan.
                # The problematic missed branch on a revisit is normally LEFT
                # or RIGHT after returning from a dead end.
                if relative not in ("LEFT", "RIGHT"):
                    continue
                remembered.append((
                    preference_rank.get(relative, 99),
                    relative,
                    abs_index,
                    state,
                ))

            if remembered:
                remembered.sort(key=lambda item: item[0])
                _, relative, abs_index, state = remembered[0]
                self.completed = False
                return ExplorationDecision(
                    direction=relative,
                    node_id=self.current_node_id,
                    reason="REMEMBERED_LOCAL_FRONTIER",
                    visits_before=state.visits,
                    absolute_heading=self.heading_name(abs_index),
                )

        global_frontiers = self.all_frontiers()

        # 2) No active frontier anywhere -> true global completion.
        if not global_frontiers:
            self.completed = True
            return ExplorationDecision(
                direction="COMPLETE",
                node_id=self.current_node_id,
                reason="ALL_FRONTIERS_EXPLORED",
                visits_before=0,
                absolute_heading=self.heading_name(),
            )

        frontier_signature = tuple(global_frontiers)
        physically_allowed_abs = {abs_index for _, abs_index in candidates}
        repeated_abs = self._repeated_route_abs(candidates, frontier_signature)
        allowed_first_abs = set(physically_allowed_abs) - repeated_abs

        # If exactly the same ROUTE_* departure has already been tried with the
        # same frontier set and made no progress, do not U-turn through it again.
        # Prefer continuing straight through the known junction when physically
        # open; this is the situation seen in the supplied J3/J9 loop log.
        if repeated_abs and front_open:
            front_abs = self.absolute_index("FRONT")
            if front_abs in allowed_first_abs:
                front_state = self._exit(self.current_node_id, front_abs)
                if front_state.visits > 0 or front_state.target is not None:
                    self.completed = False
                    return ExplorationDecision(
                        direction="FRONT",
                        node_id=self.current_node_id,
                        reason="LOOP_BREAK_CONTINUE_FRONT",
                        visits_before=front_state.visits,
                        absolute_heading=self.heading_name(front_abs),
                    )

        # If loop protection temporarily removed every first step, fall back to
        # the physical candidates; the later recovery stage will still avoid an
        # immediate false COMPLETE.
        if not allowed_first_abs:
            allowed_first_abs = set(physically_allowed_abs)

        # 3) Prefer classic DFS backtracking along the current stack toward the
        # nearest ancestor that still owns a frontier.
        stack_target = self._preferred_stack_frontier_target()
        if stack_target is not None:
            if len(self.dfs_stack) >= 2:
                parent = self.dfs_stack[-2]
                abs_to_parent = self._abs_to_target(
                    self.current_node_id,
                    parent,
                    allowed_abs=allowed_first_abs,
                )
                if abs_to_parent is not None:
                    relative = self.relative_for_absolute(abs_to_parent)
                    visits = self._exit(self.current_node_id, abs_to_parent).visits
                    self.completed = False
                    return ExplorationDecision(
                        direction=relative,
                        node_id=self.current_node_id,
                        reason="DFS_BACKTRACK_TO_FRONTIER",
                        visits_before=visits,
                        absolute_heading=self.heading_name(abs_to_parent),
                    )

            path = self._shortest_path(
                self.current_node_id,
                stack_target,
                allowed_first_abs=allowed_first_abs,
            )
            if path and len(path) >= 2:
                abs_index = self._abs_to_target(
                    self.current_node_id,
                    path[1],
                    allowed_abs=allowed_first_abs,
                )
                if abs_index is not None:
                    relative = self.relative_for_absolute(abs_index)
                    visits = self._exit(self.current_node_id, abs_index).visits
                    self.completed = False
                    return ExplorationDecision(
                        direction=relative,
                        node_id=self.current_node_id,
                        reason="ROUTE_TO_DFS_FRONTIER",
                        visits_before=visits,
                        absolute_heading=self.heading_name(abs_index),
                    )

        # 4) Loops / merged topology can leave a frontier outside the active DFS
        # stack. Route to the nearest reachable frontier node in the graph.
        path = self._nearest_reachable_frontier_path(allowed_first_abs)
        if path and len(path) >= 2:
            abs_index = self._abs_to_target(
                self.current_node_id,
                path[1],
                allowed_abs=allowed_first_abs,
            )
            if abs_index is not None:
                relative = self.relative_for_absolute(abs_index)
                visits = self._exit(self.current_node_id, abs_index).visits
                self.completed = False
                return ExplorationDecision(
                    direction=relative,
                    node_id=self.current_node_id,
                    reason="ROUTE_TO_NEAREST_FRONTIER",
                    visits_before=visits,
                    absolute_heading=self.heading_name(abs_index),
                )

        # 5) Frontiers exist but current graph/sensor snapshot cannot route to
        # one. Do not falsely COMPLETE. Use any physically available known edge
        # as a recovery transit; BACK naturally wins preference when it is the
        # only route from a dead end.
        known_transit = []
        for item in scored:
            visits, rank, relative, abs_index, state = item
            if state.target is not None and abs_index in allowed_first_abs:
                known_transit.append((visits, rank, relative, abs_index, state))
        # If every known transit was loop-blocked, allow them as a final safety
        # fallback rather than falsely declaring completion.
        if not known_transit:
            for item in scored:
                visits, rank, relative, abs_index, state = item
                if state.target is not None:
                    known_transit.append((visits, rank, relative, abs_index, state))
        if known_transit:
            # Prefer BACK for recovery, then lower traversal count / preference.
            known_transit.sort(
                key=lambda item: (
                    0 if item[2] == "BACK" else 1,
                    item[0],
                    item[1],
                )
            )
            visits, _, relative, abs_index, _ = known_transit[0]
            self.completed = False
            return ExplorationDecision(
                direction=relative,
                node_id=self.current_node_id,
                reason="FRONTIER_RECOVERY_TRANSIT",
                visits_before=visits,
                absolute_heading=self.heading_name(abs_index),
            )

        # This is a graph-integrity failure, not successful completion.  Keep the
        # reason explicit so logs distinguish it from ALL_FRONTIERS_EXPLORED.
        self.completed = False
        return ExplorationDecision(
            direction="COMPLETE",
            node_id=self.current_node_id,
            reason="FRONTIERS_EXIST_BUT_UNREACHABLE",
            visits_before=0,
            absolute_heading=self.heading_name(),
        )

    def commit_decision(self, decision):
        if decision.direction == "COMPLETE":
            # Only the explicit global-completion reason is a real completion.
            self.completed = decision.reason == "ALL_FRONTIERS_EXPLORED"
            return

        abs_index = self.absolute_index(decision.direction)

        # V4 anti-oscillation: remember directed graph-routing attempts while
        # the frontier set is unchanged. A second visit to the same node will
        # therefore choose an alternative instead of repeating the same U-turn.
        route_reasons = {
            "DFS_BACKTRACK_TO_FRONTIER",
            "ROUTE_TO_DFS_FRONTIER",
            "ROUTE_TO_NEAREST_FRONTIER",
            "FRONTIER_RECOVERY_TRANSIT",
        }
        if decision.reason in route_reasons:
            signature = self._frontier_signature()
            key = self._route_attempt_key(abs_index, signature)
            self.route_attempt_counts[key] = self.route_attempt_counts.get(key, 0) + 1

        new_visits = self._increment_departure(
            self.current_node_id,
            abs_index,
        )
        self.pending_from_node = self.current_node_id
        self.pending_abs_dir = abs_index
        self.route_history.append({
            "time": time.time(),
            "node": self.current_node_id,
            "direction": decision.direction,
            "absolute_heading": self.heading_name(abs_index),
            "edge_visits": new_visits,
            "reason": decision.reason,
            "frontiers_remaining": len(self.all_frontiers()),
        })
        self.heading_index = abs_index
        self.completed = False

    # ========================================================
    # DEBUG / SAVE
    # ========================================================

    def describe_node(self, node_id=None):
        if node_id is None:
            node_id = self.current_node_id
        if node_id is None or node_id not in self.nodes:
            return "NO_NODE"

        node = self.nodes[node_id]
        parts = []
        for abs_index in range(4):
            if abs_index not in node.exits:
                continue
            exit_state = node.exits[abs_index]
            target = exit_state.target or "?"
            suffix = ""
            if self._is_frontier_state(exit_state):
                suffix = "[FRONTIER]"
            elif exit_state.blocked:
                suffix = "[STALE]"
            parts.append(
                f"{HEADINGS[abs_index]}:{exit_state.visits}->{target}{suffix}"
            )
        return " | ".join(parts) if parts else "NO_EXITS"

    def save_memory(self, filepath=None):
        if filepath is None:
            filepath = config.MAZE_MEMORY_FILE

        data = {
            "start_node_id": self.start_node_id,
            "root_decision_node_id": self.root_decision_node_id,
            "root_entry_heading": (
                self.heading_name(self.root_entry_abs_dir)
                if self.root_entry_abs_dir is not None
                else None
            ),
            "current_node_id": self.current_node_id,
            "heading": self.heading_name(),
            "completed": self.completed,
            "frontiers": [
                {"node": node_id, "heading": self.heading_name(abs_index)}
                for node_id, abs_index in self.all_frontiers()
            ],
            "dfs_stack": list(self.dfs_stack),
            "nodes": {},
            "route_history": self.route_history,
            "graph_events": self.graph_events,
        }

        for node_id, node in self.nodes.items():
            data["nodes"][node_id] = {
                "x": node.x,
                "y": node.y,
                "seen_count": node.seen_count,
                "exits": {
                    HEADINGS[int(abs_index)]: {
                        "visits": exit_state.visits,
                        "target": exit_state.target,
                        "seen_open_count": exit_state.seen_open_count,
                        "miss_count": exit_state.miss_count,
                        "blocked": exit_state.blocked,
                        "frontier": self._is_frontier_state(exit_state),
                    }
                    for abs_index, exit_state in node.exits.items()
                },
            }

        with open(filepath, "w", encoding="utf-8") as file:
            json.dump(data, file, indent=2, ensure_ascii=False)

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

    V7 intentionally avoids chassis.move(...).wait_for_completed() for normal
    turns because a lost/rejected action-completion ACK can block the program.
    The turn is instead finished from real attitude feedback and always has a
    hard watchdog timeout.
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

    # Preserve the already field-verified logical turn convention:
    # RIGHT command=-90 with move_sign=-1 => attitude target +90 deg.
    target_yaw = normalize_angle_deg(start_yaw + command_deg * move_sign)

    timeout_sec = (
        config.TURN_FEEDBACK_TIMEOUT_180_SEC
        if abs(command_deg) > 135.0
        else config.TURN_FEEDBACK_TIMEOUT_90_SEC
    )

    print(
        f">>> TURN {decision.name} [FEEDBACK]: command={command_deg:+.1f} deg "
        f"start_yaw={start_yaw:+.1f} target={target_yaw:+.1f}"
    )

    _safe_stop(chassis)
    time.sleep(config.TURN_PRE_SETTLE_SEC)

    started = time.monotonic()
    stable_samples = 0
    last_print = 0.0

    try:
        while True:
            now = time.monotonic()
            current_yaw = pose_tracker.get_yaw()

            if current_yaw is None:
                if now - started >= timeout_sec:
                    _safe_stop(chassis)
                    print("TURN FAILED: attitude yaw unavailable until watchdog timeout.")
                    return False
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
                        f"err={error:+.1f} z={z_cmd:+.1f}"
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
                print(
                    "TURN WATCHDOG TIMEOUT: "
                    + (
                        f"yaw={final_yaw:+.1f} target={target_yaw:+.1f} "
                        f"error={final_error:+.1f} deg"
                        if final_yaw is not None
                        else "yaw unavailable"
                    )
                )
                return False

            time.sleep(config.TURN_FEEDBACK_LOOP_SEC)

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

    Returns True on success, False on a safely-aborted turn.
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

# ==================== MAIN WALK TEST ====================
"""Main entry point for the RoboMaster Trémaux / DFS maze explorer."""

import statistics
import time




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
    print(" MAZE SOLVER V6 - INTERSECTION WINDOW + FRONTIER DFS")
    print("==========================================================")
    print()
    print(f"Program version     : {config.PROGRAM_VERSION}")
    print(f"Forward speed       : {config.FORWARD_SPEED:.2f} m/s")
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
    if config.SIDE_OPEN_ENTER_CM < 15.0:
        print("*** WARNING: SIDE OPEN threshold is suspiciously low (<15 cm). ***")
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
        max(0.0, float(requested_backtrack)),
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
"""Configuration for the RoboMaster maze solver.

ปรับค่าจูนของหุ่นทั้งหมดจากไฟล์นี้ไฟล์เดียว
"""

# ============================================================
# GENERAL
# ============================================================

ENABLE_MOTION = True


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

FORWARD_SPEED = 0.15
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
JUNCTION_CREEP_SEC = 0.50
JUNCTION_CREEP_ABORT_FRONT_CM = 16.0
JUNCTION_CREEP_LOOP_SEC = 0.05

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
DECISION_SCAN_SAMPLES = 3
DECISION_SCAN_INTERVAL_SEC = 0.04


# ============================================================
# TRÉMAUX / DFS EXPLORATION
# ============================================================

# จาก log สนามจริง Sharp ด้านที่เปิดขึ้นได้ประมาณ 20-24 cm เท่านั้น
# จึงใช้ 20 cm เป็นเกณฑ์เปิด และยืนยันหลาย sample เพื่อลด false positive
EXPLORATION_SIDE_OPEN_CM = 20.0

# ระยะหน้า 17-24 cm ยังใกล้กำแพงเกินไปที่จะถือว่าเป็นทางตรง
# ใช้ 35 cm จาก log สนามจริง: กำแพงหน้า ~27 cm ที่ T-junction ต้องเป็น BLOCK
EXPLORATION_FRONT_OPEN_CM = 35.0

# Number of consecutive samples needed before declaring a decision point.
JUNCTION_CONFIRM_SAMPLES = 3

# Samples of normal corridor needed before the same detector can trigger again.
JUNCTION_REARM_SAMPLES = 4

# Odometry radius used to decide that a junction is one already seen before.
NODE_MATCH_RADIUS_M = 0.18

# Small position averaging when the same junction is re-observed.
NODE_POSITION_UPDATE_ALPHA = 0.20

# Trémaux normally considers an edge fully covered after two traversals.
MAX_EDGE_VISITS = 2

# Tie-breaker ONLY when paths have the same mark count.
# Unvisited paths always beat visited paths regardless of this order.
EXPLORATION_PREFERENCE = ("FRONT", "LEFT", "RIGHT", "BACK")

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

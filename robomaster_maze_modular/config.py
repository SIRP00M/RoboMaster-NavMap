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

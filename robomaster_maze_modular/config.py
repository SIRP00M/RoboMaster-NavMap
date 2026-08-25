"""Central configuration for RoboMaster Maze Explorer V10.1.

All tunable constants live here. Other modules import this module as `config`,
so changing a value here is immediately used everywhere on the next run.
"""

# ==================== CONFIG ====================
"""Configuration for the RoboMaster maze solver.

ปรับค่าจูนของหุ่นทั้งหมดจากไฟล์นี้ไฟล์เดียว
"""

# ============================================================
# GENERAL
# ============================================================

ENABLE_MOTION = True
PROGRAM_VERSION = "V10.1_SENSOR_TIMEOUT_GUARD_HEADING_ENTRY_REALIGN"


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

# V10.1: synchronous Sensor Adapter ADC reads can occasionally time out and
# return None. Never feed None into the median filter. A single timeout may
# reuse the most recent valid Sharp value briefly; a prolonged outage makes
# the main loop stop and wait for the sensors to recover.
SHARP_STALE_HOLD_SEC = 1.50
SHARP_INVALID_WARN_INTERVAL_SEC = 1.00
SHARP_SENSOR_RECOVERY_DELAY_SEC = 0.08


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

# V10: corridor-referenced self calibration.  The IMU yaw can drift slowly
# during a long run.  In a straight, wall-bounded corridor the change in
# Sharp wall distance gives an external estimate of chassis skew.  Apply only
# a small fraction to the absolute yaw grid so noise/junction edges cannot
# jerk the heading reference.
ENABLE_CORRIDOR_HEADING_CALIBRATION = True
CORRIDOR_CAL_MIN_TRAVEL_M = 0.18
CORRIDOR_CAL_MIN_WALL_CM = 5.8
CORRIDOR_CAL_MAX_WALL_CM = 16.0
CORRIDOR_CAL_MAX_ESTIMATE_DEG = 7.0
CORRIDOR_CAL_ALPHA = 0.12
CORRIDOR_CAL_MAX_STEP_DEG = 0.65
CORRIDOR_CAL_MIN_FRONT_CM = 40.0
CORRIDOR_CAL_LOG_MIN_STEP_DEG = 0.12


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
OPENING_ZONE_CENTERING_MAX_BACKTRACK_M = 0.48
OPENING_ZONE_CENTERING_MAX_SEC = 7.0
OPENING_ZONE_CENTERING_LOOP_SEC = 0.04
# The detector starts only after several OPEN samples, so its measured near
# edge is a little late.  Bias the return slightly farther backward.
OPENING_ZONE_CENTER_REVERSE_BIAS_M = 0.035

# V10: before a LEFT/RIGHT rotation, the chosen side must be physically open
# at the robot's current pivot.  If accumulated Intersection Window memory says
# a branch exists but the stopped Sharp scan no longer sees it, search backward
# a short distance before turning.  This prevents rotating after overshooting
# the mouth and sweeping the chassis into the wall.
ENABLE_TURN_ENTRY_REALIGN = True
TURN_ENTRY_OPEN_CM = 17.5
TURN_ENTRY_CONFIRM_SAMPLES = 2
TURN_ENTRY_SEARCH_SPEED = 0.045
TURN_ENTRY_MAX_BACKTRACK_M = 0.13
TURN_ENTRY_MAX_SEC = 3.0
TURN_ENTRY_FRONT_SAFE_CM = 11.5
TURN_ENTRY_LOOP_SEC = 0.04

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


# ============================================================
# OPEN AREA + EXIT DETECTION
# ============================================================

# Large open spaces need different lateral behaviour from a corridor.
# When BOTH Sharp sensors stay far from walls and the front is traversable,
# latch OPEN_AREA and let attitude heading-hold keep the robot straight.
ENABLE_OPEN_AREA_HEADING_HOLD = True
OPEN_AREA_SIDE_ENTER_CM = 35.0
OPEN_AREA_SIDE_EXIT_CM = 28.0
OPEN_AREA_FRONT_MIN_CM = 45.0
OPEN_AREA_ENTER_SAMPLES = 4
OPEN_AREA_EXIT_SAMPLES = 3

# Exit detection is intentionally conservative. A single wide junction must
# never become EXIT. The robot first enters open area, then all three measured
# directions must be very open for several samples, and the robot must travel
# forward through that open region before EXIT is confirmed.
ENABLE_EXIT_DETECTION = True
STOP_WHEN_EXIT_FOUND = True
EXIT_FRONT_START_CM = 165.0
EXIT_FRONT_KEEP_CM = 130.0
EXIT_SIDE_START_CM = 55.0
EXIT_SIDE_KEEP_CM = 42.0
EXIT_START_SAMPLES = 4
EXIT_CONFIRM_STRONG_SAMPLES = 8
EXIT_CONFIRM_DISTANCE_M = 0.60
EXIT_CONFIRM_MIN_SEC = 1.80
EXIT_MIN_RUNTIME_SEC = 5.0
EXIT_MIN_NODE_COUNT = 2
EXIT_MAX_HEADING_ERROR_DEG = 8.0
# Slow down while proving an exit so there is room to stop outside the maze.
EXIT_CANDIDATE_SPEED = 0.12


# ============================================================
# START-GATE GUARD (V7)
# ============================================================
# The entrance is physically open just like a true exit. V7 treats the direction
# behind the initial pose as a virtual wall. This protection exists at THREE
# levels: planner filtering, exit-candidate rejection, and a geometric emergency
# recovery if the robot gets close to crossing the start line while facing out.
ENABLE_START_GATE_GUARD = True

# Learn the physical inward vector from the robot's first straight displacement.
# This avoids assuming how RoboMaster raw x/y axes are oriented.
START_GATE_LEARN_DISTANCE_M = 0.12

# Geometric safety gate around the starting opening.
START_GATE_HALF_WIDTH_M = 0.45
START_GATE_BLOCK_INNER_M = 0.20
START_GATE_RECOVERY_COOLDOWN_SEC = 1.0

# An EXIT candidate is forbidden near the start even if ToF/Sharp all look open.
START_EXIT_REJECT_RADIUS_M = 0.90
START_EXIT_REJECT_INNER_PROGRESS_M = 0.45
START_EXIT_REJECT_LATERAL_M = 0.65


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
# set of remaining exploration obligations has not changed. This breaks
# J3<->...<->J9 style oscillation without globally preferring FRONT over correct
# DFS backtracking.
ENABLE_ROUTE_LOOP_BREAK = True
ROUTE_REPEAT_LIMIT = 1

# V8 unresolved-edge recovery.
# A physically open exit with visits>0 but target=None means the robot has used
# that direction before but the graph never linked it to a destination node.
# This is NOT a completed corridor. Prefer it locally before routing around the
# graph, and also treat its owner as a pending exploration target globally.
ENABLE_UNRESOLVED_EDGE_RECOVERY = True
# One initial traversal + at most one recovery traversal is normally enough.
# More repeated retries can make a cyclic maze look like the robot is "orbiting"
# an island instead of heading to a remaining frontier.
UNRESOLVED_EDGE_MAX_VISITS = 2

# V10 cyclic-maze routing.
# Local unvisited exits still win immediately. When the current junction has no
# local work, route to the nearest pending junction using a weighted graph cost
# that strongly penalizes already-repeated edges. This keeps Trémaux behaviour
# but avoids blindly replaying a long DFS stack around an island.
ENABLE_WEIGHTED_PENDING_ROUTING = True
ROUTE_EDGE_BASE_COST = 1.0
ROUTE_EDGE_VISIT_PENALTY = 1.75
ROUTE_EDGE_HIGH_VISIT_EXTRA = 2.0
ROUTE_PENDING_UNRESOLVED_EXTRA = 1.25

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
# SLAM-STYLE MAPPING (PASSIVE - DOES NOT CONTROL THE ROBOT)
# ============================================================
ENABLE_MAPPING = True
MAP_OUTPUT_DIR = "mapping_output"
MAP_CLEAR_OUTPUT_ON_START = True

# Raw RoboMaster odometry -> map frame.
# From the real logs used in this project: logical EAST ~= -raw X,
# logical NORTH ~= +raw Y. Change only these if the exported map is mirrored.
MAP_SWAP_RAW_XY = False
MAP_RAW_X_SIGN = -1.0
MAP_RAW_Y_SIGN = +1.0
MAP_POSITION_ROTATION_DEG = 0.0
# V10: learn how RoboMaster raw X/Y are rotated relative to the maze from the
# first straight run, then rotate the entire map so the initial corridor is +Y.
MAP_AUTO_ALIGN_INITIAL_PATH = True
MAP_AUTO_ALIGN_MIN_TRAVEL_M = 0.18
MAP_AUTO_ALIGN_MAX_HEADING_INDEX = 0
MAP_YAW_RIGHT_SIGN = +1.0
MAP_YAW_CARDINAL_MAX_ERROR_DEG = 22.0
MAP_FALLBACK_TO_CARDINAL_HEADING = True
# Mapping rays use the logical N/E/S/W heading rather than ±1-3 deg of attitude
# noise, so straight foam walls render as straight walls.
MAP_SENSOR_USE_CARDINAL_HEADING = True
MAP_MIN_RECORD_INTERVAL_SEC = 0.045
MAP_MAX_SAMPLES = 60000

# Occupancy grid. 2.5 cm gives a cleaner SLAM-like wall shape than the old 5 cm grid.
MAP_RESOLUTION_M = 0.025
MAP_EVIDENCE_MIN = -30
MAP_EVIDENCE_MAX = +30
MAP_OCCUPIED_SCORE_THRESHOLD = 4
MAP_FREE_SCORE_THRESHOLD = -3
MAP_ROBOT_FREE_RADIUS_M = 0.11
MAP_ROBOT_FREE_SCORE = -3

# Sensor mounting model. Measure these offsets later for best geometry.
# local coordinates: +forward = robot front, +right = robot right.
MAP_FRONT_SENSOR_ANGLE_DEG = 0.0
MAP_LEFT_SENSOR_ANGLE_DEG = -90.0
MAP_RIGHT_SENSOR_ANGLE_DEG = +90.0
MAP_FRONT_SENSOR_FORWARD_M = 0.08
MAP_FRONT_SENSOR_RIGHT_M = 0.0
MAP_LEFT_SENSOR_FORWARD_M = 0.02
MAP_LEFT_SENSOR_RIGHT_M = -0.10
MAP_RIGHT_SENSOR_FORWARD_M = 0.02
MAP_RIGHT_SENSOR_RIGHT_M = +0.10

# Conservative ranges are intentional. Long ToF rays created triangular/diagonal
# artefacts in earlier maps when odometry/yaw drifted. ToF now maps only nearby
# front space/walls; Sharp is the primary side-wall source.
MAP_TOF_MIN_CM = 4.0
MAP_TOF_FREE_MAX_CM = 55.0
MAP_TOF_OCCUPIED_MAX_CM = 45.0
MAP_TOF_HIT_SCORE = 7
MAP_TOF_FREE_SCORE = -1
# A far ToF reading is useful for motion but makes ugly white "combs" when used
# as a SLAM free-space ray. With no confirmed front hit, clear only a short
# region in front of the robot; the swept robot footprint supplies the rest.
MAP_TOF_NO_HIT_FREE_MAX_CM = 28.0

MAP_SHARP_MIN_CM = 4.0
MAP_SHARP_FREE_MAX_CM = 24.0
MAP_SHARP_OCCUPIED_MAX_CM = 18.0
MAP_SHARP_HIT_SCORE = 5
MAP_SHARP_FREE_SCORE = -1

# Digital IR is binary, so it is used mainly as WALL CONFIRMATION rather than
# pretending it provides an exact range. Common IR modules are active-low.
# If your log shows IR=1 when a wall is physically in front of the IR sensor,
# change MAP_IR_WALL_LEVEL to 1.
MAP_IR_WALL_LEVEL = 0
MAP_IR_CONFIRM_LEFT_SHARP = True
MAP_IR_CONFIRM_MAX_SHARP_CM = 22.0
MAP_IR_CONFIRM_SCORE = 4

# Optional only after measuring the IR mounting angle/range. It is OFF by default
# because a binary IR sensor does not provide exact distance; using a guessed
# endpoint can make the map look worse. IR still actively confirms Sharp walls.
MAP_IR_FALLBACK_ENABLED = False
MAP_IR_SENSOR_ANGLE_DEG = -45.0
MAP_IR_SENSOR_FORWARD_M = 0.08
MAP_IR_SENSOR_RIGHT_M = -0.07
MAP_IR_ASSUMED_RANGE_M = 0.12
MAP_IR_FALLBACK_HIT_SCORE = 1
MAP_IR_FALLBACK_PATCH_RADIUS_CELLS = 1

# Wall rendering/post-processing only. Raw occupancy evidence is preserved in CSV.
MAP_DISPLAY_WALL_DILATION_CELLS = 1
MAP_DISPLAY_BRIDGE_GAP_CELLS = 2
MAP_DISPLAY_REMOVE_ISOLATED_WALLS = True
# Connect consecutive confirmed wall hits only while the robot keeps the same
# logical heading. This fills sensor-sampling gaps without bridging real doors.
MAP_CONNECT_CONSECUTIVE_WALL_HITS = True
MAP_WALL_CONNECT_MAX_M = 0.18
MAP_WALL_CONNECT_SCORE = 4

# Junction-based map-only loop closure. The controller pose is never modified.
MAP_LOOP_CLOSURE_MIN_ERROR_M = 0.015
MAP_LOOP_CLOSURE_MAX_ERROR_M = 0.35
MAP_LOOP_CLOSURE_GAIN = 1.0
MAP_SAVE_ON_JUNCTION = True
MAP_AUTOSAVE_SEC = 0.0

# Export. Unknown=gray, free=white, occupied=black, trajectory=blue.
MAP_EXPORT_MARGIN_M = 0.30
MAP_SVG_PX_PER_M = 420.0
MAP_EXPORT_PNG = True
MAP_PNG_DPI = 220
MAP_DRAW_TRAJECTORY = True
MAP_DRAW_NODES = True
MAP_DRAW_EXIT = True


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
# V8 FEEDBACK TURN / ACTION WATCHDOG
# ============================================================
# Normal 90/180 turns use attitude yaw + drive_speed closed-loop so the program
# does not depend on an SDK action-completion ACK. Real attitude telemetry can
# pause for a few samples, so V8 accepts a small final error at timeout and can
# retry toward the SAME absolute target instead of adding another 90/180 deg.
ENABLE_FEEDBACK_TURN = True
TURN_PRE_SETTLE_SEC = 0.10
TURN_FEEDBACK_KP = 1.20
TURN_FEEDBACK_MIN_Z_SPEED = 10.0
TURN_FEEDBACK_MAX_Z_SPEED = 55.0
TURN_FEEDBACK_TOLERANCE_DEG = 4.0
TURN_FEEDBACK_STABLE_SAMPLES = 2
TURN_FEEDBACK_LOOP_SEC = 0.03
TURN_FEEDBACK_DRIVE_TIMEOUT_SEC = 0.20
TURN_FEEDBACK_TIMEOUT_90_SEC = 5.00
TURN_FEEDBACK_TIMEOUT_180_SEC = 8.00
TURN_FEEDBACK_PRINT_SEC = 0.25
TURN_TIMEOUT_ACCEPT_TOLERANCE_DEG = 5.0
TURN_MAX_ATTEMPTS = 3
TURN_RETRY_SETTLE_SEC = 0.25

# Fallback only if attitude yaw is unavailable. Even this path is bounded.
TURN_ACTION_TIMEOUT_90_SEC = 3.50
TURN_ACTION_TIMEOUT_180_SEC = 6.00

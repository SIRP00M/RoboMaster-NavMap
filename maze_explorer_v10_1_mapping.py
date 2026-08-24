"""
RoboMaster Maze Walking Test - SINGLE FILE
==========================================

V10.1 maze exploration + cleaner SLAM-style occupancy-grid mapping + Sharp sensor timeout guard.
Uses the proven V6/V8 frontier-aware exploration, but routes back to pending work with a weighted least-visited graph search to reduce repeated loops in cyclic mazes.
The passive mapper auto-aligns the initial corridor, cardinal-snaps mapping rays, limits long no-hit ToF free rays, and connects consecutive wall hits for a cleaner occupancy map.
The mapper never changes motion-control odometry or planner decisions.

Important test values:
    FORWARD_SPEED = 0.20 m/s
    SIDE_OPEN_ENTER_CM = 18.0 cm / SIDE_OPEN_EXIT_CM = 15.0 cm
    TARGET_LEFT_CM / TARGET_RIGHT_CM = 8.0 cm
    SIDE_TOO_CLOSE_CM = 5.5 cm

Run:
    py maze_explorer_v10_1_mapping.py

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

        # V10.1: last-valid Sharp cache. get_adc() may return None after an
        # SDK send_sync_msg timeout without raising an exception.
        self.sharp_last_valid = {
            config.SHARP_LEFT_ID: {"raw": None, "cm": None, "time": None},
            config.SHARP_RIGHT_ID: {"raw": None, "cm": None, "time": None},
        }
        self.sharp_invalid_count = {
            config.SHARP_LEFT_ID: 0,
            config.SHARP_RIGHT_ID: 0,
        }
        self.sharp_last_warn_time = {
            config.SHARP_LEFT_ID: 0.0,
            config.SHARP_RIGHT_ID: 0.0,
        }

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

    @staticmethod
    def _valid_adc(raw):
        if raw is None:
            return False
        try:
            value = float(raw)
        except (TypeError, ValueError):
            return False
        return math.isfinite(value) and 0.0 <= value <= 1023.0

    def _cached_sharp(self, sensor_id):
        cache = self.sharp_last_valid[sensor_id]
        if cache["time"] is None or cache["cm"] is None:
            return None, None, None
        age = time.monotonic() - cache["time"]
        if age <= config.SHARP_STALE_HOLD_SEC:
            return cache["raw"], cache["cm"], age
        return None, None, age

    def _warn_invalid_sharp(self, sensor_id, message):
        now = time.monotonic()
        if now - self.sharp_last_warn_time[sensor_id] >= config.SHARP_INVALID_WARN_INTERVAL_SEC:
            side = "LEFT" if sensor_id == config.SHARP_LEFT_ID else "RIGHT"
            print(f">>> SHARP {side} WARNING: {message}")
            self.sharp_last_warn_time[sensor_id] = now

    def read_sharp_raw_and_cm(self, sensor_id):
        """Read one Sharp sensor without letting SDK timeouts crash the run.

        RoboMaster's synchronous get_adc() can log a send_sync_msg timeout and
        return None.  None must never enter statistics.median().  For a short
        outage we return the last valid filtered distance; if the cache is too
        old we return (None, None), which makes the main loop stop safely until
        the sensor recovers.
        """
        try:
            raw = self.sensor_adapter.get_adc(
                id=sensor_id,
                port=config.SENSOR_PORT,
            )
        except Exception as exc:
            raw = None
            self._warn_invalid_sharp(sensor_id, f"read exception: {exc}")

        if not self._valid_adc(raw):
            self.sharp_invalid_count[sensor_id] += 1
            cached_raw, cached_cm, age = self._cached_sharp(sensor_id)
            if cached_cm is not None:
                self._warn_invalid_sharp(
                    sensor_id,
                    f"invalid ADC={raw!r}; using cached value age={age:.2f}s "
                    f"misses={self.sharp_invalid_count[sensor_id]}",
                )
                return cached_raw, cached_cm

            age_text = "none" if age is None else f"{age:.2f}s"
            self._warn_invalid_sharp(
                sensor_id,
                f"invalid ADC={raw!r}; no fresh cache (age={age_text}) "
                f"misses={self.sharp_invalid_count[sensor_id]}",
            )
            return None, None

        raw = int(round(float(raw)))
        self.sharp_invalid_count[sensor_id] = 0

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
        cm = self.adc_to_cm(ema_val, table)
        self.sharp_last_valid[sensor_id] = {
            "raw": raw,
            "cm": cm,
            "time": time.monotonic(),
        }
        return raw, cm

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

        now = time.monotonic()
        for sensor_id in (config.SHARP_LEFT_ID, config.SHARP_RIGHT_ID):
            self.sharp_last_valid[sensor_id] = {"raw": None, "cm": None, "time": None}
            self.sharp_invalid_count[sensor_id] = 0
            self.sharp_last_warn_time[sensor_id] = min(
                self.sharp_last_warn_time.get(sensor_id, 0.0), now
            )

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

        # V10 external corridor reference for slow IMU-yaw drift correction.
        self._corr_cal_x = None
        self._corr_cal_y = None
        self._corr_cal_left = None
        self._corr_cal_right = None
        self._corr_cal_heading_index = None

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
        self.reset_corridor_heading_calibration()

    def reset_corridor_heading_calibration(self):
        self._corr_cal_x = None
        self._corr_cal_y = None
        self._corr_cal_left = None
        self._corr_cal_right = None
        self._corr_cal_heading_index = None

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

    def update_corridor_heading_reference(
        self,
        left_cm,
        right_cm,
        front_cm,
        pose_tracker,
        heading_index,
        allow=True,
    ):
        """Slowly trim the absolute yaw grid from straight-wall geometry.

        A decreasing LEFT distance means the chassis is pointing into the left
        wall; an increasing RIGHT distance says the same thing.  Convert that
        physical skew into the attitude-yaw sign learned for a logical RIGHT
        turn, then move heading_base_yaw/target_yaw only a small amount.
        """
        if not getattr(config, "ENABLE_CORRIDOR_HEADING_CALIBRATION", False):
            return None
        if not allow or pose_tracker is None:
            self.reset_corridor_heading_calibration()
            return None
        if front_cm is None or front_cm < config.CORRIDOR_CAL_MIN_FRONT_CM:
            self.reset_corridor_heading_calibration()
            return None
        if self.heading_right_step_sign not in (-1, 1):
            return None

        x, y, _ = pose_tracker.get_pose()
        if x is None or y is None:
            return None

        def wall_ok(v):
            return (
                v is not None
                and config.CORRIDOR_CAL_MIN_WALL_CM <= v <= config.CORRIDOR_CAL_MAX_WALL_CM
            )

        left_ok = wall_ok(left_cm)
        right_ok = wall_ok(right_cm)
        if not (left_ok or right_ok):
            self.reset_corridor_heading_calibration()
            return None

        if (
            self._corr_cal_x is None
            or self._corr_cal_heading_index != int(heading_index)
        ):
            self._corr_cal_x, self._corr_cal_y = x, y
            self._corr_cal_left = float(left_cm) if left_ok else None
            self._corr_cal_right = float(right_cm) if right_ok else None
            self._corr_cal_heading_index = int(heading_index)
            return None

        travel = ((x - self._corr_cal_x) ** 2 + (y - self._corr_cal_y) ** 2) ** 0.5
        if travel < config.CORRIDOR_CAL_MIN_TRAVEL_M:
            return None

        import math
        estimates = []
        # Positive estimate below means an attitude-yaw adjustment in the
        # direction required to bring the chassis parallel to the wall.
        if left_ok and self._corr_cal_left is not None:
            dd_m = (float(left_cm) - self._corr_cal_left) / 100.0
            a = math.degrees(math.atan2(dd_m, max(travel, 1e-6)))
            # LEFT distance increasing -> chassis points right -> correct left.
            estimates.append(-self.heading_right_step_sign * a)
        if right_ok and self._corr_cal_right is not None:
            dd_m = (float(right_cm) - self._corr_cal_right) / 100.0
            a = math.degrees(math.atan2(dd_m, max(travel, 1e-6)))
            # RIGHT distance increasing -> chassis points left -> correct right.
            estimates.append(self.heading_right_step_sign * a)

        # Start a fresh baseline whether or not this estimate is accepted.
        self._corr_cal_x, self._corr_cal_y = x, y
        self._corr_cal_left = float(left_cm) if left_ok else None
        self._corr_cal_right = float(right_cm) if right_ok else None
        self._corr_cal_heading_index = int(heading_index)

        if not estimates:
            return None
        estimate = sum(estimates) / len(estimates)
        if abs(estimate) > config.CORRIDOR_CAL_MAX_ESTIMATE_DEG:
            return None

        step = clamp(
            estimate * config.CORRIDOR_CAL_ALPHA,
            -config.CORRIDOR_CAL_MAX_STEP_DEG,
            config.CORRIDOR_CAL_MAX_STEP_DEG,
        )
        if abs(step) < 0.02:
            return step

        self.heading_base_yaw = normalize_angle_deg(self.heading_base_yaw + step)
        if self.heading_target_yaw is not None:
            self.heading_target_yaw = normalize_angle_deg(self.heading_target_yaw + step)

        if abs(step) >= config.CORRIDOR_CAL_LOG_MIN_STEP_DEG:
            print(
                f">>> CORRIDOR_HEADING_CAL estimate={estimate:+.2f}deg "
                f"trim={step:+.2f}deg target={self.heading_target_yaw:+.2f}"
            )
        return step

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

        # V7: the robot initially points INTO the maze. The opposite absolute
        # direction at J0 is the physical entrance/outside and must never become
        # a frontier or a valid exploration/exit direction.
        self.start_inside_abs_dir = None
        self.start_outside_abs_dir = None

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
        self.start_inside_abs_dir = abs_index % 4
        self.start_outside_abs_dir = self.opposite_index(abs_index)
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

    @staticmethod
    def _is_unresolved_state(state):
        """Traversed/open edge whose destination was never linked.

        This is different from a normal frontier: visits is already >0, but
        target is still None. Treating it as fully explored can hide a real
        branch and make graph routing loop around other nodes.
        """
        if not bool(getattr(config, "ENABLE_UNRESOLVED_EDGE_RECOVERY", True)):
            return False
        max_visits = max(1, int(getattr(config, "UNRESOLVED_EDGE_MAX_VISITS", 3)))
        return (
            0 < state.visits <= max_visits
            and state.target is None
            and not state.blocked
        )

    def unresolved_exits(self, node_id=None):
        if node_id is None:
            node_id = self.current_node_id
        if node_id is None or node_id not in self.nodes:
            return []
        result = []
        for abs_index, state in self.nodes[node_id].exits.items():
            if self._is_unresolved_state(state):
                result.append(abs_index % 4)
        return sorted(result)

    def pending_exits(self, node_id=None):
        """All exits that still require exploration or graph resolution."""
        if node_id is None:
            node_id = self.current_node_id
        if node_id is None or node_id not in self.nodes:
            return []
        result = set(self.frontier_exits(node_id))
        result.update(self.unresolved_exits(node_id))
        return sorted(result)

    def all_pending_exits(self):
        result = []
        for node_id in sorted(self.nodes, key=lambda n: int(n[1:]) if n[1:].isdigit() else n):
            for abs_index in self.pending_exits(node_id):
                result.append((node_id, abs_index))
        return result

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
            if self.pending_exits(node_id):
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
            if node_id != start and self.pending_exits(node_id):
                return path

            for nxt, abs_index in self._graph_neighbors(node_id):
                if len(path) == 1 and abs_index not in allowed_first_abs:
                    continue
                if nxt in visited:
                    continue
                visited.add(nxt)
                q.append(path + [nxt])
        return None

    def _least_cost_pending_path(self, allowed_first_abs):
        """Dijkstra route to pending work with a penalty for reused edges.

        This is intentionally used only after local FRONTIER / UNRESOLVED exits
        have already been handled. In cyclic mazes it is more stable than
        replaying the historical DFS stack because an edge crossed 4-5 times
        becomes much more expensive than a fresh known transit edge.
        """
        if not bool(getattr(config, "ENABLE_WEIGHTED_PENDING_ROUTING", True)):
            return None
        start = self.current_node_id
        if start is None or start not in self.nodes:
            return None

        import heapq
        allowed_first_abs = set(allowed_first_abs)
        base = float(getattr(config, "ROUTE_EDGE_BASE_COST", 1.0))
        visit_pen = float(getattr(config, "ROUTE_EDGE_VISIT_PENALTY", 1.75))
        high_extra = float(getattr(config, "ROUTE_EDGE_HIGH_VISIT_EXTRA", 2.0))
        unresolved_extra = float(getattr(config, "ROUTE_PENDING_UNRESOLVED_EXTRA", 1.25))

        # (cost, hops, node, path)
        pq = [(0.0, 0, start, [start])]
        best = {start: 0.0}

        while pq:
            cost, hops, node_id, path = heapq.heappop(pq)
            if cost > best.get(node_id, float("inf")) + 1e-9:
                continue

            if node_id != start and self.pending_exits(node_id):
                # Prefer true frontiers over merely-unresolved work when route
                # costs are otherwise similar.
                pending = self.pending_exits(node_id)
                if pending and all(
                    p not in self.frontier_exits(node_id)
                    for p in pending
                ):
                    cost += unresolved_extra
                return path, cost

            for nxt, abs_index in self._graph_neighbors(node_id):
                if len(path) == 1 and abs_index not in allowed_first_abs:
                    continue
                state = self._exit(node_id, abs_index)
                visits = max(0, int(state.visits))
                edge_cost = base + visit_pen * visits
                if visits >= 3:
                    edge_cost += high_extra * (visits - 2)
                new_cost = cost + edge_cost
                if new_cost + 1e-9 >= best.get(nxt, float("inf")):
                    continue
                best[nxt] = new_cost
                heapq.heappush(pq, (new_cost, hops + 1, nxt, path + [nxt]))

        return None

    def _frontier_signature(self):
        # Keep the old method name for compatibility, but include unresolved
        # edges too. Loop protection should reset whenever either a true
        # frontier or an unresolved graph edge changes.
        return tuple(self.all_pending_exits())

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

        # V7 START-GATE virtual wall. When J0 is revisited from inside, the
        # physical entrance often appears as a perfectly valid FRONT opening.
        # Never register that outside direction as a frontier and never choose it.
        if (
            bool(getattr(config, "ENABLE_START_GATE_GUARD", True))
            and self.current_node_id == self.start_node_id
            and self.start_outside_abs_dir is not None
        ):
            before = list(candidates)
            candidates = [
                item for item in candidates
                if (item[1] % 4) != (self.start_outside_abs_dir % 4)
            ]
            if len(candidates) != len(before):
                print(
                    ">>> START_GATE PLANNER BLOCK "
                    f"outside={self.heading_name(self.start_outside_abs_dir)}"
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

        # V8: a physically confirmed opening may already have visits>0 while
        # target is still None. That means we departed this way before but never
        # linked the corridor to its destination node. Do not route away from
        # such an opening as if it were explored; resolve it first.
        local_unresolved = [
            item for item in scored
            if self._is_unresolved_state(item[4])
        ]
        if local_unresolved:
            visits, _, relative, abs_index, _ = local_unresolved[0]
            self.completed = False
            self._record_graph_event(
                "UNRESOLVED_EDGE_LOCAL_RETRY",
                node=self.current_node_id,
                heading=self.heading_name(abs_index),
                visits=visits,
            )
            return ExplorationDecision(
                direction=relative,
                node_id=self.current_node_id,
                reason="UNRESOLVED_EDGE_RETRY",
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
        global_pending = self.all_pending_exits()

        # 2) No true frontier AND no recoverable unresolved edge anywhere ->
        # true global completion. This prevents a visits>0,target=None corridor
        # from disappearing from the exploration workload.
        if not global_pending:
            self.completed = True
            return ExplorationDecision(
                direction="COMPLETE",
                node_id=self.current_node_id,
                reason="ALL_FRONTIERS_EXPLORED",
                visits_before=0,
                absolute_heading=self.heading_name(),
            )

        frontier_signature = tuple(global_pending)
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

        # 3) V10: in a graph containing loops/islands, route to pending work by
        # least weighted traversal cost before replaying the historical DFS
        # stack. This sharply reduces repeated laps through heavily used edges.
        weighted = self._least_cost_pending_path(allowed_first_abs)
        if weighted is not None:
            path, route_cost = weighted
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
                    self._record_graph_event(
                        "WEIGHTED_PENDING_ROUTE",
                        node=self.current_node_id,
                        next_node=path[1],
                        target_node=path[-1],
                        heading=self.heading_name(abs_index),
                        visits=visits,
                        route_cost=route_cost,
                        path=path,
                    )
                    return ExplorationDecision(
                        direction=relative,
                        node_id=self.current_node_id,
                        reason="ROUTE_TO_LOW_COST_PENDING",
                        visits_before=visits,
                        absolute_heading=self.heading_name(abs_index),
                    )

        # 4) Fallback: classic DFS-stack routing when the weighted graph path
        # cannot be formed because topology is temporarily incomplete.
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

        # 5) Loops / merged topology can leave a frontier outside the active DFS
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

        # 6) Frontiers exist but current graph/sensor snapshot cannot route to
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
            elif self._is_unresolved_state(exit_state):
                suffix = "[UNRESOLVED]"
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
            "unresolved_edges": [
                {"node": node_id, "heading": self.heading_name(abs_index)}
                for node_id, abs_index in self.all_pending_exits()
                if abs_index in self.unresolved_exits(node_id)
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
                        "unresolved": self._is_unresolved_state(exit_state),
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


# ==================== SLAM-STYLE PASSIVE MAPPER ====================
"""Occupancy-grid mapper for the V6 maze explorer.

Map convention:
    +Y = NORTH / initial robot forward
    +X = EAST / initial robot right
    theta 0 deg = NORTH, +90 deg = EAST

This module is passive: it reads pose/sensors and writes map files. It never
changes the pose used by MotionController or TremauxExplorer.
"""

import csv
import os
from dataclasses import dataclass as _map_dataclass


def _map_cfg(name, default):
    return getattr(config, name, default)


def _map_safe_float(value):
    if value is None:
        return None
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(value):
        return None
    return value


@_map_dataclass
class _MapSample:
    index: int
    time_sec: float
    raw_x: float
    raw_y: float
    yaw_deg: float
    base_x: float
    base_y: float
    map_x: float
    map_y: float
    theta_deg: float
    heading_index: int
    front_cm: object
    left_cm: object
    right_cm: object
    ir_value: object
    mode: str
    map_ranges: bool


class SLAMStyleMazeMapper:
    SENSOR_FRONT = "front_tof"
    SENSOR_LEFT = "left_sharp"
    SENSOR_RIGHT = "right_sharp"
    SENSOR_IR = "left_front_ir"

    def __init__(self, output_dir=None):
        self.enabled = bool(_map_cfg("ENABLE_MAPPING", True))
        self.output_dir = output_dir or _map_cfg("MAP_OUTPUT_DIR", "mapping_output")
        self.initialized = False
        self.start_raw_x = None
        self.start_raw_y = None
        self.start_yaw = None
        self.start_monotonic = None
        self.global_corr_x = 0.0
        self.global_corr_y = 0.0
        self.samples = []
        self.grid = {}              # (gx,gy) -> evidence score
        self.wall_points = []
        self.node_anchors = {}
        self.node_events = []
        self.loop_closures = []
        self.exit_event = None
        self.last_known_junction_sample_index = None
        self.last_record_monotonic = None
        self.last_autosave_monotonic = None
        self.yaw_fallback_count = 0
        self.ir_confirm_count = 0
        self.ir_fallback_count = 0
        self._warned_sample_limit = False
        self.position_rotation_deg = float(_map_cfg("MAP_POSITION_ROTATION_DEG", 0.0))
        self.position_auto_aligned = not bool(_map_cfg("MAP_AUTO_ALIGN_INITIAL_PATH", True))
        self._auto_align_reported = False
        self._last_wall_hit = {}

    # --------------------------------------------------------
    # Coordinate transform
    # --------------------------------------------------------
    def initialize(self, raw_x, raw_y, start_yaw, heading_index=0):
        if not self.enabled:
            return None
        raw_x = _map_safe_float(raw_x)
        raw_y = _map_safe_float(raw_y)
        start_yaw = _map_safe_float(start_yaw)
        if raw_x is None or raw_y is None:
            raise ValueError("Mapper requires valid starting x/y")
        if start_yaw is None:
            start_yaw = 0.0

        self.start_raw_x = raw_x
        self.start_raw_y = raw_y
        self.start_yaw = normalize_angle_deg(start_yaw)
        self.start_monotonic = time.monotonic()
        self.last_autosave_monotonic = self.start_monotonic
        self.initialized = True
        os.makedirs(self.output_dir, exist_ok=True)
        if bool(_map_cfg("MAP_CLEAR_OUTPUT_ON_START", True)):
            self._clear_old_outputs()

        return self.record_pose(
            raw_x, raw_y, start_yaw,
            heading_index=heading_index,
            mode="START", force=True,
        )

    def _clear_old_outputs(self):
        for name in (
            "trajectory.csv", "wall_points.csv", "occupancy_grid.csv",
            "occupancy_grid.json", "nodes.json", "loop_closures.json",
            "mapping_summary.json", "exit.json", "maze_map.svg", "maze_map.png",
        ):
            path = os.path.join(self.output_dir, name)
            try:
                if os.path.isfile(path):
                    os.remove(path)
            except OSError:
                pass

    def _raw_position_unrotated(self, raw_x, raw_y):
        dx = float(raw_x) - self.start_raw_x
        dy = float(raw_y) - self.start_raw_y
        if bool(_map_cfg("MAP_SWAP_RAW_XY", False)):
            dx, dy = dy, dx
        dx *= float(_map_cfg("MAP_RAW_X_SIGN", -1.0))
        dy *= float(_map_cfg("MAP_RAW_Y_SIGN", +1.0))
        return dx, dy

    @staticmethod
    def _rotate_xy(dx, dy, rot_deg):
        if abs(rot_deg) <= 1e-12:
            return dx, dy
        a = math.radians(rot_deg)
        c, ss = math.cos(a), math.sin(a)
        return c * dx - ss * dy, ss * dx + c * dy

    def _raw_position_to_base_map(self, raw_x, raw_y):
        dx, dy = self._raw_position_unrotated(raw_x, raw_y)
        return self._rotate_xy(dx, dy, self.position_rotation_deg)

    def _maybe_auto_align_position(self, raw_x, raw_y, heading_index):
        if self.position_auto_aligned:
            return False
        if not bool(_map_cfg("MAP_AUTO_ALIGN_INITIAL_PATH", True)):
            self.position_auto_aligned = True
            return False
        if heading_index is None:
            return False
        wanted = int(_map_cfg("MAP_AUTO_ALIGN_MAX_HEADING_INDEX", 0)) % 4
        if int(heading_index) % 4 != wanted:
            # The robot turned before enough straight travel was collected.
            # Keep the configured rotation rather than learning from a corner.
            return False

        dx, dy = self._raw_position_unrotated(raw_x, raw_y)
        dist = math.hypot(dx, dy)
        need = float(_map_cfg("MAP_AUTO_ALIGN_MIN_TRAVEL_M", 0.18))
        if dist < need:
            return False

        # Angle measured clockwise from +Y in our map convention. Rotating the
        # raw displacement by this amount places the initial travel on +Y.
        auto_rot = math.degrees(math.atan2(dx, dy))
        fixed = float(_map_cfg("MAP_POSITION_ROTATION_DEG", 0.0))
        self.position_rotation_deg = normalize_angle_deg(fixed + auto_rot)
        self.position_auto_aligned = True

        # This happens before the first meaningful loop closure. Reproject the
        # early samples so the whole map shares one frame, then rebuild evidence.
        if not self.loop_closures and abs(self.global_corr_x) < 1e-9 and abs(self.global_corr_y) < 1e-9:
            for old_sample in self.samples:
                bx, by = self._raw_position_to_base_map(old_sample.raw_x, old_sample.raw_y)
                old_sample.base_x = bx
                old_sample.base_y = by
                old_sample.map_x = bx
                old_sample.map_y = by
            self._rebuild_grid()

        if not self._auto_align_reported:
            print(
                f">>> MAP AUTO ALIGN rotation={self.position_rotation_deg:+.1f} deg "
                f"from initial travel={dist:.3f} m"
            )
            self._auto_align_reported = True
        return True

    def _yaw_to_map_theta(self, yaw_deg, heading_index=None):
        yaw_deg = _map_safe_float(yaw_deg)
        if yaw_deg is None:
            return 0.0 if heading_index is None else float((int(heading_index) % 4) * 90)
        theta = normalize_angle_deg(
            (yaw_deg - self.start_yaw) * float(_map_cfg("MAP_YAW_RIGHT_SIGN", +1.0))
        )
        if heading_index is None:
            return theta
        expected = normalize_angle_deg(float((int(heading_index) % 4) * 90))
        if bool(_map_cfg("MAP_SENSOR_USE_CARDINAL_HEADING", True)):
            return expected
        error = shortest_angle_error_deg(expected, theta)
        if abs(error) <= float(_map_cfg("MAP_YAW_CARDINAL_MAX_ERROR_DEG", 22.0)):
            return theta
        if bool(_map_cfg("MAP_FALLBACK_TO_CARDINAL_HEADING", True)):
            self.yaw_fallback_count += 1
            return expected
        return theta

    # --------------------------------------------------------
    # Sampling / sensor integration
    # --------------------------------------------------------
    def update(
        self, raw_x, raw_y, yaw_deg,
        front_cm=None, left_cm=None, right_cm=None, ir_value=None,
        heading_index=None, mode="RUN", map_ranges=True, force=False,
    ):
        if not self.enabled or not self.initialized:
            return None
        raw_x = _map_safe_float(raw_x)
        raw_y = _map_safe_float(raw_y)
        yaw_deg = _map_safe_float(yaw_deg)
        if raw_x is None or raw_y is None:
            return None
        if yaw_deg is None:
            yaw_deg = self.start_yaw

        now = time.monotonic()
        if (
            not force and self.last_record_monotonic is not None
            and now - self.last_record_monotonic < float(_map_cfg("MAP_MIN_RECORD_INTERVAL_SEC", 0.045))
        ):
            return None
        max_samples = int(_map_cfg("MAP_MAX_SAMPLES", 60000))
        if max_samples > 0 and len(self.samples) >= max_samples:
            if not self._warned_sample_limit:
                print("MAPPER WARNING: sample limit reached; mapping samples paused")
                self._warned_sample_limit = True
            return None

        self._maybe_auto_align_position(raw_x, raw_y, heading_index)
        base_x, base_y = self._raw_position_to_base_map(raw_x, raw_y)
        map_x = base_x + self.global_corr_x
        map_y = base_y + self.global_corr_y
        theta = self._yaw_to_map_theta(yaw_deg, heading_index)
        h = -1 if heading_index is None else int(heading_index) % 4

        sample = _MapSample(
            index=len(self.samples),
            time_sec=now - self.start_monotonic,
            raw_x=raw_x, raw_y=raw_y, yaw_deg=yaw_deg,
            base_x=base_x, base_y=base_y, map_x=map_x, map_y=map_y,
            theta_deg=theta, heading_index=h,
            front_cm=_map_safe_float(front_cm),
            left_cm=_map_safe_float(left_cm),
            right_cm=_map_safe_float(right_cm),
            ir_value=ir_value,
            mode=str(mode or ""), map_ranges=bool(map_ranges),
        )
        self.samples.append(sample)
        self.last_record_monotonic = now
        if sample.map_ranges:
            self._integrate_sample(sample)

        autosave = float(_map_cfg("MAP_AUTOSAVE_SEC", 0.0))
        if autosave > 0 and now - self.last_autosave_monotonic >= autosave:
            self.save_all(rebuild=False, quiet=True)
            self.last_autosave_monotonic = now
        return sample

    def record_pose(self, raw_x, raw_y, yaw_deg, heading_index=None, mode="POSE_ONLY", force=True):
        return self.update(
            raw_x, raw_y, yaw_deg,
            heading_index=heading_index, mode=mode,
            map_ranges=False, force=force,
        )

    @staticmethod
    def _forward_right_to_world(x, y, theta_deg, forward_m, right_m):
        a = math.radians(theta_deg)
        fx, fy = math.sin(a), math.cos(a)
        rx, ry = math.cos(a), -math.sin(a)
        return x + forward_m * fx + right_m * rx, y + forward_m * fy + right_m * ry

    def _sensor_params(self, name):
        if name == self.SENSOR_FRONT:
            return dict(
                angle=float(_map_cfg("MAP_FRONT_SENSOR_ANGLE_DEG", 0.0)),
                forward=float(_map_cfg("MAP_FRONT_SENSOR_FORWARD_M", 0.08)),
                right=float(_map_cfg("MAP_FRONT_SENSOR_RIGHT_M", 0.0)),
                min_cm=float(_map_cfg("MAP_TOF_MIN_CM", 4.0)),
                free_max=float(_map_cfg("MAP_TOF_FREE_MAX_CM", 70.0)),
                hit_max=float(_map_cfg("MAP_TOF_OCCUPIED_MAX_CM", 45.0)),
                hit_score=int(_map_cfg("MAP_TOF_HIT_SCORE", 7)),
                free_score=int(_map_cfg("MAP_TOF_FREE_SCORE", -1)),
            )
        if name == self.SENSOR_LEFT:
            return dict(
                angle=float(_map_cfg("MAP_LEFT_SENSOR_ANGLE_DEG", -90.0)),
                forward=float(_map_cfg("MAP_LEFT_SENSOR_FORWARD_M", 0.02)),
                right=float(_map_cfg("MAP_LEFT_SENSOR_RIGHT_M", -0.10)),
                min_cm=float(_map_cfg("MAP_SHARP_MIN_CM", 4.0)),
                free_max=float(_map_cfg("MAP_SHARP_FREE_MAX_CM", 24.0)),
                hit_max=float(_map_cfg("MAP_SHARP_OCCUPIED_MAX_CM", 18.0)),
                hit_score=int(_map_cfg("MAP_SHARP_HIT_SCORE", 5)),
                free_score=int(_map_cfg("MAP_SHARP_FREE_SCORE", -1)),
            )
        if name == self.SENSOR_RIGHT:
            return dict(
                angle=float(_map_cfg("MAP_RIGHT_SENSOR_ANGLE_DEG", +90.0)),
                forward=float(_map_cfg("MAP_RIGHT_SENSOR_FORWARD_M", 0.02)),
                right=float(_map_cfg("MAP_RIGHT_SENSOR_RIGHT_M", +0.10)),
                min_cm=float(_map_cfg("MAP_SHARP_MIN_CM", 4.0)),
                free_max=float(_map_cfg("MAP_SHARP_FREE_MAX_CM", 24.0)),
                hit_max=float(_map_cfg("MAP_SHARP_OCCUPIED_MAX_CM", 18.0)),
                hit_score=int(_map_cfg("MAP_SHARP_HIT_SCORE", 5)),
                free_score=int(_map_cfg("MAP_SHARP_FREE_SCORE", -1)),
            )
        raise ValueError(name)

    def _ray(self, sample, name, distance_cm):
        distance_cm = _map_safe_float(distance_cm)
        if distance_cm is None:
            return None
        p = self._sensor_params(name)
        if distance_cm < p["min_cm"]:
            return None
        used_cm = min(distance_cm, p["free_max"])
        if used_cm <= 0:
            return None
        has_hit = distance_cm <= p["hit_max"]
        if (
            name == self.SENSOR_FRONT
            and not has_hit
            and float(_map_cfg("MAP_TOF_NO_HIT_FREE_MAX_CM", 28.0)) > 0
        ):
            used_cm = min(
                used_cm,
                float(_map_cfg("MAP_TOF_NO_HIT_FREE_MAX_CM", 28.0)),
            )
        ox, oy = self._forward_right_to_world(
            sample.map_x, sample.map_y, sample.theta_deg, p["forward"], p["right"]
        )
        ray_theta = normalize_angle_deg(sample.theta_deg + p["angle"])
        a = math.radians(ray_theta)
        d = used_cm / 100.0
        ex = ox + d * math.sin(a)
        ey = oy + d * math.cos(a)
        return dict(
            sensor=name, origin_x=ox, origin_y=oy, end_x=ex, end_y=ey,
            measured_cm=distance_cm, used_cm=used_cm, has_hit=has_hit,
            hit_score=p["hit_score"], free_score=p["free_score"],
        )

    def _world_to_cell(self, x, y):
        r = max(0.005, float(_map_cfg("MAP_RESOLUTION_M", 0.025)))
        return int(round(x / r)), int(round(y / r))

    def _cell_to_world(self, gx, gy):
        r = max(0.005, float(_map_cfg("MAP_RESOLUTION_M", 0.025)))
        return gx * r, gy * r

    @staticmethod
    def _bresenham(x0, y0, x1, y1):
        cells = []
        dx, sx = abs(x1 - x0), 1 if x0 < x1 else -1
        dy, sy = -abs(y1 - y0), 1 if y0 < y1 else -1
        err = dx + dy
        x, y = x0, y0
        while True:
            cells.append((x, y))
            if x == x1 and y == y1:
                return cells
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x += sx
            if e2 <= dx:
                err += dx
                y += sy

    def _update_cell(self, cell, delta):
        lo = int(_map_cfg("MAP_EVIDENCE_MIN", -30))
        hi = int(_map_cfg("MAP_EVIDENCE_MAX", +30))
        self.grid[cell] = int(clamp(int(self.grid.get(cell, 0)) + int(delta), lo, hi))

    def _mark_robot_footprint_free(self, sample):
        radius = float(_map_cfg("MAP_ROBOT_FREE_RADIUS_M", 0.11))
        score = int(_map_cfg("MAP_ROBOT_FREE_SCORE", -3))
        res = float(_map_cfg("MAP_RESOLUTION_M", 0.025))
        c0 = self._world_to_cell(sample.map_x, sample.map_y)
        n = max(0, int(math.ceil(radius / res)))
        for dx in range(-n, n + 1):
            for dy in range(-n, n + 1):
                if math.hypot(dx * res, dy * res) <= radius:
                    self._update_cell((c0[0] + dx, c0[1] + dy), score)

    def _integrate_ray(self, sample, ray):
        sc = self._world_to_cell(ray["origin_x"], ray["origin_y"])
        ec = self._world_to_cell(ray["end_x"], ray["end_y"])
        cells = self._bresenham(sc[0], sc[1], ec[0], ec[1])
        free_cells = cells[:-1] if ray["has_hit"] else cells
        for cell in free_cells:
            self._update_cell(cell, ray["free_score"])

        sensor = ray["sensor"]
        if ray["has_hit"] and cells:
            self._update_cell(cells[-1], ray["hit_score"])

            if bool(_map_cfg("MAP_CONNECT_CONSECUTIVE_WALL_HITS", True)):
                prev = self._last_wall_hit.get(sensor)
                current = {
                    "x": ray["end_x"],
                    "y": ray["end_y"],
                    "heading_index": sample.heading_index,
                }
                if (
                    prev is not None
                    and prev.get("heading_index") == sample.heading_index
                    and sample.heading_index >= 0
                ):
                    gap = math.hypot(
                        current["x"] - prev["x"],
                        current["y"] - prev["y"],
                    )
                    if gap <= float(_map_cfg("MAP_WALL_CONNECT_MAX_M", 0.18)):
                        pc = self._world_to_cell(prev["x"], prev["y"])
                        cc = self._world_to_cell(current["x"], current["y"])
                        for cell in self._bresenham(pc[0], pc[1], cc[0], cc[1]):
                            self._update_cell(
                                cell,
                                int(_map_cfg("MAP_WALL_CONNECT_SCORE", 4)),
                            )
                self._last_wall_hit[sensor] = current

            self.wall_points.append(dict(
                sample_index=sample.index, time_sec=sample.time_sec,
                sensor=sensor, x=ray["end_x"], y=ray["end_y"],
                distance_cm=ray["measured_cm"],
            ))
        else:
            # An opening/no-hit breaks the wall chain so a doorway cannot be
            # bridged by the next wall sample.
            self._last_wall_hit.pop(sensor, None)
        return ray

    def _ir_is_wall(self, value):
        if value is None:
            return False
        try:
            return int(value) == int(_map_cfg("MAP_IR_WALL_LEVEL", 0))
        except Exception:
            return False

    def _integrate_ir(self, sample, left_ray):
        if not self._ir_is_wall(sample.ir_value):
            return

        # Safest use of a binary sensor: strengthen a geometrically measured
        # left Sharp wall, without changing the measured position.
        if (
            bool(_map_cfg("MAP_IR_CONFIRM_LEFT_SHARP", True))
            and left_ray is not None and left_ray["has_hit"]
            and left_ray["measured_cm"] <= float(_map_cfg("MAP_IR_CONFIRM_MAX_SHARP_CM", 22.0))
        ):
            cell = self._world_to_cell(left_ray["end_x"], left_ray["end_y"])
            self._update_cell(cell, int(_map_cfg("MAP_IR_CONFIRM_SCORE", 4)))
            self.wall_points.append(dict(
                sample_index=sample.index, time_sec=sample.time_sec,
                sensor="ir_confirm_left", x=left_ray["end_x"], y=left_ray["end_y"],
                distance_cm=left_ray["measured_cm"],
            ))
            self.ir_confirm_count += 1
            return

        if not bool(_map_cfg("MAP_IR_FALLBACK_ENABLED", True)):
            return

        # Binary-only fallback: weak hit around an assumed location. Because
        # score=1 and occupied threshold=4, several consistent samples are
        # required before this becomes a visible wall.
        ox, oy = self._forward_right_to_world(
            sample.map_x, sample.map_y, sample.theta_deg,
            float(_map_cfg("MAP_IR_SENSOR_FORWARD_M", 0.08)),
            float(_map_cfg("MAP_IR_SENSOR_RIGHT_M", -0.07)),
        )
        theta = normalize_angle_deg(
            sample.theta_deg + float(_map_cfg("MAP_IR_SENSOR_ANGLE_DEG", -45.0))
        )
        d = float(_map_cfg("MAP_IR_ASSUMED_RANGE_M", 0.12))
        a = math.radians(theta)
        ex, ey = ox + d * math.sin(a), oy + d * math.cos(a)
        c = self._world_to_cell(ex, ey)
        radius = max(0, int(_map_cfg("MAP_IR_FALLBACK_PATCH_RADIUS_CELLS", 1)))
        score = int(_map_cfg("MAP_IR_FALLBACK_HIT_SCORE", 1))
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                if abs(dx) + abs(dy) <= radius:
                    self._update_cell((c[0] + dx, c[1] + dy), score)
        self.wall_points.append(dict(
            sample_index=sample.index, time_sec=sample.time_sec,
            sensor=self.SENSOR_IR, x=ex, y=ey, distance_cm=d * 100.0,
        ))
        self.ir_fallback_count += 1

    def _integrate_sample(self, sample):
        self._mark_robot_footprint_free(sample)
        left_ray = None
        for name, dist in (
            (self.SENSOR_FRONT, sample.front_cm),
            (self.SENSOR_LEFT, sample.left_cm),
            (self.SENSOR_RIGHT, sample.right_cm),
        ):
            ray = self._ray(sample, name, dist)
            if ray is not None:
                self._integrate_ray(sample, ray)
                if name == self.SENSOR_LEFT:
                    left_ray = ray
        self._integrate_ir(sample, left_ray)

    def _rebuild_grid(self):
        self.grid = {}
        self.wall_points = []
        self.ir_confirm_count = 0
        self.ir_fallback_count = 0
        self._last_wall_hit = {}
        for s in self.samples:
            if s.map_ranges:
                self._integrate_sample(s)

    # --------------------------------------------------------
    # Junction loop closure (map only)
    # --------------------------------------------------------
    def observe_junction(self, node_id, is_new, raw_x, raw_y, yaw_deg, heading_index=None):
        if not self.enabled or not self.initialized or node_id is None:
            return None
        sample = self.record_pose(
            raw_x, raw_y, yaw_deg,
            heading_index=heading_index,
            mode="JUNCTION_" + ("NEW" if is_new else "KNOWN"),
            force=True,
        )
        if sample is None:
            return None
        idx, now = sample.index, sample.time_sec

        if node_id not in self.node_anchors:
            self.node_anchors[node_id] = dict(
                x=sample.map_x, y=sample.map_y,
                sample_index=idx, first_seen_time=now,
            )
            self.node_events.append(dict(
                time_sec=now, node_id=node_id,
                event="ANCHOR_NEW" if is_new else "ANCHOR_RECOVERED",
                sample_index=idx, map_x=sample.map_x, map_y=sample.map_y,
            ))
            if self.last_known_junction_sample_index is None or not is_new:
                self.last_known_junction_sample_index = idx
            if bool(_map_cfg("MAP_SAVE_ON_JUNCTION", True)):
                self.save_all(rebuild=False, quiet=True)
            return dict(corrected=False, error_m=0.0)

        anchor = self.node_anchors[node_id]
        ex = anchor["x"] - sample.map_x
        ey = anchor["y"] - sample.map_y
        em = math.hypot(ex, ey)
        min_e = float(_map_cfg("MAP_LOOP_CLOSURE_MIN_ERROR_M", 0.015))
        max_e = float(_map_cfg("MAP_LOOP_CLOSURE_MAX_ERROR_M", 0.35))
        gain = clamp(float(_map_cfg("MAP_LOOP_CLOSURE_GAIN", 1.0)), 0.0, 1.0)
        corrected = False
        reason = "NO_CORRECTION_NEEDED"

        if em >= min_e:
            if em <= max_e:
                start_idx = self.last_known_junction_sample_index
                if start_idx is None:
                    start_idx = 0
                start_idx = max(0, min(int(start_idx), idx))
                ax, ay = ex * gain, ey * gain
                denom = max(1, idx - start_idx)
                for i in range(start_idx, idx + 1):
                    t = float(i - start_idx) / denom
                    self.samples[i].map_x += ax * t
                    self.samples[i].map_y += ay * t
                for nid, a in self.node_anchors.items():
                    if nid == node_id:
                        continue
                    ai = int(a.get("sample_index", -1))
                    if start_idx <= ai <= idx:
                        t = float(ai - start_idx) / denom
                        a["x"] += ax * t
                        a["y"] += ay * t
                self.global_corr_x += ax
                self.global_corr_y += ay
                self.loop_closures.append(dict(
                    time_sec=now, node_id=node_id,
                    sample_index=idx, segment_start_index=start_idx,
                    raw_error_x_m=ex, raw_error_y_m=ey, raw_error_m=em,
                    applied_x_m=ax, applied_y_m=ay,
                ))
                self._rebuild_grid()
                corrected = True
                reason = "LOOP_CLOSURE_APPLIED"
            else:
                reason = "LOOP_CLOSURE_REJECTED_TOO_LARGE"

        self.node_events.append(dict(
            time_sec=now, node_id=node_id, event=reason,
            sample_index=idx, error_x_m=ex, error_y_m=ey, error_m=em,
        ))
        if not is_new or corrected:
            self.last_known_junction_sample_index = idx
        if bool(_map_cfg("MAP_SAVE_ON_JUNCTION", True)):
            self.save_all(rebuild=False, quiet=True)
        return dict(corrected=corrected, reason=reason, error_m=em)

    # --------------------------------------------------------
    # Map states / visual cleanup
    # --------------------------------------------------------
    def cell_state(self, score):
        if score >= int(_map_cfg("MAP_OCCUPIED_SCORE_THRESHOLD", 4)):
            return "OCCUPIED"
        if score <= int(_map_cfg("MAP_FREE_SCORE_THRESHOLD", -3)):
            return "FREE"
        return "UNKNOWN"

    def _display_sets(self):
        occupied = {c for c, s in self.grid.items() if self.cell_state(s) == "OCCUPIED"}
        free = {c for c, s in self.grid.items() if self.cell_state(s) == "FREE"}

        # Remove isolated one-cell hits from display only; raw evidence remains.
        if bool(_map_cfg("MAP_DISPLAY_REMOVE_ISOLATED_WALLS", True)) and occupied:
            keep = set()
            for x, y in occupied:
                neighbours = sum(
                    ((x + dx, y + dy) in occupied)
                    for dx in (-1, 0, 1) for dy in (-1, 0, 1)
                    if not (dx == 0 and dy == 0)
                )
                score = self.grid.get((x, y), 0)
                if neighbours > 0 or score >= int(_map_cfg("MAP_OCCUPIED_SCORE_THRESHOLD", 4)) + 4:
                    keep.add((x, y))
            occupied = keep

        # Bridge tiny 1-2 cell gaps along horizontal/vertical walls.
        gap = max(0, int(_map_cfg("MAP_DISPLAY_BRIDGE_GAP_CELLS", 2)))
        bridged = set(occupied)
        for x, y in list(occupied):
            for d in range(2, gap + 2):
                if (x + d, y) in occupied:
                    for k in range(1, d): bridged.add((x + k, y))
                if (x, y + d) in occupied:
                    for k in range(1, d): bridged.add((x, y + k))
        occupied = bridged

        # Slight wall thickness gives a lidar/SLAM-map appearance.
        radius = max(0, int(_map_cfg("MAP_DISPLAY_WALL_DILATION_CELLS", 1)))
        if radius:
            dilated = set(occupied)
            for x, y in occupied:
                for dx in range(-radius, radius + 1):
                    for dy in range(-radius, radius + 1):
                        if max(abs(dx), abs(dy)) <= radius:
                            dilated.add((x + dx, y + dy))
            occupied = dilated

        free -= occupied
        return occupied, free

    def _bounds(self, occupied=None, free=None):
        cells = set(self.grid)
        if occupied: cells |= set(occupied)
        if free: cells |= set(free)
        points = [(s.map_x, s.map_y) for s in self.samples]
        points += [(a["x"], a["y"]) for a in self.node_anchors.values()]
        points += [self._cell_to_world(*c) for c in cells]
        if not points:
            return -0.5, 0.5, -0.5, 0.5
        xs, ys = [p[0] for p in points], [p[1] for p in points]
        m = float(_map_cfg("MAP_EXPORT_MARGIN_M", 0.30))
        minx, maxx, miny, maxy = min(xs)-m, max(xs)+m, min(ys)-m, max(ys)+m
        if maxx-minx < 0.5:
            c=(minx+maxx)/2; minx,maxx=c-0.25,c+0.25
        if maxy-miny < 0.5:
            c=(miny+maxy)/2; miny,maxy=c-0.25,c+0.25
        return minx,maxx,miny,maxy

    def mark_exit(self, raw_x, raw_y, yaw_deg, heading_index=None, details=None):
        """Record a confirmed maze exit in map coordinates."""
        if not self.enabled or not self.initialized:
            return None
        sample = self.record_pose(
            raw_x, raw_y, yaw_deg,
            heading_index=heading_index, mode="EXIT_FOUND", force=True,
        )
        if sample is None:
            return None
        self.exit_event = {
            "time_sec": float(sample.time_sec),
            "sample_index": int(sample.index),
            "map_x": float(sample.map_x),
            "map_y": float(sample.map_y),
            "theta_deg": float(sample.theta_deg),
            "heading_index": int(sample.heading_index),
            "details": dict(details or {}),
        }
        return self.exit_event

    # --------------------------------------------------------
    # Exports
    # --------------------------------------------------------
    def save_all(self, rebuild=True, quiet=False):
        if not self.enabled or not self.initialized:
            return
        os.makedirs(self.output_dir, exist_ok=True)
        if rebuild:
            self._rebuild_grid()
        self._save_csvs()
        self._save_jsons()
        self._save_svg()
        self._try_save_png()
        if not quiet:
            print(
                f"MAPPER SAVED: {self.output_dir} | samples={len(self.samples)} "
                f"wall_points={len(self.wall_points)} cells={len(self.grid)} "
                f"IRconfirm={self.ir_confirm_count} IRfallback={self.ir_fallback_count}"
            )
            print(f"  -> {os.path.join(self.output_dir, 'maze_map.png')}")
            print(f"  -> {os.path.join(self.output_dir, 'maze_map.svg')}")

    def _save_csvs(self):
        with open(os.path.join(self.output_dir, "trajectory.csv"), "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["index","time_sec","raw_x","raw_y","yaw_deg","map_x","map_y","theta_deg","heading_index","front_cm","left_cm","right_cm","ir","mode"])
            for s in self.samples:
                w.writerow([s.index,f"{s.time_sec:.6f}",f"{s.raw_x:.6f}",f"{s.raw_y:.6f}",f"{s.yaw_deg:.6f}",f"{s.map_x:.6f}",f"{s.map_y:.6f}",f"{s.theta_deg:.4f}",s.heading_index,"" if s.front_cm is None else f"{s.front_cm:.3f}","" if s.left_cm is None else f"{s.left_cm:.3f}","" if s.right_cm is None else f"{s.right_cm:.3f}","" if s.ir_value is None else s.ir_value,s.mode])
        with open(os.path.join(self.output_dir, "wall_points.csv"), "w", newline="", encoding="utf-8") as f:
            w=csv.writer(f); w.writerow(["sample_index","time_sec","sensor","x_m","y_m","distance_cm"])
            for p in self.wall_points:
                w.writerow([p["sample_index"],f'{p["time_sec"]:.6f}',p["sensor"],f'{p["x"]:.6f}',f'{p["y"]:.6f}',f'{p["distance_cm"]:.3f}'])
        with open(os.path.join(self.output_dir, "occupancy_grid.csv"), "w", newline="", encoding="utf-8") as f:
            w=csv.writer(f); w.writerow(["gx","gy","x_m","y_m","score","state"])
            for (gx,gy),score in sorted(self.grid.items()):
                x,y=self._cell_to_world(gx,gy); w.writerow([gx,gy,f"{x:.6f}",f"{y:.6f}",score,self.cell_state(score)])

    def _save_jsons(self):
        occupied, free = self._display_sets()
        cells=[]
        for c,s in sorted(self.grid.items()):
            st=self.cell_state(s)
            if st != "UNKNOWN": cells.append(dict(gx=c[0],gy=c[1],score=s,state=st))
        with open(os.path.join(self.output_dir,"occupancy_grid.json"),"w",encoding="utf-8") as f:
            json.dump(dict(resolution_m=float(_map_cfg("MAP_RESOLUTION_M",0.025)),cells=cells),f,indent=2)
        with open(os.path.join(self.output_dir,"nodes.json"),"w",encoding="utf-8") as f:
            json.dump(dict(anchors=self.node_anchors,events=self.node_events),f,indent=2)
        with open(os.path.join(self.output_dir,"loop_closures.json"),"w",encoding="utf-8") as f:
            json.dump(self.loop_closures,f,indent=2)
        with open(os.path.join(self.output_dir,"exit.json"),"w",encoding="utf-8") as f:
            json.dump(self.exit_event,f,indent=2)
        states={"FREE":0,"OCCUPIED":0,"UNKNOWN":0}
        for s in self.grid.values(): states[self.cell_state(s)] += 1
        with open(os.path.join(self.output_dir,"mapping_summary.json"),"w",encoding="utf-8") as f:
            json.dump(dict(
                coordinate_convention="+Y=NORTH, +X=EAST, 0deg=NORTH +90deg=EAST",
                resolution_m=float(_map_cfg("MAP_RESOLUTION_M",0.025)),
                samples=len(self.samples), wall_points=len(self.wall_points), grid_states=states,
                display_occupied_cells=len(occupied), display_free_cells=len(free),
                nodes=len(self.node_anchors), loop_closures=len(self.loop_closures),
                exit_found=self.exit_event is not None,
                ir_wall_level=int(_map_cfg("MAP_IR_WALL_LEVEL",0)),
                ir_confirm_count=self.ir_confirm_count, ir_fallback_count=self.ir_fallback_count,
                yaw_fallback_count=self.yaw_fallback_count,
                position_rotation_deg=self.position_rotation_deg,
                position_auto_aligned=self.position_auto_aligned,
            ),f,indent=2)

    @staticmethod
    def _xml(text):
        return str(text).replace("&","&amp;").replace("<","&lt;").replace(">","&gt;").replace('"',"&quot;")

    def _save_svg(self):
        occupied, free = self._display_sets()
        minx,maxx,miny,maxy = self._bounds(occupied,free)
        wm,hm=maxx-minx,maxy-miny
        ppm=float(_map_cfg("MAP_SVG_PX_PER_M",420.0))
        W=int(clamp(wm*ppm,600,2200)); H=int(clamp(hm*ppm,600,2200))
        sx=lambda x:(x-minx)/wm*W
        sy=lambda y:H-(y-miny)/hm*H
        res=float(_map_cfg("MAP_RESOLUTION_M",0.025))
        cw=max(1.0,res/wm*W); ch=max(1.0,res/hm*H)
        p=[]
        p.append('<?xml version="1.0" encoding="UTF-8"?>')
        p.append(f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">')
        # ROS/SLAM-like palette: gray unknown, white observed free, dark occupied.
        p.append('<rect width="100%" height="100%" fill="#bfc3c7"/>')
        for gx,gy in free:
            x,y=self._cell_to_world(gx,gy)
            p.append(f'<rect x="{sx(x)-cw/2:.2f}" y="{sy(y)-ch/2:.2f}" width="{cw+0.6:.2f}" height="{ch+0.6:.2f}" fill="#f7f7f7"/>')
        for gx,gy in occupied:
            x,y=self._cell_to_world(gx,gy)
            p.append(f'<rect x="{sx(x)-cw/2:.2f}" y="{sy(y)-ch/2:.2f}" width="{cw+0.7:.2f}" height="{ch+0.7:.2f}" fill="#202428"/>')
        if bool(_map_cfg("MAP_DRAW_TRAJECTORY",True)) and len(self.samples)>1:
            pts=" ".join(f"{sx(s.map_x):.2f},{sy(s.map_y):.2f}" for s in self.samples)
            p.append(f'<polyline points="{pts}" fill="none" stroke="#2463b5" stroke-width="2.4" stroke-linecap="round" stroke-linejoin="round" opacity="0.92"/>')
        if self.samples:
            s=self.samples[0]
            p.append(f'<circle cx="{sx(s.map_x):.2f}" cy="{sy(s.map_y):.2f}" r="6" fill="#19a15f" stroke="white" stroke-width="2"/>')
            e=self.samples[-1]
            p.append(f'<circle cx="{sx(e.map_x):.2f}" cy="{sy(e.map_y):.2f}" r="5" fill="#f59e0b" stroke="white" stroke-width="1.5"/>')
        if bool(_map_cfg("MAP_DRAW_NODES",True)):
            for nid,a in sorted(self.node_anchors.items()):
                x,y=sx(a["x"]),sy(a["y"])
                p.append(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="4" fill="#d14343" stroke="white" stroke-width="1.2"/>')
                p.append(f'<text x="{x+6:.2f}" y="{y-6:.2f}" font-family="Arial,sans-serif" font-size="11" font-weight="700" fill="#111827">{self._xml(nid)}</text>')
        if self.exit_event is not None and bool(_map_cfg("MAP_DRAW_EXIT",True)):
            x,y=sx(self.exit_event["map_x"]),sy(self.exit_event["map_y"])
            p.append(f'<polygon points="{x:.2f},{y-9:.2f} {x+9:.2f},{y:.2f} {x:.2f},{y+9:.2f} {x-9:.2f},{y:.2f}" fill="#7c3aed" stroke="white" stroke-width="2"/>')
            p.append(f'<text x="{x+12:.2f}" y="{y-9:.2f}" font-family="Arial,sans-serif" font-size="12" font-weight="700" fill="#5b21b6">EXIT</text>')
        p.append('<rect x="14" y="14" width="235" height="108" rx="9" fill="white" fill-opacity="0.92" stroke="#9ca3af"/>')
        p.append('<text x="27" y="36" font-family="Arial,sans-serif" font-size="12" font-weight="700" fill="#111827">SLAM-style Maze Map</text>')
        p.append('<text x="27" y="55" font-family="Arial,sans-serif" font-size="11" fill="#374151">Black: wall  White: observed free</text>')
        p.append('<text x="27" y="72" font-family="Arial,sans-serif" font-size="11" fill="#374151">Gray: unknown  Blue: trajectory</text>')
        p.append('<text x="27" y="89" font-family="Arial,sans-serif" font-size="11" fill="#374151">Red: junction  Green: start</text>')
        p.append('<text x="27" y="106" font-family="Arial,sans-serif" font-size="11" fill="#374151">Purple diamond: confirmed exit</text>')
        p.append('</svg>')
        Path = __import__('pathlib').Path
        Path(os.path.join(self.output_dir,"maze_map.svg")).write_text("\n".join(p),encoding="utf-8")

    def _try_save_png(self):
        if not bool(_map_cfg("MAP_EXPORT_PNG",True)):
            return
        try:
            import matplotlib.pyplot as plt
            import numpy as np
            from matplotlib.colors import ListedColormap
        except Exception as exc:
            # SVG is always available; PNG is optional.
            return

        occupied, free = self._display_sets()
        minx,maxx,miny,maxy = self._bounds(occupied,free)
        res=float(_map_cfg("MAP_RESOLUTION_M",0.025))
        gx0=int(math.floor(minx/res)); gx1=int(math.ceil(maxx/res))
        gy0=int(math.floor(miny/res)); gy1=int(math.ceil(maxy/res))
        width=max(1,gx1-gx0+1); height=max(1,gy1-gy0+1)
        # 0 unknown, 1 free, 2 occupied
        img=np.zeros((height,width),dtype=np.uint8)
        for gx,gy in free:
            if gx0<=gx<=gx1 and gy0<=gy<=gy1: img[gy-gy0,gx-gx0]=1
        for gx,gy in occupied:
            if gx0<=gx<=gx1 and gy0<=gy<=gy1: img[gy-gy0,gx-gx0]=2

        cmap=ListedColormap(["#bfc3c7","#f8f8f8","#171a1d"])
        fig,ax=plt.subplots(figsize=(8.5,8.5))
        ax.imshow(img,origin="lower",extent=[gx0*res,(gx1+1)*res,gy0*res,(gy1+1)*res],interpolation="nearest",cmap=cmap,vmin=0,vmax=2)
        if bool(_map_cfg("MAP_DRAW_TRAJECTORY",True)) and self.samples:
            ax.plot([s.map_x for s in self.samples],[s.map_y for s in self.samples],linewidth=1.8,label="trajectory")
            ax.scatter([self.samples[0].map_x],[self.samples[0].map_y],s=45,label="start",zorder=5)
            ax.scatter([self.samples[-1].map_x],[self.samples[-1].map_y],s=35,label="current/end",zorder=5)
        if bool(_map_cfg("MAP_DRAW_NODES",True)):
            for nid,a in self.node_anchors.items():
                ax.scatter([a["x"]],[a["y"]],s=20,zorder=5)
                ax.text(a["x"],a["y"]," "+str(nid),fontsize=7,zorder=6)
        if self.exit_event is not None and bool(_map_cfg("MAP_DRAW_EXIT",True)):
            ax.scatter([self.exit_event["map_x"]],[self.exit_event["map_y"]],s=90,marker="*",label="exit",zorder=7)
            ax.text(self.exit_event["map_x"],self.exit_event["map_y"]," EXIT",fontsize=8,fontweight="bold",zorder=8)
        ax.set_aspect("equal",adjustable="box")
        ax.set_xlabel("East / X (m)")
        ax.set_ylabel("North / Y (m)")
        ax.set_title("RoboMaster Maze Occupancy Map")
        ax.grid(False)
        if self.samples: ax.legend(loc="best",fontsize=8,framealpha=0.9)
        fig.tight_layout()
        fig.savefig(os.path.join(self.output_dir,"maze_map.png"),dpi=int(_map_cfg("MAP_PNG_DPI",220)))
        plt.close(fig)



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
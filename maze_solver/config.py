"""Configuration for RoboMaster maze solver V9."""

# ============================================================
# GENERAL
# ============================================================
ENABLE_MOTION = True

# ============================================================
# SENSOR CONFIG
# ============================================================
# Sharp side sensors
SHARP_LEFT_ID = 2
SHARP_RIGHT_ID = 3
SENSOR_PORT = 1

# V11: digital IR side confirmation sensors
# User installation: Hub/Adapter ID 1 = LEFT, ID 4 = RIGHT, both on Port 1.
IR_LEFT_ID = 1
IR_RIGHT_ID = 4
IR_PORT = 1

# Polarity is configurable because digital IR modules vary.
# Current default follows the observed setup/log assumption: level 1 = WALL.
# If a hand test shows the opposite, change the corresponding value to 0.
IR_LEFT_WALL_LEVEL = 1
IR_RIGHT_WALL_LEVEL = 1
ENABLE_IR_SIDE_FUSION = True
IR_FILTER_SIZE = 3
IR_MIN_SAMPLES = 3
IR_ADC_FALLBACK_THRESHOLD = 300
IR_CONFLICT_RESCAN_SAMPLES = 3

# Backward-compatible old name used by older code paths.
IR_LEFT_FRONT_ID = IR_LEFT_ID

SHARP_FILTER_SIZE = 3
TOF_FILTER_SIZE = 3
SHARP_EMA_NEW_WEIGHT = 0.6
SHARP_EMA_OLD_WEIGHT = 0.4
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

# Wall-following hysteresis
SIDE_WALL_ENTER_CM = 28.0
SIDE_WALL_EXIT_CM = 32.0
SIDE_WALL_DETECT_CM = 30.0  # compatibility/debug
SIDE_OPEN_DIFFERENCE_CM = 20.0

# ============================================================
# V9 SIDE OPENING BORDERLINE / SCHMITT TRIGGER
# ============================================================
# <= 14 cm : BLOCKED แน่นอน
# 14-20 cm : BORDERLINE -> คงสถานะก่อนหน้า
# >= 20 cm : OPEN แน่นอน
SIDE_BLOCKED_MAX_CM = 14.0
SIDE_OPEN_MIN_CM = 20.0
EXPLORATION_SIDE_OPEN_CM = SIDE_OPEN_MIN_CM

# V11 fusion policy:
#   Sharp <=14 cm  -> BLOCKED regardless of IR (Sharp strong evidence)
#   Sharp 14-20 cm -> IR decides when stable; otherwise hold previous state
#   Sharp >=20 cm  -> OPEN regardless of IR; IR=WALL is flagged CONFLICT
# This keeps IR as confirmation rather than allowing one digital sensor to
# override a strong Sharp measurement.

# Front path must be clearly long enough to count as a straight exit.
EXPLORATION_FRONT_OPEN_CM = 35.0

# ============================================================
# ROBOT SPEED
# ============================================================
FORWARD_SPEED = 0.15
MIN_FORWARD_SPEED = 0.05
UNKNOWN_FRONT_SPEED = 0.0
ESCAPE_FORWARD_SPEED = 0.04

# ============================================================
# DIRECTION
# ============================================================
Y_DIR_SIGN = 1
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

ENABLE_YAW_CORRECTION = True
ATTITUDE_FREQ_HZ = 20
YAW_SETTLE_SEC = 0.08
YAW_TOLERANCE_DEG = 3.0
YAW_MAX_CORRECTION_DEG = 15.0
TURN_CORRECTION_SPEED = 35

# ============================================================
# HEADING HOLD / STRAIGHT-LINE CONTROL
# ============================================================
# Real robot logs showed different sign conventions for these APIs.
DEFAULT_MOVE_TO_YAW_SIGN = -1
DEFAULT_DRIVE_TO_YAW_SIGN = +1

ENABLE_HEADING_HOLD = True
HEADING_KP_Z = 1.5
HEADING_MAX_Z_SPEED = 12.0
HEADING_DEADBAND_DEG = 0.8
HEADING_RECOVER_TRIGGER_DEG = 8.0
HEADING_RECOVER_RELEASE_DEG = 2.0
HEADING_RECOVER_MAX_Z_SPEED = 18.0

ENABLE_ABSOLUTE_HEADING_ALIGN = True
HEADING_ALIGN_TOLERANCE_DEG = 2.0
HEADING_ALIGN_TIMEOUT_SEC = 1.20
HEADING_ALIGN_LOOP_SEC = 0.04

# ============================================================
# JUNCTION CENTERING / CORNER GEOMETRY
# ============================================================
ENABLE_JUNCTION_CREEP = True
JUNCTION_CREEP_SPEED = 0.07
JUNCTION_CREEP_DISTANCE_M = 0.06
JUNCTION_CREEP_MAX_SEC = 1.20
JUNCTION_CREEP_ABORT_FRONT_CM = 16.0
JUNCTION_CREEP_LOOP_SEC = 0.05

ENABLE_CORNER_TURN_SETUP = True
CORNER_TURN_SETUP_SPEED = 0.05
# V8: allow deeper setup; ToF target normally stops it first.
CORNER_TURN_SETUP_DISTANCE_M = 0.14
CORNER_TURN_SETUP_MAX_SEC = 3.00
CORNER_TURN_FRONT_TARGET_CM = 14.0
CORNER_TURN_FRONT_HARD_STOP_CM = 10.5
CORNER_TURN_SETUP_LOOP_SEC = 0.04

# After a 90-degree turn, crawl out if the inside side is still too close.
ENABLE_POST_TURN_CLEARANCE = True
POST_TURN_CLEARANCE_TRIGGER_CM = 6.5
POST_TURN_CLEARANCE_RELEASE_CM = 7.5
POST_TURN_CLEARANCE_FORWARD_SPEED = 0.045
POST_TURN_CLEARANCE_Y_SPEED = 0.035
POST_TURN_CLEARANCE_MAX_DISTANCE_M = 0.07
POST_TURN_CLEARANCE_MAX_SEC = 1.50
POST_TURN_CLEARANCE_FRONT_STOP_CM = 12.0
POST_TURN_CLEARANCE_LOOP_SEC = 0.04

# ============================================================
# JUNCTION REARM
# ============================================================
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
DECISION_SCAN_SAMPLES = 3
DECISION_SCAN_INTERVAL_SEC = 0.04
JUNCTION_CONFIRM_SAMPLES = 3
JUNCTION_REARM_SAMPLES = 4
JUNCTION_SETTLE_SEC = 0.15

# ============================================================
# TRÉMAUX / DFS
# ============================================================
NODE_MATCH_RADIUS_M = 0.18

# V10: when traversing an already-known edge, the graph already knows which
# node should be reached next. Allow a wider radius ONLY for that exact target
# to tolerate different detection positions on outbound vs backtrack runs.
EXPECTED_NODE_MATCH_RADIUS_M = 0.40
# If no other normal-radius node is present, a known edge may trust its target
# a bit farther to absorb odometry drift on long backtracks.
EXPECTED_NODE_FALLBACK_RADIUS_M = 0.65

NODE_POSITION_UPDATE_ALPHA = 0.20
MAX_EDGE_VISITS = 2
EXPLORATION_PREFERENCE = ("FRONT", "LEFT", "RIGHT", "BACK")
POSE_FREQ_HZ = 20
POSE_WAIT_SEC = 1.0
SAVE_MAZE_MEMORY = True
MAZE_MEMORY_FILE = "maze_memory.json"
STOP_WHEN_EXPLORATION_COMPLETE = True

# ============================================================
# V7 FEEDBACK TURN / WATCHDOG
# ============================================================
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
TURN_ACTION_TIMEOUT_90_SEC = 3.50
TURN_ACTION_TIMEOUT_180_SEC = 6.00

# ============================================================
# SHARP CALIBRATION: ADC -> CM
# ============================================================
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
CALIBRATION_SHARP2 = CALIBRATION_SHARP_LEFT

"""RoboMaster Maze Explorer V12.5 — FIELD-READY SINGLE-FILE BUILD.

Generated from the modular V11.3 project.  Everything required by the maze
solver is kept in this one Python file so emergency edits can be made without
chasing cross-file imports.

Run:
    python maze_monster.py --guide known_route.json  # route OR saved maze; soft guide + DFS
    python maze_monster.py --no-guide                # pure Trémaux/DFS
    python maze_monster.py --test-ir                 # focused front IR test
    python maze_monster.py --sweep                   # raw Sensor Adapter port sweep
"""
from __future__ import annotations
import csv
import json
import math
import os
import statistics
import threading
import time
from collections import deque
from dataclasses import asdict, dataclass, field
from dataclasses import dataclass as _map_dataclass
from typing import Dict, List, Optional, Tuple
ENABLE_MOTION = True
PROGRAM_VERSION = 'V12.5.2_MAP_EXIT_LOCK_AND_ENTRY_BARRIER'
IR_FRONT_LEFT_ID = 1
SHARP_LEFT_ID = 2
SHARP_RIGHT_ID = 3
IR_FRONT_RIGHT_ID = 4
SENSOR_PORT = 1
FRONT_IR_DIGITAL_METHOD = 'get_io'
FRONT_IR_CLEAR_LEVEL = 1
IR_LEFT_FRONT_ID = IR_FRONT_LEFT_ID
ENABLE_DUAL_FRONT_IR_GUARD = True
FRONT_IR_ACTIVE_LEVEL = 0
FRONT_IR_AUTO_LEARN_CLEAR_LEVEL = False
FRONT_IR_AUTO_LEARN_SAMPLES = 12
FRONT_IR_AUTO_LEARN_INTERVAL_SEC = 0.025
FRONT_IR_AUTO_LEARN_REQUIRE_STABLE_RATIO = 0.7
FRONT_IR_BLOCK_MODE = 'BOTH'
FRONT_IR_ENTER_SAMPLES = 1
FRONT_IR_RELEASE_SAMPLES = 2
FRONT_IR_SYNTHETIC_BLOCK_CM = 14.0
FRONT_IR_PRINT_STATE_CHANGES = True
FRONT_IR_ADC_FUSION_ENABLED = False
FRONT_IR_ADC_REFRESH_SEC = 0.03
FRONT_IR_ADC_HIT_DELTA = 220.0
FRONT_IR_ADC_PRINT_IN_DEBUG = False
ENABLE_FRONT_IR_COLLISION_AVOIDANCE = True
FRONT_IR_SIDE_ENTER_SAMPLES = 1
FRONT_IR_SIDE_RELEASE_SAMPLES = 2
FRONT_IR_SINGLE_AVOID_FORWARD_SPEED = 0.035
FRONT_IR_SINGLE_AVOID_Y_SPEED = 0.045
FRONT_IR_AVOID_OPPOSITE_SIDE_MIN_CM = 6.5
ENABLE_TURN_IR_COLLISION_GUARD = True
TURN_IR_ARM_AFTER_DEG = 0.5
TURN_IR_MAX_RECOVERIES = 4
TURN_IR_SINGLE_ESCAPE_M = 0.04
TURN_IR_BOTH_ESCAPE_M = 0.055
TURN_IR_RECOVERY_MIN_M = 0.018
TURN_IR_ESCAPE_Y_SPEED = 0.045
TURN_IR_ESCAPE_BACK_SPEED = 0.045
TURN_IR_SINGLE_BACK_BIAS_SPEED = 0.025
TURN_IR_OPPOSITE_SIDE_MIN_CM = 6.0
TURN_IR_RECOVERY_MAX_SEC = 1.25
TURN_IR_RECOVERY_LOOP_SEC = 0.04
TURN_IR_RECOVERY_SETTLE_SEC = 0.1
TURN_IR_RECHECK_SAMPLES = 2
TURN_IR_ACTION_STEP_DEG = 7.5
SHARP_FILTER_SIZE = 3
TOF_FILTER_SIZE = 3
SHARP_EMA_NEW_WEIGHT = 0.6
SHARP_EMA_OLD_WEIGHT = 0.4
TOF_STALE_SEC = 0.4
SHARP_STALE_HOLD_SEC = 1.5
SHARP_INVALID_WARN_INTERVAL_SEC = 1.0
SHARP_SENSOR_RECOVERY_DELAY_SEC = 0.08
TARGET_LEFT_CM = 8.0
TARGET_RIGHT_CM = 8.0
SLOW_FRONT_CM = 18.0
STOP_FRONT_CM = 15.0
SIDE_DEAD_END_CM = 15.0
SIDE_TOO_CLOSE_CM = 5.5
SIDE_WALL_ENTER_CM = 28.0
SIDE_WALL_EXIT_CM = 32.0
SIDE_WALL_DETECT_CM = 30.0
SIDE_OPEN_DIFFERENCE_CM = 20.0
FORWARD_SPEED = 0.2
MIN_FORWARD_SPEED = 0.05
UNKNOWN_FRONT_SPEED = 0.0
ESCAPE_FORWARD_SPEED = 0.04
Y_DIR_SIGN = 1
Z_DIR_SIGN = 1
SIDE_KP_STRAFE = 0.012
SIDE_MAX_Y = 0.05
SIDE_DEADBAND_CM = 1.0
CENTER_TRIGGER_CM = 2.0
CENTER_RELEASE_CM = 0.7
CENTER_HOLD_SEC = 0.3
CENTER_KP_STRAFE = 0.009
CENTER_MAX_Y = 0.04
OWNER_PERSIST_THROUGH_SINGLE_WALL = True
OWNER_PERSIST_OPEN_SEC = 0.75
CENTER_OWNER_FORCE_SWITCH_CM = 4.0
ESCAPE_Y_SPEED = 0.07
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
HEADING_ALIGN_TIMEOUT_SEC = 1.2
HEADING_ALIGN_LOOP_SEC = 0.04
ENABLE_CORRIDOR_HEADING_CALIBRATION = False
CORRIDOR_CAL_MIN_TRAVEL_M = 0.18
CORRIDOR_CAL_MIN_WALL_CM = 5.8
CORRIDOR_CAL_MAX_WALL_CM = 16.0
CORRIDOR_CAL_MAX_ESTIMATE_DEG = 7.0
CORRIDOR_CAL_ALPHA = 0.12
CORRIDOR_CAL_MAX_STEP_DEG = 0.65
CORRIDOR_CAL_MIN_FRONT_CM = 40.0
CORRIDOR_CAL_LOG_MIN_STEP_DEG = 0.12

# V12.5 WALL PARALLEL ASSIST. The old corridor yaw learner remains disabled;
# this replacement requires repeated, geometrically consistent Sharp trends
# before trimming the absolute heading reference. Dual-wall evidence is trusted
# faster; single-wall evidence is deliberately slower to reject wall-shape noise.
ENABLE_WALL_PARALLEL_ASSIST = True
WALL_PARALLEL_MIN_TRAVEL_M = 0.12
WALL_PARALLEL_MIN_WALL_CM = 5.8
WALL_PARALLEL_MAX_WALL_CM = 18.0
WALL_PARALLEL_MIN_ESTIMATE_DEG = 0.9
WALL_PARALLEL_MAX_ESTIMATE_DEG = 9.0
WALL_PARALLEL_DUAL_AGREE_DEG = 4.0
WALL_PARALLEL_DUAL_CONFIRM_WINDOWS = 2
WALL_PARALLEL_SINGLE_CONFIRM_WINDOWS = 3
WALL_PARALLEL_DUAL_GAIN = 0.18
WALL_PARALLEL_SINGLE_GAIN = 0.10
WALL_PARALLEL_MAX_STEP_DEG = 0.75
WALL_PARALLEL_LOG_MIN_STEP_DEG = 0.12

ENABLE_JUNCTION_CREEP = True
JUNCTION_CREEP_SPEED = 0.07
JUNCTION_CREEP_DISTANCE_M = 0.06
JUNCTION_CREEP_MAX_SEC = 1.2
JUNCTION_CREEP_ABORT_FRONT_CM = 16.0
JUNCTION_CREEP_LOOP_SEC = 0.05
ENABLE_CORNER_TURN_SETUP = True
CORNER_TURN_SETUP_SPEED = 0.05
CORNER_TURN_SETUP_DISTANCE_M = 0.14
CORNER_TURN_SETUP_MAX_SEC = 3.0
CORNER_TURN_FRONT_TARGET_CM = 11.0
CORNER_TURN_FRONT_HARD_STOP_CM = 10.5
CORNER_TURN_SETUP_LOOP_SEC = 0.04
ENABLE_POST_TURN_CLEARANCE = True
POST_TURN_CLEARANCE_TRIGGER_CM = 6.5
POST_TURN_CLEARANCE_RELEASE_CM = 7.5
POST_TURN_CLEARANCE_FORWARD_SPEED = 0.045
POST_TURN_CLEARANCE_Y_SPEED = 0.035
POST_TURN_CLEARANCE_MAX_DISTANCE_M = 0.10
POST_TURN_CLEARANCE_MAX_SEC = 2.0
POST_TURN_CLEARANCE_FRONT_STOP_CM = 12.0
POST_TURN_CLEARANCE_LOOP_SEC = 0.04
JUNCTION_REARM_MIN_DISTANCE_M = 0.2
JUNCTION_REARM_DISTANCE_M = 0.32
JUNCTION_REARM_TIMEOUT_SEC = 2.5
JUNCTION_REARM_EMERGENCY_SEC = 0.25
AFTER_TURN_DELAY_SEC = 0.12
LOOP_DELAY_SEC = 0.05
DRIVE_TIMEOUT_SEC = 0.15
DECISION_SCAN_SAMPLES = 3
DECISION_SCAN_INTERVAL_SEC = 0.03
SIDE_OPEN_ENTER_CM = 18.0
SIDE_OPEN_EXIT_CM = 15.0
EXPLORATION_SIDE_OPEN_CM = SIDE_OPEN_ENTER_CM
ENABLE_OPENING_ZONE_DETECTION = True
OPENING_ZONE_ENTER_SAMPLES = 3
OPENING_ZONE_EXIT_SAMPLES = 3
OPENING_ZONE_MIN_LENGTH_M = 0.1
OPENING_ZONE_MAX_LENGTH_M = 0.7
ENABLE_OPENING_ZONE_CENTERING = True
OPENING_ZONE_CENTERING_SPEED = 0.07
OPENING_ZONE_CENTERING_MAX_BACKTRACK_M = 0.48
OPENING_ZONE_CENTERING_MAX_SEC = 7.0
OPENING_ZONE_CENTERING_LOOP_SEC = 0.04
OPENING_ZONE_CENTER_REVERSE_BIAS_M = 0.035
ENABLE_TURN_ENTRY_REALIGN = True
TURN_ENTRY_OPEN_CM = 17.5
TURN_ENTRY_CONFIRM_SAMPLES = 2
TURN_ENTRY_SEARCH_SPEED = 0.045
TURN_ENTRY_MAX_BACKTRACK_M = 0.13
TURN_ENTRY_MAX_SEC = 3.0
TURN_ENTRY_FRONT_SAFE_CM = 11.5
TURN_ENTRY_LOOP_SEC = 0.04
ENABLE_INTERSECTION_WINDOW = True
INTERSECTION_WINDOW_LOOKAHEAD_M = 0.18
INTERSECTION_WINDOW_MAX_M = 0.55
INTERSECTION_MIN_OPEN_SAMPLES = 2
INTERSECTION_FRONT_OPEN_CM = 35.0
INTERSECTION_SIDE_OPEN_CM = SIDE_OPEN_ENTER_CM

# ==================== V12.5 ADAPTIVE JUNCTION REGION ====================
# A FRONT drive-through must not blind the detector for a fixed 0.32 m.
# Re-arm as soon as a *real corridor* has been spatially reacquired, while
# keeping the latch through a broad/open junction where the side walls have
# not returned yet. This lets a second branch ~0.30 m later become its own node.
ENABLE_ADAPTIVE_JUNCTION_REGION = True
JUNCTION_REGION_WALL_REACQUIRE_CM = 16.5
JUNCTION_REGION_CLEAR_M = 0.12
JUNCTION_REGION_FRONT_REARM_CLEAR_M = 0.12
JUNCTION_REGION_OBSERVE_SPEED = 0.12
JUNCTION_REGION_DEBUG = True

# ==================== V12 ROLLING JUNCTION DECISION ====================
# High-confidence intersection windows are planned while the chassis is still
# moving. FRONT can drive through without a stop. LEFT/RIGHT/BACK stop only
# after the planner has selected them. Ambiguous evidence falls back to the
# conservative stopped-scan pipeline.
ENABLE_ROLLING_JUNCTION_DECISION = True
ROLLING_SIDE_MIN_SAMPLES = 6
ROLLING_FRONT_MIN_SAMPLES = 3
DECISION_MEMORY_SIDE_MIN_SAMPLES = 6
ROLLING_FRONT_STRONG_MAX_CM = 45.0
ROLLING_FRONT_CONTINUE_SPEED = 0.15
ROLLING_TURN_MAX_BACKTRACK_M = 0.40
ROLLING_TURN_BACKTRACK_MARGIN_M = 0.05
# V12.1: rolling detection may finish ~0.25-0.35 m beyond the mouth.
# Do NOT reuse the old V11 0.045 m/s / 3 s realign budget: that can only
# travel ~0.135 m and therefore guarantees TURN_ENTRY_REALIGN failure.
ROLLING_TURN_REALIGN_SPEED = 0.08
ROLLING_TURN_REALIGN_MAX_SEC = 5.0

# V12.2: mecanum turn-pocket positioning.  For LEFT/RIGHT decisions the robot
# first returns to the remembered opening centre (rolling junctions), then
# shifts its chassis centre a few centimetres INTO the chosen branch before
# rotating 90 degrees.  This increases swept-body clearance without over-turning.
ENABLE_TURN_POCKET_POSITIONING = False  # V12.4: no pre-turn lateral nudge
TURN_POCKET_CENTER_ROLLING_OPENING = True
TURN_POCKET_STRAFE_M = 0.045
TURN_POCKET_STRAFE_SPEED = 0.050
TURN_POCKET_MAX_SEC = 1.50
TURN_POCKET_LOOP_SEC = 0.04
TURN_POCKET_REQUIRED_OPEN_CM = 17.5
TURN_POCKET_SIDE_HARD_STOP_CM = 6.0

# V12.3: after a 90-degree turn, do NOT hand control directly back to
# full-speed wall-follow.  Crawl straight out of the junction first so the
# side Sharp sensors are looking at the new corridor rather than the old corner.
ENABLE_POST_TURN_CORRIDOR_ACQUIRE = True
POST_TURN_ACQUIRE_DISTANCE_M = 0.12
POST_TURN_ACQUIRE_SPEED = 0.060
POST_TURN_ACQUIRE_MAX_SEC = 3.0
POST_TURN_ACQUIRE_LOOP_SEC = 0.04
POST_TURN_ACQUIRE_FRONT_STOP_CM = 15.0
POST_TURN_ACQUIRE_SIDE_HARD_STOP_CM = 5.5
POST_TURN_ACQUIRE_ESCAPE_Y_SPEED = 0.050
POST_TURN_ACQUIRE_ESCAPE_BACK_SPEED = 0.030
POST_TURN_ACQUIRE_CLEAR_SAMPLES = 2

ROLLING_SAVE_MEMORY_ON_FRONT = False
ROLLING_DEBUG = True
ENABLE_OPEN_AREA_HEADING_HOLD = True
OPEN_AREA_SIDE_ENTER_CM = 35.0
OPEN_AREA_SIDE_EXIT_CM = 28.0
OPEN_AREA_FRONT_MIN_CM = 45.0
OPEN_AREA_ENTER_SAMPLES = 4
OPEN_AREA_EXIT_SAMPLES = 3
ENABLE_EXIT_DETECTION = True
STOP_WHEN_EXIT_FOUND = True
EXIT_FRONT_START_CM = 165.0
EXIT_FRONT_KEEP_CM = 130.0
EXIT_SIDE_START_CM = 55.0
EXIT_SIDE_KEEP_CM = 42.0
EXIT_START_SAMPLES = 4
EXIT_CONFIRM_STRONG_SAMPLES = 8
EXIT_CONFIRM_DISTANCE_M = 0.6
EXIT_CONFIRM_MIN_SEC = 1.8
EXIT_MIN_RUNTIME_SEC = 5.0
EXIT_MIN_NODE_COUNT = 2
EXIT_MAX_HEADING_ERROR_DEG = 8.0
EXIT_CANDIDATE_SPEED = 0.12

# V12.5 EXIT PROOF V2: an exit is only committed after the normal junction
# detector has also had a chance to veto the same control cycle.  Require a
# sustained wall-free run measured from OPEN_AREA entry in addition to the
# original candidate displacement/time/sample checks.
ENABLE_EXIT_PROOF_V2 = True
EXIT_OPEN_AREA_MIN_TRAVEL_M = 0.75
EXIT_READY_CONFIRM_SAMPLES = 2

ENABLE_START_GATE_GUARD = True
START_GATE_LEARN_DISTANCE_M = 0.12
START_GATE_HALF_WIDTH_M = 0.85
START_GATE_BLOCK_INNER_M = 0.2
START_GATE_RECOVERY_COOLDOWN_SEC = 1.0
# When returning toward the old entrance, do not wait for a wide opening window
# to finish after the robot has already crossed the branch mouth. Finalize the
# active junction window before the chassis reaches the virtual start barrier;
# the normal guide/DFS decision then chooses a mapped branch.
ENABLE_START_GATE_EARLY_DECISION_BARRIER = True
START_GATE_DECISION_TRIGGER_PROGRESS_M = 0.70
START_GATE_DECISION_TRIGGER_LATERAL_M = 0.85
START_GATE_DECISION_MIN_WINDOW_M = 0.08
# The chassis may be placed in a wall-free staging area before the physical
# entrance.  Until a corridor/landmark is acquired, drive inward with heading
# hold but suppress junction and exit detection so the staging-area boundary
# cannot be merged with the first real maze junction.
ENABLE_START_ENTRY_ACQUISITION = True
START_ENTRY_SEARCH_SPEED = 0.10
START_ENTRY_WALL_CM = 28.0
# Always make a small, slow forward nudge before treating ordinary sensor
# readings as the physical maze entrance.  This prevents the open staging area
# and an initial short ToF reflection from becoming the first topology node.
# A confirmed front IR hit or an extremely near ToF reading may arm early as a
# safety exception; the collision supervisor still has final authority.
START_ENTRY_NUDGE_DISTANCE_M = 0.05
START_ENTRY_NUDGE_SPEED = 0.06
START_ENTRY_NUDGE_MAX_SEC = 2.0
START_ENTRY_NUDGE_EMERGENCY_FRONT_CM = 6.0
START_ENTRY_SHARP_SAMPLES = 3
START_ENTRY_TOF_SAMPLES = 3
START_ENTRY_IR_SAMPLES = 2
START_ENTRY_MAX_TRAVEL_M = 1.50
START_ENTRY_MAX_SEC = 15.0
START_EXIT_REJECT_RADIUS_M = 0.9
START_EXIT_REJECT_INNER_PROGRESS_M = 0.45
START_EXIT_REJECT_LATERAL_M = 1.25
EXPLORATION_FRONT_OPEN_CM = 35.0
JUNCTION_CONFIRM_SAMPLES = 3
JUNCTION_REARM_SAMPLES = 4
NODE_MATCH_RADIUS_M = 0.24
EXPECTED_TARGET_MATCH_RADIUS_M = 0.42
ENABLE_TOPOLOGY_NODE_MATCH = True
TOPOLOGY_NODE_MATCH_RADIUS_M = 0.38
TOPOLOGY_NODE_MATCH_MIN_SHARED_EXITS = 1
TOPOLOGY_NODE_MATCH_STRICT_SHARED_EXITS = 2
NODE_POSITION_UPDATE_ALPHA = 0.05
NODE_POSITION_UPDATE_REJECT_M = 0.32
EDGE_DEPARTURE_NODE_FOOTPRINT_M = 0.32
MAX_EDGE_VISITS = 2
EXPLORATION_PREFERENCE = ('FRONT', 'LEFT', 'RIGHT', 'BACK')
ENABLE_FRONTIER_AWARE_DFS = True
ENABLE_REMEMBERED_LOCAL_FRONTIER = True
REMEMBERED_FRONTIER_MIN_SEEN = 1
ENABLE_STALE_FRONTIER_RETIRE = True
FRONTIER_STALE_MISS_LIMIT = 2
ENABLE_INTERMEDIATE_NODE_EDGE_SPLIT = True
ENABLE_ROUTE_LOOP_BREAK = True
ROUTE_REPEAT_LIMIT = 1
ENABLE_UNRESOLVED_EDGE_RECOVERY = True
UNRESOLVED_EDGE_MAX_VISITS = 2
ENABLE_WEIGHTED_PENDING_ROUTING = True
ROUTE_EDGE_BASE_COST = 1.0
ROUTE_EDGE_VISIT_PENALTY = 1.75
ROUTE_EDGE_HIGH_VISIT_EXTRA = 2.0
ROUTE_PENDING_UNRESOLVED_EXTRA = 1.25
DFS_STACK_MAX_LEN = 128
POSE_FREQ_HZ = 20
POSE_WAIT_SEC = 1.0
SAVE_MAZE_MEMORY = True
MAZE_MEMORY_FILE = 'maze_memory.json'
STOP_WHEN_EXPLORATION_COMPLETE = True
JUNCTION_SETTLE_SEC = 0.05
ENABLE_MAPPING = True
MAP_OUTPUT_DIR = 'mapping_output'
MAP_CLEAR_OUTPUT_ON_START = True
MAP_SWAP_RAW_XY = False
MAP_RAW_X_SIGN = -1.0
MAP_RAW_Y_SIGN = +1.0
MAP_POSITION_ROTATION_DEG = 0.0
MAP_AUTO_ALIGN_INITIAL_PATH = True
MAP_AUTO_ALIGN_MIN_TRAVEL_M = 0.18
MAP_AUTO_ALIGN_MAX_HEADING_INDEX = 0
MAP_YAW_RIGHT_SIGN = +1.0
MAP_YAW_CARDINAL_MAX_ERROR_DEG = 22.0
MAP_FALLBACK_TO_CARDINAL_HEADING = True
MAP_SENSOR_USE_CARDINAL_HEADING = True
MAP_MIN_RECORD_INTERVAL_SEC = 0.045
MAP_MAX_SAMPLES = 60000
MAP_RESOLUTION_M = 0.025
MAP_EVIDENCE_MIN = -30
MAP_EVIDENCE_MAX = +30
MAP_OCCUPIED_SCORE_THRESHOLD = 4
MAP_FREE_SCORE_THRESHOLD = -3
MAP_ROBOT_FREE_RADIUS_M = 0.11
MAP_ROBOT_FREE_SCORE = -3
MAP_FRONT_SENSOR_ANGLE_DEG = 0.0
MAP_LEFT_SENSOR_ANGLE_DEG = -90.0
MAP_RIGHT_SENSOR_ANGLE_DEG = +90.0
MAP_FRONT_SENSOR_FORWARD_M = 0.08
MAP_FRONT_SENSOR_RIGHT_M = 0.0
MAP_LEFT_SENSOR_FORWARD_M = 0.02
MAP_LEFT_SENSOR_RIGHT_M = -0.1
MAP_RIGHT_SENSOR_FORWARD_M = 0.02
MAP_RIGHT_SENSOR_RIGHT_M = +0.1
MAP_TOF_MIN_CM = 4.0
MAP_TOF_FREE_MAX_CM = 55.0
MAP_TOF_OCCUPIED_MAX_CM = 45.0
MAP_TOF_HIT_SCORE = 7
MAP_TOF_FREE_SCORE = -1
MAP_TOF_NO_HIT_FREE_MAX_CM = 28.0
MAP_SHARP_MIN_CM = 4.0
MAP_SHARP_FREE_MAX_CM = 24.0
MAP_SHARP_OCCUPIED_MAX_CM = 18.0
MAP_SHARP_HIT_SCORE = 5
MAP_SHARP_FREE_SCORE = -1
MAP_IR_WALL_LEVEL = 0
MAP_IR_CONFIRM_LEFT_SHARP = False
MAP_IR_CONFIRM_MAX_SHARP_CM = 22.0
MAP_IR_CONFIRM_SCORE = 4
MAP_IR_FALLBACK_ENABLED = False
MAP_IR_SENSOR_ANGLE_DEG = -45.0
MAP_IR_SENSOR_FORWARD_M = 0.08
MAP_IR_SENSOR_RIGHT_M = -0.07
MAP_IR_ASSUMED_RANGE_M = 0.12
MAP_IR_FALLBACK_HIT_SCORE = 1
MAP_IR_FALLBACK_PATCH_RADIUS_CELLS = 1
MAP_DISPLAY_WALL_DILATION_CELLS = 1
MAP_DISPLAY_BRIDGE_GAP_CELLS = 2
MAP_DISPLAY_REMOVE_ISOLATED_WALLS = True
MAP_CONNECT_CONSECUTIVE_WALL_HITS = True
MAP_WALL_CONNECT_MAX_M = 0.18
MAP_WALL_CONNECT_SCORE = 4
MAP_LOOP_CLOSURE_MIN_ERROR_M = 0.015
MAP_LOOP_CLOSURE_MAX_ERROR_M = 0.35
MAP_LOOP_CLOSURE_GAIN = 1.0
MAP_SAVE_ON_JUNCTION = False
MAP_AUTOSAVE_SEC = 0.0
MAP_EXPORT_MARGIN_M = 0.3
MAP_SVG_PX_PER_M = 420.0
MAP_EXPORT_PNG = True
MAP_PNG_DPI = 220
MAP_DRAW_TRAJECTORY = True
MAP_DRAW_NODES = True
MAP_DRAW_EXIT = True
CALIBRATION_SHARP_LEFT = [(675, 5.0), (343, 10.0), (236, 15.0), (166, 20.0), (126, 25.0), (105, 30.0), (50, 80.0)]
CALIBRATION_SHARP_RIGHT = list(CALIBRATION_SHARP_LEFT)
CALIBRATION_SHARP2 = CALIBRATION_SHARP_LEFT
ENABLE_FEEDBACK_TURN = True
TURN_PRE_SETTLE_SEC = 0.1
TURN_FEEDBACK_KP = 1.2
TURN_FEEDBACK_MIN_Z_SPEED = 10.0
TURN_FEEDBACK_MAX_Z_SPEED = 55.0
TURN_FEEDBACK_TOLERANCE_DEG = 4.0
TURN_FEEDBACK_STABLE_SAMPLES = 2
TURN_FEEDBACK_LOOP_SEC = 0.03
TURN_FEEDBACK_DRIVE_TIMEOUT_SEC = 0.2
TURN_FEEDBACK_TIMEOUT_90_SEC = 5.0
TURN_FEEDBACK_TIMEOUT_180_SEC = 8.0
TURN_FEEDBACK_PRINT_SEC = 0.25
TURN_TIMEOUT_ACCEPT_TOLERANCE_DEG = 5.0
TURN_MAX_ATTEMPTS = 3
TURN_RETRY_SETTLE_SEC = 0.25
TURN_ACTION_TIMEOUT_90_SEC = 3.5
TURN_ACTION_TIMEOUT_180_SEC = 6.0
ENABLE_V11_EDGE_FSM = True
EDGE_OBS_SIDE_ENTER_CM = 18.0
EDGE_OBS_SIDE_EXIT_CM = 15.0
EDGE_OBS_ENTER_SAMPLES = 3
EDGE_OBS_EXIT_SAMPLES = 2
EDGE_OBS_IGNORE_FROM_NODE_M = 0.2
EDGE_OBS_INTERRUPT_MIN_WIDTH_M = 0.12
EDGE_OBS_INTERRUPT_MIN_SAMPLES = 4
EDGE_OBS_MAX_OPENING_M = 0.75
EDGE_CANDIDATE_MIN_WIDTH_M = 0.08
EDGE_CANDIDATE_MIN_SAMPLES = 4
EDGE_CANDIDATE_MATCH_RADIUS_M = 0.22
EDGE_CANDIDATE_PROMOTE_RADIUS_M = 0.24
EDGE_CANDIDATE_REANCHOR_MAX_M = 0.3
EDGE_CANDIDATE_UPDATE_ALPHA = 0.35
EDGE_CANDIDATE_REPEAT_BONUS = 0.18
EDGE_CANDIDATE_LATENT_CONFIDENCE = 0.68
EDGE_CANDIDATE_LATENT_PASSES = 2
EDGE_CANDIDATE_CONF_WIDTH_M = 0.16
EDGE_CANDIDATE_CONF_SAMPLES = 7
EDGE_CANDIDATE_CONF_RANGE_CM = 40.0
ENABLE_LATENT_FRONTIER_VERIFICATION = True
EDGE_VERIFY_MIN_CONFIDENCE = 0.52
EDGE_VERIFY_SLOW_RADIUS_M = 0.28
EDGE_VERIFY_CREEP_RADIUS_M = 0.14
EDGE_VERIFY_SLOW_SPEED = 0.09
EDGE_VERIFY_CREEP_SPEED = 0.045
EDGE_VERIFY_SEEN_BONUS = 0.08
EDGE_VERIFY_MISS_PENALTY = 0.3
EDGE_VERIFY_RETIRE_MISSES = 2
EDGE_VERIFY_RETIRE_CONFIDENCE = 0.25
EDGE_MEMORY_FILE = 'edge_memory.json'
EDGE_MEMORY_PRINT_EVENTS = True
EDGE_MEMORY_EVENT_RAM_LIMIT = 2000
EDGE_MEMORY_EVENT_SAVE_LIMIT = 500
SAVE_EDGE_MEMORY = True
ENABLE_COLLISION_SUPERVISOR = True
COLLISION_RAW_IR_EMERGENCY = True
COLLISION_BLOCK_ROTATION_ON_RAW_IR = True
COLLISION_STOP_ON_MISSING_BOTH_IR = False
COLLISION_LOG_COOLDOWN_SEC = 0.2
COLLISION_ACTIVE_FORWARD_ESCAPE = True
COLLISION_ACTIVE_ESCAPE_HOLD_SEC = 0.45
COLLISION_ACTIVE_ESCAPE_MAX_SEC = 1.0
COLLISION_ACTIVE_ESCAPE_Y_SPEED = 0.06
COLLISION_ACTIVE_ESCAPE_BACK_BIAS_SPEED = 0.03
COLLISION_ACTIVE_ESCAPE_BOTH_BACK_SPEED = 0.055
COLLISION_ACTIVE_ESCAPE_OPPOSITE_MIN_CM = 6.5
TURN_IR_USE_RAW_EMERGENCY = True
TURN_IR_RAW_RECHECK_SAMPLES = 1
TURN_IR_RAW_RECHECK_INTERVAL_SEC = 0.025
FSM_INITIAL_STATE = 'EDGE_TRAVERSE'
NODE_LOCK_RELEASE_PROGRESS_M = 0.3

# ==================== PRE-DRAWN TOPOLOGY GUIDE ====================
# The drawing is deliberately *not* a metric map. The guide compares only
# recognisable topology (junction/corner/dead-end openings), tries all four
# rotations plus their mirrored forms, and yields to the live sensors whenever
# the drawing and the physical maze disagree.
ENABLE_PREDRAWN_TOPOLOGY_GUIDE = True
PREDRAWN_GUIDE_FILE = 'known_route.json'
GUIDE_MAX_HYPOTHESES = 32
GUIDE_SCORE_WINDOW = 9.0
GUIDE_RELOCALIZE_BELOW_SCORE = -12.0
GUIDE_RELOCALIZE_PENALTY = 4.0
GUIDE_MIN_OBSERVED_NODES = 1
GUIDE_MIN_VOTE_RATIO = 0.67
GUIDE_MIN_PHYSICAL_SUPPORT_RATIO = 0.55
GUIDE_MARKER_CONSENSUS_RATIO = 0.72
GUIDE_ROUTE_GRID_HINT_WEIGHT = 0.03
GUIDE_AUTO_ADVANCE_PICKUP_DROP = True
GUIDE_REQUIRE_CONTINUITY_AFTER_RELOCALIZE = True
GUIDE_RELOCALIZE_MIN_TRANSITIONS = 1
GUIDE_MAX_COMMAND_HYPOTHESES = 12
GUIDE_MARKER_MIN_OBSERVATIONS = 1
GUIDE_MISSION_MARKER_DWELL_SEC = 1.0
GUIDE_DEBUG = True

# ==================== CONFIG / COMPATIBILITY ====================
class _MonsterConfigView:

    def __getattr__(self, name):
        try:
            return globals()[name]
        except KeyError as exc:
            raise AttributeError(name) from exc

    def __setattr__(self, name, value):
        globals()[name] = value
config = _MonsterConfigView()

def normalize_angle_deg(angle):
    """Normalize angle to [-180, 180)."""
    return (float(angle) + 180.0) % 360.0 - 180.0

def shortest_angle_error_deg(target, current):
    """Signed shortest error needed to rotate current -> target."""
    return normalize_angle_deg(float(target) - float(current))

# ==================== POSE TRACKING ====================
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
        self.move_to_yaw_sign = config.DEFAULT_MOVE_TO_YAW_SIGN if hasattr(config, 'DEFAULT_MOVE_TO_YAW_SIGN') else None
        self.drive_to_yaw_sign = config.DEFAULT_DRIVE_TO_YAW_SIGN if hasattr(config, 'DEFAULT_DRIVE_TO_YAW_SIGN') else None

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
            print('Position callback error:', exc)

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
            print('Attitude callback error:', exc)

    def get_pose(self):
        """Return (x, y, yaw_deg). yaw comes from sub_attitude()."""
        with self._lock:
            return (self.x, self.y, self.yaw_deg)

    def get_position(self):
        with self._lock:
            return (self.x, self.y, self.position_z)

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
        self.front_ir_left_raw = None
        self.front_ir_right_raw = None
        self.front_ir_left_hit_count = 0
        self.front_ir_right_hit_count = 0
        self.front_ir_left_clear_count = 0
        self.front_ir_right_clear_count = 0
        self.front_ir_left_confirmed = False
        self.front_ir_right_confirmed = False
        self.front_ir_blocked = False
        self.front_ir_left_clear_level = None
        self.front_ir_right_clear_level = None
        self.front_ir_left_clear_adc = None
        self.front_ir_right_clear_adc = None
        self.front_ir_left_adc = None
        self.front_ir_right_adc = None
        self.front_ir_left_adc_time = None
        self.front_ir_right_adc_time = None
        self.sharp_last_valid = {config.SHARP_LEFT_ID: {'raw': None, 'cm': None, 'time': None}, config.SHARP_RIGHT_ID: {'raw': None, 'cm': None, 'time': None}}
        self.sharp_invalid_count = {config.SHARP_LEFT_ID: 0, config.SHARP_RIGHT_ID: 0}
        self.sharp_last_warn_time = {config.SHARP_LEFT_ID: 0.0, config.SHARP_RIGHT_ID: 0.0}

    def tof_callback(self, data):
        try:
            if not data or data[0] is None:
                return
            mm = data[0]
            if mm < 20 or mm > 4000:
                return
            cm = mm / 10.0
            self.tof_buffer.append(cm)
            self.front_cm = statistics.median(self.tof_buffer)
            self.tof_last_update = time.monotonic()
        except Exception as exc:
            print('ToF callback error:', exc)

    def get_front_cm(self):
        """Return fresh ToF distance, or None if data is absent/stale."""
        if self.front_cm is None or self.tof_last_update is None:
            return None
        age = time.monotonic() - self.tof_last_update
        if age > config.TOF_STALE_SEC:
            return None
        return self.front_cm

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
        raise ValueError(f'Unknown Sharp sensor id: {sensor_id}')

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
        if cache['time'] is None or cache['cm'] is None:
            return (None, None, None)
        age = time.monotonic() - cache['time']
        if age <= config.SHARP_STALE_HOLD_SEC:
            return (cache['raw'], cache['cm'], age)
        return (None, None, age)

    def _warn_invalid_sharp(self, sensor_id, message):
        now = time.monotonic()
        if now - self.sharp_last_warn_time[sensor_id] >= config.SHARP_INVALID_WARN_INTERVAL_SEC:
            side = 'LEFT' if sensor_id == config.SHARP_LEFT_ID else 'RIGHT'
            print(f'>>> SHARP {side} WARNING: {message}')
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
            raw = self.sensor_adapter.get_adc(id=sensor_id, port=config.SENSOR_PORT)
        except Exception as exc:
            raw = None
            self._warn_invalid_sharp(sensor_id, f'read exception: {exc}')
        if not self._valid_adc(raw):
            self.sharp_invalid_count[sensor_id] += 1
            cached_raw, cached_cm, age = self._cached_sharp(sensor_id)
            if cached_cm is not None:
                self._warn_invalid_sharp(sensor_id, f'invalid ADC={raw!r}; using cached value age={age:.2f}s misses={self.sharp_invalid_count[sensor_id]}')
                return (cached_raw, cached_cm)
            age_text = 'none' if age is None else f'{age:.2f}s'
            self._warn_invalid_sharp(sensor_id, f'invalid ADC={raw!r}; no fresh cache (age={age_text}) misses={self.sharp_invalid_count[sensor_id]}')
            return (None, None)
        raw = int(round(float(raw)))
        self.sharp_invalid_count[sensor_id] = 0
        if sensor_id == config.SHARP_LEFT_ID:
            self.sharp_left_buffer.append(raw)
            median_adc = statistics.median(self.sharp_left_buffer)
            if self.sharp_left_ema is None:
                self.sharp_left_ema = median_adc
            else:
                self.sharp_left_ema = config.SHARP_EMA_NEW_WEIGHT * median_adc + config.SHARP_EMA_OLD_WEIGHT * self.sharp_left_ema
            ema_val = self.sharp_left_ema
        elif sensor_id == config.SHARP_RIGHT_ID:
            self.sharp_right_buffer.append(raw)
            median_adc = statistics.median(self.sharp_right_buffer)
            if self.sharp_right_ema is None:
                self.sharp_right_ema = median_adc
            else:
                self.sharp_right_ema = config.SHARP_EMA_NEW_WEIGHT * median_adc + config.SHARP_EMA_OLD_WEIGHT * self.sharp_right_ema
            ema_val = self.sharp_right_ema
        else:
            raise ValueError(f'Unknown Sharp sensor id: {sensor_id}')
        table = self.calibration_for_sensor(sensor_id)
        cm = self.adc_to_cm(ema_val, table)
        self.sharp_last_valid[sensor_id] = {'raw': raw, 'cm': cm, 'time': time.monotonic()}
        return (raw, cm)

    def read_left_sharp(self):
        return self.read_sharp_raw_and_cm(config.SHARP_LEFT_ID)

    def read_right_sharp(self):
        return self.read_sharp_raw_and_cm(config.SHARP_RIGHT_ID)

    def _read_digital_sensor(self, sensor_id):
        """Read the verified ACTIVE-LOW front IR digital input.

        Real hardware sweep on this robot:
          ID1 port1: CLEAR=1, OBSTACLE=0
          ID4 port1: CLEAR=1, OBSTACLE=0

        IMPORTANT: RoboMaster Python SDK uses sensor_adapter.get_io(), not
        get_io_level(), for this Sensor Adapter digital input.  Do not infer the
        digital state from ADC here.
        """
        try:
            value = self.sensor_adapter.get_io(id=int(sensor_id), port=int(config.SENSOR_PORT))
        except Exception:
            return None
        if value in (0, 1):
            return int(value)
        try:
            value = int(value)
        except (TypeError, ValueError):
            return None
        return value if value in (0, 1) else None

    def _read_front_ir_adc(self, sensor_id, side, force=False):
        """Read/cache the analogue value of a front digital IR output."""
        if not bool(getattr(config, 'FRONT_IR_ADC_FUSION_ENABLED', True)):
            return None
        now = time.monotonic()
        if side == 'LEFT':
            cached = self.front_ir_left_adc
            last_t = self.front_ir_left_adc_time
        else:
            cached = self.front_ir_right_adc
            last_t = self.front_ir_right_adc_time
        refresh = max(0.0, float(getattr(config, 'FRONT_IR_ADC_REFRESH_SEC', 0.03)))
        if not force and cached is not None and (last_t is not None) and (now - last_t < refresh):
            return cached
        try:
            raw = self.sensor_adapter.get_adc(id=sensor_id, port=config.SENSOR_PORT)
            if raw is None:
                return cached
            raw = float(raw)
            if side == 'LEFT':
                self.front_ir_left_adc = raw
                self.front_ir_left_adc_time = now
            else:
                self.front_ir_right_adc = raw
                self.front_ir_right_adc_time = now
            return raw
        except Exception:
            return cached

    def read_front_left_ir(self):
        value = self._read_digital_sensor(config.IR_FRONT_LEFT_ID)
        self._read_front_ir_adc(config.IR_FRONT_LEFT_ID, 'LEFT')
        return value

    def read_front_right_ir(self):
        value = self._read_digital_sensor(config.IR_FRONT_RIGHT_ID)
        self._read_front_ir_adc(config.IR_FRONT_RIGHT_ID, 'RIGHT')
        return value

    def read_front_ir_pair(self):
        return (self.read_front_left_ir(), self.read_front_right_ir())

    def read_front_ir_electrical_state(self, force_adc=False):
        """Return IO + ADC for both front IR sensors in one diagnostic object."""
        left_raw = self._read_digital_sensor(config.IR_FRONT_LEFT_ID)
        right_raw = self._read_digital_sensor(config.IR_FRONT_RIGHT_ID)
        left_adc = self._read_front_ir_adc(config.IR_FRONT_LEFT_ID, 'LEFT', force=force_adc)
        right_adc = self._read_front_ir_adc(config.IR_FRONT_RIGHT_ID, 'RIGHT', force=force_adc)
        self.front_ir_left_raw = left_raw
        self.front_ir_right_raw = right_raw
        return {'left_raw': left_raw, 'right_raw': right_raw, 'left_adc': left_adc, 'right_adc': right_adc}

    def calibrate_front_ir_clear_levels(self):
        """Learn CLEAR digital level and ADC baseline for each front IR.

        Optional compatibility helper for other hardware revisions.
        V11.3 on this robot disables this path and uses fixed get_io() ACTIVE-LOW
        semantics (CLEAR=1, HIT=0).
        """
        if not bool(getattr(config, 'FRONT_IR_AUTO_LEARN_CLEAR_LEVEL', True)):
            return False
        n = max(3, int(getattr(config, 'FRONT_IR_AUTO_LEARN_SAMPLES', 12)))
        dt = float(getattr(config, 'FRONT_IR_AUTO_LEARN_INTERVAL_SEC', 0.025))
        min_ratio = float(getattr(config, 'FRONT_IR_AUTO_LEARN_REQUIRE_STABLE_RATIO', 0.7))
        ls, rs, ladcs, radcs = ([], [], [], [])
        for _ in range(n):
            state = self.read_front_ir_electrical_state(force_adc=True)
            l, r = (state['left_raw'], state['right_raw'])
            if l in (0, 1):
                ls.append(int(l))
            if r in (0, 1):
                rs.append(int(r))
            if state['left_adc'] is not None:
                ladcs.append(float(state['left_adc']))
            if state['right_adc'] is not None:
                radcs.append(float(state['right_adc']))
            time.sleep(dt)

        def learn(values):
            if not values:
                return None
            zeros = values.count(0)
            ones = values.count(1)
            level = 1 if ones >= zeros else 0
            ratio = max(zeros, ones) / float(len(values))
            return level if ratio >= min_ratio else None
        self.front_ir_left_clear_level = learn(ls)
        self.front_ir_right_clear_level = learn(rs)
        self.front_ir_left_clear_adc = statistics.median(ladcs) if ladcs else None
        self.front_ir_right_clear_adc = statistics.median(radcs) if radcs else None
        self.front_ir_left_hit_count = 0
        self.front_ir_right_hit_count = 0
        self.front_ir_left_clear_count = 0
        self.front_ir_right_clear_count = 0
        self.front_ir_left_confirmed = False
        self.front_ir_right_confirmed = False
        self.front_ir_blocked = False
        ok = self.front_ir_left_clear_level is not None and self.front_ir_right_clear_level is not None
        ladc = '---' if self.front_ir_left_clear_adc is None else f'{self.front_ir_left_clear_adc:.0f}'
        radc = '---' if self.front_ir_right_clear_adc is None else f'{self.front_ir_right_clear_adc:.0f}'
        print(f'>>> FRONT IR AUTO BASELINE Lclear={self.front_ir_left_clear_level} Rclear={self.front_ir_right_clear_level} Ladc={ladc} Radc={radc} samples={len(ls)}/{len(rs)} ' + ('OK' if ok else 'UNSTABLE -> config fallback'))
        return ok

    def _adc_ir_is_hit(self, side):
        if not bool(getattr(config, 'FRONT_IR_ADC_FUSION_ENABLED', True)):
            return False
        if side == 'LEFT':
            adc = self.front_ir_left_adc
            base = self.front_ir_left_clear_adc
        else:
            adc = self.front_ir_right_adc
            base = self.front_ir_right_clear_adc
        if adc is None or base is None:
            return False
        delta = abs(float(adc) - float(base))
        return delta >= float(getattr(config, 'FRONT_IR_ADC_HIT_DELTA', 220.0))

    def _ir_is_hit(self, value, side=None):
        digital_hit = False
        try:
            if value is not None:
                v = int(value)
                if side == 'LEFT' and self.front_ir_left_clear_level in (0, 1):
                    digital_hit = v != int(self.front_ir_left_clear_level)
                elif side == 'RIGHT' and self.front_ir_right_clear_level in (0, 1):
                    digital_hit = v != int(self.front_ir_right_clear_level)
                else:
                    digital_hit = v == int(config.FRONT_IR_ACTIVE_LEVEL)
        except (TypeError, ValueError):
            digital_hit = False
        return bool(digital_hit or self._adc_ir_is_hit(side))

    def read_front_ir_raw_state(self):
        """Fresh, non-debounced collision snapshot used by V11.2 safety code."""
        state = self.read_front_ir_electrical_state(force_adc=False)
        left_raw = state['left_raw']
        right_raw = state['right_raw']
        return {'left_raw': left_raw, 'right_raw': right_raw, 'left_adc': state['left_adc'], 'right_adc': state['right_adc'], 'left_hit': self._ir_is_hit(left_raw, 'LEFT'), 'right_hit': self._ir_is_hit(right_raw, 'RIGHT'), 'missing': left_raw is None and state['left_adc'] is None or (right_raw is None and state['right_adc'] is None)}

    def latest_sharp_cm(self, side):
        """Return the most recent non-stale Sharp value without another SDK read."""
        sensor_id = config.SHARP_LEFT_ID if side == 'LEFT' else config.SHARP_RIGHT_ID
        item = self.sharp_last_valid.get(sensor_id, {})
        t = item.get('time')
        cm = item.get('cm')
        if t is None or cm is None:
            return None
        if time.monotonic() - float(t) > float(config.SHARP_STALE_HOLD_SEC):
            return None
        return float(cm)

    @staticmethod
    def _debounce_side(hit, confirmed, hit_count, clear_count):
        enter_n = max(1, int(getattr(config, 'FRONT_IR_SIDE_ENTER_SAMPLES', 2)))
        release_n = max(1, int(getattr(config, 'FRONT_IR_SIDE_RELEASE_SAMPLES', 2)))
        if hit:
            hit_count += 1
            clear_count = 0
            if hit_count >= enter_n:
                confirmed = True
        else:
            hit_count = 0
            clear_count += 1
            if confirmed and clear_count >= release_n:
                confirmed = False
        return (confirmed, hit_count, clear_count)

    def update_front_ir_guard(self, left_raw=None, right_raw=None, refresh=False):
        """Read/debounce both front-corner IR modules.

        Per-side confirmed states drive V10.3 collision avoidance.  The aggregate
        ``blocked`` state is deliberately separate: with the recommended BOTH
        mode, a single corner reflection can make the chassis slide away without
        falsely telling the Trémaux planner that the whole FRONT corridor is shut.
        """
        if refresh or (left_raw is None and right_raw is None):
            left_raw, right_raw = self.read_front_ir_pair()
        left_hit = self._ir_is_hit(left_raw, 'LEFT')
        right_hit = self._ir_is_hit(right_raw, 'RIGHT')
        mode = str(getattr(config, 'FRONT_IR_BLOCK_MODE', 'BOTH')).upper()
        if left_raw is None or right_raw is None:
            self.front_ir_left_raw = left_raw
            self.front_ir_right_raw = right_raw
            return {'left_raw': left_raw, 'right_raw': right_raw, 'left_hit': left_hit, 'right_hit': right_hit, 'left_confirmed': bool(self.front_ir_left_confirmed), 'right_confirmed': bool(self.front_ir_right_confirmed), 'blocked': bool(self.front_ir_blocked), 'mode': mode, 'missing': True}
        old_l = self.front_ir_left_confirmed
        old_r = self.front_ir_right_confirmed
        old_b = self.front_ir_blocked
        self.front_ir_left_confirmed, self.front_ir_left_hit_count, self.front_ir_left_clear_count = self._debounce_side(left_hit, self.front_ir_left_confirmed, self.front_ir_left_hit_count, self.front_ir_left_clear_count)
        self.front_ir_right_confirmed, self.front_ir_right_hit_count, self.front_ir_right_clear_count = self._debounce_side(right_hit, self.front_ir_right_confirmed, self.front_ir_right_hit_count, self.front_ir_right_clear_count)
        if not getattr(config, 'ENABLE_DUAL_FRONT_IR_GUARD', True):
            self.front_ir_blocked = False
        elif mode == 'EITHER':
            self.front_ir_blocked = bool(self.front_ir_left_confirmed or self.front_ir_right_confirmed)
        else:
            self.front_ir_blocked = bool(self.front_ir_left_confirmed and self.front_ir_right_confirmed)
        if getattr(config, 'FRONT_IR_PRINT_STATE_CHANGES', True):
            if old_l != self.front_ir_left_confirmed:
                print('>>> FRONT IR LEFT: ' + ('HIT' if self.front_ir_left_confirmed else 'CLEAR'))
            if old_r != self.front_ir_right_confirmed:
                print('>>> FRONT IR RIGHT: ' + ('HIT' if self.front_ir_right_confirmed else 'CLEAR'))
            if old_b != self.front_ir_blocked:
                print('>>> FRONT IR GUARD: ' + ('BLOCKED' if self.front_ir_blocked else 'CLEAR'))
        self.front_ir_left_raw = left_raw
        self.front_ir_right_raw = right_raw
        return {'left_raw': left_raw, 'right_raw': right_raw, 'left_hit': left_hit, 'right_hit': right_hit, 'left_confirmed': bool(self.front_ir_left_confirmed), 'right_confirmed': bool(self.front_ir_right_confirmed), 'blocked': bool(self.front_ir_blocked), 'mode': mode, 'missing': False}

    def effective_front_cm(self, tof_front_cm, ir_blocked=None):
        """Return navigation-only front range with the aggregate IR guard.

        A single confirmed corner hit is handled by lateral collision avoidance.
        Only the configured aggregate condition (BOTH by default) injects the
        synthetic FRONT-block distance used by junction/exit logic.
        """
        if ir_blocked is None:
            ir_blocked = bool(getattr(self, 'front_ir_blocked', False))
        if not ir_blocked:
            return tof_front_cm
        synthetic = float(getattr(config, 'FRONT_IR_SYNTHETIC_BLOCK_CM', config.STOP_FRONT_CM))
        if tof_front_cm is None:
            return synthetic
        return min(float(tof_front_cm), synthetic)

    def read_ir_digital_io(self):
        return self.read_front_left_ir()

    def reset_filters(self):
        self.tof_buffer.clear()
        self.sharp_left_buffer.clear()
        self.sharp_right_buffer.clear()
        self.sharp_left_ema = None
        self.sharp_right_ema = None
        self.front_cm = None
        self.tof_last_update = None
        self.front_ir_left_raw = None
        self.front_ir_right_raw = None
        self.front_ir_left_hit_count = 0
        self.front_ir_right_hit_count = 0
        self.front_ir_left_clear_count = 0
        self.front_ir_right_clear_count = 0
        self.front_ir_left_confirmed = False
        self.front_ir_right_confirmed = False
        self.front_ir_blocked = False
        now = time.monotonic()
        for sensor_id in (config.SHARP_LEFT_ID, config.SHARP_RIGHT_ID):
            self.sharp_last_valid[sensor_id] = {'raw': None, 'cm': None, 'time': None}
            self.sharp_invalid_count[sensor_id] = 0
            self.sharp_last_warn_time[sensor_id] = min(self.sharp_last_warn_time.get(sensor_id, 0.0), now)

# ==================== COLLISION SUPERVISOR / SAFE CHASSIS ====================
class SafeChassisProxy:
    """Delegate RoboMaster chassis APIs while supervising drive_speed()."""

    def __init__(self, chassis, sensors):
        self._chassis = chassis
        self._sensors = sensors
        self.last_raw_state = None
        self.last_veto = None
        self.last_command = (0.0, 0.0, 0.0)
        self._last_log = 0.0
        self._escape_kind = None
        self._escape_until = 0.0
        self._escape_started = 0.0
        self._escape_fault = False

    def __getattr__(self, name):
        return getattr(self._chassis, name)

    @property
    def raw_chassis(self):
        return self._chassis

    def _log(self, text, force=False):
        now = time.monotonic()
        cooldown = float(getattr(config, 'COLLISION_LOG_COOLDOWN_SEC', 0.2))
        if force or now - self._last_log >= cooldown:
            print(text)
            self._last_log = now

    def _clear_escape(self):
        self._escape_kind = None
        self._escape_until = 0.0
        self._escape_started = 0.0
        self._escape_fault = False

    def _choose_forward_escape(self, left, right):
        """Choose a body-frame clearance command using cached Sharp room."""
        y_sign = 1.0 if float(getattr(config, 'Y_DIR_SIGN', 1.0)) >= 0 else -1.0
        y_speed = abs(float(getattr(config, 'COLLISION_ACTIVE_ESCAPE_Y_SPEED', 0.06)))
        back_bias = abs(float(getattr(config, 'COLLISION_ACTIVE_ESCAPE_BACK_BIAS_SPEED', 0.03)))
        both_back = abs(float(getattr(config, 'COLLISION_ACTIVE_ESCAPE_BOTH_BACK_SPEED', 0.055)))
        room_min = float(getattr(config, 'COLLISION_ACTIVE_ESCAPE_OPPOSITE_MIN_CM', 6.5))
        if left and right:
            return ('IR_BOTH_ACTIVE_BACK', -both_back, 0.0, 0.0)
        if left:
            opposite = self._sensors.latest_sharp_cm('RIGHT')
            if opposite is not None and opposite <= room_min:
                return ('IR_LEFT_ACTIVE_BACK', -both_back, 0.0, 0.0)
            return ('IR_LEFT_ACTIVE_RIGHT', -back_bias, +y_speed * y_sign, 0.0)
        opposite = self._sensors.latest_sharp_cm('LEFT')
        if opposite is not None and opposite <= room_min:
            return ('IR_RIGHT_ACTIVE_BACK', -both_back, 0.0, 0.0)
        return ('IR_RIGHT_ACTIVE_LEFT', -back_bias, -y_speed * y_sign, 0.0)

    def _start_or_refresh_forward_escape(self, left, right):
        now = time.monotonic()
        hold = max(0.05, float(getattr(config, 'COLLISION_ACTIVE_ESCAPE_HOLD_SEC', 0.45)))
        max_sec = max(hold, float(getattr(config, 'COLLISION_ACTIVE_ESCAPE_MAX_SEC', 1.0)))
        if self._escape_kind is None:
            kind, _, _, _ = self._choose_forward_escape(left, right)
            self._escape_kind = kind
            self._escape_started = now
            self._escape_until = now + hold
            self._escape_fault = False
            self._log(f'>>> ACTIVE IR BUMPER START {kind}: raw hit -> physical clearance hold={hold:.2f}s', force=True)
        else:
            kind, _, _, _ = self._choose_forward_escape(left, right)
            self._escape_kind = kind
            if now - self._escape_started < max_sec:
                self._escape_until = max(self._escape_until, now + hold)
            else:
                self._escape_fault = True
                self._log('>>> ACTIVE IR BUMPER FAULT: IR stayed hit too long -> HARD STOP', force=True)

    def _active_escape_command(self, raw_state):
        left = bool(raw_state.get('left_hit', False))
        right = bool(raw_state.get('right_hit', False))
        if self._escape_fault:
            if not (left or right):
                self._clear_escape()
                return None
            return ('IR_ACTIVE_FAULT_STOP', 0.0, 0.0, 0.0)
        if self._escape_kind is None:
            return None
        now = time.monotonic()
        if now >= self._escape_until and (not (left or right)):
            self._log(f'>>> ACTIVE IR BUMPER CLEAR {self._escape_kind}', force=True)
            self._clear_escape()
            return None
        kind = self._escape_kind
        y_sign = 1.0 if float(getattr(config, 'Y_DIR_SIGN', 1.0)) >= 0 else -1.0
        y_speed = abs(float(getattr(config, 'COLLISION_ACTIVE_ESCAPE_Y_SPEED', 0.06)))
        back_bias = abs(float(getattr(config, 'COLLISION_ACTIVE_ESCAPE_BACK_BIAS_SPEED', 0.03)))
        both_back = abs(float(getattr(config, 'COLLISION_ACTIVE_ESCAPE_BOTH_BACK_SPEED', 0.055)))
        if kind in ('IR_BOTH_ACTIVE_BACK', 'IR_LEFT_ACTIVE_BACK', 'IR_RIGHT_ACTIVE_BACK'):
            return (kind, -both_back, 0.0, 0.0)
        if kind == 'IR_LEFT_ACTIVE_RIGHT':
            return (kind, -back_bias, +y_speed * y_sign, 0.0)
        if kind == 'IR_RIGHT_ACTIVE_LEFT':
            return (kind, -back_bias, -y_speed * y_sign, 0.0)
        return ('IR_ACTIVE_UNKNOWN_STOP', 0.0, 0.0, 0.0)

    def drive_speed(self, x=0.0, y=0.0, z=0.0, timeout=None, **kwargs):
        if timeout is None:
            timeout = getattr(config, 'DRIVE_TIMEOUT_SEC', 0.15)
        if not bool(getattr(config, 'ENABLE_COLLISION_SUPERVISOR', True)):
            self.last_command = (float(x), float(y), float(z))
            return self._chassis.drive_speed(x=x, y=y, z=z, timeout=timeout, **kwargs)
        x = float(x)
        y = float(y)
        z = float(z)
        original = (x, y, z)
        needs_guard = x > 0.0 or abs(z) > 1e-06 or abs(y) > 1e-06 or (self._escape_kind is not None)
        if needs_guard and bool(getattr(config, 'COLLISION_RAW_IR_EMERGENCY', True)):
            state = self._sensors.read_front_ir_raw_state()
            self.last_raw_state = state
            left = bool(state.get('left_hit'))
            right = bool(state.get('right_hit'))
            missing = bool(state.get('missing'))
            if missing and bool(getattr(config, 'COLLISION_STOP_ON_MISSING_BOTH_IR', False)):
                x = 0.0
                z = 0.0
                self.last_veto = 'IR_MISSING'
                self._log('>>> COLLISION SUPERVISOR: IR missing -> veto forward/rotation')
            if bool(getattr(config, 'COLLISION_ACTIVE_FORWARD_ESCAPE', True)) and x > 0.0 and (left or right):
                self._start_or_refresh_forward_escape(left, right)
            if self._escape_kind is not None and original[0] > 0.0:
                active = self._active_escape_command(state)
                if active is not None:
                    kind, x, y, z = active
                    self.last_veto = kind
                    if (x, y, z) != original:
                        self._log(f'>>> COLLISION SUPERVISOR {kind}: ({original[0]:+.3f},{original[1]:+.3f},{original[2]:+.1f}) -> ({x:+.3f},{y:+.3f},{z:+.1f})')
                    self.last_command = (float(x), float(y), float(z))
                    return self._chassis.drive_speed(x=x, y=y, z=z, timeout=timeout, **kwargs)
            if left or right:
                if x > 0.0:
                    x = 0.0
                if abs(z) > 1e-06 and bool(getattr(config, 'COLLISION_BLOCK_ROTATION_ON_RAW_IR', True)):
                    z = 0.0
                y_sign = 1.0 if float(getattr(config, 'Y_DIR_SIGN', 1.0)) >= 0 else -1.0
                if left and right:
                    y = 0.0
                    why = 'IR_BOTH'
                elif left:
                    if y * y_sign < 0.0:
                        y = 0.0
                    why = 'IR_LEFT'
                else:
                    if y * y_sign > 0.0:
                        y = 0.0
                    why = 'IR_RIGHT'
                self.last_veto = why
                if (x, y, z) != original:
                    self._log(f'>>> COLLISION SUPERVISOR {why}: ({original[0]:+.3f},{original[1]:+.3f},{original[2]:+.1f}) -> ({x:+.3f},{y:+.3f},{z:+.1f})')
            elif self._escape_kind is None:
                self.last_veto = None
        self.last_command = (float(x), float(y), float(z))
        return self._chassis.drive_speed(x=x, y=y, z=z, timeout=timeout, **kwargs)

    def hard_stop(self):
        try:
            return self._chassis.drive_speed(x=0.0, y=0.0, z=0.0, timeout=0.1)
        except Exception:
            return None

def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))

# ==================== MOTION CONTROLLER ====================
class MotionController:

    def __init__(self):
        self.side_owner = 'NONE'
        self.side_owner_since = 0.0
        self.side_owner_open_since = None
        self.left_wall_present = False
        self.right_wall_present = False
        self.heading_base_yaw = None
        self.heading_target_yaw = None
        self.heading_right_step_sign = None
        self.heading_recovering = False
        self._corr_cal_x = None
        self._corr_cal_y = None
        self._corr_cal_left = None
        self._corr_cal_right = None
        self._corr_cal_heading_index = None
        self._wall_parallel_history = deque(maxlen=max(3, int(getattr(config, 'WALL_PARALLEL_SINGLE_CONFIRM_WINDOWS', 3))))

    def reset_side_owner(self):
        self.side_owner = 'NONE'
        self.side_owner_since = 0.0
        self.side_owner_open_since = None

    def reset_wall_states(self):
        self.left_wall_present = False
        self.right_wall_present = False

    def reset_after_turn(self):
        self.reset_side_owner()
        self.reset_wall_states()
        self.reset_corridor_heading_calibration()

    def reset_corridor_heading_calibration(self):
        self._corr_cal_x = None
        self._corr_cal_y = None
        self._corr_cal_left = None
        self._corr_cal_right = None
        self._corr_cal_heading_index = None
        if hasattr(self, '_wall_parallel_history'):
            self._wall_parallel_history.clear()

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
        right_command_sign = 1 if config.TURN_RIGHT_DEG * config.Z_DIR_SIGN > 0 else -1
        self.heading_right_step_sign = right_command_sign * sign_map
        return True

    @staticmethod
    def _signed_cardinal_step(heading_index):
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
        self.heading_target_yaw = normalize_angle_deg(self.heading_base_yaw + self.heading_right_step_sign * 90.0 * step)
        return self.heading_target_yaw

    def heading_error(self, current_yaw):
        if self.heading_target_yaw is None or current_yaw is None:
            return None
        return shortest_angle_error_deg(self.heading_target_yaw, current_yaw)

    def calculate_heading_hold(self, current_yaw, pose_tracker, recover=False):
        """Return (z_command_deg_s, yaw_error_deg)."""
        if not config.ENABLE_HEADING_HOLD:
            return (0.0, None)
        error = self.heading_error(current_yaw)
        if error is None:
            return (0.0, None)
        if abs(error) <= config.HEADING_DEADBAND_DEG:
            return (0.0, error)
        sign_map = pose_tracker.get_drive_to_yaw_sign()
        if sign_map not in (-1, 1):
            return (0.0, error)
        max_z = config.HEADING_RECOVER_MAX_Z_SPEED if recover else config.HEADING_MAX_Z_SPEED
        desired_yaw_rate = clamp(error * config.HEADING_KP_Z, -max_z, max_z)
        z_cmd = desired_yaw_rate / sign_map
        return (z_cmd, error)

    def apply_heading_hold(self, x, y, current_yaw, pose_tracker, mode):
        """Add z correction and stop translation if yaw has drifted too far."""
        if not config.ENABLE_HEADING_HOLD:
            return (x, y, 0.0, mode, None)
        error = self.heading_error(current_yaw)
        if error is None:
            return (x, y, 0.0, mode, None)
        if self.heading_recovering:
            if abs(error) <= config.HEADING_RECOVER_RELEASE_DEG:
                self.heading_recovering = False
        elif abs(error) >= config.HEADING_RECOVER_TRIGGER_DEG:
            self.heading_recovering = True
        if self.heading_recovering:
            z_cmd, error = self.calculate_heading_hold(current_yaw, pose_tracker, recover=True)
            return (0.0, 0.0, z_cmd, 'HEADING_RECOVER', error)
        z_cmd, error = self.calculate_heading_hold(current_yaw, pose_tracker, recover=False)
        return (x, y, z_cmd, mode, error)

    def update_corridor_heading_reference(self, left_cm, right_cm, front_cm, pose_tracker, heading_index, allow=True):
        """Trim yaw so the chassis becomes parallel with stable corridor walls.

        V12.5 uses spatial Sharp trends rather than one instantaneous distance.
        A wall distance that changes while the robot travels straight implies a
        small yaw skew.  Dual-wall estimates must agree; single-wall estimates
        need more consecutive windows before they are allowed to move the yaw
        reference.  This keeps the correction useful without letting one noisy
        Sharp reading rotate the whole cardinal heading grid.
        """
        new_assist = bool(getattr(config, 'ENABLE_WALL_PARALLEL_ASSIST', False))
        legacy = bool(getattr(config, 'ENABLE_CORRIDOR_HEADING_CALIBRATION', False))
        if not (new_assist or legacy):
            return None
        if not allow or pose_tracker is None:
            self.reset_corridor_heading_calibration()
            return None

        if new_assist:
            min_travel = float(getattr(config, 'WALL_PARALLEL_MIN_TRAVEL_M', 0.12))
            min_wall = float(getattr(config, 'WALL_PARALLEL_MIN_WALL_CM', 5.8))
            max_wall = float(getattr(config, 'WALL_PARALLEL_MAX_WALL_CM', 18.0))
            min_est = float(getattr(config, 'WALL_PARALLEL_MIN_ESTIMATE_DEG', 0.9))
            max_est = float(getattr(config, 'WALL_PARALLEL_MAX_ESTIMATE_DEG', 9.0))
            agree_deg = float(getattr(config, 'WALL_PARALLEL_DUAL_AGREE_DEG', 4.0))
            max_step = float(getattr(config, 'WALL_PARALLEL_MAX_STEP_DEG', 0.75))
            log_step = float(getattr(config, 'WALL_PARALLEL_LOG_MIN_STEP_DEG', 0.12))
        else:
            min_travel = float(config.CORRIDOR_CAL_MIN_TRAVEL_M)
            min_wall = float(config.CORRIDOR_CAL_MIN_WALL_CM)
            max_wall = float(config.CORRIDOR_CAL_MAX_WALL_CM)
            min_est = 0.0
            max_est = float(config.CORRIDOR_CAL_MAX_ESTIMATE_DEG)
            agree_deg = 999.0
            max_step = float(config.CORRIDOR_CAL_MAX_STEP_DEG)
            log_step = float(config.CORRIDOR_CAL_LOG_MIN_STEP_DEG)

        min_front = float(getattr(config, 'CORRIDOR_CAL_MIN_FRONT_CM', 40.0))
        if front_cm is None or front_cm < min_front:
            self.reset_corridor_heading_calibration()
            return None
        if self.heading_right_step_sign not in (-1, 1):
            return None

        x, y, _ = pose_tracker.get_pose()
        if x is None or y is None:
            return None

        def wall_ok(v):
            return v is not None and min_wall <= float(v) <= max_wall

        left_ok = wall_ok(left_cm)
        right_ok = wall_ok(right_cm)
        if not (left_ok or right_ok):
            self.reset_corridor_heading_calibration()
            return None

        if self._corr_cal_x is None or self._corr_cal_heading_index != int(heading_index):
            self._corr_cal_x, self._corr_cal_y = (x, y)
            self._corr_cal_left = float(left_cm) if left_ok else None
            self._corr_cal_right = float(right_cm) if right_ok else None
            self._corr_cal_heading_index = int(heading_index)
            if hasattr(self, '_wall_parallel_history'):
                self._wall_parallel_history.clear()
            return None

        travel = math.hypot(x - self._corr_cal_x, y - self._corr_cal_y)
        if travel < min_travel:
            return None

        left_est = None
        right_est = None
        if left_ok and self._corr_cal_left is not None:
            dd_m = (float(left_cm) - self._corr_cal_left) / 100.0
            angle = math.degrees(math.atan2(dd_m, max(travel, 1e-06)))
            left_est = -self.heading_right_step_sign * angle
        if right_ok and self._corr_cal_right is not None:
            dd_m = (float(right_cm) - self._corr_cal_right) / 100.0
            angle = math.degrees(math.atan2(dd_m, max(travel, 1e-06)))
            right_est = self.heading_right_step_sign * angle

        # Always move the spatial baseline forward; one window must never be
        # counted repeatedly just because the next control cycles are close.
        self._corr_cal_x, self._corr_cal_y = (x, y)
        self._corr_cal_left = float(left_cm) if left_ok else None
        self._corr_cal_right = float(right_cm) if right_ok else None
        self._corr_cal_heading_index = int(heading_index)

        estimates = [v for v in (left_est, right_est) if v is not None]
        if not estimates:
            return None
        dual = left_est is not None and right_est is not None
        if dual and abs(left_est - right_est) > agree_deg:
            if hasattr(self, '_wall_parallel_history'):
                self._wall_parallel_history.clear()
            return None

        estimate = statistics.median(estimates)
        if abs(estimate) > max_est:
            if hasattr(self, '_wall_parallel_history'):
                self._wall_parallel_history.clear()
            return None
        if abs(estimate) < min_est:
            if hasattr(self, '_wall_parallel_history'):
                self._wall_parallel_history.clear()
            return 0.0

        if not hasattr(self, '_wall_parallel_history'):
            self._wall_parallel_history = deque(maxlen=3)
        self._wall_parallel_history.append((float(estimate), bool(dual)))

        need = int(getattr(config, 'WALL_PARALLEL_DUAL_CONFIRM_WINDOWS', 2) if dual else getattr(config, 'WALL_PARALLEL_SINGLE_CONFIRM_WINDOWS', 3)) if new_assist else 1
        if len(self._wall_parallel_history) < need:
            return None
        recent = list(self._wall_parallel_history)[-need:]
        # Do not combine a newly dual-wall estimate with old single-wall evidence.
        if new_assist and any(flag != dual for _, flag in recent):
            return None
        vals = [v for v, _ in recent]
        same_sign = all(v > 0.0 for v in vals) or all(v < 0.0 for v in vals)
        if not same_sign:
            self._wall_parallel_history.clear()
            return None

        stable_estimate = statistics.median(vals)
        if new_assist:
            gain = float(getattr(config, 'WALL_PARALLEL_DUAL_GAIN', 0.18) if dual else getattr(config, 'WALL_PARALLEL_SINGLE_GAIN', 0.10))
        else:
            gain = float(config.CORRIDOR_CAL_ALPHA)
        step = clamp(stable_estimate * gain, -max_step, max_step)
        self._wall_parallel_history.clear()
        if abs(step) < 0.02:
            return step

        self.heading_base_yaw = normalize_angle_deg(self.heading_base_yaw + step)
        if self.heading_target_yaw is not None:
            self.heading_target_yaw = normalize_angle_deg(self.heading_target_yaw + step)
        if abs(step) >= log_step:
            label = 'WALL_PARALLEL' if new_assist else 'CORRIDOR_HEADING_CAL'
            source = 'DUAL' if dual else 'SINGLE'
            print(f'>>> {label} {source} estimate={stable_estimate:+.2f}deg trim={step:+.2f}deg target={self.heading_target_yaw:+.2f}')
        return step

    @staticmethod
    def calculate_forward_speed(front_distance):
        if front_distance is None:
            return config.UNKNOWN_FRONT_SPEED
        if front_distance >= config.SLOW_FRONT_CM:
            return config.FORWARD_SPEED
        if front_distance <= config.STOP_FRONT_CM:
            return 0.0
        ratio = (front_distance - config.STOP_FRONT_CM) / (config.SLOW_FRONT_CM - config.STOP_FRONT_CM)
        return config.MIN_FORWARD_SPEED + ratio * (config.FORWARD_SPEED - config.MIN_FORWARD_SPEED)

    @staticmethod
    def _update_wall_state(distance_cm, current_state):
        if distance_cm is None:
            return False
        if current_state:
            return distance_cm < config.SIDE_WALL_EXIT_CM
        return distance_cm < config.SIDE_WALL_ENTER_CM

    def update_wall_states(self, sharp_left_cm, sharp_right_cm):
        left_is_opening = sharp_left_cm is not None and sharp_left_cm >= config.EXPLORATION_SIDE_OPEN_CM
        right_is_opening = sharp_right_cm is not None and sharp_right_cm >= config.EXPLORATION_SIDE_OPEN_CM
        if left_is_opening:
            self.left_wall_present = False
        else:
            self.left_wall_present = self._update_wall_state(sharp_left_cm, self.left_wall_present)
        if right_is_opening:
            self.right_wall_present = False
        else:
            self.right_wall_present = self._update_wall_state(sharp_right_cm, self.right_wall_present)
        return (self.left_wall_present, self.right_wall_present)

    def calculate_center_owner(self, sharp_left_cm, sharp_right_cm):
        now = time.time()
        delta = sharp_left_cm - sharp_right_cm
        abs_delta = abs(delta)
        if self.side_owner == 'NONE':
            if abs_delta < config.CENTER_TRIGGER_CM:
                return (0.0, 'CENTER_STABLE')
            self.side_owner = 'LEFT' if delta < 0 else 'RIGHT'
            self.side_owner_since = now
        self.side_owner_open_since = None
        owner_age = now - self.side_owner_since
        force_switch_cm = float(getattr(config, 'CENTER_OWNER_FORCE_SWITCH_CM', 4.0))
        if abs_delta >= force_switch_cm and (self.side_owner == 'LEFT' and delta > 0 or (self.side_owner == 'RIGHT' and delta < 0)):
            self.side_owner = 'RIGHT' if delta > 0 else 'LEFT'
            self.side_owner_since = now
            owner_age = 0.0
        if owner_age >= config.CENTER_HOLD_SEC:
            if abs_delta <= config.CENTER_RELEASE_CM:
                self.reset_side_owner()
                return (0.0, 'CENTER_RELEASE')
            if self.side_owner == 'LEFT' and delta >= config.CENTER_TRIGGER_CM:
                self.side_owner = 'RIGHT'
                self.side_owner_since = now
            elif self.side_owner == 'RIGHT' and delta <= -config.CENTER_TRIGGER_CM:
                self.side_owner = 'LEFT'
                self.side_owner_since = now
        correction = clamp(abs_delta * config.CENTER_KP_STRAFE, 0.0, config.CENTER_MAX_Y)
        if self.side_owner == 'LEFT':
            return (+correction * config.Y_DIR_SIGN, 'CENTER_LEFT_OWNER')
        if self.side_owner == 'RIGHT':
            return (-correction * config.Y_DIR_SIGN, 'CENTER_RIGHT_OWNER')
        return (0.0, 'CENTER_STABLE')

    def calculate_motion_control(self, raw_adc_l, sharp_left_cm, raw_adc_r, sharp_right_cm, front_ir_blocked=False):
        """Sharp controls Y; heading hold controls Z.

        front_ir_blocked is accepted for integration/debug compatibility; the
        final forward-motion veto lives in maze_runtime.apply_motion_safety().
        """
        _ = (raw_adc_l, raw_adc_r, front_ir_blocked)
        if sharp_left_cm is None or sharp_right_cm is None:
            self.reset_side_owner()
            self.reset_wall_states()
            return (0.0, 0.0, 'NO_SENSOR')
        if sharp_left_cm <= config.SIDE_TOO_CLOSE_CM and sharp_right_cm <= config.SIDE_TOO_CLOSE_CM:
            self.reset_side_owner()
            return (0.0, 0.0, 'BOTH_TOO_CLOSE')
        if sharp_left_cm <= config.SIDE_TOO_CLOSE_CM:
            self.reset_side_owner()
            return (+config.ESCAPE_Y_SPEED * config.Y_DIR_SIGN, 0.0, 'ESCAPE_LEFT')
        if sharp_right_cm <= config.SIDE_TOO_CLOSE_CM:
            self.reset_side_owner()
            return (-config.ESCAPE_Y_SPEED * config.Y_DIR_SIGN, 0.0, 'ESCAPE_RIGHT')
        left_wall, right_wall = self.update_wall_states(sharp_left_cm, sharp_right_cm)
        if left_wall and right_wall:
            y_cmd, mode = self.calculate_center_owner(sharp_left_cm, sharp_right_cm)
            return (y_cmd, 0.0, mode)
        if left_wall:
            if not bool(getattr(config, 'OWNER_PERSIST_THROUGH_SINGLE_WALL', True)):
                self.reset_side_owner()
            else:
                self.side_owner_open_since = None
            error = sharp_left_cm - config.TARGET_LEFT_CM
            if abs(error) <= config.SIDE_DEADBAND_CM:
                y_cmd = 0.0
            else:
                y_cmd = clamp(-error * config.SIDE_KP_STRAFE * config.Y_DIR_SIGN, -config.SIDE_MAX_Y, config.SIDE_MAX_Y)
            return (y_cmd, 0.0, 'FOLLOW_LEFT')
        if right_wall:
            if not bool(getattr(config, 'OWNER_PERSIST_THROUGH_SINGLE_WALL', True)):
                self.reset_side_owner()
            else:
                self.side_owner_open_since = None
            error = sharp_right_cm - config.TARGET_RIGHT_CM
            if abs(error) <= config.SIDE_DEADBAND_CM:
                y_cmd = 0.0
            else:
                y_cmd = clamp(error * config.SIDE_KP_STRAFE * config.Y_DIR_SIGN, -config.SIDE_MAX_Y, config.SIDE_MAX_Y)
            return (y_cmd, 0.0, 'FOLLOW_RIGHT')
        if bool(getattr(config, 'OWNER_PERSIST_THROUGH_SINGLE_WALL', True)) and self.side_owner != 'NONE':
            now = time.time()
            if self.side_owner_open_since is None:
                self.side_owner_open_since = now
            elif now - self.side_owner_open_since >= float(getattr(config, 'OWNER_PERSIST_OPEN_SEC', 0.75)):
                self.reset_side_owner()
        else:
            self.reset_side_owner()
        return (0.0, 0.0, 'OPEN_SPACE')

@dataclass(frozen=True)

# ==================== NAVIGATION / FEEDBACK TURN ====================
class TurnDecision:
    name: str
    angle_deg: float
RELATIVE_TO_TURN = {'FRONT': TurnDecision('FRONT', 0.0), 'FORWARD': TurnDecision('FRONT', 0.0), 'LEFT': TurnDecision('LEFT_90', config.TURN_LEFT_DEG), 'RIGHT': TurnDecision('RIGHT_90', config.TURN_RIGHT_DEG), 'BACK': TurnDecision('BACK_180', config.TURN_AROUND_DEG), 'COMPLETE': TurnDecision('COMPLETE', 0.0)}

def decision_from_relative(relative_direction):
    try:
        return RELATIVE_TO_TURN[relative_direction]
    except KeyError as exc:
        raise ValueError(f'Unknown relative direction: {relative_direction}') from exc

def print_exploration_decision(exploration_decision):
    print()
    print('========== TRÉMAUX / DFS DECISION ==========')
    print(f'NODE       : {exploration_decision.node_id}')
    print(f'DIRECTION  : {exploration_decision.direction}')
    print(f'ABS HEADING: {exploration_decision.absolute_heading}')
    print(f'MARK BEFORE: {exploration_decision.visits_before}')
    print(f'REASON     : {exploration_decision.reason}')
    print('============================================')

def _safe_stop(chassis):
    try:
        chassis.drive_speed(x=0.0, y=0.0, z=0.0, timeout=0.2)
    except Exception:
        pass

def _control_clamp(value, low, high):
    return max(low, min(high, value))

def _control_pose_xy(pose_tracker):
    if pose_tracker is None:
        return (None, None)
    x, y, _ = pose_tracker.get_pose()
    return (x, y)

def _control_travelled_m(start_x, start_y, pose_tracker):
    x, y = _control_pose_xy(pose_tracker)
    if None in (start_x, start_y, x, y):
        return None
    return math.hypot(float(x) - float(start_x), float(y) - float(start_y))

def _confirmed_ir_hit(ir_state):
    if not ir_state:
        return False
    return bool(ir_state.get('left_confirmed', False) or ir_state.get('right_confirmed', False))

def _raw_ir_hit(ir_state):
    if not ir_state:
        return False
    return bool(ir_state.get('left_hit', False) or ir_state.get('right_hit', False))

def _confirm_raw_turn_hit(sensors, initial_raw):
    """Stop first on RAW hit, then confirm while stationary.

    A corner can sweep into a wall faster than a two-sample moving debounce.  V11
    therefore treats the first raw edge as an emergency stop.  Recovery itself is
    only started if stopped rechecks keep seeing the hit.
    """
    left_votes = 1 if initial_raw.get('left_hit') else 0
    right_votes = 1 if initial_raw.get('right_hit') else 0
    n = max(1, int(getattr(config, 'TURN_IR_RAW_RECHECK_SAMPLES', 2)))
    for _ in range(n):
        time.sleep(float(getattr(config, 'TURN_IR_RAW_RECHECK_INTERVAL_SEC', 0.025)))
        raw = sensors.read_front_ir_raw_state()
        left_votes += 1 if raw.get('left_hit') else 0
        right_votes += 1 if raw.get('right_hit') else 0
        sensors.update_front_ir_guard(raw.get('left_raw'), raw.get('right_raw'))
    needed = max(1, (n + 1) // 2)
    return {'left_confirmed': left_votes >= needed, 'right_confirmed': right_votes >= needed, 'left_raw': initial_raw.get('left_raw'), 'right_raw': initial_raw.get('right_raw')}

def _turn_ir_recovery(chassis, sensors, pose_tracker, ir_state, recovery_index):
    """Pause rotation, make a short mecanum clearance move, then stop.

    The caller keeps the original absolute target yaw.  Therefore recovery does
    not add another 90 degrees; after translation the feedback loop simply
    finishes whatever yaw error remains.
    """
    if sensors is None:
        return False
    left = bool(ir_state.get('left_confirmed', False))
    right = bool(ir_state.get('right_confirmed', False))
    if not (left or right):
        return False
    _safe_stop(chassis)
    time.sleep(0.03)
    x_cmd = 0.0
    y_cmd = 0.0
    kind = ''
    target_m = float(config.TURN_IR_SINGLE_ESCAPE_M)
    if left and right:
        kind = 'BOTH_BACK'
        x_cmd = -abs(float(config.TURN_IR_ESCAPE_BACK_SPEED))
        target_m = float(config.TURN_IR_BOTH_ESCAPE_M)
    elif left:
        _, opposite_cm = sensors.read_right_sharp()
        if opposite_cm is not None and opposite_cm <= config.TURN_IR_OPPOSITE_SIDE_MIN_CM:
            kind = 'LEFT_HIT_BACK_FALLBACK'
            x_cmd = -abs(float(config.TURN_IR_ESCAPE_BACK_SPEED))
        else:
            kind = 'LEFT_HIT_STRAFE_RIGHT'
            x_cmd = -abs(float(config.TURN_IR_SINGLE_BACK_BIAS_SPEED))
            y_cmd = +abs(float(config.TURN_IR_ESCAPE_Y_SPEED)) * config.Y_DIR_SIGN
    else:
        _, opposite_cm = sensors.read_left_sharp()
        if opposite_cm is not None and opposite_cm <= config.TURN_IR_OPPOSITE_SIDE_MIN_CM:
            kind = 'RIGHT_HIT_BACK_FALLBACK'
            x_cmd = -abs(float(config.TURN_IR_ESCAPE_BACK_SPEED))
        else:
            kind = 'RIGHT_HIT_STRAFE_LEFT'
            x_cmd = -abs(float(config.TURN_IR_SINGLE_BACK_BIAS_SPEED))
            y_cmd = -abs(float(config.TURN_IR_ESCAPE_Y_SPEED)) * config.Y_DIR_SIGN
    start_x, start_y = _control_pose_xy(pose_tracker)
    started = time.monotonic()
    switched_to_back = False
    print(f'>>> TURN IR RECOVERY #{recovery_index}: {kind} L={int(left)} R={int(right)} target={target_m:.3f}m cmd=({x_cmd:+.3f},{y_cmd:+.3f})')
    while time.monotonic() - started < float(config.TURN_IR_RECOVERY_MAX_SEC):
        state = sensors.update_front_ir_guard(refresh=True)
        travelled = _control_travelled_m(start_x, start_y, pose_tracker)
        if abs(y_cmd) > 1e-06:
            if y_cmd * config.Y_DIR_SIGN > 0:
                _, opposite_cm = sensors.read_right_sharp()
            else:
                _, opposite_cm = sensors.read_left_sharp()
            if opposite_cm is not None and opposite_cm <= float(config.TURN_IR_OPPOSITE_SIDE_MIN_CM):
                x_cmd = -abs(float(config.TURN_IR_ESCAPE_BACK_SPEED))
                y_cmd = 0.0
                if not switched_to_back:
                    print('>>> TURN IR RECOVERY: opposite wall too close; switching to short reverse')
                    switched_to_back = True
        chassis.drive_speed(x=x_cmd, y=y_cmd, z=0.0, timeout=max(0.1, float(config.TURN_IR_RECOVERY_LOOP_SEC) * 3.0))
        if travelled is not None and travelled >= target_m:
            break
        if travelled is not None and travelled >= float(config.TURN_IR_RECOVERY_MIN_M):
            if not _confirmed_ir_hit(state):
                break
        time.sleep(float(config.TURN_IR_RECOVERY_LOOP_SEC))
    _safe_stop(chassis)
    time.sleep(float(config.TURN_IR_RECOVERY_SETTLE_SEC))
    for _ in range(max(1, int(config.TURN_IR_RECHECK_SAMPLES))):
        sensors.update_front_ir_guard(refresh=True)
        time.sleep(float(config.TURN_IR_RECOVERY_LOOP_SEC))
    travelled = _control_travelled_m(start_x, start_y, pose_tracker)
    print('>>> TURN IR RECOVERY DONE' + (f' travelled={travelled:.3f}m' if travelled is not None else ''))
    return True

def _feedback_turn(chassis, decision, pose_tracker, sensors=None):
    """Closed-loop yaw turn with interruptible front-IR collision recovery."""
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
    target_yaw = normalize_angle_deg(start_yaw + command_deg * move_sign)
    timeout_sec = config.TURN_FEEDBACK_TIMEOUT_180_SEC if abs(command_deg) > 135.0 else config.TURN_FEEDBACK_TIMEOUT_90_SEC
    max_attempts = max(1, int(getattr(config, 'TURN_MAX_ATTEMPTS', 1)))
    timeout_accept = float(getattr(config, 'TURN_TIMEOUT_ACCEPT_TOLERANCE_DEG', config.TURN_FEEDBACK_TOLERANCE_DEG))
    print(f'>>> TURN {decision.name} [FEEDBACK+IR]: command={command_deg:+.1f} deg start_yaw={start_yaw:+.1f} target={target_yaw:+.1f} max_attempts={max_attempts}')
    _safe_stop(chassis)
    time.sleep(config.TURN_PRE_SETTLE_SEC)
    recoveries = 0
    try:
        for attempt in range(1, max_attempts + 1):
            if attempt > 1:
                current = pose_tracker.get_yaw()
                remaining = shortest_angle_error_deg(target_yaw, current) if current is not None else None
                print(f'>>> TURN RETRY {attempt}/{max_attempts} SAME TARGET ' + (f'yaw={current:+.1f} remaining={remaining:+.1f} deg' if current is not None else 'yaw unavailable'))
                _safe_stop(chassis)
                time.sleep(getattr(config, 'TURN_RETRY_SETTLE_SEC', 0.25))
            started = time.monotonic()
            stable_samples = 0
            last_print = 0.0
            while True:
                now = time.monotonic()
                current_yaw = pose_tracker.get_yaw()
                if current_yaw is None:
                    if now - started >= timeout_sec:
                        _safe_stop(chassis)
                        print(f'TURN ATTEMPT {attempt}/{max_attempts} TIMEOUT: attitude yaw unavailable.')
                        break
                    time.sleep(config.TURN_FEEDBACK_LOOP_SEC)
                    continue
                error = shortest_angle_error_deg(target_yaw, current_yaw)
                abs_error = abs(error)
                progress_deg = abs(shortest_angle_error_deg(current_yaw, start_yaw))
                if sensors is not None and bool(getattr(config, 'ENABLE_TURN_IR_COLLISION_GUARD', True)) and (progress_deg >= float(config.TURN_IR_ARM_AFTER_DEG)):
                    if bool(getattr(config, 'TURN_IR_USE_RAW_EMERGENCY', True)):
                        raw_ir = sensors.read_front_ir_raw_state()
                        if _raw_ir_hit(raw_ir):
                            _safe_stop(chassis)
                            ir_state = _confirm_raw_turn_hit(sensors, raw_ir)
                        else:
                            ir_state = sensors.update_front_ir_guard(raw_ir.get('left_raw'), raw_ir.get('right_raw'))
                    else:
                        ir_state = sensors.update_front_ir_guard(refresh=True)
                    if _confirmed_ir_hit(ir_state):
                        recoveries += 1
                        if recoveries > int(config.TURN_IR_MAX_RECOVERIES):
                            _safe_stop(chassis)
                            print('>>> TURN IR ABORT: recovery limit exceeded; stopping instead of forcing rotation')
                            return False
                        _turn_ir_recovery(chassis, sensors, pose_tracker, ir_state, recoveries)
                        started = time.monotonic()
                        stable_samples = 0
                        continue
                if abs_error <= config.TURN_FEEDBACK_TOLERANCE_DEG:
                    stable_samples += 1
                    _safe_stop(chassis)
                    if stable_samples >= config.TURN_FEEDBACK_STABLE_SAMPLES:
                        time.sleep(config.YAW_SETTLE_SEC)
                        final_yaw = pose_tracker.get_yaw()
                        final_error = shortest_angle_error_deg(target_yaw, final_yaw) if final_yaw is not None else error
                        print(f'TURN OK: yaw={final_yaw:+.1f} target={target_yaw:+.1f} error={final_error:+.1f} deg recoveries={recoveries}' if final_yaw is not None else 'TURN OK')
                        return True
                else:
                    stable_samples = 0
                    speed = abs_error * config.TURN_FEEDBACK_KP
                    speed = _control_clamp(speed, config.TURN_FEEDBACK_MIN_Z_SPEED, config.TURN_FEEDBACK_MAX_Z_SPEED)
                    z_cmd = math.copysign(speed, error) / drive_sign
                    chassis.drive_speed(x=0.0, y=0.0, z=z_cmd, timeout=config.TURN_FEEDBACK_DRIVE_TIMEOUT_SEC)
                    if now - last_print >= config.TURN_FEEDBACK_PRINT_SEC:
                        print(f'    turn yaw={current_yaw:+.1f} target={target_yaw:+.1f} err={error:+.1f} z={z_cmd:+.1f} progress={progress_deg:.1f} IRrec={recoveries} attempt={attempt}/{max_attempts}')
                        last_print = now
                if now - started >= timeout_sec:
                    _safe_stop(chassis)
                    final_yaw = pose_tracker.get_yaw()
                    final_error = shortest_angle_error_deg(target_yaw, final_yaw) if final_yaw is not None else None
                    if final_error is not None and abs(final_error) <= timeout_accept:
                        print(f'TURN OK AT TIMEOUT: yaw={final_yaw:+.1f} target={target_yaw:+.1f} error={final_error:+.1f} deg (accept<={timeout_accept:.1f})')
                        return True
                    print(f'TURN ATTEMPT {attempt}/{max_attempts} WATCHDOG TIMEOUT: ' + (f'yaw={final_yaw:+.1f} target={target_yaw:+.1f} error={final_error:+.1f} deg' if final_yaw is not None else 'yaw unavailable'))
                    break
                time.sleep(config.TURN_FEEDBACK_LOOP_SEC)
        _safe_stop(chassis)
        final_yaw = pose_tracker.get_yaw()
        final_error = shortest_angle_error_deg(target_yaw, final_yaw) if final_yaw is not None else None
        print('TURN FAILED AFTER RETRIES: ' + (f'yaw={final_yaw:+.1f} target={target_yaw:+.1f} error={final_error:+.1f} deg' if final_yaw is not None else 'yaw unavailable'))
        return False
    except KeyboardInterrupt:
        _safe_stop(chassis)
        raise
    except Exception as exc:
        _safe_stop(chassis)
        print(f'TURN FEEDBACK ERROR: {exc}')
        return False

def _action_turn_with_timeout(chassis, decision, pose_tracker=None, sensors=None):
    """Finite-time fallback, split into small chunks when IR guard is enabled."""
    command_deg = decision.angle_deg * config.Z_DIR_SIGN
    if abs(command_deg) < 0.001:
        return True
    start_yaw = pose_tracker.get_yaw() if pose_tracker is not None else None
    print(f'>>> TURN {decision.name} [ACTION FALLBACK]: command={command_deg:+.1f} deg' + (f' start_yaw={start_yaw:+.1f}' if start_yaw is not None else ''))
    _safe_stop(chassis)
    time.sleep(config.TURN_PRE_SETTLE_SEC)
    timeout_sec = config.TURN_ACTION_TIMEOUT_180_SEC if abs(command_deg) > 135.0 else config.TURN_ACTION_TIMEOUT_90_SEC
    use_ir = sensors is not None and bool(getattr(config, 'ENABLE_TURN_IR_COLLISION_GUARD', True))
    if not use_ir:
        try:
            action = chassis.move(x=0, y=0, z=command_deg, z_speed=config.TURN_SPEED)
            completed = action.wait_for_completed(timeout=timeout_sec)
        except KeyboardInterrupt:
            _safe_stop(chassis)
            raise
        except Exception as exc:
            _safe_stop(chassis)
            print(f'TURN ACTION ERROR: {exc}')
            return False
        if not completed:
            _safe_stop(chassis)
            print(f'TURN ACTION TIMEOUT after {timeout_sec:.1f}s - stopped safely.')
            return False
        return True
    remaining = float(command_deg)
    cumulative = 0.0
    recoveries = 0
    step_mag = max(2.0, abs(float(config.TURN_IR_ACTION_STEP_DEG)))
    started = time.monotonic()
    while abs(remaining) > 0.01:
        if time.monotonic() - started > timeout_sec + 4.0:
            _safe_stop(chassis)
            print('TURN ACTION CHUNKED TIMEOUT - stopped safely.')
            return False
        step = math.copysign(min(step_mag, abs(remaining)), remaining)
        if cumulative >= float(config.TURN_IR_ARM_AFTER_DEG):
            raw = sensors.read_front_ir_raw_state()
            if _raw_ir_hit(raw):
                _safe_stop(chassis)
                state = _confirm_raw_turn_hit(sensors, raw)
                if _confirmed_ir_hit(state):
                    recoveries += 1
                    if recoveries > int(config.TURN_IR_MAX_RECOVERIES):
                        print('TURN IR ABORT in action fallback: recovery limit exceeded')
                        return False
                    _turn_ir_recovery(chassis, sensors, pose_tracker, state, recoveries)
        try:
            action = chassis.move(x=0, y=0, z=step, z_speed=config.TURN_SPEED)
            if not action.wait_for_completed(timeout=max(0.6, timeout_sec)):
                _safe_stop(chassis)
                print('TURN ACTION CHUNK timeout')
                return False
        except Exception as exc:
            _safe_stop(chassis)
            print(f'TURN ACTION CHUNK ERROR: {exc}')
            return False
        remaining -= step
        cumulative += abs(step)
        _safe_stop(chassis)
        if cumulative >= float(config.TURN_IR_ARM_AFTER_DEG):
            state = sensors.update_front_ir_guard(refresh=True)
            if _confirmed_ir_hit(state):
                recoveries += 1
                if recoveries > int(config.TURN_IR_MAX_RECOVERIES):
                    print('TURN IR ABORT in action fallback: recovery limit exceeded')
                    return False
                _turn_ir_recovery(chassis, sensors, pose_tracker, state, recoveries)
    _safe_stop(chassis)
    return True

def execute_turn(chassis, decision, pose_tracker=None, sensors=None):
    """Execute one bounded turn; front IR may interrupt/reposition/resume."""
    if not config.ENABLE_MOTION:
        return True
    command_deg = decision.angle_deg * config.Z_DIR_SIGN
    if abs(command_deg) < 0.001:
        return True
    if config.ENABLE_FEEDBACK_TURN and pose_tracker is not None and (pose_tracker.get_yaw() is not None):
        result = _feedback_turn(chassis, decision, pose_tracker, sensors=sensors)
        if result is not None:
            return result
    return _action_turn_with_timeout(chassis, decision, pose_tracker=pose_tracker, sensors=sensors)
HEADINGS = ('N', 'E', 'S', 'W')
RELATIVE_ORDER = ('FRONT', 'RIGHT', 'BACK', 'LEFT')
RELATIVE_OFFSET = {'FRONT': 0, 'RIGHT': 1, 'BACK': 2, 'LEFT': -1}

@dataclass

# ==================== TRÉMAUX GRAPH / JUNCTION DETECTION ====================
class ExitState:
    visits: int = 0
    target: Optional[str] = None
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
        self._latch_clear_x = None
        self._latch_clear_y = None

    @staticmethod
    def _new_zone():
        return {'candidate_count': 0, 'candidate_start_x': None, 'candidate_start_y': None, 'active': False, 'start_x': None, 'start_y': None, 'exit_count': 0, 'max_cm': 0.0}

    @staticmethod
    def _new_intersection_window():
        return {'active': False, 'start_x': None, 'start_y': None, 'last_side_open_x': None, 'last_side_open_y': None, 'lookahead_start_x': None, 'lookahead_start_y': None, 'front_open_samples': 0, 'left_open_samples': 0, 'right_open_samples': 0, 'front_max_cm': 0.0, 'left_max_cm': 0.0, 'right_max_cm': 0.0, 'completed_sides': set()}

    @staticmethod
    def classify_openings(front_cm, left_cm, right_cm):
        """Stopped-scan classification using strict thresholds."""
        front_open = front_cm is not None and front_cm >= config.EXPLORATION_FRONT_OPEN_CM
        front_blocked = front_cm is not None and 0.0 < front_cm <= config.STOP_FRONT_CM
        left_open = left_cm is not None and left_cm >= config.SIDE_OPEN_ENTER_CM
        right_open = right_cm is not None and right_cm >= config.SIDE_OPEN_ENTER_CM
        return (front_open, front_blocked, left_open, right_open)

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
        self._latch_clear_x = None
        self._latch_clear_y = None
        self._reset_zones()

    def _latch_now(self, now, pose_x, pose_y):
        self.latched = True
        self.front_candidate_count = 0
        self.clear_count = 0
        self.latch_x = float(pose_x) if pose_x is not None else None
        self.latch_y = float(pose_y) if pose_y is not None else None
        self.latch_time = now
        self._latch_clear_x = None
        self._latch_clear_y = None

    def _track_side_zone(self, zone, side_cm, pose_x, pose_y, side_name):
        """Track one physical side-opening mouth; return completion metadata."""
        if side_cm is None:
            return None
        enter = float(config.SIDE_OPEN_ENTER_CM)
        exit_threshold = float(config.SIDE_OPEN_EXIT_CM)
        if not zone['active']:
            if side_cm >= enter:
                if zone['candidate_count'] == 0:
                    zone['candidate_start_x'] = pose_x
                    zone['candidate_start_y'] = pose_y
                zone['candidate_count'] += 1
                zone['max_cm'] = max(zone['max_cm'], float(side_cm))
                if zone['candidate_count'] >= config.OPENING_ZONE_ENTER_SAMPLES:
                    zone['active'] = True
                    zone['start_x'] = zone['candidate_start_x']
                    zone['start_y'] = zone['candidate_start_y']
                    zone['exit_count'] = 0
                    print(f'>>> OPENING_ZONE {side_name} START Sharp={side_cm:.1f}cm enter>={enter:.1f}')
            else:
                zone.update(self._new_zone())
            return None
        zone['max_cm'] = max(zone['max_cm'], float(side_cm))
        length = self._distance_xy(zone['start_x'], zone['start_y'], pose_x, pose_y)
        if side_cm < exit_threshold:
            zone['exit_count'] += 1
        else:
            zone['exit_count'] = 0
        completed_by_exit = zone['exit_count'] >= config.OPENING_ZONE_EXIT_SAMPLES
        completed_by_max = length is not None and length >= config.OPENING_ZONE_MAX_LENGTH_M
        if not (completed_by_exit or completed_by_max):
            return None
        if length is None or length < config.OPENING_ZONE_MIN_LENGTH_M:
            print(f'>>> OPENING_ZONE {side_name} REJECT length={(0.0 if length is None else length):.3f}m < {config.OPENING_ZONE_MIN_LENGTH_M:.3f}m')
            zone.update(self._new_zone())
            return None
        event = {'type': 'SIDE_OPENING_ZONE', 'side': side_name, 'length_m': float(length), 'start_x': zone['start_x'], 'start_y': zone['start_y'], 'end_x': pose_x, 'end_y': pose_y, 'max_cm': float(zone['max_cm']), 'forced_by_max': bool(completed_by_max and (not completed_by_exit))}
        print(f">>> OPENING_ZONE {side_name} END length={event['length_m']:.3f}m maxSharp={event['max_cm']:.1f}cm")
        zone.update(self._new_zone())
        return event

    def _start_intersection_window(self, pose_x, pose_y):
        if self.intersection_window['active']:
            return
        starts = []
        for side_name, zone in (('LEFT', self.left_zone), ('RIGHT', self.right_zone)):
            if zone['active']:
                starts.append((side_name, zone['start_x'], zone['start_y']))
        if not starts:
            return
        side_name, start_x, start_y = starts[0]
        w = self.intersection_window
        w['active'] = True
        w['start_x'] = start_x if start_x is not None else pose_x
        w['start_y'] = start_y if start_y is not None else pose_y
        w['last_side_open_x'] = pose_x
        w['last_side_open_y'] = pose_y
        if self.left_zone['active']:
            w['left_open_samples'] = max(w['left_open_samples'], config.OPENING_ZONE_ENTER_SAMPLES)
        if self.right_zone['active']:
            w['right_open_samples'] = max(w['right_open_samples'], config.OPENING_ZONE_ENTER_SAMPLES)
        print(f'>>> INTERSECTION_WINDOW START by={side_name} lookahead={config.INTERSECTION_WINDOW_LOOKAHEAD_M:.2f}m')

    def _accumulate_intersection(self, front_cm, left_cm, right_cm, pose_x, pose_y):
        w = self.intersection_window
        if not w['active']:
            return
        if front_cm is not None:
            w['front_max_cm'] = max(w['front_max_cm'], float(front_cm))
            if front_cm >= config.INTERSECTION_FRONT_OPEN_CM:
                w['front_open_samples'] += 1
        left_phys_open = left_cm is not None and left_cm >= config.INTERSECTION_SIDE_OPEN_CM
        right_phys_open = right_cm is not None and right_cm >= config.INTERSECTION_SIDE_OPEN_CM
        if left_cm is not None:
            w['left_max_cm'] = max(w['left_max_cm'], float(left_cm))
        if right_cm is not None:
            w['right_max_cm'] = max(w['right_max_cm'], float(right_cm))
        if left_phys_open:
            w['left_open_samples'] += 1
        if right_phys_open:
            w['right_open_samples'] += 1
        left_still_open = left_cm is not None and left_cm >= config.SIDE_OPEN_EXIT_CM
        right_still_open = right_cm is not None and right_cm >= config.SIDE_OPEN_EXIT_CM
        if left_still_open or right_still_open or self.left_zone['active'] or self.right_zone['active']:
            w['last_side_open_x'] = pose_x
            w['last_side_open_y'] = pose_y
            w['lookahead_start_x'] = None
            w['lookahead_start_y'] = None
        elif w['lookahead_start_x'] is None:
            w['lookahead_start_x'] = pose_x
            w['lookahead_start_y'] = pose_y
            print(f'>>> INTERSECTION_WINDOW LOOKAHEAD target={config.INTERSECTION_WINDOW_LOOKAHEAD_M:.2f}m')

    def _intersection_should_finalize(self, pose_x, pose_y):
        w = self.intersection_window
        if not w['active']:
            return (False, None)
        total = self._distance_xy(w['start_x'], w['start_y'], pose_x, pose_y)
        if total is not None and total >= config.INTERSECTION_WINDOW_MAX_M:
            return (True, 'MAX_LENGTH')
        if w['lookahead_start_x'] is not None:
            lookahead = self._distance_xy(w['lookahead_start_x'], w['lookahead_start_y'], pose_x, pose_y)
            if lookahead is not None and lookahead >= config.INTERSECTION_WINDOW_LOOKAHEAD_M:
                return (True, 'LOOKAHEAD_COMPLETE')
        return (False, None)

    def _finalize_intersection_window(self, pose_x, pose_y, reason):
        w = self.intersection_window
        if not w['active']:
            return None
        total_length = self._distance_xy(w['start_x'], w['start_y'], pose_x, pose_y)
        opening_span = self._distance_xy(w['start_x'], w['start_y'], w['last_side_open_x'], w['last_side_open_y'])
        total_length = float(total_length or 0.0)
        opening_span = float(opening_span or total_length)
        backtrack_m = max(0.0, total_length - 0.5 * opening_span)
        clear_after_opening_m = self._distance_xy(w['last_side_open_x'], w['last_side_open_y'], pose_x, pose_y)
        clear_after_opening_m = float(clear_after_opening_m or 0.0)
        min_samples = int(config.INTERSECTION_MIN_OPEN_SAMPLES)
        observed = {'FRONT': w['front_open_samples'] >= min_samples, 'LEFT': w['left_open_samples'] >= min_samples, 'RIGHT': w['right_open_samples'] >= min_samples}
        counts = {'FRONT': int(w['front_open_samples']), 'LEFT': int(w['left_open_samples']), 'RIGHT': int(w['right_open_samples'])}
        event = {'type': 'INTERSECTION_WINDOW', 'side': 'MULTI', 'length_m': total_length, 'opening_span_m': opening_span, 'backtrack_m': backtrack_m, 'clear_after_opening_m': clear_after_opening_m, 'start_x': w['start_x'], 'start_y': w['start_y'], 'end_x': pose_x, 'end_y': pose_y, 'observed_open': observed, 'open_samples': counts, 'max_cm': {'FRONT': float(w['front_max_cm']), 'LEFT': float(w['left_max_cm']), 'RIGHT': float(w['right_max_cm'])}, 'completed_sides': sorted(w['completed_sides']), 'finish_reason': reason}
        print(f'>>> INTERSECTION_WINDOW END reason={reason} length={total_length:.3f}m span={opening_span:.3f}m clear={clear_after_opening_m:.3f}m backtrack={backtrack_m:.3f}m')
        print(f">>> INTERSECTION_ACCUM F={int(observed['FRONT'])}({counts['FRONT']}) L={int(observed['LEFT'])}({counts['LEFT']}) R={int(observed['RIGHT'])}({counts['RIGHT']})")
        self.intersection_window = self._new_intersection_window()
        return event

    def consume_pending_zone(self):
        event = self.pending_zone_event
        self.pending_zone_event = None
        return event

    def force_finalize_active_intersection(
        self, pose_x, pose_y, reason='VIRTUAL_BARRIER', minimum_length_m=0.0,
    ):
        """Close a proven side-opening window before crossing a safety barrier.

        This never invents an opening: the normal tracker must already have
        started an intersection window from stable Sharp evidence. The event is
        consumed by the ordinary rolling scan and guide/DFS decision pipeline.
        """
        w = self.intersection_window
        if not w.get('active', False):
            return None
        min_samples = int(config.INTERSECTION_MIN_OPEN_SAMPLES)
        side_samples = max(
            int(w.get('left_open_samples', 0)),
            int(w.get('right_open_samples', 0)),
        )
        if side_samples < min_samples:
            return None
        length = self._distance_xy(w.get('start_x'), w.get('start_y'), pose_x, pose_y)
        length = float(length or 0.0)
        if length < max(0.0, float(minimum_length_m)):
            return None
        event = self._finalize_intersection_window(pose_x, pose_y, reason)
        if event is None:
            return None
        self.pending_zone_event = event
        self._latch_now(time.monotonic(), pose_x, pose_y)
        return event

    def release_after_front_drive_through(self, event):
        """Fast re-arm after FRONT only when a corridor gap was already proven."""
        if not bool(getattr(config, 'ENABLE_ADAPTIVE_JUNCTION_REGION', False)):
            return False
        if not self.latched or not event or event.get('type') != 'INTERSECTION_WINDOW':
            return False
        clear_m = float(event.get('clear_after_opening_m', 0.0) or 0.0)
        need = float(getattr(config, 'JUNCTION_REGION_FRONT_REARM_CLEAR_M', 0.12))
        if event.get('finish_reason') != 'LOOKAHEAD_COMPLETE' or clear_m < need:
            return False
        if bool(getattr(config, 'JUNCTION_REGION_DEBUG', True)):
            print(f'>>> JUNCTION_REGION FRONT REARM clear={clear_m:.3f}m >= {need:.3f}m')
        self._release_latch()
        return True

    def update(self, front_cm, left_cm, right_cm, pose_x=None, pose_y=None):
        _, front_blocked, _, _ = self.classify_openings(front_cm, left_cm, right_cm)
        now = time.monotonic()
        left_still_open = left_cm is not None and left_cm >= config.SIDE_OPEN_EXIT_CM
        right_still_open = right_cm is not None and right_cm >= config.SIDE_OPEN_EXIT_CM
        if self.latched:
            normal_corridor = not left_still_open and (not right_still_open) and (not front_blocked)
            self.clear_count = self.clear_count + 1 if normal_corridor else 0
            distance = self._distance_from_latch(pose_x, pose_y)
            elapsed = now - self.latch_time if self.latch_time is not None else None
            moved_minimum = distance is None or distance >= config.JUNCTION_REARM_MIN_DISTANCE_M
            released_by_corridor = self.clear_count >= config.JUNCTION_REARM_SAMPLES and moved_minimum
            released_by_distance = distance is not None and distance >= config.JUNCTION_REARM_DISTANCE_M and (not left_still_open) and (not right_still_open)
            released_by_timeout = elapsed is not None and elapsed >= config.JUNCTION_REARM_TIMEOUT_SEC and (distance is not None) and (distance >= config.JUNCTION_REARM_MIN_DISTANCE_M) and (not left_still_open) and (not right_still_open)
            released_by_emergency_front = front_blocked and elapsed is not None and (elapsed >= config.JUNCTION_REARM_EMERGENCY_SEC)

            released_by_spatial_corridor = False
            clear_distance = None
            if bool(getattr(config, 'ENABLE_ADAPTIVE_JUNCTION_REGION', False)):
                wall_limit = float(getattr(config, 'JUNCTION_REGION_WALL_REACQUIRE_CM', 16.5))
                walls_reacquired = (left_cm is not None and 0.0 < float(left_cm) <= wall_limit and right_cm is not None and 0.0 < float(right_cm) <= wall_limit and (not front_blocked))
                if walls_reacquired:
                    if self._latch_clear_x is None:
                        self._latch_clear_x = pose_x
                        self._latch_clear_y = pose_y
                    clear_distance = self._distance_xy(self._latch_clear_x, self._latch_clear_y, pose_x, pose_y)
                    released_by_spatial_corridor = clear_distance is not None and clear_distance >= float(getattr(config, 'JUNCTION_REGION_CLEAR_M', 0.12))
                else:
                    self._latch_clear_x = None
                    self._latch_clear_y = None

            if not (released_by_spatial_corridor or released_by_corridor or released_by_distance or released_by_timeout or released_by_emergency_front):
                return False
            if released_by_spatial_corridor and bool(getattr(config, 'JUNCTION_REGION_DEBUG', True)):
                print(f'>>> JUNCTION_REGION REARM corridor_clear={clear_distance:.3f}m')
            self._release_latch()
        if front_blocked:
            self.front_candidate_count += 1
        else:
            self.front_candidate_count = 0
        if not getattr(config, 'ENABLE_OPENING_ZONE_DETECTION', True):
            left_open = left_cm is not None and left_cm >= config.SIDE_OPEN_ENTER_CM
            right_open = right_cm is not None and right_cm >= config.SIDE_OPEN_ENTER_CM
            if self.front_candidate_count >= config.JUNCTION_CONFIRM_SAMPLES or left_open or right_open:
                self.pending_zone_event = None
                self._latch_now(now, pose_x, pose_y)
                return True
            return False
        completed = []
        left_event = self._track_side_zone(self.left_zone, left_cm, pose_x, pose_y, 'LEFT')
        if left_event is not None:
            completed.append(left_event)
        right_event = self._track_side_zone(self.right_zone, right_cm, pose_x, pose_y, 'RIGHT')
        if right_event is not None:
            completed.append(right_event)
        if getattr(config, 'ENABLE_INTERSECTION_WINDOW', True):
            self._start_intersection_window(pose_x, pose_y)
            if self.intersection_window['active']:
                for item in completed:
                    self.intersection_window['completed_sides'].add(item['side'])
                self._accumulate_intersection(front_cm, left_cm, right_cm, pose_x, pose_y)
                if self.front_candidate_count >= config.JUNCTION_CONFIRM_SAMPLES:
                    event = self._finalize_intersection_window(pose_x, pose_y, 'FRONT_BLOCKED')
                    self.pending_zone_event = event
                    self._latch_now(now, pose_x, pose_y)
                    return True
                should_finish, reason = self._intersection_should_finalize(pose_x, pose_y)
                if should_finish:
                    event = self._finalize_intersection_window(pose_x, pose_y, reason)
                    self.pending_zone_event = event
                    self._latch_now(now, pose_x, pose_y)
                    return True
                return False
        if self.front_candidate_count >= config.JUNCTION_CONFIRM_SAMPLES:
            self.pending_zone_event = None
            self._latch_now(now, pose_x, pose_y)
            return True
        if completed:
            event = max(completed, key=lambda item: item['length_m'])
            event['all_sides'] = [item['side'] for item in completed]
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
        self.heading_index = 0
        self.start_node_id = None
        self.current_node_id = None
        self.root_decision_node_id = None
        self.root_entry_abs_dir = None
        self.start_inside_abs_dir = None
        self.start_outside_abs_dir = None
        self.pending_from_node = None
        self.pending_abs_dir = None
        self.route_history = []
        self.dfs_stack = []
        self.completed = False
        self.graph_events = []
        self.route_attempt_counts = {}

    def heading_name(self, index=None):
        if index is None:
            index = self.heading_index
        return HEADINGS[index % 4]

    def absolute_index(self, relative_direction):
        return (self.heading_index + RELATIVE_OFFSET[relative_direction]) % 4

    @staticmethod
    def opposite_index(abs_index):
        return (abs_index + 2) % 4

    def relative_for_absolute(self, abs_index):
        diff = (abs_index - self.heading_index) % 4
        return {0: 'FRONT', 1: 'RIGHT', 2: 'BACK', 3: 'LEFT'}[diff]

    def _create_node(self, x, y):
        node_id = f'J{self.next_node_index}'
        self.next_node_index += 1
        self.nodes[node_id] = MazeNode(node_id=node_id, x=float(x), y=float(y))
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
        """Update a known-node anchor conservatively.

        V11 used to drag the graph anchor toward every revisit measurement.
        When chassis odometry had accumulated 20-35 cm of error, the anchor
        itself moved, making the next loop closure even harder.  V11.1 keeps
        the first anchor mostly fixed and rejects very large corrections.
        """
        node = self.nodes[node_id]
        node.seen_count += 1
        x = float(x)
        y = float(y)
        error = math.hypot(x - node.x, y - node.y)
        reject = float(getattr(config, 'NODE_POSITION_UPDATE_REJECT_M', 0.32))
        if reject > 0.0 and error > reject:
            self._record_graph_event('NODE_POSITION_UPDATE_REJECTED', node=node_id, error_m=error)
            return
        alpha = float(getattr(config, 'NODE_POSITION_UPDATE_ALPHA', 0.05))
        node.x = (1.0 - alpha) * node.x + alpha * x
        node.y = (1.0 - alpha) * node.y + alpha * y

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
        radius = float(getattr(config, 'EXPECTED_TARGET_MATCH_RADIUS_M', config.NODE_MATCH_RADIUS_M))
        if distance <= radius:
            return state.target
        return None

    def _find_topology_compatible_node(self, x, y, observed_abs=None):
        """Find a known junction missed by pure odometry matching.

        Used mainly for island/cyclic mazes.  A larger radius is allowed only
        when the candidate node's known exits are compatible with the stopped
        scan and/or its back edge already points to the node we came from.
        This avoids blindly merging nearby parallel corridors across an island.
        """
        if not bool(getattr(config, 'ENABLE_TOPOLOGY_NODE_MATCH', True)):
            return None
        if self.pending_from_node is None or self.pending_abs_dir is None:
            return None
        observed = set((int(v) % 4 for v in observed_abs or []))
        incoming_abs = int(self.pending_abs_dir) % 4
        back_abs = self.opposite_index(incoming_abs)
        previous = self.pending_from_node
        radius = float(getattr(config, 'TOPOLOGY_NODE_MATCH_RADIUS_M', 0.38))
        min_shared = int(getattr(config, 'TOPOLOGY_NODE_MATCH_MIN_SHARED_EXITS', 1))
        strict_shared = int(getattr(config, 'TOPOLOGY_NODE_MATCH_STRICT_SHARED_EXITS', 2))
        best = None
        for node_id, node in self.nodes.items():
            if node_id == previous:
                continue
            d = math.hypot(float(x) - node.x, float(y) - node.y)
            if d > radius:
                continue
            known = {int(a) % 4 for a, st in node.exits.items() if not getattr(st, 'blocked', False)}
            shared = len((known & observed) - {back_abs})
            back_state = node.exits.get(back_abs)
            reverse_link = back_state is not None and back_state.target == previous
            back_known = back_state is not None and (not back_state.blocked)
            compatible = False
            score = 0.0
            if reverse_link:
                compatible = True
                score += 5.0
            elif back_known and shared >= min_shared:
                compatible = True
                score += 3.0 + shared
            elif shared >= strict_shared:
                compatible = True
                score += 2.0 + shared
            if not compatible:
                continue
            score += max(0.0, 1.0 - d / max(radius, 1e-06))
            item = (score, -d, node_id, d, shared, reverse_link)
            if best is None or item > best:
                best = item
        if best is None:
            return None
        _, _, node_id, d, shared, reverse_link = best
        self._record_graph_event('TOPOLOGY_NODE_MATCH', node=node_id, distance_m=d, shared_exits=shared, reverse_link=bool(reverse_link), previous_node=previous, incoming_heading=self.heading_name(incoming_abs))
        return node_id

    def _get_or_create_node(self, x, y, observed_abs=None):
        expected_id = self._expected_arrival_node(x, y)
        if expected_id is not None:
            self._touch_node_position(expected_id, x, y)
            return (expected_id, False)
        node_id = self._find_nearby_node(x, y)
        if node_id is None:
            node_id = self._find_topology_compatible_node(x, y, observed_abs)
        is_new = node_id is None
        if is_new:
            node_id = self._create_node(x, y)
        self._touch_node_position(node_id, x, y)
        return (node_id, is_new)

    def remember_absolute_opening(self, node_id, abs_index, source='EDGE_ECHO'):
        """Remember a branch on an existing junction without creating a node.

        This is used when the edge observer sees the tail of the junction just
        after a turn.  A genuinely new direction is kept as a frontier on the
        existing node; an already-known direction is simply refreshed.
        """
        if node_id is None or node_id not in self.nodes:
            return None
        abs_index = int(abs_index) % 4
        state = self._exit(node_id, abs_index)
        state.seen_open_count += 1
        state.miss_count = 0
        state.blocked = False
        self._record_graph_event('OPENING_MERGED_INTO_EXISTING_NODE', node=node_id, heading=self.heading_name(abs_index), source=source, visits=state.visits, target=state.target)
        return state

    def _exit(self, node_id, abs_index):
        node = self.nodes[node_id]
        abs_index %= 4
        if abs_index not in node.exits:
            node.exits[abs_index] = ExitState()
        return node.exits[abs_index]

    def _record_graph_event(self, kind, **payload):
        event = {'time': time.time(), 'kind': kind}
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
            self._record_graph_event('SOURCE_EDGE_CONFLICT_PROTECTED', from_node=from_node_id, heading=self.heading_name(abs_index), existing_target=source_exit.target, attempted_target=to_node_id)
            return False
        if target_exit.target not in (None, from_node_id):
            self._record_graph_event('TARGET_EDGE_CONFLICT_PROTECTED', to_node=to_node_id, heading=self.heading_name(opposite), existing_target=target_exit.target, attempted_target=from_node_id)
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
        if source_exit.target in (None, to_node_id):
            self._connect_direct(from_node_id, abs_index, to_node_id)
            return
        chain = self._follow_same_heading_chain(from_node_id, abs_index)
        if to_node_id in chain:
            cut = chain[:chain.index(to_node_id) + 1]
            self._increment_chain_after_first_edge(cut, abs_index)
            self._record_graph_event('SKIPPED_NODE_CHAIN_REUSED', from_node=from_node_id, to_node=to_node_id, heading=self.heading_name(abs_index), chain=cut)
            return
        tail = chain[-1]
        tail_exit = self._exit(tail, abs_index)
        if tail_exit.target is None and tail != from_node_id:
            if len(chain) > 1:
                for node_id in chain[1:]:
                    state = self._exit(node_id, abs_index)
                    if state.target is not None:
                        self._increment_departure(node_id, abs_index)
            if self._connect_direct(tail, abs_index, to_node_id):
                self._increment_departure(tail, abs_index)
                self._record_graph_event('SKIPPED_NODE_CHAIN_EXTENDED', from_node=from_node_id, via_tail=tail, to_node=to_node_id, heading=self.heading_name(abs_index), chain=chain + [to_node_id])
                return
        self._record_graph_event('DIRECT_LINK_REJECTED_TO_PROTECT_GRAPH', from_node=from_node_id, to_node=to_node_id, heading=self.heading_name(abs_index), existing_target=source_exit.target, chain=chain)

    def _split_known_edge_with_intermediate(self, from_node_id, abs_index, new_node_id):
        """Insert a newly detected junction into an already-known corridor.

        Example: an earlier pass stored A -- B directly because the side branch
        at X was missed. A later pass detects X before reaching B. The correct
        topology is A -- X -- B; rejecting X leaves the graph inconsistent and
        can make frontier routing oscillate forever.
        """
        if not bool(getattr(config, 'ENABLE_INTERMEDIATE_NODE_EDGE_SPLIT', True)):
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
        if old_target_back.target != from_node_id:
            return False
        new_back = self._exit(new_node_id, opposite)
        new_forward = self._exit(new_node_id, abs_index)
        if new_back.target not in (None, from_node_id):
            return False
        if new_forward.target not in (None, old_target_id):
            return False
        inherited_visits = max(source_exit.visits, old_target_back.visits)
        source_exit.target = new_node_id
        new_back.target = from_node_id
        new_forward.target = old_target_id
        old_target_back.target = new_node_id
        for state in (source_exit, new_back, new_forward, old_target_back):
            state.visits = max(state.visits, inherited_visits)
            state.blocked = False
            state.seen_open_count = max(state.seen_open_count, 1)
            state.miss_count = 0
        self._record_graph_event('INTERMEDIATE_NODE_EDGE_SPLIT', from_node=from_node_id, inserted_node=new_node_id, old_target=old_target_id, heading=self.heading_name(abs_index), inherited_visits=inherited_visits)
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
            raise RuntimeError('initialize_start() must be called first')
        abs_index = self.heading_index
        self.start_inside_abs_dir = abs_index % 4
        self.start_outside_abs_dir = self.opposite_index(abs_index)
        self._increment_departure(self.current_node_id, abs_index)
        self.pending_from_node = self.current_node_id
        self.pending_abs_dir = abs_index
        self.route_history.append({'time': time.time(), 'node': self.current_node_id, 'direction': 'FRONT', 'heading': self.heading_name(abs_index), 'kind': 'initial_departure'})

    def arrive_at_decision_point(self, x, y, observed_abs=None):
        node_id, is_new = self._get_or_create_node(x, y, observed_abs=observed_abs)
        previous_node = self.pending_from_node
        incoming_abs_dir = self.pending_abs_dir
        if previous_node is not None and incoming_abs_dir is not None and (previous_node != node_id):
            split_done = False
            if is_new:
                split_done = self._split_known_edge_with_intermediate(previous_node, incoming_abs_dir, node_id)
            if not split_done:
                self._link_nodes(previous_node, incoming_abs_dir, node_id)
        if self.root_decision_node_id is None and previous_node == self.start_node_id and (incoming_abs_dir is not None):
            self.root_decision_node_id = node_id
            self.root_entry_abs_dir = self.opposite_index(incoming_abs_dir)
        self.current_node_id = node_id
        if not self.dfs_stack:
            self.dfs_stack.append(node_id)
        elif self.dfs_stack[-1] != node_id:
            if len(self.dfs_stack) >= 2 and self.dfs_stack[-2] == node_id:
                self.dfs_stack.pop()
            else:
                if node_id in self.dfs_stack:
                    self._record_graph_event('LOOP_REVISIT_STACK_PRESERVED', node=node_id, previous_node=previous_node)
                self.dfs_stack.append(node_id)
                max_len = int(getattr(config, 'DFS_STACK_MAX_LEN', 128))
                if max_len > 0 and len(self.dfs_stack) > max_len:
                    self.dfs_stack = self.dfs_stack[-max_len:]
        self.pending_from_node = None
        self.pending_abs_dir = None
        return (node_id, is_new)

    def _physical_candidates(self, front_open, left_open, right_open, allow_back=True):
        relative_candidates = []
        if front_open:
            relative_candidates.append('FRONT')
        if left_open:
            relative_candidates.append('LEFT')
        if right_open:
            relative_candidates.append('RIGHT')
        if allow_back:
            relative_candidates.append('BACK')
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
        for _, abs_index in candidates:
            state = self._exit(self.current_node_id, abs_index)
            state.seen_open_count += 1
            state.miss_count = 0
            state.blocked = False
        retire_enabled = bool(getattr(config, 'ENABLE_STALE_FRONTIER_RETIRE', True))
        miss_limit = int(getattr(config, 'FRONTIER_STALE_MISS_LIMIT', 3))
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
                self._record_graph_event('STALE_FRONTIER_RETIRED', node=self.current_node_id, heading=self.heading_name(abs_index), misses=state.miss_count)

    @staticmethod
    def _is_frontier_state(state):
        return state.visits == 0 and state.target is None and (not state.blocked)

    @staticmethod
    def _is_unresolved_state(state):
        """Traversed/open edge whose destination was never linked.

        This is different from a normal frontier: visits is already >0, but
        target is still None. Treating it as fully explored can hide a real
        branch and make graph routing loop around other nodes.
        """
        if not bool(getattr(config, 'ENABLE_UNRESOLVED_EDGE_RECOVERY', True)):
            return False
        max_visits = max(1, int(getattr(config, 'UNRESOLVED_EDGE_MAX_VISITS', 3)))
        return 0 < state.visits <= max_visits and state.target is None and (not state.blocked)

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
            return 'NONE'
        return ' | '.join((f'{node_id}.{self.heading_name(abs_index)}' for node_id, abs_index in items))

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
        if not bool(getattr(config, 'ENABLE_WEIGHTED_PENDING_ROUTING', True)):
            return None
        start = self.current_node_id
        if start is None or start not in self.nodes:
            return None
        import heapq
        allowed_first_abs = set(allowed_first_abs)
        base = float(getattr(config, 'ROUTE_EDGE_BASE_COST', 1.0))
        visit_pen = float(getattr(config, 'ROUTE_EDGE_VISIT_PENALTY', 1.75))
        high_extra = float(getattr(config, 'ROUTE_EDGE_HIGH_VISIT_EXTRA', 2.0))
        unresolved_extra = float(getattr(config, 'ROUTE_PENDING_UNRESOLVED_EXTRA', 1.25))
        pq = [(0.0, 0, start, [start])]
        best = {start: 0.0}
        while pq:
            cost, hops, node_id, path = heapq.heappop(pq)
            if cost > best.get(node_id, float('inf')) + 1e-09:
                continue
            if node_id != start and self.pending_exits(node_id):
                pending = self.pending_exits(node_id)
                if pending and all((p not in self.frontier_exits(node_id) for p in pending)):
                    cost += unresolved_extra
                return (path, cost)
            for nxt, abs_index in self._graph_neighbors(node_id):
                if len(path) == 1 and abs_index not in allowed_first_abs:
                    continue
                state = self._exit(node_id, abs_index)
                visits = max(0, int(state.visits))
                edge_cost = base + visit_pen * visits
                if visits >= 3:
                    edge_cost += high_extra * (visits - 2)
                new_cost = cost + edge_cost
                if new_cost + 1e-09 >= best.get(nxt, float('inf')):
                    continue
                best[nxt] = new_cost
                heapq.heappush(pq, (new_cost, hops + 1, nxt, path + [nxt]))
        return None

    def _frontier_signature(self):
        return tuple(self.all_pending_exits())

    def _route_attempt_key(self, abs_index, frontier_signature=None):
        if frontier_signature is None:
            frontier_signature = self._frontier_signature()
        return (frontier_signature, self.current_node_id, abs_index % 4)

    def _repeated_route_abs(self, candidates, frontier_signature):
        if not bool(getattr(config, 'ENABLE_ROUTE_LOOP_BREAK', True)):
            return set()
        limit = max(1, int(getattr(config, 'ROUTE_REPEAT_LIMIT', 1)))
        blocked = set()
        for _, abs_index in candidates:
            key = self._route_attempt_key(abs_index, frontier_signature)
            if self.route_attempt_counts.get(key, 0) >= limit:
                blocked.add(abs_index % 4)
        return blocked

    def plan_direction(self, front_open, left_open, right_open):
        if self.current_node_id is None:
            raise RuntimeError('No current node. Call arrive_at_decision_point()')
        allow_back = self.current_node_id != self.start_node_id or bool(self.nodes[self.current_node_id].exits)
        candidates = self._physical_candidates(front_open, left_open, right_open, allow_back=allow_back)
        if bool(getattr(config, 'ENABLE_START_GATE_GUARD', True)) and self.current_node_id == self.start_node_id and (self.start_outside_abs_dir is not None):
            before = list(candidates)
            candidates = [item for item in candidates if item[1] % 4 != self.start_outside_abs_dir % 4]
            if len(candidates) != len(before):
                print(f'>>> START_GATE PLANNER BLOCK outside={self.heading_name(self.start_outside_abs_dir)}')
        self._update_frontier_observations(candidates)
        preference_rank = {name: index for index, name in enumerate(config.EXPLORATION_PREFERENCE)}
        scored = []
        for relative, abs_index in candidates:
            state = self._exit(self.current_node_id, abs_index)
            scored.append((state.visits, preference_rank.get(relative, 99), relative, abs_index, state))
        scored.sort(key=lambda item: (item[0], item[1]))
        local_unvisited = [item for item in scored if self._is_frontier_state(item[4])]
        if local_unvisited:
            visits, _, relative, abs_index, _ = local_unvisited[0]
            self.completed = False
            return ExplorationDecision(direction=relative, node_id=self.current_node_id, reason='UNVISITED_EXIT', visits_before=visits, absolute_heading=self.heading_name(abs_index))
        local_unresolved = [item for item in scored if self._is_unresolved_state(item[4])]
        if local_unresolved:
            visits, _, relative, abs_index, _ = local_unresolved[0]
            self.completed = False
            self._record_graph_event('UNRESOLVED_EDGE_LOCAL_RETRY', node=self.current_node_id, heading=self.heading_name(abs_index), visits=visits)
            return ExplorationDecision(direction=relative, node_id=self.current_node_id, reason='UNRESOLVED_EDGE_RETRY', visits_before=visits, absolute_heading=self.heading_name(abs_index))
        if bool(getattr(config, 'ENABLE_REMEMBERED_LOCAL_FRONTIER', True)):
            observed_abs = {abs_index for _, abs_index in candidates}
            min_seen = int(getattr(config, 'REMEMBERED_FRONTIER_MIN_SEEN', 1))
            remembered = []
            for abs_index in self.frontier_exits(self.current_node_id):
                if abs_index in observed_abs:
                    continue
                state = self._exit(self.current_node_id, abs_index)
                if state.seen_open_count < min_seen:
                    continue
                relative = self.relative_for_absolute(abs_index)
                if relative not in ('LEFT', 'RIGHT'):
                    continue
                remembered.append((preference_rank.get(relative, 99), relative, abs_index, state))
            if remembered:
                remembered.sort(key=lambda item: item[0])
                _, relative, abs_index, state = remembered[0]
                self.completed = False
                return ExplorationDecision(direction=relative, node_id=self.current_node_id, reason='REMEMBERED_LOCAL_FRONTIER', visits_before=state.visits, absolute_heading=self.heading_name(abs_index))
        global_frontiers = self.all_frontiers()
        global_pending = self.all_pending_exits()
        if not global_pending:
            self.completed = True
            return ExplorationDecision(direction='COMPLETE', node_id=self.current_node_id, reason='ALL_FRONTIERS_EXPLORED', visits_before=0, absolute_heading=self.heading_name())
        frontier_signature = tuple(global_pending)
        physically_allowed_abs = {abs_index for _, abs_index in candidates}
        repeated_abs = self._repeated_route_abs(candidates, frontier_signature)
        allowed_first_abs = set(physically_allowed_abs) - repeated_abs
        if repeated_abs and front_open:
            front_abs = self.absolute_index('FRONT')
            if front_abs in allowed_first_abs:
                front_state = self._exit(self.current_node_id, front_abs)
                if front_state.visits > 0 or front_state.target is not None:
                    self.completed = False
                    return ExplorationDecision(direction='FRONT', node_id=self.current_node_id, reason='LOOP_BREAK_CONTINUE_FRONT', visits_before=front_state.visits, absolute_heading=self.heading_name(front_abs))
        if not allowed_first_abs:
            allowed_first_abs = set(physically_allowed_abs)
        weighted = self._least_cost_pending_path(allowed_first_abs)
        if weighted is not None:
            path, route_cost = weighted
            if path and len(path) >= 2:
                abs_index = self._abs_to_target(self.current_node_id, path[1], allowed_abs=allowed_first_abs)
                if abs_index is not None:
                    relative = self.relative_for_absolute(abs_index)
                    visits = self._exit(self.current_node_id, abs_index).visits
                    self.completed = False
                    self._record_graph_event('WEIGHTED_PENDING_ROUTE', node=self.current_node_id, next_node=path[1], target_node=path[-1], heading=self.heading_name(abs_index), visits=visits, route_cost=route_cost, path=path)
                    return ExplorationDecision(direction=relative, node_id=self.current_node_id, reason='ROUTE_TO_LOW_COST_PENDING', visits_before=visits, absolute_heading=self.heading_name(abs_index))
        stack_target = self._preferred_stack_frontier_target()
        if stack_target is not None:
            if len(self.dfs_stack) >= 2:
                parent = self.dfs_stack[-2]
                abs_to_parent = self._abs_to_target(self.current_node_id, parent, allowed_abs=allowed_first_abs)
                if abs_to_parent is not None:
                    relative = self.relative_for_absolute(abs_to_parent)
                    visits = self._exit(self.current_node_id, abs_to_parent).visits
                    self.completed = False
                    return ExplorationDecision(direction=relative, node_id=self.current_node_id, reason='DFS_BACKTRACK_TO_FRONTIER', visits_before=visits, absolute_heading=self.heading_name(abs_to_parent))
            path = self._shortest_path(self.current_node_id, stack_target, allowed_first_abs=allowed_first_abs)
            if path and len(path) >= 2:
                abs_index = self._abs_to_target(self.current_node_id, path[1], allowed_abs=allowed_first_abs)
                if abs_index is not None:
                    relative = self.relative_for_absolute(abs_index)
                    visits = self._exit(self.current_node_id, abs_index).visits
                    self.completed = False
                    return ExplorationDecision(direction=relative, node_id=self.current_node_id, reason='ROUTE_TO_DFS_FRONTIER', visits_before=visits, absolute_heading=self.heading_name(abs_index))
        path = self._nearest_reachable_frontier_path(allowed_first_abs)
        if path and len(path) >= 2:
            abs_index = self._abs_to_target(self.current_node_id, path[1], allowed_abs=allowed_first_abs)
            if abs_index is not None:
                relative = self.relative_for_absolute(abs_index)
                visits = self._exit(self.current_node_id, abs_index).visits
                self.completed = False
                return ExplorationDecision(direction=relative, node_id=self.current_node_id, reason='ROUTE_TO_NEAREST_FRONTIER', visits_before=visits, absolute_heading=self.heading_name(abs_index))
        known_transit = []
        for item in scored:
            visits, rank, relative, abs_index, state = item
            if state.target is not None and abs_index in allowed_first_abs:
                known_transit.append((visits, rank, relative, abs_index, state))
        if not known_transit:
            for item in scored:
                visits, rank, relative, abs_index, state = item
                if state.target is not None:
                    known_transit.append((visits, rank, relative, abs_index, state))
        if known_transit:
            known_transit.sort(key=lambda item: (0 if item[2] == 'BACK' else 1, item[0], item[1]))
            visits, _, relative, abs_index, _ = known_transit[0]
            self.completed = False
            return ExplorationDecision(direction=relative, node_id=self.current_node_id, reason='FRONTIER_RECOVERY_TRANSIT', visits_before=visits, absolute_heading=self.heading_name(abs_index))
        self.completed = False
        return ExplorationDecision(direction='COMPLETE', node_id=self.current_node_id, reason='FRONTIERS_EXIST_BUT_UNREACHABLE', visits_before=0, absolute_heading=self.heading_name())

    def shortest_path_to_node(self, start_id, target_id):
        """Public wrapper used by the V11 latent-frontier verifier."""
        return self._shortest_path(start_id, target_id)

    def weighted_path_to_node(self, start_id, target_id, allowed_first_abs=None):
        """Dijkstra path to a specific node, penalising repeatedly used edges.

        This is the V11 route used in cyclic/island mazes. Tree mazes collapse to
        the same unique path, while loops prefer a less-repeated transit route.
        """
        if start_id == target_id:
            return [start_id]
        if start_id not in self.nodes or target_id not in self.nodes:
            return None
        import heapq
        allowed_first_abs = None if allowed_first_abs is None else set(allowed_first_abs)
        base = float(getattr(config, 'ROUTE_EDGE_BASE_COST', 1.0))
        visit_pen = float(getattr(config, 'ROUTE_EDGE_VISIT_PENALTY', 1.75))
        high_extra = float(getattr(config, 'ROUTE_EDGE_HIGH_VISIT_EXTRA', 2.0))
        pq = [(0.0, 0, start_id, [start_id])]
        best = {start_id: 0.0}
        while pq:
            cost, hops, node_id, path = heapq.heappop(pq)
            if cost > best.get(node_id, float('inf')) + 1e-09:
                continue
            if node_id == target_id:
                return path
            for nxt, abs_index in self._graph_neighbors(node_id):
                if len(path) == 1 and allowed_first_abs is not None and (abs_index not in allowed_first_abs):
                    continue
                state = self._exit(node_id, abs_index)
                visits = max(0, int(state.visits))
                edge_cost = base + visit_pen * visits
                if visits >= 3:
                    edge_cost += high_extra * (visits - 2)
                new_cost = cost + edge_cost
                if new_cost + 1e-09 >= best.get(nxt, float('inf')):
                    continue
                best[nxt] = new_cost
                heapq.heappush(pq, (new_cost, hops + 1, nxt, path + [nxt]))
        return None

    def graph_path_cost(self, path):
        """Return the same repeated-edge weighted cost used by V11 Dijkstra."""
        if not path or len(path) < 2:
            return 0.0
        base = float(getattr(config, 'ROUTE_EDGE_BASE_COST', 1.0))
        visit_pen = float(getattr(config, 'ROUTE_EDGE_VISIT_PENALTY', 1.75))
        high_extra = float(getattr(config, 'ROUTE_EDGE_HIGH_VISIT_EXTRA', 2.0))
        total = 0.0
        for a, b in zip(path, path[1:]):
            abs_index = self._abs_to_target(a, b)
            if abs_index is None:
                return float('inf')
            state = self._exit(a, abs_index)
            visits = max(0, int(state.visits))
            total += base + visit_pen * visits
            if visits >= 3:
                total += high_extra * (visits - 2)
        return total

    def decision_for_absolute(self, abs_index, reason='V11_ABSOLUTE_ROUTE'):
        """Build an ExplorationDecision for one known absolute direction."""
        if self.current_node_id is None:
            return None
        abs_index = int(abs_index) % 4
        relative = self.relative_for_absolute(abs_index)
        state = self._exit(self.current_node_id, abs_index)
        return ExplorationDecision(direction=relative, node_id=self.current_node_id, reason=reason, visits_before=state.visits, absolute_heading=self.heading_name(abs_index))

    def route_decision_to_node(self, target_node_id, front_open, left_open, right_open, reason='ROUTE_TO_LATENT_FRONTIER'):
        """Return the first physically-valid step toward a known graph node.

        This is intentionally separate from normal frontier planning.  It is used
        only after hard DFS/frontier work has been exhausted and V11 wants to
        revisit a *soft* edge observation.  Known cycles are handled naturally by
        the graph shortest path; no historical DFS stack replay is required.
        """
        if self.current_node_id is None or target_node_id not in self.nodes:
            return None
        if self.current_node_id == target_node_id:
            return None
        physical = self._physical_candidates(front_open, left_open, right_open, allow_back=True)
        allowed_abs = {abs_index for _, abs_index in physical}
        path = self.weighted_path_to_node(self.current_node_id, target_node_id, allowed_first_abs=allowed_abs)
        if not path:
            path = self.weighted_path_to_node(self.current_node_id, target_node_id)
        if not path or len(path) < 2:
            return None
        abs_index = self._abs_to_target(self.current_node_id, path[1])
        if abs_index is None:
            return None
        decision = self.decision_for_absolute(abs_index, reason=reason)
        if decision is not None:
            self._record_graph_event('V11_ROUTE_TO_NODE', node=self.current_node_id, target_node=target_node_id, next_node=path[1], heading=self.heading_name(abs_index), path=path, reason=reason)
        return decision

    def has_hard_pending_work(self):
        """True while confirmed frontier/unresolved-edge work remains anywhere."""
        return bool(self.all_pending_exits())

    def commit_decision(self, decision):
        if decision.direction == 'COMPLETE':
            self.completed = decision.reason == 'ALL_FRONTIERS_EXPLORED'
            return
        abs_index = self.absolute_index(decision.direction)
        route_reasons = {'DFS_BACKTRACK_TO_FRONTIER', 'ROUTE_TO_DFS_FRONTIER', 'ROUTE_TO_NEAREST_FRONTIER', 'FRONTIER_RECOVERY_TRANSIT'}
        if decision.reason in route_reasons or str(decision.reason).startswith('ROUTE_TO_LATENT_'):
            signature = self._frontier_signature()
            key = self._route_attempt_key(abs_index, signature)
            self.route_attempt_counts[key] = self.route_attempt_counts.get(key, 0) + 1
        new_visits = self._increment_departure(self.current_node_id, abs_index)
        self.pending_from_node = self.current_node_id
        self.pending_abs_dir = abs_index
        self.route_history.append({'time': time.time(), 'node': self.current_node_id, 'direction': decision.direction, 'absolute_heading': self.heading_name(abs_index), 'edge_visits': new_visits, 'reason': decision.reason, 'frontiers_remaining': len(self.all_frontiers())})
        self.heading_index = abs_index
        self.completed = False

    def describe_node(self, node_id=None):
        if node_id is None:
            node_id = self.current_node_id
        if node_id is None or node_id not in self.nodes:
            return 'NO_NODE'
        node = self.nodes[node_id]
        parts = []
        for abs_index in range(4):
            if abs_index not in node.exits:
                continue
            exit_state = node.exits[abs_index]
            target = exit_state.target or '?'
            suffix = ''
            if self._is_frontier_state(exit_state):
                suffix = '[FRONTIER]'
            elif self._is_unresolved_state(exit_state):
                suffix = '[UNRESOLVED]'
            elif exit_state.blocked:
                suffix = '[STALE]'
            parts.append(f'{HEADINGS[abs_index]}:{exit_state.visits}->{target}{suffix}')
        return ' | '.join(parts) if parts else 'NO_EXITS'

    def save_memory(self, filepath=None):
        if filepath is None:
            filepath = config.MAZE_MEMORY_FILE
        data = {'start_node_id': self.start_node_id, 'root_decision_node_id': self.root_decision_node_id, 'root_entry_heading': self.heading_name(self.root_entry_abs_dir) if self.root_entry_abs_dir is not None else None, 'current_node_id': self.current_node_id, 'heading': self.heading_name(), 'completed': self.completed, 'frontiers': [{'node': node_id, 'heading': self.heading_name(abs_index)} for node_id, abs_index in self.all_frontiers()], 'unresolved_edges': [{'node': node_id, 'heading': self.heading_name(abs_index)} for node_id, abs_index in self.all_pending_exits() if abs_index in self.unresolved_exits(node_id)], 'dfs_stack': list(self.dfs_stack), 'nodes': {}, 'route_history': self.route_history, 'graph_events': self.graph_events}
        for node_id, node in self.nodes.items():
            data['nodes'][node_id] = {'x': node.x, 'y': node.y, 'seen_count': node.seen_count, 'exits': {HEADINGS[int(abs_index)]: {'visits': exit_state.visits, 'target': exit_state.target, 'seen_open_count': exit_state.seen_open_count, 'miss_count': exit_state.miss_count, 'blocked': exit_state.blocked, 'frontier': self._is_frontier_state(exit_state), 'unresolved': self._is_unresolved_state(exit_state)} for abs_index, exit_state in node.exits.items()}}
        with open(filepath, 'w', encoding='utf-8') as file:
            json.dump(data, file, indent=2, ensure_ascii=False)
HEADINGS = ('N', 'E', 'S', 'W')

def _distance(x1, y1, x2, y2):
    if None in (x1, y1, x2, y2):
        return None
    return math.hypot(float(x2) - float(x1), float(y2) - float(y1))

def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


# ==================== ORIENTATION-FREE PRE-DRAWN GUIDE ====================
@dataclass
class _GuideHypothesis:
    rotation: int
    mirrored: bool
    map_node_id: str
    score: float = 0.0
    observations: int = 0
    robot_to_map: dict = field(default_factory=dict)
    map_to_robot: dict = field(default_factory=dict)


class TopologicalMazeGuide:
    """Soft map matcher for a hand-drawn, possibly rotated/mirrored maze.

    One hypothesis is maintained for every D4 orientation (R0/R90/R180/R270,
    with and without reflection). Grid lengths only break routing ties. A guide
    decision is emitted only when the surviving hypotheses agree and the chosen
    physical opening is confirmed by the normal sensor scan.
    """

    def __init__(self, guide_data, source_path='<memory>'):
        self.source_path = source_path
        self.data = dict(guide_data or {})
        self.nodes = self.data.get('nodes') or {}
        self.start_node_id = self.data.get('start_node_id')
        self.mission_order = [str(v).upper() for v in self.data.get('mission_order', [])]
        self.marker_nodes = {
            str(name).upper(): list(node_ids)
            for name, node_ids in (self.data.get('marker_nodes') or {}).items()
        }
        if not self.marker_nodes:
            for node_id, node in self.nodes.items():
                for marker in node.get('markers', []):
                    self.marker_nodes.setdefault(str(marker).upper(), []).append(node_id)
        self.stage_index = 0
        self.hypotheses = []
        self.initialized = False
        self.exit_hint_reached = False
        self.exit_route_committed = False
        self.exit_route_commit_ratio = 0.0
        self._announced_marker_keys = set()
        self._pending_mission_event = None
        self._relocalized_since_anchor = False
        self._relocalize_consistent_transitions = 0
        self._departure_since_relocalize = False
        self.last_status = 'NOT_INITIALIZED'
        self._validate()

    @staticmethod
    def _compile_raw_maze_topology(raw, allowed_cells=None, mission_order_override=None):
        """Compile a saved MazeData JSON into the same topology guide as maze_designer.

        This is intentionally duplicated in the field build so --guide can accept
        either Export Route output *or* a normal Save Maze file.  It also keeps old
        saved mazes usable when somebody accidentally passes them as known_route.json.
        """
        if not isinstance(raw, dict):
            raise ValueError('raw maze payload is not a JSON object')

        try:
            rows = int(raw.get('rows'))
            cols = int(raw.get('cols'))
        except (TypeError, ValueError):
            raise ValueError('raw maze is missing valid rows/cols')
        if rows <= 0 or cols <= 0:
            raise ValueError('raw maze rows/cols must be positive')

        origin = str(raw.get('coordinate_origin', 'TOP_LEFT')).upper()
        legacy_top_left = origin != 'BOTTOM_LEFT'

        def convert_cell(value):
            if value is None or not isinstance(value, (list, tuple)) or len(value) < 2:
                return None
            try:
                r, c = int(value[0]), int(value[1])
            except (TypeError, ValueError):
                return None
            if legacy_top_left:
                r = rows - 1 - r
            return (r, c)

        def in_bounds(cell):
            return cell is not None and 0 <= cell[0] < rows and 0 <= cell[1] < cols

        def canon(a, b):
            return (a, b) if a <= b else (b, a)

        walls = set()
        for item in raw.get('walls', []) or []:
            if not isinstance(item, (list, tuple)) or len(item) != 2:
                continue
            a, b = convert_cell(item[0]), convert_cell(item[1])
            if in_bounds(a) and in_bounds(b) and abs(a[0] - b[0]) + abs(a[1] - b[1]) == 1:
                walls.add(canon(a, b))

        start = convert_cell(raw.get('start'))
        pickup = convert_cell(raw.get('pickup'))
        if pickup is None:
            pickup = convert_cell(raw.get('object'))
        drop = convert_cell(raw.get('drop'))
        goal = convert_cell(raw.get('exit'))
        if goal is None:
            goal = convert_cell(raw.get('goal'))

        if not in_bounds(start):
            raise ValueError('raw maze has no valid START cell')

        vec = {
            'N': (+1, 0),
            'E': (0, +1),
            'S': (-1, 0),
            'W': (0, -1),
        }

        def base_neighbors(cell):
            r, c = cell
            for dr, dc in vec.values():
                nxt = (r + dr, c + dc)
                if in_bounds(nxt) and canon(cell, nxt) not in walls:
                    yield nxt

        route_sequence = []
        if isinstance(allowed_cells, (list, tuple)):
            for item in allowed_cells:
                cell = convert_cell(item)
                if in_bounds(cell):
                    route_sequence.append(cell)

        # A normal Save Maze file may not contain an exported root "path".
        # When mission markers exist, compile the same ordered shortest legs so
        # old files also avoid treating every blank outside cell as traversable.
        if not route_sequence:
            mission_cells = [start]
            mission_cells.extend(cell for cell in (pickup, drop, goal) if cell is not None)

            def shortest_leg(leg_start, target):
                queue = [leg_start]
                came_from = {}
                seen = {leg_start}
                while queue:
                    current = queue.pop(0)
                    if current == target:
                        path = [current]
                        while current in came_from:
                            current = came_from[current]
                            path.append(current)
                        path.reverse()
                        return path
                    for nxt in base_neighbors(current):
                        if nxt in seen:
                            continue
                        seen.add(nxt)
                        came_from[nxt] = current
                        queue.append(nxt)
                return None

            if len(mission_cells) > 1:
                route_sequence = [start]
                for leg_start, target in zip(mission_cells, mission_cells[1:]):
                    leg = shortest_leg(leg_start, target)
                    if not leg:
                        raise ValueError(f'no path from {leg_start} to {target}')
                    route_sequence.extend(leg[1:])

        scoped_cells = set(route_sequence) if route_sequence else None
        if scoped_cells is not None:
            scoped_cells.add(start)

        def neighbors(cell):
            for nxt in base_neighbors(cell):
                if scoped_cells is None or nxt in scoped_cells:
                    yield nxt

        def open_headings(cell):
            r, c = cell
            result = []
            for heading in HEADINGS:
                dr, dc = vec[heading]
                nxt = (r + dr, c + dc)
                if (
                    in_bounds(nxt)
                    and (scoped_cells is None or nxt in scoped_cells)
                    and canon(cell, nxt) not in walls
                ):
                    result.append(heading)
            return result

        def markers(cell):
            out = []
            if cell == start:
                out.append('START')
            if pickup is not None and cell == pickup:
                out.append('PICKUP')
            if drop is not None and cell == drop:
                out.append('DROP')
            if goal is not None and cell == goal:
                out.append('EXIT')
            return out

        def is_topology_node(cell):
            if markers(cell):
                return True
            headings = open_headings(cell)
            if len(headings) != 2:
                return True
            a = HEADINGS.index(headings[0])
            b = HEADINGS.index(headings[1])
            return (a - b) % 4 != 2

        reachable = {start}
        queue = [start]
        while queue:
            cell = queue.pop(0)
            for nxt in neighbors(cell):
                if nxt not in reachable:
                    reachable.add(nxt)
                    queue.append(nxt)

        node_cells = sorted(cell for cell in reachable if is_topology_node(cell))
        if start not in node_cells:
            node_cells.insert(0, start)
        ordered = [start] + [cell for cell in node_cells if cell != start]
        cell_to_id = {cell: f'M{index}' for index, cell in enumerate(ordered)}
        nodes = {}

        for cell in ordered:
            node_id = cell_to_id[cell]
            exits = {}
            r, c = cell
            for heading in open_headings(cell):
                dr, dc = vec[heading]
                previous = cell
                current = (r + dr, c + dc)
                steps = 1
                visited_walk = {cell}

                while current not in cell_to_id:
                    if current in visited_walk or not in_bounds(current):
                        current = None
                        break
                    visited_walk.add(current)
                    onward = [nxt for nxt in neighbors(current) if nxt != previous]
                    if len(onward) != 1:
                        current = None
                        break
                    previous, current = current, onward[0]
                    steps += 1

                if current is None:
                    continue
                exits[heading] = {
                    'target': cell_to_id[current],
                    'grid_steps_hint': steps,
                }

            nodes[node_id] = {
                'id': node_id,
                'cell': list(cell),
                'markers': markers(cell),
                'degree': len(exits),
                'exits': exits,
            }

        if isinstance(mission_order_override, (list, tuple)):
            mission_order = [str(value).upper() for value in mission_order_override]
        else:
            mission_order = []
            if pickup is not None:
                mission_order.append('PICKUP')
            if drop is not None:
                mission_order.append('DROP')
            if goal is not None:
                mission_order.append('EXIT')

        marker_nodes = {}
        for node_id, node in nodes.items():
            for marker in node.get('markers', []):
                marker_nodes.setdefault(marker, []).append(node_id)

        return {
            'format': 'robomaster_topology_guide',
            'graph_scope': 'MISSION_ROUTE_CELLS' if scoped_cells is not None else 'REACHABLE_DRAWING',
            'route_cells': [list(cell) for cell in route_sequence],
            'orientation_policy': 'TRY_4_ROTATIONS_AND_BOTH_MIRRORS',
            'distance_policy': 'TOPOLOGY_PRIMARY_GRID_STEPS_HINT_ONLY',
            'sensor_policy': 'LIVE_SENSORS_OVERRIDE_DRAWING',
            'start_heading_required': False,
            'start_node_id': cell_to_id[start],
            'mission_order': mission_order,
            'marker_nodes': marker_nodes,
            'nodes': nodes,
        }

    @classmethod
    def load(cls, path):
        with open(path, 'r', encoding='utf-8') as file:
            payload = json.load(file)

        if not isinstance(payload, dict):
            raise ValueError('guide file must contain a JSON object')

        embedded_guide = payload.get('topology_guide')
        guide_data = None
        load_mode = None

        raw_maze = payload.get('maze') if isinstance(payload.get('maze'), dict) else payload
        looks_like_maze = (
            isinstance(raw_maze, dict)
            and raw_maze.get('start') is not None
            and raw_maze.get('rows') is not None
            and raw_maze.get('cols') is not None
            and isinstance(raw_maze.get('walls', []), list)
        )

        # Prefer rebuilding exported route files from their ordered path.  This
        # automatically repairs older Designer exports whose embedded topology
        # accidentally included all blank cells outside the physical maze.
        exported_path = payload.get('path')
        if looks_like_maze and isinstance(exported_path, list) and exported_path:
            guide_data = cls._compile_raw_maze_topology(
                raw_maze,
                allowed_cells=exported_path,
                mission_order_override=payload.get('mission_order'),
            )
            load_mode = 'route-scoped topology rebuilt from exported path'

        if guide_data is None and isinstance(embedded_guide, dict):
            guide_data = embedded_guide
            load_mode = 'embedded topology_guide'

        # Direct topology-guide JSON is also accepted.
        if guide_data is None and isinstance(payload.get('nodes'), dict) and payload.get('start_node_id') is not None:
            guide_data = payload
            load_mode = 'direct topology guide'

        # Robust field behaviour: accept normal Save Maze output directly.
        if guide_data is None and looks_like_maze:
            guide_data = cls._compile_raw_maze_topology(raw_maze)
            load_mode = 'auto-compiled route-scoped saved maze'

        if not isinstance(guide_data, dict):
            raise ValueError(
                'no usable topology guide or raw maze found; provide an Export Route JSON '
                'or a Save Maze JSON containing START/rows/cols/walls'
            )

        instance = cls(guide_data, source_path=path)
        instance.load_mode = load_mode
        return instance

    def _validate(self):
        # Stable family name only.  Accept:
        #   robomaster_topology_guide
        #   robomaster_topology_guide_v2 / _v3 / any future suffix
        # and even a missing format field when the required structure is valid.
        fmt = str(self.data.get('format', '') or '').strip()
        if fmt and not fmt.startswith('robomaster_topology_guide'):
            raise ValueError(f"unsupported topology guide type: {fmt!r}")

        if not isinstance(self.nodes, dict) or not self.nodes:
            raise ValueError('topology guide has no nodes')
        if self.start_node_id not in self.nodes:
            raise ValueError('topology guide start_node_id is missing')
        for node_id, node in self.nodes.items():
            exits = node.get('exits') or {}
            for heading, edge in exits.items():
                if heading not in HEADINGS:
                    raise ValueError(f'node {node_id} has invalid heading {heading!r}')
                target = edge.get('target') if isinstance(edge, dict) else edge
                if target not in self.nodes:
                    raise ValueError(f'node {node_id}.{heading} targets missing node {target!r}')

    @staticmethod
    def _copy_hypothesis(hypothesis):
        return _GuideHypothesis(
            rotation=hypothesis.rotation,
            mirrored=hypothesis.mirrored,
            map_node_id=hypothesis.map_node_id,
            score=hypothesis.score,
            observations=hypothesis.observations,
            robot_to_map=dict(hypothesis.robot_to_map),
            map_to_robot=dict(hypothesis.map_to_robot),
        )

    @staticmethod
    def _map_to_robot_dir(hypothesis, map_abs_index):
        value = int(map_abs_index) % 4
        if hypothesis.mirrored:
            value = (-value) % 4
        return (int(hypothesis.rotation) + value) % 4

    @staticmethod
    def _robot_to_map_dir(hypothesis, robot_abs_index):
        value = (int(robot_abs_index) - int(hypothesis.rotation)) % 4
        if hypothesis.mirrored:
            value = (-value) % 4
        return value

    @staticmethod
    def _transform_name(hypothesis):
        prefix = 'MIRROR+' if hypothesis.mirrored else ''
        return f'{prefix}R{(int(hypothesis.rotation) % 4) * 90}'

    def _edge_target(self, node_id, map_abs_index):
        node = self.nodes.get(node_id) or {}
        edge = (node.get('exits') or {}).get(HEADINGS[int(map_abs_index) % 4])
        if edge is None:
            return None
        return edge.get('target') if isinstance(edge, dict) else edge

    def _edge_hint(self, node_id, map_abs_index):
        node = self.nodes.get(node_id) or {}
        edge = (node.get('exits') or {}).get(HEADINGS[int(map_abs_index) % 4])
        if not isinstance(edge, dict):
            return 1.0
        try:
            return max(1.0, float(edge.get('grid_steps_hint', 1.0)))
        except (TypeError, ValueError):
            return 1.0

    def _expected_robot_openings(self, hypothesis):
        node = self.nodes.get(hypothesis.map_node_id) or {}
        result = set()
        for heading in (node.get('exits') or {}):
            if heading in HEADINGS:
                result.add(self._map_to_robot_dir(hypothesis, HEADINGS.index(heading)))
        return result

    def _signature_delta(self, hypothesis, observed_abs):
        expected = self._expected_robot_openings(hypothesis)
        observed = {int(v) % 4 for v in observed_abs}
        shared = len(expected & observed)
        unexpected_physical = len(observed - expected)
        sensor_missed_or_drawing_extra = len(expected - observed)
        delta = 1.65 * shared - 2.15 * unexpected_physical - 0.75 * sensor_missed_or_drawing_extra
        if expected == observed:
            delta += 1.25
        delta -= 0.30 * abs(len(expected) - len(observed))
        return delta

    def _prune(self, hypotheses):
        if not hypotheses:
            return []
        hypotheses.sort(key=lambda h: h.score, reverse=True)
        best_score = hypotheses[0].score
        window = float(getattr(config, 'GUIDE_SCORE_WINDOW', 9.0))
        maximum = max(1, int(getattr(config, 'GUIDE_MAX_HYPOTHESES', 32)))
        result = []
        seen = set()
        for hypothesis in hypotheses:
            if hypothesis.score < best_score - window:
                continue
            key = (
                hypothesis.rotation % 4,
                bool(hypothesis.mirrored),
                hypothesis.map_node_id,
                tuple(sorted(hypothesis.robot_to_map.items())),
            )
            if key in seen:
                continue
            seen.add(key)
            result.append(hypothesis)
            if len(result) >= maximum:
                break
        return result

    def initialize(self, robot_start_node_id):
        self.hypotheses = []
        for mirrored in (False, True):
            for rotation in range(4):
                self.hypotheses.append(
                    _GuideHypothesis(
                        rotation=rotation,
                        mirrored=mirrored,
                        map_node_id=self.start_node_id,
                        robot_to_map={robot_start_node_id: self.start_node_id},
                        map_to_robot={self.start_node_id: robot_start_node_id},
                    )
                )
        self.initialized = True
        self._relocalized_since_anchor = False
        self._relocalize_consistent_transitions = 0
        self._departure_since_relocalize = False
        self.exit_route_committed = False
        self.exit_route_commit_ratio = 0.0
        self.last_status = '8 ORIENTATION HYPOTHESES'
        if bool(getattr(config, 'GUIDE_DEBUG', True)):
            print('>>> MAP GUIDE initialized: 4 rotations x mirror/no-mirror; START heading ignored')

    def commit_departure(self, robot_node_id, robot_abs_index, reason='ROBOT_DECISION'):
        """Advance each hypothesis along the edge the robot actually chose."""
        if not self.initialized or not self.hypotheses:
            return
        active_marker = self.active_target()
        exit_targets = set(self.marker_nodes.get('EXIT', []))
        weighted = self._weighted_hypotheses()
        total_weight = sum(weight for _, weight in weighted)
        exit_weight = 0.0
        if active_marker == 'EXIT' and exit_targets:
            for old, weight in weighted:
                map_abs = self._robot_to_map_dir(old, robot_abs_index)
                target = self._edge_target(old.map_node_id, map_abs)
                if target in exit_targets:
                    exit_weight += weight
        exit_ratio = exit_weight / total_weight if total_weight > 0.0 else 0.0
        exit_threshold = float(
            getattr(config, 'GUIDE_MARKER_CONSENSUS_RATIO', 0.72)
        )
        exit_departure_approved = bool(
            active_marker == 'EXIT'
            and exit_targets
            and (not self._relocalized_since_anchor)
            and exit_ratio >= exit_threshold
        )
        advanced = []
        for old in self.hypotheses:
            hypothesis = self._copy_hypothesis(old)
            mapped_start = hypothesis.robot_to_map.get(robot_node_id)
            if mapped_start is not None and mapped_start != hypothesis.map_node_id:
                continue
            map_abs = self._robot_to_map_dir(hypothesis, robot_abs_index)
            target = self._edge_target(hypothesis.map_node_id, map_abs)
            if target is None:
                continue
            hypothesis.map_node_id = target
            hypothesis.score += 0.10
            advanced.append(hypothesis)
        self.hypotheses = self._prune(advanced)
        if active_marker == 'EXIT':
            self.exit_route_committed = bool(exit_departure_approved and self.hypotheses)
            self.exit_route_commit_ratio = exit_ratio if self.exit_route_committed else 0.0
            if self.exit_route_committed and bool(getattr(config, 'GUIDE_DEBUG', True)):
                print(
                    f'>>> MAP GUIDE EXIT APPROACH COMMITTED consensus='
                    f'{self.exit_route_commit_ratio:.0%} reason={reason}'
                )
        if self.hypotheses and self._relocalized_since_anchor:
            self._departure_since_relocalize = True
        if not self.hypotheses:
            self.last_status = 'LOST_AFTER_DEPARTURE; WILL RELOCALIZE'
            if bool(getattr(config, 'GUIDE_DEBUG', True)):
                print(f'>>> MAP GUIDE lost after {reason}: drawing has no matching edge; DFS remains active')

    def _relocalize(self, robot_node_id, observed_abs):
        penalty = float(getattr(config, 'GUIDE_RELOCALIZE_PENALTY', 4.0))
        candidates = []
        for map_node_id in self.nodes:
            for mirrored in (False, True):
                for rotation in range(4):
                    hypothesis = _GuideHypothesis(
                        rotation=rotation,
                        mirrored=mirrored,
                        map_node_id=map_node_id,
                        score=-penalty,
                        observations=1,
                        robot_to_map={robot_node_id: map_node_id},
                        map_to_robot={map_node_id: robot_node_id},
                    )
                    hypothesis.score += self._signature_delta(hypothesis, observed_abs)
                    candidates.append(hypothesis)
        self.hypotheses = self._prune(candidates)
        self._relocalized_since_anchor = True
        self._relocalize_consistent_transitions = 0
        self._departure_since_relocalize = False
        self.exit_route_committed = False
        self.exit_route_commit_ratio = 0.0
        self.last_status = 'GLOBAL TOPOLOGY RELOCALIZATION'
        if bool(getattr(config, 'GUIDE_DEBUG', True)):
            print(f'>>> MAP GUIDE relocalized from local topology: {len(self.hypotheses)} hypotheses')

    def observe_node(self, robot_node_id, observed_abs):
        """Fuse a real junction/corner/dead-end signature into all hypotheses."""
        observed_abs = {int(v) % 4 for v in observed_abs}
        if not self.initialized:
            self.initialize(robot_node_id)
        if not self.hypotheses:
            self._relocalize(robot_node_id, observed_abs)
            return

        updated = []
        for old in self.hypotheses:
            mapped = old.robot_to_map.get(robot_node_id)
            reverse = old.map_to_robot.get(old.map_node_id)
            if mapped is not None and mapped != old.map_node_id:
                continue
            if reverse is not None and reverse != robot_node_id:
                continue
            hypothesis = self._copy_hypothesis(old)
            hypothesis.score += self._signature_delta(hypothesis, observed_abs)
            hypothesis.observations += 1
            hypothesis.robot_to_map[robot_node_id] = hypothesis.map_node_id
            hypothesis.map_to_robot[hypothesis.map_node_id] = robot_node_id
            updated.append(hypothesis)

        self.hypotheses = self._prune(updated)
        if (
            self.hypotheses
            and self._relocalized_since_anchor
            and self._departure_since_relocalize
        ):
            self._relocalize_consistent_transitions += 1
            self._departure_since_relocalize = False
            required = max(1, int(getattr(config, 'GUIDE_RELOCALIZE_MIN_TRANSITIONS', 1)))
            if self._relocalize_consistent_transitions >= required:
                self._relocalized_since_anchor = False
                if bool(getattr(config, 'GUIDE_DEBUG', True)):
                    print(
                        f'>>> MAP GUIDE continuity restored after '
                        f'{self._relocalize_consistent_transitions} consistent transition(s)'
                    )
        if not self.hypotheses:
            self._relocalize(robot_node_id, observed_abs)
        elif self.hypotheses[0].score < float(getattr(config, 'GUIDE_RELOCALIZE_BELOW_SCORE', -12.0)):
            old = list(self.hypotheses)
            self._relocalize(robot_node_id, observed_abs)
            self.hypotheses = self._prune(old + self.hypotheses)
        self.last_status = self.describe()
        self._advance_stage_if_confident()
        if bool(getattr(config, 'GUIDE_DEBUG', True)):
            print('Map Guide:', self.describe())

    def active_target(self):
        while self.stage_index < len(self.mission_order):
            marker = self.mission_order[self.stage_index]
            if self.marker_nodes.get(marker):
                return marker
            self.stage_index += 1
        return None

    def physical_exit_allowed(self):
        """Allow open-area EXIT proof only on a map-approved final approach."""
        marker = self.active_target()
        if marker != 'EXIT':
            return False
        if not self.marker_nodes.get('EXIT'):
            return True
        return bool(self.exit_hint_reached or self.exit_route_committed)

    def _weighted_hypotheses(self):
        if not self.hypotheses:
            return []
        best = max(h.score for h in self.hypotheses)
        return [
            (hypothesis, math.exp(max(-12.0, min(0.0, hypothesis.score - best))))
            for hypothesis in self.hypotheses
        ]

    def _advance_stage_if_confident(self):
        marker = self.active_target()
        if marker is None or not self.hypotheses:
            return
        if (
            bool(getattr(config, 'GUIDE_REQUIRE_CONTINUITY_AFTER_RELOCALIZE', True))
            and self._relocalized_since_anchor
        ):
            return
        targets = set(self.marker_nodes.get(marker, []))
        weighted = self._weighted_hypotheses()
        min_observations = max(1, int(getattr(config, 'GUIDE_MARKER_MIN_OBSERVATIONS', 1)))
        weighted = [
            (hypothesis, weight)
            for hypothesis, weight in weighted
            if hypothesis.observations >= min_observations
        ]
        if not weighted:
            return
        total = sum(weight for _, weight in weighted)
        at_target = sum(weight for hypothesis, weight in weighted if hypothesis.map_node_id in targets)
        ratio = at_target / total if total > 0.0 else 0.0
        threshold = float(getattr(config, 'GUIDE_MARKER_CONSENSUS_RATIO', 0.72))
        if ratio < threshold:
            return
        key = (self.stage_index, marker)
        if key not in self._announced_marker_keys:
            print(f'>>> MAP GUIDE MISSION HINT REACHED: {marker} consensus={ratio:.0%}')
            self._announced_marker_keys.add(key)
            self._pending_mission_event = {
                'marker': marker,
                'stage_index': self.stage_index,
                'consensus': ratio,
            }
        if marker == 'EXIT':
            self.exit_hint_reached = True
            self.exit_route_committed = True
            self.exit_route_commit_ratio = max(self.exit_route_commit_ratio, ratio)
            return
        if bool(getattr(config, 'GUIDE_AUTO_ADVANCE_PICKUP_DROP', True)):
            self.stage_index += 1
            print(f'>>> MAP GUIDE mission advances to: {self.active_target() or "COMPLETE"}')

    def consume_mission_event(self):
        event = self._pending_mission_event
        self._pending_mission_event = None
        return event

    def _route_to_any(self, start_node_id, target_node_ids):
        """Topology-first Dijkstra; drawn cell count is only a tiny tie-breaker."""
        import heapq
        targets = set(target_node_ids)
        if start_node_id in targets:
            return [start_node_id]
        weight_hint = max(0.0, float(getattr(config, 'GUIDE_ROUTE_GRID_HINT_WEIGHT', 0.03)))
        pq = [(0.0, start_node_id, [start_node_id])]
        best = {start_node_id: 0.0}
        while pq:
            cost, node_id, path = heapq.heappop(pq)
            if cost > best.get(node_id, float('inf')) + 1e-09:
                continue
            if node_id in targets:
                return path
            node = self.nodes.get(node_id) or {}
            for heading, edge in (node.get('exits') or {}).items():
                if heading not in HEADINGS:
                    continue
                target = edge.get('target') if isinstance(edge, dict) else edge
                if target not in self.nodes or target in path:
                    continue
                hint = self._edge_hint(node_id, HEADINGS.index(heading))
                new_cost = cost + 1.0 + weight_hint * math.log1p(hint)
                if new_cost + 1e-09 >= best.get(target, float('inf')):
                    continue
                best[target] = new_cost
                heapq.heappush(pq, (new_cost, target, path + [target]))
        return None

    def recommend(self, explorer, scan):
        """Return a sensor-valid guided ExplorationDecision, or None."""
        marker = self.active_target()
        if marker is None or not self.hypotheses:
            return None
        if (
            bool(getattr(config, 'GUIDE_REQUIRE_CONTINUITY_AFTER_RELOCALIZE', True))
            and self._relocalized_since_anchor
        ):
            if bool(getattr(config, 'GUIDE_DEBUG', True)):
                print('>>> MAP GUIDE waits for one consistent transition after relocalization; DFS decides')
            return None
        max_hypotheses = max(1, int(getattr(config, 'GUIDE_MAX_COMMAND_HYPOTHESES', 12)))
        if len(self.hypotheses) > max_hypotheses:
            if bool(getattr(config, 'GUIDE_DEBUG', True)):
                print(
                    f'>>> MAP GUIDE ambiguous ({len(self.hypotheses)} hypotheses > '
                    f'{max_hypotheses}); DFS decides'
                )
            return None
        target_nodes = self.marker_nodes.get(marker, [])
        physical_abs = {explorer.opposite_index(explorer.heading_index)}
        if scan.get('front_open'):
            physical_abs.add(explorer.heading_index % 4)
        if scan.get('left_open'):
            physical_abs.add((explorer.heading_index - 1) % 4)
        if scan.get('right_open'):
            physical_abs.add((explorer.heading_index + 1) % 4)
        forbidden_abs = {
            int(value) % 4 for value in (scan.get('forbidden_abs') or set())
        }
        physical_abs.difference_update(forbidden_abs)
        if explorer.current_node_id == explorer.start_node_id and explorer.start_outside_abs_dir is not None:
            physical_abs.discard(explorer.start_outside_abs_dir % 4)

        votes = {}
        total_route_weight = 0.0
        viable_weight = 0.0
        weighted = self._weighted_hypotheses()
        for hypothesis, weight in weighted:
            if hypothesis.observations < int(getattr(config, 'GUIDE_MIN_OBSERVED_NODES', 1)):
                continue
            route = self._route_to_any(hypothesis.map_node_id, target_nodes)
            if not route or len(route) < 2:
                continue
            total_route_weight += weight
            next_node = route[1]
            map_abs = None
            node = self.nodes.get(hypothesis.map_node_id) or {}
            for heading, edge in (node.get('exits') or {}).items():
                target = edge.get('target') if isinstance(edge, dict) else edge
                if target == next_node and heading in HEADINGS:
                    map_abs = HEADINGS.index(heading)
                    break
            if map_abs is None:
                continue
            robot_abs = self._map_to_robot_dir(hypothesis, map_abs)
            if robot_abs not in physical_abs:
                continue
            viable_weight += weight
            votes[robot_abs] = votes.get(robot_abs, 0.0) + weight

        if not votes or total_route_weight <= 0.0 or viable_weight <= 0.0:
            return None
        physical_support = viable_weight / total_route_weight
        if physical_support < float(getattr(config, 'GUIDE_MIN_PHYSICAL_SUPPORT_RATIO', 0.55)):
            return None
        best_abs, best_vote = max(votes.items(), key=lambda item: item[1])
        vote_ratio = best_vote / viable_weight
        if vote_ratio < float(getattr(config, 'GUIDE_MIN_VOTE_RATIO', 0.67)):
            return None
        decision = explorer.decision_for_absolute(
            best_abs,
            reason=f'MAP_GUIDE_{marker}_{int(round(vote_ratio * 100.0))}PCT',
        )
        if decision is not None and bool(getattr(config, 'GUIDE_DEBUG', True)):
            print(f'>>> MAP GUIDE recommends {decision.direction} toward {marker} vote={vote_ratio:.0%} physical={physical_support:.0%}')
        return decision

    def describe(self):
        marker = self.active_target() or 'COMPLETE'
        if not self.hypotheses:
            return f'LOST | target={marker} | DFS_ONLY'
        best = max(self.hypotheses, key=lambda h: h.score)
        relocalize_text = ' | RELOCALIZING' if self._relocalized_since_anchor else ''
        return (
            f'{len(self.hypotheses)} hyp | best={self._transform_name(best)}@{best.map_node_id} '
            f'score={best.score:+.1f} obs={best.observations} | target={marker}{relocalize_text}'
        )


@dataclass

# ==================== V11 EDGE-FSM / LATENT FRONTIER MEMORY ====================
class OpeningCandidate:
    candidate_id: str
    edge_from: str
    edge_to: Optional[str]
    edge_abs_dir: int
    branch_abs_dir: int
    side_when_first_seen: str
    x: float
    y: float
    progress_from_from_m: float
    edge_length_m: Optional[float]
    opening_width_m: float
    max_distance_cm: float
    sample_count: int
    seen_passes: int = 1
    miss_passes: int = 0
    confidence: float = 0.55
    status: str = 'SUSPECTED'
    promoted_node: Optional[str] = None
    first_seen_time: float = field(default_factory=time.time)
    last_seen_time: float = field(default_factory=time.time)
    pass_ids: List[int] = field(default_factory=list)

    def pending(self):
        return self.status in ('SUSPECTED', 'LATENT')

@dataclass
class _Zone:
    side: str
    candidate_count: int = 0
    active: bool = False
    start_progress_m: Optional[float] = None
    start_x: Optional[float] = None
    start_y: Optional[float] = None
    last_progress_m: Optional[float] = None
    last_x: Optional[float] = None
    last_y: Optional[float] = None
    max_cm: float = 0.0
    samples: int = 0
    exit_count: int = 0
    interrupt_sent: bool = False

    def reset(self):
        side = self.side
        self.__dict__.update(_Zone(side).__dict__)

@dataclass
class TraversalContext:
    from_node: str
    abs_dir: int
    start_x: float
    start_y: float
    expected_to: Optional[str]
    pass_id: int
    started_at: float = field(default_factory=time.monotonic)
    max_progress_m: float = 0.0

class EdgeTraversalMemory:
    """Observe side-range changes continuously while an edge is traversed."""

    def __init__(self):
        self.current: Optional[TraversalContext] = None
        self.left_zone = _Zone('LEFT')
        self.right_zone = _Zone('RIGHT')
        self.candidates: Dict[str, OpeningCandidate] = {}
        self.next_candidate_index = 0
        self.pass_counter = 0
        self.events: List[dict] = []
        self.verify_candidate_id: Optional[str] = None
        self.verify_seen_this_pass = False
        self.verify_started_from: Optional[str] = None
        self.verify_expected_to: Optional[str] = None

    def begin_from_explorer(self, explorer):
        """Start observing the edge that explorer just committed to traverse."""
        from_node = explorer.pending_from_node
        abs_dir = explorer.pending_abs_dir
        if from_node is None or abs_dir is None or from_node not in explorer.nodes:
            self.current = None
            return None
        node = explorer.nodes[from_node]
        state = node.exits.get(abs_dir % 4)
        expected_to = state.target if state is not None else None
        self.pass_counter += 1
        self.current = TraversalContext(from_node=from_node, abs_dir=int(abs_dir) % 4, start_x=float(node.x), start_y=float(node.y), expected_to=expected_to, pass_id=self.pass_counter)
        self.left_zone.reset()
        self.right_zone.reset()
        self.verify_seen_this_pass = False
        self.verify_started_from = from_node if self.verify_candidate_id else None
        self._event('EDGE_BEGIN', from_node=from_node, heading=HEADINGS[int(abs_dir) % 4], expected_to=expected_to, pass_id=self.pass_counter)
        return self.current

    def progress_m(self, pose_x, pose_y):
        if self.current is None:
            return None
        return _distance(self.current.start_x, self.current.start_y, pose_x, pose_y)

    def finish_at_node(self, to_node: Optional[str], pose_x, pose_y, promoted_node=None):
        """Close the current edge when a decision node is reached."""
        if self.current is None:
            return
        progress = self.progress_m(pose_x, pose_y)
        if progress is None:
            progress = self.current.max_progress_m
        self._finish_zone_if_active(self.left_zone, pose_x, pose_y, progress, forced=True)
        self._finish_zone_if_active(self.right_zone, pose_x, pose_y, progress, forced=True)
        edge_length = max(float(progress or 0.0), self.current.max_progress_m)
        for candidate in self.candidates.values():
            if not candidate.pending():
                continue
            if candidate.edge_from != self.current.from_node:
                continue
            if candidate.edge_abs_dir % 4 != self.current.abs_dir % 4:
                continue
            if candidate.edge_to is None and to_node is not None:
                candidate.edge_to = to_node
                candidate.edge_length_m = edge_length
        self._event('EDGE_END', from_node=self.current.from_node, to_node=to_node, heading=HEADINGS[self.current.abs_dir], length_m=edge_length, pass_id=self.current.pass_id)
        if promoted_node is not None:
            self.promote_near_position(pose_x, pose_y, promoted_node)
        if self.verify_candidate_id is not None and self.verify_expected_to is not None and (to_node == self.verify_expected_to):
            self.finish_verification_pass(reached_other_endpoint=True)
        self.current = None
        self.left_zone.reset()
        self.right_zone.reset()

    def observe(self, pose_x, pose_y, left_cm, right_cm, front_cm=None):
        """Observe one loop sample.

        Returns an INTERSECTION_WINDOW-compatible event when the independent edge
        observer sees a strong side opening.  main.py may use this event to
        interrupt even while DecisionPointDetector is latched.
        """
        if self.current is None or pose_x is None or pose_y is None:
            return None
        progress = self.progress_m(pose_x, pose_y)
        if progress is None:
            return None
        self.current.max_progress_m = max(self.current.max_progress_m, progress)
        left_event = self._update_zone(self.left_zone, left_cm, pose_x, pose_y, progress)
        right_event = self._update_zone(self.right_zone, right_cm, pose_x, pose_y, progress)
        return left_event or right_event

    def event_branch_abs(self, event):
        if self.current is None or not event:
            return None
        side = str(event.get('side', '')).upper()
        if side == 'LEFT':
            return (self.current.abs_dir - 1) % 4
        if side == 'RIGHT':
            return (self.current.abs_dir + 1) % 4
        return None

    def merge_departure_echo(self, event, explorer, progress_m):
        """Collapse a post-turn opening back into the departure junction.

        On the real maze the front/side sensors can keep seeing the *same wide
        intersection* for 20-30 cm after a 90-degree turn. V11 interpreted that
        as another junction and created pairs such as J3/J18 only a few
        centimetres apart. V11.1 treats this region as the departure node's
        physical footprint. Any genuinely new branch is still remembered as an
        exit of that node, but no new node/edge is created.
        """
        if self.current is None or not event or progress_m is None:
            return False
        limit = float(getattr(config, 'EDGE_DEPARTURE_NODE_FOOTPRINT_M', 0.32))
        if float(progress_m) > limit:
            return False
        from_node = self.current.from_node
        branch_abs = self.event_branch_abs(event)
        if branch_abs is None or from_node not in getattr(explorer, 'nodes', {}):
            return False
        if hasattr(explorer, 'remember_absolute_opening'):
            explorer.remember_absolute_opening(from_node, branch_abs, source='EDGE_DEPARTURE_FOOTPRINT')
        matching = []
        for cand in self.candidates.values():
            if not cand.pending():
                continue
            if cand.edge_from != from_node:
                continue
            if cand.branch_abs_dir % 4 != branch_abs % 4:
                continue
            if int(self.current.pass_id) not in cand.pass_ids:
                continue
            matching.append(cand)
        if matching:
            cand = max(matching, key=lambda c: c.last_seen_time)
            cand.status = 'PROMOTED'
            cand.promoted_node = from_node
            cand.confidence = 1.0
            cand.last_seen_time = time.time()
            cid = cand.candidate_id
        else:
            cid = None
        self._event('EDGE_DEPARTURE_ECHO_MERGED', from_node=from_node, branch=HEADINGS[branch_abs], progress_m=float(progress_m), candidate_id=cid)
        return True

    def _update_zone(self, zone, distance_cm, x, y, progress):
        if distance_cm is None:
            return None
        enter_cm = float(getattr(config, 'EDGE_OBS_SIDE_ENTER_CM', config.SIDE_OPEN_ENTER_CM))
        exit_cm = float(getattr(config, 'EDGE_OBS_SIDE_EXIT_CM', config.SIDE_OPEN_EXIT_CM))
        enter_samples = max(1, int(getattr(config, 'EDGE_OBS_ENTER_SAMPLES', 3)))
        exit_samples = max(1, int(getattr(config, 'EDGE_OBS_EXIT_SAMPLES', 2)))
        ignore_m = float(getattr(config, 'EDGE_OBS_IGNORE_FROM_NODE_M', 0.18))
        max_len = float(getattr(config, 'EDGE_OBS_MAX_OPENING_M', 0.75))
        physically_open = float(distance_cm) >= enter_cm
        still_open = float(distance_cm) >= exit_cm
        if not zone.active:
            if progress < ignore_m:
                zone.candidate_count = 0
                return None
            if physically_open:
                if zone.candidate_count == 0:
                    zone.start_progress_m = progress
                    zone.start_x = float(x)
                    zone.start_y = float(y)
                zone.candidate_count += 1
                zone.max_cm = max(zone.max_cm, float(distance_cm))
                if zone.candidate_count >= enter_samples:
                    zone.active = True
                    zone.samples = zone.candidate_count
                    zone.last_progress_m = progress
                    zone.last_x = float(x)
                    zone.last_y = float(y)
                    zone.exit_count = 0
                    self._event('EDGE_OPENING_START', side=zone.side, from_node=self.current.from_node, progress_m=progress, heading=HEADINGS[self.current.abs_dir])
            else:
                zone.candidate_count = 0
                zone.start_progress_m = None
                zone.start_x = None
                zone.start_y = None
                zone.max_cm = 0.0
            return None
        zone.samples += 1
        zone.max_cm = max(zone.max_cm, float(distance_cm))
        zone.last_progress_m = progress
        zone.last_x = float(x)
        zone.last_y = float(y)
        zone.exit_count = 0 if still_open else zone.exit_count + 1
        width = max(0.0, progress - float(zone.start_progress_m or progress))
        if not zone.interrupt_sent:
            min_w = float(getattr(config, 'EDGE_OBS_INTERRUPT_MIN_WIDTH_M', 0.09))
            min_s = max(1, int(getattr(config, 'EDGE_OBS_INTERRUPT_MIN_SAMPLES', 4)))
            if width >= min_w and zone.samples >= min_s:
                candidate = self._upsert_candidate(zone, x, y, progress)
                zone.interrupt_sent = True
                self._note_verification_seen(candidate)
                return self._intersection_event_from_zone(zone, progress)
        if zone.exit_count >= exit_samples or width >= max_len:
            self._finish_zone_if_active(zone, x, y, progress, forced=width >= max_len)
        return None

    def _finish_zone_if_active(self, zone, x, y, progress, forced=False):
        if not zone.active:
            return None
        width = max(0.0, progress - float(zone.start_progress_m or progress))
        candidate = None
        min_width = float(getattr(config, 'EDGE_CANDIDATE_MIN_WIDTH_M', 0.08))
        if width >= min_width and zone.samples >= int(getattr(config, 'EDGE_CANDIDATE_MIN_SAMPLES', 4)):
            candidate = self._upsert_candidate(zone, x, y, progress)
            self._note_verification_seen(candidate)
        self._event('EDGE_OPENING_END', side=zone.side, from_node=self.current.from_node, width_m=width, samples=zone.samples, candidate_id=candidate.candidate_id if candidate else None, forced=bool(forced))
        zone.reset()
        return candidate

    def _intersection_event_from_zone(self, zone, progress):
        start = float(zone.start_progress_m or progress)
        span = max(0.0, progress - start)
        backtrack = max(0.0, 0.5 * span)
        observed = {'FRONT': False, 'LEFT': False, 'RIGHT': False}
        counts = {'FRONT': 0, 'LEFT': 0, 'RIGHT': 0}
        observed[zone.side] = True
        counts[zone.side] = int(zone.samples)
        return {'type': 'INTERSECTION_WINDOW', 'source': 'EDGE_OBSERVER', 'side': zone.side, 'length_m': span, 'opening_span_m': span, 'backtrack_m': backtrack, 'observed_open': observed, 'open_samples': counts, 'max_cm': {'FRONT': 0.0, 'LEFT': zone.max_cm if zone.side == 'LEFT' else 0.0, 'RIGHT': zone.max_cm if zone.side == 'RIGHT' else 0.0}, 'finish_reason': 'EDGE_OBSERVER_STRONG_OPENING'}

    def _zone_branch_abs(self, side):
        if side == 'LEFT':
            return (self.current.abs_dir - 1) % 4
        return (self.current.abs_dir + 1) % 4

    def _candidate_centre(self, zone, x, y):
        sx = zone.start_x if zone.start_x is not None else x
        sy = zone.start_y if zone.start_y is not None else y
        ex = zone.last_x if zone.last_x is not None else x
        ey = zone.last_y if zone.last_y is not None else y
        return (0.5 * (float(sx) + float(ex)), 0.5 * (float(sy) + float(ey)))

    def _upsert_candidate(self, zone, x, y, progress):
        cx, cy = self._candidate_centre(zone, x, y)
        branch_abs = self._zone_branch_abs(zone.side)
        width = max(0.0, progress - float(zone.start_progress_m or progress))
        match_r = float(getattr(config, 'EDGE_CANDIDATE_MATCH_RADIUS_M', 0.22))
        best = None
        best_d = None
        for cand in self.candidates.values():
            if not cand.pending():
                continue
            if cand.branch_abs_dir % 4 != branch_abs % 4:
                continue
            d = math.hypot(cand.x - cx, cand.y - cy)
            if d <= match_r and (best_d is None or d < best_d):
                best = cand
                best_d = d
        base_conf = self._observation_confidence(width, zone.samples, zone.max_cm)
        pass_id = int(self.current.pass_id)
        if best is None:
            cid = f'C{self.next_candidate_index}'
            self.next_candidate_index += 1
            best = OpeningCandidate(candidate_id=cid, edge_from=self.current.from_node, edge_to=self.current.expected_to, edge_abs_dir=self.current.abs_dir, branch_abs_dir=branch_abs, side_when_first_seen=zone.side, x=cx, y=cy, progress_from_from_m=max(0.0, 0.5 * ((zone.start_progress_m or progress) + progress)), edge_length_m=None, opening_width_m=width, max_distance_cm=zone.max_cm, sample_count=zone.samples, confidence=base_conf, pass_ids=[pass_id])
            self.candidates[cid] = best
            self._event('LATENT_CANDIDATE_NEW', candidate_id=cid, from_node=best.edge_from, branch=HEADINGS[branch_abs], confidence=best.confidence, x=cx, y=cy)
        else:
            new_pass = pass_id not in best.pass_ids
            if new_pass:
                best.pass_ids.append(pass_id)
                best.seen_passes += 1
            alpha = float(getattr(config, 'EDGE_CANDIDATE_UPDATE_ALPHA', 0.35))
            best.x = (1.0 - alpha) * best.x + alpha * cx
            best.y = (1.0 - alpha) * best.y + alpha * cy
            best.opening_width_m = max(best.opening_width_m, width)
            best.max_distance_cm = max(best.max_distance_cm, zone.max_cm)
            best.sample_count = max(best.sample_count, zone.samples)
            best.confidence = max(best.confidence, base_conf)
            if new_pass:
                best.confidence = _clamp(best.confidence + float(getattr(config, 'EDGE_CANDIDATE_REPEAT_BONUS', 0.18)), 0.0, 0.99)
            best.last_seen_time = time.time()
            self._event('LATENT_CANDIDATE_MATCH', candidate_id=best.candidate_id, seen_passes=best.seen_passes, confidence=best.confidence)
        latent_conf = float(getattr(config, 'EDGE_CANDIDATE_LATENT_CONFIDENCE', 0.68))
        latent_passes = int(getattr(config, 'EDGE_CANDIDATE_LATENT_PASSES', 2))
        if best.status == 'SUSPECTED' and (best.confidence >= latent_conf or best.seen_passes >= latent_passes):
            best.status = 'LATENT'
            self._event('LATENT_CANDIDATE_PROMOTED_SOFT', candidate_id=best.candidate_id, confidence=best.confidence)
        best.last_seen_time = time.time()
        return best

    @staticmethod
    def _observation_confidence(width, samples, max_cm):
        width_ref = max(0.01, float(getattr(config, 'EDGE_CANDIDATE_CONF_WIDTH_M', 0.16)))
        sample_ref = max(1, int(getattr(config, 'EDGE_CANDIDATE_CONF_SAMPLES', 7)))
        range_ref = max(1.0, float(getattr(config, 'EDGE_CANDIDATE_CONF_RANGE_CM', 40.0)))
        w = min(1.0, width / width_ref)
        s = min(1.0, float(samples) / sample_ref)
        r = min(1.0, max(0.0, float(max_cm) - config.SIDE_OPEN_ENTER_CM) / range_ref)
        return _clamp(0.3 + 0.3 * w + 0.25 * s + 0.15 * r, 0.0, 0.95)

    def _note_verification_seen(self, candidate):
        if candidate is None or self.verify_candidate_id is None:
            return
        if candidate.candidate_id == self.verify_candidate_id:
            self.verify_seen_this_pass = True

    def reanchor_candidates_to_graph(self, explorer):
        """Attach pending candidates to the nearest currently-known graph edge.

        When V10/V11 later inserts an intermediate node into A--B, a soft
        observation recorded on the old A--B edge must move to A--X or X--B.
        World-space candidate positions make that repair possible.
        """
        max_d = float(getattr(config, 'EDGE_CANDIDATE_REANCHOR_MAX_M', 0.3))
        edges = []
        seen_pairs = set()
        for a_id, a in explorer.nodes.items():
            for abs_dir, state in a.exits.items():
                b_id = state.target
                if b_id is None or b_id not in explorer.nodes:
                    continue
                pair = tuple(sorted((a_id, b_id)))
                if pair in seen_pairs:
                    continue
                seen_pairs.add(pair)
                b = explorer.nodes[b_id]
                edges.append((a_id, b_id, int(abs_dir) % 4, float(a.x), float(a.y), float(b.x), float(b.y)))

        def point_segment(px, py, x1, y1, x2, y2):
            vx, vy = (x2 - x1, y2 - y1)
            denom = vx * vx + vy * vy
            if denom <= 1e-12:
                return (math.hypot(px - x1, py - y1), 0.0)
            t = ((px - x1) * vx + (py - y1) * vy) / denom
            t = _clamp(t, 0.0, 1.0)
            qx, qy = (x1 + t * vx, y1 + t * vy)
            return (math.hypot(px - qx, py - qy), t)
        for cand in self.candidates.values():
            if not cand.pending():
                continue
            best = None
            for e in edges:
                d, t = point_segment(cand.x, cand.y, e[3], e[4], e[5], e[6])
                if d <= max_d and (best is None or d < best[0]):
                    best = (d, t, e)
            if best is None:
                continue
            d, t, e = best
            a_id, b_id, abs_dir, x1, y1, x2, y2 = e
            changed = cand.edge_from != a_id or cand.edge_to != b_id
            cand.edge_from = a_id
            cand.edge_to = b_id
            cand.edge_abs_dir = abs_dir
            length = math.hypot(x2 - x1, y2 - y1)
            cand.edge_length_m = length
            cand.progress_from_from_m = t * length
            if changed:
                self._event('LATENT_REANCHORED', candidate_id=cand.candidate_id, edge_from=a_id, edge_to=b_id, distance_m=d)

    def pending_candidates(self):
        min_conf = float(getattr(config, 'EDGE_VERIFY_MIN_CONFIDENCE', 0.52))
        result = [c for c in self.candidates.values() if c.pending() and c.confidence >= min_conf]
        result.sort(key=lambda c: (-c.confidence, c.miss_passes, -c.seen_passes, c.candidate_id))
        return result

    def candidate(self, candidate_id):
        return self.candidates.get(candidate_id)

    def select_best_candidate(self):
        items = self.pending_candidates()
        return items[0] if items else None

    def plan_verification(self, explorer, candidate=None):
        """Pick a reachable pending candidate and the cheaper endpoint.

        Candidates are considered in confidence order.  An unreachable stale
        soft observation therefore cannot hide another reachable one.
        """
        candidates = [candidate] if candidate is not None else self.pending_candidates()
        current = explorer.current_node_id
        for candidate in candidates:
            if candidate is None:
                continue
            endpoints = []
            if candidate.edge_from in explorer.nodes:
                endpoints.append(candidate.edge_from)
            if candidate.edge_to in explorer.nodes and candidate.edge_to not in endpoints:
                endpoints.append(candidate.edge_to)
            if not endpoints:
                continue
            best = None
            for start in endpoints:
                if current == start:
                    path = [start]
                elif hasattr(explorer, 'weighted_path_to_node'):
                    path = explorer.weighted_path_to_node(current, start)
                else:
                    path = explorer.shortest_path_to_node(current, start)
                if not path:
                    continue
                route_cost = explorer.graph_path_cost(path) if hasattr(explorer, 'graph_path_cost') else float(len(path) - 1)
                other = None
                abs_dir = None
                if start == candidate.edge_from:
                    other = candidate.edge_to
                    abs_dir = candidate.edge_abs_dir % 4
                elif start == candidate.edge_to:
                    other = candidate.edge_from
                    abs_dir = (candidate.edge_abs_dir + 2) % 4
                score = route_cost
                if start != candidate.edge_from:
                    score += 0.05
                item = (score, start, other, abs_dir, path)
                if best is None or item[0] < best[0]:
                    best = item
            if best is not None:
                _, start, other, abs_dir, path = best
                return {'candidate_id': candidate.candidate_id, 'candidate': candidate, 'start_node': start, 'other_node': other, 'edge_abs_dir': abs_dir, 'path_to_start': path}
        return None

    def arm_verification(self, candidate_id, start_node=None, expected_to=None):
        if candidate_id not in self.candidates:
            return False
        self.verify_candidate_id = candidate_id
        self.verify_seen_this_pass = False
        self.verify_started_from = start_node
        self.verify_expected_to = expected_to
        self._event('VERIFY_ARM', candidate_id=candidate_id, start_node=start_node, expected_to=expected_to)
        return True

    def verification_speed_limit(self, pose_x, pose_y):
        """Return a forward-speed cap near the active latent candidate."""
        if self.verify_candidate_id is None or self.current is None:
            return None
        cand = self.candidates.get(self.verify_candidate_id)
        if cand is None or not cand.pending():
            return None
        if pose_x is None or pose_y is None:
            return None
        d = math.hypot(float(pose_x) - cand.x, float(pose_y) - cand.y)
        near = float(getattr(config, 'EDGE_VERIFY_SLOW_RADIUS_M', 0.28))
        very = float(getattr(config, 'EDGE_VERIFY_CREEP_RADIUS_M', 0.14))
        if d <= very:
            return float(getattr(config, 'EDGE_VERIFY_CREEP_SPEED', 0.045))
        if d <= near:
            return float(getattr(config, 'EDGE_VERIFY_SLOW_SPEED', 0.09))
        return None

    def finish_verification_pass(self, reached_other_endpoint=True):
        cid = self.verify_candidate_id
        if cid is None:
            return
        cand = self.candidates.get(cid)
        if cand is None:
            self.verify_candidate_id = None
            self.verify_expected_to = None
            return
        if cand.status == 'PROMOTED':
            self._event('VERIFY_COMPLETE_PROMOTED', candidate_id=cid)
            self.verify_candidate_id = None
            self.verify_expected_to = None
            return
        if self.verify_seen_this_pass:
            cand.confidence = _clamp(cand.confidence + float(getattr(config, 'EDGE_VERIFY_SEEN_BONUS', 0.08)), 0.0, 0.99)
            self._event('VERIFY_SEEN_NOT_PROMOTED', candidate_id=cid, confidence=cand.confidence)
        elif reached_other_endpoint:
            cand.miss_passes += 1
            cand.confidence = _clamp(cand.confidence - float(getattr(config, 'EDGE_VERIFY_MISS_PENALTY', 0.24)), 0.0, 0.99)
            miss_limit = int(getattr(config, 'EDGE_VERIFY_RETIRE_MISSES', 3))
            if cand.miss_passes >= miss_limit or cand.confidence < float(getattr(config, 'EDGE_VERIFY_RETIRE_CONFIDENCE', 0.25)):
                cand.status = 'RETIRED'
                self._event('VERIFY_RETIRED', candidate_id=cid, misses=cand.miss_passes)
            else:
                self._event('VERIFY_MISS', candidate_id=cid, misses=cand.miss_passes, confidence=cand.confidence)
        self.verify_candidate_id = None
        self.verify_seen_this_pass = False
        self.verify_expected_to = None

    def promote_near_position(self, x, y, node_id, allowed_branch_abs=None):
        if x is None or y is None:
            return []
        allowed = None if allowed_branch_abs is None else {int(v) % 4 for v in allowed_branch_abs}
        radius = float(getattr(config, 'EDGE_CANDIDATE_PROMOTE_RADIUS_M', 0.24))
        promoted = []
        for cand in self.candidates.values():
            if not cand.pending():
                continue
            if allowed is not None and cand.branch_abs_dir % 4 not in allowed:
                continue
            if math.hypot(cand.x - float(x), cand.y - float(y)) <= radius:
                cand.status = 'PROMOTED'
                cand.promoted_node = node_id
                cand.confidence = 1.0
                promoted.append(cand.candidate_id)
                self._event('LATENT_PROMOTED_TO_NODE', candidate_id=cand.candidate_id, node=node_id)
                if self.verify_candidate_id == cand.candidate_id:
                    self.verify_candidate_id = None
                    self.verify_seen_this_pass = False
                    self.verify_expected_to = None
        return promoted

    def describe(self):
        pending = self.pending_candidates()
        if not pending:
            return 'NONE'
        return ' | '.join((f"{c.candidate_id}:{HEADINGS[c.branch_abs_dir]} conf={c.confidence:.2f} seen={c.seen_passes} miss={c.miss_passes} {c.edge_from}->{c.edge_to or '?'}" for c in pending))

    def save(self, filepath=None):
        filepath = filepath or getattr(config, 'EDGE_MEMORY_FILE', 'edge_memory.json')
        data = {'version': 'V11_EDGE_MEMORY', 'verify_candidate_id': self.verify_candidate_id, 'verify_expected_to': self.verify_expected_to, 'candidates': [asdict(c) for c in self.candidates.values()], 'events': self.events[-int(getattr(config, 'EDGE_MEMORY_EVENT_SAVE_LIMIT', 500)):]}
        with open(filepath, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=2, ensure_ascii=False)

    def _event(self, kind, **payload):
        item = {'time': time.time(), 'kind': kind}
        item.update(payload)
        self.events.append(item)
        max_events = int(getattr(config, 'EDGE_MEMORY_EVENT_RAM_LIMIT', 2000))
        if max_events > 0 and len(self.events) > max_events:
            self.events = self.events[-max_events:]
        if bool(getattr(config, 'EDGE_MEMORY_PRINT_EVENTS', True)) and kind in {'LATENT_CANDIDATE_NEW', 'LATENT_CANDIDATE_MATCH', 'LATENT_CANDIDATE_PROMOTED_SOFT', 'LATENT_PROMOTED_TO_NODE', 'VERIFY_ARM', 'VERIFY_MISS', 'VERIFY_RETIRED'}:
            extra = ' '.join((f'{k}={v}' for k, v in payload.items()))
            print(f'>>> {kind} {extra}')

def fmt(value):
    if value is None:
        return '---'
    return f'{value:4.1f}'

def fmt_adc(value):
    if value is None:
        return ' ---'
    try:
        return f'{int(value):4d}'
    except (TypeError, ValueError):
        return ' ---'

def stop_chassis(chassis):
    if not config.ENABLE_MOTION or chassis is None:
        return
    chassis.drive_speed(x=0, y=0, z=0, timeout=0.1)

def print_startup_info():
    print()
    print('==========================================================')
    print(' MAZE SOLVER V12.5.2 - MAP EXIT LOCK + ENTRY BARRIER')
    print('==========================================================')
    print()
    print(f'Program version     : {config.PROGRAM_VERSION}')
    print(f'Forward speed       : {config.FORWARD_SPEED:.2f} m/s')
    print(f'Sharp stale hold    : {config.SHARP_STALE_HOLD_SEC:.2f} s')
    print(f'Front Slow          : {config.SLOW_FRONT_CM:.1f} cm')
    print(f'Front Stop          : {config.STOP_FRONT_CM:.1f} cm')
    print(f'Front IR Guard      : {config.FRONT_IR_BLOCK_MODE} (L ID={config.IR_FRONT_LEFT_ID}, R ID={config.IR_FRONT_RIGHT_ID}, active={config.FRONT_IR_ACTIVE_LEVEL})')
    print(f'IR Collision Avoid  : {config.ENABLE_FRONT_IR_COLLISION_AVOIDANCE} (forward x<={config.FRONT_IR_SINGLE_AVOID_FORWARD_SPEED:.3f}, strafe={config.FRONT_IR_SINGLE_AVOID_Y_SPEED:.3f})')
    print(f'IR Turn Recovery    : {config.ENABLE_TURN_IR_COLLISION_GUARD} (arm>{config.TURN_IR_ARM_AFTER_DEG:.1f}deg, single={config.TURN_IR_SINGLE_ESCAPE_M:.3f}m, both={config.TURN_IR_BOTH_ESCAPE_M:.3f}m)')
    print(f'Side Opening        : ENTER >= {config.SIDE_OPEN_ENTER_CM:.1f} cm / EXIT < {config.SIDE_OPEN_EXIT_CM:.1f} cm')
    print(f'Opening Zone        : min {config.OPENING_ZONE_MIN_LENGTH_M:.2f} m, centre-backtrack enabled={config.ENABLE_OPENING_ZONE_CENTERING}')
    print(f'Intersection Window : lookahead {config.INTERSECTION_WINDOW_LOOKAHEAD_M:.2f} m / max {config.INTERSECTION_WINDOW_MAX_M:.2f} m / evidence {config.INTERSECTION_MIN_OPEN_SAMPLES} samples')
    print(f'Front Traversable   : >= {config.EXPLORATION_FRONT_OPEN_CM:.1f} cm')
    print()
    print(f'Side Target         : {config.TARGET_LEFT_CM:.1f} cm')
    print(f'Wall Hysteresis     : enter<{config.SIDE_WALL_ENTER_CM:.1f} leave>{config.SIDE_WALL_EXIT_CM:.1f} cm')
    print(f'Side Danger         : {config.SIDE_TOO_CLOSE_CM:.1f} cm')
    print()
    print(f'Node Match Radius   : {config.NODE_MATCH_RADIUS_M:.2f} m generic')
    print(f'Expected Target     : {config.EXPECTED_TARGET_MATCH_RADIUS_M:.2f} m')
    print(f'Topology Match      : {config.ENABLE_TOPOLOGY_NODE_MATCH} ({config.TOPOLOGY_NODE_MATCH_RADIUS_M:.2f} m)')
    print(f'Departure Footprint : {config.EDGE_DEPARTURE_NODE_FOOTPRINT_M:.2f} m (edge echo -> same node)')
    print(f'Rearm Distance      : {config.JUNCTION_REARM_DISTANCE_M:.2f} m')
    print(f'Edge Max Visits     : {config.MAX_EDGE_VISITS}')
    print(f'DFS Preference      : {config.EXPLORATION_PREFERENCE}')
    print(f'Edge Split          : {config.ENABLE_INTERMEDIATE_NODE_EDGE_SPLIT}')
    print(f'Route Loop Break    : {config.ENABLE_ROUTE_LOOP_BREAK} (repeat={config.ROUTE_REPEAT_LIMIT})')
    print(f'Unresolved Recovery : {config.ENABLE_UNRESOLVED_EDGE_RECOVERY} (max visits={config.UNRESOLVED_EDGE_MAX_VISITS})')
    print(f'Junction Creep      : {config.ENABLE_JUNCTION_CREEP} ({config.JUNCTION_CREEP_DISTANCE_M:.2f} m)')
    print(f'Corner Turn Setup   : {config.ENABLE_CORNER_TURN_SETUP} (ToF->{config.CORNER_TURN_FRONT_TARGET_CM:.1f} cm, {config.CORNER_TURN_SETUP_DISTANCE_M:.2f} m max)')
    print(f'Post-turn Clearance : {config.ENABLE_POST_TURN_CLEARANCE} (release>{config.POST_TURN_CLEARANCE_RELEASE_CM:.1f} cm)')
    print(f'Yaw Correction      : {config.ENABLE_YAW_CORRECTION}')
    print(f'Feedback Turn       : {config.ENABLE_FEEDBACK_TURN} (tol=±{config.TURN_FEEDBACK_TOLERANCE_DEG:.1f}°, stable={config.TURN_FEEDBACK_STABLE_SAMPLES}, attempts={config.TURN_MAX_ATTEMPTS})')
    print(f'Turn Watchdog       : 90={config.TURN_FEEDBACK_TIMEOUT_90_SEC:.1f}s / 180={config.TURN_FEEDBACK_TIMEOUT_180_SEC:.1f}s / timeout-accept=±{config.TURN_TIMEOUT_ACCEPT_TOLERANCE_DEG:.1f}°')
    print(f'Heading Hold        : {config.ENABLE_HEADING_HOLD}')
    print(f'Corridor Yaw Learn  : {config.ENABLE_CORRIDOR_HEADING_CALIBRATION} (legacy OFF)')
    print(f'Wall Parallel Assist: {getattr(config, "ENABLE_WALL_PARALLEL_ASSIST", False)} (travel>={getattr(config, "WALL_PARALLEL_MIN_TRAVEL_M", 0.12):.2f}m)')
    print(f'Adaptive Junction   : {getattr(config, "ENABLE_ADAPTIVE_JUNCTION_REGION", False)} (clear corridor={getattr(config, "JUNCTION_REGION_CLEAR_M", 0.12):.2f}m)')
    print(f'Owner Persistence   : {config.OWNER_PERSIST_THROUGH_SINGLE_WALL} (open_hold={config.OWNER_PERSIST_OPEN_SEC:.2f}s, force_switch={config.CENTER_OWNER_FORCE_SWITCH_CM:.1f}cm)')
    print(f'Move Z -> Yaw Sign  : {config.DEFAULT_MOVE_TO_YAW_SIGN:+d}')
    print(f'Drive Z -> Yaw Sign : {config.DEFAULT_DRIVE_TO_YAW_SIGN:+d}')
    print(f'Heading Recover     : >={config.HEADING_RECOVER_TRIGGER_DEG:.1f} deg')
    print()
    print('--- V11 edge memory / collision supervisor ---')
    print(f'Edge observer       : enter>={config.EDGE_OBS_SIDE_ENTER_CM:.1f}cm interrupt width>={config.EDGE_OBS_INTERRUPT_MIN_WIDTH_M:.2f}m')
    print(f'Latent verification : {config.ENABLE_LATENT_FRONTIER_VERIFICATION} min_conf={config.EDGE_VERIFY_MIN_CONFIDENCE:.2f} retire_misses={config.EDGE_VERIFY_RETIRE_MISSES}')
    print(f'Collision supervisor: {config.ENABLE_COLLISION_SUPERVISOR} rawIR={config.COLLISION_RAW_IR_EMERGENCY} turnRaw={config.TURN_IR_USE_RAW_EMERGENCY}')
    print(f'IR digital input    : get_io() port={config.SENSOR_PORT} CLEAR={config.FRONT_IR_CLEAR_LEVEL} HIT={config.FRONT_IR_ACTIVE_LEVEL}')
    print(f'IR ADC fusion       : {config.FRONT_IR_ADC_FUSION_ENABLED} (disabled)')
    print(f'IR active bumper    : {config.COLLISION_ACTIVE_FORWARD_ESCAPE} hold={config.COLLISION_ACTIVE_ESCAPE_HOLD_SEC:.2f}s strafe={config.COLLISION_ACTIVE_ESCAPE_Y_SPEED:.3f} backBias={config.COLLISION_ACTIVE_ESCAPE_BACK_BIAS_SPEED:.3f}')
    print()
    print('--- Open area / exit ---')
    print(f'Open Area           : {config.ENABLE_OPEN_AREA_HEADING_HOLD} (side>={config.OPEN_AREA_SIDE_ENTER_CM:.0f} cm, front>={config.OPEN_AREA_FRONT_MIN_CM:.0f} cm)')
    print(f'Exit Detection      : {config.ENABLE_EXIT_DETECTION} (front>={config.EXIT_FRONT_START_CM:.0f} cm, sides>={config.EXIT_SIDE_START_CM:.0f} cm)')
    print(f'Start Gate Guard    : {config.ENABLE_START_GATE_GUARD} (block outside J0 + geometric guard)')
    print(
        f'Entrance Barrier    : {getattr(config, "ENABLE_START_GATE_EARLY_DECISION_BARRIER", True)} '
        f'(force junction decision before progress<='
        f'{getattr(config, "START_GATE_DECISION_TRIGGER_PROGRESS_M", 0.70):.2f}m)'
    )
    print(
        f'Start Entry Acquire : {getattr(config, "ENABLE_START_ENTRY_ACQUISITION", True)} '
        f'(nudge={getattr(config, "START_ENTRY_NUDGE_DISTANCE_M", 0.05):.2f}m '
        f'@{getattr(config, "START_ENTRY_NUDGE_SPEED", 0.06):.2f}m/s, '
        f'then ToF/Sharp-L/Sharp-R/IR)'
    )
    print(f'Pre-drawn Guide     : {config.ENABLE_PREDRAWN_TOPOLOGY_GUIDE} (auto rotate + mirror, topology only)')
    print(f'Exit Confirmation   : {config.EXIT_CONFIRM_DISTANCE_M:.2f} m + {config.EXIT_CONFIRM_MIN_SEC:.1f} s + open-run {getattr(config, "EXIT_OPEN_AREA_MIN_TRAVEL_M", 0.75):.2f}m')
    print(f'Stop on Exit        : {config.STOP_WHEN_EXIT_FOUND}')
    print()
    print('Sharp controls Y; attitude yaw holds Z while driving corridors.')
    print('Trémaux chooses FRONT / LEFT / RIGHT / BACK at junctions.')
    print('Unvisited exits are always preferred over visited exits.')
    if config.SIDE_OPEN_ENTER_CM < 15.0:
        print('*** WARNING: SIDE OPEN threshold is suspiciously low (<15 cm). ***')
    print()
    if getattr(config, 'ENABLE_MAPPING', False):
        print('--- SLAM-style mapping ---')
        print(f'Map resolution       : {config.MAP_RESOLUTION_M * 100:.1f} cm/cell')
        print(f'ToF map range        : free<={config.MAP_TOF_FREE_MAX_CM:.0f} cm, wall<={config.MAP_TOF_OCCUPIED_MAX_CM:.0f} cm')
        print(f'Sharp wall range     : wall<={config.MAP_SHARP_OCCUPIED_MAX_CM:.0f} cm')
        print('Front IR mapping     : disabled (binary front IR is safety-only)')
        print(f'Map output           : {config.MAP_OUTPUT_DIR}/maze_map.png + .svg')
        print('Mapper is passive: it never changes robot movement or DFS decisions.')
        print()

def wait_for_pose(pose_tracker):
    deadline = time.time() + config.POSE_WAIT_SEC
    while time.time() < deadline:
        if pose_tracker.has_pose():
            x, y, _ = pose_tracker.get_position()
            return (x, y)
        time.sleep(0.05)
    print('WARNING: chassis position not ready; using start pose (0, 0).')
    return (0.0, 0.0)

def wait_for_yaw(pose_tracker):
    deadline = time.time() + config.POSE_WAIT_SEC
    while time.time() < deadline:
        yaw = pose_tracker.get_yaw()
        if yaw is not None:
            return yaw
        time.sleep(0.05)
    print('WARNING: attitude yaw not ready; heading hold will wait for data.')
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
        z_cmd, _ = controller.calculate_heading_hold(yaw, pose_tracker, recover=True)
        chassis.drive_speed(x=0.0, y=0.0, z=z_cmd, timeout=config.DRIVE_TIMEOUT_SEC)
        time.sleep(config.HEADING_ALIGN_LOOP_SEC)
    stop_chassis(chassis)
    yaw = pose_tracker.get_yaw()
    error = controller.heading_error(yaw)
    if yaw is not None and error is not None:
        print(f'>>> ABS HEADING ALIGN target={target:+.1f} yaw={yaw:+.1f} error={error:+.1f}')

def median_or_none(values):
    return statistics.median(values) if values else None

def scan_decision_point(detector, sensors, intersection_event=None):
    """Stopped re-scan merged with intersection memory and dual front IR.

    ToF remains the geometric front-range measurement.  A confirmed dual-front
    IR hit is converted to a *navigation-only* synthetic short front range so
    the planner sees BLOCKED while mapping still receives the real ToF value.
    """
    time.sleep(config.JUNCTION_SETTLE_SEC)
    left_samples = []
    right_samples = []
    front_samples = []
    ir_state = {'left_raw': None, 'right_raw': None, 'left_hit': False, 'right_hit': False, 'blocked': False}
    for index in range(config.DECISION_SCAN_SAMPLES):
        _, left_cm = sensors.read_left_sharp()
        _, right_cm = sensors.read_right_sharp()
        front_cm = sensors.get_front_cm()
        ir_l, ir_r = sensors.read_front_ir_pair()
        ir_state = sensors.update_front_ir_guard(ir_l, ir_r)
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
    nav_front_cm = sensors.effective_front_cm(front_cm, ir_state['blocked'])
    raw_front_open, front_blocked, raw_left_open, raw_right_open = detector.classify_openings(nav_front_cm, left_cm, right_cm)
    front_open = raw_front_open
    left_open = raw_left_open
    right_open = raw_right_open
    print(f"Decision Scan RAW -> ToF:{fmt(front_cm)} NavF:{fmt(nav_front_cm)} IRF:{ir_state.get('left_raw')}/{ir_state.get('right_raw')} IRB:{int(bool(ir_state.get('blocked')))} | Front:{('OPEN' if raw_front_open else 'BLOCK')} (need>={config.EXPLORATION_FRONT_OPEN_CM:.1f}) | L:{fmt(left_cm)} ({('OPEN' if raw_left_open else 'BLOCK')}, enter>={config.SIDE_OPEN_ENTER_CM:.1f}) | R:{fmt(right_cm)} ({('OPEN' if raw_right_open else 'BLOCK')}, enter>={config.SIDE_OPEN_ENTER_CM:.1f})")
    if intersection_event is not None and intersection_event.get('type') == 'INTERSECTION_WINDOW':
        observed = intersection_event.get('observed_open', {})
        counts = intersection_event.get('open_samples', {})
        side_memory_need = max(1, int(getattr(config, 'DECISION_MEMORY_SIDE_MIN_SAMPLES', 6)))
        left_memory = bool(observed.get('LEFT', False)) and int(counts.get('LEFT', 0)) >= side_memory_need
        right_memory = bool(observed.get('RIGHT', False)) and int(counts.get('RIGHT', 0)) >= side_memory_need
        front_open = (front_open or bool(observed.get('FRONT', False))) and (not front_blocked)
        # A weak historical side flash must not overrule the stopped sensor.
        # This is what previously merged the open START staging area with the
        # first junction (LEFT=4 samples versus RIGHT=23 samples).
        left_open = left_open or left_memory
        right_open = right_open or right_memory
        print(f">>> INTERSECTION MEMORY -> F={('OPEN' if observed.get('FRONT') else '---')}({counts.get('FRONT', 0)}) L={('OPEN' if observed.get('LEFT') else '---')}({counts.get('LEFT', 0)}) R={('OPEN' if observed.get('RIGHT') else '---')}({counts.get('RIGHT', 0)})")
        print(f">>> DECISION MERGED     -> F={('OPEN' if front_open else 'BLOCK')} L={('OPEN' if left_open else 'BLOCK')} R={('OPEN' if right_open else 'BLOCK')}")
    return {'front_cm': front_cm, 'nav_front_cm': nav_front_cm, 'left_cm': left_cm, 'right_cm': right_cm, 'front_open': front_open, 'front_blocked': front_blocked, 'left_open': left_open, 'right_open': right_open, 'raw_front_open': raw_front_open, 'raw_left_open': raw_left_open, 'raw_right_open': raw_right_open, 'ir_front_left': ir_state.get('left_raw'), 'ir_front_right': ir_state.get('right_raw'), 'ir_front_blocked': bool(ir_state.get('blocked'))}

def _pose_xy(pose_tracker):
    x, y, _ = pose_tracker.get_pose()
    return (x, y)

def _travelled_m(start_x, start_y, pose_tracker):
    x, y = _pose_xy(pose_tracker)
    if start_x is None or start_y is None or x is None or (y is None):
        return None
    return ((x - start_x) ** 2 + (y - start_y) ** 2) ** 0.5

def build_rolling_decision_scan(detector, sensors, intersection_event, front_cm, left_cm, right_cm, ir_state):
    """Build a planner-compatible scan from evidence collected while moving.

    Returns None when evidence is not strong enough. That deliberately sends the
    event to the original stopped-scan fallback instead of guessing.
    """
    if not bool(getattr(config, 'ENABLE_ROLLING_JUNCTION_DECISION', False)):
        return None
    if not intersection_event or intersection_event.get('type') != 'INTERSECTION_WINDOW':
        return None
    # EdgeTraversalMemory can interrupt from only one side. Keep that valuable
    # detector, but require the conservative stopped scan before turning it into
    # a graph node.
    if str(intersection_event.get('source', '')).upper() == 'EDGE_OBSERVER':
        return None
    if front_cm is None or left_cm is None or right_cm is None:
        return None

    nav_front_cm = sensors.effective_front_cm(front_cm, bool(ir_state.get('blocked', False)))
    if nav_front_cm is None:
        return None
    raw_front_open, front_hard_blocked, raw_left_open, raw_right_open = detector.classify_openings(
        nav_front_cm, left_cm, right_cm
    )
    observed = intersection_event.get('observed_open', {}) or {}
    counts = intersection_event.get('open_samples', {}) or {}
    maxima = intersection_event.get('max_cm', {}) or {}
    side_need = max(1, int(getattr(config, 'ROLLING_SIDE_MIN_SAMPLES', 3)))
    front_need = max(1, int(getattr(config, 'ROLLING_FRONT_MIN_SAMPLES', 3)))

    left_memory = bool(observed.get('LEFT', False)) and int(counts.get('LEFT', 0)) >= side_need
    right_memory = bool(observed.get('RIGHT', False)) and int(counts.get('RIGHT', 0)) >= side_need
    # Rolling mode is for side-junction windows. Pure front-wall/dead-end events
    # remain on the stopped path because they are cheap and safety critical.
    if not (left_memory or right_memory or raw_left_open or raw_right_open):
        return None

    front_memory = (
        bool(observed.get('FRONT', False))
        and int(counts.get('FRONT', 0)) >= front_need
        and float(maxima.get('FRONT', 0.0) or 0.0) >= float(getattr(config, 'ROLLING_FRONT_STRONG_MAX_CM', 45.0))
    )
    # A confirmed virtual bumper always wins over historical FRONT-open evidence.
    if bool(ir_state.get('blocked', False)) or front_hard_blocked:
        front_open = False
    elif raw_front_open:
        front_open = True
    elif front_memory:
        front_open = True
    else:
        # Current ToF below the exploration threshold is useful evidence that the
        # continuation is not safely traversable. We can still plan LEFT/RIGHT.
        front_open = False

    left_open = bool(raw_left_open or left_memory)
    right_open = bool(raw_right_open or right_memory)
    # Require every remembered OPEN side to have strong evidence. A weak side
    # observation means the geometry is ambiguous -> stopped scan.
    if bool(observed.get('LEFT', False)) and not left_memory and not raw_left_open:
        return None
    if bool(observed.get('RIGHT', False)) and not right_memory and not raw_right_open:
        return None

    result = {
        'front_cm': front_cm,
        'nav_front_cm': nav_front_cm,
        'left_cm': left_cm,
        'right_cm': right_cm,
        'front_open': front_open,
        'front_blocked': bool(front_hard_blocked or ir_state.get('blocked', False)),
        'left_open': left_open,
        'right_open': right_open,
        'raw_front_open': bool(raw_front_open),
        'raw_left_open': bool(raw_left_open),
        'raw_right_open': bool(raw_right_open),
        'ir_front_left': ir_state.get('left_raw'),
        'ir_front_right': ir_state.get('right_raw'),
        'ir_front_blocked': bool(ir_state.get('blocked', False)),
        'rolling': True,
        'rolling_finish_reason': intersection_event.get('finish_reason'),
    }
    if bool(getattr(config, 'ROLLING_DEBUG', True)):
        print(
            '>>> V12 ROLLING SCAN '
            f"F={'OPEN' if front_open else 'BLOCK'} "
            f"L={'OPEN' if left_open else 'BLOCK'} "
            f"R={'OPEN' if right_open else 'BLOCK'} "
            f"samples=F{counts.get('FRONT', 0)}/L{counts.get('LEFT', 0)}/R{counts.get('RIGHT', 0)} "
            f"reason={intersection_event.get('finish_reason')}"
        )
    return result


def estimate_rolling_junction_anchor(intersection_event, current_x, current_y):
    """Estimate the opening midpoint without physically reversing to it."""
    if not intersection_event:
        return current_x, current_y
    vals = (
        intersection_event.get('start_x'), intersection_event.get('start_y'),
        intersection_event.get('end_x'), intersection_event.get('end_y'),
    )
    if any(v is None for v in vals):
        return current_x, current_y
    sx, sy, ex, ey = map(float, vals)
    geometric = math.hypot(ex - sx, ey - sy)
    if geometric <= 1e-4:
        return current_x, current_y
    total = max(geometric, float(intersection_event.get('length_m', geometric) or geometric))
    backtrack = max(0.0, float(intersection_event.get('backtrack_m', 0.0) or 0.0))
    centre_from_start = max(0.0, min(total, total - backtrack))
    ratio = max(0.0, min(1.0, centre_from_start / total))
    return sx + (ex - sx) * ratio, sy + (ey - sy) * ratio


def rolling_turn_backtrack_limit(intersection_event):
    """Distance budget used only when the chosen turn mouth is behind the robot."""
    base = float(getattr(config, 'TURN_ENTRY_MAX_BACKTRACK_M', 0.13))
    if not intersection_event:
        return base
    remembered = max(0.0, float(intersection_event.get('backtrack_m', 0.0) or 0.0))
    remembered += float(getattr(config, 'ROLLING_TURN_BACKTRACK_MARGIN_M', 0.03))
    return min(
        max(base, remembered),
        float(getattr(config, 'ROLLING_TURN_MAX_BACKTRACK_M', 0.30)),
    )


def command_rolling_front(chassis, controller, sensors, pose_tracker, nav_front_cm, ir_state):
    """Refresh a short forward command so processing a node does not create a stop."""
    if not config.ENABLE_MOTION:
        return
    if bool(ir_state.get('blocked', False)):
        stop_chassis(chassis)
        return
    x = min(
        controller.calculate_forward_speed(nav_front_cm),
        float(getattr(config, 'ROLLING_FRONT_CONTINUE_SPEED', 0.15)),
    )
    x, y, z, _, _ = controller.apply_heading_hold(
        x, 0.0, pose_tracker.get_yaw(), pose_tracker, 'ROLLING_FRONT_CONTINUE'
    )
    x, y, z, _ = apply_motion_safety(x, y, z, 'ROLLING_FRONT_CONTINUE', front_ir_blocked=False)
    chassis.drive_speed(x=x, y=y, z=z, timeout=config.DRIVE_TIMEOUT_SEC)


def center_on_opening_zone(chassis, controller, pose_tracker, zone_event):
    """Reverse from the far edge of a measured side opening to its midpoint.

    The robot has just traversed this corridor segment, so the path directly
    behind it is known free. We nevertheless cap the backtrack distance and
    timeout. No rear range sensor is assumed.
    """
    if not zone_event:
        return
    if not getattr(config, 'ENABLE_OPENING_ZONE_CENTERING', True):
        return
    if not config.ENABLE_MOTION:
        print('OPENING_ZONE_CENTER skipped: motion disabled')
        return
    length = max(0.0, float(zone_event.get('length_m', 0.0)))
    requested_backtrack = zone_event.get('backtrack_m')
    if requested_backtrack is None:
        requested_backtrack = 0.5 * length
    target = min(max(0.0, float(requested_backtrack)) + float(getattr(config, 'OPENING_ZONE_CENTER_REVERSE_BIAS_M', 0.0)), float(config.OPENING_ZONE_CENTERING_MAX_BACKTRACK_M))
    if target <= 0.005:
        return
    start_x, start_y = _pose_xy(pose_tracker)
    start_time = time.monotonic()
    print(f">>> INTERSECTION_CENTER type={zone_event.get('type')} span={zone_event.get('opening_span_m', length):.3f}m window={length:.3f}m backtrack={target:.3f}m")
    while time.monotonic() - start_time < config.OPENING_ZONE_CENTERING_MAX_SEC:
        travelled = _travelled_m(start_x, start_y, pose_tracker)
        if travelled is not None and travelled >= target:
            print(f'OPENING_ZONE_CENTER done: travelled={travelled:.3f} m')
            break
        back_x, back_y, back_z, _, _ = controller.apply_heading_hold(-config.OPENING_ZONE_CENTERING_SPEED, 0.0, pose_tracker.get_yaw(), pose_tracker, 'OPENING_ZONE_CENTER')
        chassis.drive_speed(x=back_x, y=back_y, z=back_z, timeout=config.DRIVE_TIMEOUT_SEC)
        time.sleep(config.OPENING_ZONE_CENTERING_LOOP_SEC)
    stop_chassis(chassis)

def align_to_selected_side_opening(chassis, sensors, controller, pose_tracker, direction, raw_side_open, max_backtrack_m=None, search_speed=None, max_search_sec=None):
    """Ensure the pivot is physically inside the chosen side opening.

    Intersection Window intentionally remembers openings seen while moving.
    That is good for topology, but the final stopped position can be a few cm
    beyond a mouth.  Never rotate into a remembered LEFT/RIGHT branch unless
    Sharp at that side confirms usable clearance.  Search backward first,
    matching the field failure observed on the real maze.
    """
    if direction not in ('LEFT', 'RIGHT'):
        return True
    if not getattr(config, 'ENABLE_TURN_ENTRY_REALIGN', True):
        return True
    if not config.ENABLE_MOTION:
        return False

    # V12.2: rolling scan evidence can be several centimetres old by the time
    # the robot stops. Always confirm the chosen side at the CURRENT pivot.
    read_side = sensors.read_left_sharp if direction == 'LEFT' else sensors.read_right_sharp
    fresh_good = 0
    fresh_side = None
    for _ in range(max(1, int(config.TURN_ENTRY_CONFIRM_SAMPLES))):
        _, fresh_side = read_side()
        if fresh_side is not None and fresh_side >= config.TURN_ENTRY_OPEN_CM:
            fresh_good += 1
        else:
            fresh_good = 0
        time.sleep(config.TURN_ENTRY_LOOP_SEC)
    if fresh_good >= config.TURN_ENTRY_CONFIRM_SAMPLES:
        print(f'>>> TURN_ENTRY CURRENT OK {direction} side={fresh_side:.1f}cm')
        return True
    limit_m = float(config.TURN_ENTRY_MAX_BACKTRACK_M if max_backtrack_m is None else max_backtrack_m)
    speed = float(config.TURN_ENTRY_SEARCH_SPEED if search_speed is None else search_speed)
    max_sec = float(config.TURN_ENTRY_MAX_SEC if max_search_sec is None else max_search_sec)
    reachable_m = speed * max_sec
    print(
        f'>>> TURN_ENTRY_REALIGN {direction}: opening remembered but current Sharp is not open; '
        f'searching backward max={limit_m:.3f}m speed={speed:.3f}m/s timeout={max_sec:.1f}s '
        f'reachable~{reachable_m:.3f}m'
    )
    sx, sy = _pose_xy(pose_tracker)
    t0 = time.monotonic()
    good = 0
    while time.monotonic() - t0 < max_sec:
        travelled = _travelled_m(sx, sy, pose_tracker)
        if travelled is not None and travelled >= limit_m:
            break
        _, left_cm = sensors.read_left_sharp()
        _, right_cm = sensors.read_right_sharp()
        side_cm = left_cm if direction == 'LEFT' else right_cm
        if side_cm is not None and side_cm >= config.TURN_ENTRY_OPEN_CM:
            good += 1
            if good >= config.TURN_ENTRY_CONFIRM_SAMPLES:
                stop_chassis(chassis)
                print(f'>>> TURN_ENTRY_REALIGN OK side={side_cm:.1f}cm backtracked={float(travelled or 0.0):.3f}m')
                return True
        else:
            good = 0
        bx, by, bz, _, _ = controller.apply_heading_hold(-speed, 0.0, pose_tracker.get_yaw(), pose_tracker, 'TURN_ENTRY_REALIGN')
        chassis.drive_speed(x=bx, y=by, z=bz, timeout=config.DRIVE_TIMEOUT_SEC)
        time.sleep(config.TURN_ENTRY_LOOP_SEC)
    stop_chassis(chassis)
    _, left_cm = sensors.read_left_sharp()
    _, right_cm = sensors.read_right_sharp()
    side_cm = left_cm if direction == 'LEFT' else right_cm
    print(f">>> TURN_ENTRY_REALIGN FAILED {direction} side={(side_cm if side_cm is not None else 'None')}; turn cancelled safely")
    return False

def creep_to_junction_center(chassis, sensors, controller, pose_tracker, front_open, left_open, right_open):
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
        print('JUNCTION_CREEP skipped: motion disabled')
        return
    start_x, start_y = _pose_xy(pose_tracker)
    start_time = time.monotonic()
    print(f'>>> JUNCTION_CREEP speed={config.JUNCTION_CREEP_SPEED:.2f} m/s target={config.JUNCTION_CREEP_DISTANCE_M:.2f}m max={config.JUNCTION_CREEP_MAX_SEC:.2f}s')
    while time.monotonic() - start_time < config.JUNCTION_CREEP_MAX_SEC:
        ir_state = sensors.update_front_ir_guard(refresh=True)
        if ir_state['blocked']:
            print('JUNCTION_CREEP abort: dual front IR blocked')
            break
        front_cm = sensors.get_front_cm()
        if front_cm is None:
            print('JUNCTION_CREEP abort: ToF unavailable')
            break
        if front_cm <= config.JUNCTION_CREEP_ABORT_FRONT_CM:
            print(f'JUNCTION_CREEP abort: front={front_cm:.1f} cm <= {config.JUNCTION_CREEP_ABORT_FRONT_CM:.1f} cm')
            break
        travelled = _travelled_m(start_x, start_y, pose_tracker)
        if travelled is not None and travelled >= config.JUNCTION_CREEP_DISTANCE_M:
            print(f'JUNCTION_CREEP done: travelled={travelled:.3f} m')
            break
        creep_x, creep_y, creep_z, _, _ = controller.apply_heading_hold(config.JUNCTION_CREEP_SPEED, 0.0, pose_tracker.get_yaw(), pose_tracker, 'JUNCTION_CREEP')
        chassis.drive_speed(x=creep_x, y=creep_y, z=creep_z, timeout=config.DRIVE_TIMEOUT_SEC)
        time.sleep(config.JUNCTION_CREEP_LOOP_SEC)
    stop_chassis(chassis)

def shift_into_turn_pocket(chassis, sensors, controller, pose_tracker, relative_direction):
    """Move the mecanum chassis centre slightly into the chosen branch before yawing.

    A correct 90-degree yaw is not enough when the pivot is still on the corridor
    edge: the rectangular chassis sweeps the inside corner.  This pure-Y shift
    moves the centre of rotation into known open space while heading-hold keeps
    the original yaw.  LEFT = -Y, RIGHT = +Y in this project's convention.
    """
    if not bool(getattr(config, 'ENABLE_TURN_POCKET_POSITIONING', True)):
        return True
    if relative_direction not in ('LEFT', 'RIGHT'):
        return True
    if not config.ENABLE_MOTION:
        return True

    read_side = sensors.read_left_sharp if relative_direction == 'LEFT' else sensors.read_right_sharp
    _, side_cm = read_side()
    required = float(getattr(config, 'TURN_POCKET_REQUIRED_OPEN_CM', config.TURN_ENTRY_OPEN_CM))
    if side_cm is None or side_cm < required:
        print(f">>> TURN_POCKET CANCEL {relative_direction}: side not open enough ({side_cm})")
        return False

    distance = max(0.0, float(getattr(config, 'TURN_POCKET_STRAFE_M', 0.045)))
    speed = abs(float(getattr(config, 'TURN_POCKET_STRAFE_SPEED', 0.05)))
    max_sec = float(getattr(config, 'TURN_POCKET_MAX_SEC', 1.5))
    loop_sec = float(getattr(config, 'TURN_POCKET_LOOP_SEC', 0.04))
    hard_stop = float(getattr(config, 'TURN_POCKET_SIDE_HARD_STOP_CM', 6.0))
    y_sign = -1.0 if relative_direction == 'LEFT' else +1.0
    y_cmd = y_sign * speed * float(config.Y_DIR_SIGN)

    sx, sy = _pose_xy(pose_tracker)
    t0 = time.monotonic()
    print(f'>>> TURN_POCKET {relative_direction}: strafe={distance:.3f}m y={y_cmd:+.3f}m/s')
    while time.monotonic() - t0 < max_sec:
        travelled = _travelled_m(sx, sy, pose_tracker)
        if travelled is not None and travelled >= distance:
            print(f'>>> TURN_POCKET OK {relative_direction}: shifted={travelled:.3f}m')
            stop_chassis(chassis)
            return True

        _, side_cm = read_side()
        if side_cm is None:
            print(f'>>> TURN_POCKET stop {relative_direction}: Sharp unavailable')
            stop_chassis(chassis)
            return False
        if side_cm <= hard_stop:
            print(f'>>> TURN_POCKET stop {relative_direction}: side={side_cm:.1f}cm <= {hard_stop:.1f}cm')
            stop_chassis(chassis)
            return False

        # Heading hold only; no forward motion. SafeChassisProxy still supervises
        # the command, while the chosen Sharp guards the open branch itself.
        x_cmd, y_hold, z_cmd, _, _ = controller.apply_heading_hold(
            0.0, y_cmd, pose_tracker.get_yaw(), pose_tracker, 'TURN_POCKET'
        )
        chassis.drive_speed(x=x_cmd, y=y_hold, z=z_cmd, timeout=config.DRIVE_TIMEOUT_SEC)
        time.sleep(loop_sec)

    stop_chassis(chassis)
    travelled = _travelled_m(sx, sy, pose_tracker)
    if travelled is not None and travelled >= max(0.015, distance * 0.70):
        print(f'>>> TURN_POCKET partial accept {relative_direction}: shifted={travelled:.3f}m')
        return True
    print(f'>>> TURN_POCKET FAILED {relative_direction}: shifted={float(travelled or 0.0):.3f}m')
    return False

def corner_turn_setup(chassis, sensors, controller, pose_tracker, relative_direction, front_open):
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
    if relative_direction not in ('LEFT', 'RIGHT'):
        return
    if front_open:
        return
    if not config.ENABLE_MOTION:
        print('TURN_SETUP skipped: motion disabled')
        return
    start_x, start_y = _pose_xy(pose_tracker)
    start_time = time.monotonic()
    start_front = sensors.get_front_cm()
    print(f">>> TURN_SETUP {relative_direction} speed={config.CORNER_TURN_SETUP_SPEED:.2f} m/s target_move={config.CORNER_TURN_SETUP_DISTANCE_M:.2f}m front_target={config.CORNER_TURN_FRONT_TARGET_CM:.1f}cm start_front={(start_front if start_front is not None else 'None')}")
    while time.monotonic() - start_time < config.CORNER_TURN_SETUP_MAX_SEC:
        ir_state = sensors.update_front_ir_guard(refresh=True)
        if ir_state['blocked']:
            print('TURN_SETUP stop: dual front IR blocked')
            break
        front_cm = sensors.get_front_cm()
        if front_cm is None:
            print('TURN_SETUP abort: ToF unavailable')
            break
        if front_cm <= config.CORNER_TURN_FRONT_HARD_STOP_CM:
            print(f'TURN_SETUP HARD STOP: front={front_cm:.1f} cm <= {config.CORNER_TURN_FRONT_HARD_STOP_CM:.1f} cm')
            break
        if front_cm <= config.CORNER_TURN_FRONT_TARGET_CM:
            print(f'TURN_SETUP done: front target reached ({front_cm:.1f} cm)')
            break
        travelled = _travelled_m(start_x, start_y, pose_tracker)
        if travelled is not None and travelled >= config.CORNER_TURN_SETUP_DISTANCE_M:
            print(f'TURN_SETUP done: travelled={travelled:.3f} m')
            break
        x_cmd, y_cmd, z_cmd, _, _ = controller.apply_heading_hold(config.CORNER_TURN_SETUP_SPEED, 0.0, pose_tracker.get_yaw(), pose_tracker, 'TURN_SETUP')
        chassis.drive_speed(x=x_cmd, y=y_cmd, z=z_cmd, timeout=config.DRIVE_TIMEOUT_SEC)
        time.sleep(config.CORNER_TURN_SETUP_LOOP_SEC)
    stop_chassis(chassis)

def post_turn_corridor_acquire(chassis, sensors, controller, pose_tracker, relative_direction):
    """Crawl straight out of a junction before restoring normal wall-follow.

    Why this exists:
      Immediately after a 90-degree turn, the side Sharp sensors may still see
      the old corner / open junction.  If the normal controller is enabled at
      once it can command x=0.20 together with a large lateral y correction,
      which drives a front corner into the wall even though the yaw turn itself
      was correct.

    During this short launch phase:
      * forward speed is capped;
      * lateral wall-follow is disabled (y=0 unless emergency clearance needed);
      * heading hold remains active;
      * raw/confirmed front IR has immediate priority;
      * very close side Sharp causes a small escape away from that side.
    """
    if not bool(getattr(config, 'ENABLE_POST_TURN_CORRIDOR_ACQUIRE', True)):
        return
    if relative_direction not in ('LEFT', 'RIGHT'):
        return
    if not config.ENABLE_MOTION:
        return

    distance_target = max(0.0, float(getattr(config, 'POST_TURN_ACQUIRE_DISTANCE_M', 0.12)))
    speed = max(0.0, float(getattr(config, 'POST_TURN_ACQUIRE_SPEED', 0.06)))
    max_sec = max(0.2, float(getattr(config, 'POST_TURN_ACQUIRE_MAX_SEC', 3.0)))
    loop_sec = max(0.01, float(getattr(config, 'POST_TURN_ACQUIRE_LOOP_SEC', 0.04)))
    front_stop = float(getattr(config, 'POST_TURN_ACQUIRE_FRONT_STOP_CM', config.STOP_FRONT_CM))
    side_hard = float(getattr(config, 'POST_TURN_ACQUIRE_SIDE_HARD_STOP_CM', config.SIDE_TOO_CLOSE_CM))
    escape_y = abs(float(getattr(config, 'POST_TURN_ACQUIRE_ESCAPE_Y_SPEED', 0.05))) * config.Y_DIR_SIGN
    escape_back = abs(float(getattr(config, 'POST_TURN_ACQUIRE_ESCAPE_BACK_SPEED', 0.03)))
    clear_needed = max(1, int(getattr(config, 'POST_TURN_ACQUIRE_CLEAR_SAMPLES', 2)))

    sensors.reset_filters()
    controller.reset_side_owner()
    start_x, start_y = _pose_xy(pose_tracker)
    start_time = time.monotonic()
    clear_samples = 0

    print(
        f'>>> POST_TURN_ACQUIRE {relative_direction}: '
        f'straight={distance_target:.3f}m speed={speed:.3f}m/s; wall-follow temporarily OFF'
    )

    while time.monotonic() - start_time < max_sec:
        ir_state = sensors.update_front_ir_guard(refresh=True)
        left_hit = bool(ir_state.get('left_hit', False) or ir_state.get('left_confirmed', False))
        right_hit = bool(ir_state.get('right_hit', False) or ir_state.get('right_confirmed', False))

        _, left_cm = sensors.read_left_sharp()
        _, right_cm = sensors.read_right_sharp()
        front_cm = sensors.get_front_cm()

        # A front-corner hit after the turn means the chassis is not yet aligned
        # with the corridor.  Translate away before trying forward motion again.
        if left_hit or right_hit:
            clear_samples = 0
            if left_hit and right_hit:
                ex, ey = -escape_back, 0.0
                reason = 'BOTH_FRONT_IR'
            elif left_hit:
                ex, ey = -0.5 * escape_back, +escape_y
                reason = 'LEFT_FRONT_IR'
            else:
                ex, ey = -0.5 * escape_back, -escape_y
                reason = 'RIGHT_FRONT_IR'
            print(f'>>> POST_TURN_ACQUIRE ESCAPE {reason}: cmd=({ex:+.3f},{ey:+.3f})')
            chassis.drive_speed(x=ex, y=ey, z=0.0, timeout=config.DRIVE_TIMEOUT_SEC)
            time.sleep(loop_sec)
            continue

        # Geometric front range is still a hard safety condition.
        if front_cm is None:
            stop_chassis(chassis)
            clear_samples = 0
            time.sleep(loop_sec)
            continue
        if front_cm <= front_stop:
            print(f'>>> POST_TURN_ACQUIRE STOP: front={front_cm:.1f}cm <= {front_stop:.1f}cm')
            break

        # Side emergency only. Do not do normal centering yet.
        y_escape = 0.0
        x_cmd = speed
        if left_cm is not None and left_cm <= side_hard:
            y_escape = +escape_y
            x_cmd = min(speed, 0.035)
            clear_samples = 0
        elif right_cm is not None and right_cm <= side_hard:
            y_escape = -escape_y
            x_cmd = min(speed, 0.035)
            clear_samples = 0
        else:
            clear_samples += 1

        x_cmd, y_cmd, z_cmd, _, _ = controller.apply_heading_hold(
            x_cmd, y_escape, pose_tracker.get_yaw(), pose_tracker, 'POST_TURN_ACQUIRE'
        )
        chassis.drive_speed(x=x_cmd, y=y_cmd, z=z_cmd, timeout=config.DRIVE_TIMEOUT_SEC)

        travelled = _travelled_m(start_x, start_y, pose_tracker)
        if travelled is not None and travelled >= distance_target and clear_samples >= clear_needed:
            print(
                f'>>> POST_TURN_ACQUIRE OK: travelled={travelled:.3f}m '
                f'L={left_cm if left_cm is not None else "None"} '
                f'R={right_cm if right_cm is not None else "None"}'
            )
            break
        time.sleep(loop_sec)

    stop_chassis(chassis)
    controller.reset_side_owner()
    sensors.reset_filters()


def post_turn_clearance(chassis, sensors, controller, pose_tracker, relative_direction):
    """Crawl clear of the inside corner after a LEFT/RIGHT turn.

    If the inner-side Sharp sensor still sees the old corner wall very close,
    move forward slowly while adding a small outward strafe.  This prevents
    resuming 0.15 m/s while the rear/side of the chassis is still beside the
    corner edge.
    """
    if not config.ENABLE_POST_TURN_CLEARANCE:
        return
    if relative_direction not in ('LEFT', 'RIGHT'):
        return
    if not config.ENABLE_MOTION:
        return
    sensors.reset_filters()
    read_inner = sensors.read_left_sharp if relative_direction == 'LEFT' else sensors.read_right_sharp
    y_out = +config.POST_TURN_CLEARANCE_Y_SPEED * config.Y_DIR_SIGN if relative_direction == 'LEFT' else -config.POST_TURN_CLEARANCE_Y_SPEED * config.Y_DIR_SIGN
    inner_cm = None
    for _ in range(2):
        _, inner_cm = read_inner()
        time.sleep(config.POST_TURN_CLEARANCE_LOOP_SEC)
    if inner_cm is None:
        print('POST_TURN_CLEARANCE skipped: inner Sharp unavailable')
        return
    if inner_cm > config.POST_TURN_CLEARANCE_TRIGGER_CM:
        print(f'POST_TURN_CLEARANCE not needed: {relative_direction} inner={inner_cm:.1f} cm')
        return
    start_x, start_y = _pose_xy(pose_tracker)
    start_time = time.monotonic()
    print(f'>>> POST_TURN_CLEARANCE {relative_direction} inner={inner_cm:.1f}cm release={config.POST_TURN_CLEARANCE_RELEASE_CM:.1f}cm')
    while time.monotonic() - start_time < config.POST_TURN_CLEARANCE_MAX_SEC:
        ir_state = sensors.update_front_ir_guard(refresh=True)
        if ir_state['blocked']:
            print('POST_TURN_CLEARANCE stop: dual front IR blocked')
            break
        front_cm = sensors.get_front_cm()
        if front_cm is None:
            stop_chassis(chassis)
            time.sleep(config.POST_TURN_CLEARANCE_LOOP_SEC)
            continue
        if front_cm <= config.POST_TURN_CLEARANCE_FRONT_STOP_CM:
            print(f'POST_TURN_CLEARANCE stop: front={front_cm:.1f} cm')
            break
        _, inner_cm = read_inner()
        if inner_cm is None:
            print('POST_TURN_CLEARANCE abort: inner Sharp unavailable')
            break
        if inner_cm >= config.POST_TURN_CLEARANCE_RELEASE_CM:
            print(f'POST_TURN_CLEARANCE done: inner={inner_cm:.1f} cm')
            break
        travelled = _travelled_m(start_x, start_y, pose_tracker)
        if travelled is not None and travelled >= config.POST_TURN_CLEARANCE_MAX_DISTANCE_M:
            print(f'POST_TURN_CLEARANCE done: travelled={travelled:.3f} m, inner={inner_cm:.1f} cm')
            break
        x_cmd, y_cmd, z_cmd, _, _ = controller.apply_heading_hold(config.POST_TURN_CLEARANCE_FORWARD_SPEED, y_out, pose_tracker.get_yaw(), pose_tracker, 'POST_TURN_CLEARANCE')
        chassis.drive_speed(x=x_cmd, y=y_cmd, z=z_cmd, timeout=config.DRIVE_TIMEOUT_SEC)
        time.sleep(config.POST_TURN_CLEARANCE_LOOP_SEC)
    stop_chassis(chassis)

def apply_front_ir_collision_avoidance(x, y, z, mode, ir_state, sharp_left_cm, sharp_right_cm):
    """Final per-corner IR avoidance for normal forward driving.

    LEFT confirmed  -> slow + strafe RIGHT.
    RIGHT confirmed -> slow + strafe LEFT.
    BOTH confirmed  -> stop translation; aggregate FRONT logic will handle it.

    This function does not alter DFS graph state.  It only modifies the current
    chassis command, so a temporary corner reflection cannot mark a maze edge as
    blocked or explored.
    """
    if not bool(getattr(config, 'ENABLE_FRONT_IR_COLLISION_AVOIDANCE', True)):
        return (x, y, z, mode)
    if not ir_state or x <= 0.0:
        return (x, y, z, mode)
    left = bool(ir_state.get('left_confirmed', False))
    right = bool(ir_state.get('right_confirmed', False))
    if not (left or right):
        return (x, y, z, mode)
    if left and right:
        return (0.0, 0.0, z, mode + '_IR_BOTH_STOP')
    x = min(float(x), float(config.FRONT_IR_SINGLE_AVOID_FORWARD_SPEED))
    y_mag = float(config.FRONT_IR_SINGLE_AVOID_Y_SPEED) * config.Y_DIR_SIGN
    opposite_limit = float(config.FRONT_IR_AVOID_OPPOSITE_SIDE_MIN_CM)
    if left:
        if sharp_right_cm is not None and sharp_right_cm <= opposite_limit:
            return (0.0, 0.0, z, mode + '_IR_LEFT_NO_ROOM')
        return (x, +y_mag, z, mode + '_IR_LEFT_AVOID')
    if sharp_left_cm is not None and sharp_left_cm <= opposite_limit:
        return (0.0, 0.0, z, mode + '_IR_RIGHT_NO_ROOM')
    return (x, -y_mag, z, mode + '_IR_RIGHT_AVOID')

def apply_motion_safety(x, y, z, mode, front_ir_blocked=False):
    """Final safety layer before sending chassis.drive_speed().

    A confirmed dual-front-IR hit vetoes positive X only. Rotation, lateral
    escape, and reverse motion remain available so the robot can recover.
    """
    if front_ir_blocked and x > 0.0:
        return (0.0, y, z, mode + '_FRONT_IR_STOP')
    if mode == 'BOTH_TOO_CLOSE':
        return (0.0, y, z, mode + '_STOP_X')
    if 'ESCAPE_' in mode:
        x = min(x, config.ESCAPE_FORWARD_SPEED)
        return (x, y, z, mode + '_SLOW_X')
    if mode == 'NO_SENSOR':
        return (0.0, y, z, mode + '_STOP_X')
    return (x, y, z, mode)

# ==================== START GATE / OPEN AREA / EXIT ====================
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
        self.last_recovery_time = -1000000000.0
        self._learn_announced = False
        self._reject_announced = False
        self.entry_acquired = not bool(getattr(config, 'ENABLE_START_ENTRY_ACQUISITION', True))
        self.entry_failed = False
        self.entry_started_at = time.monotonic()
        self.entry_both_wall_count = 0
        self.entry_single_wall_count = 0
        self.entry_left_sharp_count = 0
        self.entry_right_sharp_count = 0
        self.entry_front_tof_count = 0
        self.entry_ir_left_count = 0
        self.entry_ir_right_count = 0
        self.entry_reason = 'DISABLED' if self.entry_acquired else None
        self._entry_announced = False
        self._entry_armed_announced = False

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
                    print(f'>>> START_GATE LEARNED inward=({self.inward_unit[0]:+.3f},{self.inward_unit[1]:+.3f}) from {distance:.3f}m')
                    self._learn_announced = True
        return self.metrics(x, y)

    def metrics(self, x, y):
        if x is None or y is None:
            return {'learned': self.inward_unit is not None, 'distance_m': None, 'progress_m': None, 'lateral_m': None}
        dx = float(x) - self.start_x
        dy = float(y) - self.start_y
        distance = math.hypot(dx, dy)
        if self.inward_unit is None:
            return {'learned': False, 'distance_m': distance, 'progress_m': None, 'lateral_m': None}
        ux, uy = self.inward_unit
        progress = dx * ux + dy * uy
        lateral = abs(-uy * dx + ux * dy)
        return {'learned': True, 'distance_m': distance, 'progress_m': progress, 'lateral_m': lateral}

    def is_outward_heading(self, heading_index):
        return int(heading_index) % 4 == self.outside_abs_dir

    @staticmethod
    def _wall_seen(value):
        return value is not None and 0.0 < float(value) <= float(config.START_ENTRY_WALL_CM)

    @staticmethod
    def _stable_count(count, seen):
        return count + 1 if seen else 0

    def update_entry_acquisition(
        self, left_cm, right_cm, front_cm, x, y, front_ir_state=None,
    ):
        """Acquire the physical entrance before enabling junction/exit logic.

        Phase 1 requests a short forward nudge.  Phase 2 accepts any stable maze
        landmark: left Sharp, right Sharp, front ToF, or either front IR.  The
        nudge prevents a sensor already seeing the staging boundary at startup
        from immediately becoming maze topology.  A confirmed IR hit or an
        emergency-near ToF reading arms the detector early for collision safety.
        """
        if self.entry_acquired:
            return {'state': 'ACQUIRED', 'reason': self.entry_reason}
        if self.entry_failed:
            return {'state': 'FAILED', 'reason': self.entry_reason}

        metrics = self.metrics(x, y)
        travelled = metrics.get('distance_m')
        elapsed = time.monotonic() - self.entry_started_at
        left_wall = self._wall_seen(left_cm)
        right_wall = self._wall_seen(right_cm)
        front_wall = self._wall_seen(front_cm)
        ir_state = front_ir_state or {}
        ir_left = bool(ir_state.get('left_confirmed', False))
        ir_right = bool(ir_state.get('right_confirmed', False))

        nudge_distance = float(getattr(config, 'START_ENTRY_NUDGE_DISTANCE_M', 0.05))
        nudge_timeout = float(getattr(config, 'START_ENTRY_NUDGE_MAX_SEC', 2.0))
        emergency_front_cm = float(
            getattr(config, 'START_ENTRY_NUDGE_EMERGENCY_FRONT_CM', 6.0)
        )
        distance_armed = travelled is not None and travelled >= nudge_distance
        timeout_armed = elapsed >= nudge_timeout
        emergency_tof = (
            front_cm is not None
            and 0.0 < float(front_cm) <= emergency_front_cm
        )
        safety_armed = ir_left or ir_right or emergency_tof
        armed = bool(distance_armed or timeout_armed or safety_armed)

        if not armed:
            self.entry_both_wall_count = 0
            self.entry_single_wall_count = 0
            self.entry_left_sharp_count = 0
            self.entry_right_sharp_count = 0
            self.entry_front_tof_count = 0
            self.entry_ir_left_count = 0
            self.entry_ir_right_count = 0
            return {
                'state': 'SEARCH',
                'phase': 'NUDGE',
                'reason': 'FORWARD_NUDGE',
                'armed': False,
                'travelled_m': travelled,
                'left_wall': left_wall,
                'right_wall': right_wall,
                'front_wall': front_wall,
                'ir_left': ir_left,
                'ir_right': ir_right,
            }

        if not self._entry_armed_announced:
            if safety_armed:
                armed_reason = 'SAFETY_SENSOR'
            elif distance_armed:
                armed_reason = 'NUDGE_DISTANCE'
            else:
                armed_reason = 'NUDGE_TIMEOUT'
            print(
                f'>>> START ENTRY SENSORS ARMED reason={armed_reason} '
                f'travel={(travelled if travelled is not None else 0.0):.3f}m'
            )
            self._entry_armed_announced = True

        self.entry_left_sharp_count = self._stable_count(
            self.entry_left_sharp_count, left_wall,
        )
        self.entry_right_sharp_count = self._stable_count(
            self.entry_right_sharp_count, right_wall,
        )
        self.entry_front_tof_count = self._stable_count(
            self.entry_front_tof_count, front_wall,
        )
        self.entry_ir_left_count = self._stable_count(
            self.entry_ir_left_count, ir_left,
        )
        self.entry_ir_right_count = self._stable_count(
            self.entry_ir_right_count, ir_right,
        )
        self.entry_both_wall_count = min(
            self.entry_left_sharp_count, self.entry_right_sharp_count,
        )
        self.entry_single_wall_count = max(
            self.entry_left_sharp_count, self.entry_right_sharp_count,
        )

        reason = None
        sharp_samples = int(getattr(config, 'START_ENTRY_SHARP_SAMPLES', 3))
        tof_samples = int(getattr(config, 'START_ENTRY_TOF_SAMPLES', 3))
        ir_samples = int(getattr(config, 'START_ENTRY_IR_SAMPLES', 2))
        if self.entry_both_wall_count >= sharp_samples:
            reason = 'SHARP_BOTH'
        elif min(self.entry_ir_left_count, self.entry_ir_right_count) >= ir_samples:
            reason = 'IR_BOTH'
        elif self.entry_left_sharp_count >= sharp_samples:
            reason = 'SHARP_LEFT'
        elif self.entry_right_sharp_count >= sharp_samples:
            reason = 'SHARP_RIGHT'
        elif self.entry_front_tof_count >= tof_samples:
            reason = 'TOF_FRONT'
        elif self.entry_ir_left_count >= ir_samples:
            reason = 'IR_LEFT'
        elif self.entry_ir_right_count >= ir_samples:
            reason = 'IR_RIGHT'

        if reason is not None:
            self.entry_acquired = True
            self.entry_reason = reason
            if not self._entry_announced:
                print(
                    f'>>> START ENTRY ACQUIRED reason={reason} '
                    f'travel={(travelled if travelled is not None else 0.0):.3f}m '
                    f'F={front_cm if front_cm is not None else "---"} '
                    f'L={left_cm if left_cm is not None else "---"} '
                    f'R={right_cm if right_cm is not None else "---"} '
                    f'IR={int(ir_left)}/{int(ir_right)}'
                )
                self._entry_announced = True
            return {'state': 'ACQUIRED', 'reason': reason}

        over_distance = travelled is not None and travelled >= float(config.START_ENTRY_MAX_TRAVEL_M)
        over_time = elapsed >= float(config.START_ENTRY_MAX_SEC)
        if over_distance or over_time:
            self.entry_failed = True
            self.entry_reason = 'NO_MAZE_WALL_LANDMARK'
            print(
                f'>>> START ENTRY FAILED: no stable maze wall/landmark after '
                f'{(travelled if travelled is not None else 0.0):.3f}m / {elapsed:.1f}s'
            )
            return {'state': 'FAILED', 'reason': self.entry_reason}

        return {
            'state': 'SEARCH',
            'phase': 'SENSOR_SEARCH',
            'reason': 'WAITING_FOR_ANY_ENTRY_SENSOR',
            'armed': True,
            'travelled_m': travelled,
            'left_wall': left_wall,
            'right_wall': right_wall,
            'front_wall': front_wall,
            'ir_left': ir_left,
            'ir_right': ir_right,
        }

    def should_reject_exit(self, x, y, heading_index):
        if not bool(getattr(config, 'ENABLE_START_GATE_GUARD', True)):
            return False
        m = self.metrics(x, y)
        distance = m['distance_m']
        if distance is not None and distance <= float(config.START_EXIT_REJECT_RADIUS_M):
            return True
        if m['learned'] and self.is_outward_heading(heading_index) and (m['progress_m'] is not None) and (m['progress_m'] <= float(config.START_EXIT_REJECT_INNER_PROGRESS_M)) and (m['lateral_m'] is not None) and (m['lateral_m'] <= float(config.START_EXIT_REJECT_LATERAL_M)):
            return True
        return False

    def should_force_junction_decision(self, x, y, heading_index):
        """True near the old entrance while travelling in the outward heading."""
        if not bool(
            getattr(config, 'ENABLE_START_GATE_EARLY_DECISION_BARRIER', True)
        ):
            return False
        if not self.is_outward_heading(heading_index):
            return False
        m = self.metrics(x, y)
        if not m['learned'] or m['progress_m'] is None or m['lateral_m'] is None:
            return False
        return bool(
            m['progress_m'] <= float(
                getattr(config, 'START_GATE_DECISION_TRIGGER_PROGRESS_M', 0.70)
            )
            and m['lateral_m'] <= float(
                getattr(config, 'START_GATE_DECISION_TRIGGER_LATERAL_M', 0.85)
            )
        )

    def should_force_return(self, x, y, heading_index):
        if not bool(getattr(config, 'ENABLE_START_GATE_GUARD', True)):
            return False
        if not self.is_outward_heading(heading_index):
            return False
        m = self.metrics(x, y)
        if not m['learned']:
            return False
        if m['progress_m'] is None or m['lateral_m'] is None:
            return False
        in_gate = m['progress_m'] <= float(config.START_GATE_BLOCK_INNER_M) and m['lateral_m'] <= float(config.START_GATE_HALF_WIDTH_M)
        cooldown_ok = time.monotonic() - self.last_recovery_time >= float(config.START_GATE_RECOVERY_COOLDOWN_SEC)
        return in_gate and cooldown_ok

    def mark_recovery(self):
        self.last_recovery_time = time.monotonic()

class OpenAreaExitManager:
    """Hysteretic open-area state + V12.5 two-stage exit proof."""

    def __init__(self):
        self.started_at = time.monotonic()
        self.open_area_active = False
        self.open_enter_count = 0
        self.open_exit_count = 0
        self.open_area_start_x = None
        self.open_area_start_y = None
        self.open_area_start_time = None
        self.exit_start_count = 0
        self.exit_candidate = None
        self.exit_ready_event = None
        self.exit_ready_count = 0
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
        had = self.exit_candidate is not None or self.exit_start_count > 0 or self.exit_ready_event is not None
        if had and reason:
            print(f'>>> EXIT_CANDIDATE CANCEL reason={reason}')
        self.exit_candidate = None
        self.exit_start_count = 0
        self.exit_ready_event = None
        self.exit_ready_count = 0

    def cancel_exit_candidate(self, reason=None):
        self._cancel_exit_candidate(reason)

    def confirm_exit(self):
        """Commit a ready exit only after the junction detector did not veto it."""
        if self.exit_found:
            return True
        if self.exit_ready_event is None:
            return False
        self.exit_found = True
        self.exit_event = dict(self.exit_ready_event)
        print(
            f">>> EXIT FOUND travelled={self.exit_event.get('travelled_m', 0.0):.3f}m "
            f"open_run={self.exit_event.get('open_area_travel_m', 0.0):.3f}m "
            f"time={self.exit_event.get('confirm_sec', 0.0):.2f}s "
            f"F={self.exit_event.get('front_cm')} L={self.exit_event.get('left_cm')} R={self.exit_event.get('right_cm')}"
        )
        return True

    def update(self, front_cm, left_cm, right_cm, pose_x, pose_y, node_count=0, heading_error=None, start_gate_block_exit=False):
        now = time.monotonic()
        runtime = now - self.started_at
        candidate_started = False
        candidate_cancelled = False
        open_area_entered = False
        open_area_left = False
        front_blocked = self._at_most(front_cm, config.STOP_FRONT_CM)
        broad_open_sample = self._at_least(front_cm, config.OPEN_AREA_FRONT_MIN_CM) and self._at_least(left_cm, config.OPEN_AREA_SIDE_ENTER_CM) and self._at_least(right_cm, config.OPEN_AREA_SIDE_ENTER_CM)
        wall_reacquired = self._at_most(left_cm, config.OPEN_AREA_SIDE_EXIT_CM) or self._at_most(right_cm, config.OPEN_AREA_SIDE_EXIT_CM)

        if front_blocked:
            if self.open_area_active:
                open_area_left = True
                print('>>> OPEN_AREA EXIT reason=FRONT_BLOCKED')
            self.open_area_active = False
            self.open_enter_count = 0
            self.open_exit_count = 0
            self.open_area_start_x = None
            self.open_area_start_y = None
            self.open_area_start_time = None
        elif not self.open_area_active:
            self.open_enter_count = self.open_enter_count + 1 if broad_open_sample else 0
            if self.open_enter_count >= int(config.OPEN_AREA_ENTER_SAMPLES):
                self.open_area_active = True
                self.open_enter_count = 0
                self.open_exit_count = 0
                self.open_area_start_x = pose_x
                self.open_area_start_y = pose_y
                self.open_area_start_time = now
                open_area_entered = True
                print(f'>>> OPEN_AREA ENTER F={fmt(front_cm)} L={fmt(left_cm)} R={fmt(right_cm)}')
        else:
            self.open_exit_count = self.open_exit_count + 1 if wall_reacquired else 0
            if self.open_exit_count >= int(config.OPEN_AREA_EXIT_SAMPLES):
                self.open_area_active = False
                self.open_exit_count = 0
                self.open_enter_count = 0
                self.open_area_start_x = None
                self.open_area_start_y = None
                self.open_area_start_time = None
                open_area_left = True
                print(f'>>> OPEN_AREA EXIT reason=WALL_REACQUIRED L={fmt(left_cm)} R={fmt(right_cm)}')

        if start_gate_block_exit:
            if self.exit_candidate is not None or self.exit_start_count > 0 or self.exit_ready_event is not None:
                self._cancel_exit_candidate('START_GATE')
            self.exit_start_count = 0

        heading_ok = heading_error is None or abs(float(heading_error)) <= float(config.EXIT_MAX_HEADING_ERROR_DEG)
        enough_history = runtime >= float(config.EXIT_MIN_RUNTIME_SEC) and int(node_count) >= int(config.EXIT_MIN_NODE_COUNT)
        exit_strong = bool(config.ENABLE_EXIT_DETECTION) and (not start_gate_block_exit) and self.open_area_active and enough_history and heading_ok and self._at_least(front_cm, config.EXIT_FRONT_START_CM) and self._at_least(left_cm, config.EXIT_SIDE_START_CM) and self._at_least(right_cm, config.EXIT_SIDE_START_CM)
        exit_keep = not start_gate_block_exit and self.open_area_active and heading_ok and self._at_least(front_cm, config.EXIT_FRONT_KEEP_CM) and self._at_least(left_cm, config.EXIT_SIDE_KEEP_CM) and self._at_least(right_cm, config.EXIT_SIDE_KEEP_CM)

        open_area_travel = self._distance_xy(self.open_area_start_x, self.open_area_start_y, pose_x, pose_y) if self.open_area_active else None
        if self.exit_found:
            return {'open_area_active': self.open_area_active, 'open_area_entered': open_area_entered, 'open_area_left': open_area_left, 'exit_candidate_active': False, 'exit_candidate_started': False, 'exit_candidate_cancelled': False, 'exit_ready': False, 'exit_found': True, 'exit_event': self.exit_event, 'open_area_travel_m': open_area_travel}

        if self.exit_candidate is None:
            self.exit_start_count = self.exit_start_count + 1 if exit_strong else 0
            if self.exit_start_count >= int(config.EXIT_START_SAMPLES):
                self.exit_candidate = {
                    'start_x': pose_x,
                    'start_y': pose_y,
                    'start_time': now,
                    'strong_samples': int(self.exit_start_count),
                    'min_front_cm': float(front_cm),
                    'min_left_cm': float(left_cm),
                    'min_right_cm': float(right_cm),
                }
                candidate_started = True
                print(f'>>> EXIT_CANDIDATE START F={front_cm:.1f} L={left_cm:.1f} R={right_cm:.1f} confirm={config.EXIT_CONFIRM_DISTANCE_M:.2f}m open_run>={getattr(config, "EXIT_OPEN_AREA_MIN_TRAVEL_M", 0.75):.2f}m')
        else:
            c = self.exit_candidate
            if not exit_keep:
                candidate_cancelled = True
                self._cancel_exit_candidate('OPENNESS_LOST')
            else:
                if exit_strong:
                    c['strong_samples'] += 1
                c['min_front_cm'] = min(c['min_front_cm'], float(front_cm))
                c['min_left_cm'] = min(c['min_left_cm'], float(left_cm))
                c['min_right_cm'] = min(c['min_right_cm'], float(right_cm))
                travelled = self._distance_xy(c['start_x'], c['start_y'], pose_x, pose_y)
                elapsed = now - c['start_time']
                distance_ok = travelled is not None and travelled >= float(config.EXIT_CONFIRM_DISTANCE_M)
                time_ok = elapsed >= float(config.EXIT_CONFIRM_MIN_SEC)
                samples_ok = c['strong_samples'] >= int(config.EXIT_CONFIRM_STRONG_SAMPLES)
                open_run_ok = (not bool(getattr(config, 'ENABLE_EXIT_PROOF_V2', True))) or (open_area_travel is not None and open_area_travel >= float(getattr(config, 'EXIT_OPEN_AREA_MIN_TRAVEL_M', 0.75)))
                proof_now = distance_ok and time_ok and samples_ok and open_run_ok
                if proof_now:
                    self.exit_ready_count += 1
                else:
                    self.exit_ready_count = 0
                    self.exit_ready_event = None
                if self.exit_ready_count >= int(getattr(config, 'EXIT_READY_CONFIRM_SAMPLES', 2)):
                    self.exit_ready_event = {
                        'raw_x': None if pose_x is None else float(pose_x),
                        'raw_y': None if pose_y is None else float(pose_y),
                        'travelled_m': float(travelled),
                        'open_area_travel_m': float(open_area_travel),
                        'confirm_sec': float(elapsed),
                        'strong_samples': int(c['strong_samples']),
                        'front_cm': None if front_cm is None else float(front_cm),
                        'left_cm': None if left_cm is None else float(left_cm),
                        'right_cm': None if right_cm is None else float(right_cm),
                        'node_count': int(node_count),
                        'runtime_sec': float(runtime),
                    }
                    print(f'>>> EXIT READY travelled={travelled:.3f}m open_run={open_area_travel:.3f}m; waiting for junction veto')

        return {
            'open_area_active': self.open_area_active,
            'open_area_entered': open_area_entered,
            'open_area_left': open_area_left,
            'exit_candidate_active': self.exit_candidate is not None and (not self.exit_found),
            'exit_candidate_started': candidate_started,
            'exit_candidate_cancelled': candidate_cancelled,
            'exit_ready': self.exit_ready_event is not None and (not self.exit_found),
            'exit_found': self.exit_found,
            'exit_event': self.exit_event,
            'open_area_travel_m': open_area_travel,
        }

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

# ==================== PASSIVE SLAM-STYLE MAPPING ====================
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
    SENSOR_FRONT = 'front_tof'
    SENSOR_LEFT = 'left_sharp'
    SENSOR_RIGHT = 'right_sharp'
    SENSOR_IR = 'left_front_ir'

    def __init__(self, output_dir=None):
        self.enabled = bool(_map_cfg('ENABLE_MAPPING', True))
        self.output_dir = output_dir or _map_cfg('MAP_OUTPUT_DIR', 'mapping_output')
        self.initialized = False
        self.start_raw_x = None
        self.start_raw_y = None
        self.start_yaw = None
        self.start_monotonic = None
        self.global_corr_x = 0.0
        self.global_corr_y = 0.0
        self.samples = []
        self.grid = {}
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
        self.position_rotation_deg = float(_map_cfg('MAP_POSITION_ROTATION_DEG', 0.0))
        self.position_auto_aligned = not bool(_map_cfg('MAP_AUTO_ALIGN_INITIAL_PATH', True))
        self._auto_align_reported = False
        self._last_wall_hit = {}

    def initialize(self, raw_x, raw_y, start_yaw, heading_index=0):
        if not self.enabled:
            return None
        raw_x = _map_safe_float(raw_x)
        raw_y = _map_safe_float(raw_y)
        start_yaw = _map_safe_float(start_yaw)
        if raw_x is None or raw_y is None:
            raise ValueError('Mapper requires valid starting x/y')
        if start_yaw is None:
            start_yaw = 0.0
        self.start_raw_x = raw_x
        self.start_raw_y = raw_y
        self.start_yaw = normalize_angle_deg(start_yaw)
        self.start_monotonic = time.monotonic()
        self.last_autosave_monotonic = self.start_monotonic
        self.initialized = True
        os.makedirs(self.output_dir, exist_ok=True)
        if bool(_map_cfg('MAP_CLEAR_OUTPUT_ON_START', True)):
            self._clear_old_outputs()
        return self.record_pose(raw_x, raw_y, start_yaw, heading_index=heading_index, mode='START', force=True)

    def _clear_old_outputs(self):
        for name in ('trajectory.csv', 'wall_points.csv', 'occupancy_grid.csv', 'occupancy_grid.json', 'nodes.json', 'loop_closures.json', 'mapping_summary.json', 'exit.json', 'maze_map.svg', 'maze_map.png'):
            path = os.path.join(self.output_dir, name)
            try:
                if os.path.isfile(path):
                    os.remove(path)
            except OSError:
                pass

    def _raw_position_unrotated(self, raw_x, raw_y):
        dx = float(raw_x) - self.start_raw_x
        dy = float(raw_y) - self.start_raw_y
        if bool(_map_cfg('MAP_SWAP_RAW_XY', False)):
            dx, dy = (dy, dx)
        dx *= float(_map_cfg('MAP_RAW_X_SIGN', -1.0))
        dy *= float(_map_cfg('MAP_RAW_Y_SIGN', +1.0))
        return (dx, dy)

    @staticmethod
    def _rotate_xy(dx, dy, rot_deg):
        if abs(rot_deg) <= 1e-12:
            return (dx, dy)
        a = math.radians(rot_deg)
        c, ss = (math.cos(a), math.sin(a))
        return (c * dx - ss * dy, ss * dx + c * dy)

    def _raw_position_to_base_map(self, raw_x, raw_y):
        dx, dy = self._raw_position_unrotated(raw_x, raw_y)
        return self._rotate_xy(dx, dy, self.position_rotation_deg)

    def _maybe_auto_align_position(self, raw_x, raw_y, heading_index):
        if self.position_auto_aligned:
            return False
        if not bool(_map_cfg('MAP_AUTO_ALIGN_INITIAL_PATH', True)):
            self.position_auto_aligned = True
            return False
        if heading_index is None:
            return False
        wanted = int(_map_cfg('MAP_AUTO_ALIGN_MAX_HEADING_INDEX', 0)) % 4
        if int(heading_index) % 4 != wanted:
            return False
        dx, dy = self._raw_position_unrotated(raw_x, raw_y)
        dist = math.hypot(dx, dy)
        need = float(_map_cfg('MAP_AUTO_ALIGN_MIN_TRAVEL_M', 0.18))
        if dist < need:
            return False
        auto_rot = math.degrees(math.atan2(dx, dy))
        fixed = float(_map_cfg('MAP_POSITION_ROTATION_DEG', 0.0))
        self.position_rotation_deg = normalize_angle_deg(fixed + auto_rot)
        self.position_auto_aligned = True
        if not self.loop_closures and abs(self.global_corr_x) < 1e-09 and (abs(self.global_corr_y) < 1e-09):
            for old_sample in self.samples:
                bx, by = self._raw_position_to_base_map(old_sample.raw_x, old_sample.raw_y)
                old_sample.base_x = bx
                old_sample.base_y = by
                old_sample.map_x = bx
                old_sample.map_y = by
            self._rebuild_grid()
        if not self._auto_align_reported:
            print(f'>>> MAP AUTO ALIGN rotation={self.position_rotation_deg:+.1f} deg from initial travel={dist:.3f} m')
            self._auto_align_reported = True
        return True

    def _yaw_to_map_theta(self, yaw_deg, heading_index=None):
        yaw_deg = _map_safe_float(yaw_deg)
        if yaw_deg is None:
            return 0.0 if heading_index is None else float(int(heading_index) % 4 * 90)
        theta = normalize_angle_deg((yaw_deg - self.start_yaw) * float(_map_cfg('MAP_YAW_RIGHT_SIGN', +1.0)))
        if heading_index is None:
            return theta
        expected = normalize_angle_deg(float(int(heading_index) % 4 * 90))
        if bool(_map_cfg('MAP_SENSOR_USE_CARDINAL_HEADING', True)):
            return expected
        error = shortest_angle_error_deg(expected, theta)
        if abs(error) <= float(_map_cfg('MAP_YAW_CARDINAL_MAX_ERROR_DEG', 22.0)):
            return theta
        if bool(_map_cfg('MAP_FALLBACK_TO_CARDINAL_HEADING', True)):
            self.yaw_fallback_count += 1
            return expected
        return theta

    def update(self, raw_x, raw_y, yaw_deg, front_cm=None, left_cm=None, right_cm=None, ir_value=None, heading_index=None, mode='RUN', map_ranges=True, force=False):
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
        if not force and self.last_record_monotonic is not None and (now - self.last_record_monotonic < float(_map_cfg('MAP_MIN_RECORD_INTERVAL_SEC', 0.045))):
            return None
        max_samples = int(_map_cfg('MAP_MAX_SAMPLES', 60000))
        if max_samples > 0 and len(self.samples) >= max_samples:
            if not self._warned_sample_limit:
                print('MAPPER WARNING: sample limit reached; mapping samples paused')
                self._warned_sample_limit = True
            return None
        self._maybe_auto_align_position(raw_x, raw_y, heading_index)
        base_x, base_y = self._raw_position_to_base_map(raw_x, raw_y)
        map_x = base_x + self.global_corr_x
        map_y = base_y + self.global_corr_y
        theta = self._yaw_to_map_theta(yaw_deg, heading_index)
        h = -1 if heading_index is None else int(heading_index) % 4
        sample = _MapSample(index=len(self.samples), time_sec=now - self.start_monotonic, raw_x=raw_x, raw_y=raw_y, yaw_deg=yaw_deg, base_x=base_x, base_y=base_y, map_x=map_x, map_y=map_y, theta_deg=theta, heading_index=h, front_cm=_map_safe_float(front_cm), left_cm=_map_safe_float(left_cm), right_cm=_map_safe_float(right_cm), ir_value=ir_value, mode=str(mode or ''), map_ranges=bool(map_ranges))
        self.samples.append(sample)
        self.last_record_monotonic = now
        if sample.map_ranges:
            self._integrate_sample(sample)
        autosave = float(_map_cfg('MAP_AUTOSAVE_SEC', 0.0))
        if autosave > 0 and now - self.last_autosave_monotonic >= autosave:
            self.save_all(rebuild=False, quiet=True)
            self.last_autosave_monotonic = now
        return sample

    def record_pose(self, raw_x, raw_y, yaw_deg, heading_index=None, mode='POSE_ONLY', force=True):
        return self.update(raw_x, raw_y, yaw_deg, heading_index=heading_index, mode=mode, map_ranges=False, force=force)

    @staticmethod
    def _forward_right_to_world(x, y, theta_deg, forward_m, right_m):
        a = math.radians(theta_deg)
        fx, fy = (math.sin(a), math.cos(a))
        rx, ry = (math.cos(a), -math.sin(a))
        return (x + forward_m * fx + right_m * rx, y + forward_m * fy + right_m * ry)

    def _sensor_params(self, name):
        if name == self.SENSOR_FRONT:
            return dict(angle=float(_map_cfg('MAP_FRONT_SENSOR_ANGLE_DEG', 0.0)), forward=float(_map_cfg('MAP_FRONT_SENSOR_FORWARD_M', 0.08)), right=float(_map_cfg('MAP_FRONT_SENSOR_RIGHT_M', 0.0)), min_cm=float(_map_cfg('MAP_TOF_MIN_CM', 4.0)), free_max=float(_map_cfg('MAP_TOF_FREE_MAX_CM', 70.0)), hit_max=float(_map_cfg('MAP_TOF_OCCUPIED_MAX_CM', 45.0)), hit_score=int(_map_cfg('MAP_TOF_HIT_SCORE', 7)), free_score=int(_map_cfg('MAP_TOF_FREE_SCORE', -1)))
        if name == self.SENSOR_LEFT:
            return dict(angle=float(_map_cfg('MAP_LEFT_SENSOR_ANGLE_DEG', -90.0)), forward=float(_map_cfg('MAP_LEFT_SENSOR_FORWARD_M', 0.02)), right=float(_map_cfg('MAP_LEFT_SENSOR_RIGHT_M', -0.1)), min_cm=float(_map_cfg('MAP_SHARP_MIN_CM', 4.0)), free_max=float(_map_cfg('MAP_SHARP_FREE_MAX_CM', 24.0)), hit_max=float(_map_cfg('MAP_SHARP_OCCUPIED_MAX_CM', 18.0)), hit_score=int(_map_cfg('MAP_SHARP_HIT_SCORE', 5)), free_score=int(_map_cfg('MAP_SHARP_FREE_SCORE', -1)))
        if name == self.SENSOR_RIGHT:
            return dict(angle=float(_map_cfg('MAP_RIGHT_SENSOR_ANGLE_DEG', +90.0)), forward=float(_map_cfg('MAP_RIGHT_SENSOR_FORWARD_M', 0.02)), right=float(_map_cfg('MAP_RIGHT_SENSOR_RIGHT_M', +0.1)), min_cm=float(_map_cfg('MAP_SHARP_MIN_CM', 4.0)), free_max=float(_map_cfg('MAP_SHARP_FREE_MAX_CM', 24.0)), hit_max=float(_map_cfg('MAP_SHARP_OCCUPIED_MAX_CM', 18.0)), hit_score=int(_map_cfg('MAP_SHARP_HIT_SCORE', 5)), free_score=int(_map_cfg('MAP_SHARP_FREE_SCORE', -1)))
        raise ValueError(name)

    def _ray(self, sample, name, distance_cm):
        distance_cm = _map_safe_float(distance_cm)
        if distance_cm is None:
            return None
        p = self._sensor_params(name)
        if distance_cm < p['min_cm']:
            return None
        used_cm = min(distance_cm, p['free_max'])
        if used_cm <= 0:
            return None
        has_hit = distance_cm <= p['hit_max']
        if name == self.SENSOR_FRONT and (not has_hit) and (float(_map_cfg('MAP_TOF_NO_HIT_FREE_MAX_CM', 28.0)) > 0):
            used_cm = min(used_cm, float(_map_cfg('MAP_TOF_NO_HIT_FREE_MAX_CM', 28.0)))
        ox, oy = self._forward_right_to_world(sample.map_x, sample.map_y, sample.theta_deg, p['forward'], p['right'])
        ray_theta = normalize_angle_deg(sample.theta_deg + p['angle'])
        a = math.radians(ray_theta)
        d = used_cm / 100.0
        ex = ox + d * math.sin(a)
        ey = oy + d * math.cos(a)
        return dict(sensor=name, origin_x=ox, origin_y=oy, end_x=ex, end_y=ey, measured_cm=distance_cm, used_cm=used_cm, has_hit=has_hit, hit_score=p['hit_score'], free_score=p['free_score'])

    def _world_to_cell(self, x, y):
        r = max(0.005, float(_map_cfg('MAP_RESOLUTION_M', 0.025)))
        return (int(round(x / r)), int(round(y / r)))

    def _cell_to_world(self, gx, gy):
        r = max(0.005, float(_map_cfg('MAP_RESOLUTION_M', 0.025)))
        return (gx * r, gy * r)

    @staticmethod
    def _bresenham(x0, y0, x1, y1):
        cells = []
        dx, sx = (abs(x1 - x0), 1 if x0 < x1 else -1)
        dy, sy = (-abs(y1 - y0), 1 if y0 < y1 else -1)
        err = dx + dy
        x, y = (x0, y0)
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
        lo = int(_map_cfg('MAP_EVIDENCE_MIN', -30))
        hi = int(_map_cfg('MAP_EVIDENCE_MAX', +30))
        self.grid[cell] = int(clamp(int(self.grid.get(cell, 0)) + int(delta), lo, hi))

    def _mark_robot_footprint_free(self, sample):
        radius = float(_map_cfg('MAP_ROBOT_FREE_RADIUS_M', 0.11))
        score = int(_map_cfg('MAP_ROBOT_FREE_SCORE', -3))
        res = float(_map_cfg('MAP_RESOLUTION_M', 0.025))
        c0 = self._world_to_cell(sample.map_x, sample.map_y)
        n = max(0, int(math.ceil(radius / res)))
        for dx in range(-n, n + 1):
            for dy in range(-n, n + 1):
                if math.hypot(dx * res, dy * res) <= radius:
                    self._update_cell((c0[0] + dx, c0[1] + dy), score)

    def _integrate_ray(self, sample, ray):
        sc = self._world_to_cell(ray['origin_x'], ray['origin_y'])
        ec = self._world_to_cell(ray['end_x'], ray['end_y'])
        cells = self._bresenham(sc[0], sc[1], ec[0], ec[1])
        free_cells = cells[:-1] if ray['has_hit'] else cells
        for cell in free_cells:
            self._update_cell(cell, ray['free_score'])
        sensor = ray['sensor']
        if ray['has_hit'] and cells:
            self._update_cell(cells[-1], ray['hit_score'])
            if bool(_map_cfg('MAP_CONNECT_CONSECUTIVE_WALL_HITS', True)):
                prev = self._last_wall_hit.get(sensor)
                current = {'x': ray['end_x'], 'y': ray['end_y'], 'heading_index': sample.heading_index}
                if prev is not None and prev.get('heading_index') == sample.heading_index and (sample.heading_index >= 0):
                    gap = math.hypot(current['x'] - prev['x'], current['y'] - prev['y'])
                    if gap <= float(_map_cfg('MAP_WALL_CONNECT_MAX_M', 0.18)):
                        pc = self._world_to_cell(prev['x'], prev['y'])
                        cc = self._world_to_cell(current['x'], current['y'])
                        for cell in self._bresenham(pc[0], pc[1], cc[0], cc[1]):
                            self._update_cell(cell, int(_map_cfg('MAP_WALL_CONNECT_SCORE', 4)))
                self._last_wall_hit[sensor] = current
            self.wall_points.append(dict(sample_index=sample.index, time_sec=sample.time_sec, sensor=sensor, x=ray['end_x'], y=ray['end_y'], distance_cm=ray['measured_cm']))
        else:
            self._last_wall_hit.pop(sensor, None)
        return ray

    def _ir_is_wall(self, value):
        if value is None:
            return False
        try:
            return int(value) == int(_map_cfg('MAP_IR_WALL_LEVEL', 0))
        except Exception:
            return False

    def _integrate_ir(self, sample, left_ray):
        if not self._ir_is_wall(sample.ir_value):
            return
        if bool(_map_cfg('MAP_IR_CONFIRM_LEFT_SHARP', True)) and left_ray is not None and left_ray['has_hit'] and (left_ray['measured_cm'] <= float(_map_cfg('MAP_IR_CONFIRM_MAX_SHARP_CM', 22.0))):
            cell = self._world_to_cell(left_ray['end_x'], left_ray['end_y'])
            self._update_cell(cell, int(_map_cfg('MAP_IR_CONFIRM_SCORE', 4)))
            self.wall_points.append(dict(sample_index=sample.index, time_sec=sample.time_sec, sensor='ir_confirm_left', x=left_ray['end_x'], y=left_ray['end_y'], distance_cm=left_ray['measured_cm']))
            self.ir_confirm_count += 1
            return
        if not bool(_map_cfg('MAP_IR_FALLBACK_ENABLED', True)):
            return
        ox, oy = self._forward_right_to_world(sample.map_x, sample.map_y, sample.theta_deg, float(_map_cfg('MAP_IR_SENSOR_FORWARD_M', 0.08)), float(_map_cfg('MAP_IR_SENSOR_RIGHT_M', -0.07)))
        theta = normalize_angle_deg(sample.theta_deg + float(_map_cfg('MAP_IR_SENSOR_ANGLE_DEG', -45.0)))
        d = float(_map_cfg('MAP_IR_ASSUMED_RANGE_M', 0.12))
        a = math.radians(theta)
        ex, ey = (ox + d * math.sin(a), oy + d * math.cos(a))
        c = self._world_to_cell(ex, ey)
        radius = max(0, int(_map_cfg('MAP_IR_FALLBACK_PATCH_RADIUS_CELLS', 1)))
        score = int(_map_cfg('MAP_IR_FALLBACK_HIT_SCORE', 1))
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                if abs(dx) + abs(dy) <= radius:
                    self._update_cell((c[0] + dx, c[1] + dy), score)
        self.wall_points.append(dict(sample_index=sample.index, time_sec=sample.time_sec, sensor=self.SENSOR_IR, x=ex, y=ey, distance_cm=d * 100.0))
        self.ir_fallback_count += 1

    def _integrate_sample(self, sample):
        self._mark_robot_footprint_free(sample)
        left_ray = None
        for name, dist in ((self.SENSOR_FRONT, sample.front_cm), (self.SENSOR_LEFT, sample.left_cm), (self.SENSOR_RIGHT, sample.right_cm)):
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

    def observe_junction(self, node_id, is_new, raw_x, raw_y, yaw_deg, heading_index=None):
        if not self.enabled or not self.initialized or node_id is None:
            return None
        sample = self.record_pose(raw_x, raw_y, yaw_deg, heading_index=heading_index, mode='JUNCTION_' + ('NEW' if is_new else 'KNOWN'), force=True)
        if sample is None:
            return None
        idx, now = (sample.index, sample.time_sec)
        if node_id not in self.node_anchors:
            self.node_anchors[node_id] = dict(x=sample.map_x, y=sample.map_y, sample_index=idx, first_seen_time=now)
            self.node_events.append(dict(time_sec=now, node_id=node_id, event='ANCHOR_NEW' if is_new else 'ANCHOR_RECOVERED', sample_index=idx, map_x=sample.map_x, map_y=sample.map_y))
            if self.last_known_junction_sample_index is None or not is_new:
                self.last_known_junction_sample_index = idx
            if bool(_map_cfg('MAP_SAVE_ON_JUNCTION', True)):
                self.save_all(rebuild=False, quiet=True)
            return dict(corrected=False, error_m=0.0)
        anchor = self.node_anchors[node_id]
        ex = anchor['x'] - sample.map_x
        ey = anchor['y'] - sample.map_y
        em = math.hypot(ex, ey)
        min_e = float(_map_cfg('MAP_LOOP_CLOSURE_MIN_ERROR_M', 0.015))
        max_e = float(_map_cfg('MAP_LOOP_CLOSURE_MAX_ERROR_M', 0.35))
        gain = clamp(float(_map_cfg('MAP_LOOP_CLOSURE_GAIN', 1.0)), 0.0, 1.0)
        corrected = False
        reason = 'NO_CORRECTION_NEEDED'
        if em >= min_e:
            if em <= max_e:
                start_idx = self.last_known_junction_sample_index
                if start_idx is None:
                    start_idx = 0
                start_idx = max(0, min(int(start_idx), idx))
                ax, ay = (ex * gain, ey * gain)
                denom = max(1, idx - start_idx)
                for i in range(start_idx, idx + 1):
                    t = float(i - start_idx) / denom
                    self.samples[i].map_x += ax * t
                    self.samples[i].map_y += ay * t
                for nid, a in self.node_anchors.items():
                    if nid == node_id:
                        continue
                    ai = int(a.get('sample_index', -1))
                    if start_idx <= ai <= idx:
                        t = float(ai - start_idx) / denom
                        a['x'] += ax * t
                        a['y'] += ay * t
                self.global_corr_x += ax
                self.global_corr_y += ay
                self.loop_closures.append(dict(time_sec=now, node_id=node_id, sample_index=idx, segment_start_index=start_idx, raw_error_x_m=ex, raw_error_y_m=ey, raw_error_m=em, applied_x_m=ax, applied_y_m=ay))
                self._rebuild_grid()
                corrected = True
                reason = 'LOOP_CLOSURE_APPLIED'
            else:
                reason = 'LOOP_CLOSURE_REJECTED_TOO_LARGE'
        self.node_events.append(dict(time_sec=now, node_id=node_id, event=reason, sample_index=idx, error_x_m=ex, error_y_m=ey, error_m=em))
        if not is_new or corrected:
            self.last_known_junction_sample_index = idx
        if bool(_map_cfg('MAP_SAVE_ON_JUNCTION', True)):
            self.save_all(rebuild=False, quiet=True)
        return dict(corrected=corrected, reason=reason, error_m=em)

    def cell_state(self, score):
        if score >= int(_map_cfg('MAP_OCCUPIED_SCORE_THRESHOLD', 4)):
            return 'OCCUPIED'
        if score <= int(_map_cfg('MAP_FREE_SCORE_THRESHOLD', -3)):
            return 'FREE'
        return 'UNKNOWN'

    def _display_sets(self):
        occupied = {c for c, s in self.grid.items() if self.cell_state(s) == 'OCCUPIED'}
        free = {c for c, s in self.grid.items() if self.cell_state(s) == 'FREE'}
        if bool(_map_cfg('MAP_DISPLAY_REMOVE_ISOLATED_WALLS', True)) and occupied:
            keep = set()
            for x, y in occupied:
                neighbours = sum(((x + dx, y + dy) in occupied for dx in (-1, 0, 1) for dy in (-1, 0, 1) if not (dx == 0 and dy == 0)))
                score = self.grid.get((x, y), 0)
                if neighbours > 0 or score >= int(_map_cfg('MAP_OCCUPIED_SCORE_THRESHOLD', 4)) + 4:
                    keep.add((x, y))
            occupied = keep
        gap = max(0, int(_map_cfg('MAP_DISPLAY_BRIDGE_GAP_CELLS', 2)))
        bridged = set(occupied)
        for x, y in list(occupied):
            for d in range(2, gap + 2):
                if (x + d, y) in occupied:
                    for k in range(1, d):
                        bridged.add((x + k, y))
                if (x, y + d) in occupied:
                    for k in range(1, d):
                        bridged.add((x, y + k))
        occupied = bridged
        radius = max(0, int(_map_cfg('MAP_DISPLAY_WALL_DILATION_CELLS', 1)))
        if radius:
            dilated = set(occupied)
            for x, y in occupied:
                for dx in range(-radius, radius + 1):
                    for dy in range(-radius, radius + 1):
                        if max(abs(dx), abs(dy)) <= radius:
                            dilated.add((x + dx, y + dy))
            occupied = dilated
        free -= occupied
        return (occupied, free)

    def _bounds(self, occupied=None, free=None):
        cells = set(self.grid)
        if occupied:
            cells |= set(occupied)
        if free:
            cells |= set(free)
        points = [(s.map_x, s.map_y) for s in self.samples]
        points += [(a['x'], a['y']) for a in self.node_anchors.values()]
        points += [self._cell_to_world(*c) for c in cells]
        if not points:
            return (-0.5, 0.5, -0.5, 0.5)
        xs, ys = ([p[0] for p in points], [p[1] for p in points])
        m = float(_map_cfg('MAP_EXPORT_MARGIN_M', 0.3))
        minx, maxx, miny, maxy = (min(xs) - m, max(xs) + m, min(ys) - m, max(ys) + m)
        if maxx - minx < 0.5:
            c = (minx + maxx) / 2
            minx, maxx = (c - 0.25, c + 0.25)
        if maxy - miny < 0.5:
            c = (miny + maxy) / 2
            miny, maxy = (c - 0.25, c + 0.25)
        return (minx, maxx, miny, maxy)

    def mark_exit(self, raw_x, raw_y, yaw_deg, heading_index=None, details=None):
        """Record a confirmed maze exit in map coordinates."""
        if not self.enabled or not self.initialized:
            return None
        sample = self.record_pose(raw_x, raw_y, yaw_deg, heading_index=heading_index, mode='EXIT_FOUND', force=True)
        if sample is None:
            return None
        self.exit_event = {'time_sec': float(sample.time_sec), 'sample_index': int(sample.index), 'map_x': float(sample.map_x), 'map_y': float(sample.map_y), 'theta_deg': float(sample.theta_deg), 'heading_index': int(sample.heading_index), 'details': dict(details or {})}
        return self.exit_event

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
            print(f'MAPPER SAVED: {self.output_dir} | samples={len(self.samples)} wall_points={len(self.wall_points)} cells={len(self.grid)} IRconfirm={self.ir_confirm_count} IRfallback={self.ir_fallback_count}')
            print(f"  -> {os.path.join(self.output_dir, 'maze_map.png')}")
            print(f"  -> {os.path.join(self.output_dir, 'maze_map.svg')}")

    def _save_csvs(self):
        with open(os.path.join(self.output_dir, 'trajectory.csv'), 'w', newline='', encoding='utf-8') as f:
            w = csv.writer(f)
            w.writerow(['index', 'time_sec', 'raw_x', 'raw_y', 'yaw_deg', 'map_x', 'map_y', 'theta_deg', 'heading_index', 'front_cm', 'left_cm', 'right_cm', 'ir', 'mode'])
            for s in self.samples:
                w.writerow([s.index, f'{s.time_sec:.6f}', f'{s.raw_x:.6f}', f'{s.raw_y:.6f}', f'{s.yaw_deg:.6f}', f'{s.map_x:.6f}', f'{s.map_y:.6f}', f'{s.theta_deg:.4f}', s.heading_index, '' if s.front_cm is None else f'{s.front_cm:.3f}', '' if s.left_cm is None else f'{s.left_cm:.3f}', '' if s.right_cm is None else f'{s.right_cm:.3f}', '' if s.ir_value is None else s.ir_value, s.mode])
        with open(os.path.join(self.output_dir, 'wall_points.csv'), 'w', newline='', encoding='utf-8') as f:
            w = csv.writer(f)
            w.writerow(['sample_index', 'time_sec', 'sensor', 'x_m', 'y_m', 'distance_cm'])
            for p in self.wall_points:
                w.writerow([p['sample_index'], f"{p['time_sec']:.6f}", p['sensor'], f"{p['x']:.6f}", f"{p['y']:.6f}", f"{p['distance_cm']:.3f}"])
        with open(os.path.join(self.output_dir, 'occupancy_grid.csv'), 'w', newline='', encoding='utf-8') as f:
            w = csv.writer(f)
            w.writerow(['gx', 'gy', 'x_m', 'y_m', 'score', 'state'])
            for (gx, gy), score in sorted(self.grid.items()):
                x, y = self._cell_to_world(gx, gy)
                w.writerow([gx, gy, f'{x:.6f}', f'{y:.6f}', score, self.cell_state(score)])

    def _save_jsons(self):
        occupied, free = self._display_sets()
        cells = []
        for c, s in sorted(self.grid.items()):
            st = self.cell_state(s)
            if st != 'UNKNOWN':
                cells.append(dict(gx=c[0], gy=c[1], score=s, state=st))
        with open(os.path.join(self.output_dir, 'occupancy_grid.json'), 'w', encoding='utf-8') as f:
            json.dump(dict(resolution_m=float(_map_cfg('MAP_RESOLUTION_M', 0.025)), cells=cells), f, indent=2)
        with open(os.path.join(self.output_dir, 'nodes.json'), 'w', encoding='utf-8') as f:
            json.dump(dict(anchors=self.node_anchors, events=self.node_events), f, indent=2)
        with open(os.path.join(self.output_dir, 'loop_closures.json'), 'w', encoding='utf-8') as f:
            json.dump(self.loop_closures, f, indent=2)
        with open(os.path.join(self.output_dir, 'exit.json'), 'w', encoding='utf-8') as f:
            json.dump(self.exit_event, f, indent=2)
        states = {'FREE': 0, 'OCCUPIED': 0, 'UNKNOWN': 0}
        for s in self.grid.values():
            states[self.cell_state(s)] += 1
        with open(os.path.join(self.output_dir, 'mapping_summary.json'), 'w', encoding='utf-8') as f:
            json.dump(dict(coordinate_convention='+Y=NORTH, +X=EAST, 0deg=NORTH +90deg=EAST', resolution_m=float(_map_cfg('MAP_RESOLUTION_M', 0.025)), samples=len(self.samples), wall_points=len(self.wall_points), grid_states=states, display_occupied_cells=len(occupied), display_free_cells=len(free), nodes=len(self.node_anchors), loop_closures=len(self.loop_closures), exit_found=self.exit_event is not None, ir_wall_level=int(_map_cfg('MAP_IR_WALL_LEVEL', 0)), ir_confirm_count=self.ir_confirm_count, ir_fallback_count=self.ir_fallback_count, yaw_fallback_count=self.yaw_fallback_count, position_rotation_deg=self.position_rotation_deg, position_auto_aligned=self.position_auto_aligned), f, indent=2)

    @staticmethod
    def _xml(text):
        return str(text).replace('&', '&amp;').replace('<', '&lt;').replace('>', '&gt;').replace('"', '&quot;')

    def _save_svg(self):
        occupied, free = self._display_sets()
        minx, maxx, miny, maxy = self._bounds(occupied, free)
        wm, hm = (maxx - minx, maxy - miny)
        ppm = float(_map_cfg('MAP_SVG_PX_PER_M', 420.0))
        W = int(clamp(wm * ppm, 600, 2200))
        H = int(clamp(hm * ppm, 600, 2200))
        sx = lambda x: (x - minx) / wm * W
        sy = lambda y: H - (y - miny) / hm * H
        res = float(_map_cfg('MAP_RESOLUTION_M', 0.025))
        cw = max(1.0, res / wm * W)
        ch = max(1.0, res / hm * H)
        p = []
        p.append('<?xml version="1.0" encoding="UTF-8"?>')
        p.append(f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">')
        p.append('<rect width="100%" height="100%" fill="#bfc3c7"/>')
        for gx, gy in free:
            x, y = self._cell_to_world(gx, gy)
            p.append(f'<rect x="{sx(x) - cw / 2:.2f}" y="{sy(y) - ch / 2:.2f}" width="{cw + 0.6:.2f}" height="{ch + 0.6:.2f}" fill="#f7f7f7"/>')
        for gx, gy in occupied:
            x, y = self._cell_to_world(gx, gy)
            p.append(f'<rect x="{sx(x) - cw / 2:.2f}" y="{sy(y) - ch / 2:.2f}" width="{cw + 0.7:.2f}" height="{ch + 0.7:.2f}" fill="#202428"/>')
        if bool(_map_cfg('MAP_DRAW_TRAJECTORY', True)) and len(self.samples) > 1:
            pts = ' '.join((f'{sx(s.map_x):.2f},{sy(s.map_y):.2f}' for s in self.samples))
            p.append(f'<polyline points="{pts}" fill="none" stroke="#2463b5" stroke-width="2.4" stroke-linecap="round" stroke-linejoin="round" opacity="0.92"/>')
        if self.samples:
            s = self.samples[0]
            p.append(f'<circle cx="{sx(s.map_x):.2f}" cy="{sy(s.map_y):.2f}" r="6" fill="#19a15f" stroke="white" stroke-width="2"/>')
            e = self.samples[-1]
            p.append(f'<circle cx="{sx(e.map_x):.2f}" cy="{sy(e.map_y):.2f}" r="5" fill="#f59e0b" stroke="white" stroke-width="1.5"/>')
        if bool(_map_cfg('MAP_DRAW_NODES', True)):
            for nid, a in sorted(self.node_anchors.items()):
                x, y = (sx(a['x']), sy(a['y']))
                p.append(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="4" fill="#d14343" stroke="white" stroke-width="1.2"/>')
                p.append(f'<text x="{x + 6:.2f}" y="{y - 6:.2f}" font-family="Arial,sans-serif" font-size="11" font-weight="700" fill="#111827">{self._xml(nid)}</text>')
        if self.exit_event is not None and bool(_map_cfg('MAP_DRAW_EXIT', True)):
            x, y = (sx(self.exit_event['map_x']), sy(self.exit_event['map_y']))
            p.append(f'<polygon points="{x:.2f},{y - 9:.2f} {x + 9:.2f},{y:.2f} {x:.2f},{y + 9:.2f} {x - 9:.2f},{y:.2f}" fill="#7c3aed" stroke="white" stroke-width="2"/>')
            p.append(f'<text x="{x + 12:.2f}" y="{y - 9:.2f}" font-family="Arial,sans-serif" font-size="12" font-weight="700" fill="#5b21b6">EXIT</text>')
        p.append('<rect x="14" y="14" width="235" height="108" rx="9" fill="white" fill-opacity="0.92" stroke="#9ca3af"/>')
        p.append('<text x="27" y="36" font-family="Arial,sans-serif" font-size="12" font-weight="700" fill="#111827">SLAM-style Maze Map</text>')
        p.append('<text x="27" y="55" font-family="Arial,sans-serif" font-size="11" fill="#374151">Black: wall  White: observed free</text>')
        p.append('<text x="27" y="72" font-family="Arial,sans-serif" font-size="11" fill="#374151">Gray: unknown  Blue: trajectory</text>')
        p.append('<text x="27" y="89" font-family="Arial,sans-serif" font-size="11" fill="#374151">Red: junction  Green: start</text>')
        p.append('<text x="27" y="106" font-family="Arial,sans-serif" font-size="11" fill="#374151">Purple diamond: confirmed exit</text>')
        p.append('</svg>')
        Path = __import__('pathlib').Path
        Path(os.path.join(self.output_dir, 'maze_map.svg')).write_text('\n'.join(p), encoding='utf-8')

    def _try_save_png(self):
        if not bool(_map_cfg('MAP_EXPORT_PNG', True)):
            return
        try:
            import matplotlib.pyplot as plt
            import numpy as np
            from matplotlib.colors import ListedColormap
        except Exception as exc:
            return
        occupied, free = self._display_sets()
        minx, maxx, miny, maxy = self._bounds(occupied, free)
        res = float(_map_cfg('MAP_RESOLUTION_M', 0.025))
        gx0 = int(math.floor(minx / res))
        gx1 = int(math.ceil(maxx / res))
        gy0 = int(math.floor(miny / res))
        gy1 = int(math.ceil(maxy / res))
        width = max(1, gx1 - gx0 + 1)
        height = max(1, gy1 - gy0 + 1)
        img = np.zeros((height, width), dtype=np.uint8)
        for gx, gy in free:
            if gx0 <= gx <= gx1 and gy0 <= gy <= gy1:
                img[gy - gy0, gx - gx0] = 1
        for gx, gy in occupied:
            if gx0 <= gx <= gx1 and gy0 <= gy <= gy1:
                img[gy - gy0, gx - gx0] = 2
        cmap = ListedColormap(['#bfc3c7', '#f8f8f8', '#171a1d'])
        fig, ax = plt.subplots(figsize=(8.5, 8.5))
        ax.imshow(img, origin='lower', extent=[gx0 * res, (gx1 + 1) * res, gy0 * res, (gy1 + 1) * res], interpolation='nearest', cmap=cmap, vmin=0, vmax=2)
        if bool(_map_cfg('MAP_DRAW_TRAJECTORY', True)) and self.samples:
            ax.plot([s.map_x for s in self.samples], [s.map_y for s in self.samples], linewidth=1.8, label='trajectory')
            ax.scatter([self.samples[0].map_x], [self.samples[0].map_y], s=45, label='start', zorder=5)
            ax.scatter([self.samples[-1].map_x], [self.samples[-1].map_y], s=35, label='current/end', zorder=5)
        if bool(_map_cfg('MAP_DRAW_NODES', True)):
            for nid, a in self.node_anchors.items():
                ax.scatter([a['x']], [a['y']], s=20, zorder=5)
                ax.text(a['x'], a['y'], ' ' + str(nid), fontsize=7, zorder=6)
        if self.exit_event is not None and bool(_map_cfg('MAP_DRAW_EXIT', True)):
            ax.scatter([self.exit_event['map_x']], [self.exit_event['map_y']], s=90, marker='*', label='exit', zorder=7)
            ax.text(self.exit_event['map_x'], self.exit_event['map_y'], ' EXIT', fontsize=8, fontweight='bold', zorder=8)
        ax.set_aspect('equal', adjustable='box')
        ax.set_xlabel('East / X (m)')
        ax.set_ylabel('North / Y (m)')
        ax.set_title('RoboMaster Maze Occupancy Map')
        ax.grid(False)
        if self.samples:
            ax.legend(loc='best', fontsize=8, framealpha=0.9)
        fig.tight_layout()
        fig.savefig(os.path.join(self.output_dir, 'maze_map.png'), dpi=int(_map_cfg('MAP_PNG_DPI', 220)))
        plt.close(fig)
try:
    from robomaster import robot
except ModuleNotFoundError as exc:
    raise SystemExit('RoboMaster SDK is not installed for this Python. Install it first, then run this file again.') from exc

# ==================== MAIN INTEGRATION ====================
def choose_v11_decision(explorer, edge_memory, scan, guide=None):
    """Update DFS memory, then allow a confident sensor-valid guide override.

    Calling plan_direction() first is intentional: even while following the
    drawing, the robot keeps building its independent Trémaux fallback graph.
    """
    decision = explorer.plan_direction(front_open=scan['front_open'], left_open=scan['left_open'], right_open=scan['right_open'])
    if guide is not None:
        guided = guide.recommend(explorer, scan)
        if guided is not None:
            return (guided, None)
    if bool(scan.get('entry_barrier_active', False)):
        # If map alignment is temporarily uncertain at the old entrance, fail
        # closed by returning inward. Never let DFS select physical FRONT and
        # leave through the entrance while an EXIT marker exists elsewhere.
        inward_abs = explorer.opposite_index(explorer.heading_index)
        safe = explorer.decision_for_absolute(
            inward_abs,
            reason='ENTRY_BARRIER_GUIDE_UNCERTAIN_RETURN',
        )
        if safe is not None:
            return (safe, None)
    latent_plan = None
    if decision.direction == 'COMPLETE' and decision.reason == 'ALL_FRONTIERS_EXPLORED' and bool(getattr(config, 'ENABLE_LATENT_FRONTIER_VERIFICATION', True)):
        latent_plan = edge_memory.plan_verification(explorer)
        if latent_plan is not None:
            candidate = latent_plan['candidate']
            start_node = latent_plan['start_node']
            if explorer.current_node_id != start_node:
                routed = explorer.route_decision_to_node(start_node, scan['front_open'], scan['left_open'], scan['right_open'], reason=f'ROUTE_TO_LATENT_{candidate.candidate_id}')
                if routed is not None:
                    print(f'>>> V11 LATENT ROUTE {candidate.candidate_id}: current={explorer.current_node_id} -> start={start_node} conf={candidate.confidence:.2f}')
                    return (routed, latent_plan)
            else:
                abs_dir = latent_plan.get('edge_abs_dir')
                if abs_dir is not None:
                    verify = explorer.decision_for_absolute(abs_dir, reason=f'VERIFY_LATENT_{candidate.candidate_id}')
                    if verify is not None:
                        edge_memory.arm_verification(candidate.candidate_id, start_node=start_node, expected_to=latent_plan.get('other_node'))
                        print(f">>> V11 VERIFY {candidate.candidate_id}: edge={start_node}->{latent_plan.get('other_node') or '?'} branch={explorer.heading_name(candidate.branch_abs_dir)} conf={candidate.confidence:.2f}")
                        return (verify, latent_plan)
    return (decision, latent_plan)

def save_v11_memory(explorer, edge_memory):
    if getattr(config, 'SAVE_MAZE_MEMORY', False):
        explorer.save_memory()
    if getattr(config, 'SAVE_EDGE_MEMORY', True):
        edge_memory.save()

def maze_main(guide_path=None, disable_guide=False):
    ep_robot = robot.Robot()
    chassis = None
    tof_sensor = None
    pose_subscribed = False
    attitude_subscribed = False
    tof_subscribed = False
    mapper = None
    guide = None
    try:
        print('Connecting RoboMaster...')
        ep_robot.initialize(conn_type='ap')
        raw_chassis = ep_robot.chassis
        sensor_adapter = ep_robot.sensor_adaptor
        tof_sensor = ep_robot.sensor
        sensors = SensorManager(sensor_adapter)
        chassis = SafeChassisProxy(raw_chassis, sensors)
        controller = MotionController()
        pose_tracker = PoseTracker()
        detector = DecisionPointDetector()
        explorer = TremauxExplorer()
        edge_memory = EdgeTraversalMemory()
        open_area_exit = OpenAreaExitManager()
        mapper = SLAMStyleMazeMapper()
        configured_guide_path = guide_path
        if configured_guide_path is None:
            configured_guide_path = str(getattr(config, 'PREDRAWN_GUIDE_FILE', 'known_route.json'))
        if bool(getattr(config, 'ENABLE_PREDRAWN_TOPOLOGY_GUIDE', True)) and (not disable_guide):
            if configured_guide_path and os.path.isfile(configured_guide_path):
                try:
                    guide = TopologicalMazeGuide.load(configured_guide_path)
                    print(f'>>> MAP GUIDE loaded: {configured_guide_path} [{getattr(guide, "load_mode", "topology")}]')
                except Exception as guide_exc:
                    print(f'>>> MAP GUIDE disabled: {guide_exc}')
            elif guide_path is not None:
                print(f'>>> MAP GUIDE file not found: {configured_guide_path}; continuing with DFS')
            else:
                print(f'>>> MAP GUIDE not found ({configured_guide_path}); continuing with DFS')
        fsm_state = str(getattr(config, 'FSM_INITIAL_STATE', 'EDGE_TRAVERSE'))
        node_lock_id = None
        tof_subscribed = tof_sensor.sub_distance(freq=20, callback=sensors.tof_callback)
        pose_subscribed = chassis.sub_position(cs=1, freq=config.POSE_FREQ_HZ, callback=pose_tracker.position_callback)
        attitude_subscribed = chassis.sub_attitude(freq=config.ATTITUDE_FREQ_HZ, callback=pose_tracker.attitude_callback)
        print_startup_info()
        if bool(getattr(config, 'FRONT_IR_AUTO_LEARN_CLEAR_LEVEL', False)):
            print('>>> Keep both front IR sensors clear for startup baseline...')
            sensors.calibrate_front_ir_clear_levels()
        else:
            print(f'>>> FRONT IR FIXED DIGITAL MODE: ID{config.IR_FRONT_LEFT_ID}/ID{config.IR_FRONT_RIGHT_ID} port={config.SENSOR_PORT} CLEAR=1 HIT=0 via get_io()')
        start_x, start_y = wait_for_pose(pose_tracker)
        start_yaw = wait_for_yaw(pose_tracker)
        controller.initialize_heading(start_yaw, pose_tracker=pose_tracker)
        start_node = explorer.initialize_start(start_x, start_y)
        if guide is not None:
            guide.initialize(start_node)
            guide.commit_departure(start_node, explorer.heading_index, reason='INITIAL_FRONT')
        explorer.commit_initial_forward()
        edge_memory.begin_from_explorer(explorer)
        node_lock_id = start_node
        start_gate = StartGateGuard(start_x, start_y, inside_abs_dir=explorer.start_inside_abs_dir if explorer.start_inside_abs_dir is not None else explorer.heading_index)
        print(f'>>> START_GATE armed: inside={explorer.heading_name(start_gate.inside_abs_dir)} outside={explorer.heading_name(start_gate.outside_abs_dir)}')
        if mapper is not None and config.ENABLE_MAPPING:
            mapper.initialize(start_x, start_y, start_yaw, heading_index=explorer.heading_index)
            mapper.observe_junction(start_node, True, start_x, start_y, start_yaw, heading_index=explorer.heading_index)
        print(f'START NODE: {start_node} at ({start_x:+.2f}, {start_y:+.2f}) m')
        if bool(getattr(config, 'ENABLE_START_ENTRY_ACQUISITION', True)):
            print(
                f'Initial action: nudge FRONT '
                f'{getattr(config, "START_ENTRY_NUDGE_DISTANCE_M", 0.05):.2f}m, '
                f'then acquire entrance from ToF / Sharp-L / Sharp-R / IR'
            )
        else:
            print('Initial action: explore FRONT')
        if controller.heading_target_yaw is not None:
            print(f'Heading grid N      : {controller.heading_target_yaw:+.1f} deg')
        print()
        save_v11_memory(explorer, edge_memory)
        while True:
            raw_adc_l, sharp_left_cm = sensors.read_left_sharp()
            raw_adc_r, sharp_right_cm = sensors.read_right_sharp()
            ir_front_left, ir_front_right = sensors.read_front_ir_pair()
            front_ir_state = sensors.update_front_ir_guard(ir_front_left, ir_front_right)
            front_cm = sensors.get_front_cm()
            nav_front_cm = sensors.effective_front_cm(front_cm, front_ir_state['blocked'])
            if sharp_left_cm is None or sharp_right_cm is None:
                stop_chassis(chassis)
                controller.reset_side_owner()
                detector.cancel_event()
                open_area_exit.cancel_exit_candidate('SHARP_SENSOR_MISSING')
                missing = []
                if sharp_left_cm is None:
                    missing.append('LEFT')
                if sharp_right_cm is None:
                    missing.append('RIGHT')
                print('>>> SHARP SENSOR HOLD: missing=' + ','.join(missing) + ' | robot stopped; waiting for sensor recovery')
                time.sleep(config.SHARP_SENSOR_RECOVERY_DELAY_SEC)
                continue
            pose_x, pose_y, _ = pose_tracker.get_pose()
            # Do not let the wall-free staging area create graph branches before
            # the physical maze entrance has been acquired.
            edge_event = None
            if start_gate.entry_acquired:
                edge_event = edge_memory.observe(
                    pose_x, pose_y, sharp_left_cm, sharp_right_cm,
                    front_cm=nav_front_cm,
                )
            edge_progress = edge_memory.progress_m(pose_x, pose_y)
            if edge_event is not None and edge_memory.merge_departure_echo(edge_event, explorer, edge_progress):
                print(f">>> EDGE ECHO MERGED INTO DEPARTURE NODE {(edge_memory.current.from_node if edge_memory.current else '-')} progress={edge_progress or 0.0:.2f}m")
                edge_event = None
            if node_lock_id is not None and edge_progress is not None and (edge_progress >= float(getattr(config, 'NODE_LOCK_RELEASE_PROGRESS_M', 0.3))):
                node_lock_id = None
            x = 0.0
            y = 0.0
            z = 0.0
            mode = 'STOP'
            heading_error = controller.heading_error(pose_tracker.get_yaw())
            front_blocked_now = nav_front_cm is not None and 0.0 < nav_front_cm <= config.STOP_FRONT_CM
            start_gate.observe(pose_x, pose_y)
            entry_was_acquired = start_gate.entry_acquired
            entry_state = start_gate.update_entry_acquisition(
                sharp_left_cm, sharp_right_cm, front_cm, pose_x, pose_y,
                front_ir_state=front_ir_state,
            )

            if entry_state.get('state') == 'FAILED':
                stop_chassis(chassis)
                print()
                print('============================================')
                print(' START ENTRY ACQUISITION FAILED SAFELY')
                print(' No stable maze wall or front landmark was found.')
                print(' Check the initial heading and entrance distance.')
                print('============================================')
                break

            if not entry_was_acquired and start_gate.entry_acquired:
                # Throw away every opening sample collected in the staging area.
                # The next loop begins with a clean detector inside the entrance.
                detector.cancel_event()
                open_area_exit.cancel_exit_candidate('START_ENTRY_ACQUIRED')
                controller.reset_side_owner()
                sensors.reset_filters()
                fsm_state = 'START_ENTRY_ACQUIRED'
                time.sleep(config.LOOP_DELAY_SEC)
                continue

            if not start_gate.entry_acquired:
                detector.cancel_event()
                open_area_exit.cancel_exit_candidate('START_ENTRY_SEARCH')
                controller.reset_side_owner()
                entry_phase = str(entry_state.get('phase', 'SENSOR_SEARCH'))
                if entry_phase == 'NUDGE':
                    # Deliberately ignore ordinary ToF/Sharp steering only for
                    # this very short creep.  Raw/confirmed IR and the emergency
                    # ToF threshold still stop the nudge.
                    x = float(getattr(config, 'START_ENTRY_NUDGE_SPEED', 0.06))
                    y = 0.0
                    z = 0.0
                    entry_mode = 'START_ENTRY_NUDGE_FRONT'
                    emergency_cm = float(
                        getattr(config, 'START_ENTRY_NUDGE_EMERGENCY_FRONT_CM', 6.0)
                    )
                    if (
                        front_cm is not None
                        and 0.0 < float(front_cm) <= emergency_cm
                    ):
                        x = 0.0
                        entry_mode += '_TOF_EMERGENCY_STOP'
                else:
                    x = min(
                        controller.calculate_forward_speed(nav_front_cm),
                        float(config.START_ENTRY_SEARCH_SPEED),
                    )
                    if front_blocked_now:
                        x = 0.0
                    y, z, sensor_mode = controller.calculate_motion_control(
                        raw_adc_l, sharp_left_cm, raw_adc_r, sharp_right_cm,
                        front_ir_state['blocked'],
                    )
                    entry_mode = 'START_ENTRY_' + sensor_mode
                    x, y, z, entry_mode = apply_motion_safety(
                        x, y, z, entry_mode,
                        front_ir_blocked=front_ir_state['blocked'],
                    )
                x, y, z, entry_mode, _ = controller.apply_heading_hold(
                    x, y, pose_tracker.get_yaw(), pose_tracker, entry_mode,
                )
                x, y, z, entry_mode = apply_front_ir_collision_avoidance(
                    x, y, z, entry_mode, front_ir_state,
                    sharp_left_cm, sharp_right_cm,
                )
                if mapper is not None and config.ENABLE_MAPPING:
                    map_x_raw, map_y_raw, map_yaw = pose_tracker.get_pose()
                    mapper.update(
                        map_x_raw, map_y_raw, map_yaw,
                        front_cm=front_cm,
                        left_cm=sharp_left_cm,
                        right_cm=sharp_right_cm,
                        ir_value=None,
                        heading_index=explorer.heading_index,
                        mode=entry_mode,
                        map_ranges=True,
                    )
                if config.ENABLE_MOTION:
                    chassis.drive_speed(x=x, y=y, z=z, timeout=config.DRIVE_TIMEOUT_SEC)
                entry_metrics = start_gate.metrics(pose_x, pose_y)
                print(
                    f'START_ENTRY SEARCH travel='
                    f'{(entry_metrics.get("distance_m") if entry_metrics.get("distance_m") is not None else 0.0):.2f}m '
                    f'F={fmt(nav_front_cm)} L={fmt(sharp_left_cm)} R={fmt(sharp_right_cm)} '
                    f'IR={int(front_ir_state.get("left_confirmed", False))}/'
                    f'{int(front_ir_state.get("right_confirmed", False))} '
                    f'phase={entry_phase} '
                    f'cmd=({x:.3f},{y:+.3f},{z:+.1f}) mode={entry_mode}'
                )
                time.sleep(config.LOOP_DELAY_SEC)
                continue

            start_gate_block_exit = start_gate.should_reject_exit(pose_x, pose_y, explorer.heading_index)
            hard_start_gate_due = start_gate.should_force_return(
                pose_x, pose_y, explorer.heading_index,
            )
            start_gate_forced_decision = False
            if (
                start_gate.should_force_junction_decision(
                    pose_x, pose_y, explorer.heading_index,
                )
                and detector.intersection_window.get('active', False)
            ):
                minimum_window = 0.0 if hard_start_gate_due else float(
                    getattr(config, 'START_GATE_DECISION_MIN_WINDOW_M', 0.08)
                )
                barrier_event = detector.force_finalize_active_intersection(
                    pose_x,
                    pose_y,
                    reason='START_GATE_VIRTUAL_BARRIER',
                    minimum_length_m=minimum_window,
                )
                if barrier_event is not None:
                    stop_chassis(chassis)
                    start_gate_forced_decision = True
                    fsm_state = 'NODE_PROCESS_START_GATE_BARRIER'
                    print(
                        '>>> START GATE VIRTUAL BARRIER: active junction '
                        'finalized before old entrance'
                    )
            if hard_start_gate_due and (not start_gate_forced_decision):
                metrics = start_gate.metrics(pose_x, pose_y)
                print()
                print('============================================')
                print(' START GATE GUARD - RETURNING INTO MAZE')
                print(f" progress={metrics.get('progress_m', 0.0):+.3f}m lateral={metrics.get('lateral_m', 0.0):.3f}m heading={explorer.heading_name()}")
                print('============================================')
                stop_chassis(chassis)
                detector.cancel_event()
                open_area_exit.cancel_exit_candidate('START_GATE_FORCE_RETURN')
                controller.reset_side_owner()
                node_id, _ = explorer.arrive_at_decision_point(start_x, start_y)
                edge_memory.finish_at_node(node_id, start_x, start_y, promoted_node=None)
                inside_abs = explorer.start_inside_abs_dir if explorer.start_inside_abs_dir is not None else start_gate.inside_abs_dir
                relative = explorer.relative_for_absolute(inside_abs)
                inside_state = explorer._exit(node_id, inside_abs)
                guard_decision = ExplorationDecision(direction=relative, node_id=node_id, reason='START_GATE_RETURN_TO_MAZE', visits_before=inside_state.visits, absolute_heading=explorer.heading_name(inside_abs))
                print_exploration_decision(guard_decision)
                turn_ok = execute_turn(chassis, decision_from_relative(relative), pose_tracker=pose_tracker, sensors=sensors)
                if not turn_ok:
                    stop_chassis(chassis)
                    print('START_GATE recovery turn failed safely; stopping.')
                    break
                explorer.commit_decision(guard_decision)
                edge_memory.begin_from_explorer(explorer)
                fsm_state = 'EDGE_TRAVERSE'
                node_lock_id = node_id
                controller.set_heading_index(explorer.heading_index, pose_tracker=pose_tracker)
                align_heading_in_place(chassis, controller, pose_tracker)
                controller.reset_after_turn()
                sensors.reset_filters()
                start_gate.mark_recovery()
                rx, ry, _ = pose_tracker.get_pose()
                detector.force_latched(rx if rx is not None else start_x, ry if ry is not None else start_y)
                if mapper is not None and config.ENABLE_MAPPING:
                    mx, my, myaw = pose_tracker.get_pose()
                    mapper.update(mx, my, myaw, front_cm=None, left_cm=None, right_cm=None, ir_value=None, heading_index=explorer.heading_index, mode='START_GATE_RETURN', map_ranges=False, force=True)
                save_v11_memory(explorer, edge_memory)
                time.sleep(config.AFTER_TURN_DELAY_SEC)
                continue
            open_state = open_area_exit.update(nav_front_cm, sharp_left_cm, sharp_right_cm, pose_x, pose_y, node_count=len(explorer.nodes), heading_error=heading_error, start_gate_block_exit=start_gate_block_exit)
            active_mission_target = guide.active_target() if guide is not None else None
            mission_exit_allowed = (
                guide is None
                or bool(guide.physical_exit_allowed())
            )
            if not mission_exit_allowed:
                open_area_exit.cancel_exit_candidate(
                    f'MAP_EXIT_LOCK_{active_mission_target or "NO_TARGET"}'
                )
                open_state['exit_candidate_active'] = False
                open_state['exit_ready'] = False
                open_state['exit_found'] = False
            decision_event = start_gate_forced_decision
            if not decision_event:
                decision_event = detector.update(
                    nav_front_cm, sharp_left_cm, sharp_right_cm,
                    pose_x=pose_x, pose_y=pose_y,
                )
            forced_edge_zone = None
            if not decision_event and edge_event is not None:
                normal_tracker_active = bool(detector.intersection_window.get('active', False) or detector.left_zone.get('active', False) or detector.right_zone.get('active', False))
                if detector.latched or not normal_tracker_active:
                    decision_event = True
                    forced_edge_zone = edge_event
                    fsm_state = 'NODE_PROCESS_EDGE_INTERRUPT'
                    print(f'>>> V11 EDGE INTERRUPT: strong side opening observed while detector_latch={int(detector.latched)}')
            if decision_event and open_state['exit_candidate_active']:
                open_area_exit.cancel_exit_candidate('JUNCTION_DETECTED')
                open_state['exit_candidate_active'] = False
                open_state['exit_ready'] = False

            # V12.5 EXIT PROOF: commit only after the decision detector and the
            # independent edge observer both had a chance to veto this cycle.
            if mission_exit_allowed and (not decision_event) and bool(open_state.get('exit_ready', False)):
                if open_area_exit.confirm_exit():
                    open_state['exit_found'] = True
                    open_state['exit_event'] = open_area_exit.exit_event
                    stop_chassis(chassis)
                    if mapper is not None and config.ENABLE_MAPPING:
                        mx, my, myaw = pose_tracker.get_pose()
                        mapper.update(mx, my, myaw, front_cm=front_cm, left_cm=sharp_left_cm, right_cm=sharp_right_cm, ir_value=None, heading_index=explorer.heading_index, mode='EXIT_FOUND', map_ranges=True, force=True)
                        mapper.mark_exit(mx, my, myaw, heading_index=explorer.heading_index, details=open_area_exit.exit_event)
                        mapper.save_all(rebuild=True, quiet=False)
                    save_v11_memory(explorer, edge_memory)
                    print()
                    print('============================================')
                    print(' MAZE EXIT FOUND - V12.5 EXIT PROOF PASSED')
                    print('============================================')
                    if config.STOP_WHEN_EXIT_FOUND:
                        break

            if decision_event:
                if mapper is not None and config.ENABLE_MAPPING:
                    mapper.update(pose_x, pose_y, pose_tracker.get_yaw(), front_cm=front_cm, left_cm=sharp_left_cm, right_cm=sharp_right_cm, ir_value=None, heading_index=explorer.heading_index, mode='DECISION_TRIGGER', map_ranges=True, force=True)
                mode = 'DFS_DECISION'
                zone_event = forced_edge_zone if forced_edge_zone is not None else detector.consume_pending_zone()

                # ------------------------------------------------------------
                # V12 FAST PATH: use the completed Intersection Window directly.
                # No physical centering and no stopped 3-sample scan are needed
                # when accumulated evidence is strong enough.
                # ------------------------------------------------------------
                scan = build_rolling_decision_scan(
                    detector, sensors, zone_event,
                    front_cm, sharp_left_cm, sharp_right_cm, front_ir_state,
                )
                rolling_mode = scan is not None
                actual_pose_x, actual_pose_y, _ = pose_tracker.get_pose()
                if actual_pose_x is None or actual_pose_y is None:
                    actual_pose_x, actual_pose_y = pose_x, pose_y

                if rolling_mode:
                    node_pose_x, node_pose_y = estimate_rolling_junction_anchor(
                        zone_event, actual_pose_x, actual_pose_y
                    )
                    if node_pose_x is None or node_pose_y is None:
                        node_pose_x, node_pose_y = actual_pose_x, actual_pose_y
                    if bool(getattr(config, 'ROLLING_DEBUG', True)):
                        print(
                            f'>>> V12 ROLLING NODE anchor=({node_pose_x:+.2f},{node_pose_y:+.2f}) '
                            f'robot=({actual_pose_x:+.2f},{actual_pose_y:+.2f})'
                        )
                else:
                    # Conservative V11 fallback for ambiguous/one-sided evidence.
                    controller.reset_side_owner()
                    stop_chassis(chassis)
                    if zone_event is not None:
                        center_on_opening_zone(chassis, controller, pose_tracker, zone_event)
                    else:
                        pre_front_open, _, pre_left_open, pre_right_open = detector.classify_openings(nav_front_cm, sharp_left_cm, sharp_right_cm)
                        creep_to_junction_center(chassis, sensors, controller, pose_tracker, pre_front_open, pre_left_open, pre_right_open)
                    scan = scan_decision_point(detector, sensors, zone_event)
                    if scan['front_cm'] is None or scan['left_cm'] is None or scan['right_cm'] is None:
                        print('Decision rejected: incomplete sensor data.')
                        detector.cancel_event()
                        stop_chassis(chassis)
                        time.sleep(config.LOOP_DELAY_SEC)
                        continue
                    if scan['front_open'] and (not scan['left_open']) and (not scan['right_open']):
                        print('Decision rejected: normal corridor after re-scan.')
                        detector.cancel_event()
                        time.sleep(config.LOOP_DELAY_SEC)
                        continue
                    node_pose_x, node_pose_y, _ = pose_tracker.get_pose()
                    if node_pose_x is None or node_pose_y is None:
                        current = explorer.nodes.get(explorer.current_node_id)
                        if current is not None:
                            node_pose_x, node_pose_y = (current.x, current.y)
                        else:
                            node_pose_x, node_pose_y = (0.0, 0.0)
                    if mapper is not None and config.ENABLE_MAPPING:
                        mapper.update(node_pose_x, node_pose_y, pose_tracker.get_yaw(), front_cm=scan['front_cm'], left_cm=scan['left_cm'], right_cm=scan['right_cm'], ir_value=None, heading_index=explorer.heading_index, mode='JUNCTION_SCAN', map_ranges=True, force=True)

                if start_gate_forced_decision:
                    scan['entry_barrier_active'] = True
                    scan['forbidden_abs'] = {explorer.heading_index % 4}
                    print(
                        f'>>> ENTRY BARRIER forbids FRONT/'
                        f'{explorer.heading_name(explorer.heading_index)} '
                        'through the old entrance'
                    )

                observed_abs = {explorer.opposite_index(explorer.heading_index)}
                if scan.get('front_open'):
                    observed_abs.add(explorer.heading_index % 4)
                if scan.get('left_open'):
                    observed_abs.add((explorer.heading_index - 1) % 4)
                if scan.get('right_open'):
                    observed_abs.add((explorer.heading_index + 1) % 4)
                node_id, is_new = explorer.arrive_at_decision_point(node_pose_x, node_pose_y, observed_abs=observed_abs)
                edge_memory.finish_at_node(node_id, node_pose_x, node_pose_y, promoted_node=None)
                confirmed_branch_abs = set()
                if scan.get('front_open'):
                    confirmed_branch_abs.add(explorer.heading_index % 4)
                if scan.get('left_open'):
                    confirmed_branch_abs.add((explorer.heading_index - 1) % 4)
                if scan.get('right_open'):
                    confirmed_branch_abs.add((explorer.heading_index + 1) % 4)
                edge_memory.promote_near_position(node_pose_x, node_pose_y, node_id, allowed_branch_abs=confirmed_branch_abs)
                edge_memory.reanchor_candidates_to_graph(explorer)
                fsm_state = 'NODE_PROCESS_ROLLING' if rolling_mode else 'NODE_PROCESS'
                node_lock_id = node_id
                if mapper is not None and config.ENABLE_MAPPING:
                    map_event = mapper.observe_junction(node_id, is_new, node_pose_x, node_pose_y, pose_tracker.get_yaw(), heading_index=explorer.heading_index)
                    if map_event and map_event.get('corrected'):
                        print(f"MAP LOOP CLOSURE: {node_id} error={map_event.get('error_m', 0.0):.3f} m")
                print()
                print(f"[{'ROLLING ' if rolling_mode else ''}{'NEW' if is_new else 'KNOWN'} NODE] {node_id} at ({node_pose_x:+.2f}, {node_pose_y:+.2f}) m")
                print('Memory:', explorer.describe_node(node_id))
                print('DFS Stack:', ' -> '.join(explorer.dfs_stack))
                if guide is not None:
                    guide.observe_node(node_id, observed_abs)
                    mission_event = guide.consume_mission_event()
                    if mission_event is not None:
                        marker = str(mission_event.get('marker', '')).upper()
                        if marker in ('PICKUP', 'DROP'):
                            stop_chassis(chassis)
                            print()
                            print('============================================')
                            print(f' MISSION MARKER REACHED: {marker}')
                            print(
                                f' Stage order preserved; next target: '
                                f'{guide.active_target() or "COMPLETE"}'
                            )
                            print('============================================')
                            time.sleep(float(getattr(config, 'GUIDE_MISSION_MARKER_DWELL_SEC', 1.0)))
                exploration_decision, latent_plan = choose_v11_decision(explorer, edge_memory, scan, guide=guide)
                print_exploration_decision(exploration_decision)
                print('Latent Memory:', edge_memory.describe())
                if exploration_decision.direction == 'COMPLETE':
                    stop_chassis(chassis)
                    explorer.commit_decision(exploration_decision)
                    save_v11_memory(explorer, edge_memory)
                    print()
                    print('============================================')
                    print(' EXPLORATION COMPLETE - HARD + LATENT WORK DONE')
                    print('============================================')
                    if config.STOP_WHEN_EXPLORATION_COMPLETE:
                        break
                    detector.cancel_event()
                    continue

                # ------------------------------------------------------------
                # V12 DRIVE-THROUGH FRONT: commit the graph edge and refresh a
                # forward command. No stop, no reverse-to-centre, no turn align.
                # ------------------------------------------------------------
                if rolling_mode and exploration_decision.direction == 'FRONT':
                    if guide is not None:
                        guide.commit_departure(
                            node_id,
                            explorer.absolute_index(exploration_decision.direction),
                            reason=exploration_decision.reason,
                        )
                    explorer.commit_decision(exploration_decision)
                    edge_memory.begin_from_explorer(explorer)
                    fsm_state = 'EDGE_TRAVERSE_ROLLING_FRONT'
                    controller.set_heading_index(explorer.heading_index, pose_tracker=pose_tracker)
                    command_rolling_front(
                        chassis, controller, sensors, pose_tracker,
                        scan.get('nav_front_cm'), front_ir_state,
                    )
                    print(f'>>> V12 DRIVE-THROUGH {node_id}: FRONT selected, no stopped scan/backtrack')
                    detector.release_after_front_drive_through(zone_event)
                    print('Updated Memory:', explorer.describe_node(node_id))
                    if bool(getattr(config, 'ROLLING_SAVE_MEMORY_ON_FRONT', False)):
                        save_v11_memory(explorer, edge_memory)
                    continue

                # LEFT/RIGHT/BACK deliberately stop only after planning.
                controller.reset_side_owner()
                stop_chassis(chassis)
                turn_decision = decision_from_relative(exploration_decision.direction)
                raw_side_open = True
                if exploration_decision.direction == 'LEFT':
                    raw_side_open = bool(scan.get('raw_left_open', False))
                elif exploration_decision.direction == 'RIGHT':
                    raw_side_open = bool(scan.get('raw_right_open', False))
                max_realign = None
                if rolling_mode and exploration_decision.direction in ('LEFT', 'RIGHT'):
                    # V12.2: only a chosen side turn pays the cost of returning
                    # to the physical opening centre. FRONT drive-through remains fast.
                    if bool(getattr(config, 'TURN_POCKET_CENTER_ROLLING_OPENING', True)) and zone_event is not None:
                        center_on_opening_zone(chassis, controller, pose_tracker, zone_event)
                    max_realign = rolling_turn_backtrack_limit(zone_event)
                rolling_search_speed = None
                rolling_search_sec = None
                if rolling_mode and exploration_decision.direction in ('LEFT', 'RIGHT'):
                    rolling_search_speed = float(getattr(config, 'ROLLING_TURN_REALIGN_SPEED', 0.08))
                    rolling_search_sec = float(getattr(config, 'ROLLING_TURN_REALIGN_MAX_SEC', 5.0))
                entry_ok = align_to_selected_side_opening(
                    chassis, sensors, controller, pose_tracker,
                    exploration_decision.direction, raw_side_open,
                    max_backtrack_m=max_realign,
                    search_speed=rolling_search_speed,
                    max_search_sec=rolling_search_sec,
                )
                if not entry_ok:
                    print('>>> TURN CANCELLED: selected opening not aligned with chassis')
                    detector.cancel_event()
                    controller.reset_corridor_heading_calibration()
                    stop_chassis(chassis)
                    time.sleep(config.AFTER_TURN_DELAY_SEC)
                    continue
                corner_turn_setup(chassis, sensors, controller, pose_tracker, exploration_decision.direction, scan['front_open'])
                turn_ok = execute_turn(chassis, turn_decision, pose_tracker=pose_tracker, sensors=sensors)
                if not turn_ok:
                    stop_chassis(chassis)
                    print()
                    print('============================================')
                    print(' TURN FAILED SAFELY - MAP EDGE NOT COMMITTED')
                    print(' Check yaw / chassis communication, then retry.')
                    print('============================================')
                    if config.SAVE_MAZE_MEMORY:
                        explorer.save_memory()
                    break
                if guide is not None:
                    guide.commit_departure(
                        node_id,
                        explorer.absolute_index(exploration_decision.direction),
                        reason=exploration_decision.reason,
                    )
                explorer.commit_decision(exploration_decision)
                edge_memory.begin_from_explorer(explorer)
                fsm_state = 'VERIFY_LATENT' if str(exploration_decision.reason).startswith('VERIFY_LATENT_') else 'EDGE_TRAVERSE'
                controller.set_heading_index(explorer.heading_index, pose_tracker=pose_tracker)
                align_heading_in_place(chassis, controller, pose_tracker)
                if mapper is not None and config.ENABLE_MAPPING:
                    tx, ty, tyaw = pose_tracker.get_pose()
                    mapper.record_pose(tx, ty, tyaw, heading_index=explorer.heading_index, mode='AFTER_TURN', force=True)
                controller.reset_after_turn()
                post_turn_clearance(chassis, sensors, controller, pose_tracker, exploration_decision.direction)
                post_turn_corridor_acquire(chassis, sensors, controller, pose_tracker, exploration_decision.direction)
                print('Updated Memory:', explorer.describe_node(node_id))
                print(f'New heading: {explorer.heading_name()}')
                save_v11_memory(explorer, edge_memory)
                sensors.reset_filters()
                latch_x, latch_y = _pose_xy(pose_tracker)
                detector.force_latched(latch_x if latch_x is not None else node_pose_x, latch_y if latch_y is not None else node_pose_y)
                stop_chassis(chassis)
                time.sleep(config.AFTER_TURN_DELAY_SEC)
                continue
            if front_blocked_now:
                controller.reset_side_owner()
                x = 0.0
                y = 0.0
                z = 0.0
                mode = 'FRONT_CONFIRM'
            else:
                x = controller.calculate_forward_speed(nav_front_cm)
                verify_cap = edge_memory.verification_speed_limit(pose_x, pose_y)
                if verify_cap is not None:
                    x = min(x, verify_cap)
                    mode = 'VERIFY_LATENT_APPROACH'
                side_danger = sharp_left_cm is not None and sharp_left_cm <= config.SIDE_TOO_CLOSE_CM or (sharp_right_cm is not None and sharp_right_cm <= config.SIDE_TOO_CLOSE_CM)
                if config.ENABLE_OPEN_AREA_HEADING_HOLD and open_state['open_area_active'] and (not side_danger):
                    controller.reset_side_owner()
                    y = 0.0
                    z = 0.0
                    if open_state['exit_candidate_active']:
                        x = min(x, config.EXIT_CANDIDATE_SPEED)
                        mode = 'EXIT_CANDIDATE_HEADING_HOLD'
                    else:
                        mode = 'OPEN_AREA_HEADING_HOLD'
                else:
                    y, z, mode = controller.calculate_motion_control(raw_adc_l, sharp_left_cm, raw_adc_r, sharp_right_cm, front_ir_state['blocked'])
                if nav_front_cm is not None and config.STOP_FRONT_CM < nav_front_cm < config.SLOW_FRONT_CM:
                    mode = 'SLOW_' + mode
                if bool(getattr(config, 'ENABLE_ADAPTIVE_JUNCTION_REGION', False)) and detector.intersection_window.get('active', False):
                    x = min(x, float(getattr(config, 'JUNCTION_REGION_OBSERVE_SPEED', 0.12)))
                    mode = 'JUNCTION_REGION_' + mode
                x, y, z, mode = apply_motion_safety(x, y, z, mode, front_ir_blocked=front_ir_state['blocked'])
                corridor_cal_allowed = not side_danger and (not front_ir_state.get('left_confirmed', False)) and (not front_ir_state.get('right_confirmed', False)) and (not open_state['open_area_active']) and (not detector.latched) and (not detector.intersection_window.get('active', False)) and (not detector.left_zone.get('active', False)) and (not detector.right_zone.get('active', False)) and (mode not in ('HEADING_RECOVER',))
                controller.update_corridor_heading_reference(sharp_left_cm, sharp_right_cm, front_cm, pose_tracker, explorer.heading_index, allow=corridor_cal_allowed)
                x, y, z, mode, heading_error = controller.apply_heading_hold(x, y, pose_tracker.get_yaw(), pose_tracker, mode)
                x, y, z, mode = apply_front_ir_collision_avoidance(x, y, z, mode, front_ir_state, sharp_left_cm, sharp_right_cm)
                x, y, z, mode = apply_motion_safety(x, y, z, mode, front_ir_blocked=front_ir_state['blocked'])
            if mapper is not None and config.ENABLE_MAPPING:
                map_x_raw, map_y_raw, map_yaw = pose_tracker.get_pose()
                mapper.update(map_x_raw, map_y_raw, map_yaw, front_cm=front_cm, left_cm=sharp_left_cm, right_cm=sharp_right_cm, ir_value=None, heading_index=explorer.heading_index, mode=mode, map_ranges=True)
            if config.ENABLE_MOTION:
                chassis.drive_speed(x=x, y=y, z=z, timeout=config.DRIVE_TIMEOUT_SEC)
            if sharp_left_cm is not None and sharp_right_cm is not None:
                delta = sharp_left_cm - sharp_right_cm
            else:
                delta = 0.0
            pose_x, pose_y, yaw_deg = pose_tracker.get_pose()
            pose_text = f'({pose_x:+.2f},{pose_y:+.2f})' if pose_x is not None and pose_y is not None else '(---,---)'
            yaw_text = f'{yaw_deg:+6.1f}' if yaw_deg is not None else '  --- '
            target_yaw = controller.heading_target_yaw
            target_text = f'{target_yaw:+6.1f}' if target_yaw is not None else '  --- '
            current_heading_error = controller.heading_error(yaw_deg)
            heading_error_text = f'{current_heading_error:+5.1f}' if current_heading_error is not None else '  ---'
            ir_l_text = str(ir_front_left) if ir_front_left is not None else '-'
            ir_r_text = str(ir_front_right) if ir_front_right is not None else '-'
            if edge_memory.current is not None:
                edge_text = f"{edge_memory.current.from_node}->{edge_memory.current.expected_to or '?'}@{(edge_progress if edge_progress is not None else 0.0):.2f}"
            else:
                edge_text = '-'
            collision_text = getattr(chassis, 'last_veto', None) or 'CLEAR'
            cmd_x, cmd_y, cmd_z = getattr(chassis, 'last_command', (x, y, z))
            print(f"ToF:{fmt(front_cm)}cm NavF:{fmt(nav_front_cm)} | L:{fmt(sharp_left_cm)} ADC:{fmt_adc(raw_adc_l)} | R:{fmt(sharp_right_cm)} ADC:{fmt_adc(raw_adc_r)} | IRF:{ir_l_text}/{ir_r_text} HIT:{int(front_ir_state.get('left_hit', False))}/{int(front_ir_state.get('right_hit', False))} C:{int(front_ir_state.get('left_confirmed', False))}/{int(front_ir_state.get('right_confirmed', False))} B:{int(front_ir_state['blocked'])} | D:{delta:+5.1f} | POSE:{pose_text} | YAW:{yaw_text}/{target_text} E:{heading_error_text} | H:{explorer.heading_name()} | OWNER:{controller.side_owner:5s} | OA:{int(open_state['open_area_active'])} | EXITC:{int(open_state['exit_candidate_active'])} | FSM:{fsm_state} | NODELOCK:{node_lock_id or '-'} | DL:{int(detector.latched)} | EDGE:{edge_text} | LAT:{len(edge_memory.pending_candidates())} | COL:{collision_text} | {mode:28s} | req=({x:.3f},{y:+.3f},{z:+.1f}) act=({cmd_x:.3f},{cmd_y:+.3f},{cmd_z:+.1f})")
            time.sleep(config.LOOP_DELAY_SEC)
    except KeyboardInterrupt:
        print()
        print('STOP REQUESTED BY USER')
    except Exception as exc:
        print()
        print('ERROR:', exc)
        raise
    finally:
        try:
            stop_chassis(chassis)
        except Exception:
            pass
        try:
            if 'edge_memory' in locals() and getattr(config, 'SAVE_EDGE_MEMORY', True):
                edge_memory.save()
        except Exception as edge_exc:
            print('EDGE MEMORY SAVE ERROR:', edge_exc)
        try:
            if mapper is not None and getattr(config, 'ENABLE_MAPPING', False):
                mapper.save_all(rebuild=True, quiet=False)
        except Exception as map_exc:
            print('MAPPER SAVE ERROR:', map_exc)
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
        print('Robot stopped and disconnected.')

# ==================== BUILT-IN HARDWARE TESTS / CLI ====================
def monster_front_ir_test():
    """Focused LEFT/RIGHT/BOTH front IR test using the same SensorManager."""
    ep_robot = robot.Robot()
    try:
        print('Connecting RoboMaster...')
        ep_robot.initialize(conn_type='ap')
        sensors = SensorManager(ep_robot.sensor_adaptor)
        print('\n====================================================')
        print(' V12 DUAL FRONT IR DIGITAL get_io() TEST')
        print('====================================================')
        print(f'LEFT  : ID={config.IR_FRONT_LEFT_ID} PORT={config.SENSOR_PORT}')
        print(f'RIGHT : ID={config.IR_FRONT_RIGHT_ID} PORT={config.SENSOR_PORT}')
        print('Expected: CLEAR=1, OBSTACLE=0')
        print('Block LEFT, RIGHT, then BOTH. Ctrl+C to stop.\n')
        while True:
            raw = sensors.read_front_ir_raw_state()
            state = sensors.update_front_ir_guard(raw.get('left_raw'), raw.get('right_raw'))
            print(f"RAW L/R={raw.get('left_raw')}/{raw.get('right_raw')} | HIT L/R={int(raw.get('left_hit', False))}/{int(raw.get('right_hit', False))} | CONF={int(state.get('left_confirmed', False))}/{int(state.get('right_confirmed', False))} | BLOCK={int(state.get('blocked', False))}")
            time.sleep(0.08)
    except KeyboardInterrupt:
        print('\nStopped.')
    finally:
        try:
            ep_robot.close()
        except Exception:
            pass
        print('RoboMaster disconnected.')

def monster_front_ir_sweep():
    """Raw get_io() sweep for Sensor Adapter IDs 1 and 4, ports 1..4."""
    ep_robot = robot.Robot()
    try:
        print('Connecting RoboMaster...')
        ep_robot.initialize(conn_type='ap')
        sensor = ep_robot.sensor_adaptor
        print('\n' + '=' * 65)
        print(' SENSOR ADAPTER DIGITAL I/O PORT SWEEP')
        print('=' * 65)
        print('Expected front IR: ID1/P1 and ID4/P1; CLEAR=1, HIT=0')
        print('Press Ctrl+C to stop.\n')
        while True:
            line = []
            for adapter_id in (1, 4):
                values = []
                for port in (1, 2, 3, 4):
                    try:
                        value = sensor.get_io(id=adapter_id, port=port)
                        values.append(f'P{port}:{value}')
                    except Exception:
                        values.append(f'P{port}:ERR')
                line.append(f'ID{adapter_id} [' + ' '.join(values) + ']')
            print(' | '.join(line))
            time.sleep(0.15)
    except KeyboardInterrupt:
        print('\nStopped.')
    finally:
        try:
            ep_robot.close()
        except Exception:
            pass
        print('RoboMaster disconnected.')

def monster_cli():
    import argparse
    parser = argparse.ArgumentParser(description='RoboMaster Maze V12.5 field build + orientation-free topology guide')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--test-ir', action='store_true', help='focused LEFT/RIGHT/BOTH digital front IR test')
    mode.add_argument('--sweep', action='store_true', help='scan Sensor Adapter ID1/ID4 ports 1..4')
    parser.add_argument('--guide', metavar='MAP_OR_ROUTE.json', help='saved maze or route exported by maze_designer (default: known_route.json if present)')
    parser.add_argument('--no-guide', action='store_true', help='disable the pre-drawn guide and use pure Trémaux/DFS')
    args = parser.parse_args()
    if args.test_ir:
        monster_front_ir_test()
    elif args.sweep:
        monster_front_ir_sweep()
    else:
        maze_main(guide_path=args.guide, disable_guide=args.no_guide)
if __name__ == '__main__':
    monster_cli()

# RoboMaster Maze Explorer V10.1 — Modular Version

This directory is a structural refactor of `maze_explorer_v10_1_mapping.py`.
The navigation/exploration/control logic is intentionally preserved; the main change is module separation and explicit imports.

## File layout

```text
maze_explorer_v10_1_modular/
├─ maze_explorer_v10_1_mapping.py   # compatibility launcher (same old command name)
├─ main.py                           # connect robot, create objects, main loop
├─ config.py                         # ALL tunable constants/calibration
├─ pose_tracker.py                   # odometry + yaw/pitch/roll tracking
├─ sensors.py                        # ToF / Sharp / IR reading + filtering
├─ motion_controller.py              # wall centering, heading hold, speed, escape
├─ exploration.py                    # Trémaux/DFS, frontier graph, junction memory
├─ navigation.py                     # relative decision -> turn + feedback turn
├─ maze_runtime.py                   # scan, junction alignment, gate/exit/safety helpers
├─ mapping.py                        # passive SLAM-style occupancy-grid mapper
└─ __init__.py
```

## Shared variables between files

### 1. Configuration values
Every logic module uses:

```python
import config
```

and reads values as:

```python
config.FORWARD_SPEED
config.SIDE_OPEN_ENTER_CM
config.TURN_ENTRY_MAX_BACKTRACK_M
```

This keeps one source of truth. Do not duplicate tuning constants into other modules.

### 2. Runtime state
State is shared by passing the SAME object instances created in `main.py`:

```python
sensors = SensorManager(sensor_adapter)
controller = MotionController()
pose_tracker = PoseTracker()
detector = DecisionPointDetector()
explorer = TremauxExplorer()
open_area_exit = OpenAreaExitManager()
mapper = SLAMStyleMazeMapper()
```

Functions receive these objects as parameters, so sensor buffers, heading state, DFS graph memory, mapper state, etc. stay persistent exactly as in the original single-file program.

## Run

From this directory:

```powershell
python .\maze_explorer_v10_1_mapping.py
```

or:

```powershell
python .\main.py
```

The compatibility launcher preserves the old filename/command while the real implementation is split into modules.

## Where to edit what

- Tune distances/speeds/thresholds: `config.py`
- Sensor calibration/filtering/timeout: `sensors.py`
- Straight driving/wall centering/yaw hold: `motion_controller.py`
- DFS/frontier/graph behaviour: `exploration.py`
- Turn behaviour: `navigation.py`
- Junction scan/setup/safety/exit: `maze_runtime.py`
- Occupancy map only: `mapping.py`
- Robot startup/subscriptions/main loop/integration: `main.py`

## Refactor verification

- All Python files compile successfully.
- Every `config.<NAME>` referenced by modules is defined in `config.py`.
- Function/class AST bodies were compared against the original source; no algorithm body changed. The removed `_ConfigProxy` existed only to simulate `config.NAME` inside the former single-file merged version.

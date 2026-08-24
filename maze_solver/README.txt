RoboMaster Maze Solver V9
=========================

Full integrated version:
- Trémaux / DFS topological memory
- Sharp median + EMA filtering
- ToF stale-data safety
- Wall following + heading hold
- Feedback 90/180 turns (no unbounded SDK wait)
- Junction creep
- Corner turn setup
- Post-turn clearance
- V9 side BORDERLINE / Schmitt-trigger detection

V9 side classification
----------------------
<= 14 cm     : BLOCKED
14-20 cm     : BORDERLINE -> keep previous OPEN/BLOCK state
>= 20 cm     : OPEN

Important:
The left/right OPEN memory is robot-relative, so V9 resets that memory after
LEFT/RIGHT/BACK turns. This prevents an OPEN state from the old orientation
being reused after the robot has turned to face a new corridor.

Run:
    python main.py

Main tuning values are in config.py.

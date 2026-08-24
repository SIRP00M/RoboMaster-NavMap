V11 IR SIDE SENSOR FUSION PATCH
===============================
Replace these files in your current V10/V9 project:
- config.py
- sensors.py
- exploration.py
- main.py

IR wiring assumed:
- LEFT IR  : Hub/Adapter ID 1, Port 1
- RIGHT IR : Hub/Adapter ID 4, Port 1

Default polarity:
- digital level 1 = WALL
If your hand test shows the opposite, change IR_LEFT_WALL_LEVEL / IR_RIGHT_WALL_LEVEL to 0.

Fusion policy:
- Sharp <= 14 cm: BLOCKED (Sharp wins)
- Sharp 14-20 cm: stable IR decides; if IR unknown, keep previous state
- Sharp >= 20 cm: OPEN (Sharp wins); IR=WALL flags conflict and triggers extra re-scan

IR is debounced with a 3-sample majority vote.
V10 expected-node backtrack matching remains included in exploration.py/main.py/config.py.

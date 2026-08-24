# RoboMaster Maze Solver - Modular

โครงสร้างนี้แยกจากไฟล์ single-file เดิมโดยคง logic เดิมไว้ เพื่อให้จูนและ debug ง่ายขึ้น

```text
robomaster_maze_modular/
├── main.py          # เชื่อม RoboMaster, subscription, main loop
├── config.py        # ค่าจูนทั้งหมด
├── sensors.py       # ToF, Sharp, IR, calibration/filter
├── pose.py          # odometry + yaw tracker
├── motion.py        # wall following, centering, heading hold
├── exploration.py   # junction detector + Trémaux/frontier DFS memory
├── navigation.py    # แปลง decision เป็น turn + feedback turn
└── __init__.py
```

## Run

เปิด PowerShell/CMD ในโฟลเดอร์นี้ แล้วรัน:

```powershell
python main.py
```

ถ้าใช้ virtual environment ให้ activate environment ที่มี RoboMaster SDK ก่อน

## เวลาแก้โค้ด

- ปรับระยะ/ความเร็ว/gain/threshold -> `config.py`
- Sharp/ToF/IR อ่านผิดหรือ filter ไม่นิ่ง -> `sensors.py`
- หุ่นส่าย/ชิดกำแพง/heading hold -> `motion.py`
- จำทางผิด/DFS วน/ตรวจ junction ผิด -> `exploration.py`
- เลี้ยวซ้ายขวา/Yaw feedback ผิด -> `navigation.py`
- lifecycle, connect, main loop -> `main.py`
- odometry/yaw callback ผิด -> `pose.py`

`maze_memory.json` ยังถูกสร้างตามค่า `MAZE_MEMORY_FILE` ใน `config.py` เหมือนเดิม

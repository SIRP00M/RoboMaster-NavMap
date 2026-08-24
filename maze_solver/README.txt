RoboMaster Maze Solver - Robust Trémaux / DFS Version
======================================================

ไฟล์หลัก
--------
- config.py       : ค่าจูนทั้งหมด
- sensors.py      : Sharp / IR / ToF + median/EMA + ToF stale protection
- controller.py   : wall following, owner control, wall hysteresis, escape
- pose_tracker.py : แยก position กับ attitude yaw
- exploration.py  : Trémaux / DFS + junction memory + robust detector re-arm
- navigation.py   : FRONT/LEFT/RIGHT/BACK + yaw verification/correction
- main.py         : เชื่อมระบบทั้งหมด + junction creep + safety override

สิ่งที่แก้จากเวอร์ชันก่อน
--------------------------
1. แก้ junction detector deadlock
   - ไม่รอ normal corridor อย่างเดียวอีกแล้ว
   - re-arm ได้จากระยะที่ออกจาก node
   - re-arm ได้จาก timeout
   - ถ้าเจอกำแพงหน้าใหม่ขณะยัง latch จะ emergency re-arm

2. เพิ่ม JUNCTION_CREEP
   - ถ้าเจอ side opening และด้านหน้ายังโล่ง
   - หุ่นจะค่อย ๆ เดินเข้าไปอีกเล็กน้อยก่อน final scan/turn
   - ลดการหมุนเร็วเกินไปตรงขอบ junction
   - ถ้า ToF ใกล้กำแพงหรือหาย จะ abort ทันที

3. เพิ่ม safety override
   - BOTH_TOO_CLOSE -> x = 0
   - ESCAPE_LEFT/RIGHT -> ลด x เหลือ ESCAPE_FORWARD_SPEED
   - ToF ไม่มีข้อมูลสด -> x = UNKNOWN_FRONT_SPEED (default 0)

4. เพิ่ม wall hysteresis
   - wall ENTER < SIDE_WALL_ENTER_CM
   - wall RELEASE > SIDE_WALL_EXIT_CM
   - ลดการสลับ WALL / OPEN เมื่อ Sharp แกว่งแถว threshold

5. แยก odometry กับ yaw
   - sub_position() -> x, y, z position data
   - sub_attitude() -> yaw, pitch, roll
   - yaw ใช้ตรวจผลการหมุน

6. Yaw verification/correction
   - หลัง 90 องศา ระบบเรียนรู้เองว่า sign ของ chassis.move(z)
     สัมพันธ์กับ attitude yaw แบบเดียวกันหรือตรงข้าม
   - ถ้าคลาดมากกว่า YAW_TOLERANCE_DEG แต่ไม่เกิน
     YAW_MAX_CORRECTION_DEG จะ correction อีกครั้ง
   - ถ้าครั้งแรกเป็น 180 และยังไม่รู้ sign จะไม่เดาสุ่ม

7. Decision re-scan
   - หยุดแล้วอ่านหลาย sample ก่อนตัดสินใจจริง
   - ถ้า sensor ไม่ครบ จะ reject event
   - ถ้าหยุดแล้วพบว่าเป็น corridor ปกติ จะ reject false junction

8. Sharp calibration แยกซ้าย/ขวา
   - CALIBRATION_SHARP_LEFT
   - CALIBRATION_SHARP_RIGHT
   - ตอนนี้เริ่มจากตารางเดิมเหมือนกัน ต้องจูนแยกหาก sensor จริงต่างกัน

ค่าที่ควรจูนหน้างานก่อน
------------------------
แนะนำเรียงลำดับนี้:

1. TURN_LEFT_DEG / TURN_RIGHT_DEG และ Z_DIR_SIGN
2. TARGET_LEFT_CM / TARGET_RIGHT_CM
3. SIDE_WALL_ENTER_CM / SIDE_WALL_EXIT_CM
4. EXPLORATION_SIDE_OPEN_CM
5. JUNCTION_CREEP_SPEED / JUNCTION_CREEP_DISTANCE_M
6. JUNCTION_REARM_DISTANCE_M
7. NODE_MATCH_RADIUS_M

ค่าเริ่มต้น Junction Creep
--------------------------
JUNCTION_CREEP_SPEED = 0.07 m/s
JUNCTION_CREEP_DISTANCE_M = 0.06 m

ระยะเชิงทฤษฎีประมาณ 3.5 cm ก่อนหัก slip/acceleration
ถ้าเลี้ยวก่อนถึงกลางทางแยก ให้เพิ่มทีละน้อย เช่น 0.55 / 0.60 s
ถ้าเลย junction ให้ลดลง

วิธีรัน
------
วางไฟล์ทั้งหมดไว้โฟลเดอร์เดียวกัน แล้วรัน:

    python main.py

ไฟล์ที่สร้างระหว่างรัน
----------------------
maze_memory.json

ใช้ดู node, edge visits, target node และ route_history หลังวิ่ง

หมายเหตุสำคัญ
-------------
- ทดสอบครั้งแรกควรยก/เตรียมจับหุ่น และใช้สนามทดลองสั้น ๆ ก่อน
- ตรวจว่าคำสั่ง LEFT/RIGHT หมุนถูกทิศจริง
- ดู log YAW หลัง turn ครั้งแรก ว่าระบบเรียน sign_map ได้เป็น +1 หรือ -1
- IR ยังไม่ได้ใช้บังคับ steering เพราะต้องรู้ polarity จากการติดตั้งจริงก่อน


สนามจริง revision v2 (จาก log 24 Aug 2026):
- EXPLORATION_SIDE_OPEN_CM = 20 cm เพราะช่องขวาจริงวัดได้ประมาณ 20-24 cm
- EXPLORATION_FRONT_OPEN_CM = 35 cm เพื่อไม่ถือกำแพงหน้า 17-24 cm ว่าเป็นทางตรง
- แก้ root completion: root ที่มีแต่ทาง BACK จะ backtrack ไม่ประกาศ COMPLETE
- ตัวอย่าง log เดิม Front=8.9, L=9.1, R=22.6 จะถูกจัดเป็น RIGHT OPEN


V3 fixes:
- DFS direction FRONT now maps correctly in navigation.py (FORWARD kept as compatibility alias).
- Front traversable threshold raised to 35 cm based on real T-junction log (Front 27.3 cm must be BLOCK while Right 23.4 cm is OPEN).


V4 stability changes:
- Absolute yaw grid + heading hold while driving (x/y/z can work together).
- If yaw error becomes large, stop translation and recover heading first.
- After each DFS decision, align to absolute N/E/S/W yaw before leaving junction.
- Raw ADC spikes no longer trigger emergency strafe; escape uses filtered distance.
- A side opening cannot simultaneously be treated as a centering wall.
- Reduced lateral correction strength to prevent Mecanum over-correction.
- Junction latch does not re-arm from distance alone while still inside the same opening.

Debug log YAW now prints current/target and E is yaw error, e.g.
YAW:+105.0/+107.2 E:+2.2


V5 heading-sign fix
===================
- แยก sign ของ chassis.move(z) และ chassis.drive_speed(z) ออกจากกัน
- ของหุ่นที่ทดสอบ: move->yaw = -1, drive_speed->yaw = +1
- Heading Hold ใช้ DRIVE sign เท่านั้น
- Turn/Yaw verification ใช้ MOVE sign เท่านั้น
- ลด heading gain/max z เพื่อให้ correction นุ่มขึ้น

ถ้า log แสดง target > current แต่ +z ทำให้ yaw ลด ให้สลับ DEFAULT_DRIVE_TO_YAW_SIGN ใน config.py

V6 corner-turn fix
------------------
- Front-open junction creep now uses odometry distance (default 0.06 m), not only fixed time.
- LEFT/RIGHT corners where front is not traversable get TURN_SETUP before rotation.
- TURN_SETUP moves at 0.05 m/s, max 0.07 m, but stops early at front ToF 13 cm and has 10 cm hard stop.
- Junction latch origin is taken after turn/setup, reducing premature re-arm caused by pre-turn nudging.

V7 TURN HANG FIX
----------------
- Normal LEFT/RIGHT/BACK turns no longer use chassis.move().wait_for_completed().
- They use attitude yaw + drive_speed(z) closed-loop with a hard watchdog.
- If yaw feedback is unavailable, action fallback uses wait_for_completed(timeout=...).
- No code path uses wait_for_completed() without a timeout.
- If a turn cannot finish inside the watchdog, the robot stops and the Trémaux edge is NOT committed.

V8 corner-clearance changes
---------------------------
- Corner TURN_SETUP now allows up to 0.14 m instead of 0.07 m and primarily aims for front ToF ~= 14 cm before a 90-degree turn.
- Added POST_TURN_CLEARANCE. If the inner Sharp side is still <= 6.5 cm after LEFT/RIGHT, the robot crawls forward at 0.045 m/s with a small outward strafe until >= 7.5 cm, 0.07 m travel, 1.5 s, or front safety stop.
- Feedback yaw turn from V7 remains unchanged.

RoboMaster Maze Solver - Modular Version
=========================================

ไฟล์:
- config.py      : ค่าจูน, Sensor IDs, speed, distance, turn constants
- sensors.py     : อ่าน Sharp / IR / ToF, calibration, median + EMA filters
- controller.py  : forward slowdown, wall-following, both-wall owner/hysteresis
- navigation.py  : ตัดสินใจ LEFT / RIGHT / 180 และสั่ง chassis.move()
- main.py        : เชื่อม RoboMaster และรวมทุก module เข้าด้วยกัน

วิธีใช้:
1. วางไฟล์ทั้งหมดไว้ในโฟลเดอร์เดียวกัน
2. เปิด virtual environment ที่ติดตั้ง RoboMaster SDK แล้ว
3. cd เข้าโฟลเดอร์นี้
4. รัน:

   python main.py

ถ้าจะจูนค่าหน้างาน ให้แก้ config.py เป็นหลัก
ไม่ต้องไล่แก้ค่าซ้ำในหลายไฟล์

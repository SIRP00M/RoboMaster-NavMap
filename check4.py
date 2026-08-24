import time
import statistics
from collections import deque
from robomaster import robot

# ============================================================
# CONFIGURATION & PARAMETERS
# ============================================================
ENABLE_MOTION = True

IR_LEFT_FRONT_ID = 1      # IR Sensor ซ้ายหน้า (CAN Bus ID 1 - ขา I/O)
SHARP_LEFT_ID = 2         # Sharp Sensor ซ้าย (CAN Bus ID 2 - ขา ADC)
SENSOR_PORT = 1           # พอร์ต I/O และ ADC บนโมดูล CAN

TARGET_LEFT_CM = 6     # ระยะห่างเป้าหมายจากกำแพงซ้าย (10 cm)
STOP_FRONT_CM = 10   # ระยะหยุดหน้ากำแพง ToF (5 cm)
FORWARD_SPEED = 0.25     # ความเร็วเดินหน้า (m/s)

# ตัวคูณปรับทิศทาง (หากสไลด์หรือหมุนผิดทิศ ให้ปรับเปลี่ยนเป็น -1 หรือ 1)
Y_DIR_SIGN = 1            # เครื่องหมายแกน Y (สไลด์ซ้าย-ขวา)
Z_DIR_SIGN = 1            # เครื่องหมายแกน Z (หมุนซ้าย-ขวา)

KP_STRAFE = 0.02          # ความไวการสไลด์ข้าง (m/s per cm)
KP_ROTATE = 2.0           # ความไวการหมุนตัว (deg/s per cm)

MAX_Y_SPEED = 0.12        # สไลด์ข้างสูงสุด (m/s)
MAX_Z_SPEED = 20.0        # หมุนตัวสูงสุด (deg/s)

CALIBRATION_SHARP2 = [
    (675, 5.0), (343, 10.0), (236, 15.0), 
    (166, 20.0), (126, 25.0), (105, 30.0)
]

sharp_buffer = deque(maxlen=3)
sharp_ema = None
tof_buffer = deque(maxlen=3)
front_cm = None

# ============================================================
# FUNCTIONS
# ============================================================
def tof_callback(data):
    global front_cm
    try:
        if not data or data[0] is None:
            return
        mm = data[0]
        if mm < 20 or mm > 4000:
            return
        cm = mm / 10.0
        tof_buffer.append(cm)
        front_cm = statistics.median(tof_buffer)
    except Exception as e:
        print("ToF callback error:", e)

def adc_to_cm(adc):
    table = CALIBRATION_SHARP2
    if adc >= table[0][0]:
        return 5.0
    if adc <= table[-1][0]:
        return 30.0

    for i in range(len(table) - 1):
        adc1, cm1 = table[i]
        adc2, cm2 = table[i + 1]
        if adc1 >= adc >= adc2:
            ratio = (adc1 - adc) / (adc1 - adc2)
            return cm1 + ratio * (cm2 - cm1)
    return 30.0

def read_sharp_raw_and_cm(sensor_adapter):
    global sharp_ema
    raw = sensor_adapter.get_adc(id=SHARP_LEFT_ID, port=SENSOR_PORT)
    sharp_buffer.append(raw)
    median_adc = statistics.median(sharp_buffer)

    if sharp_ema is None:
        sharp_ema = median_adc
    else:
        sharp_ema = (0.6 * median_adc) + (0.4 * sharp_ema)

    return raw, adc_to_cm(sharp_ema)

def read_ir_digital_io(sensor_adapter):
    """
    อ่านค่า Digital I/O จาก IR Sensor (CAN Bus ID 1)
    คืนค่า 1 (พบกำแพง / High) หรือ 0 (ไม่พบกำแพง / Low)
    """
    try:
        # อ่านค่าระดับสัญญาณดิจิทัล High/Low จากขา I/O
        return sensor_adapter.get_io_level(id=IR_LEFT_FRONT_ID, port=SENSOR_PORT)
    except Exception:
        # สำรองกรณีอ่านค่า ADC หากไม่ได้ตั้งโหมด Pin เป็น I/O
        raw = sensor_adapter.get_adc(id=IR_LEFT_FRONT_ID, port=SENSOR_PORT)
        return 1 if raw > 300 else 0

def clamp(val, min_val, max_val):
    return max(min_val, min(max_val, val))

def fmt(val):
    return "---" if val is None else f"{val:4.1f}"

def calculate_motion_control(raw_adc, sharp_cm, ir_wall):
    if sharp_cm is None:
        return 0.0, 0.0

    # กรณีฉุกเฉิน: ชิดกำแพงซ้ายเกินไป (Sharp <= 6 cm หรือ ADC สูง)
    if raw_adc >= 600 or sharp_cm <= 6.0:
        y_cmd = +0.12 * Y_DIR_SIGN
        z_cmd = +18.0 * Z_DIR_SIGN
        return y_cmd, z_cmd

    # คำนวณระยะคลาดเคลื่อนจาก 10 cm
    error = sharp_cm - TARGET_LEFT_CM

    if abs(error) <= 0.8:
        y_cmd = 0.0
        z_cmd = 0.0
    else:
        y_cmd = clamp(-error * KP_STRAFE * Y_DIR_SIGN, -MAX_Y_SPEED, MAX_Y_SPEED)
        z_cmd = clamp(-error * KP_ROTATE * Z_DIR_SIGN, -MAX_Z_SPEED, MAX_Z_SPEED)

    return y_cmd, z_cmd

# ============================================================
# MAIN LOOP
# ============================================================
ep_robot = robot.Robot()

try:
    print("Connecting RoboMaster...")
    ep_robot.initialize(conn_type="ap")

    chassis = ep_robot.chassis
    sensor_adapter = ep_robot.sensor_adaptor
    tof_sensor = ep_robot.sensor

    tof_sensor.sub_distance(freq=20, callback=tof_callback)

    print("\n======================================")
    print("  LEFT WALL FOLLOW (IR: DIGITAL I/O CAN 1)")
    print("======================================\n")

    time.sleep(1.0)
    stop_confirm_count = 0

    while True:
        raw_adc, sharp_left_cm = read_sharp_raw_and_cm(sensor_adapter)
        ir_left_wall = read_ir_digital_io(sensor_adapter)

        x, y, z = 0.0, 0.0, 0.0

        # เงื่อนไขหยุดเมื่อ ToF หน้า <= 5 cm
        if front_cm is not None and 0.0 < front_cm <= STOP_FRONT_CM:
            stop_confirm_count += 1
            if stop_confirm_count >= 3:
                print(f"\n[STOP REACHED] Front Wall Distance: {front_cm:.1f} cm (<= {STOP_FRONT_CM} cm)")
                chassis.drive_speed(x=0, y=0, z=0, timeout=0.1)
                break
        else:
            stop_confirm_count = 0
            x = FORWARD_SPEED
            y, z = calculate_motion_control(raw_adc, sharp_left_cm, ir_left_wall)

        if ENABLE_MOTION:
            chassis.drive_speed(x=x, y=y, z=z, timeout=0.15)

        print(
            f"ToF:{fmt(front_cm)}cm | "
            f"Sharp:{fmt(sharp_left_cm)}cm (ADC:{raw_adc:3d}) | "
            f"IR_IO:{'1 (WALL)' if ir_left_wall else '0 (OPEN)'} | "
            f"Cmd -> x={x:.2f}, y={y:+.2f}, z={z:+.1f}"
        )

        time.sleep(0.05)

except KeyboardInterrupt:
    print("\nSTOP REQUESTED BY USER")
except Exception as e:
    print("\nERROR:", e)
finally:
    try:
        chassis.drive_speed(x=0, y=0, z=0, timeout=0.1)
    except:
        pass
    try:
        tof_sensor.unsub_distance()
    except:
        pass
    ep_robot.close()
    print("Robot stopped and disconnected.")
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
SHARP_RIGHT_ID = 3        # Sharp Sensor ขวา (CAN Bus ID 3 - ขา ADC)
SENSOR_PORT = 1           # พอร์ต I/O และ ADC บนโมดูล CAN

TARGET_LEFT_CM = 6        # ระยะห่างเป้าหมายจากกำแพงซ้าย (cm)
STOP_FRONT_CM = 15        # ระยะเริ่มตรวจจับกำแพงด้านหน้า ToF (30 cm)
SIDE_DEAD_END_CM = 15  # ระยะตรวจจับทางตันด้านข้าง (< 30 cm)
FORWARD_SPEED = 0.15
# ความเร็วเดินหน้า (m/s)

# ตัวคูณปรับทิศทาง
Y_DIR_SIGN = 1            # เครื่องหมายแกน Y (สไลด์ซ้าย-ขวา)
Z_DIR_SIGN = 1            # เครื่องหมายแกน Z (หมุนซ้าย-ขวา)

KP_STRAFE = 0.02          # ความไวการสไลด์ข้าง (m/s per cm)
KP_ROTATE = 2.0           # ความไวการหมุนตัว (deg/s per cm)

MAX_Y_SPEED = 0.12        # สไลด์ข้างสูงสุด (m/s)
MAX_Z_SPEED = 20.0        # หมุนตัวสูงสุด (deg/s)

# ตาราง Calibration ค่า ADC -> CM (ขยายระยะอ่านสูงสุดเป็น 80.0 cm)
CALIBRATION_SHARP2 = [
    (675, 5.0), (343, 10.0), (236, 15.0), 
    (166, 20.0), (126, 25.0), (105, 30.0), (50, 80.0)
]

# Filtering Buffers แยกซ้าย-ขวา
sharp_left_buffer = deque(maxlen=3)
sharp_right_buffer = deque(maxlen=3)
sharp_left_ema = None
sharp_right_ema = None

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
        return 80.0  # ปรับค่า Max ระยะซ้าย-ขวาเป็น 80.0 cm

    for i in range(len(table) - 1):
        adc1, cm1 = table[i]
        adc2, cm2 = table[i + 1]
        if adc1 >= adc >= adc2:
            ratio = (adc1 - adc) / (adc1 - adc2)
            return cm1 + ratio * (cm2 - cm1)
    return 80.0

def read_sharp_raw_and_cm(sensor_adapter, sensor_id):
    global sharp_left_ema, sharp_right_ema
    raw = sensor_adapter.get_adc(id=sensor_id, port=SENSOR_PORT)

    if sensor_id == SHARP_LEFT_ID:
        sharp_left_buffer.append(raw)
        median_adc = statistics.median(sharp_left_buffer)
        if sharp_left_ema is None:
            sharp_left_ema = median_adc
        else:
            sharp_left_ema = (0.6 * median_adc) + (0.4 * sharp_left_ema)
        ema_val = sharp_left_ema
    else:
        sharp_right_buffer.append(raw)
        median_adc = statistics.median(sharp_right_buffer)
        if sharp_right_ema is None:
            sharp_right_ema = median_adc
        else:
            sharp_right_ema = (0.6 * median_adc) + (0.4 * sharp_right_ema)
        ema_val = sharp_right_ema

    return raw, adc_to_cm(ema_val)

def read_ir_digital_io(sensor_adapter):
    try:
        return sensor_adapter.get_io_level(id=IR_LEFT_FRONT_ID, port=SENSOR_PORT)
    except Exception:
        raw = sensor_adapter.get_adc(id=IR_LEFT_FRONT_ID, port=SENSOR_PORT)
        return 1 if raw > 300 else 0

def clamp(val, min_val, max_val):
    return max(min_val, min(max_val, val))

def fmt(val):
    return "---" if val is None else f"{val:4.1f}"

def calculate_motion_control(raw_adc, sharp_cm, ir_wall):
    if sharp_cm is None:
        return 0.0, 0.0

    if raw_adc >= 600 or sharp_cm <= 6.0:
        y_cmd = +0.12 * Y_DIR_SIGN
        z_cmd = +18.0 * Z_DIR_SIGN
        return y_cmd, z_cmd

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
    print(" MAZE SOLVER (MAX SHARP 80cm & 180-DEG DEAD END)")
    print("======================================\n")

    time.sleep(1.0)
    stop_confirm_count = 0

    while True:
        raw_adc_l, sharp_left_cm = read_sharp_raw_and_cm(sensor_adapter, SHARP_LEFT_ID)
        raw_adc_r, sharp_right_cm = read_sharp_raw_and_cm(sensor_adapter, SHARP_RIGHT_ID)
        ir_left_wall = read_ir_digital_io(sensor_adapter)

        x, y, z = 0.0, 0.0, 0.0

        # ตรวจสอบกำแพงด้านหน้า (ToF <= 30 cm)
        if front_cm is not None and 0.0 < front_cm <= STOP_FRONT_CM:
            stop_confirm_count += 1
            if stop_confirm_count >= 3:
                print(f"\n[WALL DETECTED] Front Distance: {front_cm:.1f} cm (<= {STOP_FRONT_CM} cm)")
                
                # หยุดรถชั่วคราวเพื่ออ่านค่าระยะข้างให้แม่นยำ
                if ENABLE_MOTION:
                    chassis.drive_speed(x=0, y=0, z=0, timeout=0.1)
                time.sleep(0.3)

                _, l_dist = read_sharp_raw_and_cm(sensor_adapter, SHARP_LEFT_ID)
                _, r_dist = read_sharp_raw_and_cm(sensor_adapter, SHARP_RIGHT_ID)
                
                print(f"Distance Check -> Left: {l_dist:.1f} cm | Right: {r_dist:.1f} cm")

                # เงื่อนไข 1: ทางตัน 3 ด้าน (หน้า <= 30 cm, ซ้าย < 30 cm, ขวา < 30 cm) -> หมุนขวา 180 องศา
                if l_dist < SIDE_DEAD_END_CM and r_dist < SIDE_DEAD_END_CM:
                    print("--> DEAD END! (All 3 sides < 30 cm) -> Turning RIGHT 180 degrees")
                    if ENABLE_MOTION:
                        chassis.move(x=0, y=0, z=-180 * Z_DIR_SIGN, z_speed=60).wait_for_completed()

                # เงื่อนไข 2: ด้านซ้ายกว้างกว่าด้านขวา > 20 cm -> เลี้ยวซ้าย 90 องศา
                elif (l_dist - r_dist) > 20.0:
                    print("--> Left is wider (>20 cm) -> Turning LEFT 90 degrees")
                    if ENABLE_MOTION:
                        chassis.move(x=0, y=0, z=90 * Z_DIR_SIGN, z_speed=60).wait_for_completed()

                # เงื่อนไข 3: ด้านขวากว้างกว่าด้านซ้าย > 20 cm -> เลี้ยวขวา 90 องศา
                elif (r_dist - l_dist) > 20.0:
                    print("--> Right is wider (>20 cm) -> Turning RIGHT 90 degrees")
                    if ENABLE_MOTION:
                        chassis.move(x=0, y=0, z=-90 * Z_DIR_SIGN, z_speed=60).wait_for_completed()

                # เงื่อนไขสำรอง: เลี้ยวไปทางฝั่งที่กว้างกว่า 90 องศา
                else:
                    if l_dist >= r_dist:
                        print("--> Fallback: Turning LEFT 90 degrees")
                        if ENABLE_MOTION:
                            chassis.move(x=0, y=0, z=90 * Z_DIR_SIGN, z_speed=60).wait_for_completed()
                    else:
                        print("--> Fallback: Turning RIGHT 90 degrees")
                        if ENABLE_MOTION:
                            chassis.move(x=0, y=0, z=-90 * Z_DIR_SIGN, z_speed=60).wait_for_completed()

                # เคลียร์ค่า Buffer เพื่อกลับเข้า Loop เดินหน้าตามปกติทันที
                tof_buffer.clear()
                sharp_left_buffer.clear()
                sharp_right_buffer.clear()
                front_cm = None
                stop_confirm_count = 0
                time.sleep(0.2)
        else:
            stop_confirm_count = 0
            x = FORWARD_SPEED
            y, z = calculate_motion_control(raw_adc_l, sharp_left_cm, ir_left_wall)

        if ENABLE_MOTION:
            chassis.drive_speed(x=x, y=y, z=z, timeout=0.15)

        print(
            f"ToF:{fmt(front_cm)}cm | "
            f"SharpL:{fmt(sharp_left_cm)}cm | "
            f"SharpR:{fmt(sharp_right_cm)}cm | "
            f"IR_IO:{'1' if ir_left_wall else '0'} | "
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
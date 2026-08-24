import time
import statistics
from collections import deque
from robomaster import robot

# ============================================================
# CONFIGURATION & PARAMETERS
# ============================================================
ENABLE_MOTION = True

# ---------------- SENSOR ID ----------------
IR_LEFT_ID = 1
SHARP_LEFT_ID = 2

SHARP_RIGHT_ID = 3
IR_RIGHT_ID = 4

SENSOR_PORT = 1

# ---------------- DISTANCE ----------------
TARGET_SIDE_CM = 6.0
STOP_FRONT_CM = 25.0

# ---------------- SPEED ----------------
FORWARD_SPEED = 0.25

# ถ้าหุ่นแก้ทิศผิด ให้สลับ 1 <-> -1
Y_DIR_SIGN = -1
Z_DIR_SIGN = -1

# ---------------- CONTROLLER ----------------
KP_STRAFE = 0.015
KP_ROTATE = 1.5

MAX_Y_SPEED = 0.10
MAX_Z_SPEED = 18.0

# ถ้าซ้าย-ขวาต่างกันน้อยกว่านี้ ถือว่าตรงแล้ว
CENTER_DEADBAND_CM = 0.8

# ============================================================
# SHARP CALIBRATION
# ============================================================

CALIBRATION_SHARP_LEFT = [
    (675, 5.0),
    (343, 10.0),
    (236, 15.0),
    (166, 20.0),
    (126, 25.0),
    (105, 30.0)
]

CALIBRATION_SHARP_RIGHT = [
    (675, 5.0),
    (343, 10.0),
    (236, 15.0),
    (166, 20.0),
    (126, 25.0),
    (105, 30.0)
]

# ============================================================
# FILTER BUFFERS
# ============================================================

sharp_left_buffer = deque(maxlen=3)
sharp_right_buffer = deque(maxlen=3)

sharp_left_ema = None
sharp_right_ema = None

tof_buffer = deque(maxlen=3)
front_cm = None

# ============================================================
# TOF
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


# ============================================================
# SHARP CONVERSION
# ============================================================

def adc_to_cm(adc, table):

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


# ============================================================
# READ SHARP LEFT
# ============================================================

def read_sharp_left(sensor_adapter):

    global sharp_left_ema

    raw = sensor_adapter.get_adc(
        id=SHARP_LEFT_ID,
        port=SENSOR_PORT
    )

    sharp_left_buffer.append(raw)

    median_adc = statistics.median(sharp_left_buffer)

    if sharp_left_ema is None:
        sharp_left_ema = median_adc
    else:
        sharp_left_ema = (
            0.6 * median_adc
            + 0.4 * sharp_left_ema
        )

    cm = adc_to_cm(
        sharp_left_ema,
        CALIBRATION_SHARP_LEFT
    )

    return raw, cm


# ============================================================
# READ SHARP RIGHT
# ============================================================

def read_sharp_right(sensor_adapter):

    global sharp_right_ema

    raw = sensor_adapter.get_adc(
        id=SHARP_RIGHT_ID,
        port=SENSOR_PORT
    )

    sharp_right_buffer.append(raw)

    median_adc = statistics.median(sharp_right_buffer)

    if sharp_right_ema is None:
        sharp_right_ema = median_adc
    else:
        sharp_right_ema = (
            0.6 * median_adc
            + 0.4 * sharp_right_ema
        )

    cm = adc_to_cm(
        sharp_right_ema,
        CALIBRATION_SHARP_RIGHT
    )

    return raw, cm


# ============================================================
# READ IR
# ============================================================

def read_ir(sensor_adapter, sensor_id):

    try:

        return sensor_adapter.get_io_level(
            id=sensor_id,
            port=SENSOR_PORT
        )

    except Exception:

        raw = sensor_adapter.get_adc(
            id=sensor_id,
            port=SENSOR_PORT
        )

        return 1 if raw > 300 else 0


# ============================================================
# UTILITIES
# ============================================================

def clamp(val, min_val, max_val):
    return max(min_val, min(max_val, val))


def fmt(val):
    return "---" if val is None else f"{val:4.1f}"


# ============================================================
# CORRIDOR CENTERING CONTROLLER
# ============================================================

def calculate_corridor_control(
    left_cm,
    right_cm,
    ir_left,
    ir_right
):

    if left_cm is None or right_cm is None:
        return 0.0, 0.0

    # --------------------------------------------------------
    # CASE 1: มีกำแพงทั้งสองฝั่ง
    #
    # ถ้า Left > Right
    # = หุ่นอยู่ใกล้ฝั่งขวา
    # = ต้องขยับไปทางซ้าย
    #
    # ถ้า Right > Left
    # = หุ่นอยู่ใกล้ฝั่งซ้าย
    # = ต้องขยับไปทางขวา
    # --------------------------------------------------------

    if ir_left and ir_right:

        error = left_cm - right_cm

        if abs(error) <= CENTER_DEADBAND_CM:
            return 0.0, 0.0

        y_cmd = clamp(
            error * KP_STRAFE * Y_DIR_SIGN,
            -MAX_Y_SPEED,
            MAX_Y_SPEED
        )

        z_cmd = clamp(
            error * KP_ROTATE * Z_DIR_SIGN,
            -MAX_Z_SPEED,
            MAX_Z_SPEED
        )

        return y_cmd, z_cmd


    # --------------------------------------------------------
    # CASE 2: มีกำแพงเฉพาะซ้าย
    # Follow Left Wall
    # --------------------------------------------------------

    elif ir_left and not ir_right:

        error = left_cm - TARGET_SIDE_CM

        if abs(error) <= CENTER_DEADBAND_CM:
            return 0.0, 0.0

        y_cmd = clamp(
            -error * KP_STRAFE * Y_DIR_SIGN,
            -MAX_Y_SPEED,
            MAX_Y_SPEED
        )

        z_cmd = clamp(
            -error * KP_ROTATE * Z_DIR_SIGN,
            -MAX_Z_SPEED,
            MAX_Z_SPEED
        )

        return y_cmd, z_cmd


    # --------------------------------------------------------
    # CASE 3: มีกำแพงเฉพาะขวา
    # Follow Right Wall
    # --------------------------------------------------------

    elif ir_right and not ir_left:

        error = right_cm - TARGET_SIDE_CM

        if abs(error) <= CENTER_DEADBAND_CM:
            return 0.0, 0.0

        y_cmd = clamp(
            error * KP_STRAFE * Y_DIR_SIGN,
            -MAX_Y_SPEED,
            MAX_Y_SPEED
        )

        z_cmd = clamp(
            error * KP_ROTATE * Z_DIR_SIGN,
            -MAX_Z_SPEED,
            MAX_Z_SPEED
        )

        return y_cmd, z_cmd


    # --------------------------------------------------------
    # CASE 4: ไม่มีกำแพงทั้งสองด้าน
    # วิ่งตรงไป
    # --------------------------------------------------------

    else:
        return 0.0, 0.0


# ============================================================
# MAIN
# ============================================================

ep_robot = robot.Robot()

try:

    print("Connecting RoboMaster...")

    ep_robot.initialize(
        conn_type="ap"
    )

    chassis = ep_robot.chassis
    sensor_adapter = ep_robot.sensor_adaptor
    tof_sensor = ep_robot.sensor

    tof_sensor.sub_distance(
        freq=20,
        callback=tof_callback
    )

    print()
    print("======================================")
    print("       CORRIDOR CENTERING MODE")
    print("======================================")
    print()

    time.sleep(1.0)

    stop_confirm_count = 0

    while True:

        # ----------------------------------------------------
        # READ SENSOR
        # ----------------------------------------------------

        raw_left, left_cm = read_sharp_left(
            sensor_adapter
        )

        raw_right, right_cm = read_sharp_right(
            sensor_adapter
        )

        ir_left = read_ir(
            sensor_adapter,
            IR_LEFT_ID
        )

        ir_right = read_ir(
            sensor_adapter,
            IR_RIGHT_ID
        )

        x = 0.0
        y = 0.0
        z = 0.0

        # ----------------------------------------------------
        # FRONT STOP
        # ----------------------------------------------------

        if (
            front_cm is not None
            and 0.0 < front_cm <= STOP_FRONT_CM
        ):

            stop_confirm_count += 1

            if stop_confirm_count >= 3:

                print()
                print(
                    f"[STOP] Front wall "
                    f"{front_cm:.1f} cm"
                )

                chassis.drive_speed(
                    x=0,
                    y=0,
                    z=0,
                    timeout=0.1
                )

                break

        else:

            stop_confirm_count = 0

            x = FORWARD_SPEED

            y, z = calculate_corridor_control(
                left_cm,
                right_cm,
                ir_left,
                ir_right
            )

        # ----------------------------------------------------
        # DRIVE
        # ----------------------------------------------------

        if ENABLE_MOTION:

            chassis.drive_speed(
                x=x,
                y=y,
                z=z,
                timeout=0.15
            )

        # ----------------------------------------------------
        # DEBUG
        # ----------------------------------------------------

        center_error = (
            left_cm - right_cm
            if left_cm is not None
            and right_cm is not None
            else 0
        )

        print(
            f"ToF:{fmt(front_cm)}cm | "
            f"L:{fmt(left_cm)}cm ADC:{raw_left:3d} "
            f"IR:{ir_left} | "
            f"R:{fmt(right_cm)}cm ADC:{raw_right:3d} "
            f"IR:{ir_right} | "
            f"Err:{center_error:+5.1f} | "
            f"Cmd x={x:.2f} "
            f"y={y:+.2f} "
            f"z={z:+.1f}"
        )

        time.sleep(0.05)


except KeyboardInterrupt:

    print("\nSTOP REQUESTED BY USER")


except Exception as e:

    print("\nERROR:", e)


finally:

    try:
        chassis.drive_speed(
            x=0,
            y=0,
            z=0,
            timeout=0.1
        )
    except:
        pass

    try:
        tof_sensor.unsub_distance()
    except:
        pass

    ep_robot.close()

    print("Robot stopped and disconnected.")
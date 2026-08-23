from robomaster import robot
from collections import deque
import statistics
import argparse
import time


# ============================================================
# ARGUMENT
# ============================================================

parser = argparse.ArgumentParser()
parser.add_argument("--run", action="store_true")
args = parser.parse_args()

ENABLE_MOTION = args.run


# ============================================================
# SENSOR POSITION
# ============================================================

LF_ID = 1
LR_ID = 2
RF_ID = 3
RR_ID = 4

SHARP_PORT = 1


# ============================================================
# CALIBRATION
# ============================================================

CALIBRATION = {

    1: [
        (591, 5.0),
        (335, 10.0),
        (245, 15.0),
        (204, 20.0),
        (145, 25.0),
        (108, 30.0),
    ],

    2: [
        (675, 5.0),
        (343, 10.0),
        (236, 15.0),
        (166, 20.0),
        (126, 25.0),
        (105, 30.0),
    ],

    3: [
        (954, 5.0),
        (643, 10.0),
        (466, 15.0),
        (390, 20.0),
        (326, 25.0),
        (276, 30.0),
    ],

    4: [
        (586, 5.0),
        (352, 10.0),
        (246, 15.0),
        (185, 20.0),
        (144, 25.0),
        (102, 30.0),
    ],
}


# ============================================================
# FILTER
# ============================================================

MEDIAN_WINDOW = 5
EMA_ALPHA = 0.35

buffers = {
    1: deque(maxlen=MEDIAN_WINDOW),
    2: deque(maxlen=MEDIAN_WINDOW),
    3: deque(maxlen=MEDIAN_WINDOW),
    4: deque(maxlen=MEDIAN_WINDOW),
}

ema = {
    1: None,
    2: None,
    3: None,
    4: None,
}


# ============================================================
# DISTANCE SETTINGS
# ============================================================

# เริ่มหลบกำแพง
SIDE_TRIGGER_CM = 10.0

# ต้องห่างถึงนี่ถึงจะเลิกหมุน
SIDE_RELEASE_CM = 12.0

# ถ้าใกล้มาก หมุนแรงขึ้น
SIDE_DANGER_CM = 7.0

# Front ToF
FRONT_STOP_CM = 20.0
FRONT_RELEASE_CM = 23.0


# ============================================================
# SPEED
# ============================================================

FORWARD_SPEED = 0.15

TURN_SPEED = 25.0
DANGER_TURN_SPEED = 35.0

# ตอนเดินตรง ใช้ correction หมุนเล็กน้อย
# ไม่มี y / strafe
STRAIGHT_CORRECTION_MAX = 5.0

# ความต่างหน้า-หลังต้องเกินเท่านี้ถึงแก้ heading
ANGLE_DEADBAND_CM = 2.0

ANGLE_KP = 1.2


# ============================================================
# STATE
# ============================================================

STATE_FORWARD = "FORWARD"
STATE_TURN_LEFT = "TURN_LEFT"
STATE_TURN_RIGHT = "TURN_RIGHT"
STATE_FRONT_STOP = "FRONT_STOP"

state = STATE_FORWARD

# บังคับให้การเลี้ยวแต่ละครั้งอยู่ขั้นต่ำช่วงหนึ่ง
# ป้องกัน sensor เปลี่ยนค่าแล้วกลับซ้าย/ขวารัว
TURN_MIN_TIME = 0.40
turn_start_time = 0.0


# ============================================================
# TOF
# ============================================================

tof_buffer = deque(maxlen=5)

front_cm = None
front_time = 0.0


def tof_callback(data):

    global front_cm
    global front_time

    try:

        if not data:
            return

        mm = data[0]

        if mm <= 0:
            return

        cm = mm / 10.0

        tof_buffer.append(cm)

        front_cm = statistics.median(tof_buffer)
        front_time = time.time()

    except:
        pass


# ============================================================
# UTILITY
# ============================================================

def clamp(v, mn, mx):
    return max(mn, min(mx, v))


def adc_to_cm(sensor_id, adc):

    table = CALIBRATION[sensor_id]

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


def read_sharp(sensor, sensor_id):

    raw = sensor.get_adc(
        id=sensor_id,
        port=SHARP_PORT
    )

    buffers[sensor_id].append(raw)

    med = statistics.median(
        buffers[sensor_id]
    )

    if ema[sensor_id] is None:
        ema[sensor_id] = med

    else:

        ema[sensor_id] = (
            EMA_ALPHA * med
            +
            (1.0 - EMA_ALPHA) * ema[sensor_id]
        )

    cm = adc_to_cm(
        sensor_id,
        ema[sensor_id]
    )

    return cm


def fmt(v):

    if v is None:
        return "---"

    return f"{v:4.1f}"


# ============================================================
# STRAIGHT HEADING CORRECTION
#
# ไม่มีแกน y
# ใช้แค่ z เล็กน้อยเพื่อพยายามให้ขนานกับกำแพง
# ============================================================

def straight_heading_correction(lf, lr, rf, rr):

    corrections = []


    # ---------------- LEFT ----------------

    # ใช้เฉพาะตอนกำแพงอยู่ใกล้พอให้เชื่อถือได้
    if max(lf, lr) < 20.0:

        diff = lf - lr

        if abs(diff) > ANGLE_DEADBAND_CM:

            corrections.append(
                ANGLE_KP * diff
            )


    # ---------------- RIGHT ----------------

    if max(rf, rr) < 20.0:

        diff = rf - rr

        if abs(diff) > ANGLE_DEADBAND_CM:

            corrections.append(
                -ANGLE_KP * diff
            )


    if not corrections:
        return 0.0


    z = sum(corrections) / len(corrections)

    return clamp(
        z,
        -STRAIGHT_CORRECTION_MAX,
        STRAIGHT_CORRECTION_MAX
    )


# ============================================================
# ROBOT
# ============================================================

ep_robot = robot.Robot()

try:

    print("Connecting RoboMaster...")

    ep_robot.initialize(
        conn_type="ap"
    )

    chassis = ep_robot.chassis
    sensor = ep_robot.sensor_adaptor
    tof = ep_robot.sensor


    tof.sub_distance(
        freq=20,
        callback=tof_callback
    )


    print()
    print("==============================")
    print(" SIMPLE WALL AVOIDANCE")
    print("==============================")

    if ENABLE_MOTION:
        print("MOTION ENABLED")
    else:
        print("DRY RUN - ROBOT WILL NOT MOVE")
        print("Use: python wall_avoidance_simple.py --run")

    print()

    time.sleep(1)


    while True:

        # ====================================================
        # READ SHARP
        # ====================================================

        lf = read_sharp(sensor, LF_ID)
        lr = read_sharp(sensor, LR_ID)

        rf = read_sharp(sensor, RF_ID)
        rr = read_sharp(sensor, RR_ID)


        # ใช้ค่าที่ใกล้ที่สุด
        # เพราะถ้าหน้าหรือหลังด้านใดด้านหนึ่งใกล้กำแพง
        # เราต้องถือว่าอันตราย
        left_min = min(lf, lr)
        right_min = min(rf, rr)


        # ====================================================
        # COMMAND DEFAULT
        # ====================================================

        x = 0.0
        y = 0.0
        z = 0.0


        # ====================================================
        # FRONT SAFETY
        # ====================================================

        tof_valid = (
            front_cm is not None
            and
            time.time() - front_time < 1.0
        )


        if not tof_valid:

            state = STATE_FRONT_STOP


        elif state == STATE_FRONT_STOP:

            # ต้องโล่ง 13 cm ถึงเดินต่อ
            if front_cm >= FRONT_RELEASE_CM:

                state = STATE_FORWARD

            else:

                x = 0.0
                y = 0.0
                z = 0.0


        elif front_cm <= FRONT_STOP_CM:

            state = STATE_FRONT_STOP

            x = 0.0
            y = 0.0
            z = 0.0


        # ====================================================
        # TURN RIGHT STATE
        # ====================================================

        elif state == STATE_TURN_RIGHT:

            elapsed = time.time() - turn_start_time

            # ยังอยู่ใกล้กำแพง
            # หรือยังหมุนไม่ครบ minimum time
            if (
                left_min < SIDE_RELEASE_CM
                or
                elapsed < TURN_MIN_TIME
            ):

                x = 0.0
                y = 0.0

                if left_min <= SIDE_DANGER_CM:
                    z = -DANGER_TURN_SPEED
                else:
                    z = -TURN_SPEED

            else:

                state = STATE_FORWARD


        # ====================================================
        # TURN LEFT STATE
        # ====================================================

        elif state == STATE_TURN_LEFT:

            elapsed = time.time() - turn_start_time

            if (
                right_min < SIDE_RELEASE_CM
                or
                elapsed < TURN_MIN_TIME
            ):

                x = 0.0
                y = 0.0

                if right_min <= SIDE_DANGER_CM:
                    z = DANGER_TURN_SPEED
                else:
                    z = TURN_SPEED

            else:

                state = STATE_FORWARD


        # ====================================================
        # FORWARD
        # ====================================================

        if state == STATE_FORWARD:

            # -----------------------------------------------
            # BOTH SIDES TOO CLOSE
            # -----------------------------------------------

            if (
                left_min <= SIDE_TRIGGER_CM
                and
                right_min <= SIDE_TRIGGER_CM
            ):

                # เลือกหนีฝั่งที่ใกล้กว่า
                if left_min < right_min - 1.0:

                    state = STATE_TURN_RIGHT
                    turn_start_time = time.time()

                    x = 0.0
                    y = 0.0
                    z = -TURN_SPEED

                elif right_min < left_min - 1.0:

                    state = STATE_TURN_LEFT
                    turn_start_time = time.time()

                    x = 0.0
                    y = 0.0
                    z = TURN_SPEED

                else:

                    # แคบทั้งสองข้างพอ ๆ กัน
                    # อย่าตัดสินใจซ้ายขวารัว ๆ
                    x = 0.0
                    y = 0.0
                    z = 0.0


            # -----------------------------------------------
            # LEFT TOO CLOSE
            # -----------------------------------------------

            elif left_min <= SIDE_TRIGGER_CM:

                state = STATE_TURN_RIGHT
                turn_start_time = time.time()

                x = 0.0
                y = 0.0

                if left_min <= SIDE_DANGER_CM:
                    z = -DANGER_TURN_SPEED
                else:
                    z = -TURN_SPEED


            # -----------------------------------------------
            # RIGHT TOO CLOSE
            # -----------------------------------------------

            elif right_min <= SIDE_TRIGGER_CM:

                state = STATE_TURN_LEFT
                turn_start_time = time.time()

                x = 0.0
                y = 0.0

                if right_min <= SIDE_DANGER_CM:
                    z = DANGER_TURN_SPEED
                else:
                    z = TURN_SPEED


            # -----------------------------------------------
            # CLEAR -> GO STRAIGHT
            # -----------------------------------------------

            else:

                x = FORWARD_SPEED

                # สำคัญ:
                # ไม่มี strafe
                y = 0.0

                # แค่หมุนเล็กน้อยเพื่อให้ขนาน
                z = straight_heading_correction(
                    lf, lr,
                    rf, rr
                )


        # ====================================================
        # SEND
        # ====================================================

        if ENABLE_MOTION:

            chassis.drive_speed(
                x=x,
                y=y,
                z=z,
                timeout=0.3
            )


        # ====================================================
        # DISPLAY
        # ====================================================

        print(
            f"Front:{fmt(front_cm)} | "
            f"LF:{fmt(lf)} LR:{fmt(lr)} | "
            f"RF:{fmt(rf)} RR:{fmt(rr)} | "
            f"Lmin:{fmt(left_min)} "
            f"Rmin:{fmt(right_min)} | "
            f"x={x:+.2f} "
            f"y={y:+.2f} "
            f"z={z:+.1f} | "
            f"{state}"
        )


        time.sleep(0.05)


except KeyboardInterrupt:

    print()
    print("STOP")


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
        tof.unsub_distance()
    except:
        pass

    ep_robot.close()

    print("Robot stopped.")

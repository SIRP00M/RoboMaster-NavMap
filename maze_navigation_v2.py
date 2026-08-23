from robomaster import robot
from collections import deque
import statistics
import argparse
import time


# ============================================================
# ARGUMENT
# ============================================================

parser = argparse.ArgumentParser()

parser.add_argument(
    "--run",
    action="store_true",
    help="Enable chassis movement"
)

args = parser.parse_args()

ENABLE_MOTION = args.run


# ============================================================
# SENSOR POSITION
#
#                  FRONT / ToF
#                      ↓
#
#          LF  ┌──────────────┐  RF
#              │              │
#          LR  └──────────────┘  RR
#
# Sharp ทุกตัว = Port 1
# ============================================================

LF_ID = 1
LR_ID = 2
RF_ID = 3
RR_ID = 4

SHARP_PORT = 1


# ============================================================
# SHARP CALIBRATION
# จากค่าที่วัดจริง 5 / 10 / 15 / 20 / 25 / 30 cm
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
# SHARP FILTER
# ============================================================

MEDIAN_WINDOW = 5
EMA_ALPHA = 0.35

sharp_buffers = {
    1: deque(maxlen=MEDIAN_WINDOW),
    2: deque(maxlen=MEDIAN_WINDOW),
    3: deque(maxlen=MEDIAN_WINDOW),
    4: deque(maxlen=MEDIAN_WINDOW),
}

sharp_ema = {
    1: None,
    2: None,
    3: None,
    4: None,
}


# ============================================================
# NORMAL FORWARD
# ============================================================

FORWARD_SPEED = 0.15


# ============================================================
# FRONT NAVIGATION
#
# Front <= 20 cm
# ไม่เลี้ยวทันที
#
# ต้องเจอต่อเนื่อง 3 รอบก่อน
#
# จากนั้น:
#
# NAV_STOP
#     ↓
# รอ 0.30 sec
#     ↓
# NAV_DECIDE
#     ↓
# LEFT / RIGHT / UTURN
# ============================================================

FRONT_BLOCK_CM = 20.0

# ถ้าหลังหยุดแล้ว front กลับมาเกิน 23
# ยกเลิก navigation แล้วเดินต่อ
FRONT_CANCEL_CM = 23.0

FRONT_CONFIRM_FRAMES = 3

NAV_SETTLE_TIME = 0.30


# ============================================================
# SIDE WALL RECOVERY
#
# สำคัญ:
#
# Sharp ด้านข้างไม่มีสิทธิ์สั่ง 90°
#
# ถ้าใกล้กำแพง
# → เดินหน้าช้า
# → หมุนหัวออกเล็กน้อย
# → กลับ FORWARD
# ============================================================

SIDE_TRIGGER_CM = 10.0

# ต้องกลับออกมาถึง 12 cm
# ถึงเลิก recovery
SIDE_RELEASE_CM = 12.0

SIDE_DANGER_CM = 7.0


# ตอนกำลังแก้การเบี้ยว
RECOVERY_FORWARD_SPEED = 0.07

RECOVERY_TURN_SPEED = 12.0

RECOVERY_DANGER_TURN_SPEED = 20.0


# ไม่ยอมให้ติด State recovery ค้างตลอดไป
RECOVERY_TIMEOUT = 0.80


# ถ้าซ้าย-ขวาใกล้พอ ๆ กัน
# ถือว่าหุ่นอยู่กลาง corridor
SIDE_CENTER_TOLERANCE_CM = 1.5


# ============================================================
# PATH / JUNCTION DETECTION
#
# ตอน Front blocked เท่านั้นถึงใช้ค่านี้
# ============================================================

SIDE_OPEN_CM = 18.0

# ถ้าทั้งสองทางเปิดและต่างกันไม่ถึง 3cm
# ถือว่าพอ ๆ กัน
PATH_TIE_CM = 3.0


# ============================================================
# 90 / 180 TURN
#
# จากการทดสอบหุ่นจริง:
#
# +Z = RIGHT
# -Z = LEFT
# ============================================================

NAV_TURN_SPEED = 30.0

TURN_90_DEG = 90.0
TURN_180_DEG = 180.0

TURN_TOLERANCE_DEG = 3.0

POST_TURN_SETTLE = 0.35


# ============================================================
# STRAIGHT HEADING CORRECTION
#
# ไม่มี Y / strafe
#
# เดินตรงด้วย x
# แก้หัวเล็กน้อยด้วย z
# ============================================================

ANGLE_DEADBAND_CM = 2.0

ANGLE_KP = 1.2

STRAIGHT_CORRECTION_MAX = 5.0

# ใช้กำแพงช่วยจับ heading เฉพาะตอนอยู่ไม่ไกลเกิน
HEADING_WALL_MAX_CM = 20.0


# ============================================================
# STATES
# ============================================================

STATE_FORWARD = "FORWARD"

STATE_RECOVER_LEFT = "RECOVER_LEFT"
STATE_RECOVER_RIGHT = "RECOVER_RIGHT"

STATE_NAV_STOP = "NAV_STOP"
STATE_NAV_DECIDE = "NAV_DECIDE"

STATE_TURN_LEFT = "TURN_LEFT_90"
STATE_TURN_RIGHT = "TURN_RIGHT_90"
STATE_UTURN = "UTURN_180"

STATE_SETTLE = "SETTLE"

STATE_SENSOR_STOP = "SENSOR_STOP"


state = STATE_FORWARD


# ============================================================
# STATE VARIABLES
# ============================================================

front_block_counter = 0

recovery_start_time = 0.0

nav_stop_until = 0.0

settle_until = 0.0


# ============================================================
# TURN STATE VARIABLES
# ============================================================

turn_target = 0.0

turn_progress = 0.0

turn_last_yaw = None

turn_z = 0.0


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

        if mm is None:
            return

        if mm <= 0:
            return

        cm = mm / 10.0

        tof_buffer.append(cm)

        front_cm = statistics.median(
            tof_buffer
        )

        front_time = time.time()

    except Exception as e:

        print(
            "ToF callback error:",
            e
        )


# ============================================================
# ATTITUDE / YAW
# ============================================================

current_yaw = None

attitude_time = 0.0


def attitude_callback(data):

    global current_yaw
    global attitude_time

    try:

        # RoboMaster:
        # data = (yaw, pitch, roll)

        current_yaw = float(data[0])

        attitude_time = time.time()

    except Exception as e:

        print(
            "Attitude callback error:",
            e
        )


# ============================================================
# UTILITY
# ============================================================

def clamp(
    value,
    minimum,
    maximum
):

    return max(
        minimum,
        min(maximum, value)
    )


def fmt(value):

    if value is None:
        return "---"

    return f"{value:4.1f}"


# ============================================================
# YAW DELTA
#
# ป้องกันกรณี:
#
# 179 -> -179
#
# จริง ๆ หมุนแค่ 2°
# ============================================================

def angle_delta(
    new_angle,
    old_angle
):

    return (
        (
            new_angle
            -
            old_angle
            +
            180.0
        )
        %
        360.0
    ) - 180.0


# ============================================================
# ADC -> CM
#
# Piecewise Linear Interpolation
# ============================================================

def adc_to_cm(
    sensor_id,
    adc
):

    table = CALIBRATION[
        sensor_id
    ]

    # ใกล้กว่าช่วง calibration
    if adc >= table[0][0]:

        return 5.0


    # ไกลกว่าช่วง calibration
    if adc <= table[-1][0]:

        return 30.0


    for i in range(
        len(table) - 1
    ):

        adc1, cm1 = table[i]

        adc2, cm2 = table[i + 1]


        if (
            adc1
            >= adc
            >= adc2
        ):

            ratio = (
                (adc1 - adc)
                /
                (adc1 - adc2)
            )

            return (
                cm1
                +
                ratio
                *
                (cm2 - cm1)
            )


    return 30.0


# ============================================================
# READ + FILTER SHARP
# ============================================================

def read_sharp(
    sensor_adapter,
    sensor_id
):

    raw = sensor_adapter.get_adc(
        id=sensor_id,
        port=SHARP_PORT
    )


    sharp_buffers[
        sensor_id
    ].append(raw)


    median_adc = statistics.median(
        sharp_buffers[
            sensor_id
        ]
    )


    if sharp_ema[
        sensor_id
    ] is None:

        sharp_ema[
            sensor_id
        ] = median_adc


    else:

        sharp_ema[
            sensor_id
        ] = (

            EMA_ALPHA
            *
            median_adc

            +

            (1.0 - EMA_ALPHA)
            *
            sharp_ema[
                sensor_id
            ]
        )


    distance = adc_to_cm(

        sensor_id,

        sharp_ema[
            sensor_id
        ]
    )


    return distance


# ============================================================
# FRONT BLOCK CONFIRMATION
#
# ต้อง <=20cm ต่อเนื่อง 3 loop
# ถึงถือว่าตันจริง
# ============================================================

def front_is_really_blocked():

    global front_block_counter


    if front_cm is None:

        front_block_counter = 0

        return False


    if front_cm <= FRONT_BLOCK_CM:

        front_block_counter += 1


    else:

        front_block_counter = 0


    return (
        front_block_counter
        >=
        FRONT_CONFIRM_FRAMES
    )


def reset_front_confirmation():

    global front_block_counter

    front_block_counter = 0


# ============================================================
# SIDE OPENING
# ============================================================

def is_left_open(lf):

    return (
        lf is not None
        and
        lf >= SIDE_OPEN_CM
    )


def is_right_open(rf):

    return (
        rf is not None
        and
        rf >= SIDE_OPEN_CM
    )


# ============================================================
# START ROTATION
# ============================================================

def start_rotation(
    new_state,
    degrees,
    z_speed
):

    global state

    global turn_target
    global turn_progress
    global turn_last_yaw
    global turn_z


    state = new_state

    turn_target = degrees

    turn_progress = 0.0

    turn_last_yaw = current_yaw

    turn_z = z_speed


    reset_front_confirmation()


    print()

    print(
        f">>> START {state} "
        f"target={degrees:.0f} deg "
        f"z={z_speed:+.1f}"
    )


# ============================================================
# TURN FUNCTIONS
#
# จากของจริง:
#
# +Z = RIGHT
# -Z = LEFT
# ============================================================

def turn_left_90():

    start_rotation(

        STATE_TURN_LEFT,

        TURN_90_DEG,

        -NAV_TURN_SPEED
    )


def turn_right_90():

    start_rotation(

        STATE_TURN_RIGHT,

        TURN_90_DEG,

        +NAV_TURN_SPEED
    )


def uturn_180():

    # ตอนนี้เลือกหมุนกลับหัวทางขวา

    start_rotation(

        STATE_UTURN,

        TURN_180_DEG,

        +NAV_TURN_SPEED
    )


# ============================================================
# UPDATE ROTATION
#
# ไม่ใช้ sleep เดา 90°
#
# ใช้ Yaw ของ RoboMaster จริง
# ============================================================

def update_rotation():

    global state

    global turn_progress

    global turn_last_yaw

    global settle_until


    # ไม่มี Yaw
    if current_yaw is None:

        return (
            0.0,
            0.0,
            0.0
        )


    # Yaw stale
    if (
        time.time()
        -
        attitude_time
        >
        1.0
    ):

        return (
            0.0,
            0.0,
            0.0
        )


    if turn_last_yaw is None:

        turn_last_yaw = current_yaw

        return (
            0.0,
            0.0,
            turn_z
        )


    delta = angle_delta(

        current_yaw,

        turn_last_yaw
    )


    # ป้องกันค่ากระโดดผิดปกติ
    if abs(delta) < 30.0:

        turn_progress += abs(delta)


    turn_last_yaw = current_yaw


    # ========================================================
    # FINISHED
    # ========================================================

    if (
        turn_progress
        >=
        turn_target
        -
        TURN_TOLERANCE_DEG
    ):

        print(
            f">>> FINISHED {state} "
            f"{turn_progress:.1f} deg"
        )


        state = STATE_SETTLE


        settle_until = (
            time.time()
            +
            POST_TURN_SETTLE
        )


        return (
            0.0,
            0.0,
            0.0
        )


    return (
        0.0,
        0.0,
        turn_z
    )


# ============================================================
# NAVIGATION DECISION
#
# ฟังก์ชั่นนี้ถูกเรียกเฉพาะตอน:
#
# Front blocked confirmed
#        +
# หุ่นหยุดนิ่งแล้ว
#
# เท่านั้น
# ============================================================

def decide_path(
    lf,
    rf
):

    left_open = is_left_open(
        lf
    )

    right_open = is_right_open(
        rf
    )


    print()

    print(
        "========== PATH DECISION =========="
    )


    print(
        f"LEFT : {fmt(lf)} cm "
        +
        (
            "OPEN"
            if left_open
            else
            "BLOCKED"
        )
    )


    print(
        f"RIGHT: {fmt(rf)} cm "
        +
        (
            "OPEN"
            if right_open
            else
            "BLOCKED"
        )
    )


    # ========================================================
    # LEFT ONLY
    # ========================================================

    if (
        left_open
        and
        not right_open
    ):

        print(
            "DECISION -> LEFT 90"
        )

        turn_left_90()

        return


    # ========================================================
    # RIGHT ONLY
    # ========================================================

    if (
        right_open
        and
        not left_open
    ):

        print(
            "DECISION -> RIGHT 90"
        )

        turn_right_90()

        return


    # ========================================================
    # BOTH OPEN
    # ========================================================

    if (
        left_open
        and
        right_open
    ):

        difference = (
            lf - rf
        )


        # LEFT กว้างกว่า
        if difference > PATH_TIE_CM:

            print(
                "BOTH OPEN"
            )

            print(
                "LEFT IS WIDER"
            )

            print(
                "DECISION -> LEFT 90"
            )

            turn_left_90()

            return


        # RIGHT กว้างกว่า
        if difference < -PATH_TIE_CM:

            print(
                "BOTH OPEN"
            )

            print(
                "RIGHT IS WIDER"
            )

            print(
                "DECISION -> RIGHT 90"
            )

            turn_right_90()

            return


        # ใกล้เคียงกัน
        #
        # ตอนนี้กำหนด RIGHT PRIORITY
        print(
            "BOTH OPEN"
        )

        print(
            "SIMILAR WIDTH"
        )

        print(
            "RIGHT PRIORITY"
        )

        turn_right_90()

        return


    # ========================================================
    # DEAD END
    # ========================================================

    print(
        "DEAD END"
    )

    print(
        "DECISION -> UTURN 180"
    )

    uturn_180()


# ============================================================
# STRAIGHT HEADING CORRECTION
#
# ไม่มีการ Strafe
#
# y = 0 เสมอ
#
# ใช้ความต่าง Sharp หน้า/หลัง
# เพื่อแก้หัวหุ่นเบา ๆ
# ============================================================

def straight_heading_correction(
    lf,
    lr,
    rf,
    rr
):

    corrections = []


    # ========================================================
    # LEFT WALL
    #
    # LF < LR
    #
    # หัวเข้าใกล้กำแพงซ้าย
    #
    # ต้อง RIGHT
    #
    # +Z = RIGHT
    # ========================================================

    if (
        lf is not None
        and
        lr is not None
        and
        max(lf, lr)
        <
        HEADING_WALL_MAX_CM
    ):

        diff = (
            lf - lr
        )


        if (
            abs(diff)
            >
            ANGLE_DEADBAND_CM
        ):

            corrections.append(

                -ANGLE_KP
                *
                diff
            )


    # ========================================================
    # RIGHT WALL
    #
    # RF < RR
    #
    # หัวเข้าใกล้กำแพงขวา
    #
    # ต้อง LEFT
    #
    # -Z = LEFT
    # ========================================================

    if (
        rf is not None
        and
        rr is not None
        and
        max(rf, rr)
        <
        HEADING_WALL_MAX_CM
    ):

        diff = (
            rf - rr
        )


        if (
            abs(diff)
            >
            ANGLE_DEADBAND_CM
        ):

            corrections.append(

                +ANGLE_KP
                *
                diff
            )


    if not corrections:

        return 0.0


    z = (
        sum(corrections)
        /
        len(corrections)
    )


    return clamp(

        z,

        -STRAIGHT_CORRECTION_MAX,

        +STRAIGHT_CORRECTION_MAX
    )


# ============================================================
# ROBOT
# ============================================================

ep_robot = robot.Robot()


try:

    print(
        "Connecting RoboMaster..."
    )


    ep_robot.initialize(
        conn_type="ap"
    )


    chassis = ep_robot.chassis

    sensor_adapter = (
        ep_robot.sensor_adaptor
    )

    tof_sensor = (
        ep_robot.sensor
    )


    # ========================================================
    # SUBSCRIBE ToF
    # ========================================================

    tof_sensor.sub_distance(

        freq=20,

        callback=tof_callback
    )


    # ========================================================
    # SUBSCRIBE YAW
    # ========================================================

    chassis.sub_attitude(

        freq=20,

        callback=attitude_callback
    )


    print()

    print(
        "========================================"
    )

    print(
        " MAZE NAVIGATION V2"
    )

    print(
        "========================================"
    )

    print(
        "+Z = RIGHT"
    )

    print(
        "-Z = LEFT"
    )

    print(
        "Side Sharp = recovery only"
    )

    print(
        "Front ToF = navigation trigger"
    )

    print()


    if ENABLE_MOTION:

        print(
            "!!! MOTION ENABLED !!!"
        )


    else:

        print(
            "DRY RUN MODE"
        )

        print(
            "Robot WILL NOT MOVE"
        )

        print()

        print(
            "Run:"
        )

        print(
            "python maze_navigation_v2.py --run"
        )


    print()

    print(
        "Ctrl+C = STOP"
    )

    print()


    # รอ callback sensor
    time.sleep(1.0)


    # ========================================================
    # MAIN LOOP
    # ========================================================

    while True:

        now = time.time()


        # ====================================================
        # READ SHARP
        # ====================================================

        lf = read_sharp(
            sensor_adapter,
            LF_ID
        )

        lr = read_sharp(
            sensor_adapter,
            LR_ID
        )

        rf = read_sharp(
            sensor_adapter,
            RF_ID
        )

        rr = read_sharp(
            sensor_adapter,
            RR_ID
        )


        left_min = min(
            lf,
            lr
        )

        right_min = min(
            rf,
            rr
        )


        left_open = is_left_open(
            lf
        )

        right_open = is_right_open(
            rf
        )


        # ====================================================
        # DEFAULT COMMAND
        # ====================================================

        x = 0.0

        y = 0.0

        z = 0.0


        # ====================================================
        # SENSOR VALIDITY
        # ====================================================

        tof_valid = (

            front_cm
            is not None

            and

            now - front_time
            <
            1.0
        )


        attitude_valid = (

            current_yaw
            is not None

            and

            now - attitude_time
            <
            1.0
        )


        # ====================================================
        # PRIORITY 0:
        # ToF LOST
        # ====================================================

        if not tof_valid:

            state = STATE_SENSOR_STOP

            x = 0.0
            y = 0.0
            z = 0.0


        # ====================================================
        # ROTATING 90 / 180
        # ====================================================

        elif state in [

            STATE_TURN_LEFT,

            STATE_TURN_RIGHT,

            STATE_UTURN,

        ]:

            if not attitude_valid:

                x = 0.0
                y = 0.0
                z = 0.0

            else:

                (
                    x,
                    y,
                    z
                ) = update_rotation()


        # ====================================================
        # SETTLE AFTER TURN
        # ====================================================

        elif state == STATE_SETTLE:

            x = 0.0
            y = 0.0
            z = 0.0


            if now >= settle_until:

                reset_front_confirmation()

                state = STATE_FORWARD


        # ====================================================
        # NAVIGATION STOP
        #
        # Front blocked confirmed แล้ว
        #
        # หยุดนิ่งก่อนตัดสิน
        # ====================================================

        elif state == STATE_NAV_STOP:

            x = 0.0
            y = 0.0
            z = 0.0


            if now >= nav_stop_until:

                state = STATE_NAV_DECIDE


        # ====================================================
        # NAVIGATION DECISION
        # ====================================================

        elif state == STATE_NAV_DECIDE:

            x = 0.0
            y = 0.0
            z = 0.0


            # หลังหยุดแล้ว Front กลับมาโล่ง
            #
            # อาจเป็น noise
            # หรือหุ่นแค่เฉไปนิดเดียว
            #
            # ไม่ต้อง force turn
            if front_cm > FRONT_CANCEL_CM:

                print(
                    "NAV CANCEL -> FRONT CLEAR"
                )

                reset_front_confirmation()

                state = STATE_FORWARD


            else:

                decide_path(
                    lf,
                    rf
                )


        # ====================================================
        # RECOVER LEFT
        #
        # ซ้ายใกล้เกิน
        #
        # ไม่หมุน 90
        #
        # เดินหน้าช้า + หันขวาเบา ๆ
        # ====================================================

        elif state == STATE_RECOVER_LEFT:

            elapsed = (
                now
                -
                recovery_start_time
            )


            # -----------------------------------------------
            # Front obstacle มี priority สูงกว่า
            # -----------------------------------------------

            if front_is_really_blocked():

                state = STATE_NAV_STOP

                nav_stop_until = (
                    now
                    +
                    NAV_SETTLE_TIME
                )

                x = 0.0
                y = 0.0
                z = 0.0


            # -----------------------------------------------
            # กลับออกจากกำแพงแล้ว
            # -----------------------------------------------

            elif left_min >= SIDE_RELEASE_CM:

                state = STATE_FORWARD

                x = FORWARD_SPEED
                y = 0.0
                z = 0.0


            # -----------------------------------------------
            # Recovery นานเกิน
            #
            # ไม่ยอมค้าง State
            # -----------------------------------------------

            elif elapsed >= RECOVERY_TIMEOUT:

                state = STATE_FORWARD

                x = FORWARD_SPEED
                y = 0.0
                z = 0.0


            # -----------------------------------------------
            # Continue Recovery
            # -----------------------------------------------

            else:

                x = RECOVERY_FORWARD_SPEED

                y = 0.0


                # LEFT wall
                #
                # turn RIGHT
                #
                # +Z
                if left_min <= SIDE_DANGER_CM:

                    z = (
                        +RECOVERY_DANGER_TURN_SPEED
                    )

                else:

                    z = (
                        +RECOVERY_TURN_SPEED
                    )


        # ====================================================
        # RECOVER RIGHT
        # ====================================================

        elif state == STATE_RECOVER_RIGHT:

            elapsed = (
                now
                -
                recovery_start_time
            )


            if front_is_really_blocked():

                state = STATE_NAV_STOP

                nav_stop_until = (
                    now
                    +
                    NAV_SETTLE_TIME
                )

                x = 0.0
                y = 0.0
                z = 0.0


            elif right_min >= SIDE_RELEASE_CM:

                state = STATE_FORWARD

                x = FORWARD_SPEED
                y = 0.0
                z = 0.0


            elif elapsed >= RECOVERY_TIMEOUT:

                state = STATE_FORWARD

                x = FORWARD_SPEED
                y = 0.0
                z = 0.0


            else:

                x = RECOVERY_FORWARD_SPEED

                y = 0.0


                # RIGHT wall
                #
                # turn LEFT
                #
                # -Z
                if right_min <= SIDE_DANGER_CM:

                    z = (
                        -RECOVERY_DANGER_TURN_SPEED
                    )

                else:

                    z = (
                        -RECOVERY_TURN_SPEED
                    )


        # ====================================================
        # SENSOR RECOVER
        # ====================================================

        elif state == STATE_SENSOR_STOP:

            x = 0.0
            y = 0.0
            z = 0.0


            if tof_valid:

                reset_front_confirmation()

                state = STATE_FORWARD


        # ====================================================
        # FORWARD
        # ====================================================

        elif state == STATE_FORWARD:

            # =================================================
            # PRIORITY 1
            #
            # FRONT BLOCKED
            #
            # ต้อง confirm ก่อน
            # =================================================

            if front_is_really_blocked():

                print()

                print(
                    "FRONT BLOCKED CONFIRMED"
                )

                print(
                    f"Front = {front_cm:.1f} cm"
                )

                print(
                    "STATE -> NAV_STOP"
                )


                state = STATE_NAV_STOP


                nav_stop_until = (
                    now
                    +
                    NAV_SETTLE_TIME
                )


                x = 0.0
                y = 0.0
                z = 0.0


            # =================================================
            # PRIORITY 2
            #
            # BOTH SIDE CLOSE
            # =================================================

            elif (
                left_min <= SIDE_TRIGGER_CM
                and
                right_min <= SIDE_TRIGGER_CM
            ):

                difference = (
                    left_min
                    -
                    right_min
                )


                # ---------------------------------------------
                # อยู่กลางช่องพอดี
                #
                # ไปตรงเลย
                # ---------------------------------------------

                if (
                    abs(difference)
                    <=
                    SIDE_CENTER_TOLERANCE_CM
                ):

                    x = FORWARD_SPEED

                    y = 0.0

                    z = 0.0


                # ---------------------------------------------
                # LEFT closer
                # ---------------------------------------------

                elif difference < 0:

                    state = STATE_RECOVER_LEFT

                    recovery_start_time = now


                    x = RECOVERY_FORWARD_SPEED

                    y = 0.0

                    z = +RECOVERY_TURN_SPEED


                # ---------------------------------------------
                # RIGHT closer
                # ---------------------------------------------

                else:

                    state = STATE_RECOVER_RIGHT

                    recovery_start_time = now


                    x = RECOVERY_FORWARD_SPEED

                    y = 0.0

                    z = -RECOVERY_TURN_SPEED


            # =================================================
            # LEFT TOO CLOSE
            # =================================================

            elif left_min <= SIDE_TRIGGER_CM:

                state = STATE_RECOVER_LEFT

                recovery_start_time = now


                x = RECOVERY_FORWARD_SPEED

                y = 0.0


                if left_min <= SIDE_DANGER_CM:

                    z = (
                        +RECOVERY_DANGER_TURN_SPEED
                    )

                else:

                    z = (
                        +RECOVERY_TURN_SPEED
                    )


            # =================================================
            # RIGHT TOO CLOSE
            # =================================================

            elif right_min <= SIDE_TRIGGER_CM:

                state = STATE_RECOVER_RIGHT

                recovery_start_time = now


                x = RECOVERY_FORWARD_SPEED

                y = 0.0


                if right_min <= SIDE_DANGER_CM:

                    z = (
                        -RECOVERY_DANGER_TURN_SPEED
                    )

                else:

                    z = (
                        -RECOVERY_TURN_SPEED
                    )


            # =================================================
            # NORMAL CORRIDOR
            # =================================================

            else:

                x = FORWARD_SPEED

                # ไม่มี Strafe
                y = 0.0


                z = straight_heading_correction(

                    lf,
                    lr,

                    rf,
                    rr
                )


        # ====================================================
        # SEND COMMAND
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

        turn_text = ""


        if state in [

            STATE_TURN_LEFT,

            STATE_TURN_RIGHT,

            STATE_UTURN,

        ]:

            turn_text = (

                f" | TURN="
                f"{turn_progress:5.1f}"
                f"/"
                f"{turn_target:5.1f}"
            )


        print(

            f"STATE:{state:16s} | "

            f"Front:{fmt(front_cm)} | "

            f"LF:{fmt(lf)} "
            f"LR:{fmt(lr)} | "

            f"RF:{fmt(rf)} "
            f"RR:{fmt(rr)} | "

            f"Lmin:{fmt(left_min)} "
            f"Rmin:{fmt(right_min)} | "

            f"L:{'OPEN' if left_open else 'WALL':4s} "
            f"R:{'OPEN' if right_open else 'WALL':4s} | "

            f"x={x:+.2f} "
            f"y={y:+.2f} "
            f"z={z:+.1f}"

            f"{turn_text}"
        )


        time.sleep(0.05)


# ============================================================
# CTRL+C
# ============================================================

except KeyboardInterrupt:

    print()

    print(
        "STOP REQUESTED"
    )


# ============================================================
# ERROR
# ============================================================

except Exception as e:

    print()

    print(
        "ERROR:",
        e
    )


# ============================================================
# CLEANUP
# ============================================================

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


    try:

        chassis.unsub_attitude()

    except:
        pass


    ep_robot.close()


    print(
        "Robot stopped and disconnected."
    )

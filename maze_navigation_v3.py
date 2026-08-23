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
#
#                  FRONT / ToF
#                       ↓
#
#           LF  ┌────────────┐  RF
#               │   ROBOT    │
#           LR  └────────────┘  RR
#
# ============================================================

LF_ID = 1
LR_ID = 2
RF_ID = 3
RR_ID = 4

SHARP_PORT = 1


# ============================================================
# SHARP CALIBRATION
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
# NORMAL SPEED
# ============================================================

FORWARD_SPEED = 0.15


# ============================================================
# FRONT NAVIGATION
#
# <= 20 cm ต่อเนื่อง 3 loop
# -> NAV_STOP
# -> รอ 0.30 sec
# -> NAV_SCAN 1 sec
# -> NAV_DECIDE
# ============================================================

FRONT_BLOCK_CM = 20.0
FRONT_CANCEL_CM = 23.0

FRONT_CONFIRM_FRAMES = 3

NAV_STOP_TIME = 0.30


# ============================================================
# NAVIGATION SCAN
#
# หยุดนิ่งแล้วเก็บ Sharp ซ้าย/ขวาหลาย sample
#
# >= 18 cm = OPEN sample
# ต้อง OPEN >= 80% ถึงยืนยันว่าเป็นทางจริง
# ============================================================

SIDE_OPEN_CM = 18.0

NAV_SCAN_TIME = 1.0
OPEN_CONFIRM_RATIO = 0.80

# ถ้าทั้งสองด้าน OPEN และ median ต่างกัน <= 3cm
# ถือว่าใกล้เคียงกัน
PATH_TIE_CM = 3.0


nav_scan_left = []
nav_scan_right = []

nav_scan_until = 0.0


# ============================================================
# SIDE WALL RECOVERY
#
# Sharp ด้านข้าง:
#
# ห้ามสั่ง 90 / 180
#
# มีหน้าที่:
# - ป้องกันชนกำแพง
# - แก้หัวรถ
# ============================================================

SIDE_TRIGGER_CM = 10.0
SIDE_RELEASE_CM = 12.0
SIDE_DANGER_CM = 7.0

RECOVERY_FORWARD_SPEED = 0.07

RECOVERY_TURN_SPEED = 12.0
RECOVERY_DANGER_TURN_SPEED = 20.0

RECOVERY_TIMEOUT = 0.80

SIDE_CENTER_TOLERANCE_CM = 1.5


# ============================================================
# 90 / 180 DEGREE TURN
#
# จากหุ่นจริง:
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
# ============================================================

ANGLE_DEADBAND_CM = 2.0
ANGLE_KP = 1.2

STRAIGHT_CORRECTION_MAX = 5.0

HEADING_WALL_MAX_CM = 20.0


# ============================================================
# STATES
# ============================================================

STATE_FORWARD = "FORWARD"

STATE_RECOVER_LEFT = "RECOVER_LEFT"
STATE_RECOVER_RIGHT = "RECOVER_RIGHT"

STATE_NAV_STOP = "NAV_STOP"
STATE_NAV_SCAN = "NAV_SCAN"
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
# TURN STATE
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

        if mm is None or mm <= 0:
            return

        cm = mm / 10.0

        tof_buffer.append(cm)

        front_cm = statistics.median(
            tof_buffer
        )

        front_time = time.time()

    except Exception as e:

        print("ToF callback error:", e)


# ============================================================
# YAW
# ============================================================

current_yaw = None
attitude_time = 0.0


def attitude_callback(data):

    global current_yaw
    global attitude_time

    try:

        current_yaw = float(
            data[0]
        )

        attitude_time = time.time()

    except Exception as e:

        print(
            "Attitude callback error:",
            e
        )


# ============================================================
# UTILITIES
# ============================================================

def clamp(value, minimum, maximum):

    return max(
        minimum,
        min(maximum, value)
    )


def fmt(value):

    if value is None:
        return "---"

    return f"{value:4.1f}"


def angle_delta(new_angle, old_angle):

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
# ============================================================

def adc_to_cm(sensor_id, adc):

    table = CALIBRATION[
        sensor_id
    ]

    if adc >= table[0][0]:
        return 5.0

    if adc <= table[-1][0]:
        return 30.0

    for i in range(
        len(table) - 1
    ):

        adc1, cm1 = table[i]
        adc2, cm2 = table[i + 1]

        if adc1 >= adc >= adc2:

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
# SHARP READ
# ============================================================

def read_sharp(sensor_adapter, sensor_id):

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

    return adc_to_cm(

        sensor_id,

        sharp_ema[
            sensor_id
        ]
    )


# ============================================================
# FRONT CONFIRMATION
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
# SIDE OPEN
# ============================================================

def is_left_open(value):

    return (
        value is not None
        and
        value >= SIDE_OPEN_CM
    )


def is_right_open(value):

    return (
        value is not None
        and
        value >= SIDE_OPEN_CM
    )


# ============================================================
# NAV SCAN
# ============================================================

def start_nav_scan(now):

    global state
    global nav_scan_until

    nav_scan_left.clear()
    nav_scan_right.clear()

    nav_scan_until = (
        now
        +
        NAV_SCAN_TIME
    )

    state = STATE_NAV_SCAN

    print()
    print(">>> NAV SCAN START")


def confirmed_open(samples):

    valid = [
        value
        for value in samples
        if value is not None
    ]

    if not valid:
        return False

    open_count = sum(

        1
        for value in valid

        if value >= SIDE_OPEN_CM
    )

    ratio = (
        open_count
        /
        len(valid)
    )

    return (
        ratio
        >=
        OPEN_CONFIRM_RATIO
    )


def open_count(samples):

    valid = [
        value
        for value in samples
        if value is not None
    ]

    count = sum(

        1
        for value in valid

        if value >= SIDE_OPEN_CM
    )

    return (
        count,
        len(valid)
    )


def safe_median(samples):

    valid = [
        value
        for value in samples
        if value is not None
    ]

    if not valid:
        return None

    return statistics.median(
        valid
    )


# ============================================================
# ROTATION
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
        f">>> START {state} | "
        f"target={degrees:.0f} deg | "
        f"z={z_speed:+.1f}"
    )


# ============================================================
# TURN FUNCTIONS
#
# +Z RIGHT
# -Z LEFT
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

    # เลือก U-Turn ทางขวา

    start_rotation(
        STATE_UTURN,
        TURN_180_DEG,
        +NAV_TURN_SPEED
    )


# ============================================================
# UPDATE TURN
# ============================================================

def update_rotation():

    global state
    global turn_progress
    global turn_last_yaw
    global settle_until

    if current_yaw is None:

        return 0.0, 0.0, 0.0

    if (
        time.time()
        -
        attitude_time
        >
        1.0
    ):

        return 0.0, 0.0, 0.0

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

    # ป้องกัน yaw jump
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
            f">>> FINISHED {state} | "
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
# PATH DECISION
#
# ใช้ NAV SCAN หลาย sample
# ไม่ใช้ค่าครั้งเดียว
# ============================================================

def decide_path():

    left_value = safe_median(
        nav_scan_left
    )

    right_value = safe_median(
        nav_scan_right
    )

    left_open = confirmed_open(
        nav_scan_left
    )

    right_open = confirmed_open(
        nav_scan_right
    )

    left_count, left_total = open_count(
        nav_scan_left
    )

    right_count, right_total = open_count(
        nav_scan_right
    )


    print()

    print(
        "========== PATH CONFIRMATION =========="
    )


    print(
        f"LEFT : median={fmt(left_value)} cm | "
        f"OPEN={left_count}/{left_total} | "
        f"{'CONFIRMED OPEN' if left_open else 'BLOCKED'}"
    )


    print(
        f"RIGHT: median={fmt(right_value)} cm | "
        f"OPEN={right_count}/{right_total} | "
        f"{'CONFIRMED OPEN' if right_open else 'BLOCKED'}"
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

        # safety fallback
        if (
            left_value is None
            or
            right_value is None
        ):

            print(
                "SCAN INVALID -> RIGHT PRIORITY"
            )

            turn_right_90()

            return


        difference = (
            left_value
            -
            right_value
        )


        if difference > PATH_TIE_CM:

            print(
                "BOTH OPEN -> LEFT WIDER"
            )

            print(
                "DECISION -> LEFT 90"
            )

            turn_left_90()

            return


        if difference < -PATH_TIE_CM:

            print(
                "BOTH OPEN -> RIGHT WIDER"
            )

            print(
                "DECISION -> RIGHT 90"
            )

            turn_right_90()

            return


        print(
            "BOTH OPEN -> SIMILAR"
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
        "LEFT BLOCKED + RIGHT BLOCKED"
    )

    print(
        "DECISION -> UTURN 180"
    )

    uturn_180()


# ============================================================
# STRAIGHT HEADING CORRECTION
#
# ไม่มี Y
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
    # หัวเข้าใกล้ซ้าย
    # ต้องหมุนขวา
    #
    # +Z
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
    # หัวเข้าใกล้ขวา
    # ต้องหมุนซ้าย
    #
    # -Z
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


    value = (
        sum(corrections)
        /
        len(corrections)
    )


    return clamp(

        value,

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
    # SUBSCRIBE SENSOR
    # ========================================================

    tof_sensor.sub_distance(
        freq=20,
        callback=tof_callback
    )


    chassis.sub_attitude(
        freq=20,
        callback=attitude_callback
    )


    print()

    print(
        "======================================"
    )

    print(
        " MAZE NAVIGATION V3"
    )

    print(
        "======================================"
    )

    print(
        "+Z = RIGHT"
    )

    print(
        "-Z = LEFT"
    )

    print(
        "Front <= 20cm -> Confirm -> Scan"
    )

    print(
        "NAV Scan = 1.0 sec"
    )

    print(
        "OPEN confirm >= 80%"
    )

    print(
        "Side Sharp = recovery only"
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
            "python maze_navigation_v3.py --run"
        )


    print()

    print(
        "Ctrl+C = STOP"
    )

    print()


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


        # ====================================================
        # DISPLAY ONLY
        # ====================================================

        left_live_open = is_left_open(
            lf
        )

        right_live_open = is_right_open(
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
        # PRIORITY 0 - SENSOR LOST
        # ====================================================

        if not tof_valid:

            state = STATE_SENSOR_STOP

            x = 0.0
            y = 0.0
            z = 0.0


        # ====================================================
        # ROTATING
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

                x, y, z = update_rotation()


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
        # NAV STOP
        # ====================================================

        elif state == STATE_NAV_STOP:

            x = 0.0
            y = 0.0
            z = 0.0


            if now >= nav_stop_until:

                # หลังหยุด ถ้าหน้ากลับมาโล่งมาก
                # ยกเลิก navigation
                if front_cm > FRONT_CANCEL_CM:

                    print()

                    print(
                        "NAV CANCEL -> FRONT CLEAR"
                    )

                    reset_front_confirmation()

                    state = STATE_FORWARD

                else:

                    start_nav_scan(now)


        # ====================================================
        # NAV SCAN
        #
        # หุ่นอยู่นิ่ง
        # เก็บ sample ซ้ายขวา
        # ====================================================

        elif state == STATE_NAV_SCAN:

            x = 0.0
            y = 0.0
            z = 0.0


            nav_scan_left.append(
                lf
            )

            nav_scan_right.append(
                rf
            )


            if now >= nav_scan_until:

                print(
                    f">>> NAV SCAN FINISHED | "
                    f"samples={len(nav_scan_left)}"
                )

                state = STATE_NAV_DECIDE


        # ====================================================
        # NAV DECIDE
        # ====================================================

        elif state == STATE_NAV_DECIDE:

            x = 0.0
            y = 0.0
            z = 0.0


            # เช็ก front อีกรอบสุดท้าย
            if front_cm > FRONT_CANCEL_CM:

                print()

                print(
                    "NAV DECISION CANCELLED"
                )

                print(
                    "FRONT IS CLEAR"
                )

                reset_front_confirmation()

                state = STATE_FORWARD

            else:

                decide_path()


        # ====================================================
        # RECOVER LEFT
        # ====================================================

        elif state == STATE_RECOVER_LEFT:

            elapsed = (
                now
                -
                recovery_start_time
            )


            # Front มี priority สูงกว่า
            if front_is_really_blocked():

                print()

                print(
                    "FRONT BLOCKED DURING LEFT RECOVERY"
                )

                state = STATE_NAV_STOP

                nav_stop_until = (
                    now
                    +
                    NAV_STOP_TIME
                )

                x = 0.0
                y = 0.0
                z = 0.0


            elif left_min >= SIDE_RELEASE_CM:

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

                print()

                print(
                    "FRONT BLOCKED DURING RIGHT RECOVERY"
                )

                state = STATE_NAV_STOP

                nav_stop_until = (
                    now
                    +
                    NAV_STOP_TIME
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


                if right_min <= SIDE_DANGER_CM:

                    z = (
                        -RECOVERY_DANGER_TURN_SPEED
                    )

                else:

                    z = (
                        -RECOVERY_TURN_SPEED
                    )


        # ====================================================
        # SENSOR RECOVERY
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
            # PRIORITY 1:
            # FRONT
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
                    NAV_STOP_TIME
                )


                x = 0.0
                y = 0.0
                z = 0.0


            # =================================================
            # BOTH SIDES CLOSE
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


                # อยู่กลางทางแคบ
                if (
                    abs(difference)
                    <=
                    SIDE_CENTER_TOLERANCE_CM
                ):

                    x = FORWARD_SPEED
                    y = 0.0
                    z = 0.0


                # ซ้ายใกล้กว่า
                elif difference < 0:

                    state = STATE_RECOVER_LEFT

                    recovery_start_time = now


                    x = RECOVERY_FORWARD_SPEED
                    y = 0.0
                    z = +RECOVERY_TURN_SPEED


                # ขวาใกล้กว่า
                else:

                    state = STATE_RECOVER_RIGHT

                    recovery_start_time = now


                    x = RECOVERY_FORWARD_SPEED
                    y = 0.0
                    z = -RECOVERY_TURN_SPEED


            # =================================================
            # LEFT CLOSE
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
            # RIGHT CLOSE
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
            # NORMAL
            # =================================================

            else:

                x = FORWARD_SPEED

                # no strafe
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

        extra = ""


        if state in [

            STATE_TURN_LEFT,
            STATE_TURN_RIGHT,
            STATE_UTURN,

        ]:

            extra += (

                f" | TURN="
                f"{turn_progress:5.1f}"
                f"/"
                f"{turn_target:5.1f}"
            )


        if state == STATE_NAV_SCAN:

            extra += (

                f" | SCAN="
                f"{len(nav_scan_left)}"
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

            f"L:{'OPEN' if left_live_open else 'WALL':4s} "
            f"R:{'OPEN' if right_live_open else 'WALL':4s} | "

            f"x={x:+.2f} "
            f"y={y:+.2f} "
            f"z={z:+.1f}"

            f"{extra}"
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


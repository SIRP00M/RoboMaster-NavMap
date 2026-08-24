import time
import statistics

from collections import deque
from robomaster import robot


# ============================================================
# CONFIGURATION & PARAMETERS
# ============================================================

ENABLE_MOTION = True


# ============================================================
# SENSOR CONFIG
# ============================================================

IR_LEFT_FRONT_ID = 1

SHARP_LEFT_ID = 2
SHARP_RIGHT_ID = 3

SENSOR_PORT = 1


# ============================================================
# DISTANCE CONFIG
# ============================================================

# ระยะเป้าหมายจากกำแพงกรณีมีกำแพงเพียงข้างเดียว
TARGET_LEFT_CM = 8.0
TARGET_RIGHT_CM = 8.0

# เริ่มชะลอด้านหน้า
SLOW_FRONT_CM = 18.0

# หยุดด้านหน้า
STOP_FRONT_CM = 15.0

# ใช้ตัดสิน Dead End
SIDE_DEAD_END_CM = 15.0

# ต่ำกว่านี้ถือว่ามีกำแพงด้านข้าง
SIDE_WALL_DETECT_CM = 30.0

# ต่ำกว่านี้หนีออกทันที
SIDE_TOO_CLOSE_CM = 5.5


# ============================================================
# ROBOT SPEED
# ============================================================

FORWARD_SPEED = 0.15
MIN_FORWARD_SPEED = 0.05


# ============================================================
# DIRECTION
# ============================================================

# ถ้าสไลด์ซ้าย/ขวากลับทิศ เปลี่ยนเป็น -1
Y_DIR_SIGN = 1

# ถ้าหมุนซ้าย/ขวากลับทิศ เปลี่ยนเป็น -1
Z_DIR_SIGN = 1


# ============================================================
# SINGLE WALL CONTROLLER
# ============================================================

# ใช้ตอนมีกำแพงแค่ซ้ายหรือขวา
SIDE_KP_STRAFE = 0.015

SIDE_MAX_Y = 0.08

SIDE_DEADBAND_CM = 1.0


# ============================================================
# BOTH-WALL / OWNER CONTROLLER
# ============================================================

# ต้องต่างกันอย่างน้อยเท่านี้ถึงเริ่มแก้
CENTER_TRIGGER_CM = 2.0

# กลับมาเหลื่อมน้อยกว่านี้ จึงปล่อย Owner
CENTER_RELEASE_CM = 0.7

# Owner อย่างน้อยต้องถือการควบคุมเท่านี้
CENTER_HOLD_SEC = 0.30

# Gain สำหรับการจัดกลาง
CENTER_KP_STRAFE = 0.012

# จำกัดความเร็วสไลด์ตอนจัดกลาง
CENTER_MAX_Y = 0.07


# ============================================================
# ESCAPE CONFIG
# ============================================================

ESCAPE_Y_SPEED = 0.10


# ============================================================
# TURN CONFIG
# ============================================================

TURN_SPEED = 60

TURN_LEFT_DEG = 90
TURN_RIGHT_DEG = -90
TURN_AROUND_DEG = -180


# ============================================================
# SHARP CALIBRATION
# ADC -> CM
# ============================================================

CALIBRATION_SHARP2 = [
    (675, 5.0),
    (343, 10.0),
    (236, 15.0),
    (166, 20.0),
    (126, 25.0),
    (105, 30.0),
    (50, 80.0)
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
# OWNER STATE
# ============================================================

side_owner = "NONE"
side_owner_since = 0.0


# ============================================================
# TOF CALLBACK
# ============================================================

def tof_callback(data):

    global front_cm

    try:

        if not data:
            return

        if data[0] is None:
            return

        mm = data[0]

        # Reject invalid values
        if mm < 20 or mm > 4000:
            return

        cm = mm / 10.0

        tof_buffer.append(cm)

        front_cm = statistics.median(tof_buffer)

    except Exception as e:

        print("ToF callback error:", e)


# ============================================================
# ADC -> CM
# ============================================================

def adc_to_cm(adc):

    table = CALIBRATION_SHARP2

    if adc >= table[0][0]:
        return 5.0

    if adc <= table[-1][0]:
        return 80.0

    for i in range(len(table) - 1):

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
                + ratio * (cm2 - cm1)
            )

    return 80.0


# ============================================================
# READ SHARP
# ============================================================

def read_sharp_raw_and_cm(sensor_adapter, sensor_id):

    global sharp_left_ema
    global sharp_right_ema

    raw = sensor_adapter.get_adc(
        id=sensor_id,
        port=SENSOR_PORT
    )

    # --------------------------------------------------------
    # LEFT
    # --------------------------------------------------------

    if sensor_id == SHARP_LEFT_ID:

        sharp_left_buffer.append(raw)

        median_adc = statistics.median(
            sharp_left_buffer
        )

        if sharp_left_ema is None:

            sharp_left_ema = median_adc

        else:

            sharp_left_ema = (
                0.6 * median_adc
                + 0.4 * sharp_left_ema
            )

        ema_val = sharp_left_ema

    # --------------------------------------------------------
    # RIGHT
    # --------------------------------------------------------

    else:

        sharp_right_buffer.append(raw)

        median_adc = statistics.median(
            sharp_right_buffer
        )

        if sharp_right_ema is None:

            sharp_right_ema = median_adc

        else:

            sharp_right_ema = (
                0.6 * median_adc
                + 0.4 * sharp_right_ema
            )

        ema_val = sharp_right_ema

    cm = adc_to_cm(ema_val)

    return raw, cm


# ============================================================
# READ IR
# ============================================================

def read_ir_digital_io(sensor_adapter):

    try:

        return sensor_adapter.get_io_level(
            id=IR_LEFT_FRONT_ID,
            port=SENSOR_PORT
        )

    except Exception:

        raw = sensor_adapter.get_adc(
            id=IR_LEFT_FRONT_ID,
            port=SENSOR_PORT
        )

        return 1 if raw > 300 else 0


# ============================================================
# UTILITY
# ============================================================

def clamp(val, min_val, max_val):

    return max(
        min_val,
        min(max_val, val)
    )


def fmt(val):

    if val is None:
        return "---"

    return f"{val:4.1f}"


# ============================================================
# RESET OWNER
# ============================================================

def reset_side_owner():

    global side_owner
    global side_owner_since

    side_owner = "NONE"
    side_owner_since = 0.0


# ============================================================
# FRONT SPEED CONTROL
# ============================================================

def calculate_forward_speed(front_distance):

    """
    >= 18 cm
        0.15 m/s

    18 -> 15 cm
        Linear slowdown

    <= 15 cm
        0
    """

    if front_distance is None:

        return FORWARD_SPEED


    if front_distance >= SLOW_FRONT_CM:

        return FORWARD_SPEED


    if front_distance <= STOP_FRONT_CM:

        return 0.0


    ratio = (
        (front_distance - STOP_FRONT_CM)
        /
        (SLOW_FRONT_CM - STOP_FRONT_CM)
    )


    speed = (
        MIN_FORWARD_SPEED
        + ratio
        * (FORWARD_SPEED - MIN_FORWARD_SPEED)
    )


    return speed


# ============================================================
# BOTH WALL CONTROLLER
# ============================================================

def calculate_center_owner(
    sharp_left_cm,
    sharp_right_cm
):

    """
    BOTH WALL MODE

    delta = Left - Right

    delta < 0
        ซ้ายใกล้กว่า
        -> LEFT_OWNER
        -> สไลด์ขวา

    delta > 0
        ขวาใกล้กว่า
        -> RIGHT_OWNER
        -> สไลด์ซ้าย

    มี Hysteresis + Hold Time
    เพื่อไม่สลับ L/R รัว ๆ
    """

    global side_owner
    global side_owner_since


    now = time.time()

    delta = (
        sharp_left_cm
        - sharp_right_cm
    )

    abs_delta = abs(delta)


    # ========================================================
    # NO OWNER
    # ========================================================

    if side_owner == "NONE":

        # ยังกลางพอ -> ไม่ต้องทำอะไร
        if abs_delta < CENTER_TRIGGER_CM:

            return (
                0.0,
                "CENTER_STABLE"
            )


        # ซ้ายใกล้กว่า
        if delta < 0:

            side_owner = "LEFT"
            side_owner_since = now

        # ขวาใกล้กว่า
        else:

            side_owner = "RIGHT"
            side_owner_since = now


    # ========================================================
    # OWNER EXISTS
    # ========================================================

    owner_age = (
        now
        - side_owner_since
    )


    # --------------------------------------------------------
    # Hold Time
    # --------------------------------------------------------

    if owner_age >= CENTER_HOLD_SEC:

        # กลับมากลางแล้ว
        if abs_delta <= CENTER_RELEASE_CM:

            reset_side_owner()

            return (
                0.0,
                "CENTER_RELEASE"
            )


        # ----------------------------------------------------
        # LEFT OWNER แต่ตอนนี้ขวาใกล้กว่าชัดเจน
        # ----------------------------------------------------

        if (
            side_owner == "LEFT"
            and
            delta >= CENTER_TRIGGER_CM
        ):

            side_owner = "RIGHT"
            side_owner_since = now


        # ----------------------------------------------------
        # RIGHT OWNER แต่ตอนนี้ซ้ายใกล้กว่าชัดเจน
        # ----------------------------------------------------

        elif (
            side_owner == "RIGHT"
            and
            delta <= -CENTER_TRIGGER_CM
        ):

            side_owner = "LEFT"
            side_owner_since = now


    # ========================================================
    # COMMAND
    # ========================================================

    correction = clamp(

        abs_delta
        * CENTER_KP_STRAFE,

        0.0,
        CENTER_MAX_Y
    )


    # --------------------------------------------------------
    # LEFT WALL IS CLOSER
    # -> move RIGHT
    # --------------------------------------------------------

    if side_owner == "LEFT":

        y_cmd = (
            +correction
            * Y_DIR_SIGN
        )

        return (
            y_cmd,
            "CENTER_LEFT_OWNER"
        )


    # --------------------------------------------------------
    # RIGHT WALL IS CLOSER
    # -> move LEFT
    # --------------------------------------------------------

    elif side_owner == "RIGHT":

        y_cmd = (
            -correction
            * Y_DIR_SIGN
        )

        return (
            y_cmd,
            "CENTER_RIGHT_OWNER"
        )


    return (
        0.0,
        "CENTER_STABLE"
    )


# ============================================================
# SIDE MOTION CONTROLLER
# ============================================================

def calculate_motion_control(
    raw_adc_l,
    sharp_left_cm,
    raw_adc_r,
    sharp_right_cm,
    ir_left_wall
):

    """
    Sharp sensors คุมเฉพาะ Y

    z = 0 ระหว่างการเดินตามกำแพง

    ป้องกัน Sharp Noise ทำให้รถหมุนซ้าย-ขวา
    """


    if (
        sharp_left_cm is None
        or
        sharp_right_cm is None
    ):

        reset_side_owner()

        return (
            0.0,
            0.0,
            "NO_SENSOR"
        )


    # ========================================================
    # BOTH SIDES TOO CLOSE
    # ========================================================

    if (
        sharp_left_cm <= SIDE_TOO_CLOSE_CM
        and
        sharp_right_cm <= SIDE_TOO_CLOSE_CM
    ):

        reset_side_owner()

        # อย่าเลือกซ้าย/ขวาแบบสุ่ม
        return (
            0.0,
            0.0,
            "BOTH_TOO_CLOSE"
        )


    # ========================================================
    # LEFT TOO CLOSE
    # ========================================================

    if (
        raw_adc_l >= 600
        or
        sharp_left_cm <= SIDE_TOO_CLOSE_CM
    ):

        reset_side_owner()

        # หนีขวา
        y_cmd = (
            +ESCAPE_Y_SPEED
            * Y_DIR_SIGN
        )

        return (
            y_cmd,
            0.0,
            "ESCAPE_LEFT"
        )


    # ========================================================
    # RIGHT TOO CLOSE
    # ========================================================

    if (
        raw_adc_r >= 600
        or
        sharp_right_cm <= SIDE_TOO_CLOSE_CM
    ):

        reset_side_owner()

        # หนีซ้าย
        y_cmd = (
            -ESCAPE_Y_SPEED
            * Y_DIR_SIGN
        )

        return (
            y_cmd,
            0.0,
            "ESCAPE_RIGHT"
        )


    # ========================================================
    # WALL EXISTENCE
    # ========================================================

    left_wall = (
        sharp_left_cm
        < SIDE_WALL_DETECT_CM
    )

    right_wall = (
        sharp_right_cm
        < SIDE_WALL_DETECT_CM
    )


    # ========================================================
    # BOTH WALL
    # ========================================================

    if left_wall and right_wall:

        y_cmd, mode = (
            calculate_center_owner(
                sharp_left_cm,
                sharp_right_cm
            )
        )

        # สำคัญ:
        # Sharp ไม่คุมการหมุน
        z_cmd = 0.0

        return (
            y_cmd,
            z_cmd,
            mode
        )


    # ========================================================
    # LEFT WALL ONLY
    # ========================================================

    elif left_wall:

        reset_side_owner()


        error = (
            sharp_left_cm
            - TARGET_LEFT_CM
        )


        if abs(error) <= SIDE_DEADBAND_CM:

            y_cmd = 0.0


        else:

            # ถ้า L > Target
            # แปลว่าไกลกำแพงซ้ายเกิน
            # -> ขยับซ้าย
            #
            # ถ้า L < Target
            # -> ขยับขวา

            y_cmd = clamp(

                -error
                * SIDE_KP_STRAFE
                * Y_DIR_SIGN,

                -SIDE_MAX_Y,
                SIDE_MAX_Y
            )


        return (
            y_cmd,
            0.0,
            "FOLLOW_LEFT"
        )


    # ========================================================
    # RIGHT WALL ONLY
    # ========================================================

    elif right_wall:

        reset_side_owner()


        error = (
            sharp_right_cm
            - TARGET_RIGHT_CM
        )


        if abs(error) <= SIDE_DEADBAND_CM:

            y_cmd = 0.0


        else:

            y_cmd = clamp(

                error
                * SIDE_KP_STRAFE
                * Y_DIR_SIGN,

                -SIDE_MAX_Y,
                SIDE_MAX_Y
            )


        return (
            y_cmd,
            0.0,
            "FOLLOW_RIGHT"
        )


    # ========================================================
    # OPEN SPACE
    # ========================================================

    else:

        reset_side_owner()

        return (
            0.0,
            0.0,
            "OPEN_SPACE"
        )


# ============================================================
# MAIN
# ============================================================

ep_robot = robot.Robot()

chassis = None
tof_sensor = None


try:

    # ========================================================
    # CONNECT
    # ========================================================

    print("Connecting RoboMaster...")


    ep_robot.initialize(
        conn_type="ap"
    )


    chassis = ep_robot.chassis

    sensor_adapter = (
        ep_robot.sensor_adaptor
    )

    tof_sensor = ep_robot.sensor


    # ========================================================
    # SUBSCRIBE TOF
    # ========================================================

    tof_sensor.sub_distance(
        freq=20,
        callback=tof_callback
    )


    # ========================================================
    # INFO
    # ========================================================

    print()
    print(
        "=========================================================="
    )
    print(
        " MAZE SOLVER - SIDE OWNER / HYSTERESIS CONTROL"
    )
    print(
        "=========================================================="
    )

    print()

    print(
        f"Forward speed       : "
        f"{FORWARD_SPEED:.2f} m/s"
    )

    print(
        f"Front Slow          : "
        f"{SLOW_FRONT_CM:.1f} cm"
    )

    print(
        f"Front Stop          : "
        f"{STOP_FRONT_CM:.1f} cm"
    )

    print()

    print(
        f"Side Target         : "
        f"{TARGET_LEFT_CM:.1f} cm"
    )

    print(
        f"Side Danger         : "
        f"{SIDE_TOO_CLOSE_CM:.1f} cm"
    )

    print(
        f"Wall Detect         : "
        f"{SIDE_WALL_DETECT_CM:.1f} cm"
    )

    print()

    print(
        f"Center Trigger      : "
        f"{CENTER_TRIGGER_CM:.1f} cm"
    )

    print(
        f"Center Release      : "
        f"{CENTER_RELEASE_CM:.1f} cm"
    )

    print(
        f"Owner Hold          : "
        f"{CENTER_HOLD_SEC:.2f} sec"
    )

    print()

    print(
        "Sharp controls Y only."
    )

    print(
        "Sharp Z correction = DISABLED."
    )

    print()

    print("Starting...")

    print()


    time.sleep(1.0)


    # ========================================================
    # FRONT CONFIRM COUNTER
    # ========================================================

    stop_confirm_count = 0


    # ========================================================
    # MAIN LOOP
    # ========================================================

    while True:


        # ====================================================
        # READ LEFT SHARP
        # ====================================================

        raw_adc_l, sharp_left_cm = (
            read_sharp_raw_and_cm(
                sensor_adapter,
                SHARP_LEFT_ID
            )
        )


        # ====================================================
        # READ RIGHT SHARP
        # ====================================================

        raw_adc_r, sharp_right_cm = (
            read_sharp_raw_and_cm(
                sensor_adapter,
                SHARP_RIGHT_ID
            )
        )


        # ====================================================
        # READ IR
        # ====================================================

        ir_left_wall = (
            read_ir_digital_io(
                sensor_adapter
            )
        )


        # ====================================================
        # DEFAULT COMMAND
        # ====================================================

        x = 0.0
        y = 0.0
        z = 0.0

        mode = "STOP"


        # ====================================================
        # FRONT WALL
        # ====================================================

        if (
            front_cm is not None
            and
            0.0 < front_cm <= STOP_FRONT_CM
        ):

            stop_confirm_count += 1

            reset_side_owner()


            # ------------------------------------------------
            # STOP IMMEDIATELY
            # ------------------------------------------------

            x = 0.0
            y = 0.0
            z = 0.0

            mode = "FRONT_STOP"


            # =================================================
            # CONFIRM WALL
            # =================================================

            if stop_confirm_count >= 3:

                print()

                print(
                    "======================================"
                )

                print(
                    f"[FRONT WALL] "
                    f"{front_cm:.1f} cm"
                )

                print(
                    "======================================"
                )


                # =============================================
                # STOP
                # =============================================

                if ENABLE_MOTION:

                    chassis.drive_speed(
                        x=0,
                        y=0,
                        z=0,
                        timeout=0.1
                    )


                time.sleep(0.30)


                # =============================================
                # RECHECK SIDE
                # =============================================

                raw_l_check, l_dist = (
                    read_sharp_raw_and_cm(
                        sensor_adapter,
                        SHARP_LEFT_ID
                    )
                )

                raw_r_check, r_dist = (
                    read_sharp_raw_and_cm(
                        sensor_adapter,
                        SHARP_RIGHT_ID
                    )
                )


                print(
                    f"Side Check -> "
                    f"L:{l_dist:.1f} cm | "
                    f"R:{r_dist:.1f} cm"
                )


                # =============================================
                # DEAD END
                # =============================================

                if (
                    l_dist < SIDE_DEAD_END_CM
                    and
                    r_dist < SIDE_DEAD_END_CM
                ):

                    print(
                        "--> DEAD END"
                    )

                    print(
                        "--> TURN 180"
                    )


                    if ENABLE_MOTION:

                        chassis.move(

                            x=0,
                            y=0,

                            z=(
                                TURN_AROUND_DEG
                                * Z_DIR_SIGN
                            ),

                            z_speed=TURN_SPEED

                        ).wait_for_completed()


                # =============================================
                # LEFT CLEARER
                # =============================================

                elif (
                    l_dist - r_dist
                ) > 20.0:

                    print(
                        "--> LEFT OPEN"
                    )

                    print(
                        "--> TURN LEFT 90"
                    )


                    if ENABLE_MOTION:

                        chassis.move(

                            x=0,
                            y=0,

                            z=(
                                TURN_LEFT_DEG
                                * Z_DIR_SIGN
                            ),

                            z_speed=TURN_SPEED

                        ).wait_for_completed()


                # =============================================
                # RIGHT CLEARER
                # =============================================

                elif (
                    r_dist - l_dist
                ) > 20.0:

                    print(
                        "--> RIGHT OPEN"
                    )

                    print(
                        "--> TURN RIGHT 90"
                    )


                    if ENABLE_MOTION:

                        chassis.move(

                            x=0,
                            y=0,

                            z=(
                                TURN_RIGHT_DEG
                                * Z_DIR_SIGN
                            ),

                            z_speed=TURN_SPEED

                        ).wait_for_completed()


                # =============================================
                # FALLBACK
                # =============================================

                else:

                    if l_dist >= r_dist:

                        print(
                            "--> FALLBACK LEFT"
                        )


                        if ENABLE_MOTION:

                            chassis.move(

                                x=0,
                                y=0,

                                z=(
                                    TURN_LEFT_DEG
                                    * Z_DIR_SIGN
                                ),

                                z_speed=TURN_SPEED

                            ).wait_for_completed()


                    else:

                        print(
                            "--> FALLBACK RIGHT"
                        )


                        if ENABLE_MOTION:

                            chassis.move(

                                x=0,
                                y=0,

                                z=(
                                    TURN_RIGHT_DEG
                                    * Z_DIR_SIGN
                                ),

                                z_speed=TURN_SPEED

                            ).wait_for_completed()


                # =============================================
                # RESET AFTER TURN
                # =============================================

                tof_buffer.clear()

                sharp_left_buffer.clear()
                sharp_right_buffer.clear()

                sharp_left_ema = None
                sharp_right_ema = None

                front_cm = None

                stop_confirm_count = 0

                reset_side_owner()


                time.sleep(0.20)


        # ====================================================
        # NORMAL MOVEMENT
        # ====================================================

        else:

            stop_confirm_count = 0


            # =================================================
            # X CONTROL
            # =================================================

            x = calculate_forward_speed(
                front_cm
            )


            # =================================================
            # Y CONTROL
            # =================================================

            y, z, mode = (
                calculate_motion_control(

                    raw_adc_l,
                    sharp_left_cm,

                    raw_adc_r,
                    sharp_right_cm,

                    ir_left_wall
                )
            )


            # =================================================
            # SLOW MODE
            # =================================================

            if (
                front_cm is not None
                and
                STOP_FRONT_CM
                < front_cm
                < SLOW_FRONT_CM
            ):

                mode = (
                    "SLOW_"
                    + mode
                )


        # ====================================================
        # SEND COMMAND
        # ====================================================

        if ENABLE_MOTION:

            chassis.drive_speed(

                x=x,
                y=y,
                z=z,

                timeout=0.15
            )


        # ====================================================
        # DEBUG
        # ====================================================

        if (
            sharp_left_cm is not None
            and
            sharp_right_cm is not None
        ):

            delta = (
                sharp_left_cm
                - sharp_right_cm
            )

        else:

            delta = 0.0


        print(

            f"ToF:{fmt(front_cm)}cm | "

            f"L:{fmt(sharp_left_cm)} "
            f"ADC:{raw_adc_l:4d} | "

            f"R:{fmt(sharp_right_cm)} "
            f"ADC:{raw_adc_r:4d} | "

            f"D:{delta:+5.1f} | "

            f"OWNER:{side_owner:5s} | "

            f"{mode:20s} | "

            f"x={x:.3f} "
            f"y={y:+.3f} "
            f"z={z:+.1f}"
        )


        time.sleep(0.05)


# ============================================================
# CTRL+C
# ============================================================

except KeyboardInterrupt:

    print()

    print(
        "STOP REQUESTED BY USER"
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

        if chassis is not None:

            chassis.drive_speed(
                x=0,
                y=0,
                z=0,
                timeout=0.1
            )

    except Exception:

        pass


    try:

        if tof_sensor is not None:

            tof_sensor.unsub_distance()

    except Exception:

        pass


    ep_robot.close()

    print(
        "Robot stopped and disconnected."
    )
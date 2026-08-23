from robomaster import robot
from collections import deque
import statistics
import argparse
import datetime
import threading
import csv
import math
import os
import time


# ============================================================
# ARGUMENT
# ============================================================

parser = argparse.ArgumentParser()
parser.add_argument(
    "--run",
    action="store_true",
    help="Allow robot to move"
)

args = parser.parse_args()
ENABLE_MOTION = args.run


# ============================================================
# DIRECTORY / RUN ID
# ============================================================

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

LOG_DIR = os.path.join(BASE_DIR, "logs")
MAP_DIR = os.path.join(BASE_DIR, "maps")

os.makedirs(LOG_DIR, exist_ok=True)
os.makedirs(MAP_DIR, exist_ok=True)

RUN_ID = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

TELEMETRY_FILE = os.path.join(
    LOG_DIR,
    f"telemetry_{RUN_ID}.csv"
)

MAP_POINTS_FILE = os.path.join(
    LOG_DIR,
    f"map_points_{RUN_ID}.csv"
)

EVENT_FILE = os.path.join(
    LOG_DIR,
    f"events_{RUN_ID}.txt"
)

MAP_IMAGE_FILE = os.path.join(
    MAP_DIR,
    f"maze_map_{RUN_ID}.png"
)


# ============================================================
# SENSOR POSITION
#
# ใช้ mapping เดิมที่เราทดสอบแล้ว
# ============================================================

LF_ID = 1
LR_ID = 2
RF_ID = 3
RR_ID = 4

SHARP_PORT = 1


# ============================================================
# ROBOT PHYSICAL SENSOR POSITION
#
# หน่วย = เมตร
#
# สำคัญ:
# ค่านี้คือ "ตำแหน่ง sensor จากจุดกลางหุ่น"
#
# ตอนนี้ใช้ค่าประมาณก่อน
# ถ้าวัดจริงได้ ค่อยมาแก้ให้ map แม่นขึ้น
# ============================================================

SHARP_FRONT_X = 0.13
SHARP_REAR_X = -0.13

SHARP_SIDE_Y = 0.11

TOF_X = 0.17


# ============================================================
# SENSOR CALIBRATION
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

# SIDE
SIDE_TRIGGER_CM = 10.0
SIDE_RELEASE_CM = 12.0
SIDE_DANGER_CM = 7.0

# FRONT
FRONT_STOP_CM = 20.0
FRONT_RELEASE_CM = 23.0


# ============================================================
# SPEED SETTINGS
# ============================================================

FORWARD_SPEED = 0.15

TURN_SPEED = 25.0
DANGER_TURN_SPEED = 35.0

STRAIGHT_CORRECTION_MAX = 5.0

ANGLE_DEADBAND_CM = 2.0
ANGLE_KP = 1.2


# ============================================================
# MAPPING SETTINGS
# ============================================================

# Sharp ช่วงเกิน 25 cm เริ่ม noise สูง
# จึงไม่เอามาวาด map
MAP_SHARP_MAX_CM = 25.0

# ToF ให้วาดกำแพงไม่เกิน 1.5 เมตร
MAP_TOF_MAX_CM = 150.0

# Grid resolution
# 0.025 = 2.5 cm ต่อ cell
MAP_RESOLUTION_M = 0.025

# เก็บ trajectory ทุกระยะขั้นต่ำ
PATH_MIN_DISTANCE_M = 0.01


# ============================================================
# STATE
# ============================================================

STATE_FORWARD = "FORWARD"
STATE_TURN_LEFT = "TURN_LEFT"
STATE_TURN_RIGHT = "TURN_RIGHT"
STATE_FRONT_STOP = "FRONT_STOP"

state = STATE_FORWARD
previous_state = None

TURN_MIN_TIME = 0.40
turn_start_time = 0.0


# ============================================================
# ROBOT POSE
# ============================================================

pose_lock = threading.Lock()

robot_x = 0.0
robot_y = 0.0
robot_yaw = 0.0

pose_valid = False

trajectory = []


# ============================================================
# ToF
# ============================================================

tof_buffer = deque(maxlen=5)

front_cm = None
front_time = 0.0


# ============================================================
# MAP STORAGE
# ============================================================

map_points = []

# ใช้ grid กันจุดซ้ำมหาศาล
occupied_cells = set()


# ============================================================
# TIME
# ============================================================

program_start_time = time.time()


# ============================================================
# LOG FILES
# ============================================================

telemetry_fp = open(
    TELEMETRY_FILE,
    "w",
    newline="",
    encoding="utf-8"
)

telemetry_writer = csv.writer(telemetry_fp)

telemetry_writer.writerow([
    "time_s",
    "x_m",
    "y_m",
    "yaw_deg",

    "tof_cm",

    "lf_cm",
    "lr_cm",
    "rf_cm",
    "rr_cm",

    "left_min_cm",
    "right_min_cm",

    "cmd_x",
    "cmd_y",
    "cmd_z",

    "state"
])


map_fp = open(
    MAP_POINTS_FILE,
    "w",
    newline="",
    encoding="utf-8"
)

map_writer = csv.writer(map_fp)

map_writer.writerow([
    "time_s",
    "sensor",
    "x_m",
    "y_m",
    "distance_cm"
])


event_fp = open(
    EVENT_FILE,
    "w",
    encoding="utf-8"
)


# ============================================================
# UTILITIES
# ============================================================

def clamp(v, mn, mx):
    return max(mn, min(mx, v))


def fmt(v):

    if v is None:
        return "---"

    return f"{v:4.1f}"


# ============================================================
# POSITION CALLBACK
#
# RoboMaster callback:
# (x, y, z)
#
# x = m
# y = m
# z = rotation degree
# ============================================================

def position_callback(position_info):

    global robot_x
    global robot_y
    global robot_yaw
    global pose_valid

    try:

        x, y, z = position_info

        with pose_lock:

            robot_x = float(x)
            robot_y = float(y)
            robot_yaw = float(z)

            pose_valid = True

            # ---------------- PATH ----------------

            if not trajectory:

                trajectory.append(
                    (robot_x, robot_y)
                )

            else:

                last_x, last_y = trajectory[-1]

                distance = math.hypot(
                    robot_x - last_x,
                    robot_y - last_y
                )

                if distance >= PATH_MIN_DISTANCE_M:

                    trajectory.append(
                        (robot_x, robot_y)
                    )

    except Exception as e:

        print(
            "Position callback error:",
            e
        )


# ============================================================
# TOF CALLBACK
# ============================================================

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

        print(
            "ToF callback error:",
            e
        )


# ============================================================
# ADC -> CM
# ============================================================

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

            ratio = (
                (adc1 - adc)
                /
                (adc1 - adc2)
            )

            return (
                cm1
                +
                ratio * (cm2 - cm1)
            )

    return 30.0


# ============================================================
# READ SHARP
# ============================================================

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
            (1.0 - EMA_ALPHA)
            * ema[sensor_id]
        )

    cm = adc_to_cm(
        sensor_id,
        ema[sensor_id]
    )

    return cm


# ============================================================
# BODY -> WORLD TRANSFORM
# ============================================================

def body_to_world(body_x, body_y):

    with pose_lock:

        px = robot_x
        py = robot_y
        yaw = robot_yaw

    theta = math.radians(yaw)

    world_x = (
        px
        +
        body_x * math.cos(theta)
        -
        body_y * math.sin(theta)
    )

    world_y = (
        py
        +
        body_x * math.sin(theta)
        +
        body_y * math.cos(theta)
    )

    return world_x, world_y


# ============================================================
# ADD MAP POINT
# ============================================================

def add_map_point(
    sensor_name,
    body_x,
    body_y,
    distance_cm
):

    if not pose_valid:
        return

    world_x, world_y = body_to_world(
        body_x,
        body_y
    )

    cell_x = round(
        world_x / MAP_RESOLUTION_M
    )

    cell_y = round(
        world_y / MAP_RESOLUTION_M
    )

    cell = (
        cell_x,
        cell_y
    )

    # มีจุดใน cell นี้แล้ว
    if cell in occupied_cells:
        return

    occupied_cells.add(cell)

    map_points.append(
        (
            world_x,
            world_y,
            sensor_name
        )
    )

    elapsed = (
        time.time()
        -
        program_start_time
    )

    map_writer.writerow([
        f"{elapsed:.3f}",
        sensor_name,
        f"{world_x:.4f}",
        f"{world_y:.4f}",
        f"{distance_cm:.2f}"
    ])


# ============================================================
# UPDATE MAP
# ============================================================

def update_map(lf, lr, rf, rr, tof_cm):

    # --------------------------------------------------------
    # LEFT FRONT
    # body Y negative = LEFT
    # --------------------------------------------------------

    if lf <= MAP_SHARP_MAX_CM:

        d = lf / 100.0

        add_map_point(
            "LF",
            SHARP_FRONT_X,
            -(SHARP_SIDE_Y + d),
            lf
        )


    # --------------------------------------------------------
    # LEFT REAR
    # --------------------------------------------------------

    if lr <= MAP_SHARP_MAX_CM:

        d = lr / 100.0

        add_map_point(
            "LR",
            SHARP_REAR_X,
            -(SHARP_SIDE_Y + d),
            lr
        )


    # --------------------------------------------------------
    # RIGHT FRONT
    # --------------------------------------------------------

    if rf <= MAP_SHARP_MAX_CM:

        d = rf / 100.0

        add_map_point(
            "RF",
            SHARP_FRONT_X,
            SHARP_SIDE_Y + d,
            rf
        )


    # --------------------------------------------------------
    # RIGHT REAR
    # --------------------------------------------------------

    if rr <= MAP_SHARP_MAX_CM:

        d = rr / 100.0

        add_map_point(
            "RR",
            SHARP_REAR_X,
            SHARP_SIDE_Y + d,
            rr
        )


    # --------------------------------------------------------
    # FRONT TOF
    # --------------------------------------------------------

    if (
        tof_cm is not None
        and
        tof_cm <= MAP_TOF_MAX_CM
    ):

        d = tof_cm / 100.0

        add_map_point(
            "TOF",
            TOF_X + d,
            0.0,
            tof_cm
        )


# ============================================================
# STRAIGHT HEADING CORRECTION
# ============================================================

def straight_heading_correction(
    lf,
    lr,
    rf,
    rr
):

    corrections = []


    # LEFT WALL

    if max(lf, lr) < 20.0:

        diff = lf - lr

        if abs(diff) > ANGLE_DEADBAND_CM:

            corrections.append(
                ANGLE_KP * diff
            )


    # RIGHT WALL

    if max(rf, rr) < 20.0:

        diff = rf - rr

        if abs(diff) > ANGLE_DEADBAND_CM:

            corrections.append(
                -ANGLE_KP * diff
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
        STRAIGHT_CORRECTION_MAX
    )


# ============================================================
# EVENT LOG
# ============================================================

def log_state_change(new_state):

    global previous_state

    if new_state == previous_state:
        return

    elapsed = (
        time.time()
        -
        program_start_time
    )

    line = (
        f"{elapsed:8.3f}s : "
        f"{previous_state} -> {new_state}\n"
    )

    event_fp.write(line)
    event_fp.flush()

    previous_state = new_state


# ============================================================
# SAVE MAP IMAGE
# ============================================================

def save_map_image():

    try:

        import matplotlib.pyplot as plt

    except ImportError:

        print()
        print("matplotlib not installed.")
        print("Map CSV was still saved.")

        return


    if (
        not trajectory
        and
        not map_points
    ):

        print("No map data to save.")
        return


    plt.figure(
        figsize=(9, 9)
    )


    # ========================================================
    # WALL POINTS
    # ========================================================

    if map_points:

        wall_x = [
            p[0]
            for p in map_points
        ]

        wall_y = [
            p[1]
            for p in map_points
        ]

        plt.scatter(
            wall_x,
            wall_y,
            s=8,
            label="Detected wall / obstacle"
        )


    # ========================================================
    # ROBOT PATH
    # ========================================================

    if trajectory:

        path_x = [
            p[0]
            for p in trajectory
        ]

        path_y = [
            p[1]
            for p in trajectory
        ]

        plt.plot(
            path_x,
            path_y,
            linewidth=2,
            label="Robot path"
        )


        # START

        plt.scatter(
            [path_x[0]],
            [path_y[0]],
            s=80,
            marker="o",
            label="START"
        )


        # END

        plt.scatter(
            [path_x[-1]],
            [path_y[-1]],
            s=80,
            marker="x",
            label="END"
        )


    plt.xlabel("X position (m)")
    plt.ylabel("Y position (m)")

    plt.title(
        f"RoboMaster Maze Map\nRun: {RUN_ID}"
    )

    plt.axis("equal")
    plt.grid(True)

    plt.legend()

    plt.tight_layout()

    plt.savefig(
        MAP_IMAGE_FILE,
        dpi=200
    )

    plt.close()


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
    sensor = ep_robot.sensor_adaptor
    tof = ep_robot.sensor


    # ========================================================
    # SUBSCRIPTIONS
    # ========================================================

    # Position + rotation
    chassis.sub_position(
        cs=0,
        freq=20,
        callback=position_callback
    )

    # Front ToF
    tof.sub_distance(
        freq=20,
        callback=tof_callback
    )


    print()
    print("============================================")
    print(" WALL AVOIDANCE + MAPPING + LOGGER")
    print("============================================")
    print()

    print(
        f"Telemetry : {TELEMETRY_FILE}"
    )

    print(
        f"Map data  : {MAP_POINTS_FILE}"
    )

    print(
        f"Events    : {EVENT_FILE}"
    )

    print(
        f"Map image : {MAP_IMAGE_FILE}"
    )

    print()


    if ENABLE_MOTION:

        print("!!! MOTION ENABLED !!!")

    else:

        print("DRY RUN")
        print("Robot WILL NOT MOVE")
        print()
        print(
            "Run:"
        )
        print(
            "python wall_avoidance_mapping.py --run"
        )


    print()
    print("Ctrl+C = STOP + SAVE MAP")
    print()

    time.sleep(1.0)


    # ========================================================
    # MAIN LOOP
    # ========================================================

    while True:

        # ====================================================
        # READ SENSOR
        # ====================================================

        lf = read_sharp(
            sensor,
            LF_ID
        )

        lr = read_sharp(
            sensor,
            LR_ID
        )

        rf = read_sharp(
            sensor,
            RF_ID
        )

        rr = read_sharp(
            sensor,
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
        # MAP UPDATE
        # ====================================================

        update_map(
            lf,
            lr,
            rf,
            rr,
            front_cm
        )


        # ====================================================
        # DEFAULT COMMAND
        # ====================================================

        x = 0.0
        y = 0.0
        z = 0.0


        # ====================================================
        # FRONT VALID
        # ====================================================

        tof_valid = (
            front_cm is not None
            and
            time.time() - front_time < 1.0
        )


        # ====================================================
        # FRONT STOP
        # ====================================================

        if not tof_valid:

            state = STATE_FRONT_STOP


        elif state == STATE_FRONT_STOP:

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
        # TURN RIGHT
        # ====================================================

        elif state == STATE_TURN_RIGHT:

            elapsed_turn = (
                time.time()
                -
                turn_start_time
            )

            if (
                left_min < SIDE_RELEASE_CM
                or
                elapsed_turn < TURN_MIN_TIME
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
        # TURN LEFT
        # ====================================================

        elif state == STATE_TURN_LEFT:

            elapsed_turn = (
                time.time()
                -
                turn_start_time
            )

            if (
                right_min < SIDE_RELEASE_CM
                or
                elapsed_turn < TURN_MIN_TIME
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
        # FORWARD STATE
        # ====================================================

        if state == STATE_FORWARD:


            # BOTH SIDE CLOSE

            if (
                left_min <= SIDE_TRIGGER_CM
                and
                right_min <= SIDE_TRIGGER_CM
            ):

                if left_min < right_min - 1.0:

                    state = STATE_TURN_RIGHT

                    turn_start_time = (
                        time.time()
                    )

                    x = 0.0
                    y = 0.0
                    z = -TURN_SPEED


                elif right_min < left_min - 1.0:

                    state = STATE_TURN_LEFT

                    turn_start_time = (
                        time.time()
                    )

                    x = 0.0
                    y = 0.0
                    z = TURN_SPEED


                else:

                    x = 0.0
                    y = 0.0
                    z = 0.0


            # LEFT CLOSE

            elif left_min <= SIDE_TRIGGER_CM:

                state = STATE_TURN_RIGHT

                turn_start_time = (
                    time.time()
                )

                x = 0.0
                y = 0.0

                if left_min <= SIDE_DANGER_CM:

                    z = -DANGER_TURN_SPEED

                else:

                    z = -TURN_SPEED


            # RIGHT CLOSE

            elif right_min <= SIDE_TRIGGER_CM:

                state = STATE_TURN_LEFT

                turn_start_time = (
                    time.time()
                )

                x = 0.0
                y = 0.0

                if right_min <= SIDE_DANGER_CM:

                    z = DANGER_TURN_SPEED

                else:

                    z = TURN_SPEED


            # CLEAR

            else:

                x = FORWARD_SPEED
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
        # STATE EVENT
        # ====================================================

        log_state_change(
            state
        )


        # ====================================================
        # COPY POSE
        # ====================================================

        with pose_lock:

            px = robot_x
            py = robot_y
            yaw = robot_yaw


        # ====================================================
        # TELEMETRY LOG
        # ====================================================

        elapsed = (
            time.time()
            -
            program_start_time
        )

        telemetry_writer.writerow([
            f"{elapsed:.3f}",

            f"{px:.4f}",
            f"{py:.4f}",
            f"{yaw:.2f}",

            (
                f"{front_cm:.2f}"
                if front_cm is not None
                else ""
            ),

            f"{lf:.2f}",
            f"{lr:.2f}",
            f"{rf:.2f}",
            f"{rr:.2f}",

            f"{left_min:.2f}",
            f"{right_min:.2f}",

            f"{x:.3f}",
            f"{y:.3f}",
            f"{z:.3f}",

            state
        ])


        # ====================================================
        # DISPLAY
        # ====================================================

        print(
            f"POS({px:+.2f},{py:+.2f}) "
            f"YAW:{yaw:+6.1f} | "
            f"F:{fmt(front_cm)} | "
            f"LF:{fmt(lf)} LR:{fmt(lr)} | "
            f"RF:{fmt(rf)} RR:{fmt(rr)} | "
            f"x={x:+.2f} "
            f"z={z:+.1f} | "
            f"{state} | "
            f"MAP:{len(map_points)}"
        )


        time.sleep(0.05)


# ============================================================
# CTRL+C
# ============================================================

except KeyboardInterrupt:

    print()
    print("STOP requested")


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

    print()
    print("Stopping robot...")


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

        chassis.unsub_position()

    except:
        pass


    try:

        tof.unsub_distance()

    except:
        pass


    # --------------------------------------------------------
    # SAVE FILES
    # --------------------------------------------------------

    try:

        telemetry_fp.flush()
        telemetry_fp.close()

    except:
        pass


    try:

        map_fp.flush()
        map_fp.close()

    except:
        pass


    try:

        event_fp.flush()
        event_fp.close()

    except:
        pass


    print()
    print("Generating map...")


    try:

        save_map_image()

    except Exception as e:

        print(
            "Map generation error:",
            e
        )


    try:

        ep_robot.close()

    except:
        pass


    print()
    print("============================================")
    print(" RUN COMPLETE")
    print("============================================")
    print()
    print("Telemetry:")
    print(TELEMETRY_FILE)
    print()
    print("Map points:")
    print(MAP_POINTS_FILE)
    print()
    print("Event log:")
    print(EVENT_FILE)
    print()
    print("Map:")
    print(MAP_IMAGE_FILE)
    print()

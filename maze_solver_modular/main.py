"""Main entry point for the modular RoboMaster maze solver."""

import time

from robomaster import robot

import config
from controller import MotionController
from navigation import decide_turn, execute_turn, print_turn_decision
from sensors import SensorManager


def fmt(value):
    if value is None:
        return "---"
    return f"{value:4.1f}"


def print_startup_info():
    print()
    print("==========================================================")
    print(" MAZE SOLVER - SIDE OWNER / HYSTERESIS CONTROL")
    print("==========================================================")
    print()
    print(f"Forward speed       : {config.FORWARD_SPEED:.2f} m/s")
    print(f"Front Slow          : {config.SLOW_FRONT_CM:.1f} cm")
    print(f"Front Stop          : {config.STOP_FRONT_CM:.1f} cm")
    print()
    print(f"Side Target         : {config.TARGET_LEFT_CM:.1f} cm")
    print(f"Side Danger         : {config.SIDE_TOO_CLOSE_CM:.1f} cm")
    print(f"Wall Detect         : {config.SIDE_WALL_DETECT_CM:.1f} cm")
    print()
    print(f"Center Trigger      : {config.CENTER_TRIGGER_CM:.1f} cm")
    print(f"Center Release      : {config.CENTER_RELEASE_CM:.1f} cm")
    print(f"Owner Hold          : {config.CENTER_HOLD_SEC:.2f} sec")
    print()
    print("Sharp controls Y only.")
    print("Sharp Z correction = DISABLED.")
    print()
    print("Starting...")
    print()


def main():
    ep_robot = robot.Robot()
    chassis = None
    tof_sensor = None

    try:
        # ====================================================
        # CONNECT
        # ====================================================

        print("Connecting RoboMaster...")
        ep_robot.initialize(conn_type="ap")

        chassis = ep_robot.chassis
        sensor_adapter = ep_robot.sensor_adaptor
        tof_sensor = ep_robot.sensor

        sensors = SensorManager(sensor_adapter)
        controller = MotionController()

        # ====================================================
        # SUBSCRIBE TOF
        # ====================================================

        tof_sensor.sub_distance(
            freq=20,
            callback=sensors.tof_callback,
        )

        print_startup_info()
        time.sleep(1.0)

        stop_confirm_count = 0

        # ====================================================
        # MAIN LOOP
        # ====================================================

        while True:
            raw_adc_l, sharp_left_cm = sensors.read_left_sharp()
            raw_adc_r, sharp_right_cm = sensors.read_right_sharp()
            ir_left_wall = sensors.read_ir_digital_io()

            x = 0.0
            y = 0.0
            z = 0.0
            mode = "STOP"

            front_cm = sensors.front_cm

            # ================================================
            # FRONT WALL
            # ================================================

            if (
                front_cm is not None
                and 0.0 < front_cm <= config.STOP_FRONT_CM
            ):
                stop_confirm_count += 1
                controller.reset_side_owner()

                # Stop immediately
                x = 0.0
                y = 0.0
                z = 0.0
                mode = "FRONT_STOP"

                if stop_confirm_count >= config.STOP_CONFIRM_SAMPLES:
                    print()
                    print("======================================")
                    print(f"[FRONT WALL] {front_cm:.1f} cm")
                    print("======================================")

                    if config.ENABLE_MOTION:
                        chassis.drive_speed(
                            x=0,
                            y=0,
                            z=0,
                            timeout=0.1,
                        )

                    time.sleep(config.FRONT_RECHECK_DELAY_SEC)

                    # Recheck side distances after full stop
                    _, l_dist = sensors.read_left_sharp()
                    _, r_dist = sensors.read_right_sharp()

                    print(
                        f"Side Check -> "
                        f"L:{l_dist:.1f} cm | "
                        f"R:{r_dist:.1f} cm"
                    )

                    decision = decide_turn(l_dist, r_dist)
                    print_turn_decision(decision)
                    execute_turn(chassis, decision)

                    # Reset stale sensor/controller state after turn
                    sensors.reset_filters()
                    controller.reset_side_owner()
                    stop_confirm_count = 0

                    time.sleep(config.AFTER_TURN_DELAY_SEC)

            # ================================================
            # NORMAL MOVEMENT
            # ================================================

            else:
                stop_confirm_count = 0

                x = controller.calculate_forward_speed(front_cm)

                y, z, mode = controller.calculate_motion_control(
                    raw_adc_l,
                    sharp_left_cm,
                    raw_adc_r,
                    sharp_right_cm,
                    ir_left_wall,
                )

                if (
                    front_cm is not None
                    and config.STOP_FRONT_CM
                    < front_cm
                    < config.SLOW_FRONT_CM
                ):
                    mode = "SLOW_" + mode

            # ================================================
            # SEND COMMAND
            # ================================================

            if config.ENABLE_MOTION:
                chassis.drive_speed(
                    x=x,
                    y=y,
                    z=z,
                    timeout=config.DRIVE_TIMEOUT_SEC,
                )

            # ================================================
            # DEBUG
            # ================================================

            if sharp_left_cm is not None and sharp_right_cm is not None:
                delta = sharp_left_cm - sharp_right_cm
            else:
                delta = 0.0

            print(
                f"ToF:{fmt(sensors.front_cm)}cm | "
                f"L:{fmt(sharp_left_cm)} ADC:{raw_adc_l:4d} | "
                f"R:{fmt(sharp_right_cm)} ADC:{raw_adc_r:4d} | "
                f"D:{delta:+5.1f} | "
                f"OWNER:{controller.side_owner:5s} | "
                f"{mode:20s} | "
                f"x={x:.3f} "
                f"y={y:+.3f} "
                f"z={z:+.1f}"
            )

            time.sleep(config.LOOP_DELAY_SEC)

    except KeyboardInterrupt:
        print()
        print("STOP REQUESTED BY USER")

    except Exception as exc:
        print()
        print("ERROR:", exc)

    finally:
        try:
            if chassis is not None:
                chassis.drive_speed(
                    x=0,
                    y=0,
                    z=0,
                    timeout=0.1,
                )
        except Exception:
            pass

        try:
            if tof_sensor is not None:
                tof_sensor.unsub_distance()
        except Exception:
            pass

        ep_robot.close()
        print("Robot stopped and disconnected.")


if __name__ == "__main__":
    main()

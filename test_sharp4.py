from robomaster import robot
import time

PORT = 1
ADAPTER_IDS = [1, 2, 3, 4]

ep_robot = robot.Robot()

try:
    print("Connecting to RoboMaster...")
    ep_robot.initialize(conn_type="ap")

    sensor = ep_robot.sensor_adaptor

    print("Connected!")
    print("Reading Sharp sensors...")
    print("Press Ctrl+C to stop")
    print()

    while True:
        values = []

        for adapter_id in ADAPTER_IDS:
            try:
                adc = sensor.get_adc(
                    id=adapter_id,
                    port=PORT
                )

                voltage = (adc / 1023.0) * 3.3

                values.append(
                    f"Sharp {adapter_id}: ADC={adc:4d}  V={voltage:.3f}V"
                )

            except Exception as e:
                values.append(
                    f"Sharp {adapter_id}: ERROR ({e})"
                )

        print(" | ".join(values))

        time.sleep(0.2)

except KeyboardInterrupt:
    print("\nStopped.")

finally:
    ep_robot.close()
    print("RoboMaster disconnected.")

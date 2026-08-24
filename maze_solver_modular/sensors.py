"""Sensor reading, calibration and filtering."""

import statistics
import time
from collections import deque

import config


class SensorManager:
    def __init__(self, sensor_adapter):
        self.sensor_adapter = sensor_adapter

        self.sharp_left_buffer = deque(maxlen=config.SHARP_FILTER_SIZE)
        self.sharp_right_buffer = deque(maxlen=config.SHARP_FILTER_SIZE)
        self.tof_buffer = deque(maxlen=config.TOF_FILTER_SIZE)

        self.sharp_left_ema = None
        self.sharp_right_ema = None
        self.front_cm = None
        self.tof_last_update = None

    # ========================================================
    # TOF
    # ========================================================

    def tof_callback(self, data):
        try:
            if not data or data[0] is None:
                return

            mm = data[0]

            # Reject invalid values
            if mm < 20 or mm > 4000:
                return

            cm = mm / 10.0
            self.tof_buffer.append(cm)
            self.front_cm = statistics.median(self.tof_buffer)
            self.tof_last_update = time.monotonic()

        except Exception as exc:
            print("ToF callback error:", exc)

    def get_front_cm(self):
        """Return fresh ToF distance, or None if data is absent/stale."""
        if self.front_cm is None or self.tof_last_update is None:
            return None

        age = time.monotonic() - self.tof_last_update
        if age > config.TOF_STALE_SEC:
            return None

        return self.front_cm

    # ========================================================
    # SHARP CALIBRATION
    # ========================================================

    @staticmethod
    def adc_to_cm(adc, table):
        if adc >= table[0][0]:
            return float(table[0][1])

        if adc <= table[-1][0]:
            return float(table[-1][1])

        for i in range(len(table) - 1):
            adc1, cm1 = table[i]
            adc2, cm2 = table[i + 1]

            if adc1 >= adc >= adc2:
                ratio = (adc1 - adc) / (adc1 - adc2)
                return cm1 + ratio * (cm2 - cm1)

        return float(table[-1][1])

    def calibration_for_sensor(self, sensor_id):
        if sensor_id == config.SHARP_LEFT_ID:
            return config.CALIBRATION_SHARP_LEFT
        if sensor_id == config.SHARP_RIGHT_ID:
            return config.CALIBRATION_SHARP_RIGHT
        raise ValueError(f"Unknown Sharp sensor id: {sensor_id}")

    # ========================================================
    # SHARP READ + FILTER
    # ========================================================

    def read_sharp_raw_and_cm(self, sensor_id):
        try:
            raw = self.sensor_adapter.get_adc(
                id=sensor_id,
                port=config.SENSOR_PORT,
            )
        except Exception as exc:
            print(f"Sharp {sensor_id} read error: {exc}")
            return 0, None

        if sensor_id == config.SHARP_LEFT_ID:
            self.sharp_left_buffer.append(raw)
            median_adc = statistics.median(self.sharp_left_buffer)

            if self.sharp_left_ema is None:
                self.sharp_left_ema = median_adc
            else:
                self.sharp_left_ema = (
                    config.SHARP_EMA_NEW_WEIGHT * median_adc
                    + config.SHARP_EMA_OLD_WEIGHT * self.sharp_left_ema
                )

            ema_val = self.sharp_left_ema

        elif sensor_id == config.SHARP_RIGHT_ID:
            self.sharp_right_buffer.append(raw)
            median_adc = statistics.median(self.sharp_right_buffer)

            if self.sharp_right_ema is None:
                self.sharp_right_ema = median_adc
            else:
                self.sharp_right_ema = (
                    config.SHARP_EMA_NEW_WEIGHT * median_adc
                    + config.SHARP_EMA_OLD_WEIGHT * self.sharp_right_ema
                )

            ema_val = self.sharp_right_ema

        else:
            raise ValueError(f"Unknown Sharp sensor id: {sensor_id}")

        table = self.calibration_for_sensor(sensor_id)
        return raw, self.adc_to_cm(ema_val, table)

    def read_left_sharp(self):
        return self.read_sharp_raw_and_cm(config.SHARP_LEFT_ID)

    def read_right_sharp(self):
        return self.read_sharp_raw_and_cm(config.SHARP_RIGHT_ID)

    # ========================================================
    # DIGITAL IR
    # ========================================================

    def read_ir_digital_io(self):
        try:
            return self.sensor_adapter.get_io_level(
                id=config.IR_LEFT_FRONT_ID,
                port=config.SENSOR_PORT,
            )

        except Exception:
            try:
                raw = self.sensor_adapter.get_adc(
                    id=config.IR_LEFT_FRONT_ID,
                    port=config.SENSOR_PORT,
                )
                return 1 if raw > 300 else 0
            except Exception:
                return None

    # ========================================================
    # RESET FILTERS AFTER TURN
    # ========================================================

    def reset_filters(self):
        self.tof_buffer.clear()
        self.sharp_left_buffer.clear()
        self.sharp_right_buffer.clear()

        self.sharp_left_ema = None
        self.sharp_right_ema = None
        self.front_cm = None
        self.tof_last_update = None

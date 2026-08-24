"""Sensor reading, calibration and filtering."""

import statistics
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

        except Exception as exc:
            print("ToF callback error:", exc)

    # ========================================================
    # SHARP CALIBRATION
    # ========================================================

    @staticmethod
    def adc_to_cm(adc):
        table = config.CALIBRATION_SHARP2

        if adc >= table[0][0]:
            return 5.0

        if adc <= table[-1][0]:
            return 80.0

        for i in range(len(table) - 1):
            adc1, cm1 = table[i]
            adc2, cm2 = table[i + 1]

            if adc1 >= adc >= adc2:
                ratio = (adc1 - adc) / (adc1 - adc2)
                return cm1 + ratio * (cm2 - cm1)

        return 80.0

    # ========================================================
    # SHARP READ + FILTER
    # ========================================================

    def read_sharp_raw_and_cm(self, sensor_id):
        raw = self.sensor_adapter.get_adc(
            id=sensor_id,
            port=config.SENSOR_PORT,
        )

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

        return raw, self.adc_to_cm(ema_val)

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
            raw = self.sensor_adapter.get_adc(
                id=config.IR_LEFT_FRONT_ID,
                port=config.SENSOR_PORT,
            )
            return 1 if raw > 300 else 0

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

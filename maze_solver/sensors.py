"""Sensor reading, calibration, filtering and V11 digital-IR debounce."""

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

        # V11: separate IR histories.  Store raw digital levels so polarity can
        # be changed from config without changing the filtering algorithm.
        self.ir_left_buffer = deque(maxlen=config.IR_FILTER_SIZE)
        self.ir_right_buffer = deque(maxlen=config.IR_FILTER_SIZE)

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
            if mm < 20 or mm > 4000:
                return
            cm = mm / 10.0
            self.tof_buffer.append(cm)
            self.front_cm = statistics.median(self.tof_buffer)
            self.tof_last_update = time.monotonic()
        except Exception as exc:
            print("ToF callback error:", exc)

    def get_front_cm(self):
        if self.front_cm is None or self.tof_last_update is None:
            return None
        if time.monotonic() - self.tof_last_update > config.TOF_STALE_SEC:
            return None
        return self.front_cm

    # ========================================================
    # SHARP
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

        return raw, self.adc_to_cm(
            ema_val,
            self.calibration_for_sensor(sensor_id),
        )

    def read_left_sharp(self):
        return self.read_sharp_raw_and_cm(config.SHARP_LEFT_ID)

    def read_right_sharp(self):
        return self.read_sharp_raw_and_cm(config.SHARP_RIGHT_ID)

    # ========================================================
    # V11 DIGITAL IR SIDE CONFIRMATION
    # ========================================================

    def _read_ir_level(self, sensor_id):
        """Read one digital IR level, with the old ADC fallback if needed."""
        try:
            level = self.sensor_adapter.get_io_level(
                id=sensor_id,
                port=config.IR_PORT,
            )
            if level is None:
                return None
            return 1 if int(level) else 0
        except Exception:
            try:
                raw = self.sensor_adapter.get_adc(
                    id=sensor_id,
                    port=config.IR_PORT,
                )
                return 1 if raw > config.IR_ADC_FALLBACK_THRESHOLD else 0
            except Exception:
                return None

    @staticmethod
    def _stable_ir_wall(buffer, wall_level):
        """Majority vote; None until enough fresh samples exist."""
        if len(buffer) < config.IR_MIN_SAMPLES:
            return None

        wall_votes = sum(1 for level in buffer if level == wall_level)
        clear_votes = len(buffer) - wall_votes

        if wall_votes == clear_votes:
            return None
        return wall_votes > clear_votes

    def _read_ir(self, sensor_id, buffer, wall_level):
        level = self._read_ir_level(sensor_id)
        if level is not None:
            buffer.append(level)
        return level, self._stable_ir_wall(buffer, wall_level)

    def read_left_ir(self):
        """Return (raw_level, stable_wall_bool_or_None)."""
        return self._read_ir(
            config.IR_LEFT_ID,
            self.ir_left_buffer,
            config.IR_LEFT_WALL_LEVEL,
        )

    def read_right_ir(self):
        """Return (raw_level, stable_wall_bool_or_None)."""
        return self._read_ir(
            config.IR_RIGHT_ID,
            self.ir_right_buffer,
            config.IR_RIGHT_WALL_LEVEL,
        )

    def read_ir_digital_io(self):
        """Backward-compatible old API: return the LEFT raw digital level."""
        level, _ = self.read_left_ir()
        return level

    # ========================================================
    # RESET AFTER TURN
    # ========================================================

    def reset_filters(self):
        self.tof_buffer.clear()
        self.sharp_left_buffer.clear()
        self.sharp_right_buffer.clear()
        self.ir_left_buffer.clear()
        self.ir_right_buffer.clear()

        self.sharp_left_ema = None
        self.sharp_right_ema = None
        self.front_cm = None
        self.tof_last_update = None

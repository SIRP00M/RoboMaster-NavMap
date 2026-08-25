"""Sensor acquisition, calibration, filtering, and timeout handling."""

import math
import config

# ==================== SENSOR MANAGER ====================
"""Sensor reading, calibration and filtering."""

import statistics
import time
from collections import deque



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

        # V10.1: last-valid Sharp cache. get_adc() may return None after an
        # SDK send_sync_msg timeout without raising an exception.
        self.sharp_last_valid = {
            config.SHARP_LEFT_ID: {"raw": None, "cm": None, "time": None},
            config.SHARP_RIGHT_ID: {"raw": None, "cm": None, "time": None},
        }
        self.sharp_invalid_count = {
            config.SHARP_LEFT_ID: 0,
            config.SHARP_RIGHT_ID: 0,
        }
        self.sharp_last_warn_time = {
            config.SHARP_LEFT_ID: 0.0,
            config.SHARP_RIGHT_ID: 0.0,
        }

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

    @staticmethod
    def _valid_adc(raw):
        if raw is None:
            return False
        try:
            value = float(raw)
        except (TypeError, ValueError):
            return False
        return math.isfinite(value) and 0.0 <= value <= 1023.0

    def _cached_sharp(self, sensor_id):
        cache = self.sharp_last_valid[sensor_id]
        if cache["time"] is None or cache["cm"] is None:
            return None, None, None
        age = time.monotonic() - cache["time"]
        if age <= config.SHARP_STALE_HOLD_SEC:
            return cache["raw"], cache["cm"], age
        return None, None, age

    def _warn_invalid_sharp(self, sensor_id, message):
        now = time.monotonic()
        if now - self.sharp_last_warn_time[sensor_id] >= config.SHARP_INVALID_WARN_INTERVAL_SEC:
            side = "LEFT" if sensor_id == config.SHARP_LEFT_ID else "RIGHT"
            print(f">>> SHARP {side} WARNING: {message}")
            self.sharp_last_warn_time[sensor_id] = now

    def read_sharp_raw_and_cm(self, sensor_id):
        """Read one Sharp sensor without letting SDK timeouts crash the run.

        RoboMaster's synchronous get_adc() can log a send_sync_msg timeout and
        return None.  None must never enter statistics.median().  For a short
        outage we return the last valid filtered distance; if the cache is too
        old we return (None, None), which makes the main loop stop safely until
        the sensor recovers.
        """
        try:
            raw = self.sensor_adapter.get_adc(
                id=sensor_id,
                port=config.SENSOR_PORT,
            )
        except Exception as exc:
            raw = None
            self._warn_invalid_sharp(sensor_id, f"read exception: {exc}")

        if not self._valid_adc(raw):
            self.sharp_invalid_count[sensor_id] += 1
            cached_raw, cached_cm, age = self._cached_sharp(sensor_id)
            if cached_cm is not None:
                self._warn_invalid_sharp(
                    sensor_id,
                    f"invalid ADC={raw!r}; using cached value age={age:.2f}s "
                    f"misses={self.sharp_invalid_count[sensor_id]}",
                )
                return cached_raw, cached_cm

            age_text = "none" if age is None else f"{age:.2f}s"
            self._warn_invalid_sharp(
                sensor_id,
                f"invalid ADC={raw!r}; no fresh cache (age={age_text}) "
                f"misses={self.sharp_invalid_count[sensor_id]}",
            )
            return None, None

        raw = int(round(float(raw)))
        self.sharp_invalid_count[sensor_id] = 0

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
        cm = self.adc_to_cm(ema_val, table)
        self.sharp_last_valid[sensor_id] = {
            "raw": raw,
            "cm": cm,
            "time": time.monotonic(),
        }
        return raw, cm

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

        now = time.monotonic()
        for sensor_id in (config.SHARP_LEFT_ID, config.SHARP_RIGHT_ID):
            self.sharp_last_valid[sensor_id] = {"raw": None, "cm": None, "time": None}
            self.sharp_invalid_count[sensor_id] = 0
            self.sharp_last_warn_time[sensor_id] = min(
                self.sharp_last_warn_time.get(sensor_id, 0.0), now
            )

"""Forward speed and side-wall motion controller."""

import time

import config


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


class MotionController:
    def __init__(self):
        self.side_owner = "NONE"
        self.side_owner_since = 0.0

    # ========================================================
    # OWNER STATE
    # ========================================================

    def reset_side_owner(self):
        self.side_owner = "NONE"
        self.side_owner_since = 0.0

    # ========================================================
    # FRONT SPEED CONTROL
    # ========================================================

    @staticmethod
    def calculate_forward_speed(front_distance):
        """Linear slowdown from SLOW_FRONT_CM to STOP_FRONT_CM."""

        if front_distance is None:
            return config.FORWARD_SPEED

        if front_distance >= config.SLOW_FRONT_CM:
            return config.FORWARD_SPEED

        if front_distance <= config.STOP_FRONT_CM:
            return 0.0

        ratio = (
            (front_distance - config.STOP_FRONT_CM)
            / (config.SLOW_FRONT_CM - config.STOP_FRONT_CM)
        )

        return (
            config.MIN_FORWARD_SPEED
            + ratio * (config.FORWARD_SPEED - config.MIN_FORWARD_SPEED)
        )

    # ========================================================
    # BOTH-WALL OWNER CONTROLLER
    # ========================================================

    def calculate_center_owner(self, sharp_left_cm, sharp_right_cm):
        """ใช้ owner + hysteresis ลดการสลับแก้ซ้าย/ขวารัว ๆ."""

        now = time.time()
        delta = sharp_left_cm - sharp_right_cm
        abs_delta = abs(delta)

        if self.side_owner == "NONE":
            if abs_delta < config.CENTER_TRIGGER_CM:
                return 0.0, "CENTER_STABLE"

            if delta < 0:
                self.side_owner = "LEFT"
            else:
                self.side_owner = "RIGHT"

            self.side_owner_since = now

        owner_age = now - self.side_owner_since

        if owner_age >= config.CENTER_HOLD_SEC:
            if abs_delta <= config.CENTER_RELEASE_CM:
                self.reset_side_owner()
                return 0.0, "CENTER_RELEASE"

            if (
                self.side_owner == "LEFT"
                and delta >= config.CENTER_TRIGGER_CM
            ):
                self.side_owner = "RIGHT"
                self.side_owner_since = now

            elif (
                self.side_owner == "RIGHT"
                and delta <= -config.CENTER_TRIGGER_CM
            ):
                self.side_owner = "LEFT"
                self.side_owner_since = now

        correction = clamp(
            abs_delta * config.CENTER_KP_STRAFE,
            0.0,
            config.CENTER_MAX_Y,
        )

        if self.side_owner == "LEFT":
            y_cmd = +correction * config.Y_DIR_SIGN
            return y_cmd, "CENTER_LEFT_OWNER"

        if self.side_owner == "RIGHT":
            y_cmd = -correction * config.Y_DIR_SIGN
            return y_cmd, "CENTER_RIGHT_OWNER"

        return 0.0, "CENTER_STABLE"

    # ========================================================
    # SIDE MOTION CONTROL
    # ========================================================

    def calculate_motion_control(
        self,
        raw_adc_l,
        sharp_left_cm,
        raw_adc_r,
        sharp_right_cm,
        ir_left_wall,
    ):
        """Sharp คุมเฉพาะแกน Y; z คงเป็น 0 ระหว่างตามกำแพง."""

        # เก็บ parameter นี้ไว้เพื่อ interface เดิม แม้ logic ปัจจุบันยังไม่ได้ใช้
        _ = ir_left_wall

        if sharp_left_cm is None or sharp_right_cm is None:
            self.reset_side_owner()
            return 0.0, 0.0, "NO_SENSOR"

        # ทั้งสองข้างชิดเกินไป
        if (
            sharp_left_cm <= config.SIDE_TOO_CLOSE_CM
            and sharp_right_cm <= config.SIDE_TOO_CLOSE_CM
        ):
            self.reset_side_owner()
            return 0.0, 0.0, "BOTH_TOO_CLOSE"

        # ซ้ายชิดเกินไป -> หนีขวา
        if raw_adc_l >= 600 or sharp_left_cm <= config.SIDE_TOO_CLOSE_CM:
            self.reset_side_owner()
            y_cmd = +config.ESCAPE_Y_SPEED * config.Y_DIR_SIGN
            return y_cmd, 0.0, "ESCAPE_LEFT"

        # ขวาชิดเกินไป -> หนีซ้าย
        if raw_adc_r >= 600 or sharp_right_cm <= config.SIDE_TOO_CLOSE_CM:
            self.reset_side_owner()
            y_cmd = -config.ESCAPE_Y_SPEED * config.Y_DIR_SIGN
            return y_cmd, 0.0, "ESCAPE_RIGHT"

        left_wall = sharp_left_cm < config.SIDE_WALL_DETECT_CM
        right_wall = sharp_right_cm < config.SIDE_WALL_DETECT_CM

        # กำแพงสองข้าง -> owner controller
        if left_wall and right_wall:
            y_cmd, mode = self.calculate_center_owner(
                sharp_left_cm,
                sharp_right_cm,
            )
            return y_cmd, 0.0, mode

        # กำแพงซ้ายอย่างเดียว
        if left_wall:
            self.reset_side_owner()
            error = sharp_left_cm - config.TARGET_LEFT_CM

            if abs(error) <= config.SIDE_DEADBAND_CM:
                y_cmd = 0.0
            else:
                y_cmd = clamp(
                    -error * config.SIDE_KP_STRAFE * config.Y_DIR_SIGN,
                    -config.SIDE_MAX_Y,
                    config.SIDE_MAX_Y,
                )

            return y_cmd, 0.0, "FOLLOW_LEFT"

        # กำแพงขวาอย่างเดียว
        if right_wall:
            self.reset_side_owner()
            error = sharp_right_cm - config.TARGET_RIGHT_CM

            if abs(error) <= config.SIDE_DEADBAND_CM:
                y_cmd = 0.0
            else:
                y_cmd = clamp(
                    error * config.SIDE_KP_STRAFE * config.Y_DIR_SIGN,
                    -config.SIDE_MAX_Y,
                    config.SIDE_MAX_Y,
                )

            return y_cmd, 0.0, "FOLLOW_RIGHT"

        self.reset_side_owner()
        return 0.0, 0.0, "OPEN_SPACE"

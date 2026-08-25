"""Forward, wall-centering, escape, and heading-hold control logic."""

import config
from pose_tracker import normalize_angle_deg, shortest_angle_error_deg

# ==================== MOTION CONTROLLER ====================
"""Forward speed, side-wall and heading-hold motion controller."""

import time



def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


class MotionController:
    def __init__(self):
        self.side_owner = "NONE"
        self.side_owner_since = 0.0

        self.left_wall_present = False
        self.right_wall_present = False

        # Absolute yaw grid. Initial physical yaw = internal N.
        self.heading_base_yaw = None
        self.heading_target_yaw = None
        self.heading_right_step_sign = None
        self.heading_recovering = False

        # V10 external corridor reference for slow IMU-yaw drift correction.
        self._corr_cal_x = None
        self._corr_cal_y = None
        self._corr_cal_left = None
        self._corr_cal_right = None
        self._corr_cal_heading_index = None

    # ========================================================
    # RESET / STATE
    # ========================================================

    def reset_side_owner(self):
        self.side_owner = "NONE"
        self.side_owner_since = 0.0

    def reset_wall_states(self):
        self.left_wall_present = False
        self.right_wall_present = False

    def reset_after_turn(self):
        # Important: do NOT reset absolute heading target here.
        self.reset_side_owner()
        self.reset_wall_states()
        self.reset_corridor_heading_calibration()

    def reset_corridor_heading_calibration(self):
        self._corr_cal_x = None
        self._corr_cal_y = None
        self._corr_cal_left = None
        self._corr_cal_right = None
        self._corr_cal_heading_index = None

    # ========================================================
    # HEADING HOLD
    # ========================================================

    def initialize_heading(self, yaw_deg, pose_tracker=None):
        if yaw_deg is None:
            return False

        self.heading_base_yaw = normalize_angle_deg(yaw_deg)
        self.heading_target_yaw = self.heading_base_yaw
        self.heading_recovering = False

        if pose_tracker is not None:
            if pose_tracker.get_move_to_yaw_sign() is None:
                pose_tracker.set_move_to_yaw_sign(config.DEFAULT_MOVE_TO_YAW_SIGN)
            self._learn_heading_axis_from_pose(pose_tracker)

        return True

    def _learn_heading_axis_from_pose(self, pose_tracker):
        """Learn attitude-yaw direction for one logical RIGHT grid step."""
        sign_map = pose_tracker.get_move_to_yaw_sign()
        if sign_map not in (-1, 1):
            return False

        # Logical RIGHT uses TURN_RIGHT_DEG (-90 by default).
        # actual yaw delta sign = command sign * move_to_yaw_sign.
        right_command_sign = 1 if (config.TURN_RIGHT_DEG * config.Z_DIR_SIGN) > 0 else -1
        self.heading_right_step_sign = right_command_sign * sign_map
        return True

    @staticmethod
    def _signed_cardinal_step(heading_index):
        # 0=N, 1=E/right, 2=S, 3=W/left
        return {0: 0, 1: 1, 2: 2, 3: -1}[heading_index % 4]

    def set_heading_index(self, heading_index, pose_tracker=None):
        if self.heading_base_yaw is None:
            current_yaw = pose_tracker.get_yaw() if pose_tracker is not None else None
            if current_yaw is None:
                return None
            self.initialize_heading(current_yaw, pose_tracker=pose_tracker)

        if pose_tracker is not None:
            self._learn_heading_axis_from_pose(pose_tracker)

        if self.heading_right_step_sign not in (-1, 1):
            return self.heading_target_yaw

        step = self._signed_cardinal_step(heading_index)
        self.heading_target_yaw = normalize_angle_deg(
            self.heading_base_yaw
            + self.heading_right_step_sign * 90.0 * step
        )
        return self.heading_target_yaw

    def heading_error(self, current_yaw):
        if self.heading_target_yaw is None or current_yaw is None:
            return None
        return shortest_angle_error_deg(self.heading_target_yaw, current_yaw)

    def calculate_heading_hold(self, current_yaw, pose_tracker, recover=False):
        """Return (z_command_deg_s, yaw_error_deg)."""
        if not config.ENABLE_HEADING_HOLD:
            return 0.0, None

        error = self.heading_error(current_yaw)
        if error is None:
            return 0.0, None

        if abs(error) <= config.HEADING_DEADBAND_DEG:
            return 0.0, error

        # Heading hold is sent through chassis.drive_speed(), NOT chassis.move().
        # These two RoboMaster APIs have separate yaw-sign mappings on this robot.
        sign_map = pose_tracker.get_drive_to_yaw_sign()
        if sign_map not in (-1, 1):
            return 0.0, error

        max_z = (
            config.HEADING_RECOVER_MAX_Z_SPEED
            if recover
            else config.HEADING_MAX_Z_SPEED
        )

        desired_yaw_rate = clamp(
            error * config.HEADING_KP_Z,
            -max_z,
            max_z,
        )

        # drive_speed z sign -> attitude yaw sign through DRIVE sign_map.
        # With DRIVE_TO_YAW_SIGN=+1, positive error correctly yields +z.
        z_cmd = desired_yaw_rate / sign_map
        return z_cmd, error

    def apply_heading_hold(self, x, y, current_yaw, pose_tracker, mode):
        """Add z correction and stop translation if yaw has drifted too far."""
        if not config.ENABLE_HEADING_HOLD:
            return x, y, 0.0, mode, None

        error = self.heading_error(current_yaw)
        if error is None:
            return x, y, 0.0, mode, None

        if self.heading_recovering:
            if abs(error) <= config.HEADING_RECOVER_RELEASE_DEG:
                self.heading_recovering = False
        elif abs(error) >= config.HEADING_RECOVER_TRIGGER_DEG:
            self.heading_recovering = True

        if self.heading_recovering:
            z_cmd, error = self.calculate_heading_hold(
                current_yaw,
                pose_tracker,
                recover=True,
            )
            return 0.0, 0.0, z_cmd, "HEADING_RECOVER", error

        z_cmd, error = self.calculate_heading_hold(
            current_yaw,
            pose_tracker,
            recover=False,
        )
        return x, y, z_cmd, mode, error

    def update_corridor_heading_reference(
        self,
        left_cm,
        right_cm,
        front_cm,
        pose_tracker,
        heading_index,
        allow=True,
    ):
        """Slowly trim the absolute yaw grid from straight-wall geometry.

        A decreasing LEFT distance means the chassis is pointing into the left
        wall; an increasing RIGHT distance says the same thing.  Convert that
        physical skew into the attitude-yaw sign learned for a logical RIGHT
        turn, then move heading_base_yaw/target_yaw only a small amount.
        """
        if not getattr(config, "ENABLE_CORRIDOR_HEADING_CALIBRATION", False):
            return None
        if not allow or pose_tracker is None:
            self.reset_corridor_heading_calibration()
            return None
        if front_cm is None or front_cm < config.CORRIDOR_CAL_MIN_FRONT_CM:
            self.reset_corridor_heading_calibration()
            return None
        if self.heading_right_step_sign not in (-1, 1):
            return None

        x, y, _ = pose_tracker.get_pose()
        if x is None or y is None:
            return None

        def wall_ok(v):
            return (
                v is not None
                and config.CORRIDOR_CAL_MIN_WALL_CM <= v <= config.CORRIDOR_CAL_MAX_WALL_CM
            )

        left_ok = wall_ok(left_cm)
        right_ok = wall_ok(right_cm)
        if not (left_ok or right_ok):
            self.reset_corridor_heading_calibration()
            return None

        if (
            self._corr_cal_x is None
            or self._corr_cal_heading_index != int(heading_index)
        ):
            self._corr_cal_x, self._corr_cal_y = x, y
            self._corr_cal_left = float(left_cm) if left_ok else None
            self._corr_cal_right = float(right_cm) if right_ok else None
            self._corr_cal_heading_index = int(heading_index)
            return None

        travel = ((x - self._corr_cal_x) ** 2 + (y - self._corr_cal_y) ** 2) ** 0.5
        if travel < config.CORRIDOR_CAL_MIN_TRAVEL_M:
            return None

        import math
        estimates = []
        # Positive estimate below means an attitude-yaw adjustment in the
        # direction required to bring the chassis parallel to the wall.
        if left_ok and self._corr_cal_left is not None:
            dd_m = (float(left_cm) - self._corr_cal_left) / 100.0
            a = math.degrees(math.atan2(dd_m, max(travel, 1e-6)))
            # LEFT distance increasing -> chassis points right -> correct left.
            estimates.append(-self.heading_right_step_sign * a)
        if right_ok and self._corr_cal_right is not None:
            dd_m = (float(right_cm) - self._corr_cal_right) / 100.0
            a = math.degrees(math.atan2(dd_m, max(travel, 1e-6)))
            # RIGHT distance increasing -> chassis points left -> correct right.
            estimates.append(self.heading_right_step_sign * a)

        # Start a fresh baseline whether or not this estimate is accepted.
        self._corr_cal_x, self._corr_cal_y = x, y
        self._corr_cal_left = float(left_cm) if left_ok else None
        self._corr_cal_right = float(right_cm) if right_ok else None
        self._corr_cal_heading_index = int(heading_index)

        if not estimates:
            return None
        estimate = sum(estimates) / len(estimates)
        if abs(estimate) > config.CORRIDOR_CAL_MAX_ESTIMATE_DEG:
            return None

        step = clamp(
            estimate * config.CORRIDOR_CAL_ALPHA,
            -config.CORRIDOR_CAL_MAX_STEP_DEG,
            config.CORRIDOR_CAL_MAX_STEP_DEG,
        )
        if abs(step) < 0.02:
            return step

        self.heading_base_yaw = normalize_angle_deg(self.heading_base_yaw + step)
        if self.heading_target_yaw is not None:
            self.heading_target_yaw = normalize_angle_deg(self.heading_target_yaw + step)

        if abs(step) >= config.CORRIDOR_CAL_LOG_MIN_STEP_DEG:
            print(
                f">>> CORRIDOR_HEADING_CAL estimate={estimate:+.2f}deg "
                f"trim={step:+.2f}deg target={self.heading_target_yaw:+.2f}"
            )
        return step

    # ========================================================
    # FRONT SPEED CONTROL
    # ========================================================

    @staticmethod
    def calculate_forward_speed(front_distance):
        if front_distance is None:
            return config.UNKNOWN_FRONT_SPEED

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
    # WALL HYSTERESIS
    # ========================================================

    @staticmethod
    def _update_wall_state(distance_cm, current_state):
        if distance_cm is None:
            return False

        if current_state:
            return distance_cm < config.SIDE_WALL_EXIT_CM

        return distance_cm < config.SIDE_WALL_ENTER_CM

    def update_wall_states(self, sharp_left_cm, sharp_right_cm):
        # IMPORTANT: an observed maze opening must not simultaneously be treated
        # as a wall by the centering controller. The old thresholds overlapped
        # (opening >=20 cm while wall-enter <28 cm), which caused violent y kicks.
        left_is_opening = (
            sharp_left_cm is not None
            and sharp_left_cm >= config.EXPLORATION_SIDE_OPEN_CM
        )
        right_is_opening = (
            sharp_right_cm is not None
            and sharp_right_cm >= config.EXPLORATION_SIDE_OPEN_CM
        )

        if left_is_opening:
            self.left_wall_present = False
        else:
            self.left_wall_present = self._update_wall_state(
                sharp_left_cm,
                self.left_wall_present,
            )

        if right_is_opening:
            self.right_wall_present = False
        else:
            self.right_wall_present = self._update_wall_state(
                sharp_right_cm,
                self.right_wall_present,
            )

        return self.left_wall_present, self.right_wall_present

    # ========================================================
    # BOTH-WALL OWNER CONTROLLER
    # ========================================================

    def calculate_center_owner(self, sharp_left_cm, sharp_right_cm):
        now = time.time()
        delta = sharp_left_cm - sharp_right_cm
        abs_delta = abs(delta)

        if self.side_owner == "NONE":
            if abs_delta < config.CENTER_TRIGGER_CM:
                return 0.0, "CENTER_STABLE"

            self.side_owner = "LEFT" if delta < 0 else "RIGHT"
            self.side_owner_since = now

        owner_age = now - self.side_owner_since

        if owner_age >= config.CENTER_HOLD_SEC:
            if abs_delta <= config.CENTER_RELEASE_CM:
                self.reset_side_owner()
                return 0.0, "CENTER_RELEASE"

            if self.side_owner == "LEFT" and delta >= config.CENTER_TRIGGER_CM:
                self.side_owner = "RIGHT"
                self.side_owner_since = now
            elif self.side_owner == "RIGHT" and delta <= -config.CENTER_TRIGGER_CM:
                self.side_owner = "LEFT"
                self.side_owner_since = now

        correction = clamp(
            abs_delta * config.CENTER_KP_STRAFE,
            0.0,
            config.CENTER_MAX_Y,
        )

        if self.side_owner == "LEFT":
            return +correction * config.Y_DIR_SIGN, "CENTER_LEFT_OWNER"
        if self.side_owner == "RIGHT":
            return -correction * config.Y_DIR_SIGN, "CENTER_RIGHT_OWNER"

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
        """Sharp controls Y; heading hold in main/controller controls Z."""
        _ = raw_adc_l, raw_adc_r, ir_left_wall

        if sharp_left_cm is None or sharp_right_cm is None:
            self.reset_side_owner()
            self.reset_wall_states()
            return 0.0, 0.0, "NO_SENSOR"

        if (
            sharp_left_cm <= config.SIDE_TOO_CLOSE_CM
            and sharp_right_cm <= config.SIDE_TOO_CLOSE_CM
        ):
            self.reset_side_owner()
            return 0.0, 0.0, "BOTH_TOO_CLOSE"

        # Use FILTERED distance only. A single raw ADC spike used to trigger
        # full ±0.10 m/s strafes even while filtered distance was ~8 cm.
        if sharp_left_cm <= config.SIDE_TOO_CLOSE_CM:
            self.reset_side_owner()
            return +config.ESCAPE_Y_SPEED * config.Y_DIR_SIGN, 0.0, "ESCAPE_LEFT"

        if sharp_right_cm <= config.SIDE_TOO_CLOSE_CM:
            self.reset_side_owner()
            return -config.ESCAPE_Y_SPEED * config.Y_DIR_SIGN, 0.0, "ESCAPE_RIGHT"

        left_wall, right_wall = self.update_wall_states(
            sharp_left_cm,
            sharp_right_cm,
        )

        if left_wall and right_wall:
            y_cmd, mode = self.calculate_center_owner(
                sharp_left_cm,
                sharp_right_cm,
            )
            return y_cmd, 0.0, mode

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

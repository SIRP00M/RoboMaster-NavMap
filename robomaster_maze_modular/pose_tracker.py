"""Thread-safe RoboMaster odometry and attitude tracking."""

import config

# ==================== POSE TRACKER ====================
"""Thread-safe chassis position + attitude tracker."""

import threading



def normalize_angle_deg(angle):
    """Normalize angle to [-180, 180)."""
    return (float(angle) + 180.0) % 360.0 - 180.0


def shortest_angle_error_deg(target, current):
    """Signed shortest error needed to rotate current -> target."""
    return normalize_angle_deg(float(target) - float(current))


class PoseTracker:
    """Hold odometry position and chassis attitude from separate subscriptions."""

    def __init__(self):
        self._lock = threading.Lock()

        self.x = None
        self.y = None
        self.position_z = None

        self.yaw_deg = None
        self.pitch_deg = None
        self.roll_deg = None

        # Keep separate sign mappings for the two RoboMaster APIs.
        # +1 = command z sign and attitude-yaw sign agree.
        # -1 = command z sign and attitude-yaw sign are opposite.
        #
        # IMPORTANT: on this robot chassis.move(z) and drive_speed(z) do NOT
        # behave with the same yaw-sign convention (confirmed by real logs).
        self.move_to_yaw_sign = (
            config.DEFAULT_MOVE_TO_YAW_SIGN
            if hasattr(config, "DEFAULT_MOVE_TO_YAW_SIGN")
            else None
        )
        self.drive_to_yaw_sign = (
            config.DEFAULT_DRIVE_TO_YAW_SIGN
            if hasattr(config, "DEFAULT_DRIVE_TO_YAW_SIGN")
            else None
        )

    def position_callback(self, data):
        """RoboMaster sub_position callback: data = (x, y, z)."""
        try:
            if data is None or len(data) < 3:
                return

            x, y, z = data[:3]

            with self._lock:
                self.x = float(x)
                self.y = float(y)
                self.position_z = float(z)

        except Exception as exc:
            print("Position callback error:", exc)

    def attitude_callback(self, data):
        """RoboMaster sub_attitude callback: data = (yaw, pitch, roll)."""
        try:
            if data is None or len(data) < 3:
                return

            yaw, pitch, roll = data[:3]

            with self._lock:
                self.yaw_deg = normalize_angle_deg(yaw)
                self.pitch_deg = float(pitch)
                self.roll_deg = float(roll)

        except Exception as exc:
            print("Attitude callback error:", exc)

    def get_pose(self):
        """Return (x, y, yaw_deg). yaw comes from sub_attitude()."""
        with self._lock:
            return self.x, self.y, self.yaw_deg

    def get_position(self):
        with self._lock:
            return self.x, self.y, self.position_z

    def get_yaw(self):
        with self._lock:
            return self.yaw_deg

    def has_pose(self):
        x, y, _ = self.get_position()
        return x is not None and y is not None

    def has_yaw(self):
        return self.get_yaw() is not None

    def set_move_to_yaw_sign(self, sign):
        if sign not in (-1, 1):
            return
        with self._lock:
            self.move_to_yaw_sign = sign

    def get_move_to_yaw_sign(self):
        with self._lock:
            return self.move_to_yaw_sign

    def set_drive_to_yaw_sign(self, sign):
        if sign not in (-1, 1):
            return
        with self._lock:
            self.drive_to_yaw_sign = sign

    def get_drive_to_yaw_sign(self):
        with self._lock:
            return self.drive_to_yaw_sign

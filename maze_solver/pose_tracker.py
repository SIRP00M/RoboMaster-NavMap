"""Thread-safe chassis position + attitude tracker."""

import threading
import config


def normalize_angle_deg(angle):
    return (float(angle) + 180.0) % 360.0 - 180.0


def shortest_angle_error_deg(target, current):
    return normalize_angle_deg(float(target) - float(current))


class PoseTracker:
    def __init__(self):
        self._lock = threading.Lock()
        self.x = None
        self.y = None
        self.position_z = None
        self.yaw_deg = None
        self.pitch_deg = None
        self.roll_deg = None
        self.move_to_yaw_sign = config.DEFAULT_MOVE_TO_YAW_SIGN
        self.drive_to_yaw_sign = config.DEFAULT_DRIVE_TO_YAW_SIGN

    def position_callback(self, data):
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
        if sign in (-1, 1):
            with self._lock:
                self.move_to_yaw_sign = sign

    def get_move_to_yaw_sign(self):
        with self._lock:
            return self.move_to_yaw_sign

    def set_drive_to_yaw_sign(self, sign):
        if sign in (-1, 1):
            with self._lock:
                self.drive_to_yaw_sign = sign

    def get_drive_to_yaw_sign(self):
        with self._lock:
            return self.drive_to_yaw_sign

"""Passive SLAM-style occupancy-grid mapper. It does not control robot motion."""

import json
import math
import time
import config
from motion_controller import clamp
from pose_tracker import normalize_angle_deg, shortest_angle_error_deg

# ==================== SLAM-STYLE PASSIVE MAPPER ====================
"""Occupancy-grid mapper for the V6 maze explorer.

Map convention:
    +Y = NORTH / initial robot forward
    +X = EAST / initial robot right
    theta 0 deg = NORTH, +90 deg = EAST

This module is passive: it reads pose/sensors and writes map files. It never
changes the pose used by MotionController or TremauxExplorer.
"""

import csv
import os
from dataclasses import dataclass as _map_dataclass


def _map_cfg(name, default):
    return getattr(config, name, default)


def _map_safe_float(value):
    if value is None:
        return None
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(value):
        return None
    return value


@_map_dataclass
class _MapSample:
    index: int
    time_sec: float
    raw_x: float
    raw_y: float
    yaw_deg: float
    base_x: float
    base_y: float
    map_x: float
    map_y: float
    theta_deg: float
    heading_index: int
    front_cm: object
    left_cm: object
    right_cm: object
    ir_value: object
    mode: str
    map_ranges: bool


class SLAMStyleMazeMapper:
    SENSOR_FRONT = "front_tof"
    SENSOR_LEFT = "left_sharp"
    SENSOR_RIGHT = "right_sharp"
    SENSOR_IR = "left_front_ir"

    def __init__(self, output_dir=None):
        self.enabled = bool(_map_cfg("ENABLE_MAPPING", True))
        self.output_dir = output_dir or _map_cfg("MAP_OUTPUT_DIR", "mapping_output")
        self.initialized = False
        self.start_raw_x = None
        self.start_raw_y = None
        self.start_yaw = None
        self.start_monotonic = None
        self.global_corr_x = 0.0
        self.global_corr_y = 0.0
        self.samples = []
        self.grid = {}              # (gx,gy) -> evidence score
        self.wall_points = []
        self.node_anchors = {}
        self.node_events = []
        self.loop_closures = []
        self.exit_event = None
        self.last_known_junction_sample_index = None
        self.last_record_monotonic = None
        self.last_autosave_monotonic = None
        self.yaw_fallback_count = 0
        self.ir_confirm_count = 0
        self.ir_fallback_count = 0
        self._warned_sample_limit = False
        self.position_rotation_deg = float(_map_cfg("MAP_POSITION_ROTATION_DEG", 0.0))
        self.position_auto_aligned = not bool(_map_cfg("MAP_AUTO_ALIGN_INITIAL_PATH", True))
        self._auto_align_reported = False
        self._last_wall_hit = {}

    # --------------------------------------------------------
    # Coordinate transform
    # --------------------------------------------------------
    def initialize(self, raw_x, raw_y, start_yaw, heading_index=0):
        if not self.enabled:
            return None
        raw_x = _map_safe_float(raw_x)
        raw_y = _map_safe_float(raw_y)
        start_yaw = _map_safe_float(start_yaw)
        if raw_x is None or raw_y is None:
            raise ValueError("Mapper requires valid starting x/y")
        if start_yaw is None:
            start_yaw = 0.0

        self.start_raw_x = raw_x
        self.start_raw_y = raw_y
        self.start_yaw = normalize_angle_deg(start_yaw)
        self.start_monotonic = time.monotonic()
        self.last_autosave_monotonic = self.start_monotonic
        self.initialized = True
        os.makedirs(self.output_dir, exist_ok=True)
        if bool(_map_cfg("MAP_CLEAR_OUTPUT_ON_START", True)):
            self._clear_old_outputs()

        return self.record_pose(
            raw_x, raw_y, start_yaw,
            heading_index=heading_index,
            mode="START", force=True,
        )

    def _clear_old_outputs(self):
        for name in (
            "trajectory.csv", "wall_points.csv", "occupancy_grid.csv",
            "occupancy_grid.json", "nodes.json", "loop_closures.json",
            "mapping_summary.json", "exit.json", "maze_map.svg", "maze_map.png",
        ):
            path = os.path.join(self.output_dir, name)
            try:
                if os.path.isfile(path):
                    os.remove(path)
            except OSError:
                pass

    def _raw_position_unrotated(self, raw_x, raw_y):
        dx = float(raw_x) - self.start_raw_x
        dy = float(raw_y) - self.start_raw_y
        if bool(_map_cfg("MAP_SWAP_RAW_XY", False)):
            dx, dy = dy, dx
        dx *= float(_map_cfg("MAP_RAW_X_SIGN", -1.0))
        dy *= float(_map_cfg("MAP_RAW_Y_SIGN", +1.0))
        return dx, dy

    @staticmethod
    def _rotate_xy(dx, dy, rot_deg):
        if abs(rot_deg) <= 1e-12:
            return dx, dy
        a = math.radians(rot_deg)
        c, ss = math.cos(a), math.sin(a)
        return c * dx - ss * dy, ss * dx + c * dy

    def _raw_position_to_base_map(self, raw_x, raw_y):
        dx, dy = self._raw_position_unrotated(raw_x, raw_y)
        return self._rotate_xy(dx, dy, self.position_rotation_deg)

    def _maybe_auto_align_position(self, raw_x, raw_y, heading_index):
        if self.position_auto_aligned:
            return False
        if not bool(_map_cfg("MAP_AUTO_ALIGN_INITIAL_PATH", True)):
            self.position_auto_aligned = True
            return False
        if heading_index is None:
            return False
        wanted = int(_map_cfg("MAP_AUTO_ALIGN_MAX_HEADING_INDEX", 0)) % 4
        if int(heading_index) % 4 != wanted:
            # The robot turned before enough straight travel was collected.
            # Keep the configured rotation rather than learning from a corner.
            return False

        dx, dy = self._raw_position_unrotated(raw_x, raw_y)
        dist = math.hypot(dx, dy)
        need = float(_map_cfg("MAP_AUTO_ALIGN_MIN_TRAVEL_M", 0.18))
        if dist < need:
            return False

        # Angle measured clockwise from +Y in our map convention. Rotating the
        # raw displacement by this amount places the initial travel on +Y.
        auto_rot = math.degrees(math.atan2(dx, dy))
        fixed = float(_map_cfg("MAP_POSITION_ROTATION_DEG", 0.0))
        self.position_rotation_deg = normalize_angle_deg(fixed + auto_rot)
        self.position_auto_aligned = True

        # This happens before the first meaningful loop closure. Reproject the
        # early samples so the whole map shares one frame, then rebuild evidence.
        if not self.loop_closures and abs(self.global_corr_x) < 1e-9 and abs(self.global_corr_y) < 1e-9:
            for old_sample in self.samples:
                bx, by = self._raw_position_to_base_map(old_sample.raw_x, old_sample.raw_y)
                old_sample.base_x = bx
                old_sample.base_y = by
                old_sample.map_x = bx
                old_sample.map_y = by
            self._rebuild_grid()

        if not self._auto_align_reported:
            print(
                f">>> MAP AUTO ALIGN rotation={self.position_rotation_deg:+.1f} deg "
                f"from initial travel={dist:.3f} m"
            )
            self._auto_align_reported = True
        return True

    def _yaw_to_map_theta(self, yaw_deg, heading_index=None):
        yaw_deg = _map_safe_float(yaw_deg)
        if yaw_deg is None:
            return 0.0 if heading_index is None else float((int(heading_index) % 4) * 90)
        theta = normalize_angle_deg(
            (yaw_deg - self.start_yaw) * float(_map_cfg("MAP_YAW_RIGHT_SIGN", +1.0))
        )
        if heading_index is None:
            return theta
        expected = normalize_angle_deg(float((int(heading_index) % 4) * 90))
        if bool(_map_cfg("MAP_SENSOR_USE_CARDINAL_HEADING", True)):
            return expected
        error = shortest_angle_error_deg(expected, theta)
        if abs(error) <= float(_map_cfg("MAP_YAW_CARDINAL_MAX_ERROR_DEG", 22.0)):
            return theta
        if bool(_map_cfg("MAP_FALLBACK_TO_CARDINAL_HEADING", True)):
            self.yaw_fallback_count += 1
            return expected
        return theta

    # --------------------------------------------------------
    # Sampling / sensor integration
    # --------------------------------------------------------
    def update(
        self, raw_x, raw_y, yaw_deg,
        front_cm=None, left_cm=None, right_cm=None, ir_value=None,
        heading_index=None, mode="RUN", map_ranges=True, force=False,
    ):
        if not self.enabled or not self.initialized:
            return None
        raw_x = _map_safe_float(raw_x)
        raw_y = _map_safe_float(raw_y)
        yaw_deg = _map_safe_float(yaw_deg)
        if raw_x is None or raw_y is None:
            return None
        if yaw_deg is None:
            yaw_deg = self.start_yaw

        now = time.monotonic()
        if (
            not force and self.last_record_monotonic is not None
            and now - self.last_record_monotonic < float(_map_cfg("MAP_MIN_RECORD_INTERVAL_SEC", 0.045))
        ):
            return None
        max_samples = int(_map_cfg("MAP_MAX_SAMPLES", 60000))
        if max_samples > 0 and len(self.samples) >= max_samples:
            if not self._warned_sample_limit:
                print("MAPPER WARNING: sample limit reached; mapping samples paused")
                self._warned_sample_limit = True
            return None

        self._maybe_auto_align_position(raw_x, raw_y, heading_index)
        base_x, base_y = self._raw_position_to_base_map(raw_x, raw_y)
        map_x = base_x + self.global_corr_x
        map_y = base_y + self.global_corr_y
        theta = self._yaw_to_map_theta(yaw_deg, heading_index)
        h = -1 if heading_index is None else int(heading_index) % 4

        sample = _MapSample(
            index=len(self.samples),
            time_sec=now - self.start_monotonic,
            raw_x=raw_x, raw_y=raw_y, yaw_deg=yaw_deg,
            base_x=base_x, base_y=base_y, map_x=map_x, map_y=map_y,
            theta_deg=theta, heading_index=h,
            front_cm=_map_safe_float(front_cm),
            left_cm=_map_safe_float(left_cm),
            right_cm=_map_safe_float(right_cm),
            ir_value=ir_value,
            mode=str(mode or ""), map_ranges=bool(map_ranges),
        )
        self.samples.append(sample)
        self.last_record_monotonic = now
        if sample.map_ranges:
            self._integrate_sample(sample)

        autosave = float(_map_cfg("MAP_AUTOSAVE_SEC", 0.0))
        if autosave > 0 and now - self.last_autosave_monotonic >= autosave:
            self.save_all(rebuild=False, quiet=True)
            self.last_autosave_monotonic = now
        return sample

    def record_pose(self, raw_x, raw_y, yaw_deg, heading_index=None, mode="POSE_ONLY", force=True):
        return self.update(
            raw_x, raw_y, yaw_deg,
            heading_index=heading_index, mode=mode,
            map_ranges=False, force=force,
        )

    @staticmethod
    def _forward_right_to_world(x, y, theta_deg, forward_m, right_m):
        a = math.radians(theta_deg)
        fx, fy = math.sin(a), math.cos(a)
        rx, ry = math.cos(a), -math.sin(a)
        return x + forward_m * fx + right_m * rx, y + forward_m * fy + right_m * ry

    def _sensor_params(self, name):
        if name == self.SENSOR_FRONT:
            return dict(
                angle=float(_map_cfg("MAP_FRONT_SENSOR_ANGLE_DEG", 0.0)),
                forward=float(_map_cfg("MAP_FRONT_SENSOR_FORWARD_M", 0.08)),
                right=float(_map_cfg("MAP_FRONT_SENSOR_RIGHT_M", 0.0)),
                min_cm=float(_map_cfg("MAP_TOF_MIN_CM", 4.0)),
                free_max=float(_map_cfg("MAP_TOF_FREE_MAX_CM", 70.0)),
                hit_max=float(_map_cfg("MAP_TOF_OCCUPIED_MAX_CM", 45.0)),
                hit_score=int(_map_cfg("MAP_TOF_HIT_SCORE", 7)),
                free_score=int(_map_cfg("MAP_TOF_FREE_SCORE", -1)),
            )
        if name == self.SENSOR_LEFT:
            return dict(
                angle=float(_map_cfg("MAP_LEFT_SENSOR_ANGLE_DEG", -90.0)),
                forward=float(_map_cfg("MAP_LEFT_SENSOR_FORWARD_M", 0.02)),
                right=float(_map_cfg("MAP_LEFT_SENSOR_RIGHT_M", -0.10)),
                min_cm=float(_map_cfg("MAP_SHARP_MIN_CM", 4.0)),
                free_max=float(_map_cfg("MAP_SHARP_FREE_MAX_CM", 24.0)),
                hit_max=float(_map_cfg("MAP_SHARP_OCCUPIED_MAX_CM", 18.0)),
                hit_score=int(_map_cfg("MAP_SHARP_HIT_SCORE", 5)),
                free_score=int(_map_cfg("MAP_SHARP_FREE_SCORE", -1)),
            )
        if name == self.SENSOR_RIGHT:
            return dict(
                angle=float(_map_cfg("MAP_RIGHT_SENSOR_ANGLE_DEG", +90.0)),
                forward=float(_map_cfg("MAP_RIGHT_SENSOR_FORWARD_M", 0.02)),
                right=float(_map_cfg("MAP_RIGHT_SENSOR_RIGHT_M", +0.10)),
                min_cm=float(_map_cfg("MAP_SHARP_MIN_CM", 4.0)),
                free_max=float(_map_cfg("MAP_SHARP_FREE_MAX_CM", 24.0)),
                hit_max=float(_map_cfg("MAP_SHARP_OCCUPIED_MAX_CM", 18.0)),
                hit_score=int(_map_cfg("MAP_SHARP_HIT_SCORE", 5)),
                free_score=int(_map_cfg("MAP_SHARP_FREE_SCORE", -1)),
            )
        raise ValueError(name)

    def _ray(self, sample, name, distance_cm):
        distance_cm = _map_safe_float(distance_cm)
        if distance_cm is None:
            return None
        p = self._sensor_params(name)
        if distance_cm < p["min_cm"]:
            return None
        used_cm = min(distance_cm, p["free_max"])
        if used_cm <= 0:
            return None
        has_hit = distance_cm <= p["hit_max"]
        if (
            name == self.SENSOR_FRONT
            and not has_hit
            and float(_map_cfg("MAP_TOF_NO_HIT_FREE_MAX_CM", 28.0)) > 0
        ):
            used_cm = min(
                used_cm,
                float(_map_cfg("MAP_TOF_NO_HIT_FREE_MAX_CM", 28.0)),
            )
        ox, oy = self._forward_right_to_world(
            sample.map_x, sample.map_y, sample.theta_deg, p["forward"], p["right"]
        )
        ray_theta = normalize_angle_deg(sample.theta_deg + p["angle"])
        a = math.radians(ray_theta)
        d = used_cm / 100.0
        ex = ox + d * math.sin(a)
        ey = oy + d * math.cos(a)
        return dict(
            sensor=name, origin_x=ox, origin_y=oy, end_x=ex, end_y=ey,
            measured_cm=distance_cm, used_cm=used_cm, has_hit=has_hit,
            hit_score=p["hit_score"], free_score=p["free_score"],
        )

    def _world_to_cell(self, x, y):
        r = max(0.005, float(_map_cfg("MAP_RESOLUTION_M", 0.025)))
        return int(round(x / r)), int(round(y / r))

    def _cell_to_world(self, gx, gy):
        r = max(0.005, float(_map_cfg("MAP_RESOLUTION_M", 0.025)))
        return gx * r, gy * r

    @staticmethod
    def _bresenham(x0, y0, x1, y1):
        cells = []
        dx, sx = abs(x1 - x0), 1 if x0 < x1 else -1
        dy, sy = -abs(y1 - y0), 1 if y0 < y1 else -1
        err = dx + dy
        x, y = x0, y0
        while True:
            cells.append((x, y))
            if x == x1 and y == y1:
                return cells
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x += sx
            if e2 <= dx:
                err += dx
                y += sy

    def _update_cell(self, cell, delta):
        lo = int(_map_cfg("MAP_EVIDENCE_MIN", -30))
        hi = int(_map_cfg("MAP_EVIDENCE_MAX", +30))
        self.grid[cell] = int(clamp(int(self.grid.get(cell, 0)) + int(delta), lo, hi))

    def _mark_robot_footprint_free(self, sample):
        radius = float(_map_cfg("MAP_ROBOT_FREE_RADIUS_M", 0.11))
        score = int(_map_cfg("MAP_ROBOT_FREE_SCORE", -3))
        res = float(_map_cfg("MAP_RESOLUTION_M", 0.025))
        c0 = self._world_to_cell(sample.map_x, sample.map_y)
        n = max(0, int(math.ceil(radius / res)))
        for dx in range(-n, n + 1):
            for dy in range(-n, n + 1):
                if math.hypot(dx * res, dy * res) <= radius:
                    self._update_cell((c0[0] + dx, c0[1] + dy), score)

    def _integrate_ray(self, sample, ray):
        sc = self._world_to_cell(ray["origin_x"], ray["origin_y"])
        ec = self._world_to_cell(ray["end_x"], ray["end_y"])
        cells = self._bresenham(sc[0], sc[1], ec[0], ec[1])
        free_cells = cells[:-1] if ray["has_hit"] else cells
        for cell in free_cells:
            self._update_cell(cell, ray["free_score"])

        sensor = ray["sensor"]
        if ray["has_hit"] and cells:
            self._update_cell(cells[-1], ray["hit_score"])

            if bool(_map_cfg("MAP_CONNECT_CONSECUTIVE_WALL_HITS", True)):
                prev = self._last_wall_hit.get(sensor)
                current = {
                    "x": ray["end_x"],
                    "y": ray["end_y"],
                    "heading_index": sample.heading_index,
                }
                if (
                    prev is not None
                    and prev.get("heading_index") == sample.heading_index
                    and sample.heading_index >= 0
                ):
                    gap = math.hypot(
                        current["x"] - prev["x"],
                        current["y"] - prev["y"],
                    )
                    if gap <= float(_map_cfg("MAP_WALL_CONNECT_MAX_M", 0.18)):
                        pc = self._world_to_cell(prev["x"], prev["y"])
                        cc = self._world_to_cell(current["x"], current["y"])
                        for cell in self._bresenham(pc[0], pc[1], cc[0], cc[1]):
                            self._update_cell(
                                cell,
                                int(_map_cfg("MAP_WALL_CONNECT_SCORE", 4)),
                            )
                self._last_wall_hit[sensor] = current

            self.wall_points.append(dict(
                sample_index=sample.index, time_sec=sample.time_sec,
                sensor=sensor, x=ray["end_x"], y=ray["end_y"],
                distance_cm=ray["measured_cm"],
            ))
        else:
            # An opening/no-hit breaks the wall chain so a doorway cannot be
            # bridged by the next wall sample.
            self._last_wall_hit.pop(sensor, None)
        return ray

    def _ir_is_wall(self, value):
        if value is None:
            return False
        try:
            return int(value) == int(_map_cfg("MAP_IR_WALL_LEVEL", 0))
        except Exception:
            return False

    def _integrate_ir(self, sample, left_ray):
        if not self._ir_is_wall(sample.ir_value):
            return

        # Safest use of a binary sensor: strengthen a geometrically measured
        # left Sharp wall, without changing the measured position.
        if (
            bool(_map_cfg("MAP_IR_CONFIRM_LEFT_SHARP", True))
            and left_ray is not None and left_ray["has_hit"]
            and left_ray["measured_cm"] <= float(_map_cfg("MAP_IR_CONFIRM_MAX_SHARP_CM", 22.0))
        ):
            cell = self._world_to_cell(left_ray["end_x"], left_ray["end_y"])
            self._update_cell(cell, int(_map_cfg("MAP_IR_CONFIRM_SCORE", 4)))
            self.wall_points.append(dict(
                sample_index=sample.index, time_sec=sample.time_sec,
                sensor="ir_confirm_left", x=left_ray["end_x"], y=left_ray["end_y"],
                distance_cm=left_ray["measured_cm"],
            ))
            self.ir_confirm_count += 1
            return

        if not bool(_map_cfg("MAP_IR_FALLBACK_ENABLED", True)):
            return

        # Binary-only fallback: weak hit around an assumed location. Because
        # score=1 and occupied threshold=4, several consistent samples are
        # required before this becomes a visible wall.
        ox, oy = self._forward_right_to_world(
            sample.map_x, sample.map_y, sample.theta_deg,
            float(_map_cfg("MAP_IR_SENSOR_FORWARD_M", 0.08)),
            float(_map_cfg("MAP_IR_SENSOR_RIGHT_M", -0.07)),
        )
        theta = normalize_angle_deg(
            sample.theta_deg + float(_map_cfg("MAP_IR_SENSOR_ANGLE_DEG", -45.0))
        )
        d = float(_map_cfg("MAP_IR_ASSUMED_RANGE_M", 0.12))
        a = math.radians(theta)
        ex, ey = ox + d * math.sin(a), oy + d * math.cos(a)
        c = self._world_to_cell(ex, ey)
        radius = max(0, int(_map_cfg("MAP_IR_FALLBACK_PATCH_RADIUS_CELLS", 1)))
        score = int(_map_cfg("MAP_IR_FALLBACK_HIT_SCORE", 1))
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                if abs(dx) + abs(dy) <= radius:
                    self._update_cell((c[0] + dx, c[1] + dy), score)
        self.wall_points.append(dict(
            sample_index=sample.index, time_sec=sample.time_sec,
            sensor=self.SENSOR_IR, x=ex, y=ey, distance_cm=d * 100.0,
        ))
        self.ir_fallback_count += 1

    def _integrate_sample(self, sample):
        self._mark_robot_footprint_free(sample)
        left_ray = None
        for name, dist in (
            (self.SENSOR_FRONT, sample.front_cm),
            (self.SENSOR_LEFT, sample.left_cm),
            (self.SENSOR_RIGHT, sample.right_cm),
        ):
            ray = self._ray(sample, name, dist)
            if ray is not None:
                self._integrate_ray(sample, ray)
                if name == self.SENSOR_LEFT:
                    left_ray = ray
        self._integrate_ir(sample, left_ray)

    def _rebuild_grid(self):
        self.grid = {}
        self.wall_points = []
        self.ir_confirm_count = 0
        self.ir_fallback_count = 0
        self._last_wall_hit = {}
        for s in self.samples:
            if s.map_ranges:
                self._integrate_sample(s)

    # --------------------------------------------------------
    # Junction loop closure (map only)
    # --------------------------------------------------------
    def observe_junction(self, node_id, is_new, raw_x, raw_y, yaw_deg, heading_index=None):
        if not self.enabled or not self.initialized or node_id is None:
            return None
        sample = self.record_pose(
            raw_x, raw_y, yaw_deg,
            heading_index=heading_index,
            mode="JUNCTION_" + ("NEW" if is_new else "KNOWN"),
            force=True,
        )
        if sample is None:
            return None
        idx, now = sample.index, sample.time_sec

        if node_id not in self.node_anchors:
            self.node_anchors[node_id] = dict(
                x=sample.map_x, y=sample.map_y,
                sample_index=idx, first_seen_time=now,
            )
            self.node_events.append(dict(
                time_sec=now, node_id=node_id,
                event="ANCHOR_NEW" if is_new else "ANCHOR_RECOVERED",
                sample_index=idx, map_x=sample.map_x, map_y=sample.map_y,
            ))
            if self.last_known_junction_sample_index is None or not is_new:
                self.last_known_junction_sample_index = idx
            if bool(_map_cfg("MAP_SAVE_ON_JUNCTION", True)):
                self.save_all(rebuild=False, quiet=True)
            return dict(corrected=False, error_m=0.0)

        anchor = self.node_anchors[node_id]
        ex = anchor["x"] - sample.map_x
        ey = anchor["y"] - sample.map_y
        em = math.hypot(ex, ey)
        min_e = float(_map_cfg("MAP_LOOP_CLOSURE_MIN_ERROR_M", 0.015))
        max_e = float(_map_cfg("MAP_LOOP_CLOSURE_MAX_ERROR_M", 0.35))
        gain = clamp(float(_map_cfg("MAP_LOOP_CLOSURE_GAIN", 1.0)), 0.0, 1.0)
        corrected = False
        reason = "NO_CORRECTION_NEEDED"

        if em >= min_e:
            if em <= max_e:
                start_idx = self.last_known_junction_sample_index
                if start_idx is None:
                    start_idx = 0
                start_idx = max(0, min(int(start_idx), idx))
                ax, ay = ex * gain, ey * gain
                denom = max(1, idx - start_idx)
                for i in range(start_idx, idx + 1):
                    t = float(i - start_idx) / denom
                    self.samples[i].map_x += ax * t
                    self.samples[i].map_y += ay * t
                for nid, a in self.node_anchors.items():
                    if nid == node_id:
                        continue
                    ai = int(a.get("sample_index", -1))
                    if start_idx <= ai <= idx:
                        t = float(ai - start_idx) / denom
                        a["x"] += ax * t
                        a["y"] += ay * t
                self.global_corr_x += ax
                self.global_corr_y += ay
                self.loop_closures.append(dict(
                    time_sec=now, node_id=node_id,
                    sample_index=idx, segment_start_index=start_idx,
                    raw_error_x_m=ex, raw_error_y_m=ey, raw_error_m=em,
                    applied_x_m=ax, applied_y_m=ay,
                ))
                self._rebuild_grid()
                corrected = True
                reason = "LOOP_CLOSURE_APPLIED"
            else:
                reason = "LOOP_CLOSURE_REJECTED_TOO_LARGE"

        self.node_events.append(dict(
            time_sec=now, node_id=node_id, event=reason,
            sample_index=idx, error_x_m=ex, error_y_m=ey, error_m=em,
        ))
        if not is_new or corrected:
            self.last_known_junction_sample_index = idx
        if bool(_map_cfg("MAP_SAVE_ON_JUNCTION", True)):
            self.save_all(rebuild=False, quiet=True)
        return dict(corrected=corrected, reason=reason, error_m=em)

    # --------------------------------------------------------
    # Map states / visual cleanup
    # --------------------------------------------------------
    def cell_state(self, score):
        if score >= int(_map_cfg("MAP_OCCUPIED_SCORE_THRESHOLD", 4)):
            return "OCCUPIED"
        if score <= int(_map_cfg("MAP_FREE_SCORE_THRESHOLD", -3)):
            return "FREE"
        return "UNKNOWN"

    def _display_sets(self):
        occupied = {c for c, s in self.grid.items() if self.cell_state(s) == "OCCUPIED"}
        free = {c for c, s in self.grid.items() if self.cell_state(s) == "FREE"}

        # Remove isolated one-cell hits from display only; raw evidence remains.
        if bool(_map_cfg("MAP_DISPLAY_REMOVE_ISOLATED_WALLS", True)) and occupied:
            keep = set()
            for x, y in occupied:
                neighbours = sum(
                    ((x + dx, y + dy) in occupied)
                    for dx in (-1, 0, 1) for dy in (-1, 0, 1)
                    if not (dx == 0 and dy == 0)
                )
                score = self.grid.get((x, y), 0)
                if neighbours > 0 or score >= int(_map_cfg("MAP_OCCUPIED_SCORE_THRESHOLD", 4)) + 4:
                    keep.add((x, y))
            occupied = keep

        # Bridge tiny 1-2 cell gaps along horizontal/vertical walls.
        gap = max(0, int(_map_cfg("MAP_DISPLAY_BRIDGE_GAP_CELLS", 2)))
        bridged = set(occupied)
        for x, y in list(occupied):
            for d in range(2, gap + 2):
                if (x + d, y) in occupied:
                    for k in range(1, d): bridged.add((x + k, y))
                if (x, y + d) in occupied:
                    for k in range(1, d): bridged.add((x, y + k))
        occupied = bridged

        # Slight wall thickness gives a lidar/SLAM-map appearance.
        radius = max(0, int(_map_cfg("MAP_DISPLAY_WALL_DILATION_CELLS", 1)))
        if radius:
            dilated = set(occupied)
            for x, y in occupied:
                for dx in range(-radius, radius + 1):
                    for dy in range(-radius, radius + 1):
                        if max(abs(dx), abs(dy)) <= radius:
                            dilated.add((x + dx, y + dy))
            occupied = dilated

        free -= occupied
        return occupied, free

    def _bounds(self, occupied=None, free=None):
        cells = set(self.grid)
        if occupied: cells |= set(occupied)
        if free: cells |= set(free)
        points = [(s.map_x, s.map_y) for s in self.samples]
        points += [(a["x"], a["y"]) for a in self.node_anchors.values()]
        points += [self._cell_to_world(*c) for c in cells]
        if not points:
            return -0.5, 0.5, -0.5, 0.5
        xs, ys = [p[0] for p in points], [p[1] for p in points]
        m = float(_map_cfg("MAP_EXPORT_MARGIN_M", 0.30))
        minx, maxx, miny, maxy = min(xs)-m, max(xs)+m, min(ys)-m, max(ys)+m
        if maxx-minx < 0.5:
            c=(minx+maxx)/2; minx,maxx=c-0.25,c+0.25
        if maxy-miny < 0.5:
            c=(miny+maxy)/2; miny,maxy=c-0.25,c+0.25
        return minx,maxx,miny,maxy

    def mark_exit(self, raw_x, raw_y, yaw_deg, heading_index=None, details=None):
        """Record a confirmed maze exit in map coordinates."""
        if not self.enabled or not self.initialized:
            return None
        sample = self.record_pose(
            raw_x, raw_y, yaw_deg,
            heading_index=heading_index, mode="EXIT_FOUND", force=True,
        )
        if sample is None:
            return None
        self.exit_event = {
            "time_sec": float(sample.time_sec),
            "sample_index": int(sample.index),
            "map_x": float(sample.map_x),
            "map_y": float(sample.map_y),
            "theta_deg": float(sample.theta_deg),
            "heading_index": int(sample.heading_index),
            "details": dict(details or {}),
        }
        return self.exit_event

    # --------------------------------------------------------
    # Exports
    # --------------------------------------------------------
    def save_all(self, rebuild=True, quiet=False):
        if not self.enabled or not self.initialized:
            return
        os.makedirs(self.output_dir, exist_ok=True)
        if rebuild:
            self._rebuild_grid()
        self._save_csvs()
        self._save_jsons()
        self._save_svg()
        self._try_save_png()
        if not quiet:
            print(
                f"MAPPER SAVED: {self.output_dir} | samples={len(self.samples)} "
                f"wall_points={len(self.wall_points)} cells={len(self.grid)} "
                f"IRconfirm={self.ir_confirm_count} IRfallback={self.ir_fallback_count}"
            )
            print(f"  -> {os.path.join(self.output_dir, 'maze_map.png')}")
            print(f"  -> {os.path.join(self.output_dir, 'maze_map.svg')}")

    def _save_csvs(self):
        with open(os.path.join(self.output_dir, "trajectory.csv"), "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["index","time_sec","raw_x","raw_y","yaw_deg","map_x","map_y","theta_deg","heading_index","front_cm","left_cm","right_cm","ir","mode"])
            for s in self.samples:
                w.writerow([s.index,f"{s.time_sec:.6f}",f"{s.raw_x:.6f}",f"{s.raw_y:.6f}",f"{s.yaw_deg:.6f}",f"{s.map_x:.6f}",f"{s.map_y:.6f}",f"{s.theta_deg:.4f}",s.heading_index,"" if s.front_cm is None else f"{s.front_cm:.3f}","" if s.left_cm is None else f"{s.left_cm:.3f}","" if s.right_cm is None else f"{s.right_cm:.3f}","" if s.ir_value is None else s.ir_value,s.mode])
        with open(os.path.join(self.output_dir, "wall_points.csv"), "w", newline="", encoding="utf-8") as f:
            w=csv.writer(f); w.writerow(["sample_index","time_sec","sensor","x_m","y_m","distance_cm"])
            for p in self.wall_points:
                w.writerow([p["sample_index"],f'{p["time_sec"]:.6f}',p["sensor"],f'{p["x"]:.6f}',f'{p["y"]:.6f}',f'{p["distance_cm"]:.3f}'])
        with open(os.path.join(self.output_dir, "occupancy_grid.csv"), "w", newline="", encoding="utf-8") as f:
            w=csv.writer(f); w.writerow(["gx","gy","x_m","y_m","score","state"])
            for (gx,gy),score in sorted(self.grid.items()):
                x,y=self._cell_to_world(gx,gy); w.writerow([gx,gy,f"{x:.6f}",f"{y:.6f}",score,self.cell_state(score)])

    def _save_jsons(self):
        occupied, free = self._display_sets()
        cells=[]
        for c,s in sorted(self.grid.items()):
            st=self.cell_state(s)
            if st != "UNKNOWN": cells.append(dict(gx=c[0],gy=c[1],score=s,state=st))
        with open(os.path.join(self.output_dir,"occupancy_grid.json"),"w",encoding="utf-8") as f:
            json.dump(dict(resolution_m=float(_map_cfg("MAP_RESOLUTION_M",0.025)),cells=cells),f,indent=2)
        with open(os.path.join(self.output_dir,"nodes.json"),"w",encoding="utf-8") as f:
            json.dump(dict(anchors=self.node_anchors,events=self.node_events),f,indent=2)
        with open(os.path.join(self.output_dir,"loop_closures.json"),"w",encoding="utf-8") as f:
            json.dump(self.loop_closures,f,indent=2)
        with open(os.path.join(self.output_dir,"exit.json"),"w",encoding="utf-8") as f:
            json.dump(self.exit_event,f,indent=2)
        states={"FREE":0,"OCCUPIED":0,"UNKNOWN":0}
        for s in self.grid.values(): states[self.cell_state(s)] += 1
        with open(os.path.join(self.output_dir,"mapping_summary.json"),"w",encoding="utf-8") as f:
            json.dump(dict(
                coordinate_convention="+Y=NORTH, +X=EAST, 0deg=NORTH +90deg=EAST",
                resolution_m=float(_map_cfg("MAP_RESOLUTION_M",0.025)),
                samples=len(self.samples), wall_points=len(self.wall_points), grid_states=states,
                display_occupied_cells=len(occupied), display_free_cells=len(free),
                nodes=len(self.node_anchors), loop_closures=len(self.loop_closures),
                exit_found=self.exit_event is not None,
                ir_wall_level=int(_map_cfg("MAP_IR_WALL_LEVEL",0)),
                ir_confirm_count=self.ir_confirm_count, ir_fallback_count=self.ir_fallback_count,
                yaw_fallback_count=self.yaw_fallback_count,
                position_rotation_deg=self.position_rotation_deg,
                position_auto_aligned=self.position_auto_aligned,
            ),f,indent=2)

    @staticmethod
    def _xml(text):
        return str(text).replace("&","&amp;").replace("<","&lt;").replace(">","&gt;").replace('"',"&quot;")

    def _save_svg(self):
        occupied, free = self._display_sets()
        minx,maxx,miny,maxy = self._bounds(occupied,free)
        wm,hm=maxx-minx,maxy-miny
        ppm=float(_map_cfg("MAP_SVG_PX_PER_M",420.0))
        W=int(clamp(wm*ppm,600,2200)); H=int(clamp(hm*ppm,600,2200))
        sx=lambda x:(x-minx)/wm*W
        sy=lambda y:H-(y-miny)/hm*H
        res=float(_map_cfg("MAP_RESOLUTION_M",0.025))
        cw=max(1.0,res/wm*W); ch=max(1.0,res/hm*H)
        p=[]
        p.append('<?xml version="1.0" encoding="UTF-8"?>')
        p.append(f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">')
        # ROS/SLAM-like palette: gray unknown, white observed free, dark occupied.
        p.append('<rect width="100%" height="100%" fill="#bfc3c7"/>')
        for gx,gy in free:
            x,y=self._cell_to_world(gx,gy)
            p.append(f'<rect x="{sx(x)-cw/2:.2f}" y="{sy(y)-ch/2:.2f}" width="{cw+0.6:.2f}" height="{ch+0.6:.2f}" fill="#f7f7f7"/>')
        for gx,gy in occupied:
            x,y=self._cell_to_world(gx,gy)
            p.append(f'<rect x="{sx(x)-cw/2:.2f}" y="{sy(y)-ch/2:.2f}" width="{cw+0.7:.2f}" height="{ch+0.7:.2f}" fill="#202428"/>')
        if bool(_map_cfg("MAP_DRAW_TRAJECTORY",True)) and len(self.samples)>1:
            pts=" ".join(f"{sx(s.map_x):.2f},{sy(s.map_y):.2f}" for s in self.samples)
            p.append(f'<polyline points="{pts}" fill="none" stroke="#2463b5" stroke-width="2.4" stroke-linecap="round" stroke-linejoin="round" opacity="0.92"/>')
        if self.samples:
            s=self.samples[0]
            p.append(f'<circle cx="{sx(s.map_x):.2f}" cy="{sy(s.map_y):.2f}" r="6" fill="#19a15f" stroke="white" stroke-width="2"/>')
            e=self.samples[-1]
            p.append(f'<circle cx="{sx(e.map_x):.2f}" cy="{sy(e.map_y):.2f}" r="5" fill="#f59e0b" stroke="white" stroke-width="1.5"/>')
        if bool(_map_cfg("MAP_DRAW_NODES",True)):
            for nid,a in sorted(self.node_anchors.items()):
                x,y=sx(a["x"]),sy(a["y"])
                p.append(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="4" fill="#d14343" stroke="white" stroke-width="1.2"/>')
                p.append(f'<text x="{x+6:.2f}" y="{y-6:.2f}" font-family="Arial,sans-serif" font-size="11" font-weight="700" fill="#111827">{self._xml(nid)}</text>')
        if self.exit_event is not None and bool(_map_cfg("MAP_DRAW_EXIT",True)):
            x,y=sx(self.exit_event["map_x"]),sy(self.exit_event["map_y"])
            p.append(f'<polygon points="{x:.2f},{y-9:.2f} {x+9:.2f},{y:.2f} {x:.2f},{y+9:.2f} {x-9:.2f},{y:.2f}" fill="#7c3aed" stroke="white" stroke-width="2"/>')
            p.append(f'<text x="{x+12:.2f}" y="{y-9:.2f}" font-family="Arial,sans-serif" font-size="12" font-weight="700" fill="#5b21b6">EXIT</text>')
        p.append('<rect x="14" y="14" width="235" height="108" rx="9" fill="white" fill-opacity="0.92" stroke="#9ca3af"/>')
        p.append('<text x="27" y="36" font-family="Arial,sans-serif" font-size="12" font-weight="700" fill="#111827">SLAM-style Maze Map</text>')
        p.append('<text x="27" y="55" font-family="Arial,sans-serif" font-size="11" fill="#374151">Black: wall  White: observed free</text>')
        p.append('<text x="27" y="72" font-family="Arial,sans-serif" font-size="11" fill="#374151">Gray: unknown  Blue: trajectory</text>')
        p.append('<text x="27" y="89" font-family="Arial,sans-serif" font-size="11" fill="#374151">Red: junction  Green: start</text>')
        p.append('<text x="27" y="106" font-family="Arial,sans-serif" font-size="11" fill="#374151">Purple diamond: confirmed exit</text>')
        p.append('</svg>')
        Path = __import__('pathlib').Path
        Path(os.path.join(self.output_dir,"maze_map.svg")).write_text("\n".join(p),encoding="utf-8")

    def _try_save_png(self):
        if not bool(_map_cfg("MAP_EXPORT_PNG",True)):
            return
        try:
            import matplotlib.pyplot as plt
            import numpy as np
            from matplotlib.colors import ListedColormap
        except Exception as exc:
            # SVG is always available; PNG is optional.
            return

        occupied, free = self._display_sets()
        minx,maxx,miny,maxy = self._bounds(occupied,free)
        res=float(_map_cfg("MAP_RESOLUTION_M",0.025))
        gx0=int(math.floor(minx/res)); gx1=int(math.ceil(maxx/res))
        gy0=int(math.floor(miny/res)); gy1=int(math.ceil(maxy/res))
        width=max(1,gx1-gx0+1); height=max(1,gy1-gy0+1)
        # 0 unknown, 1 free, 2 occupied
        img=np.zeros((height,width),dtype=np.uint8)
        for gx,gy in free:
            if gx0<=gx<=gx1 and gy0<=gy<=gy1: img[gy-gy0,gx-gx0]=1
        for gx,gy in occupied:
            if gx0<=gx<=gx1 and gy0<=gy<=gy1: img[gy-gy0,gx-gx0]=2

        cmap=ListedColormap(["#bfc3c7","#f8f8f8","#171a1d"])
        fig,ax=plt.subplots(figsize=(8.5,8.5))
        ax.imshow(img,origin="lower",extent=[gx0*res,(gx1+1)*res,gy0*res,(gy1+1)*res],interpolation="nearest",cmap=cmap,vmin=0,vmax=2)
        if bool(_map_cfg("MAP_DRAW_TRAJECTORY",True)) and self.samples:
            ax.plot([s.map_x for s in self.samples],[s.map_y for s in self.samples],linewidth=1.8,label="trajectory")
            ax.scatter([self.samples[0].map_x],[self.samples[0].map_y],s=45,label="start",zorder=5)
            ax.scatter([self.samples[-1].map_x],[self.samples[-1].map_y],s=35,label="current/end",zorder=5)
        if bool(_map_cfg("MAP_DRAW_NODES",True)):
            for nid,a in self.node_anchors.items():
                ax.scatter([a["x"]],[a["y"]],s=20,zorder=5)
                ax.text(a["x"],a["y"]," "+str(nid),fontsize=7,zorder=6)
        if self.exit_event is not None and bool(_map_cfg("MAP_DRAW_EXIT",True)):
            ax.scatter([self.exit_event["map_x"]],[self.exit_event["map_y"]],s=90,marker="*",label="exit",zorder=7)
            ax.text(self.exit_event["map_x"],self.exit_event["map_y"]," EXIT",fontsize=8,fontweight="bold",zorder=8)
        ax.set_aspect("equal",adjustable="box")
        ax.set_xlabel("East / X (m)")
        ax.set_ylabel("North / Y (m)")
        ax.set_title("RoboMaster Maze Occupancy Map")
        ax.grid(False)
        if self.samples: ax.legend(loc="best",fontsize=8,framealpha=0.9)
        fig.tight_layout()
        fig.savefig(os.path.join(self.output_dir,"maze_map.png"),dpi=int(_map_cfg("MAP_PNG_DPI",220)))
        plt.close(fig)

"""
Lightweight Vector Maze Mapper (Continuous Polyline & Auto-Rotate)
Replaces the old SLAM-style occupancy grid mapper with a simple, memory-efficient
vectorized approach that connects wall hits into continuous lines and auto-rotates
the map to align with the primary initial trajectory.
"""

import os
import csv
import math
import time
from map_engine import map_geometry
from scipy.ndimage import gaussian_filter
import numpy as np

try:
    import config
    def _map_cfg(name, default):
        return getattr(config, name, default)
except ImportError:
    def _map_cfg(name, default):
        return default

def _map_safe_float(val):
    try:
        return float(val)
    except (TypeError, ValueError):
        return None

def normalize_angle_deg(a):
    return (a % 360.0 + 360.0) % 360.0


class SLAMStyleMazeMapper:
    """
    Maintained class name for backward compatibility with main.py.
    This now operates as a pure Vector Logger and Continuous Line Plotter.
    """
    def __init__(self, output_dir=None):
        self.enabled = bool(_map_cfg("ENABLE_MAPPING", True))
        
        # Override output_dir to always use a local 'output' folder inside map_engine
        local_output = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output")
        self.output_dir = output_dir or local_output
        self.initialized = False
        
        self.start_raw_x = None
        self.start_raw_y = None
        self.start_yaw = None
        self.samples = []
        self.live_csv = None

    def initialize(self, raw_x, raw_y, start_yaw, heading_index=0):
        if not self.enabled:
            return
            
        self.start_raw_x = _map_safe_float(raw_x) or 0.0
        self.start_raw_y = _map_safe_float(raw_y) or 0.0
        self.start_yaw = _map_safe_float(start_yaw) or 0.0
        self.initialized = True
        
        os.makedirs(self.output_dir, exist_ok=True)
        
        csv_path = os.path.join(self.output_dir, "live_trajectory.csv")
        self.live_csv = open(csv_path, "w", newline="", encoding="utf-8")
        self.live_writer = csv.writer(self.live_csv)
        self.live_writer.writerow([
            "index", "time_sec", "map_x", "map_y", "theta_deg", 
            "front_cm", "left_cm", "right_cm"
        ])
        self.live_csv.flush()
        
        print(f"Continuous Vector Mapper initialized. Live stream: {csv_path}")

    def observe_junction(self, *args, **kwargs):
        pass

    def observe_exit(self, *args, **kwargs):
        pass

    def record_pose(self, raw_x, raw_y, yaw_deg, heading_index=None, mode="RUN", force=False):
        return self.update(
            raw_x, raw_y, yaw_deg,
            front_cm=None, left_cm=None, right_cm=None, ir_value=None,
            heading_index=heading_index, mode=mode, map_ranges=False, force=force
        )

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
        if raw_x is None or raw_y is None or yaw_deg is None:
            return None

        # Coordinate Transformation
        dx = raw_x - self.start_raw_x
        dy = raw_y - self.start_raw_y
        
        if bool(_map_cfg("MAP_SWAP_RAW_XY", False)):
            dx, dy = dy, dx
            
        map_x = dx * float(_map_cfg("MAP_RAW_X_SIGN", -1.0))
        map_y = dy * float(_map_cfg("MAP_RAW_Y_SIGN", +1.0))
        
        theta_deg = normalize_angle_deg(
            (yaw_deg - self.start_yaw) * float(_map_cfg("MAP_YAW_RIGHT_SIGN", +1.0))
        )

        sample = {
            "index": len(self.samples),
            "time_sec": time.time(),
            "map_x": map_x,
            "map_y": map_y,
            "theta_deg": theta_deg,
            "front_cm": _map_safe_float(front_cm),
            "left_cm": _map_safe_float(left_cm),
            "right_cm": _map_safe_float(right_cm)
        }
        self.samples.append(sample)

        # Stream to CSV for live_viewer.py
        if self.live_csv and not self.live_csv.closed:
            def fmt(v): return f"{v:.4f}" if v is not None else ""
            self.live_writer.writerow([
                sample["index"], f"{sample['time_sec']:.6f}",
                fmt(sample["map_x"]), fmt(sample["map_y"]), fmt(sample["theta_deg"]),
                fmt(sample["front_cm"]), fmt(sample["left_cm"]), fmt(sample["right_cm"])
            ])
            self.live_csv.flush()
            
        return sample

    def save_all(self, rebuild=True, quiet=False):
        if not self.enabled or not self.initialized:
            return
            
        if self.live_csv and not self.live_csv.closed:
            self.live_csv.close()
            
        self._generate_vector_image(quiet)
        
    def _split_into_polylines(self, points, max_gap=0.20):
        """Splits a list of points into separate lines if distance > max_gap (20cm)"""
        if not points: return []
        lines = []
        current_line = [points[0]]
        for i in range(1, len(points)):
            p1 = points[i-1]
            p2 = points[i]
            dist = math.hypot(p2[0]-p1[0], p2[1]-p1[1])
            if dist > max_gap:
                if len(current_line) > 1:
                    lines.append(current_line)
                current_line = [p2]
            else:
                current_line.append(p2)
        if len(current_line) > 1:
            lines.append(current_line)
        return lines

    def _generate_vector_image(self, quiet=False):
        if not self.samples:
            return
            
        try:
            import matplotlib.pyplot as plt
            import matplotlib.patches as mpatches
            from matplotlib.collections import LineCollection, PolyCollection
        except ImportError:
            if not quiet:
                print("matplotlib not installed. Cannot generate vector image.")
            return

        # Use the advanced geometry post-processing pipeline
        result = map_geometry.process_map_data(self.samples)
        if len(result) == 3:
            polygons, wall_lines, trajectory_pts = result
        else:
            return

        # --- PLOTTING ---
        fig, ax = plt.subplots(figsize=(10, 10), facecolor="#E2E8F0")
        ax.set_facecolor("#E2E8F0") # Unknown space (Light Gray)
        
        # 1. Render Corridor Floor (White Polygons)
        if polygons:
            poly_col = PolyCollection(polygons, facecolors="#FFFFFF", edgecolors="none", zorder=1)
            ax.add_collection(poly_col)
            
        # 2. Render Vector Walls (Black Lines)
        if wall_lines:
            line_col = LineCollection(wall_lines, colors="#000000", linewidths=3.0, zorder=2, capstyle='round')
            ax.add_collection(line_col)

        # 3. Draw Trajectory (Red)
        if trajectory_pts:
            tx = [p[0] for p in trajectory_pts]
            ty = [p[1] for p in trajectory_pts]
            ax.plot(tx, ty, color="#e11d48", linewidth=2.0, alpha=0.9, zorder=4, label="Trajectory")
            ax.scatter(tx[0], ty[0], c="#facc15", edgecolors="#000000", s=100, linewidths=1.2, zorder=6, label="Start")
            ax.scatter(tx[-1], ty[-1], c="#dc2626", marker="*", s=150, edgecolors="#7f1d1d", zorder=6, label="End")

        ax.set_aspect("equal")
        ax.grid(False) # Turn off matplotlib grid since we have our own map
        ax.axis("off")

        # Zoom into the bounding box of the trajectory with some padding
        if trajectory_pts:
            all_x = tx.copy()
            all_y = ty.copy()
            for poly in polygons:
                for p in poly:
                    all_x.append(p[0])
                    all_y.append(p[1])
            for line in wall_lines:
                for p in line:
                    all_x.append(p[0])
                    all_y.append(p[1])
                    
            pad = 0.5
            ax.set_xlim([min(all_x) - pad, max(all_x) + pad])
            ax.set_ylim([min(all_y) - pad, max(all_y) + pad])

        ax.set_title("Continuous Vectorized Maze Floorplan", fontsize=14, fontweight="bold", color="#0f172a", pad=15)
        
        legend_items = [
            mpatches.Patch(color="#000000", label="Continuous Walls"),
            mpatches.Patch(color="#e11d48", label="Robot Path"),
            mpatches.Patch(facecolor="#facc15", edgecolor="#000000", label="Start Point"),
            mpatches.Patch(facecolor="#dc2626", edgecolor="#7f1d1d", label="End Point"),
        ]
        ax.legend(handles=legend_items, loc="upper right", framealpha=1.0, fontsize=9)

        plt.tight_layout()
        save_path = os.path.join(self.output_dir, "vector_maze.png")
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        plt.close(fig)
        
        if not quiet:
            print(f"MAPPER SAVED: {self.output_dir} | samples={len(self.samples)}")
            print(f"  -> {save_path}")

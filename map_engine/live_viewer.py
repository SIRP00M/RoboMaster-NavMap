import os
import csv
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection, LineCollection
import matplotlib.patches as mpatches
from matplotlib.animation import FuncAnimation
import sys

from map_engine import map_geometry

# Locate the output directory relative to this script's location (map_engine/output)
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CSV_FILE = os.path.join(BASE_DIR, "output", "live_trajectory.csv")

fig, ax = plt.subplots(figsize=(10, 10), facecolor="#E2E8F0")

def update_plot(frame):
    if not os.path.exists(CSV_FILE):
        return

    samples = []
    with open(CSV_FILE, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            samples.append(row)

    if not samples:
        return

    # Use the advanced geometry post-processing pipeline
    result = map_geometry.process_map_data(samples)
    if len(result) == 3:
        polygons, wall_lines, trajectory_pts = result
    else:
        return

    ax.clear()
    ax.set_facecolor("#E2E8F0") # Unknown space (Light Gray)
    
    # Render Corridor Floor (White Polygons)
    if polygons:
        poly_col = PolyCollection(polygons, facecolors="#FFFFFF", edgecolors="none", zorder=1)
        ax.add_collection(poly_col)
        
    # Render Vector Walls (Black Lines)
    if wall_lines:
        line_col = LineCollection(wall_lines, colors="#000000", linewidths=3.0, zorder=2, capstyle='round')
        ax.add_collection(line_col)

    # Draw Trajectory (Red)
    if trajectory_pts:
        tx = [p[0] for p in trajectory_pts]
        ty = [p[1] for p in trajectory_pts]
        ax.plot(tx, ty, color="#e11d48", linewidth=2.0, alpha=0.9, zorder=4, label="Robot Path")
        ax.scatter(tx[0], ty[0], c="#facc15", edgecolors="#000000", s=100, linewidths=1.2, zorder=6, label="Start")
        ax.scatter(tx[-1], ty[-1], c="#dc2626", marker="*", s=150, edgecolors="#7f1d1d", zorder=6, label="Current")
        
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

    ax.set_aspect("equal")
    ax.grid(False)
    ax.axis("off")
    ax.set_title("Live HD Vectorized SLAM", fontsize=12, fontweight="bold", color="#0f172a", pad=12)
    
    legend_items = [
        mpatches.Patch(color="#000000", label="Continuous Walls"),
        mpatches.Patch(color="#FFFFFF", label="Free Space"),
        mpatches.Patch(color="#e11d48", label="Robot Path"),
    ]
    ax.legend(handles=legend_items, loc="upper right", framealpha=0.95, fontsize=8)
    plt.tight_layout()

ani = FuncAnimation(fig, update_plot, interval=1000, cache_frame_data=False)
print(f"Live viewer started. Watching {CSV_FILE}. Close the window to stop.")
plt.show()

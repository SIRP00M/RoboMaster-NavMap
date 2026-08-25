import os
import csv
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection, LineCollection
import matplotlib.patches as mpatches
import sys

from map_engine import map_geometry

# Locate the output directory relative to this script's location (map_engine/output)
BASE_DIR = os.path.dirname(os.path.abspath(__file__))

def create_mock_data():
    """Generates a mock T-junction if real data is not available."""
    samples = []
    for i in range(30):
        y = i * 0.05
        samples.append({
            "map_x": 0.0, "map_y": y, "theta_deg": 0.0,
            "left_cm": "30.0", "right_cm": "30.0", "front_cm": "" if i < 20 else str((30-i)*5)
        })
    for i in range(10):
        samples.append({
            "map_x": 0.0, "map_y": 1.5, "theta_deg": i * 9.0,
            "left_cm": "", "right_cm": "", "front_cm": ""
        })
    for i in range(20):
        x = i * 0.05
        samples.append({
            "map_x": x, "map_y": 1.5, "theta_deg": 90.0,
            "left_cm": "30.0", "right_cm": "30.0", "front_cm": "" if i < 15 else str((20-i)*5)
        })
    return samples

def main():
    csv_file = os.path.join(BASE_DIR, "output", "sensor_log.csv")
    samples = []
    if os.path.exists(csv_file):
        with open(csv_file, "r", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            samples = list(reader)
            
    if not samples or len(samples) < 5:
        print(f"Using mock data (could not read enough rows from {csv_file})...")
        samples = create_mock_data()
        
    result = map_geometry.process_map_data(samples)
    if len(result) != 3:
        print("Failed to process data")
        return
        
    polygons, wall_lines, trajectory_pts = result
    
    fig, ax = plt.subplots(figsize=(10, 10), facecolor="#E2E8F0")
    ax.set_facecolor("#E2E8F0")
    
    # Render Corridor Floor (White Polygons)
    if polygons:
        poly_col = PolyCollection(polygons, facecolors="#FFFFFF", edgecolors="none", zorder=1)
        ax.add_collection(poly_col)
        
    # Render Vector Walls (Black Lines)
    if wall_lines:
        line_col = LineCollection(wall_lines, colors="#000000", linewidths=3.0, zorder=2, capstyle='round')
        ax.add_collection(line_col)
        
    # Render Trajectory (Red Line)
    if trajectory_pts:
        tx = [p[0] for p in trajectory_pts]
        ty = [p[1] for p in trajectory_pts]
        ax.plot(tx, ty, color="#e11d48", linewidth=2.0, alpha=0.9, zorder=3, label="Robot Path")
        ax.scatter(tx[0], ty[0], c="#facc15", edgecolors="#000000", s=100, linewidths=1.2, zorder=4, label="Start Point")
        ax.scatter(tx[-1], ty[-1], c="#dc2626", marker="*", s=150, edgecolors="#7f1d1d", zorder=4, label="End Point")
        
        # Determine bounds
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
    ax.set_title("Continuous Vectorized Maze Floorplan", fontsize=14, fontweight="bold", color="#1e293b", pad=15)
    
    legend_items = [
        mpatches.Patch(color="#000000", label="Continuous Walls"),
        mpatches.Patch(color="#e11d48", label="Robot Path"),
        mpatches.Patch(color="#facc15", label="Start Point"),
        mpatches.Patch(color="#dc2626", label="End Point"),
    ]
    ax.legend(handles=legend_items, loc="upper right", framealpha=1.0, fontsize=9)

    plt.tight_layout()
    output_path = os.path.join(BASE_DIR, "output", "preview.png")
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"Saved to {output_path}")

if __name__ == "__main__":
    main()

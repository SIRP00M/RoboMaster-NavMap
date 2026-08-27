import json
import matplotlib.pyplot as plt
import argparse
import os

# ==============================================================
# CONFIGURATION
# ==============================================================
# แก้ไข Path ของไฟล์ JSON ที่นี่ได้เลย หากรันจาก IDE โดยตรง
DEFAULT_JSON_FILE = "working.json"
# ==============================================================

def plot_maze(json_file):
    # พยายามหาไฟล์จากโฟลเดอร์เดียวกับโค้ด หากรันจากโฟลเดอร์อื่น (เช่น รันจาก root ของโปรเจกต์)
    if not os.path.isabs(json_file) and not os.path.exists(json_file):
        script_dir = os.path.dirname(os.path.abspath(__file__))
        json_file = os.path.join(script_dir, json_file)

    if not os.path.exists(json_file):
        print(f"Error: Could not find file {json_file}")
        return

    with open(json_file, 'r', encoding='utf-8') as f:
        data = json.load(f)
        
    map_data = data.get('map', {})
    rows = map_data.get('rows', 0)
    cols = map_data.get('cols', 0)
    
    fig, ax = plt.subplots(figsize=(8, 8))
    
    # Draw background grid
    for r in range(rows + 1):
        ax.axhline(r, color='lightgray', linestyle='-', linewidth=1)
    for c in range(cols + 1):
        ax.axvline(c, color='lightgray', linestyle='-', linewidth=1)
        
    # Helper to draw walls
    # heading_index: 0=N(up), 1=E(right), 2=S(down), 3=W(left)
    # We map row r so that row 0 is at the top of the plot
    def draw_wall(r, c, h, color, linewidth, label=None):
        y = rows - r - 1
        x = c
        if h == 0: # N (top)
            line, = ax.plot([x, x+1], [y+1, y+1], color=color, linewidth=linewidth, label=label)
        elif h == 1: # E (right)
            line, = ax.plot([x+1, x+1], [y, y+1], color=color, linewidth=linewidth, label=label)
        elif h == 2: # S (bottom)
            line, = ax.plot([x, x+1], [y, y], color=color, linewidth=linewidth, label=label)
        elif h == 3: # W (left)
            line, = ax.plot([x, x], [y, y+1], color=color, linewidth=linewidth, label=label)
        return line

    # Draw manual walls (from GUI)
    manual_walls_drawn = False
    for w in map_data.get('manual_walls', []):
        label = "Manual Wall" if not manual_walls_drawn else None
        draw_wall(w[0], w[1], w[2], 'black', 3, label=label)
        manual_walls_drawn = True
        
    # Draw sensor walls (discovered by robot)
    sensor_walls_drawn = False
    for w in map_data.get('sensor_walls', []):
        label = "Sensor Wall" if not sensor_walls_drawn else None
        draw_wall(w[0], w[1], w[2], 'red', 3, label=label)
        sensor_walls_drawn = True
        
    # Draw travel path
    path = map_data.get('travel_path', [])
    if path:
        px = [p[1] + 0.5 for p in path]
        py = [rows - p[0] - 1 + 0.5 for p in path]
        ax.plot(px, py, color='blue', linewidth=2.5, marker='o', markersize=5, label='Travel Path', alpha=0.7)
        
    # Draw special points
    def draw_point(point, color, label, marker='*'):
        if point:
            r, c = point
            ax.plot(c + 0.5, rows - r - 1 + 0.5, marker=marker, color=color, markersize=15, label=label, linestyle='None')
            
    draw_point(map_data.get('start'), 'green', 'Start Point', marker='o')
    draw_point(map_data.get('drop'), 'purple', 'Drop Point', marker='s')
    draw_point(map_data.get('exit'), 'orange', 'Exit Point', marker='X')
    
    # Draw robot current position if available
    robot_cell = map_data.get('robot_cell')
    if robot_cell:
        draw_point(robot_cell, 'cyan', 'Robot', marker='^')

    # Formatting
    ax.set_xlim(0, cols)
    ax.set_ylim(0, rows)
    ax.set_aspect('equal')
    ax.set_title(f"Maze Map ({cols}x{rows})")
    
    # Remove axis ticks and labels for cleaner map look
    ax.set_xticks([])
    ax.set_yticks([])
    
    # Add legend outside the plot
    handles, labels = ax.get_legend_handles_labels()
    by_label = dict(zip(labels, handles)) # Remove duplicates
    ax.legend(by_label.values(), by_label.keys(), loc='center left', bbox_to_anchor=(1, 0.5))
    
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Plot RoboMaster Grid Maze from JSON")
    # แบบ Hybrid: หากรันผ่าน Terminal และใส่ชื่อไฟล์มาด้วย จะใช้ชื่อไฟล์นั้น
    # แต่ถ้าไม่ได้ใส่ชื่อไฟล์มา หรือกดรันผ่านโปรแกรม IDE (เช่น VS Code) จะใช้ค่าจาก DEFAULT_JSON_FILE ด้านบนแทน
    parser.add_argument("json_file", nargs='?', default=DEFAULT_JSON_FILE, help=f"Path to JSON file (default: {DEFAULT_JSON_FILE})")
    args = parser.parse_args()
    
    print(f"Plotting {args.json_file}...")
    plot_maze(args.json_file)

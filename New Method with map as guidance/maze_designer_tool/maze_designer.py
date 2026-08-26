"""Pre-drawn topology-guide designer for RoboMaster.

Features
--------
- Draw maze walls on a grid with the mouse.
- Place START, PICKUP, DROP and EXIT hints.
- Draw in any orientation; the robot tests all 4 rotations and both mirrors.
- Preview the mission route START -> PICKUP -> DROP -> EXIT with A*.
- Save/load the maze as JSON.
- Export a topology-first guide. Grid-cell count and metres are hints only;
  live sensors remain authoritative.

Run:
    python maze_designer.py

No RoboMaster SDK is required. tkinter is part of normal Python installs.
"""

from __future__ import annotations

import argparse
import heapq
import json
import math
import os
import sys
import copy
import tkinter as tk
import customtkinter as ctk
from dataclasses import dataclass
from tkinter import filedialog, messagebox
from typing import Dict, Iterable, List, Optional, Sequence, Set, Tuple


Cell = Tuple[int, int]  # row, col
Edge = Tuple[Cell, Cell]

HEADINGS = ("N", "E", "S", "W")
HEADING_VEC: Dict[str, Tuple[int, int]] = {
    "N": (-1, 0),
    "E": (0, +1),
    "S": (+1, 0),
    "W": (0, -1),
}
VEC_HEADING = {v: k for k, v in HEADING_VEC.items()}

RELATIVE_BY_DIFF = {
    0: "FRONT",
    1: "RIGHT",
    2: "BACK",
    3: "LEFT",
}


@dataclass
class MazeData:
    rows: int = 8
    cols: int = 10
    cell_size_m: float = 0.40
    walls: Set[Edge] = None
    start: Optional[Cell] = None
    start_heading: str = "N"
    object_cell: Optional[Cell] = None
    drop_cell: Optional[Cell] = None
    goal: Optional[Cell] = None

    def __post_init__(self):
        if self.walls is None:
            self.walls = set()
        self.start_heading = str(self.start_heading).upper()
        if self.start_heading not in HEADINGS:
            self.start_heading = "N"

    @staticmethod
    def canonical_edge(a: Cell, b: Cell) -> Edge:
        return (a, b) if a <= b else (b, a)

    def in_bounds(self, cell: Cell) -> bool:
        r, c = cell
        return 0 <= r < self.rows and 0 <= c < self.cols

    def has_wall_between(self, a: Cell, b: Cell) -> bool:
        if not self.in_bounds(a) or not self.in_bounds(b):
            return True
        return self.canonical_edge(a, b) in self.walls

    def set_wall_between(self, a: Cell, b: Cell, present: bool) -> None:
        if not (self.in_bounds(a) and self.in_bounds(b)):
            return
        if abs(a[0] - b[0]) + abs(a[1] - b[1]) != 1:
            return
        edge = self.canonical_edge(a, b)
        if present:
            self.walls.add(edge)
        else:
            self.walls.discard(edge)

    def neighbors(self, cell: Cell) -> Iterable[Cell]:
        r, c = cell
        for dr, dc in HEADING_VEC.values():
            nxt = (r + dr, c + dc)
            if self.in_bounds(nxt) and not self.has_wall_between(cell, nxt):
                yield nxt

    def to_dict(self) -> dict:
        walls = []
        for a, b in sorted(self.walls):
            walls.append([[a[0], a[1]], [b[0], b[1]]])
        return {
            "format": "robomaster_predrawn_maze_v2",
            "rows": self.rows,
            "cols": self.cols,
            "cell_size_m": self.cell_size_m,
            "walls": walls,
            "start": list(self.start) if self.start is not None else None,
            # Kept for backward compatibility. The topology guide deliberately
            # does not trust this heading.
            "start_heading": self.start_heading,
            "pickup": list(self.object_cell) if self.object_cell is not None else None,
            "object": list(self.object_cell) if self.object_cell is not None else None,
            "drop": list(self.drop_cell) if self.drop_cell is not None else None,
            "exit": list(self.goal) if self.goal is not None else None,
            "goal": list(self.goal) if self.goal is not None else None,
        }

    @classmethod
    def from_dict(cls, data: dict) -> "MazeData":
        maze = cls(
            rows=int(data.get("rows", 8)),
            cols=int(data.get("cols", 10)),
            cell_size_m=float(data.get("cell_size_m", 0.40)),
            start=tuple(data["start"]) if data.get("start") is not None else None,
            start_heading=str(data.get("start_heading", "N")),
            object_cell=(
                tuple(data["pickup"])
                if data.get("pickup") is not None
                else (tuple(data["object"]) if data.get("object") is not None else None)
            ),
            drop_cell=tuple(data["drop"]) if data.get("drop") is not None else None,
            goal=(
                tuple(data["exit"])
                if data.get("exit") is not None
                else (tuple(data["goal"]) if data.get("goal") is not None else None)
            ),
        )
        for item in data.get("walls", []):
            if len(item) != 2:
                continue
            a = tuple(item[0])
            b = tuple(item[1])
            if maze.in_bounds(a) and maze.in_bounds(b):
                maze.set_wall_between(a, b, True)
        return maze


# -----------------------------------------------------------------------------
# A* and route compilation
# -----------------------------------------------------------------------------

def manhattan(a: Cell, b: Cell) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def astar(maze: MazeData, start: Cell, goal: Cell) -> Optional[List[Cell]]:
    if start == goal:
        return [start]

    counter = 0
    pq = [(manhattan(start, goal), 0, counter, start)]
    came_from: Dict[Cell, Cell] = {}
    g_score: Dict[Cell, int] = {start: 0}
    closed: Set[Cell] = set()

    while pq:
        _, g, _, current = heapq.heappop(pq)
        if current in closed:
            continue
        closed.add(current)

        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path

        for nxt in maze.neighbors(current):
            tentative = g + 1
            if tentative >= g_score.get(nxt, 10**12):
                continue
            g_score[nxt] = tentative
            came_from[nxt] = current
            counter += 1
            f = tentative + manhattan(nxt, goal)
            heapq.heappush(pq, (f, tentative, counter, nxt))

    return None


def merge_paths(a: Sequence[Cell], b: Sequence[Cell]) -> List[Cell]:
    if not a:
        return list(b)
    if not b:
        return list(a)
    if a[-1] == b[0]:
        return list(a) + list(b[1:])
    return list(a) + list(b)


def direction_between(a: Cell, b: Cell) -> str:
    vec = (b[0] - a[0], b[1] - a[1])
    if vec not in VEC_HEADING:
        raise ValueError(f"Cells are not adjacent: {a} -> {b}")
    return VEC_HEADING[vec]


def relative_turn(current_heading: str, target_heading: str) -> str:
    a = HEADINGS.index(current_heading)
    b = HEADINGS.index(target_heading)
    return RELATIVE_BY_DIFF[(b - a) % 4]


def route_segments(path: Sequence[Cell], start_heading: str) -> List[dict]:
    """Compress cell path into straight runs.

    Each segment tells the robot which relative turn to make before travelling a
    known number of grid cells. This is useful for the later RoboMaster adapter.
    """
    if len(path) < 2:
        return []

    steps: List[str] = [direction_between(a, b) for a, b in zip(path, path[1:])]
    segments = []
    current_heading = start_heading
    i = 0
    while i < len(steps):
        heading = steps[i]
        count = 1
        while i + count < len(steps) and steps[i + count] == heading:
            count += 1

        segments.append(
            {
                "from_cell": list(path[i]),
                "to_cell": list(path[i + count]),
                "heading": heading,
                "relative_action": relative_turn(current_heading, heading),
                "cells": count,
            }
        )
        current_heading = heading
        i += count
    return segments


def decision_actions(maze: MazeData, path: Sequence[Cell], start_heading: str) -> List[dict]:
    """Compile actions at physical decision points along a path.

    A decision cell is included when it is a corner, branch/junction, dead-end,
    object/goal marker, or the final route cell. Straight 2-neighbour corridor
    cells are omitted because V12.5 does not need to stop there.
    """
    if len(path) < 2:
        return []

    step_headings = [direction_between(a, b) for a, b in zip(path, path[1:])]
    actions: List[dict] = []
    current_heading = start_heading

    # First motion is handled specially by the V12.5 startup path.
    first_heading = step_headings[0]
    actions.append(
        {
            "kind": "START_DEPARTURE",
            "cell": list(path[0]),
            "map_heading_before": current_heading,
            "next_heading": first_heading,
            "relative_action": relative_turn(current_heading, first_heading),
            "next_cell": list(path[1]),
        }
    )
    current_heading = first_heading

    for idx in range(1, len(path)):
        cell = path[idx]
        degree = sum(1 for _ in maze.neighbors(cell))
        marker = None
        if maze.object_cell == cell:
            marker = "PICKUP"
        if maze.drop_cell == cell:
            marker = "DROP" if marker is None else marker + "+DROP"
        if maze.goal == cell:
            marker = "EXIT" if marker is None else marker + "+EXIT"

        if idx == len(path) - 1:
            actions.append(
                {
                    "kind": "ROUTE_END",
                    "cell": list(cell),
                    "marker": marker,
                    "arrive_heading": current_heading,
                }
            )
            break

        next_heading = step_headings[idx]
        is_corner = next_heading != current_heading
        is_junction = degree != 2
        is_marker = marker is not None
        if not (is_corner or is_junction or is_marker):
            continue

        actions.append(
            {
                "kind": "DECISION",
                "cell": list(cell),
                "degree": degree,
                "marker": marker,
                "map_heading_before": current_heading,
                "next_heading": next_heading,
                "relative_action": relative_turn(current_heading, next_heading),
                "next_cell": list(path[idx + 1]),
            }
        )
        current_heading = next_heading

    return actions


def open_headings(maze: MazeData, cell: Cell) -> List[str]:
    result = []
    r, c = cell
    for heading in HEADINGS:
        dr, dc = HEADING_VEC[heading]
        nxt = (r + dr, c + dc)
        if maze.in_bounds(nxt) and not maze.has_wall_between(cell, nxt):
            result.append(heading)
    return result


def cell_markers(maze: MazeData, cell: Cell) -> List[str]:
    markers = []
    if maze.start == cell:
        markers.append("START")
    if maze.object_cell == cell:
        markers.append("PICKUP")
    if maze.drop_cell == cell:
        markers.append("DROP")
    if maze.goal == cell:
        markers.append("EXIT")
    return markers


def is_topology_node(maze: MazeData, cell: Cell) -> bool:
    """Keep only places the robot can recognise without trusting metres.

    Junctions, dead-ends, corners and mission markers become graph nodes.
    Anonymous straight corridor cells are compressed into one graph edge.
    """
    if cell_markers(maze, cell):
        return True
    headings = open_headings(maze, cell)
    if len(headings) != 2:
        return True
    a = HEADINGS.index(headings[0])
    b = HEADINGS.index(headings[1])
    return (a - b) % 4 != 2


def reachable_cells(maze: MazeData, start: Cell) -> Set[Cell]:
    seen = {start}
    queue = [start]
    while queue:
        cell = queue.pop(0)
        for nxt in maze.neighbors(cell):
            if nxt not in seen:
                seen.add(nxt)
                queue.append(nxt)
    return seen


def build_topology_guide(maze: MazeData) -> dict:
    """Compile the drawing into an orientation-free, distance-soft graph."""
    if maze.start is None:
        raise ValueError("START is not set")

    reachable = reachable_cells(maze, maze.start)
    node_cells = sorted(cell for cell in reachable if is_topology_node(maze, cell))
    if maze.start not in node_cells:
        node_cells.insert(0, maze.start)

    # START is M0 to make field logs easier to read; all other IDs are stable.
    ordered = [maze.start] + [cell for cell in node_cells if cell != maze.start]
    cell_to_id = {cell: f"M{index}" for index, cell in enumerate(ordered)}
    nodes = {}

    for cell in ordered:
        node_id = cell_to_id[cell]
        exits = {}
        r, c = cell
        for heading in open_headings(maze, cell):
            dr, dc = HEADING_VEC[heading]
            previous = cell
            current = (r + dr, c + dc)
            steps = 1
            visited_walk = {cell}

            while current not in cell_to_id:
                if current in visited_walk:
                    current = None
                    break
                visited_walk.add(current)
                onward = [nxt for nxt in maze.neighbors(current) if nxt != previous]
                if len(onward) != 1:
                    current = None
                    break
                previous, current = current, onward[0]
                steps += 1

            if current is None:
                continue
            exits[heading] = {
                "target": cell_to_id[current],
                "grid_steps_hint": steps,
            }

        nodes[node_id] = {
            "id": node_id,
            "cell": list(cell),
            "markers": cell_markers(maze, cell),
            "degree": len(exits),
            "exits": exits,
        }

    mission_order = [
        name
        for name, cell in (
            ("PICKUP", maze.object_cell),
            ("DROP", maze.drop_cell),
            ("EXIT", maze.goal),
        )
        if cell is not None
    ]
    marker_nodes = {}
    for node_id, node in nodes.items():
        for marker in node["markers"]:
            marker_nodes.setdefault(marker, []).append(node_id)

    return {
        "format": "robomaster_topology_guide_v2",
        "orientation_policy": "TRY_4_ROTATIONS_AND_BOTH_MIRRORS",
        "distance_policy": "TOPOLOGY_PRIMARY_GRID_STEPS_HINT_ONLY",
        "sensor_policy": "LIVE_SENSORS_OVERRIDE_DRAWING",
        "start_heading_required": False,
        "start_node_id": cell_to_id[maze.start],
        "mission_order": mission_order,
        "marker_nodes": marker_nodes,
        "nodes": nodes,
    }


def validate_and_plan(maze: MazeData) -> dict:
    errors = []
    if maze.start is None:
        errors.append("START is not set")
    if maze.object_cell is None:
        errors.append("PICKUP is not set")
    if maze.goal is None:
        errors.append("EXIT is not set")
    if errors:
        raise ValueError("; ".join(errors))

    mission_points = [("START", maze.start)]
    mission_points.append(("PICKUP", maze.object_cell))
    if maze.drop_cell is not None:
        mission_points.append(("DROP", maze.drop_cell))
    mission_points.append(("EXIT", maze.goal))

    path: List[Cell] = []
    leg_summaries = []
    for (from_name, start), (to_name, target) in zip(mission_points, mission_points[1:]):
        leg = astar(maze, start, target)
        if leg is None:
            raise ValueError(f"No path from {from_name} to {to_name}")
        path = merge_paths(path, leg)
        leg_summaries.append(
            {
                "from": from_name,
                "to": to_name,
                "cells_hint": len(leg) - 1,
            }
        )

    inferred_heading = direction_between(path[0], path[1]) if len(path) >= 2 else "N"
    segments = route_segments(path, inferred_heading)
    actions = decision_actions(maze, path, inferred_heading)
    topology_guide = build_topology_guide(maze)

    warnings = []
    if maze.drop_cell is None:
        warnings.append(
            "DROP is not set; mission guide will use START -> PICKUP -> EXIT."
        )
    for marker_name, cell in mission_points[1:]:
        headings = open_headings(maze, cell)
        if len(headings) == 2:
            a = HEADINGS.index(headings[0])
            b = HEADINGS.index(headings[1])
            if (a - b) % 4 == 2:
                warnings.append(
                    f"{marker_name} is in an anonymous straight corridor. "
                    "Place it at a corner, junction or dead-end if the robot must recognise it reliably."
                )

    return {
        "format": "robomaster_known_route_v2",
        "maze": maze.to_dict(),
        "topology_guide": topology_guide,
        "path": [list(c) for c in path],
        "mission_legs": leg_summaries,
        "total_cells": len(path) - 1,
        # Display-only estimate. The runtime guide never treats this as a hard
        # drive distance.
        "total_distance_m": (len(path) - 1) * maze.cell_size_m,
        "segments": segments,
        "decision_actions": actions,
        "warnings": warnings,
    }


def load_maze(path: str) -> MazeData:
    with open(path, "r", encoding="utf-8") as f:
        return MazeData.from_dict(json.load(f))


def save_maze(path: str, maze: MazeData) -> None:
    with open(path, "w", encoding="utf-8") as f:
        json.dump(maze.to_dict(), f, indent=2, ensure_ascii=False)


def save_plan(path: str, plan: dict) -> None:
    with open(path, "w", encoding="utf-8") as f:
        json.dump(plan, f, indent=2, ensure_ascii=False)


# -----------------------------------------------------------------------------
# GUI
# -----------------------------------------------------------------------------


try:
    from PIL import Image, ImageTk, ImageGrab
    HAS_PIL = True
except ImportError:
    HAS_PIL = False
class MazeDesignerApp:
    CELL_PX = 62
    PAD = 28
    EDGE_PICK_PX = 12

    def __init__(self, root: ctk.CTk, rows: int = 8, cols: int = 10, cell_size_m: float = 0.40):
        self.root = root
        self.root.title("RoboMaster Pre-drawn Maze Designer (Y2K Edition + Pro)")
        
        # Y2K Theme Colors
        self.color_bg = "#d6e5ff"       
        self.color_sidebar = "#7ea8f8"  
        self.color_text = "#1f2a63"     
        self.color_btn_green = "#b1f071" 
        self.color_btn_yellow = "#fff785" 
        self.color_btn_hover = "#95d654"
        self.color_yellow_hover = "#e6de65"
        self.color_border = "#1f2a63"
        
        ctk.set_appearance_mode("Light")
        self.root.configure(fg_color=self.color_bg)
        
        self.font_main = ctk.CTkFont(family="Courier New", size=14, weight="bold")
        self.font_title = ctk.CTkFont(family="Courier New", size=22, weight="bold")
        
        self.maze = MazeData(rows=rows, cols=cols, cell_size_m=cell_size_m)
        self.mode = tk.StringVar(value="WALL")
        self.status = tk.StringVar(value="Ready! Use W/E/S/P/D/X keys to switch modes.")
        self.route: List[Cell] = []
        self.route_plan: Optional[dict] = None
        self.current_file: Optional[str] = None
        self.appearance_mode = "Light"
        
        # --- NEW: History System ---
        self.history_undo = []
        self.history_redo = []
        
        # --- NEW: Animation System ---
        self.anim_id = None
        self.anim_index = 0
        self.robot_image = None
        self.robot_frames = []
        self.current_frame_idx = 0
        self.gif_anim_id = None
        self.load_icon()

        self._build_ui()
        self._resize_window()
        self.redraw()
        
        # --- NEW: Key Bindings ---
        self.root.bind("<Control-z>", self.undo)
        self.root.bind("<Control-y>", self.redo)
        self.root.bind("<Key>", self.on_key_press)
        self.root.bind("<Button-1>", lambda e: self.root.focus_set())

    def load_icon(self):
        if HAS_PIL:
            assets_dir = os.path.join(os.path.dirname(__file__), "assets")
            if not os.path.exists(assets_dir): return
            for f in os.listdir(assets_dir):
                if f.lower().endswith((".png", ".jpg", ".jpeg", ".gif")):
                    path = os.path.join(assets_dir, f)
                    try:
                        img = Image.open(path)
                        is_gif = f.lower().endswith(".gif")
                        size = int(self.CELL_PX * 3.2)
                        
                        try:
                            while True:
                                frame = img.convert("RGBA")
                                data = frame.getdata()
                                new_data = []
                                for item in data:
                                    if item[0] > 235 and item[1] > 235 and item[2] > 235:
                                        new_data.append((255, 255, 255, 0))
                                    elif item[0] < 150 and item[1] > 200 and item[2] < 150:
                                        new_data.append((0, 255, 0, 0))
                                    else:
                                        new_data.append(item)
                                frame.putdata(new_data)
                                frame = frame.resize((size, size), Image.Resampling.LANCZOS)
                                self.robot_frames.append(ImageTk.PhotoImage(frame))
                                
                                if not is_gif: break
                                img.seek(img.tell() + 1)
                        except EOFError:
                            pass
                            
                        if self.robot_frames:
                            self.robot_image = self.robot_frames[0]
                        break
                    except Exception as e:
                        print(f"Error loading icon: {e}")

    def update_gif_frame(self):
        if not hasattr(self, 'robot_frames') or len(self.robot_frames) <= 1:
            return
        # Stop animating if route finished
        if self.anim_index >= len(self.route):
            return
            
        self.current_frame_idx = (self.current_frame_idx + 1) % len(self.robot_frames)
        self.robot_image = self.robot_frames[self.current_frame_idx]
        
        if self.canvas.find_withtag("robot"):
            self.canvas.itemconfig("robot", image=self.robot_image)
            
        self.gif_anim_id = self.root.after(80, self.update_gif_frame)

    def save_state(self):
        self.history_undo.append(copy.deepcopy(self.maze))
        self.history_redo.clear()

    def undo(self, event=None):
        if self.history_undo:
            self.history_redo.append(copy.deepcopy(self.maze))
            self.maze = self.history_undo.pop()
            self.clear_route()
            self.status.set("Undo successful.")
            self.redraw()

    def redo(self, event=None):
        if self.history_redo:
            self.history_undo.append(copy.deepcopy(self.maze))
            self.maze = self.history_redo.pop()
            self.clear_route()
            self.status.set("Redo successful.")
            self.redraw()

    def on_key_press(self, event):
        if isinstance(event.widget, (tk.Entry, ctk.CTkEntry)):
            return
        if not event.char: return
        char = event.char.lower()
        if char == 'w': self.mode.set("WALL")
        elif char == 'e': self.mode.set("ERASE")
        elif char == 's': self.mode.set("START")
        elif char == 'p': self.mode.set("PICKUP")
        elif char == 'd': self.mode.set("DROP")
        elif char == 'x': self.mode.set("EXIT")

    def _build_ui(self):
        self.root.grid_rowconfigure(0, weight=1)
        self.root.grid_columnconfigure(1, weight=1)

        # Sidebar
        self.sidebar_frame = ctk.CTkFrame(
            self.root, width=240, corner_radius=10, 
            fg_color=self.color_sidebar, border_width=3, border_color=self.color_border
        )
        self.sidebar_frame.grid(row=0, column=0, sticky="nsew", padx=10, pady=10)
        self.sidebar_frame.grid_rowconfigure(11, weight=1)

        logo_label = ctk.CTkLabel(
            self.sidebar_frame, text="E kae\nMaze Designer", 
            font=self.font_title, text_color="white"
        )
        logo_label.grid(row=0, column=0, padx=20, pady=(20, 10))

        # Mode Selection
        self.mode_label = ctk.CTkLabel(
            self.sidebar_frame, text="* DRAWING MODE *", anchor="w", 
            font=self.font_main, text_color="white"
        )
        self.mode_label.grid(row=1, column=0, padx=20, pady=(10, 5), sticky="w")

        modes = [
            ("Wall (W)", "WALL"),
            ("Erase (E)", "ERASE"),
            ("Start (S)", "START"),
            ("Pickup (P)", "PICKUP"),
            ("Drop (D)", "DROP"),
            ("Exit (X)", "EXIT"),
        ]
        
        self.radio_buttons = []
        for i, (text, value) in enumerate(modes):
            rb = ctk.CTkRadioButton(
                self.sidebar_frame, text=text.upper(), variable=self.mode, value=value,
                font=self.font_main, text_color="white",
                fg_color=self.color_btn_yellow, border_color="white", hover_color=self.color_btn_green
            )
            rb.grid(row=2+i, column=0, pady=6, padx=30, sticky="w")
            self.radio_buttons.append(rb)

        # Actions
        actions_label = ctk.CTkLabel(
            self.sidebar_frame, text="* ACTIONS *", anchor="w", 
            font=self.font_main, text_color="white"
        )
        actions_label.grid(row=8, column=0, padx=20, pady=(15, 0), sticky="w")

        self.btn_preview = ctk.CTkButton(
            self.sidebar_frame, text="PREVIEW ANIMATION", command=self.preview_route,
            font=self.font_main, fg_color=self.color_btn_green, text_color=self.color_text,
            border_width=2, border_color=self.color_border, hover_color=self.color_btn_hover, corner_radius=15
        )
        self.btn_preview.grid(row=9, column=0, padx=20, pady=5)
        
        self.btn_clear = ctk.CTkButton(
            self.sidebar_frame, text="CLEAR ANIMATION", command=self.clear_route, 
            font=self.font_main, fg_color=self.color_btn_yellow, text_color=self.color_text,
            border_width=2, border_color=self.color_border, hover_color=self.color_yellow_hover, corner_radius=15
        )
        self.btn_clear.grid(row=10, column=0, padx=20, pady=5)
        
        # Undo/Redo
        ur_frame = ctk.CTkFrame(self.sidebar_frame, fg_color="transparent")
        ur_frame.grid(row=11, column=0, pady=5)
        ctk.CTkButton(ur_frame, text="UNDO", width=70, font=self.font_main, command=self.undo, 
                     border_width=2, border_color=self.color_border, fg_color="#fff", text_color="#000").pack(side="left", padx=5)
        ctk.CTkButton(ur_frame, text="REDO", width=70, font=self.font_main, command=self.redo,
                     border_width=2, border_color=self.color_border, fg_color="#fff", text_color="#000").pack(side="left", padx=5)

        self.sidebar_frame.grid_rowconfigure(12, weight=1)

        # File Operations
        self.btn_new = ctk.CTkButton(
            self.sidebar_frame, text="NEW MAZE", command=self.new_maze,
            font=self.font_main, fg_color="white", text_color=self.color_text,
            border_width=2, border_color=self.color_border, hover_color="#eeeeee", corner_radius=8
        )
        self.btn_new.grid(row=13, column=0, padx=20, pady=5)
        
        self.btn_save = ctk.CTkButton(
            self.sidebar_frame, text="SAVE MAZE", command=self.save_maze_dialog,
            font=self.font_main, fg_color="white", text_color=self.color_text,
            border_width=2, border_color=self.color_border, hover_color="#eeeeee", corner_radius=8
        )
        self.btn_save.grid(row=14, column=0, padx=20, pady=5)
        
        self.btn_load = ctk.CTkButton(
            self.sidebar_frame, text="LOAD MAZE", command=self.load_maze_dialog,
            font=self.font_main, fg_color="white", text_color=self.color_text,
            border_width=2, border_color=self.color_border, hover_color="#eeeeee", corner_radius=8
        )
        self.btn_load.grid(row=15, column=0, padx=20, pady=5)
        
        self.btn_export = ctk.CTkButton(
            self.sidebar_frame, text="EXPORT ROUTE", command=self.export_route_dialog, 
            font=self.font_main, fg_color="#ff9eb5", text_color="white",
            border_width=2, border_color=self.color_border, hover_color="#e07790", corner_radius=8
        )
        self.btn_export.grid(row=16, column=0, padx=20, pady=5)
        
        self.btn_png = ctk.CTkButton(
            self.sidebar_frame, text="SAVE AS PNG 📸", command=self.export_png_dialog, 
            font=self.font_main, fg_color="#91d2ff", text_color="white",
            border_width=2, border_color=self.color_border, hover_color="#7cbce8", corner_radius=8
        )
        self.btn_png.grid(row=17, column=0, padx=20, pady=(5, 20))

        # Main Frame
        self.main_frame = ctk.CTkFrame(self.root, corner_radius=0, fg_color="transparent")
        self.main_frame.grid(row=0, column=1, sticky="nsew", pady=10, padx=(0, 10))
        self.main_frame.grid_rowconfigure(1, weight=1)
        self.main_frame.grid_columnconfigure(0, weight=1)

        # Top Bar (Cell Size + Grid Size)
        self.top_bar = ctk.CTkFrame(
            self.main_frame, height=50, corner_radius=10, 
            fg_color="white", border_width=3, border_color=self.color_border
        )
        self.top_bar.grid(row=0, column=0, sticky="ew")
        
        # Dynamic Grid Inputs
        ctk.CTkLabel(self.top_bar, text="R:", font=self.font_main, text_color=self.color_text).pack(side="left", padx=(10,2), pady=10)
        self.rows_entry = ctk.CTkEntry(self.top_bar, width=40, font=self.font_main, border_width=2, border_color=self.color_border)
        self.rows_entry.insert(0, str(self.maze.rows))
        self.rows_entry.pack(side="left", padx=2, pady=10)
        
        ctk.CTkLabel(self.top_bar, text="C:", font=self.font_main, text_color=self.color_text).pack(side="left", padx=(5,2), pady=10)
        self.cols_entry = ctk.CTkEntry(self.top_bar, width=40, font=self.font_main, border_width=2, border_color=self.color_border)
        self.cols_entry.insert(0, str(self.maze.cols))
        self.cols_entry.pack(side="left", padx=2, pady=10)
        
        ctk.CTkLabel(self.top_bar, text="SIZE(M):", font=self.font_main, text_color=self.color_text).pack(side="left", padx=(10,2), pady=10)
        self.cell_size_entry = ctk.CTkEntry(self.top_bar, width=50, font=self.font_main, border_width=2, border_color=self.color_border)
        self.cell_size_entry.insert(0, str(self.maze.cell_size_m))
        self.cell_size_entry.pack(side="left", padx=2, pady=10)
        
        ctk.CTkButton(
            self.top_bar, text="APPLY GRID", command=self.apply_grid_settings, width=80,
            font=self.font_main, fg_color=self.color_btn_green, text_color=self.color_text,
            border_width=2, border_color=self.color_border, hover_color=self.color_btn_hover
        ).pack(side="left", padx=10, pady=10)
        
        info_text = "Click internal edges for WALLS.\nClick inside cells for MARKERS."
        ctk.CTkLabel(
            self.top_bar, text=info_text, text_color=self.color_sidebar, 
            font=ctk.CTkFont(family="Courier New", size=11, weight="bold"), justify="right"
        ).pack(side="right", padx=15)

        # Canvas Setup
        self.canvas_frame = ctk.CTkFrame(
            self.main_frame, corner_radius=10, fg_color="white", 
            border_width=3, border_color=self.color_border
        )
        self.canvas_frame.grid(row=1, column=0, sticky="nsew", pady=10)
        
        self.canvas_title = ctk.CTkLabel(
            self.canvas_frame, text=" MAZE VIEW ", font=self.font_main, 
            text_color="white", fg_color=self.color_sidebar, corner_radius=0
        )
        self.canvas_title.pack(fill="x", padx=3, pady=(3, 0))

        self.canvas_bg = "#f4f8ff"
        self.canvas = tk.Canvas(self.canvas_frame, bg=self.canvas_bg, highlightthickness=0)
        self.canvas.pack(fill="both", expand=True, padx=3, pady=(0, 3))
        self.canvas.bind("<Button-1>", self.on_click)

        # Status Bar
        self.status_label = ctk.CTkLabel(
            self.main_frame, textvariable=self.status, anchor="w", 
            font=self.font_main, text_color=self.color_text, 
            fg_color=self.color_btn_yellow, corner_radius=10,
            border_width=2, border_color=self.color_border, padx=15
        )
        self.status_label.grid(row=2, column=0, sticky="ew", pady=(0, 0))

    def _resize_window(self):
        w = 2 * self.PAD + self.maze.cols * self.CELL_PX + 340
        h = 2 * self.PAD + self.maze.rows * self.CELL_PX + 220
        self.root.geometry(f"{int(w)}x{int(h)}")
        self.root.minsize(int(w), int(h))

    def new_maze(self):
        if not messagebox.askyesno("New maze", "Clear the current maze?"):
            return
        self.save_state()
        self.maze = MazeData(rows=self.maze.rows, cols=self.maze.cols, cell_size_m=self.maze.cell_size_m)
        self.current_file = None
        self.clear_route()
        self.redraw()

    def apply_grid_settings(self):
        try:
            r = int(self.rows_entry.get())
            c = int(self.cols_entry.get())
            cs = float(self.cell_size_entry.get())
            if not (2 <= r <= 40 and 2 <= c <= 40): raise ValueError
            if not 0.05 <= cs <= 5.0: raise ValueError
        except ValueError:
            messagebox.showerror("Invalid input", "Rows/Cols 2-40. Cell size 0.05-5.0")
            return
        
        self.save_state()
        self.maze.rows = r
        self.maze.cols = c
        self.maze.cell_size_m = cs
        
        # Cleanup out of bounds
        out_of_bounds = [w for w in self.maze.walls if not (self.maze.in_bounds(w[0]) and self.maze.in_bounds(w[1]))]
        for w in out_of_bounds: self.maze.walls.discard(w)
        if self.maze.start and not self.maze.in_bounds(self.maze.start): self.maze.start = None
        if self.maze.object_cell and not self.maze.in_bounds(self.maze.object_cell): self.maze.object_cell = None
        if self.maze.drop_cell and not self.maze.in_bounds(self.maze.drop_cell): self.maze.drop_cell = None
        if self.maze.goal and not self.maze.in_bounds(self.maze.goal): self.maze.goal = None
        
        self.route = []
        self.route_plan = None
        self.cancel_animation()
        self._resize_window()
        self.redraw()
        self.status.set(f"Grid updated to {r}x{c}, Size: {cs}m")

    def cancel_animation(self):
        if self.anim_id:
            self.root.after_cancel(self.anim_id)
            self.anim_id = None
        if hasattr(self, 'gif_anim_id') and self.gif_anim_id:
            self.root.after_cancel(self.gif_anim_id)
            self.gif_anim_id = None
        if hasattr(self, 'gif_anim_id') and self.gif_anim_id:
            self.root.after_cancel(self.gif_anim_id)
            self.gif_anim_id = None

    def clear_route(self):
        self.cancel_animation()
        self.route = []
        self.route_plan = None
        self.redraw()

    def rotate_start(self):
        self.save_state()
        i = HEADINGS.index(self.maze.start_heading)
        self.maze.start_heading = HEADINGS[(i + 1) % 4]
        self.route_plan = None
        self.status.set(f"Start heading = {self.maze.start_heading}")
        self.redraw()

    def cell_rect(self, cell: Cell) -> Tuple[float, float, float, float]:
        r, c = cell
        x0 = self.PAD + c * self.CELL_PX
        y0 = self.PAD + r * self.CELL_PX
        return x0, y0, x0 + self.CELL_PX, y0 + self.CELL_PX

    def event_to_cell(self, x: float, y: float) -> Optional[Cell]:
        c = int((x - self.PAD) // self.CELL_PX)
        r = int((y - self.PAD) // self.CELL_PX)
        cell = (r, c)
        return cell if self.maze.in_bounds(cell) else None

    def nearest_internal_edge(self, cell: Cell, x: float, y: float) -> Optional[Tuple[Cell, Cell]]:
        x0, y0, x1, y1 = self.cell_rect(cell)
        distances = [
            (abs(y - y0), "N"),
            (abs(x - x1), "E"),
            (abs(y - y1), "S"),
            (abs(x - x0), "W"),
        ]
        dist, heading = min(distances)
        if dist > self.EDGE_PICK_PX:
            return None
        dr, dc = HEADING_VEC[heading]
        other = (cell[0] + dr, cell[1] + dc)
        if not self.maze.in_bounds(other):
            return None
        return cell, other

    def on_click(self, event):
        cell = self.event_to_cell(event.x, event.y)
        if cell is None:
            return
        
        self.save_state()
        mode = self.mode.get()

        if mode in ("WALL", "ERASE"):
            edge = self.nearest_internal_edge(cell, event.x, event.y)
            if edge is None:
                self.history_undo.pop()
                self.status.set("Click closer to an INTERNAL cell edge to edit a wall")
                return
            self.maze.set_wall_between(edge[0], edge[1], mode == "WALL")
            self.status.set(f"{'Wall added' if mode == 'WALL' else 'Wall erased'}: {edge[0]} <-> {edge[1]}")
        elif mode == "START":
            self.maze.start = cell
            self.status.set(f"START = {cell}, heading {self.maze.start_heading}")
        elif mode == "PICKUP":
            self.maze.object_cell = cell
            self.status.set(f"PICKUP = {cell}")
        elif mode == "DROP":
            self.maze.drop_cell = cell
            self.status.set(f"DROP = {cell}")
        elif mode == "EXIT":
            self.maze.goal = cell
            self.status.set(f"EXIT = {cell}")

        self.clear_route()

    def preview_route(self):
        try:
            r = int(self.rows_entry.get())
            c = int(self.cols_entry.get())
            cs = float(self.cell_size_entry.get())
            self.maze.rows = r
            self.maze.cols = c
            self.maze.cell_size_m = cs
        except: pass
        
        try:
            plan = validate_and_plan(self.maze)
        except ValueError as exc:
            messagebox.showerror("Route error", str(exc))
            return
            
        self.route_plan = plan
        self.route = [tuple(c) for c in plan["path"]]
        warning = " | ".join(plan["warnings"])
        self.status.set(
            f"Guide OK: {len(plan['topology_guide']['nodes'])} nodes, "
            f"{plan['total_cells']} cells" + (f" | WARNING: {warning}" if warning else "")
        )
        self.redraw()
        
        # Start Animation
        self.cancel_animation()
        self.anim_index = 0
        self.animate_robot()

        if warning:
            messagebox.showwarning("Route warning", warning)

    def animate_robot(self):
        if self.anim_index >= len(self.route):
            self.anim_id = None
            return
            
        if self.anim_index == 0:
            self.redraw()
            if hasattr(self, 'robot_frames') and len(self.robot_frames) > 1:
                self.update_gif_frame()
            
        cell = self.route[self.anim_index]
        x0, y0, x1, y1 = self.cell_rect(cell)
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        
        self.canvas.delete("robot")
        if self.robot_image:
            self.canvas.create_image(cx, cy, image=self.robot_image, tags="robot")
        else:
            rad = self.CELL_PX * 0.35
            self.canvas.create_oval(cx - rad, cy - rad, cx + rad, cy + rad, fill="#ff9eb5", outline="#1f2a63", width=3, tags="robot")
            self.canvas.create_oval(cx - rad/2, cy - rad/2, cx + rad/2, cy + rad/2, fill="#fff785", outline="", tags="robot")
            
        self.canvas.tag_raise("robot")
        self.anim_index += 1
        self.anim_id = self.root.after(300, self.animate_robot)

    def export_png_dialog(self):
        path = filedialog.asksaveasfilename(
            defaultextension=".png",
            filetypes=[("PNG Image", "*.png"), ("All Files", "*.*")],
            title="Save as PNG",
            initialfile="maze_screenshot.png"
        )
        if not path:
            return
        
        try:
            if HAS_PIL and hasattr(ImageGrab, "grab"):
                self.root.update_idletasks()
                x = self.canvas.winfo_rootx()
                y = self.canvas.winfo_rooty()
                w = self.canvas.winfo_width()
                h = self.canvas.winfo_height()
                img = ImageGrab.grab(bbox=(x, y, x+w, y+h))
                img.save(path)
                self.status.set(f"Saved PNG to {os.path.basename(path)}")
                messagebox.showinfo("Success", f"Saved PNG to:\n{path}")
            else:
                self.status.set("Error: PIL.ImageGrab not available.")
        except Exception as e:
            self.status.set(f"Save PNG failed: {e}")

    def get_reachable_cells(self):
        reachable = set()
        start = self.maze.start if self.maze.start else (0, 0)
        if not self.maze.in_bounds(start): return reachable
        
        queue = [start]
        visited = {start}
        
        while queue:
            curr = queue.pop(0)
            reachable.add(curr)
            for nr, nc in self.maze.neighbors(curr):
                if not self.maze.has_wall_between(curr, (nr, nc)):
                    if (nr, nc) not in visited:
                        visited.add((nr, nc))
                        queue.append((nr, nc))
        return reachable

    def save_maze_dialog(self):
        path = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=[("Maze JSON", "*.json"), ("All files", "*.*")],
            initialfile=os.path.basename(self.current_file) if self.current_file else "known_maze.json",
        )
        if not path:
            return
        save_maze(path, self.maze)
        self.current_file = path
        self.status.set(f"Saved maze: {path}")

    def load_maze_dialog(self):
        path = filedialog.askopenfilename(filetypes=[("Maze JSON", "*.json"), ("All files", "*.*")])
        if not path:
            return
        try:
            self.save_state()
            self.maze = load_maze(path)
        except Exception as exc:
            messagebox.showerror("Load error", str(exc))
            return
        self.current_file = path
        self.rows_entry.delete(0, tk.END); self.rows_entry.insert(0, str(self.maze.rows))
        self.cols_entry.delete(0, tk.END); self.cols_entry.insert(0, str(self.maze.cols))
        self.cell_size_entry.delete(0, tk.END); self.cell_size_entry.insert(0, str(self.maze.cell_size_m))
        self.clear_route()
        self._resize_window()
        self.status.set(f"Loaded maze: {path}")

    def export_route_dialog(self):
        try:
            plan = validate_and_plan(self.maze)
        except ValueError as exc:
            messagebox.showerror("Route error", str(exc))
            return
        path = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=[("Route JSON", "*.json"), ("All files", "*.*")],
            initialfile="known_route.json",
        )
        if not path:
            return
        save_plan(path, plan)
        self.route_plan = plan
        self.route = [tuple(c) for c in plan["path"]]
        self.redraw()
        self.status.set(f"Exported robot route: {path}")

    def redraw(self):
        c = self.canvas
        c.delete("all")

        route_color = "#ff7eac"
        route_glow = "#ffdbe7"
        cell_fill = "#ffffff"
        cell_outline = "#c7d7f7"
        text_color = "#99b6ea"
        wall_color = "#1f2a63"
        border_color = "#1f2a63"
        
        # Route preview first, under grid walls.
        if len(self.route) >= 2:
            pts = []
            for cell in self.route:
                x0, y0, x1, y1 = self.cell_rect(cell)
                pts.extend(((x0 + x1) / 2, (y0 + y1) / 2))
            c.create_line(*pts, width=12, fill=route_glow, smooth=False, capstyle=tk.ROUND, joinstyle=tk.ROUND)

        reachable = self.get_reachable_cells()

        # Cell background and labels.
        for r in range(self.maze.rows):
            for col in range(self.maze.cols):
                cell = (r, col)
                x0, y0, x1, y1 = self.cell_rect(cell)
                fill_color = cell_fill if cell in reachable else "#dee7f7"
                c.create_rectangle(x0, y0, x1, y1, fill=fill_color, outline=cell_outline, width=2)
                c.create_text(x0 + 6, y0 + 6, text=f"{r},{col}", anchor="nw", fill=text_color, font=("Courier New", 8, "bold"))

        # Route redraw on top of cell fill.
        if len(self.route) >= 2:
            pts = []
            for cell in self.route:
                x0, y0, x1, y1 = self.cell_rect(cell)
                pts.extend(((x0 + x1) / 2, (y0 + y1) / 2))
            c.create_line(*pts, width=6, fill=route_color, capstyle=tk.ROUND, joinstyle=tk.ROUND)

        # Outer border.
        left = self.PAD
        top = self.PAD
        right = self.PAD + self.maze.cols * self.CELL_PX
        bottom = self.PAD + self.maze.rows * self.CELL_PX
        c.create_rectangle(left, top, right, bottom, width=4, outline=border_color)

        # Internal walls.
        for a, b in self.maze.walls:
            ar, ac = a
            br, bc = b
            ax0, ay0, ax1, ay1 = self.cell_rect(a)
            if ar == br:
                x = ax1 if bc > ac else ax0
                c.create_line(x, ay0, x, ay1, width=6, fill=wall_color)
            else:
                y = ay1 if br > ar else ay0
                c.create_line(ax0, y, ax1, y, width=6, fill=wall_color)

        # Markers.
        if self.maze.start is not None:
            self._draw_marker(self.maze.start, "S", self.color_btn_green)
        if self.maze.object_cell is not None:
            self._draw_marker(self.maze.object_cell, "P", "#ffb459")
        if self.maze.drop_cell is not None:
            self._draw_marker(self.maze.drop_cell, "D", "#7ea8f8")
        if self.maze.goal is not None:
            self._draw_marker(self.maze.goal, "E", "#ff9eb5")

    def _draw_marker(self, cell: Cell, text: str, color: str):
        x0, y0, x1, y1 = self.cell_rect(cell)
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        rad = self.CELL_PX * 0.32
        
        self.canvas.create_oval(cx - rad + 3, cy - rad + 3, cx + rad + 3, cy + rad + 3, fill=self.color_border, outline="")
        self.canvas.create_oval(cx - rad, cy - rad, cx + rad, cy + rad, fill=color, outline=self.color_border, width=2)
        self.canvas.create_text(cx, cy, text=text, fill=self.color_text, font=("Courier New", 14, "bold"))


def cli_plan(maze_path: str, output: Optional[str]) -> int:
    maze = load_maze(maze_path)
    plan = validate_and_plan(maze)
    print(json.dumps(plan, indent=2, ensure_ascii=False))
    if output:
        save_plan(output, plan)
        print(f"\nSaved route -> {output}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="RoboMaster pre-drawn maze designer")
    parser.add_argument("--plan", metavar="MAZE.json", help="compile a saved maze without opening the GUI")
    parser.add_argument("--out", metavar="ROUTE.json", help="output path used with --plan")
    parser.add_argument("--rows", type=int, default=8)
    parser.add_argument("--cols", type=int, default=10)
    parser.add_argument("--cell-size", type=float, default=0.40)
    args = parser.parse_args()

    if args.plan:
        return cli_plan(args.plan, args.out)

    root = ctk.CTk()
    MazeDesignerApp(root, rows=max(2, args.rows), cols=max(2, args.cols), cell_size_m=args.cell_size)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

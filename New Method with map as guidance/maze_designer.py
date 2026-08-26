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
import tkinter as tk
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

class MazeDesignerApp:
    CELL_PX = 62
    PAD = 28
    EDGE_PICK_PX = 12

    def __init__(self, root: tk.Tk, rows: int = 8, cols: int = 10, cell_size_m: float = 0.40):
        self.root = root
        self.root.title("RoboMaster Pre-drawn Maze Designer")
        self.maze = MazeData(rows=rows, cols=cols, cell_size_m=cell_size_m)
        self.mode = tk.StringVar(value="WALL")
        self.status = tk.StringVar(value="Draw walls, then place START / PICKUP / DROP / EXIT")
        self.route: List[Cell] = []
        self.route_plan: Optional[dict] = None
        self.current_file: Optional[str] = None

        self._build_ui()
        self._resize_canvas()
        self.redraw()

    def _build_ui(self):
        top = tk.Frame(self.root, padx=8, pady=7)
        top.pack(fill="x")

        modes = [
            ("Wall", "WALL"),
            ("Erase wall", "ERASE"),
            ("Start", "START"),
            ("Pickup", "PICKUP"),
            ("Drop", "DROP"),
            ("Exit", "EXIT"),
        ]
        for text, value in modes:
            tk.Radiobutton(top, text=text, value=value, variable=self.mode, indicatoron=False, width=11).pack(side="left", padx=2)

        tk.Button(top, text="Preview Guide", command=self.preview_route).pack(side="left", padx=(10, 2))
        tk.Button(top, text="Clear Route", command=self.clear_route).pack(side="left", padx=2)

        second = tk.Frame(self.root, padx=8, pady=3)
        second.pack(fill="x")
        tk.Button(second, text="New", command=self.new_maze).pack(side="left", padx=2)
        tk.Button(second, text="Save Maze", command=self.save_maze_dialog).pack(side="left", padx=2)
        tk.Button(second, text="Load Maze", command=self.load_maze_dialog).pack(side="left", padx=2)
        tk.Button(second, text="Export Route", command=self.export_route_dialog).pack(side="left", padx=2)

        tk.Label(second, text="Cell size hint (m):").pack(side="left", padx=(14, 3))
        self.cell_size_entry = tk.Entry(second, width=7)
        self.cell_size_entry.insert(0, str(self.maze.cell_size_m))
        self.cell_size_entry.pack(side="left")
        tk.Button(second, text="Apply", command=self.apply_cell_size).pack(side="left", padx=3)

        info = tk.Label(
            self.root,
            text=(
                "WALL/ERASE: click near an internal cell edge. Markers: click inside a cell.  "
                "Drawing orientation and block size are hints; robot auto-tests rotations/mirror and trusts sensors."
            ),
            anchor="w",
            padx=10,
        )
        info.pack(fill="x")

        self.canvas = tk.Canvas(self.root, bg="#f5f5f5", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True, padx=8, pady=6)
        self.canvas.bind("<Button-1>", self.on_click)

        tk.Label(self.root, textvariable=self.status, anchor="w", relief="sunken", padx=8).pack(fill="x", side="bottom")

    def _resize_canvas(self):
        w = 2 * self.PAD + self.maze.cols * self.CELL_PX
        h = 2 * self.PAD + self.maze.rows * self.CELL_PX
        self.canvas.config(width=w, height=h)
        self.root.minsize(min(w + 20, 1200), min(h + 145, 900))

    def new_maze(self):
        if not messagebox.askyesno("New maze", "Clear the current maze?"):
            return
        self.maze = MazeData(rows=self.maze.rows, cols=self.maze.cols, cell_size_m=self.maze.cell_size_m)
        self.current_file = None
        self.clear_route()
        self.redraw()

    def apply_cell_size(self):
        try:
            value = float(self.cell_size_entry.get())
            if not 0.05 <= value <= 5.0:
                raise ValueError
        except ValueError:
            messagebox.showerror("Invalid cell size", "Enter a value between 0.05 and 5.0 metres")
            return
        self.maze.cell_size_m = value
        self.status.set(f"Cell size = {value:.3f} m")
        self.route_plan = None

    def clear_route(self):
        self.route = []
        self.route_plan = None
        self.redraw()

    def rotate_start(self):
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
            # Outer border remains a wall in this first version.
            return None
        return cell, other

    def on_click(self, event):
        cell = self.event_to_cell(event.x, event.y)
        if cell is None:
            return
        mode = self.mode.get()

        if mode in ("WALL", "ERASE"):
            edge = self.nearest_internal_edge(cell, event.x, event.y)
            if edge is None:
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

        self.route = []
        self.route_plan = None
        self.redraw()

    def preview_route(self):
        self.apply_cell_size()
        try:
            plan = validate_and_plan(self.maze)
        except ValueError as exc:
            messagebox.showerror("Route error", str(exc))
            return
        self.route_plan = plan
        self.route = [tuple(c) for c in plan["path"]]
        warning = " | ".join(plan["warnings"])
        self.status.set(
            f"Guide OK: {len(plan['topology_guide']['nodes'])} recognisable nodes, "
            f"{plan['total_cells']} drawn cells (distance hint only)" + (f" | WARNING: {warning}" if warning else "")
        )
        self.redraw()
        if warning:
            messagebox.showwarning("Route warning", warning)

    def save_maze_dialog(self):
        self.apply_cell_size()
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
            self.maze = load_maze(path)
        except Exception as exc:
            messagebox.showerror("Load error", str(exc))
            return
        self.current_file = path
        self.cell_size_entry.delete(0, tk.END)
        self.cell_size_entry.insert(0, str(self.maze.cell_size_m))
        self.route = []
        self.route_plan = None
        self._resize_canvas()
        self.redraw()
        self.status.set(f"Loaded maze: {path}")

    def export_route_dialog(self):
        self.apply_cell_size()
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

        # Route preview first, under grid walls.
        if len(self.route) >= 2:
            pts = []
            for cell in self.route:
                x0, y0, x1, y1 = self.cell_rect(cell)
                pts.extend(((x0 + x1) / 2, (y0 + y1) / 2))
            c.create_line(*pts, width=6, fill="#6a5acd", smooth=False, capstyle=tk.ROUND, joinstyle=tk.ROUND)

        # Cell background and labels.
        for r in range(self.maze.rows):
            for col in range(self.maze.cols):
                cell = (r, col)
                x0, y0, x1, y1 = self.cell_rect(cell)
                c.create_rectangle(x0, y0, x1, y1, fill="#ffffff", outline="#dedede")
                c.create_text(x0 + 5, y0 + 5, text=f"{r},{col}", anchor="nw", fill="#aaaaaa", font=("Arial", 7))

        # Route redraw on top of cell fill.
        if len(self.route) >= 2:
            pts = []
            for cell in self.route:
                x0, y0, x1, y1 = self.cell_rect(cell)
                pts.extend(((x0 + x1) / 2, (y0 + y1) / 2))
            c.create_line(*pts, width=5, fill="#6a5acd", capstyle=tk.ROUND, joinstyle=tk.ROUND)

        # Outer border.
        left = self.PAD
        top = self.PAD
        right = self.PAD + self.maze.cols * self.CELL_PX
        bottom = self.PAD + self.maze.rows * self.CELL_PX
        c.create_rectangle(left, top, right, bottom, width=4, outline="#111111")

        # Internal walls.
        for a, b in self.maze.walls:
            ar, ac = a
            br, bc = b
            ax0, ay0, ax1, ay1 = self.cell_rect(a)
            if ar == br:
                x = ax1 if bc > ac else ax0
                c.create_line(x, ay0, x, ay1, width=5, fill="#111111")
            else:
                y = ay1 if br > ar else ay0
                c.create_line(ax0, y, ax1, y, width=5, fill="#111111")

        # Markers.
        if self.maze.start is not None:
            self._draw_marker(self.maze.start, "S", "#2e8b57")
        if self.maze.object_cell is not None:
            self._draw_marker(self.maze.object_cell, "P", "#ff8c00")
        if self.maze.drop_cell is not None:
            self._draw_marker(self.maze.drop_cell, "D", "#1e90ff")
        if self.maze.goal is not None:
            self._draw_marker(self.maze.goal, "E", "#b22222")

    def _draw_marker(self, cell: Cell, text: str, color: str):
        x0, y0, x1, y1 = self.cell_rect(cell)
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        rad = self.CELL_PX * 0.25
        self.canvas.create_oval(cx - rad, cy - rad, cx + rad, cy + rad, fill=color, outline="")
        self.canvas.create_text(cx, cy, text=text, fill="white", font=("Arial", 14, "bold"))

    def _draw_heading_arrow(self, cell: Cell, heading: str):
        x0, y0, x1, y1 = self.cell_rect(cell)
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        dr, dc = HEADING_VEC[heading]
        length = self.CELL_PX * 0.37
        # row increases downward, which already matches canvas Y.
        ex = cx + dc * length
        ey = cy + dr * length
        self.canvas.create_line(cx, cy, ex, ey, width=4, fill="#006400", arrow=tk.LAST, arrowshape=(10, 12, 5))


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

    root = tk.Tk()
    MazeDesignerApp(root, rows=max(2, args.rows), cols=max(2, args.cols), cell_size_m=args.cell_size)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

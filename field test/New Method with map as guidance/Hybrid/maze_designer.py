"""Pre-drawn topology-guide designer for RoboMaster.

Features
--------
- Draw maze walls on a grid with the mouse.
- Place START, PICKUP, DROP and EXIT hints; PICKUP/DROP/EXIT are optional.
- Missions may start already carrying an object: START -> DROP -> EXIT.
- A START-only design can still export a topology guide for DFS/mapping.
- Draw in any orientation; the robot tests all 4 rotations and both mirrors.
- Preview whatever mission stages are present with A*.
- Left-click adds walls; right-click removes walls or markers.
- Ctrl+Z undo, Ctrl+Y / Ctrl+Shift+Z redo.
- Resize the grid (Rows x Columns) from the GUI.
- Coordinates use (0,0) at the bottom-left.
- Fine marker placement: START/PICKUP/DROP/EXIT can be placed with sub-cell
  precision by click position, absolute X/Y, or exact distance from cell walls.
- Motion Profile supports AUTO braking calculation or MANUAL slow/stop distances.
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
import os
import tkinter as tk
from dataclasses import dataclass
from tkinter import filedialog, messagebox
from typing import Dict, Iterable, List, Optional, Sequence, Set, Tuple


Cell = Tuple[int, int]  # row/y, col/x; (0, 0) is the bottom-left cell
Edge = Tuple[Cell, Cell]
PosM = Tuple[float, float]  # (x_m, y_m) with origin at the bottom-left corner

HEADINGS = ("N", "E", "S", "W")
HEADING_VEC: Dict[str, Tuple[int, int]] = {
    # World/grid coordinates use a normal Cartesian-style origin:
    # (0, 0) is bottom-left, row/y increases NORTH/up, col/x increases EAST/right.
    "N": (+1, 0),
    "E": (0, +1),
    "S": (-1, 0),
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
    start_pos_m: Optional[PosM] = None
    object_pos_m: Optional[PosM] = None
    drop_pos_m: Optional[PosM] = None
    goal_pos_m: Optional[PosM] = None

    # Motion profile exported with the route. AUTO computes conservative
    # slow/brake trigger distances from speed and a simple braking model.
    motion_mode: str = "AUTO"
    forward_speed_mps: float = 0.20
    manual_slow_front_cm: float = 18.0
    manual_stop_front_cm: float = 15.0
    auto_reaction_time_s: float = 0.15
    auto_brake_decel_mps2: float = 0.45
    auto_safety_margin_cm: float = 3.0
    auto_clearance_cm: float = 11.0
    auto_slow_ramp_time_s: float = 0.40

    def __post_init__(self):
        if self.walls is None:
            self.walls = set()
        self.start_heading = str(self.start_heading).upper()
        if self.start_heading not in HEADINGS:
            self.start_heading = "N"
        self.motion_mode = str(self.motion_mode).upper()
        if self.motion_mode not in ("AUTO", "MANUAL"):
            self.motion_mode = "AUTO"
        self._sync_all_marker_pose_and_cells()

    @staticmethod
    def canonical_edge(a: Cell, b: Cell) -> Edge:
        return (a, b) if a <= b else (b, a)

    def in_bounds(self, cell: Cell) -> bool:
        r, c = cell
        return 0 <= r < self.rows and 0 <= c < self.cols

    def world_size_m(self) -> Tuple[float, float]:
        return self.cols * self.cell_size_m, self.rows * self.cell_size_m

    def clamp_pos_m(self, pos: PosM) -> PosM:
        x, y = float(pos[0]), float(pos[1])
        width_m, height_m = self.world_size_m()
        eps = min(self.cell_size_m * 1e-6, 1e-6)
        max_x = max(0.0, width_m - eps)
        max_y = max(0.0, height_m - eps)
        return max(0.0, min(x, max_x)), max(0.0, min(y, max_y))

    def pos_to_cell(self, pos: PosM) -> Cell:
        x, y = self.clamp_pos_m(pos)
        c = int(x / self.cell_size_m)
        r = int(y / self.cell_size_m)
        return (r, c)

    def cell_center_pos_m(self, cell: Cell) -> PosM:
        r, c = cell
        return ((c + 0.5) * self.cell_size_m, (r + 0.5) * self.cell_size_m)

    def cell_local_pos_m(self, cell: Cell, pos: PosM) -> PosM:
        """Return marker position inside a cell, measured from LEFT/BOTTOM."""
        r, c = cell
        x, y = self.clamp_pos_m(pos)
        local_x = x - c * self.cell_size_m
        local_y = y - r * self.cell_size_m
        return (
            max(0.0, min(local_x, self.cell_size_m)),
            max(0.0, min(local_y, self.cell_size_m)),
        )

    def cell_local_to_world_m(self, cell: Cell, local_x_m: float, local_y_m: float) -> PosM:
        """Convert a position relative to a cell's LEFT/BOTTOM corner to world X/Y."""
        if not self.in_bounds(cell):
            raise ValueError(f"Cell out of bounds: {cell}")
        eps = min(self.cell_size_m * 1e-6, 1e-6)
        local_x_m = max(0.0, min(float(local_x_m), self.cell_size_m - eps))
        local_y_m = max(0.0, min(float(local_y_m), self.cell_size_m - eps))
        r, c = cell
        return (c * self.cell_size_m + local_x_m, r * self.cell_size_m + local_y_m)

    def wall_clearances_m(self, cell: Cell, pos: PosM) -> dict:
        """Distances from marker centre to the four boundaries of its cell."""
        local_x, local_y = self.cell_local_pos_m(cell, pos)
        return {
            "left": local_x,
            "right": max(0.0, self.cell_size_m - local_x),
            "bottom": local_y,
            "top": max(0.0, self.cell_size_m - local_y),
        }

    def _sync_marker_pair(self, cell_attr: str, pos_attr: str) -> None:
        cell = getattr(self, cell_attr)
        pos = getattr(self, pos_attr)
        if pos is not None:
            pos = self.clamp_pos_m(pos)
            setattr(self, pos_attr, pos)
            setattr(self, cell_attr, self.pos_to_cell(pos))
        elif cell is not None and self.in_bounds(cell):
            setattr(self, pos_attr, self.cell_center_pos_m(cell))
        else:
            setattr(self, cell_attr, None)
            setattr(self, pos_attr, None)

    def _sync_all_marker_pose_and_cells(self) -> None:
        self._sync_marker_pair("start", "start_pos_m")
        self._sync_marker_pair("object_cell", "object_pos_m")
        self._sync_marker_pair("drop_cell", "drop_pos_m")
        self._sync_marker_pair("goal", "goal_pos_m")

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

    @staticmethod
    def _pose_dict(pos: Optional[PosM]) -> Optional[dict]:
        if pos is None:
            return None
        return {"x": round(pos[0], 6), "y": round(pos[1], 6)}

    @staticmethod
    def _read_pose(data: dict, *keys: str) -> Optional[PosM]:
        for key in keys:
            value = data.get(key)
            if value is None:
                continue
            if isinstance(value, dict) and "x" in value and "y" in value:
                return (float(value["x"]), float(value["y"]))
            if isinstance(value, (list, tuple)) and len(value) >= 2:
                return (float(value[0]), float(value[1]))
        return None

    def to_dict(self) -> dict:
        walls = []
        for a, b in sorted(self.walls):
            walls.append([[a[0], a[1]], [b[0], b[1]]])
        return {
            "format": "robomaster_predrawn_maze",
            "coordinate_origin": "BOTTOM_LEFT",
            "coordinate_order": "ROW_Y_COL_X",
            "row_axis": "NORTH_POSITIVE",
            "col_axis": "EAST_POSITIVE",
            "rows": self.rows,
            "cols": self.cols,
            "cell_size_m": self.cell_size_m,
            "walls": walls,
            "start": list(self.start) if self.start is not None else None,
            "start_pose_m": self._pose_dict(self.start_pos_m),
            # Kept for backward compatibility. The topology guide deliberately
            # does not trust this heading.
            "start_heading": self.start_heading,
            "pickup": list(self.object_cell) if self.object_cell is not None else None,
            "pickup_pose_m": self._pose_dict(self.object_pos_m),
            "object": list(self.object_cell) if self.object_cell is not None else None,
            "object_pose_m": self._pose_dict(self.object_pos_m),
            "drop": list(self.drop_cell) if self.drop_cell is not None else None,
            "drop_pose_m": self._pose_dict(self.drop_pos_m),
            "exit": list(self.goal) if self.goal is not None else None,
            "exit_pose_m": self._pose_dict(self.goal_pos_m),
            "goal": list(self.goal) if self.goal is not None else None,
            "goal_pose_m": self._pose_dict(self.goal_pos_m),
            "motion_settings": {
                "mode": self.motion_mode,
                "forward_speed_mps": self.forward_speed_mps,
                "manual_slow_front_cm": self.manual_slow_front_cm,
                "manual_stop_front_cm": self.manual_stop_front_cm,
                "auto_reaction_time_s": self.auto_reaction_time_s,
                "auto_brake_decel_mps2": self.auto_brake_decel_mps2,
                "auto_safety_margin_cm": self.auto_safety_margin_cm,
                "auto_clearance_cm": self.auto_clearance_cm,
                "auto_slow_ramp_time_s": self.auto_slow_ramp_time_s,
            },
        }

    @classmethod
    def from_dict(cls, data: dict) -> "MazeData":
        rows = int(data.get("rows", 8))
        cols = int(data.get("cols", 10))
        origin = str(data.get("coordinate_origin", "TOP_LEFT")).upper()

        # Legacy files may also be loaded. Older top-left files are converted here.
        legacy_top_left = origin != "BOTTOM_LEFT"

        def convert_cell(value):
            if value is None:
                return None
            r, c = int(value[0]), int(value[1])
            if legacy_top_left:
                r = rows - 1 - r
            return (r, c)

        pickup_value = data.get("pickup")
        if pickup_value is None:
            pickup_value = data.get("object")
        goal_value = data.get("exit")
        if goal_value is None:
            goal_value = data.get("goal")
        motion = data.get("motion_settings") or {}

        maze = cls(
            rows=rows,
            cols=cols,
            cell_size_m=float(data.get("cell_size_m", 0.40)),
            start=convert_cell(data.get("start")),
            start_heading=str(data.get("start_heading", "N")),
            object_cell=convert_cell(pickup_value),
            drop_cell=convert_cell(data.get("drop")),
            goal=convert_cell(goal_value),
            start_pos_m=cls._read_pose(data, "start_pose_m"),
            object_pos_m=cls._read_pose(data, "pickup_pose_m", "object_pose_m"),
            drop_pos_m=cls._read_pose(data, "drop_pose_m"),
            goal_pos_m=cls._read_pose(data, "exit_pose_m", "goal_pose_m"),
            motion_mode=str(motion.get("mode", "AUTO")),
            forward_speed_mps=float(motion.get("forward_speed_mps", 0.20)),
            manual_slow_front_cm=float(motion.get("manual_slow_front_cm", 18.0)),
            manual_stop_front_cm=float(motion.get("manual_stop_front_cm", 15.0)),
            auto_reaction_time_s=float(motion.get("auto_reaction_time_s", 0.15)),
            auto_brake_decel_mps2=float(motion.get("auto_brake_decel_mps2", 0.45)),
            auto_safety_margin_cm=float(motion.get("auto_safety_margin_cm", 3.0)),
            auto_clearance_cm=float(motion.get("auto_clearance_cm", 11.0)),
            auto_slow_ramp_time_s=float(motion.get("auto_slow_ramp_time_s", 0.40)),
        )

        for item in data.get("walls", []):
            if len(item) != 2:
                continue
            a = convert_cell(item[0])
            b = convert_cell(item[1])
            if a is not None and b is not None and maze.in_bounds(a) and maze.in_bounds(b):
                maze.set_wall_between(a, b, True)
        maze._sync_all_marker_pose_and_cells()
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

    # First motion is handled specially by the runtime startup path.
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


def open_headings(
    maze: MazeData,
    cell: Cell,
    allowed_cells: Optional[Set[Cell]] = None,
) -> List[str]:
    result = []
    r, c = cell
    for heading in HEADINGS:
        dr, dc = HEADING_VEC[heading]
        nxt = (r + dr, c + dc)
        if (
            maze.in_bounds(nxt)
            and (allowed_cells is None or nxt in allowed_cells)
            and not maze.has_wall_between(cell, nxt)
        ):
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


def is_topology_node(
    maze: MazeData,
    cell: Cell,
    allowed_cells: Optional[Set[Cell]] = None,
) -> bool:
    """Keep only places the robot can recognise without trusting metres.

    Junctions, dead-ends, corners and mission markers become graph nodes.
    Anonymous straight corridor cells are compressed into one graph edge.
    """
    if cell_markers(maze, cell):
        return True
    headings = open_headings(maze, cell, allowed_cells)
    if len(headings) != 2:
        return True
    a = HEADINGS.index(headings[0])
    b = HEADINGS.index(headings[1])
    return (a - b) % 4 != 2


def reachable_cells(
    maze: MazeData,
    start: Cell,
    allowed_cells: Optional[Set[Cell]] = None,
) -> Set[Cell]:
    seen = {start}
    queue = [start]
    while queue:
        cell = queue.pop(0)
        for nxt in maze.neighbors(cell):
            if allowed_cells is not None and nxt not in allowed_cells:
                continue
            if nxt not in seen:
                seen.add(nxt)
                queue.append(nxt)
    return seen


def mission_path_for_guide(maze: MazeData) -> Optional[List[Cell]]:
    """Return the ordered mission path used to constrain the runtime guide.

    Blank cells in the editor mean "no wall was drawn"; they do not prove that
    the robot may use every empty cell outside the physical maze.  A mission
    guide must therefore be induced by the compiled START/PICKUP/DROP/EXIT
    route.  START-only mapping projects keep the older reachable-drawing scope
    because they deliberately have no destination to guide toward.
    """
    if maze.start is None:
        return None

    mission_cells = [maze.start]
    mission_cells.extend(
        cell
        for cell in (maze.object_cell, maze.drop_cell, maze.goal)
        if cell is not None
    )
    if len(mission_cells) <= 1:
        return None

    path: List[Cell] = [mission_cells[0]]
    for leg_start, target in zip(mission_cells, mission_cells[1:]):
        leg = astar(maze, leg_start, target)
        if leg is None:
            raise ValueError(f"No path from {leg_start} to {target}")
        path = merge_paths(path, leg)
    return path


def build_topology_guide(
    maze: MazeData,
    mission_path: Optional[Sequence[Cell]] = None,
) -> dict:
    """Compile an orientation-free graph without treating blank outside cells as maze.

    When a mission target exists the graph is restricted to the union of cells
    in the compiled mission path.  Live DFS may still explore deviations, but
    the soft Guide can no longer localise onto arbitrary empty grid cells and
    then confidently route toward the wrong branch.
    """
    if maze.start is None:
        raise ValueError("START is not set")

    has_mission_target = any(
        cell is not None
        for cell in (maze.object_cell, maze.drop_cell, maze.goal)
    )
    if not has_mission_target:
        mission_path = None
    if mission_path is None:
        mission_path = mission_path_for_guide(maze)
    route_cells = set(mission_path or [])
    allowed_cells: Optional[Set[Cell]] = route_cells or None
    if allowed_cells is not None:
        allowed_cells.add(maze.start)

    reachable = reachable_cells(maze, maze.start, allowed_cells)
    node_cells = sorted(
        cell
        for cell in reachable
        if is_topology_node(maze, cell, allowed_cells)
    )
    if maze.start not in node_cells:
        node_cells.insert(0, maze.start)

    ordered = [maze.start] + [cell for cell in node_cells if cell != maze.start]
    cell_to_id = {cell: f"M{index}" for index, cell in enumerate(ordered)}
    nodes = {}

    for cell in ordered:
        node_id = cell_to_id[cell]
        exits = {}
        r, c = cell
        for heading in open_headings(maze, cell, allowed_cells):
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
                onward = [
                    nxt
                    for nxt in maze.neighbors(current)
                    if nxt != previous
                    and (allowed_cells is None or nxt in allowed_cells)
                ]
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
        "format": "robomaster_topology_guide",
        "graph_scope": (
            "MISSION_ROUTE_CELLS"
            if allowed_cells is not None
            else "REACHABLE_DRAWING"
        ),
        "route_cells": (
            [list(cell) for cell in mission_path]
            if mission_path is not None
            else []
        ),
        "orientation_policy": "TRY_4_ROTATIONS_AND_BOTH_MIRRORS",
        "distance_policy": "TOPOLOGY_PRIMARY_GRID_STEPS_HINT_ONLY",
        "sensor_policy": "LIVE_SENSORS_OVERRIDE_DRAWING",
        "start_heading_required": False,
        "start_node_id": cell_to_id[maze.start],
        "mission_order": mission_order,
        "marker_nodes": marker_nodes,
        "nodes": nodes,
    }


def topology_preview_edges(plan: dict) -> List[Tuple[Cell, Cell]]:
    """Return unique topology graph edges as cell pairs for GUI preview.

    Mission exports preview only the route-induced guide graph.  A START-only
    mapping project still previews the complete reachable drawing used by DFS.
    """
    guide = plan.get("topology_guide") or {}
    nodes = guide.get("nodes") or {}
    id_to_cell: Dict[str, Cell] = {}
    for node_id, node in nodes.items():
        cell = node.get("cell")
        if isinstance(cell, (list, tuple)) and len(cell) >= 2:
            id_to_cell[str(node_id)] = (int(cell[0]), int(cell[1]))

    edges: List[Tuple[Cell, Cell]] = []
    seen: Set[Tuple[str, str]] = set()
    for node_id, node in nodes.items():
        a_id = str(node_id)
        a_cell = id_to_cell.get(a_id)
        if a_cell is None:
            continue
        for exit_info in (node.get("exits") or {}).values():
            b_id = str(exit_info.get("target"))
            b_cell = id_to_cell.get(b_id)
            if b_cell is None:
                continue
            key = tuple(sorted((a_id, b_id)))
            if key in seen:
                continue
            seen.add(key)
            edges.append((a_cell, b_cell))
    return edges


def marker_positions_for_export(maze: MazeData) -> dict:
    def pack(cell: Optional[Cell], pos: Optional[PosM]):
        if cell is None and pos is None:
            return None
        result = {
            "cell": list(cell) if cell is not None else None,
            "pose_m": MazeData._pose_dict(pos),
        }
        if cell is not None and pos is not None:
            local_x, local_y = maze.cell_local_pos_m(cell, pos)
            clear = maze.wall_clearances_m(cell, pos)
            result["cell_offset_m"] = {
                "x_from_left": round(local_x, 6),
                "y_from_bottom": round(local_y, 6),
            }
            result["cell_wall_clearance_m"] = {k: round(v, 6) for k, v in clear.items()}
        return result

    return {
        "start": pack(maze.start, maze.start_pos_m),
        "pickup": pack(maze.object_cell, maze.object_pos_m),
        "drop": pack(maze.drop_cell, maze.drop_pos_m),
        "exit": pack(maze.goal, maze.goal_pos_m),
    }


def calculate_motion_profile(maze: MazeData) -> dict:
    """Return the route's front braking profile.

    AUTO model:
      reaction_distance = v * reaction_time
      braking_distance  = v^2 / (2*a)
      stop_trigger      = desired_clearance + reaction + braking + safety_margin
      slow_trigger      = stop_trigger + max(5 cm, v * slow_ramp_time)
    """
    v = float(maze.forward_speed_mps)
    if not 0.03 <= v <= 0.60:
        raise ValueError("Forward speed must be between 0.03 and 0.60 m/s")

    mode = str(maze.motion_mode).upper()
    if mode == "MANUAL":
        stop_cm = float(maze.manual_stop_front_cm)
        slow_cm = float(maze.manual_slow_front_cm)
        if not 5.0 <= stop_cm <= 150.0:
            raise ValueError("Manual brake/stop trigger must be between 5 and 150 cm")
        if not stop_cm + 1.0 <= slow_cm <= 250.0:
            raise ValueError("Manual slow trigger must be at least 1 cm farther than brake/stop trigger")
        return {
            "mode": "MANUAL",
            "forward_speed_mps": round(v, 4),
            "slow_front_cm": round(slow_cm, 2),
            "stop_front_cm": round(stop_cm, 2),
            "min_forward_speed_mps": round(min(0.05, max(0.03, v * 0.25)), 4),
            "calculation": "USER_DEFINED",
        }

    reaction = float(maze.auto_reaction_time_s)
    decel = float(maze.auto_brake_decel_mps2)
    margin_cm = float(maze.auto_safety_margin_cm)
    clearance_cm = float(maze.auto_clearance_cm)
    ramp_time = float(maze.auto_slow_ramp_time_s)
    if not 0.02 <= reaction <= 1.0:
        raise ValueError("Auto reaction time must be between 0.02 and 1.0 s")
    if not 0.10 <= decel <= 3.0:
        raise ValueError("Auto brake deceleration must be between 0.10 and 3.0 m/s^2")
    if not 0.0 <= margin_cm <= 50.0:
        raise ValueError("Auto safety margin must be between 0 and 50 cm")
    if not 5.0 <= clearance_cm <= 80.0:
        raise ValueError("Auto desired clearance must be between 5 and 80 cm")
    if not 0.05 <= ramp_time <= 2.0:
        raise ValueError("Auto slow-ramp time must be between 0.05 and 2.0 s")

    reaction_m = v * reaction
    braking_m = (v * v) / (2.0 * decel)
    stop_cm = clearance_cm + margin_cm + 100.0 * (reaction_m + braking_m)
    slow_extra_cm = max(5.0, 100.0 * v * ramp_time)
    slow_cm = stop_cm + slow_extra_cm
    return {
        "mode": "AUTO",
        "forward_speed_mps": round(v, 4),
        "slow_front_cm": round(slow_cm, 2),
        "stop_front_cm": round(stop_cm, 2),
        "min_forward_speed_mps": round(min(0.05, max(0.03, v * 0.25)), 4),
        "calculation": {
            "reaction_time_s": reaction,
            "brake_deceleration_mps2": decel,
            "safety_margin_cm": margin_cm,
            "desired_clearance_cm": clearance_cm,
            "slow_ramp_time_s": ramp_time,
            "reaction_distance_cm": round(reaction_m * 100.0, 2),
            "braking_distance_cm": round(braking_m * 100.0, 2),
            "slow_ramp_extra_cm": round(slow_extra_cm, 2),
        },
    }


def validate_and_plan(maze: MazeData) -> dict:
    """Compile a flexible mission route plus a topology guide.

    START is the only mandatory marker. PICKUP, DROP and EXIT are optional:

    - START -> PICKUP -> DROP -> EXIT : normal pickup mission
    - START -> DROP -> EXIT           : robot starts with the object already
    - START -> EXIT                   : navigation-only mission
    - START only                      : topology/mapping guide only; runtime DFS
      can explore without a pre-drawn mission target

    This keeps the designer useful even when the field rules give the robot the
    object before the run, or when the drawing is used only as a soft map hint.
    """
    if maze.start is None:
        raise ValueError("START is not set")

    mission_points = [("START", maze.start)]
    for name, cell in (
        ("PICKUP", maze.object_cell),
        ("DROP", maze.drop_cell),
        ("EXIT", maze.goal),
    ):
        if cell is not None:
            mission_points.append((name, cell))

    # START-only is valid: it exports a topology guide and an empty motion route.
    path: List[Cell] = [maze.start]
    leg_summaries = []
    for (from_name, leg_start), (to_name, target) in zip(mission_points, mission_points[1:]):
        leg = astar(maze, leg_start, target)
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

    inferred_heading = direction_between(path[0], path[1]) if len(path) >= 2 else maze.start_heading
    segments = route_segments(path, inferred_heading)
    actions = decision_actions(maze, path, inferred_heading)
    topology_guide = build_topology_guide(maze, mission_path=path)

    starts_with_object = maze.object_cell is None and maze.drop_cell is not None
    if maze.object_cell is not None:
        mission_mode = "PICKUP_MISSION"
    elif maze.drop_cell is not None:
        mission_mode = "START_WITH_OBJECT"
    elif maze.goal is not None:
        mission_mode = "NAVIGATION_ONLY"
    else:
        mission_mode = "MAPPING_ONLY"

    warnings = []
    if starts_with_object:
        warnings.append(
            "PICKUP is not set; mission assumes the robot STARTS WITH THE OBJECT and routes directly to DROP."
        )
    elif maze.object_cell is None and maze.drop_cell is None and maze.goal is not None:
        warnings.append("PICKUP/DROP are not set; mission is navigation-only: START -> EXIT.")
    elif maze.object_cell is None and maze.drop_cell is None and maze.goal is None:
        warnings.append(
            "No mission target is set after START; export is topology/mapping-only and runtime exploration/DFS may choose the path."
        )

    if maze.object_cell is not None and maze.drop_cell is None:
        warnings.append("DROP is not set; mission skips DROP and continues to the next configured stage.")
    if maze.goal is None:
        warnings.append(
            "EXIT is not set; the pre-drawn route ends at the last configured marker, but the topology guide can still be used for exploration/mapping."
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
        "format": "robomaster_known_route",
        "coordinate_origin": "BOTTOM_LEFT",
        "coordinate_order": "ROW_Y_COL_X",
        "mission_mode": mission_mode,
        "starts_with_object": starts_with_object,
        "mission_order": [name for name, _ in mission_points[1:]],
        "maze": maze.to_dict(),
        "marker_positions": marker_positions_for_export(maze),
        "topology_guide": topology_guide,
        "path": [list(c) for c in path],
        "mission_legs": leg_summaries,
        "total_cells": max(0, len(path) - 1),
        "total_distance_m": max(0, len(path) - 1) * maze.cell_size_m,
        "segments": segments,
        "decision_actions": actions,
        "motion_profile": calculate_motion_profile(maze),
        "warnings": warnings,
    }


def load_maze(path: str) -> MazeData:
    with open(path, "r", encoding="utf-8") as f:
        payload = json.load(f)
    # Allow reopening either a normal Save Maze file or a full Export Route file.
    if isinstance(payload, dict) and isinstance(payload.get("maze"), dict):
        payload = payload["maze"]
    return MazeData.from_dict(payload)


def save_maze(path: str, maze: MazeData) -> None:
    """Save the editable maze and embed a runtime-ready topology guide when possible.

    The extra topology_guide field is ignored by MazeData.from_dict(), so the file
    remains editable, but it can also be passed directly to maze_monster.py --guide.
    """
    payload = maze.to_dict()
    if maze.start is not None:
        try:
            payload["topology_guide"] = build_topology_guide(maze)
            payload["mission_order"] = list(payload["topology_guide"].get("mission_order") or [])
            payload["marker_positions"] = marker_positions_for_export(maze)
            payload["motion_profile"] = calculate_motion_profile(maze)
            payload["runtime_guide_ready"] = True
        except Exception as exc:
            # Never prevent saving work-in-progress geometry.  Runtime will report
            # the guide problem clearly if this unfinished file is used later.
            payload["runtime_guide_ready"] = False
            payload["runtime_guide_error"] = str(exc)
    else:
        payload["runtime_guide_ready"] = False
        payload["runtime_guide_error"] = "START is not set"
    with open(path, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, ensure_ascii=False)


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
        self.status = tk.StringVar(value="Draw walls; START is required. PICKUP / DROP / EXIT are optional mission stages.")
        self.route: List[Cell] = []
        # Full topology preview is separate from the mission path. This lets
        # START-only designs show a useful guide without requiring PICKUP.
        self.guide_edges: List[Tuple[Cell, Cell]] = []
        self.route_plan: Optional[dict] = None
        self.current_file: Optional[str] = None
        self.fine_marker_mode = tk.BooleanVar(value=True)
        self.marker_x_ref = tk.StringVar(value="LEFT")
        self.marker_y_ref = tk.StringVar(value="TOP")
        self.marker_measure_text = tk.StringVar(value="Select a marker or click inside a cell")

        self.motion_mode_var = tk.StringVar(value=self.maze.motion_mode)
        self.forward_speed_var = tk.StringVar(value=f"{self.maze.forward_speed_mps:.2f}")
        self.manual_slow_var = tk.StringVar(value=f"{self.maze.manual_slow_front_cm:.1f}")
        self.manual_stop_var = tk.StringVar(value=f"{self.maze.manual_stop_front_cm:.1f}")
        self.auto_reaction_var = tk.StringVar(value=f"{self.maze.auto_reaction_time_s:.2f}")
        self.auto_decel_var = tk.StringVar(value=f"{self.maze.auto_brake_decel_mps2:.2f}")
        self.auto_margin_var = tk.StringVar(value=f"{self.maze.auto_safety_margin_cm:.1f}")
        self.auto_clearance_var = tk.StringVar(value=f"{self.maze.auto_clearance_cm:.1f}")
        self.auto_ramp_var = tk.StringVar(value=f"{self.maze.auto_slow_ramp_time_s:.2f}")
        self.motion_result_var = tk.StringVar(value="")
        self.motion_panel_visible = tk.BooleanVar(value=False)
        self.motion_toggle_text = tk.StringVar(value="Show Motion Profile")

        # Full maze edit history. A snapshot is stored BEFORE every mutation so
        # Ctrl+Z can recover walls, markers, fine positions, grid resize, cell
        # size and START heading changes. Route preview itself is not history.
        self.undo_stack: List[dict] = []
        self.redo_stack: List[dict] = []
        self.history_limit = 150

        self._build_ui()
        self.mode.trace_add("write", lambda *_: self.current_mode_changed_marker())
        self.marker_x_ref.trace_add("write", lambda *_: self.refresh_relative_marker_fields())
        self.marker_y_ref.trace_add("write", lambda *_: self.refresh_relative_marker_fields())
        self.root.bind_all("<Control-z>", self.undo)
        self.root.bind_all("<Control-y>", self.redo)
        self.root.bind_all("<Control-Shift-Z>", self.redo)
        self.root.bind_all("<Control-Shift-z>", self.redo)
        self.root.bind_all("<Delete>", self.delete_selected_marker)
        self._resize_canvas()
        self.redraw()

    def _build_ui(self):
        top = tk.Frame(self.root, padx=8, pady=7)
        top.pack(fill="x")

        modes = [
            ("Wall", "WALL"),
            ("Erase wall", "ERASE"),
            ("Erase marker", "ERASE_MARKER"),
            ("Start", "START"),
            ("Pickup", "PICKUP"),
            ("Drop", "DROP"),
            ("Exit", "EXIT"),
        ]
        for text, value in modes:
            tk.Radiobutton(top, text=text, value=value, variable=self.mode, indicatoron=False, width=11).pack(side="left", padx=2)

        tk.Button(top, text="Preview Guide", command=self.preview_route).pack(side="left", padx=(10, 2))
        tk.Button(top, text="Clear Route", command=self.clear_route).pack(side="left", padx=2)
        tk.Button(top, text="Undo", command=self.undo).pack(side="left", padx=(10, 2))
        tk.Button(top, text="Redo", command=self.redo).pack(side="left", padx=2)
        tk.Button(top, text="Rotate Start Heading", command=self.rotate_start).pack(side="left", padx=(10, 2))

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
        tk.Button(second, textvariable=self.motion_toggle_text, command=self.toggle_motion_panel).pack(side="left", padx=(12, 2))

        gridbar = tk.Frame(self.root, padx=8, pady=3)
        gridbar.pack(fill="x")
        tk.Label(gridbar, text="Grid size:").pack(side="left", padx=(2, 4))
        tk.Label(gridbar, text="Rows").pack(side="left")
        self.rows_entry = tk.Entry(gridbar, width=5)
        self.rows_entry.insert(0, str(self.maze.rows))
        self.rows_entry.pack(side="left", padx=(3, 6))
        tk.Label(gridbar, text="×  Columns").pack(side="left")
        self.cols_entry = tk.Entry(gridbar, width=5)
        self.cols_entry.insert(0, str(self.maze.cols))
        self.cols_entry.pack(side="left", padx=(3, 6))
        tk.Button(gridbar, text="Resize Grid", command=self.apply_grid_size).pack(side="left", padx=(4, 8))
        tk.Label(
            gridbar,
            text="Coordinate origin: (0,0) = bottom-left   |   row ↑ north   |   column → east",
            fg="#555555",
        ).pack(side="left", padx=(14, 2))

        self.motion = tk.LabelFrame(self.root, text="Motion Profile / Front braking", padx=8, pady=5)
        tk.Label(self.motion, text="Cruise speed (m/s):").grid(row=0, column=0, sticky="w")
        tk.Entry(self.motion, textvariable=self.forward_speed_var, width=7).grid(row=0, column=1, padx=(3, 12))
        tk.Radiobutton(self.motion, text="AUTO", value="AUTO", variable=self.motion_mode_var, command=self._motion_mode_changed).grid(row=0, column=2, sticky="w")
        tk.Radiobutton(self.motion, text="MANUAL", value="MANUAL", variable=self.motion_mode_var, command=self._motion_mode_changed).grid(row=0, column=3, sticky="w", padx=(4, 12))
        tk.Button(self.motion, text="Calculate / Apply", command=self.apply_motion_settings).grid(row=0, column=4, padx=4)
        tk.Label(self.motion, textvariable=self.motion_result_var, anchor="w").grid(row=0, column=5, sticky="w", padx=(10, 0))

        self.manual_frame = tk.Frame(self.motion)
        self.manual_frame.grid(row=1, column=0, columnspan=7, sticky="w", pady=(4, 0))
        tk.Label(self.manual_frame, text="Manual slow trigger (cm):").pack(side="left")
        tk.Entry(self.manual_frame, textvariable=self.manual_slow_var, width=7).pack(side="left", padx=(3, 10))
        tk.Label(self.manual_frame, text="Brake/stop trigger (cm):").pack(side="left")
        tk.Entry(self.manual_frame, textvariable=self.manual_stop_var, width=7).pack(side="left", padx=(3, 10))

        self.auto_frame = tk.Frame(self.motion)
        self.auto_frame.grid(row=2, column=0, columnspan=7, sticky="w", pady=(4, 0))
        tk.Label(self.auto_frame, text="Auto model: reaction(s)").pack(side="left")
        tk.Entry(self.auto_frame, textvariable=self.auto_reaction_var, width=6).pack(side="left", padx=(3, 8))
        tk.Label(self.auto_frame, text="decel(m/s²)").pack(side="left")
        tk.Entry(self.auto_frame, textvariable=self.auto_decel_var, width=6).pack(side="left", padx=(3, 8))
        tk.Label(self.auto_frame, text="margin(cm)").pack(side="left")
        tk.Entry(self.auto_frame, textvariable=self.auto_margin_var, width=6).pack(side="left", padx=(3, 8))
        tk.Label(self.auto_frame, text="clearance(cm)").pack(side="left")
        tk.Entry(self.auto_frame, textvariable=self.auto_clearance_var, width=6).pack(side="left", padx=(3, 8))
        tk.Label(self.auto_frame, text="slow ramp(s)").pack(side="left")
        tk.Entry(self.auto_frame, textvariable=self.auto_ramp_var, width=6).pack(side="left", padx=(3, 8))
        self._motion_mode_changed()
        self.apply_motion_settings(show_errors=False, record_history=False)

        markerbar = tk.Frame(self.root, padx=8, pady=3)
        markerbar.pack(fill="x")
        tk.Checkbutton(markerbar, text="Fine marker placement", variable=self.fine_marker_mode).pack(side="left", padx=(2, 10))
        tk.Label(markerbar, text="Absolute X (m):").pack(side="left")
        self.marker_x_entry = tk.Entry(markerbar, width=8)
        self.marker_x_entry.pack(side="left", padx=(3, 8))
        tk.Label(markerbar, text="Y (m):").pack(side="left")
        self.marker_y_entry = tk.Entry(markerbar, width=8)
        self.marker_y_entry.pack(side="left", padx=(3, 8))
        tk.Button(markerbar, text="Set Absolute XY", command=self.apply_precise_marker_from_entries).pack(side="left", padx=(2, 8))
        tk.Label(markerbar, textvariable=self.marker_measure_text, fg="#245b8a").pack(side="left", padx=(8, 2))

        relativebar = tk.Frame(self.root, padx=8, pady=3)
        relativebar.pack(fill="x")
        tk.Label(relativebar, text="Marker in cell:").pack(side="left", padx=(2, 4))
        tk.Label(relativebar, text="row").pack(side="left")
        self.marker_row_entry = tk.Entry(relativebar, width=5)
        self.marker_row_entry.pack(side="left", padx=(2, 5))
        tk.Label(relativebar, text="col").pack(side="left")
        self.marker_col_entry = tk.Entry(relativebar, width=5)
        self.marker_col_entry.pack(side="left", padx=(2, 10))

        tk.Label(relativebar, text="Horizontal:").pack(side="left")
        tk.OptionMenu(relativebar, self.marker_x_ref, "LEFT", "RIGHT").pack(side="left", padx=(2, 2))
        self.marker_x_wall_entry = tk.Entry(relativebar, width=7)
        self.marker_x_wall_entry.pack(side="left", padx=(2, 2))
        tk.Label(relativebar, text="cm").pack(side="left", padx=(0, 8))

        tk.Label(relativebar, text="Vertical:").pack(side="left")
        tk.OptionMenu(relativebar, self.marker_y_ref, "BOTTOM", "TOP").pack(side="left", padx=(2, 2))
        self.marker_y_wall_entry = tk.Entry(relativebar, width=7)
        self.marker_y_wall_entry.pack(side="left", padx=(2, 2))
        tk.Label(relativebar, text="cm").pack(side="left", padx=(0, 8))
        tk.Button(relativebar, text="Apply Cell Position", command=self.apply_marker_from_cell_clearance).pack(side="left", padx=(2, 8))
        tk.Label(relativebar, text="Example: cell=1.0 m, LEFT 40 cm + TOP 40 cm", fg="#555555").pack(side="left")

        info = tk.Label(
            self.root,
            text=(
                "Mouse: LEFT near edge = add wall, RIGHT near edge = remove wall; RIGHT on S/P/D/E = erase marker.  "
                "PICKUP/DROP/EXIT are optional; START-only can preview/export the full topology/DFS guide.  "
                "Ctrl+Z Undo, Ctrl+Y/Ctrl+Shift+Z Redo.  Coordinates: (0,0) bottom-left.  "
                "Fine placement supports exact click/absolute X-Y or LEFT/RIGHT + TOP/BOTTOM wall distance."
            ),
            anchor="w",
            padx=10,
            wraplength=1200,
            justify="left",
        )
        info.pack(fill="x")

        maze_frame = tk.Frame(self.root)
        maze_frame.pack(fill="both", expand=True, padx=8, pady=6)

        self.canvas = tk.Canvas(maze_frame, bg="#f5f5f5", highlightthickness=0)
        self.canvas.pack(side="left", fill="both", expand=True)
        self.canvas.bind("<Button-1>", self.on_click)
        self.canvas.bind("<Button-3>", self.on_right_click)

        self.canvas_scrollbar = tk.Scrollbar(maze_frame, orient="vertical", command=self.canvas.yview)
        self.canvas_scrollbar.pack(side="right", fill="y")
        self.canvas.configure(yscrollcommand=self.canvas_scrollbar.set)
        self.canvas.bind_all("<MouseWheel>", self._on_mousewheel)

        tk.Label(self.root, textvariable=self.status, anchor="w", relief="sunken", padx=8).pack(fill="x", side="bottom")
        self._sync_motion_panel_visibility()

    def _resize_canvas(self):
        full_w = 2 * self.PAD + self.maze.cols * self.CELL_PX
        full_h = 2 * self.PAD + self.maze.rows * self.CELL_PX
        viewport_w = min(full_w, 1200)
        viewport_h = min(full_h, 640)
        self.canvas.config(width=viewport_w, height=viewport_h, scrollregion=(0, 0, full_w, full_h))
        self.root.minsize(min(viewport_w + 60, 1300), 620)

    def _on_mousewheel(self, event):
        if getattr(self, 'canvas', None) is None:
            return
        self.canvas.yview_scroll(int(-1 * (event.delta / 120)), 'units')

    def toggle_motion_panel(self):
        self.motion_panel_visible.set(not self.motion_panel_visible.get())
        self._sync_motion_panel_visibility()

    def _sync_motion_panel_visibility(self):
        if not hasattr(self, 'motion'):
            return
        if self.motion_panel_visible.get():
            self.motion.pack(fill='x', padx=8, pady=(3, 2), after=self.root.winfo_children()[2])
            self.motion_toggle_text.set('Hide Motion Profile')
        else:
            self.motion.pack_forget()
            self.motion_toggle_text.set('Show Motion Profile')

    def marker_attr_names(self, mode: str) -> Tuple[Optional[str], Optional[str]]:
        mapping = {
            "START": ("start", "start_pos_m"),
            "PICKUP": ("object_cell", "object_pos_m"),
            "DROP": ("drop_cell", "drop_pos_m"),
            "EXIT": ("goal", "goal_pos_m"),
        }
        return mapping.get(mode, (None, None))

    def marker_label(self, mode: str) -> str:
        return {"START": "START", "PICKUP": "PICKUP", "DROP": "DROP", "EXIT": "EXIT"}.get(mode, mode)

    def mode_supports_marker(self, mode: str) -> bool:
        return mode in ("START", "PICKUP", "DROP", "EXIT")

    def update_marker_entry_fields(self, pos: Optional[PosM], cell: Optional[Cell] = None) -> None:
        self.marker_x_entry.delete(0, tk.END)
        self.marker_y_entry.delete(0, tk.END)
        self.marker_row_entry.delete(0, tk.END)
        self.marker_col_entry.delete(0, tk.END)
        self.marker_x_wall_entry.delete(0, tk.END)
        self.marker_y_wall_entry.delete(0, tk.END)
        if pos is None:
            self.marker_measure_text.set("Select a marker or click inside a cell")
            return
        if cell is None:
            cell = self.maze.pos_to_cell(pos)
        self.marker_x_entry.insert(0, f"{pos[0]:.3f}")
        self.marker_y_entry.insert(0, f"{pos[1]:.3f}")
        self.marker_row_entry.insert(0, str(cell[0]))
        self.marker_col_entry.insert(0, str(cell[1]))
        self._fill_wall_distance_entries(cell, pos)

    def _fill_wall_distance_entries(self, cell: Cell, pos: PosM) -> None:
        clear = self.maze.wall_clearances_m(cell, pos)
        x_key = "left" if self.marker_x_ref.get() == "LEFT" else "right"
        y_key = "bottom" if self.marker_y_ref.get() == "BOTTOM" else "top"
        self.marker_x_wall_entry.delete(0, tk.END)
        self.marker_y_wall_entry.delete(0, tk.END)
        self.marker_x_wall_entry.insert(0, f"{clear[x_key] * 100.0:.1f}")
        self.marker_y_wall_entry.insert(0, f"{clear[y_key] * 100.0:.1f}")
        self.marker_measure_text.set(
            f"cell {cell} | L {clear['left']*100:.1f}  R {clear['right']*100:.1f}  "
            f"B {clear['bottom']*100:.1f}  T {clear['top']*100:.1f} cm"
        )

    def refresh_relative_marker_fields(self) -> None:
        if not hasattr(self, "marker_x_wall_entry"):
            return
        mode = self.mode.get()
        if not self.mode_supports_marker(mode):
            return
        cell_attr, pos_attr = self.marker_attr_names(mode)
        cell = getattr(self.maze, cell_attr)
        pos = getattr(self.maze, pos_attr)
        if cell is not None and pos is not None:
            self._fill_wall_distance_entries(cell, pos)
            self.redraw()

    def set_marker(self, mode: str, cell: Cell, pos_m: Optional[PosM] = None) -> None:
        cell_attr, pos_attr = self.marker_attr_names(mode)
        if cell_attr is None:
            return
        if pos_m is None:
            pos_m = self.maze.cell_center_pos_m(cell)
        else:
            pos_m = self.maze.clamp_pos_m(pos_m)
            cell = self.maze.pos_to_cell(pos_m)
        setattr(self.maze, cell_attr, cell)
        setattr(self.maze, pos_attr, pos_m)
        self.update_marker_entry_fields(pos_m, cell)

    def get_marker_pos(self, mode: str) -> Optional[PosM]:
        _, pos_attr = self.marker_attr_names(mode)
        if pos_attr is None:
            return None
        return getattr(self.maze, pos_attr)

    def describe_marker(self, mode: str) -> str:
        cell_attr, pos_attr = self.marker_attr_names(mode)
        if cell_attr is None:
            return ""
        cell = getattr(self.maze, cell_attr)
        pos = getattr(self.maze, pos_attr)
        if cell is None:
            return f"{self.marker_label(mode)} cleared"
        if pos is None:
            return f"{self.marker_label(mode)} = {cell}"
        clear = self.maze.wall_clearances_m(cell, pos)
        return (
            f"{self.marker_label(mode)} = {cell} | world=({pos[0]:.3f}, {pos[1]:.3f}) m | "
            f"L/R/B/T={clear['left']*100:.1f}/{clear['right']*100:.1f}/"
            f"{clear['bottom']*100:.1f}/{clear['top']*100:.1f} cm"
        )

    def current_mode_changed_marker(self):
        mode = self.mode.get()
        if self.mode_supports_marker(mode):
            cell_attr, pos_attr = self.marker_attr_names(mode)
            self.update_marker_entry_fields(getattr(self.maze, pos_attr), getattr(self.maze, cell_attr))
        elif mode == "ERASE_MARKER":
            self.marker_measure_text.set("Erase Marker: left-click a marker; right-click also erases markers")
        else:
            self.marker_measure_text.set("Select START/PICKUP/DROP/EXIT for fine placement")
        self.redraw()

    def _motion_mode_changed(self):
        mode = self.motion_mode_var.get().upper()
        for child in self.manual_frame.winfo_children():
            try:
                child.configure(state=("normal" if mode == "MANUAL" else "disabled"))
            except tk.TclError:
                pass
        for child in self.auto_frame.winfo_children():
            try:
                child.configure(state=("normal" if mode == "AUTO" else "disabled"))
            except tk.TclError:
                pass
        self.guide_edges = []
        self.route_plan = None

    def _sync_motion_vars_from_maze(self):
        self.motion_mode_var.set(self.maze.motion_mode)
        self.forward_speed_var.set(f"{self.maze.forward_speed_mps:.2f}")
        self.manual_slow_var.set(f"{self.maze.manual_slow_front_cm:.1f}")
        self.manual_stop_var.set(f"{self.maze.manual_stop_front_cm:.1f}")
        self.auto_reaction_var.set(f"{self.maze.auto_reaction_time_s:.2f}")
        self.auto_decel_var.set(f"{self.maze.auto_brake_decel_mps2:.2f}")
        self.auto_margin_var.set(f"{self.maze.auto_safety_margin_cm:.1f}")
        self.auto_clearance_var.set(f"{self.maze.auto_clearance_cm:.1f}")
        self.auto_ramp_var.set(f"{self.maze.auto_slow_ramp_time_s:.2f}")
        self._motion_mode_changed()
        try:
            profile = calculate_motion_profile(self.maze)
            self.motion_result_var.set(
                f"{profile['mode']}: slow @ {profile['slow_front_cm']:.1f} cm | "
                f"brake/stop @ {profile['stop_front_cm']:.1f} cm"
            )
        except Exception:
            self.motion_result_var.set("Invalid motion settings")

    def apply_motion_settings(self, show_errors: bool = True, record_history: bool = True) -> bool:
        try:
            values = {
                "motion_mode": self.motion_mode_var.get().upper(),
                "forward_speed_mps": float(self.forward_speed_var.get()),
                "manual_slow_front_cm": float(self.manual_slow_var.get()),
                "manual_stop_front_cm": float(self.manual_stop_var.get()),
                "auto_reaction_time_s": float(self.auto_reaction_var.get()),
                "auto_brake_decel_mps2": float(self.auto_decel_var.get()),
                "auto_safety_margin_cm": float(self.auto_margin_var.get()),
                "auto_clearance_cm": float(self.auto_clearance_var.get()),
                "auto_slow_ramp_time_s": float(self.auto_ramp_var.get()),
            }
            # Validate using a temporary detached MazeData so invalid text never
            # partially mutates the current design.
            temp_data = self.maze.to_dict()
            temp_motion = dict(temp_data.get("motion_settings") or {})
            temp_motion.update({
                "mode": values["motion_mode"],
                "forward_speed_mps": values["forward_speed_mps"],
                "manual_slow_front_cm": values["manual_slow_front_cm"],
                "manual_stop_front_cm": values["manual_stop_front_cm"],
                "auto_reaction_time_s": values["auto_reaction_time_s"],
                "auto_brake_decel_mps2": values["auto_brake_decel_mps2"],
                "auto_safety_margin_cm": values["auto_safety_margin_cm"],
                "auto_clearance_cm": values["auto_clearance_cm"],
                "auto_slow_ramp_time_s": values["auto_slow_ramp_time_s"],
            })
            temp_data["motion_settings"] = temp_motion
            temp_maze = MazeData.from_dict(temp_data)
            profile = calculate_motion_profile(temp_maze)
        except (ValueError, TypeError) as exc:
            self.motion_result_var.set("Invalid motion settings")
            if show_errors:
                messagebox.showerror("Invalid motion profile", str(exc))
            return False

        changed = any(getattr(self.maze, key) != value for key, value in values.items())
        if changed and record_history:
            self._push_undo()
        for key, value in values.items():
            setattr(self.maze, key, value)
        self.maze.motion_mode = str(self.maze.motion_mode).upper()
        self.motion_result_var.set(
            f"{profile['mode']}: slow @ {profile['slow_front_cm']:.1f} cm | "
            f"brake/stop @ {profile['stop_front_cm']:.1f} cm"
        )
        self.status.set(
            f"Motion {profile['mode']}: v={profile['forward_speed_mps']:.2f} m/s, "
            f"slow={profile['slow_front_cm']:.1f} cm, stop={profile['stop_front_cm']:.1f} cm"
        )
        self.guide_edges = []
        self.route_plan = None
        return True

    # ------------------------------------------------------------------
    # Edit history / marker deletion
    # ------------------------------------------------------------------
    def _snapshot_maze(self) -> dict:
        # JSON round-trip gives a detached, mutation-safe plain-data snapshot.
        return json.loads(json.dumps(self.maze.to_dict()))

    def _push_undo(self) -> None:
        self.undo_stack.append(self._snapshot_maze())
        if len(self.undo_stack) > self.history_limit:
            del self.undo_stack[0]
        self.redo_stack.clear()

    def _sync_ui_from_maze(self) -> None:
        self.cell_size_entry.delete(0, tk.END)
        self.cell_size_entry.insert(0, str(self.maze.cell_size_m))
        self.rows_entry.delete(0, tk.END)
        self.rows_entry.insert(0, str(self.maze.rows))
        self.cols_entry.delete(0, tk.END)
        self.cols_entry.insert(0, str(self.maze.cols))
        self._sync_motion_vars_from_maze()
        self.route = []
        self.guide_edges = []
        self.route_plan = None
        self._resize_canvas()
        self.current_mode_changed_marker()
        self.redraw()

    def _restore_snapshot(self, snapshot: dict) -> None:
        self.maze = MazeData.from_dict(snapshot)
        self._sync_ui_from_maze()

    def undo(self, event=None):
        if not self.undo_stack:
            self.status.set("Nothing to undo")
            return "break"
        self.redo_stack.append(self._snapshot_maze())
        snapshot = self.undo_stack.pop()
        self._restore_snapshot(snapshot)
        self.status.set(f"Undo | {len(self.undo_stack)} earlier edit(s) available")
        return "break"

    def redo(self, event=None):
        if not self.redo_stack:
            self.status.set("Nothing to redo")
            return "break"
        self.undo_stack.append(self._snapshot_maze())
        snapshot = self.redo_stack.pop()
        self._restore_snapshot(snapshot)
        self.status.set(f"Redo | {len(self.redo_stack)} later edit(s) available")
        return "break"

    def clear_marker(self, mode: str, record_history: bool = True) -> bool:
        if not self.mode_supports_marker(mode):
            return False
        cell_attr, pos_attr = self.marker_attr_names(mode)
        if getattr(self.maze, cell_attr) is None and getattr(self.maze, pos_attr) is None:
            return False
        if record_history:
            self._push_undo()
        setattr(self.maze, cell_attr, None)
        setattr(self.maze, pos_attr, None)
        self.route = []
        self.guide_edges = []
        self.route_plan = None
        if self.mode.get() == mode:
            self.update_marker_entry_fields(None)
        self.redraw()
        self.status.set(f"{self.marker_label(mode)} erased")
        return True

    def delete_selected_marker(self, event=None):
        mode = self.mode.get()
        if self.mode_supports_marker(mode):
            self.clear_marker(mode, record_history=True)
        return "break"

    def _nearest_marker_mode_at_canvas(self, x: float, y: float, max_distance_px: float = 24.0) -> Optional[str]:
        best_mode = None
        best_d2 = max_distance_px * max_distance_px
        for mode in ("START", "PICKUP", "DROP", "EXIT"):
            cell_attr, pos_attr = self.marker_attr_names(mode)
            cell = getattr(self.maze, cell_attr)
            pos = getattr(self.maze, pos_attr)
            if cell is None:
                continue
            if pos is None:
                x0, y0, x1, y1 = self.cell_rect(cell)
                mx, my = (x0 + x1) / 2.0, (y0 + y1) / 2.0
            else:
                mx, my = self.world_pos_m_to_canvas(pos)
            d2 = (float(x) - mx) ** 2 + (float(y) - my) ** 2
            if d2 <= best_d2:
                best_d2 = d2
                best_mode = mode
        return best_mode

    def erase_marker_at_canvas(self, x: float, y: float) -> bool:
        marker_mode = self._nearest_marker_mode_at_canvas(x, y)
        if marker_mode is None:
            self.status.set("No marker under cursor")
            return False
        return self.clear_marker(marker_mode, record_history=True)

    def new_maze(self):
        if not messagebox.askyesno("New maze", "Clear the current maze?"):
            return
        self._push_undo()
        old = self.maze
        self.maze = MazeData(
            rows=old.rows, cols=old.cols, cell_size_m=old.cell_size_m,
            motion_mode=old.motion_mode, forward_speed_mps=old.forward_speed_mps,
            manual_slow_front_cm=old.manual_slow_front_cm, manual_stop_front_cm=old.manual_stop_front_cm,
            auto_reaction_time_s=old.auto_reaction_time_s, auto_brake_decel_mps2=old.auto_brake_decel_mps2,
            auto_safety_margin_cm=old.auto_safety_margin_cm, auto_clearance_cm=old.auto_clearance_cm,
            auto_slow_ramp_time_s=old.auto_slow_ramp_time_s,
        )
        self._sync_motion_vars_from_maze()
        self.current_file = None
        self.update_marker_entry_fields(None)
        self.clear_route()
        self.redraw()

    def apply_grid_size(self):
        try:
            new_rows = int(self.rows_entry.get())
            new_cols = int(self.cols_entry.get())
            if not (2 <= new_rows <= 50 and 2 <= new_cols <= 50):
                raise ValueError
        except ValueError:
            messagebox.showerror(
                "Invalid grid size",
                "Rows and Columns must be whole numbers from 2 to 50.",
            )
            return False

        old_rows, old_cols = self.maze.rows, self.maze.cols
        if (new_rows, new_cols) == (old_rows, old_cols):
            self.status.set(f"Grid size unchanged: {new_rows} × {new_cols}")
            return True

        def fits(cell):
            return cell is not None and 0 <= cell[0] < new_rows and 0 <= cell[1] < new_cols

        width_new = new_cols * self.maze.cell_size_m
        height_new = new_rows * self.maze.cell_size_m
        def fits_pos(pos):
            return pos is not None and 0.0 <= pos[0] < width_new and 0.0 <= pos[1] < height_new

        removed_walls = sum(1 for a, b in self.maze.walls if not (fits(a) and fits(b)))
        removed_markers = []
        for name, cell, pos in (
            ("START", self.maze.start, self.maze.start_pos_m),
            ("PICKUP", self.maze.object_cell, self.maze.object_pos_m),
            ("DROP", self.maze.drop_cell, self.maze.drop_pos_m),
            ("EXIT", self.maze.goal, self.maze.goal_pos_m),
        ):
            if (cell is not None and not fits(cell)) or (pos is not None and not fits_pos(pos)):
                removed_markers.append(name)

        if removed_walls or removed_markers:
            details = []
            if removed_walls:
                details.append(f"{removed_walls} wall segment(s)")
            if removed_markers:
                details.append("marker(s): " + ", ".join(removed_markers))
            if not messagebox.askyesno(
                "Resize will crop maze",
                "The new grid is smaller and will remove " + " and ".join(details) + "\n\nContinue?",
            ):
                self.rows_entry.delete(0, tk.END)
                self.rows_entry.insert(0, str(old_rows))
                self.cols_entry.delete(0, tk.END)
                self.cols_entry.insert(0, str(old_cols))
                return False

        self._push_undo()
        self.maze.rows = new_rows
        self.maze.cols = new_cols
        self.maze.walls = {self.maze.canonical_edge(a, b) for a, b in self.maze.walls if fits(a) and fits(b)}
        self.maze._sync_all_marker_pose_and_cells()
        for mode in ("START", "PICKUP", "DROP", "EXIT"):
            cell_attr, pos_attr = self.marker_attr_names(mode)
            cell = getattr(self.maze, cell_attr)
            pos = getattr(self.maze, pos_attr)
            if (cell is not None and not fits(cell)) or (pos is not None and not fits_pos(pos)):
                setattr(self.maze, cell_attr, None)
                setattr(self.maze, pos_attr, None)
        self.maze._sync_all_marker_pose_and_cells()

        self.route = []
        self.guide_edges = []
        self.route_plan = None
        self._resize_canvas()
        self.redraw()
        self.status.set(
            f"Grid resized: {old_rows}×{old_cols} -> {new_rows}×{new_cols}; origin stays at bottom-left (0,0)"
        )
        self.current_mode_changed_marker()
        return True

    def apply_cell_size(self):
        try:
            value = float(self.cell_size_entry.get())
            if not 0.05 <= value <= 5.0:
                raise ValueError
        except ValueError:
            messagebox.showerror("Invalid cell size", "Enter a value between 0.05 and 5.0 metres")
            return
        if abs(value - self.maze.cell_size_m) < 1e-12:
            self.status.set(f"Cell size unchanged: {value:.3f} m")
            return True
        self._push_undo()
        self.maze.cell_size_m = value
        self.maze._sync_all_marker_pose_and_cells()
        self.current_mode_changed_marker()
        self.status.set(f"Cell size = {value:.3f} m")
        self.guide_edges = []
        self.route_plan = None
        self.redraw()

    def clear_route(self):
        self.route = []
        self.guide_edges = []
        self.route_plan = None
        self.redraw()

    def rotate_start(self):
        self._push_undo()
        i = HEADINGS.index(self.maze.start_heading)
        self.maze.start_heading = HEADINGS[(i + 1) % 4]
        self.guide_edges = []
        self.route_plan = None
        self.status.set(f"Start heading = {self.maze.start_heading}")
        self.redraw()

    def cell_rect(self, cell: Cell) -> Tuple[float, float, float, float]:
        r, c = cell
        x0 = self.PAD + c * self.CELL_PX
        screen_row = self.maze.rows - 1 - r
        y0 = self.PAD + screen_row * self.CELL_PX
        return x0, y0, x0 + self.CELL_PX, y0 + self.CELL_PX

    def event_to_cell(self, x: float, y: float) -> Optional[Cell]:
        c = int((x - self.PAD) // self.CELL_PX)
        screen_row = int((y - self.PAD) // self.CELL_PX)
        r = self.maze.rows - 1 - screen_row
        cell = (r, c)
        return cell if self.maze.in_bounds(cell) else None

    def event_to_world_pos_m(self, x: float, y: float) -> Optional[PosM]:
        if self.event_to_cell(x, y) is None:
            return None
        x_px = x - self.PAD
        y_px = y - self.PAD
        width_px = self.maze.cols * self.CELL_PX
        height_px = self.maze.rows * self.CELL_PX
        x_m = (x_px / width_px) * (self.maze.cols * self.maze.cell_size_m)
        y_m = ((height_px - y_px) / height_px) * (self.maze.rows * self.maze.cell_size_m)
        return self.maze.clamp_pos_m((x_m, y_m))

    def world_pos_m_to_canvas(self, pos_m: PosM) -> Tuple[float, float]:
        x_m, y_m = self.maze.clamp_pos_m(pos_m)
        total_w_m, total_h_m = self.maze.world_size_m()
        x = self.PAD + (x_m / total_w_m) * (self.maze.cols * self.CELL_PX) if total_w_m > 0 else self.PAD
        y = self.PAD + ((total_h_m - y_m) / total_h_m) * (self.maze.rows * self.CELL_PX) if total_h_m > 0 else self.PAD
        return x, y

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

    def apply_precise_marker_from_entries(self):
        mode = self.mode.get()
        if not self.mode_supports_marker(mode):
            messagebox.showinfo("Choose a marker mode", "Switch mode to START, PICKUP, DROP or EXIT first.")
            return
        try:
            x_m = float(self.marker_x_entry.get())
            y_m = float(self.marker_y_entry.get())
        except ValueError:
            messagebox.showerror("Invalid marker position", "Enter numeric X and Y values in metres.")
            return
        pos_m = self.maze.clamp_pos_m((x_m, y_m))
        cell = self.maze.pos_to_cell(pos_m)
        self._push_undo()
        self.set_marker(mode, cell, pos_m)
        self.route = []
        self.guide_edges = []
        self.route_plan = None
        self.redraw()
        self.status.set(self.describe_marker(mode))

    def apply_marker_from_cell_clearance(self):
        """Place current marker using exact distances from two cell walls.

        Horizontal reference is LEFT or RIGHT. Vertical reference is BOTTOM or TOP.
        Example for a 1 m cell: LEFT=40 cm and TOP=40 cm gives local
        (x=0.40 m, y=0.60 m) automatically.
        """
        mode = self.mode.get()
        if not self.mode_supports_marker(mode):
            messagebox.showinfo("Choose a marker mode", "Switch mode to START, PICKUP, DROP or EXIT first.")
            return
        try:
            row = int(self.marker_row_entry.get())
            col = int(self.marker_col_entry.get())
            x_dist_cm = float(self.marker_x_wall_entry.get())
            y_dist_cm = float(self.marker_y_wall_entry.get())
        except ValueError:
            messagebox.showerror(
                "Invalid cell placement",
                "Enter whole-number row/column and numeric wall distances in cm.",
            )
            return

        cell = (row, col)
        if not self.maze.in_bounds(cell):
            messagebox.showerror("Cell out of bounds", f"Cell {cell} is outside {self.maze.rows}×{self.maze.cols}.")
            return
        cell_cm = self.maze.cell_size_m * 100.0
        if not (0.0 <= x_dist_cm <= cell_cm and 0.0 <= y_dist_cm <= cell_cm):
            messagebox.showerror(
                "Distance outside cell",
                f"For a {self.maze.cell_size_m:.3f} m cell, each wall distance must be 0–{cell_cm:.1f} cm.",
            )
            return

        x_dist_m = x_dist_cm / 100.0
        y_dist_m = y_dist_cm / 100.0
        if self.marker_x_ref.get() == "LEFT":
            local_x_m = x_dist_m
        else:
            local_x_m = self.maze.cell_size_m - x_dist_m
        if self.marker_y_ref.get() == "BOTTOM":
            local_y_m = y_dist_m
        else:
            local_y_m = self.maze.cell_size_m - y_dist_m

        pos_m = self.maze.cell_local_to_world_m(cell, local_x_m, local_y_m)
        self._push_undo()
        self.set_marker(mode, cell, pos_m)
        self.route = []
        self.guide_edges = []
        self.route_plan = None
        self.redraw()
        self.status.set(self.describe_marker(mode))

    def on_click(self, event):
        """Primary mouse action.

        WALL mode intentionally behaves like a drawing program: left-click adds
        a wall. ERASE keeps the old left-click erase workflow for compatibility.
        Marker modes place/move the selected marker. ERASE_MARKER removes a
        marker with left-click. Every edit participates in Ctrl+Z history.
        """
        cell = self.event_to_cell(event.x, event.y)
        if cell is None:
            return
        mode = self.mode.get()

        if mode in ("WALL", "ERASE"):
            edge = self.nearest_internal_edge(cell, event.x, event.y)
            if edge is None:
                self.status.set("Click closer to an INTERNAL cell edge to edit a wall")
                return
            canonical = self.maze.canonical_edge(edge[0], edge[1])
            want_present = mode == "WALL"
            already_present = canonical in self.maze.walls
            if already_present == want_present:
                self.status.set("Wall already present" if want_present else "No wall on that edge")
                return
            self._push_undo()
            self.maze.set_wall_between(edge[0], edge[1], want_present)
            self.status.set(f"{'Wall added' if want_present else 'Wall erased'}: {edge[0]} <-> {edge[1]}")

        elif mode == "ERASE_MARKER":
            if not self.erase_marker_at_canvas(event.x, event.y):
                return

        elif self.mode_supports_marker(mode):
            self._push_undo()
            if self.fine_marker_mode.get():
                pos_m = self.event_to_world_pos_m(event.x, event.y)
                self.set_marker(mode, cell, pos_m)
            else:
                self.set_marker(mode, cell, self.maze.cell_center_pos_m(cell))
            self.status.set(self.describe_marker(mode))

        self.route = []
        self.guide_edges = []
        self.route_plan = None
        self.redraw()

    def on_right_click(self, event):
        """Context erase: right-click wall to remove it, or right-click marker.

        Wall hit-testing is tried first only when the click is close to an
        INTERNAL edge *and that edge actually contains a wall*. Otherwise the
        click is treated as a marker erase. This makes right-click safe in the
        middle of a cell.
        """
        cell = self.event_to_cell(event.x, event.y)
        if cell is None:
            return "break"

        edge = self.nearest_internal_edge(cell, event.x, event.y)
        if edge is not None:
            canonical = self.maze.canonical_edge(edge[0], edge[1])
            if canonical in self.maze.walls:
                self._push_undo()
                self.maze.set_wall_between(edge[0], edge[1], False)
                self.route = []
                self.guide_edges = []
                self.route_plan = None
                self.redraw()
                self.status.set(f"Wall erased (right-click): {edge[0]} <-> {edge[1]}")
                return "break"

        self.erase_marker_at_canvas(event.x, event.y)
        return "break"

    def preview_route(self):
        self.apply_cell_size()
        if not self.apply_motion_settings(show_errors=True, record_history=True):
            return
        try:
            plan = validate_and_plan(self.maze)
        except ValueError as exc:
            messagebox.showerror("Route error", str(exc))
            return
        self.route_plan = plan
        self.route = [tuple(c) for c in plan["path"]]
        self.guide_edges = topology_preview_edges(plan)
        warning = " | ".join(plan["warnings"])
        mission = " -> ".join(["START"] + list(plan.get("mission_order") or []))
        if plan["mission_mode"] == "MAPPING_ONLY":
            mission = "START -> runtime DFS/topology exploration (no PICKUP required)"
        elif plan["mission_mode"] == "START_WITH_OBJECT":
            mission += "  [starts with object; no PICKUP required]"
        self.status.set(
            f"Guide OK [{plan['mission_mode']}]: {mission} | "
            f"{len(plan['topology_guide']['nodes'])} recognisable nodes, "
            f"{plan['total_cells']} mission cells (distance hint only)" + (f" | WARNING: {warning}" if warning else "")
        )
        self.redraw()
        if warning:
            messagebox.showwarning("Route warning", warning)

    def save_maze_dialog(self):
        self.apply_cell_size()
        if not self.apply_motion_settings(show_errors=True, record_history=True):
            return
        path = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=[("Maze JSON", "*.json"), ("All files", "*.*")],
            initialfile=os.path.basename(self.current_file) if self.current_file else "known_maze.json",
        )
        if not path:
            return
        save_maze(path, self.maze)
        self.current_file = path
        self.status.set(f"Saved maze + embedded runtime guide: {path}" if self.maze.start is not None else f"Saved maze (START not set; guide not ready): {path}")

    def load_maze_dialog(self):
        path = filedialog.askopenfilename(filetypes=[("Maze JSON", "*.json"), ("All files", "*.*")])
        if not path:
            return
        try:
            loaded_maze = load_maze(path)
        except Exception as exc:
            messagebox.showerror("Load error", str(exc))
            return
        self._push_undo()
        self.maze = loaded_maze
        self.current_file = path
        self.cell_size_entry.delete(0, tk.END)
        self.cell_size_entry.insert(0, str(self.maze.cell_size_m))
        self.rows_entry.delete(0, tk.END)
        self.rows_entry.insert(0, str(self.maze.rows))
        self.cols_entry.delete(0, tk.END)
        self.cols_entry.insert(0, str(self.maze.cols))
        self._sync_motion_vars_from_maze()
        self.current_mode_changed_marker()
        self.route = []
        self.guide_edges = []
        self.route_plan = None
        self._resize_canvas()
        self.redraw()
        self.status.set(f"Loaded maze: {path}")

    def export_route_dialog(self):
        self.apply_cell_size()
        if not self.apply_motion_settings(show_errors=True, record_history=True):
            return
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
        self.guide_edges = topology_preview_edges(plan)
        self.redraw()
        self.status.set(f"Exported robot route [{plan['mission_mode']}]: {path}")

    def redraw(self):
        c = self.canvas
        c.delete("all")

        # Preview the complete topology guide even when there is no mission
        # target. This is the key behaviour for START-only / no-PICKUP maps.
        for a, b in self.guide_edges:
            ax0, ay0, ax1, ay1 = self.cell_rect(a)
            bx0, by0, bx1, by1 = self.cell_rect(b)
            c.create_line(
                (ax0 + ax1) / 2, (ay0 + ay1) / 2,
                (bx0 + bx1) / 2, (by0 + by1) / 2,
                width=3, fill="#a6a6a6", dash=(7, 5),
                capstyle=tk.ROUND, joinstyle=tk.ROUND,
            )

        # Mission path, when present, is drawn thicker on top of the soft guide.
        if len(self.route) >= 2:
            pts = []
            for cell in self.route:
                x0, y0, x1, y1 = self.cell_rect(cell)
                pts.extend(((x0 + x1) / 2, (y0 + y1) / 2))
            c.create_line(*pts, width=6, fill="#6a5acd", smooth=False, capstyle=tk.ROUND, joinstyle=tk.ROUND)

        for r in range(self.maze.rows):
            for col in range(self.maze.cols):
                cell = (r, col)
                x0, y0, x1, y1 = self.cell_rect(cell)
                c.create_rectangle(x0, y0, x1, y1, fill="#ffffff", outline="#dedede")
                c.create_text(x0 + 5, y0 + 5, text=f"{r},{col}", anchor="nw", fill="#aaaaaa", font=("Arial", 7))

        if len(self.route) >= 2:
            pts = []
            for cell in self.route:
                x0, y0, x1, y1 = self.cell_rect(cell)
                pts.extend(((x0 + x1) / 2, (y0 + y1) / 2))
            c.create_line(*pts, width=5, fill="#6a5acd", capstyle=tk.ROUND, joinstyle=tk.ROUND)

        left = self.PAD
        top = self.PAD
        right = self.PAD + self.maze.cols * self.CELL_PX
        bottom = self.PAD + self.maze.rows * self.CELL_PX
        c.create_rectangle(left, top, right, bottom, width=4, outline="#111111")

        for a, b in self.maze.walls:
            ar, ac = a
            br, bc = b
            ax0, ay0, ax1, ay1 = self.cell_rect(a)
            if ar == br:
                x = ax1 if bc > ac else ax0
                c.create_line(x, ay0, x, ay1, width=5, fill="#111111")
            else:
                y = ay0 if br > ar else ay1
                c.create_line(ax0, y, ax1, y, width=5, fill="#111111")

        if self.maze.start is not None:
            self._draw_marker(self.maze.start, self.maze.start_pos_m, "S", "#2e8b57")
            self._draw_heading_arrow_from_pos(self.maze.start_pos_m or self.maze.cell_center_pos_m(self.maze.start), self.maze.start_heading)
        if self.maze.object_cell is not None:
            self._draw_marker(self.maze.object_cell, self.maze.object_pos_m, "P", "#ff8c00")
        if self.maze.drop_cell is not None:
            self._draw_marker(self.maze.drop_cell, self.maze.drop_pos_m, "D", "#1e90ff")
        if self.maze.goal is not None:
            self._draw_marker(self.maze.goal, self.maze.goal_pos_m, "E", "#b22222")

        # Show exact wall clearances for the currently selected marker, similar
        # to a dimension drawing. This makes placements such as LEFT 40 cm +
        # TOP 40 cm visually checkable without doing manual coordinate math.
        mode = self.mode.get()
        if self.mode_supports_marker(mode):
            cell_attr, pos_attr = self.marker_attr_names(mode)
            cell = getattr(self.maze, cell_attr)
            pos = getattr(self.maze, pos_attr)
            if cell is not None and pos is not None:
                self._draw_marker_measurements(cell, pos)

    def _draw_marker_measurements(self, cell: Cell, pos_m: PosM):
        cx, cy = self.world_pos_m_to_canvas(pos_m)
        x0, y0, x1, y1 = self.cell_rect(cell)
        clear = self.maze.wall_clearances_m(cell, pos_m)
        color = "#2f6fa7"

        if self.marker_x_ref.get() == "RIGHT":
            hx = x1
            hdist = clear["right"]
        else:
            hx = x0
            hdist = clear["left"]
        self.canvas.create_line(hx, cy, cx, cy, fill=color, width=2, dash=(4, 2), arrow=tk.BOTH)
        self.canvas.create_text(
            (hx + cx) / 2, cy + 11, text=f"{hdist*100:.1f} cm", fill=color, font=("Arial", 8, "bold")
        )

        if self.marker_y_ref.get() == "BOTTOM":
            vy = y1
            vdist = clear["bottom"]
        else:
            vy = y0
            vdist = clear["top"]
        self.canvas.create_line(cx, vy, cx, cy, fill=color, width=2, dash=(4, 2), arrow=tk.BOTH)
        self.canvas.create_text(
            cx + 24, (vy + cy) / 2, text=f"{vdist*100:.1f} cm", fill=color, font=("Arial", 8, "bold")
        )

    def _draw_marker(self, cell: Cell, pos_m: Optional[PosM], text: str, color: str):
        if pos_m is not None:
            cx, cy = self.world_pos_m_to_canvas(pos_m)
        else:
            x0, y0, x1, y1 = self.cell_rect(cell)
            cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        rad = self.CELL_PX * 0.22
        self.canvas.create_oval(cx - rad, cy - rad, cx + rad, cy + rad, fill=color, outline="")
        self.canvas.create_text(cx, cy, text=text, fill="white", font=("Arial", 13, "bold"))
        x0, y0, x1, y1 = self.cell_rect(cell)
        self.canvas.create_rectangle(x0 + 1, y0 + 1, x1 - 1, y1 - 1, outline=color, width=2, dash=(4, 2))

    def _draw_heading_arrow_from_pos(self, pos_m: PosM, heading: str):
        cx, cy = self.world_pos_m_to_canvas(pos_m)
        dr, dc = HEADING_VEC[heading]
        length = self.CELL_PX * 0.34
        ex = cx + dc * length
        ey = cy - dr * length
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

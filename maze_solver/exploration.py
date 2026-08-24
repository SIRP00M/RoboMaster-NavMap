"""Trémaux / DFS-style maze exploration with topological memory.

The explorer remembers decision points (junctions/corners/dead ends) using
RoboMaster chassis odometry.  Every exit has a visit count:

    0 = never used        -> highest priority
    1 = used once         -> backtracking / second choice
    2+ = already covered  -> avoid unless there is no alternative

This is deliberately topological rather than a full occupancy-grid mapper.
"""

from dataclasses import dataclass, field
from typing import Optional
import json
import math
import time

import config


HEADINGS = ("N", "E", "S", "W")
RELATIVE_ORDER = ("FRONT", "RIGHT", "BACK", "LEFT")
RELATIVE_OFFSET = {
    "FRONT": 0,
    "RIGHT": 1,
    "BACK": 2,
    "LEFT": -1,
}


@dataclass
class ExitState:
    visits: int = 0
    target: Optional[str] = None


@dataclass
class MazeNode:
    node_id: str
    x: float
    y: float
    exits: dict = field(default_factory=dict)
    seen_count: int = 0


@dataclass(frozen=True)
class ExplorationDecision:
    direction: str
    node_id: str
    reason: str
    visits_before: int
    absolute_heading: str


class DecisionPointDetector:
    """Debounce decision points with side-zone hysteresis and robust re-arm."""

    def __init__(self):
        self.candidate_count = 0
        self.clear_count = 0
        self.latched = False

        self.latch_x = None
        self.latch_y = None
        self.latch_time = None

        # V9 Schmitt-trigger memory. These are ROBOT-RELATIVE states and must
        # be reset after a physical turn because left/right then see new walls.
        self.left_open_memory = False
        self.right_open_memory = False
        self.last_left_zone = "UNKNOWN"
        self.last_right_zone = "UNKNOWN"

        # V11 fusion diagnostics.
        self.last_left_fusion = "INIT"
        self.last_right_fusion = "INIT"
        self.last_left_confidence = "LOW"
        self.last_right_confidence = "LOW"

    @staticmethod
    def classify_side_zone(distance_cm):
        if distance_cm is None:
            return "UNKNOWN"
        if distance_cm <= config.SIDE_BLOCKED_MAX_CM:
            return "BLOCKED"
        if distance_cm >= config.SIDE_OPEN_MIN_CM:
            return "OPEN"
        return "BORDERLINE"

    @staticmethod
    def _resolve_side(zone, previous_open, ir_wall=None):
        """Fuse Sharp zone + stable digital IR.

        Sharp remains authoritative in the two strong zones.  IR is most useful
        in the 14-20 cm BORDERLINE band:

          BLOCKED    -> False regardless of IR
          BORDERLINE -> stable IR decides; otherwise hold previous state
          OPEN       -> True regardless of IR; IR=WALL becomes a conflict flag
          UNKNOWN    -> hold previous state
        """
        if zone == "BLOCKED":
            if ir_wall is False:
                return False, "SHARP_BLOCKED_IR_CONFLICT", "MEDIUM"
            if ir_wall is True:
                return False, "BLOCK_CONFIRMED", "HIGH"
            return False, "SHARP_BLOCKED", "HIGH"

        if zone == "OPEN":
            if ir_wall is True:
                # Strong Sharp opening is not overridden by one binary sensor.
                return True, "SHARP_OPEN_IR_CONFLICT", "MEDIUM"
            if ir_wall is False:
                return True, "OPEN_CONFIRMED", "HIGH"
            return True, "SHARP_OPEN", "HIGH"

        if zone == "BORDERLINE":
            if ir_wall is True:
                return False, "IR_CONFIRMS_BLOCK", "HIGH"
            if ir_wall is False:
                return True, "IR_CONFIRMS_OPEN", "HIGH"
            return previous_open, "BORDERLINE_HOLD", "LOW"

        return previous_open, "UNKNOWN_HOLD", "LOW"

    def classify_openings(
        self,
        front_cm,
        left_cm,
        right_cm,
        left_ir_wall=None,
        right_ir_wall=None,
    ):
        front_open = (
            front_cm is not None
            and front_cm >= config.EXPLORATION_FRONT_OPEN_CM
        )
        front_blocked = (
            front_cm is not None
            and 0.0 < front_cm <= config.STOP_FRONT_CM
        )

        self.last_left_zone = self.classify_side_zone(left_cm)
        self.last_right_zone = self.classify_side_zone(right_cm)

        # Allow quick disable from config for A/B testing.
        if not config.ENABLE_IR_SIDE_FUSION:
            left_ir_wall = None
            right_ir_wall = None

        (
            self.left_open_memory,
            self.last_left_fusion,
            self.last_left_confidence,
        ) = self._resolve_side(
            self.last_left_zone,
            self.left_open_memory,
            left_ir_wall,
        )

        (
            self.right_open_memory,
            self.last_right_fusion,
            self.last_right_confidence,
        ) = self._resolve_side(
            self.last_right_zone,
            self.right_open_memory,
            right_ir_wall,
        )

        return (
            front_open,
            front_blocked,
            self.left_open_memory,
            self.right_open_memory,
        )

    def get_side_zones(self):
        return self.last_left_zone, self.last_right_zone

    def get_side_fusion(self):
        return {
            "left": {
                "zone": self.last_left_zone,
                "resolved_open": self.left_open_memory,
                "reason": self.last_left_fusion,
                "confidence": self.last_left_confidence,
            },
            "right": {
                "zone": self.last_right_zone,
                "resolved_open": self.right_open_memory,
                "reason": self.last_right_fusion,
                "confidence": self.last_right_confidence,
            },
        }

    def reset_side_memory(self):
        self.left_open_memory = False
        self.right_open_memory = False
        self.last_left_zone = "UNKNOWN"
        self.last_right_zone = "UNKNOWN"
        self.last_left_fusion = "RESET"
        self.last_right_fusion = "RESET"
        self.last_left_confidence = "LOW"
        self.last_right_confidence = "LOW"

    def _distance_from_latch(self, pose_x, pose_y):
        if (
            pose_x is None
            or pose_y is None
            or self.latch_x is None
            or self.latch_y is None
        ):
            return None
        return math.hypot(
            float(pose_x) - self.latch_x,
            float(pose_y) - self.latch_y,
        )

    def _release_latch(self, reset_side_memory=False):
        self.latched = False
        self.candidate_count = 0
        self.clear_count = 0
        self.latch_x = None
        self.latch_y = None
        self.latch_time = None
        if reset_side_memory:
            self.reset_side_memory()

    def update(
        self,
        front_cm,
        left_cm,
        right_cm,
        left_ir_wall=None,
        right_ir_wall=None,
        pose_x=None,
        pose_y=None,
    ):
        front_open, front_blocked, left_open, right_open = self.classify_openings(
            front_cm,
            left_cm,
            right_cm,
            left_ir_wall=left_ir_wall,
            right_ir_wall=right_ir_wall,
        )
        now = time.monotonic()

        if self.latched:
            # Once both side openings disappear and front is not an emergency
            # block, the robot is physically leaving the old junction.
            normal_corridor = (
                not left_open
                and not right_open
                and not front_blocked
            )

            self.clear_count = self.clear_count + 1 if normal_corridor else 0
            distance = self._distance_from_latch(pose_x, pose_y)
            elapsed = now - self.latch_time if self.latch_time is not None else None

            moved_minimum = (
                distance is None
                or distance >= config.JUNCTION_REARM_MIN_DISTANCE_M
            )

            released_by_corridor = (
                self.clear_count >= config.JUNCTION_REARM_SAMPLES
                and moved_minimum
            )
            released_by_distance = (
                distance is not None
                and distance >= config.JUNCTION_REARM_DISTANCE_M
                and not left_open
                and not right_open
            )
            released_by_timeout = (
                elapsed is not None
                and elapsed >= config.JUNCTION_REARM_TIMEOUT_SEC
                and distance is not None
                and distance >= config.JUNCTION_REARM_MIN_DISTANCE_M
                and not left_open
                and not right_open
            )
            released_by_emergency_front = (
                front_blocked
                and elapsed is not None
                and elapsed >= config.JUNCTION_REARM_EMERGENCY_SEC
            )

            if not (
                released_by_corridor
                or released_by_distance
                or released_by_timeout
                or released_by_emergency_front
            ):
                return False

            self._release_latch(reset_side_memory=False)

        candidate = front_blocked or left_open or right_open
        self.candidate_count = self.candidate_count + 1 if candidate else 0

        if self.candidate_count >= config.JUNCTION_CONFIRM_SAMPLES:
            self.latched = True
            self.candidate_count = 0
            self.clear_count = 0
            self.latch_x = float(pose_x) if pose_x is not None else None
            self.latch_y = float(pose_y) if pose_y is not None else None
            self.latch_time = now
            return True

        return False

    def force_latched(self, pose_x=None, pose_y=None, reset_side_memory=True):
        """Lock current decision point after turn; reset robot-relative side state."""
        self.latched = True
        self.candidate_count = 0
        self.clear_count = 0
        self.latch_x = float(pose_x) if pose_x is not None else None
        self.latch_y = float(pose_y) if pose_y is not None else None
        self.latch_time = time.monotonic()
        if reset_side_memory:
            self.reset_side_memory()

    def cancel_event(self):
        self._release_latch(reset_side_memory=False)


class TremauxExplorer:
    def __init__(self):
        self.nodes = {}
        self.next_node_index = 0

        # Internal compass. Initial robot heading is defined as N.
        self.heading_index = 0

        self.start_node_id = None
        self.current_node_id = None

        # First real decision point becomes the DFS root.  The corridor back
        # to the physical start is remembered separately so the robot can stop
        # once every branch from the root has been explored.
        self.root_decision_node_id = None
        self.root_entry_abs_dir = None

        # Pending edge = route currently being travelled.
        self.pending_from_node = None
        self.pending_abs_dir = None

        # V10 diagnostic for expected-target node recognition.
        self.last_node_match = None

        self.route_history = []
        self.dfs_stack = []
        self.completed = False

    # ========================================================
    # HEADING HELPERS
    # ========================================================

    def heading_name(self, index=None):
        if index is None:
            index = self.heading_index
        return HEADINGS[index % 4]

    def absolute_index(self, relative_direction):
        return (
            self.heading_index
            + RELATIVE_OFFSET[relative_direction]
        ) % 4

    @staticmethod
    def opposite_index(abs_index):
        return (abs_index + 2) % 4

    def relative_for_absolute(self, abs_index):
        diff = (abs_index - self.heading_index) % 4
        return {
            0: "FRONT",
            1: "RIGHT",
            2: "BACK",
            3: "LEFT",
        }[diff]

    # ========================================================
    # NODE / EDGE MEMORY
    # ========================================================

    def _create_node(self, x, y):
        node_id = f"J{self.next_node_index}"
        self.next_node_index += 1

        self.nodes[node_id] = MazeNode(
            node_id=node_id,
            x=float(x),
            y=float(y),
        )

        return node_id

    def _find_nearby_node(self, x, y, radius=None):
        if radius is None:
            radius = config.NODE_MATCH_RADIUS_M

        best_id = None
        best_distance = None

        for node_id, node in self.nodes.items():
            distance = math.hypot(x - node.x, y - node.y)

            if distance <= radius:
                if best_distance is None or distance < best_distance:
                    best_id = node_id
                    best_distance = distance

        return best_id, best_distance

    def _expected_pending_target(self):
        """Return the known target of the edge currently being traversed.

        Example: if we departed J9 through E and J9.E.target == J8, then J8
        is the expected next decision node during backtracking.
        """
        if self.pending_from_node is None or self.pending_abs_dir is None:
            return None

        source = self.nodes.get(self.pending_from_node)
        if source is None:
            return None

        exit_state = source.exits.get(self.pending_abs_dir % 4)
        if exit_state is None:
            return None

        target_id = exit_state.target
        if target_id not in self.nodes:
            return None

        return target_id

    def _get_or_create_node(self, x, y):
        """Recognise a decision node with expected-target priority.

        Normal/new exploration still uses NODE_MATCH_RADIUS_M (0.18 m).
        Only an edge that already has an explicit target may use wider matching.

        Matching order:
          1) expected target within 0.40 m -> trust it immediately,
          2) otherwise probe the normal 0.18 m neighbourhood,
          3) if nothing else is nearby, allow the expected target up to 0.65 m,
          4) only then create a new node.

        This fixes duplicate junctions on reverse approaches while still leaving
        a sanity guard for a genuinely unexpected decision point.
        """
        self.last_node_match = None

        expected_id = self._expected_pending_target()
        expected_distance = None
        if expected_id is not None:
            expected = self.nodes[expected_id]
            expected_distance = math.hypot(x - expected.x, y - expected.y)

            if expected_distance <= config.EXPECTED_NODE_MATCH_RADIUS_M:
                node_id = expected_id
                is_new = False
                self.last_node_match = {
                    "mode": "EXPECTED",
                    "node_id": node_id,
                    "distance_m": expected_distance,
                    "from_node": self.pending_from_node,
                    "abs_dir": self.pending_abs_dir,
                }
            else:
                node_id = None
        else:
            node_id = None

        normal_id = None
        normal_distance = None
        if node_id is None:
            normal_id, normal_distance = self._find_nearby_node(
                x,
                y,
                radius=config.NODE_MATCH_RADIUS_M,
            )

            # Do not let a wider expected radius steal a clearly nearby node.
            if normal_id is not None:
                node_id = normal_id
                is_new = False
                self.last_node_match = {
                    "mode": "NEARBY",
                    "node_id": node_id,
                    "distance_m": normal_distance,
                    "expected_node": expected_id,
                    "expected_distance_m": expected_distance,
                }

        if (
            node_id is None
            and expected_id is not None
            and expected_distance is not None
            and expected_distance <= config.EXPECTED_NODE_FALLBACK_RADIUS_M
        ):
            node_id = expected_id
            is_new = False
            self.last_node_match = {
                "mode": "EXPECTED_RELAXED",
                "node_id": node_id,
                "distance_m": expected_distance,
                "from_node": self.pending_from_node,
                "abs_dir": self.pending_abs_dir,
            }

        if node_id is None:
            node_id = self._create_node(x, y)
            is_new = True
            self.last_node_match = {
                "mode": "NEW",
                "node_id": node_id,
                "distance_m": None,
                "expected_node": expected_id,
                "expected_distance_m": expected_distance,
            }

        node = self.nodes[node_id]
        node.seen_count += 1

        # Slowly average observed position to reduce odometry/threshold jitter.
        alpha = config.NODE_POSITION_UPDATE_ALPHA
        node.x = (1.0 - alpha) * node.x + alpha * float(x)
        node.y = (1.0 - alpha) * node.y + alpha * float(y)

        return node_id, is_new

    def _exit(self, node_id, abs_index):
        node = self.nodes[node_id]
        abs_index %= 4

        if abs_index not in node.exits:
            node.exits[abs_index] = ExitState()

        return node.exits[abs_index]

    def _link_nodes(self, from_node_id, abs_index, to_node_id):
        if from_node_id == to_node_id:
            return

        abs_index %= 4
        opposite = self.opposite_index(abs_index)

        source_exit = self._exit(from_node_id, abs_index)
        target_exit = self._exit(to_node_id, opposite)

        # A known topological edge is immutable during ordinary traversal.
        # Never silently overwrite J9->J8 with J9->J10 just because odometry
        # caused the reverse approach to be detected at a shifted position.
        if source_exit.target is not None and source_exit.target != to_node_id:
            print(
                f"GRAPH LINK CONFLICT: {from_node_id}."
                f"{HEADINGS[abs_index]} already -> {source_exit.target}, "
                f"refusing overwrite -> {to_node_id}"
            )
            return False

        if target_exit.target is not None and target_exit.target != from_node_id:
            print(
                f"GRAPH LINK CONFLICT: {to_node_id}."
                f"{HEADINGS[opposite]} already -> {target_exit.target}, "
                f"refusing overwrite -> {from_node_id}"
            )
            return False

        source_exit.target = to_node_id
        target_exit.target = from_node_id

        # Keep Trémaux marks mirrored on both ends of the same corridor.
        shared_visits = max(source_exit.visits, target_exit.visits)
        source_exit.visits = shared_visits
        target_exit.visits = shared_visits
        return True

    def _increment_departure(self, node_id, abs_index):
        exit_state = self._exit(node_id, abs_index)
        target_id = exit_state.target

        if target_id is None:
            exit_state.visits += 1
            return exit_state.visits

        opposite = self.opposite_index(abs_index)
        target_exit = self._exit(target_id, opposite)
        new_visits = max(exit_state.visits, target_exit.visits) + 1

        exit_state.visits = new_visits
        target_exit.visits = new_visits
        return new_visits

    # ========================================================
    # START / ARRIVAL
    # ========================================================

    def initialize_start(self, x, y):
        if self.start_node_id is not None:
            return self.start_node_id

        node_id = self._create_node(x, y)
        self.nodes[node_id].seen_count = 1

        self.start_node_id = node_id
        self.current_node_id = node_id
        self.dfs_stack = [node_id]

        return node_id

    def commit_initial_forward(self):
        """Mark the corridor travelled from the starting pose."""
        if self.current_node_id is None:
            raise RuntimeError("initialize_start() must be called first")

        abs_index = self.heading_index
        self._increment_departure(self.current_node_id, abs_index)
        self.pending_from_node = self.current_node_id
        self.pending_abs_dir = abs_index

        self.route_history.append({
            "time": time.time(),
            "node": self.current_node_id,
            "direction": "FRONT",
            "heading": self.heading_name(abs_index),
            "kind": "initial_departure",
        })

    def arrive_at_decision_point(self, x, y):
        node_id, is_new = self._get_or_create_node(x, y)

        previous_node = self.pending_from_node
        incoming_abs_dir = self.pending_abs_dir

        # Connect the corridor we just travelled to this node.
        if (
            previous_node is not None
            and incoming_abs_dir is not None
            and previous_node != node_id
        ):
            self._link_nodes(previous_node, incoming_abs_dir, node_id)

        # The first decision point reached from START is the DFS root.
        if (
            self.root_decision_node_id is None
            and previous_node == self.start_node_id
            and incoming_abs_dir is not None
        ):
            self.root_decision_node_id = node_id
            self.root_entry_abs_dir = self.opposite_index(incoming_abs_dir)

        self.current_node_id = node_id

        # Maintain a DFS-style traversal stack for debug/backtracking insight.
        if not self.dfs_stack:
            self.dfs_stack.append(node_id)
        elif self.dfs_stack[-1] != node_id:
            if len(self.dfs_stack) >= 2 and self.dfs_stack[-2] == node_id:
                self.dfs_stack.pop()
            elif node_id in self.dfs_stack:
                while self.dfs_stack and self.dfs_stack[-1] != node_id:
                    self.dfs_stack.pop()
            else:
                self.dfs_stack.append(node_id)

        self.pending_from_node = None
        self.pending_abs_dir = None

        return node_id, is_new

    # ========================================================
    # TRÉMAUX / DFS DECISION
    # ========================================================

    def _physical_candidates(
        self,
        front_open,
        left_open,
        right_open,
        allow_back=True,
    ):
        relative_candidates = []

        if front_open:
            relative_candidates.append("FRONT")
        if left_open:
            relative_candidates.append("LEFT")
        if right_open:
            relative_candidates.append("RIGHT")
        if allow_back:
            relative_candidates.append("BACK")

        # Remove duplicates while preserving order.
        result = []
        seen = set()
        for relative in relative_candidates:
            abs_index = self.absolute_index(relative)
            if abs_index in seen:
                continue
            seen.add(abs_index)
            result.append((relative, abs_index))

        return result

    def plan_direction(
        self,
        front_open,
        left_open,
        right_open,
    ):
        if self.current_node_id is None:
            raise RuntimeError("No current node. Call arrive_at_decision_point()")

        # BACK is valid whenever the robot reached this point through a corridor.
        allow_back = self.current_node_id != self.start_node_id or bool(
            self.nodes[self.current_node_id].exits
        )

        candidates = self._physical_candidates(
            front_open,
            left_open,
            right_open,
            allow_back=allow_back,
        )

        node = self.nodes[self.current_node_id]

        # Register every physically observed exit in node memory.
        for _, abs_index in candidates:
            self._exit(self.current_node_id, abs_index)

        if not candidates:
            self.completed = True
            return ExplorationDecision(
                direction="COMPLETE",
                node_id=self.current_node_id,
                reason="NO_AVAILABLE_EXIT",
                visits_before=0,
                absolute_heading=self.heading_name(),
            )

        preference_rank = {
            name: index
            for index, name in enumerate(config.EXPLORATION_PREFERENCE)
        }

        scored = []
        for relative, abs_index in candidates:
            visits = self._exit(self.current_node_id, abs_index).visits
            scored.append((
                visits,
                preference_rank.get(relative, 99),
                relative,
                abs_index,
            ))

        # Core Trémaux / DFS rule:
        #   1) choose an unvisited edge (0 marks),
        #   2) if none, go directly BACK over the incoming once-marked edge,
        #   3) otherwise use another once-marked edge,
        #   4) never intentionally traverse a 2-mark edge again.
        scored.sort(key=lambda item: (item[0], item[1]))

        # If we are back at the first real decision point and every branch
        # except the entrance corridor has two marks, DFS is complete.
        if self.current_node_id == self.root_decision_node_id:
            branch_visits = []
            for direction_index, exit_state in node.exits.items():
                if direction_index == self.root_entry_abs_dir:
                    continue
                branch_visits.append(exit_state.visits)

            has_unvisited_branch = any(v == 0 for v in branch_visits)
            has_once_branch = any(v == 1 for v in branch_visits)

            if (
                branch_visits
                and not has_unvisited_branch
                and not has_once_branch
            ):
                self.completed = True
                return ExplorationDecision(
                    direction="COMPLETE",
                    node_id=self.current_node_id,
                    reason="DFS_ROOT_ALL_BRANCHES_EXPLORED",
                    visits_before=0,
                    absolute_heading=self.heading_name(),
                )

        unvisited = [item for item in scored if item[0] == 0]
        if unvisited:
            visits, _, relative, abs_index = unvisited[0]
            reason = "UNVISITED_EXIT"
            return ExplorationDecision(
                direction=relative,
                node_id=self.current_node_id,
                reason=reason,
                visits_before=visits,
                absolute_heading=self.heading_name(abs_index),
            )

        # Strict DFS-style backtracking: if the corridor immediately behind us
        # has only one mark, turn around rather than wandering onto a different
        # already-used corridor.
        back_candidates = [
            item for item in scored
            if item[2] == "BACK" and item[0] < config.MAX_EDGE_VISITS
        ]
        if back_candidates:
            visits, _, relative, abs_index = back_candidates[0]
            return ExplorationDecision(
                direction=relative,
                node_id=self.current_node_id,
                reason="DFS_BACKTRACK",
                visits_before=visits,
                absolute_heading=self.heading_name(abs_index),
            )

        once_used = [
            item for item in scored
            if item[0] < config.MAX_EDGE_VISITS
        ]
        if once_used:
            visits, _, relative, abs_index = once_used[0]
            return ExplorationDecision(
                direction=relative,
                node_id=self.current_node_id,
                reason="SECOND_PASS_TO_BACKTRACK",
                visits_before=visits,
                absolute_heading=self.heading_name(abs_index),
            )

        # Every currently available edge already has two marks.  Stopping is
        # safer than intentionally entering an explored loop for a third time.
        visits, _, _, abs_index = scored[0]
        self.completed = True
        return ExplorationDecision(
            direction="COMPLETE",
            node_id=self.current_node_id,
            reason="ALL_AVAILABLE_EDGES_ALREADY_TWICE",
            visits_before=visits,
            absolute_heading=self.heading_name(abs_index),
        )

    def commit_decision(self, decision):
        if decision.direction == "COMPLETE":
            self.completed = True
            return

        abs_index = self.absolute_index(decision.direction)
        new_visits = self._increment_departure(
            self.current_node_id,
            abs_index,
        )

        self.pending_from_node = self.current_node_id
        self.pending_abs_dir = abs_index

        self.route_history.append({
            "time": time.time(),
            "node": self.current_node_id,
            "direction": decision.direction,
            "absolute_heading": self.heading_name(abs_index),
            "edge_visits": new_visits,
            "reason": decision.reason,
        })

        # Update internal compass after the physical relative turn.
        self.heading_index = abs_index

    # ========================================================
    # DEBUG / SAVE
    # ========================================================

    def describe_node(self, node_id=None):
        if node_id is None:
            node_id = self.current_node_id

        if node_id is None or node_id not in self.nodes:
            return "NO_NODE"

        node = self.nodes[node_id]
        parts = []

        for abs_index in range(4):
            if abs_index not in node.exits:
                continue

            exit_state = node.exits[abs_index]
            target = exit_state.target or "?"
            parts.append(
                f"{HEADINGS[abs_index]}:{exit_state.visits}->{target}"
            )

        return " | ".join(parts) if parts else "NO_EXITS"

    def save_memory(self, filepath=None):
        if filepath is None:
            filepath = config.MAZE_MEMORY_FILE

        data = {
            "start_node_id": self.start_node_id,
            "root_decision_node_id": self.root_decision_node_id,
            "root_entry_heading": (
                self.heading_name(self.root_entry_abs_dir)
                if self.root_entry_abs_dir is not None
                else None
            ),
            "current_node_id": self.current_node_id,
            "heading": self.heading_name(),
            "completed": self.completed,
            "dfs_stack": list(self.dfs_stack),
            "nodes": {},
            "route_history": self.route_history,
        }

        for node_id, node in self.nodes.items():
            data["nodes"][node_id] = {
                "x": node.x,
                "y": node.y,
                "seen_count": node.seen_count,
                "exits": {
                    HEADINGS[int(abs_index)]: {
                        "visits": exit_state.visits,
                        "target": exit_state.target,
                    }
                    for abs_index, exit_state in node.exits.items()
                },
            }

        with open(filepath, "w", encoding="utf-8") as file:
            json.dump(data, file, indent=2, ensure_ascii=False)

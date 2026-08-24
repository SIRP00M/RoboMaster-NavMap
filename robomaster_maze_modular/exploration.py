"""Trémaux / DFS-style maze exploration with topological memory."""

from collections import deque
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
    # A frontier is an observed physical opening that has never been traversed.
    # miss_count lets repeated stopped re-scans retire a one-off false opening.
    seen_open_count: int = 0
    miss_count: int = 0
    blocked: bool = False


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
    """Detect physical decision points using a V6 intersection window.

    V5 waited for one side-opening zone to finish, moved back to its midpoint,
    then trusted a stopped snapshot.  On the real maze this can miss a straight
    continuation: LEFT may open first, the robot moves farther, RIGHT (the old
    branch) appears later, and the stopped ToF can point at a nearby wall edge.

    V6 therefore keeps an *intersection window* after the first confirmed side
    opening.  During that window it accumulates whether FRONT / LEFT / RIGHT
    were physically open for multiple samples.  A short look-ahead lets the
    detector merge opposite-side openings that are longitudinally offset by the
    robot/sensor geometry.  The final stopped scan is merged with these
    accumulated observations before the explorer chooses a branch.
    """

    def __init__(self):
        self.front_candidate_count = 0
        self.clear_count = 0
        self.latched = False
        self.latch_x = None
        self.latch_y = None
        self.latch_time = None
        self.pending_zone_event = None
        self.left_zone = self._new_zone()
        self.right_zone = self._new_zone()
        self.intersection_window = self._new_intersection_window()

    @staticmethod
    def _new_zone():
        return {
            "candidate_count": 0,
            "candidate_start_x": None,
            "candidate_start_y": None,
            "active": False,
            "start_x": None,
            "start_y": None,
            "exit_count": 0,
            "max_cm": 0.0,
        }

    @staticmethod
    def _new_intersection_window():
        return {
            "active": False,
            "start_x": None,
            "start_y": None,
            "last_side_open_x": None,
            "last_side_open_y": None,
            "lookahead_start_x": None,
            "lookahead_start_y": None,
            "front_open_samples": 0,
            "left_open_samples": 0,
            "right_open_samples": 0,
            "front_max_cm": 0.0,
            "left_max_cm": 0.0,
            "right_max_cm": 0.0,
            "completed_sides": set(),
        }

    @staticmethod
    def classify_openings(front_cm, left_cm, right_cm):
        """Stopped-scan classification using strict thresholds."""
        front_open = (
            front_cm is not None
            and front_cm >= config.EXPLORATION_FRONT_OPEN_CM
        )
        front_blocked = (
            front_cm is not None
            and 0.0 < front_cm <= config.STOP_FRONT_CM
        )
        left_open = (
            left_cm is not None
            and left_cm >= config.SIDE_OPEN_ENTER_CM
        )
        right_open = (
            right_cm is not None
            and right_cm >= config.SIDE_OPEN_ENTER_CM
        )
        return front_open, front_blocked, left_open, right_open

    @staticmethod
    def _distance_xy(x1, y1, x2, y2):
        if None in (x1, y1, x2, y2):
            return None
        return math.hypot(float(x2) - float(x1), float(y2) - float(y1))

    def _distance_from_latch(self, pose_x, pose_y):
        return self._distance_xy(self.latch_x, self.latch_y, pose_x, pose_y)

    def _reset_zones(self):
        self.left_zone = self._new_zone()
        self.right_zone = self._new_zone()
        self.intersection_window = self._new_intersection_window()
        self.pending_zone_event = None

    def _release_latch(self):
        self.latched = False
        self.front_candidate_count = 0
        self.clear_count = 0
        self.latch_x = None
        self.latch_y = None
        self.latch_time = None
        self._reset_zones()

    def _latch_now(self, now, pose_x, pose_y):
        self.latched = True
        self.front_candidate_count = 0
        self.clear_count = 0
        self.latch_x = float(pose_x) if pose_x is not None else None
        self.latch_y = float(pose_y) if pose_y is not None else None
        self.latch_time = now

    def _track_side_zone(self, zone, side_cm, pose_x, pose_y, side_name):
        """Track one physical side-opening mouth; return completion metadata."""
        if side_cm is None:
            return None

        enter = float(config.SIDE_OPEN_ENTER_CM)
        exit_threshold = float(config.SIDE_OPEN_EXIT_CM)

        if not zone["active"]:
            if side_cm >= enter:
                if zone["candidate_count"] == 0:
                    zone["candidate_start_x"] = pose_x
                    zone["candidate_start_y"] = pose_y
                zone["candidate_count"] += 1
                zone["max_cm"] = max(zone["max_cm"], float(side_cm))

                if zone["candidate_count"] >= config.OPENING_ZONE_ENTER_SAMPLES:
                    zone["active"] = True
                    zone["start_x"] = zone["candidate_start_x"]
                    zone["start_y"] = zone["candidate_start_y"]
                    zone["exit_count"] = 0
                    print(
                        f">>> OPENING_ZONE {side_name} START "
                        f"Sharp={side_cm:.1f}cm enter>={enter:.1f}"
                    )
            else:
                zone.update(self._new_zone())
            return None

        zone["max_cm"] = max(zone["max_cm"], float(side_cm))
        length = self._distance_xy(
            zone["start_x"], zone["start_y"], pose_x, pose_y
        )

        if side_cm < exit_threshold:
            zone["exit_count"] += 1
        else:
            zone["exit_count"] = 0

        completed_by_exit = (
            zone["exit_count"] >= config.OPENING_ZONE_EXIT_SAMPLES
        )
        completed_by_max = (
            length is not None
            and length >= config.OPENING_ZONE_MAX_LENGTH_M
        )
        if not (completed_by_exit or completed_by_max):
            return None

        if length is None or length < config.OPENING_ZONE_MIN_LENGTH_M:
            print(
                f">>> OPENING_ZONE {side_name} REJECT "
                f"length={0.0 if length is None else length:.3f}m "
                f"< {config.OPENING_ZONE_MIN_LENGTH_M:.3f}m"
            )
            zone.update(self._new_zone())
            return None

        event = {
            "type": "SIDE_OPENING_ZONE",
            "side": side_name,
            "length_m": float(length),
            "start_x": zone["start_x"],
            "start_y": zone["start_y"],
            "end_x": pose_x,
            "end_y": pose_y,
            "max_cm": float(zone["max_cm"]),
            "forced_by_max": bool(completed_by_max and not completed_by_exit),
        }
        print(
            f">>> OPENING_ZONE {side_name} END "
            f"length={event['length_m']:.3f}m maxSharp={event['max_cm']:.1f}cm"
        )
        zone.update(self._new_zone())
        return event

    def _start_intersection_window(self, pose_x, pose_y):
        if self.intersection_window["active"]:
            return

        starts = []
        for side_name, zone in (("LEFT", self.left_zone), ("RIGHT", self.right_zone)):
            if zone["active"]:
                starts.append((side_name, zone["start_x"], zone["start_y"]))
        if not starts:
            return

        # Both side sensors are on the same chassis and normally trigger within
        # a few samples.  Use the first confirmed candidate start as the window
        # start.  Exact ordering is not safety-critical because centering is
        # clipped later.
        side_name, start_x, start_y = starts[0]
        w = self.intersection_window
        w["active"] = True
        w["start_x"] = start_x if start_x is not None else pose_x
        w["start_y"] = start_y if start_y is not None else pose_y
        w["last_side_open_x"] = pose_x
        w["last_side_open_y"] = pose_y

        # The initiating zone has already satisfied ENTER_SAMPLES, so seed its
        # evidence rather than pretending we only saw the current sample.
        if self.left_zone["active"]:
            w["left_open_samples"] = max(
                w["left_open_samples"], config.OPENING_ZONE_ENTER_SAMPLES
            )
        if self.right_zone["active"]:
            w["right_open_samples"] = max(
                w["right_open_samples"], config.OPENING_ZONE_ENTER_SAMPLES
            )

        print(
            f">>> INTERSECTION_WINDOW START by={side_name} "
            f"lookahead={config.INTERSECTION_WINDOW_LOOKAHEAD_M:.2f}m"
        )

    def _accumulate_intersection(self, front_cm, left_cm, right_cm, pose_x, pose_y):
        w = self.intersection_window
        if not w["active"]:
            return

        if front_cm is not None:
            w["front_max_cm"] = max(w["front_max_cm"], float(front_cm))
            if front_cm >= config.INTERSECTION_FRONT_OPEN_CM:
                w["front_open_samples"] += 1

        left_phys_open = (
            left_cm is not None
            and left_cm >= config.INTERSECTION_SIDE_OPEN_CM
        )
        right_phys_open = (
            right_cm is not None
            and right_cm >= config.INTERSECTION_SIDE_OPEN_CM
        )

        if left_cm is not None:
            w["left_max_cm"] = max(w["left_max_cm"], float(left_cm))
        if right_cm is not None:
            w["right_max_cm"] = max(w["right_max_cm"], float(right_cm))

        if left_phys_open:
            w["left_open_samples"] += 1
        if right_phys_open:
            w["right_open_samples"] += 1

        # EXIT hysteresis is used here so the last-open location reaches the
        # physical far edge of the intersection instead of ending on one noisy
        # sample below ENTER.
        left_still_open = (
            left_cm is not None and left_cm >= config.SIDE_OPEN_EXIT_CM
        )
        right_still_open = (
            right_cm is not None and right_cm >= config.SIDE_OPEN_EXIT_CM
        )
        if left_still_open or right_still_open or self.left_zone["active"] or self.right_zone["active"]:
            w["last_side_open_x"] = pose_x
            w["last_side_open_y"] = pose_y
            w["lookahead_start_x"] = None
            w["lookahead_start_y"] = None
        elif w["lookahead_start_x"] is None:
            w["lookahead_start_x"] = pose_x
            w["lookahead_start_y"] = pose_y
            print(
                ">>> INTERSECTION_WINDOW LOOKAHEAD "
                f"target={config.INTERSECTION_WINDOW_LOOKAHEAD_M:.2f}m"
            )

    def _intersection_should_finalize(self, pose_x, pose_y):
        w = self.intersection_window
        if not w["active"]:
            return False, None

        total = self._distance_xy(w["start_x"], w["start_y"], pose_x, pose_y)
        if total is not None and total >= config.INTERSECTION_WINDOW_MAX_M:
            return True, "MAX_LENGTH"

        if w["lookahead_start_x"] is not None:
            lookahead = self._distance_xy(
                w["lookahead_start_x"], w["lookahead_start_y"], pose_x, pose_y
            )
            if (
                lookahead is not None
                and lookahead >= config.INTERSECTION_WINDOW_LOOKAHEAD_M
            ):
                return True, "LOOKAHEAD_COMPLETE"

        return False, None

    def _finalize_intersection_window(self, pose_x, pose_y, reason):
        w = self.intersection_window
        if not w["active"]:
            return None

        total_length = self._distance_xy(
            w["start_x"], w["start_y"], pose_x, pose_y
        )
        opening_span = self._distance_xy(
            w["start_x"], w["start_y"],
            w["last_side_open_x"], w["last_side_open_y"],
        )
        total_length = float(total_length or 0.0)
        opening_span = float(opening_span or total_length)

        # We are at the end of the look-ahead.  To return to the centre of the
        # actual intersection (not the centre of the look-ahead tail), back up
        # by: total travelled - half the observed side-opening span.
        backtrack_m = max(0.0, total_length - 0.5 * opening_span)

        min_samples = int(config.INTERSECTION_MIN_OPEN_SAMPLES)
        observed = {
            "FRONT": w["front_open_samples"] >= min_samples,
            "LEFT": w["left_open_samples"] >= min_samples,
            "RIGHT": w["right_open_samples"] >= min_samples,
        }
        counts = {
            "FRONT": int(w["front_open_samples"]),
            "LEFT": int(w["left_open_samples"]),
            "RIGHT": int(w["right_open_samples"]),
        }
        event = {
            "type": "INTERSECTION_WINDOW",
            "side": "MULTI",
            "length_m": total_length,
            "opening_span_m": opening_span,
            "backtrack_m": backtrack_m,
            "start_x": w["start_x"],
            "start_y": w["start_y"],
            "end_x": pose_x,
            "end_y": pose_y,
            "observed_open": observed,
            "open_samples": counts,
            "max_cm": {
                "FRONT": float(w["front_max_cm"]),
                "LEFT": float(w["left_max_cm"]),
                "RIGHT": float(w["right_max_cm"]),
            },
            "completed_sides": sorted(w["completed_sides"]),
            "finish_reason": reason,
        }
        print(
            ">>> INTERSECTION_WINDOW END "
            f"reason={reason} length={total_length:.3f}m "
            f"span={opening_span:.3f}m backtrack={backtrack_m:.3f}m"
        )
        print(
            ">>> INTERSECTION_ACCUM "
            f"F={int(observed['FRONT'])}({counts['FRONT']}) "
            f"L={int(observed['LEFT'])}({counts['LEFT']}) "
            f"R={int(observed['RIGHT'])}({counts['RIGHT']})"
        )
        self.intersection_window = self._new_intersection_window()
        return event

    def consume_pending_zone(self):
        event = self.pending_zone_event
        self.pending_zone_event = None
        return event

    def update(
        self,
        front_cm,
        left_cm,
        right_cm,
        pose_x=None,
        pose_y=None,
    ):
        _, front_blocked, _, _ = self.classify_openings(
            front_cm, left_cm, right_cm
        )
        now = time.monotonic()

        left_still_open = (
            left_cm is not None and left_cm >= config.SIDE_OPEN_EXIT_CM
        )
        right_still_open = (
            right_cm is not None and right_cm >= config.SIDE_OPEN_EXIT_CM
        )

        if self.latched:
            normal_corridor = (
                not left_still_open
                and not right_still_open
                and not front_blocked
            )
            self.clear_count = self.clear_count + 1 if normal_corridor else 0

            distance = self._distance_from_latch(pose_x, pose_y)
            elapsed = (
                now - self.latch_time if self.latch_time is not None else None
            )
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
                and not left_still_open
                and not right_still_open
            )
            released_by_timeout = (
                elapsed is not None
                and elapsed >= config.JUNCTION_REARM_TIMEOUT_SEC
                and distance is not None
                and distance >= config.JUNCTION_REARM_MIN_DISTANCE_M
                and not left_still_open
                and not right_still_open
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
            self._release_latch()

        if front_blocked:
            self.front_candidate_count += 1
        else:
            self.front_candidate_count = 0

        if not getattr(config, "ENABLE_OPENING_ZONE_DETECTION", True):
            left_open = left_cm is not None and left_cm >= config.SIDE_OPEN_ENTER_CM
            right_open = right_cm is not None and right_cm >= config.SIDE_OPEN_ENTER_CM
            if self.front_candidate_count >= config.JUNCTION_CONFIRM_SAMPLES or left_open or right_open:
                self.pending_zone_event = None
                self._latch_now(now, pose_x, pose_y)
                return True
            return False

        completed = []
        left_event = self._track_side_zone(
            self.left_zone, left_cm, pose_x, pose_y, "LEFT"
        )
        if left_event is not None:
            completed.append(left_event)
        right_event = self._track_side_zone(
            self.right_zone, right_cm, pose_x, pose_y, "RIGHT"
        )
        if right_event is not None:
            completed.append(right_event)

        # Start/continue the V6 window as soon as at least one side opening has
        # been confirmed.  Completed side mouths do not immediately trigger a
        # decision; the short look-ahead may reveal the opposite-side branch.
        if getattr(config, "ENABLE_INTERSECTION_WINDOW", True):
            self._start_intersection_window(pose_x, pose_y)
            if self.intersection_window["active"]:
                for item in completed:
                    self.intersection_window["completed_sides"].add(item["side"])
                self._accumulate_intersection(
                    front_cm, left_cm, right_cm, pose_x, pose_y
                )

                # If the front becomes a confirmed hard stop while we are
                # already inside an intersection, finalize now rather than
                # discarding the side-opening evidence.
                if self.front_candidate_count >= config.JUNCTION_CONFIRM_SAMPLES:
                    event = self._finalize_intersection_window(
                        pose_x, pose_y, "FRONT_BLOCKED"
                    )
                    self.pending_zone_event = event
                    self._latch_now(now, pose_x, pose_y)
                    return True

                should_finish, reason = self._intersection_should_finalize(
                    pose_x, pose_y
                )
                if should_finish:
                    event = self._finalize_intersection_window(
                        pose_x, pose_y, reason
                    )
                    self.pending_zone_event = event
                    self._latch_now(now, pose_x, pose_y)
                    return True

                return False

        # No side intersection is active: front dead-end/corner safety retains
        # priority and triggers the normal stopped scan immediately.
        if self.front_candidate_count >= config.JUNCTION_CONFIRM_SAMPLES:
            self.pending_zone_event = None
            self._latch_now(now, pose_x, pose_y)
            return True

        # Compatibility path if V6 intersection windows are disabled.
        if completed:
            event = max(completed, key=lambda item: item["length_m"])
            event["all_sides"] = [item["side"] for item in completed]
            self.pending_zone_event = event
            self._latch_now(now, pose_x, pose_y)
            return True

        return False

    def force_latched(self, pose_x=None, pose_y=None):
        self._reset_zones()
        self._latch_now(time.monotonic(), pose_x, pose_y)

    def cancel_event(self):
        self._release_latch()


class TremauxExplorer:
    """Frontier-aware DFS explorer with a persistent topological graph.

    Important change from the old local Trémaux rule:
    - visits == 0 is a *frontier* (observed but never traversed).
    - If the current node has no actionable frontier, the robot routes through
      already-known corridors back to a node that still has a frontier.
    - Exploration is COMPLETE only when there are no active frontiers anywhere
      in the graph, not merely because the current node has been crossed twice.

    Existing corridors may therefore be traversed more than twice while they
    are being used as transit routes.  visits is kept as a traversal counter,
    not as a hard two-pass limit.
    """

    def __init__(self):
        self.nodes = {}
        self.next_node_index = 0

        # Internal compass. Initial robot heading is defined as N.
        self.heading_index = 0

        self.start_node_id = None
        self.current_node_id = None
        self.root_decision_node_id = None
        self.root_entry_abs_dir = None

        # Pending edge = corridor currently being travelled.
        self.pending_from_node = None
        self.pending_abs_dir = None

        self.route_history = []
        self.dfs_stack = []
        self.completed = False

        # Diagnostics for graph-integrity protection / skipped-node recovery.
        self.graph_events = []

        # V4 anti-oscillation memory. Key = (frontier signature, node, abs dir).
        # It only blocks a repeated ROUTE_* choice while no frontier progress
        # has happened; discovering/consuming a frontier naturally changes the
        # signature and releases the edge again.
        self.route_attempt_counts = {}

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

    def _find_nearby_node(self, x, y):
        best_id = None
        best_distance = None

        for node_id, node in self.nodes.items():
            distance = math.hypot(x - node.x, y - node.y)
            if distance <= config.NODE_MATCH_RADIUS_M:
                if best_distance is None or distance < best_distance:
                    best_id = node_id
                    best_distance = distance
        return best_id

    def _touch_node_position(self, node_id, x, y):
        node = self.nodes[node_id]
        node.seen_count += 1
        alpha = config.NODE_POSITION_UPDATE_ALPHA
        node.x = (1.0 - alpha) * node.x + alpha * float(x)
        node.y = (1.0 - alpha) * node.y + alpha * float(y)

    def _expected_arrival_node(self, x, y):
        """Prefer the known target of the corridor currently being traversed.

        On a revisit, odometry can drift enough that the normal generic radius
        creates a duplicate node at the same physical junction.  If the edge we
        are travelling already has a known target, allow a slightly larger
        target-specific radius.  This is safer than globally increasing the
        node-match radius because only the expected graph neighbor is eligible.
        """
        if self.pending_from_node is None or self.pending_abs_dir is None:
            return None
        if self.pending_from_node not in self.nodes:
            return None

        state = self.nodes[self.pending_from_node].exits.get(self.pending_abs_dir % 4)
        if state is None or state.target is None or state.target not in self.nodes:
            return None

        target = self.nodes[state.target]
        distance = math.hypot(float(x) - target.x, float(y) - target.y)
        radius = float(getattr(config, "EXPECTED_TARGET_MATCH_RADIUS_M", config.NODE_MATCH_RADIUS_M))
        if distance <= radius:
            return state.target
        return None

    def _get_or_create_node(self, x, y):
        expected_id = self._expected_arrival_node(x, y)
        if expected_id is not None:
            self._touch_node_position(expected_id, x, y)
            return expected_id, False

        node_id = self._find_nearby_node(x, y)
        is_new = node_id is None

        if is_new:
            node_id = self._create_node(x, y)

        self._touch_node_position(node_id, x, y)
        return node_id, is_new

    def _exit(self, node_id, abs_index):
        node = self.nodes[node_id]
        abs_index %= 4
        if abs_index not in node.exits:
            node.exits[abs_index] = ExitState()
        return node.exits[abs_index]

    def _record_graph_event(self, kind, **payload):
        event = {"time": time.time(), "kind": kind}
        event.update(payload)
        self.graph_events.append(event)

    def _connect_direct(self, from_node_id, abs_index, to_node_id):
        """Connect one corridor without overwriting an existing different edge."""
        if from_node_id == to_node_id:
            return False

        abs_index %= 4
        opposite = self.opposite_index(abs_index)
        source_exit = self._exit(from_node_id, abs_index)
        target_exit = self._exit(to_node_id, opposite)

        if source_exit.target not in (None, to_node_id):
            self._record_graph_event(
                "SOURCE_EDGE_CONFLICT_PROTECTED",
                from_node=from_node_id,
                heading=self.heading_name(abs_index),
                existing_target=source_exit.target,
                attempted_target=to_node_id,
            )
            return False

        if target_exit.target not in (None, from_node_id):
            self._record_graph_event(
                "TARGET_EDGE_CONFLICT_PROTECTED",
                to_node=to_node_id,
                heading=self.heading_name(opposite),
                existing_target=target_exit.target,
                attempted_target=from_node_id,
            )
            return False

        source_exit.target = to_node_id
        target_exit.target = from_node_id

        shared_visits = max(source_exit.visits, target_exit.visits)
        source_exit.visits = shared_visits
        target_exit.visits = shared_visits
        source_exit.blocked = False
        target_exit.blocked = False
        return True

    def _follow_same_heading_chain(self, start_node_id, abs_index, max_hops=64):
        """Follow already-known nodes in one absolute heading until chain ends."""
        chain = [start_node_id]
        current = start_node_id
        seen = {start_node_id}

        for _ in range(max_hops):
            state = self.nodes[current].exits.get(abs_index % 4)
            if state is None or state.target is None:
                break
            nxt = state.target
            if nxt in seen or nxt not in self.nodes:
                break
            chain.append(nxt)
            seen.add(nxt)
            current = nxt
        return chain

    def _increment_chain_after_first_edge(self, chain, abs_index):
        """A skipped intermediate decision was physically crossed this run.

        The first edge was already incremented by commit_decision() when the
        robot left pending_from_node.  Increment only later chain edges here.
        """
        if len(chain) <= 2:
            return
        for node_id in chain[1:-1]:
            self._increment_departure(node_id, abs_index)

    def _link_nodes(self, from_node_id, abs_index, to_node_id):
        """Link arrival while preserving known intermediate decision nodes.

        This fixes a failure mode where one pass detects J16 between J15/J17,
        but a later pass skips J16 and used to overwrite J15->J16 with J15->J17.
        """
        if from_node_id == to_node_id:
            return

        abs_index %= 4
        source_exit = self._exit(from_node_id, abs_index)

        # Normal first connection or already-direct connection.
        if source_exit.target in (None, to_node_id):
            self._connect_direct(from_node_id, abs_index, to_node_id)
            return

        # A known node already lies ahead in the same direction.  Follow the
        # chain rather than overwriting it.
        chain = self._follow_same_heading_chain(from_node_id, abs_index)
        if to_node_id in chain:
            cut = chain[:chain.index(to_node_id) + 1]
            self._increment_chain_after_first_edge(cut, abs_index)
            self._record_graph_event(
                "SKIPPED_NODE_CHAIN_REUSED",
                from_node=from_node_id,
                to_node=to_node_id,
                heading=self.heading_name(abs_index),
                chain=cut,
            )
            return

        # If the chain ends at an intermediate node with an empty continuation,
        # extend from that tail to the actual arrival node.
        tail = chain[-1]
        tail_exit = self._exit(tail, abs_index)
        if tail_exit.target is None and tail != from_node_id:
            # Existing edges after the first one were physically traversed.
            if len(chain) > 1:
                for node_id in chain[1:]:
                    # Do not increment an empty tail edge yet.
                    state = self._exit(node_id, abs_index)
                    if state.target is not None:
                        self._increment_departure(node_id, abs_index)
            if self._connect_direct(tail, abs_index, to_node_id):
                # Mark the newly extended tail->arrival edge as traversed once.
                self._increment_departure(tail, abs_index)
                self._record_graph_event(
                    "SKIPPED_NODE_CHAIN_EXTENDED",
                    from_node=from_node_id,
                    via_tail=tail,
                    to_node=to_node_id,
                    heading=self.heading_name(abs_index),
                    chain=chain + [to_node_id],
                )
                return

        # Protect the existing topology if the new observation is inconsistent.
        self._record_graph_event(
            "DIRECT_LINK_REJECTED_TO_PROTECT_GRAPH",
            from_node=from_node_id,
            to_node=to_node_id,
            heading=self.heading_name(abs_index),
            existing_target=source_exit.target,
            chain=chain,
        )

    def _split_known_edge_with_intermediate(self, from_node_id, abs_index, new_node_id):
        """Insert a newly detected junction into an already-known corridor.

        Example: an earlier pass stored A -- B directly because the side branch
        at X was missed. A later pass detects X before reaching B. The correct
        topology is A -- X -- B; rejecting X leaves the graph inconsistent and
        can make frontier routing oscillate forever.
        """
        if not bool(getattr(config, "ENABLE_INTERMEDIATE_NODE_EDGE_SPLIT", True)):
            return False
        if from_node_id not in self.nodes or new_node_id not in self.nodes:
            return False
        if from_node_id == new_node_id:
            return False

        abs_index %= 4
        opposite = self.opposite_index(abs_index)
        source_exit = self._exit(from_node_id, abs_index)
        old_target_id = source_exit.target
        if old_target_id is None or old_target_id == new_node_id:
            return False
        if old_target_id not in self.nodes:
            return False

        old_target_back = self._exit(old_target_id, opposite)
        # Split only a mutually confirmed direct edge A<->B.
        if old_target_back.target != from_node_id:
            return False

        new_back = self._exit(new_node_id, opposite)
        new_forward = self._exit(new_node_id, abs_index)
        # A fresh intermediate node must not already contradict this corridor.
        if new_back.target not in (None, from_node_id):
            return False
        if new_forward.target not in (None, old_target_id):
            return False

        inherited_visits = max(source_exit.visits, old_target_back.visits)

        # A <-> X
        source_exit.target = new_node_id
        new_back.target = from_node_id
        # X <-> B
        new_forward.target = old_target_id
        old_target_back.target = new_node_id

        for state in (source_exit, new_back, new_forward, old_target_back):
            state.visits = max(state.visits, inherited_visits)
            state.blocked = False
            state.seen_open_count = max(state.seen_open_count, 1)
            state.miss_count = 0

        self._record_graph_event(
            "INTERMEDIATE_NODE_EDGE_SPLIT",
            from_node=from_node_id,
            inserted_node=new_node_id,
            old_target=old_target_id,
            heading=self.heading_name(abs_index),
            inherited_visits=inherited_visits,
        )
        return True

    def _increment_departure(self, node_id, abs_index):
        exit_state = self._exit(node_id, abs_index)
        target_id = exit_state.target

        if target_id is None:
            exit_state.visits += 1
            exit_state.blocked = False
            return exit_state.visits

        opposite = self.opposite_index(abs_index)
        target_exit = self._exit(target_id, opposite)
        new_visits = max(exit_state.visits, target_exit.visits) + 1
        exit_state.visits = new_visits
        target_exit.visits = new_visits
        exit_state.blocked = False
        target_exit.blocked = False
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

        if (
            previous_node is not None
            and incoming_abs_dir is not None
            and previous_node != node_id
        ):
            split_done = False
            if is_new:
                split_done = self._split_known_edge_with_intermediate(
                    previous_node, incoming_abs_dir, node_id
                )
            if not split_done:
                self._link_nodes(previous_node, incoming_abs_dir, node_id)

        if (
            self.root_decision_node_id is None
            and previous_node == self.start_node_id
            and incoming_abs_dir is not None
        ):
            self.root_decision_node_id = node_id
            self.root_entry_abs_dir = self.opposite_index(incoming_abs_dir)

        self.current_node_id = node_id

        # V4 traversal stack: preserve the ACTUAL route through graph loops.
        # The old code collapsed the stack back to an earlier occurrence of a
        # known node. In the J12 case that discarded J12 even though J12 still
        # owned an unexplored branch, forcing global BFS and causing oscillation.
        if not self.dfs_stack:
            self.dfs_stack.append(node_id)
        elif self.dfs_stack[-1] != node_id:
            if len(self.dfs_stack) >= 2 and self.dfs_stack[-2] == node_id:
                # Normal one-edge DFS backtrack.
                self.dfs_stack.pop()
            else:
                if node_id in self.dfs_stack:
                    self._record_graph_event(
                        "LOOP_REVISIT_STACK_PRESERVED",
                        node=node_id,
                        previous_node=previous_node,
                    )
                self.dfs_stack.append(node_id)
                max_len = int(getattr(config, "DFS_STACK_MAX_LEN", 128))
                if max_len > 0 and len(self.dfs_stack) > max_len:
                    self.dfs_stack = self.dfs_stack[-max_len:]

        self.pending_from_node = None
        self.pending_abs_dir = None
        return node_id, is_new

    # ========================================================
    # FRONTIERS / GRAPH ROUTING
    # ========================================================

    def _physical_candidates(self, front_open, left_open, right_open, allow_back=True):
        relative_candidates = []
        if front_open:
            relative_candidates.append("FRONT")
        if left_open:
            relative_candidates.append("LEFT")
        if right_open:
            relative_candidates.append("RIGHT")
        if allow_back:
            relative_candidates.append("BACK")

        result = []
        seen = set()
        for relative in relative_candidates:
            abs_index = self.absolute_index(relative)
            if abs_index in seen:
                continue
            seen.add(abs_index)
            result.append((relative, abs_index))
        return result

    def _update_frontier_observations(self, candidates):
        node = self.nodes[self.current_node_id]
        observed_abs = {abs_index for _, abs_index in candidates}

        # Register / refresh every physically observed opening.
        for _, abs_index in candidates:
            state = self._exit(self.current_node_id, abs_index)
            state.seen_open_count += 1
            state.miss_count = 0
            state.blocked = False

        # Only untraversed, unlinked exits can be stale frontiers.  Known graph
        # corridors are never retired because one stopped scan failed to see it.
        retire_enabled = bool(getattr(config, "ENABLE_STALE_FRONTIER_RETIRE", True))
        miss_limit = int(getattr(config, "FRONTIER_STALE_MISS_LIMIT", 3))
        if not retire_enabled or miss_limit <= 0:
            return

        for abs_index, state in list(node.exits.items()):
            if abs_index in observed_abs:
                continue
            if state.visits != 0 or state.target is not None or state.blocked:
                continue
            state.miss_count += 1
            if state.miss_count >= miss_limit:
                state.blocked = True
                self._record_graph_event(
                    "STALE_FRONTIER_RETIRED",
                    node=self.current_node_id,
                    heading=self.heading_name(abs_index),
                    misses=state.miss_count,
                )

    @staticmethod
    def _is_frontier_state(state):
        return (
            state.visits == 0
            and state.target is None
            and not state.blocked
        )

    def frontier_exits(self, node_id=None):
        if node_id is None:
            node_id = self.current_node_id
        if node_id is None or node_id not in self.nodes:
            return []
        result = []
        for abs_index, state in self.nodes[node_id].exits.items():
            if self._is_frontier_state(state):
                result.append(abs_index % 4)
        return sorted(result)

    def all_frontiers(self):
        result = []
        for node_id in sorted(self.nodes, key=lambda n: int(n[1:]) if n[1:].isdigit() else n):
            for abs_index in self.frontier_exits(node_id):
                result.append((node_id, abs_index))
        return result

    def describe_frontiers(self):
        items = self.all_frontiers()
        if not items:
            return "NONE"
        return " | ".join(
            f"{node_id}.{self.heading_name(abs_index)}"
            for node_id, abs_index in items
        )

    def _graph_neighbors(self, node_id):
        neighbors = []
        if node_id not in self.nodes:
            return neighbors
        for abs_index, state in self.nodes[node_id].exits.items():
            if state.target is None or state.target not in self.nodes:
                continue
            neighbors.append((state.target, abs_index % 4))
        return neighbors

    def _shortest_path(self, start_id, target_id, allowed_first_abs=None):
        if start_id == target_id:
            return [start_id]
        if start_id not in self.nodes or target_id not in self.nodes:
            return None

        allowed_first_abs = None if allowed_first_abs is None else set(allowed_first_abs)
        q = deque([[start_id]])
        visited = {start_id}

        while q:
            path = q.popleft()
            node_id = path[-1]
            for nxt, abs_index in self._graph_neighbors(node_id):
                if len(path) == 1 and allowed_first_abs is not None:
                    if abs_index not in allowed_first_abs:
                        continue
                if nxt in visited:
                    continue
                new_path = path + [nxt]
                if nxt == target_id:
                    return new_path
                visited.add(nxt)
                q.append(new_path)
        return None

    def _abs_to_target(self, from_node_id, to_node_id, allowed_abs=None):
        allowed_abs = None if allowed_abs is None else set(allowed_abs)
        for abs_index, state in self.nodes[from_node_id].exits.items():
            if state.target != to_node_id:
                continue
            if allowed_abs is not None and abs_index not in allowed_abs:
                continue
            return abs_index % 4
        return None

    def _preferred_stack_frontier_target(self):
        # Nearest ancestor with an unexplored branch.
        if not self.dfs_stack:
            return None
        for node_id in reversed(self.dfs_stack[:-1]):
            if self.frontier_exits(node_id):
                return node_id
        return None

    def _nearest_reachable_frontier_path(self, allowed_first_abs):
        """BFS until reaching any node that owns an active frontier."""
        start = self.current_node_id
        if start is None:
            return None

        q = deque([[start]])
        visited = {start}
        allowed_first_abs = set(allowed_first_abs)

        while q:
            path = q.popleft()
            node_id = path[-1]
            if node_id != start and self.frontier_exits(node_id):
                return path

            for nxt, abs_index in self._graph_neighbors(node_id):
                if len(path) == 1 and abs_index not in allowed_first_abs:
                    continue
                if nxt in visited:
                    continue
                visited.add(nxt)
                q.append(path + [nxt])
        return None

    def _frontier_signature(self):
        return tuple(self.all_frontiers())

    def _route_attempt_key(self, abs_index, frontier_signature=None):
        if frontier_signature is None:
            frontier_signature = self._frontier_signature()
        return (frontier_signature, self.current_node_id, abs_index % 4)

    def _repeated_route_abs(self, candidates, frontier_signature):
        if not bool(getattr(config, "ENABLE_ROUTE_LOOP_BREAK", True)):
            return set()
        limit = max(1, int(getattr(config, "ROUTE_REPEAT_LIMIT", 1)))
        blocked = set()
        for _, abs_index in candidates:
            key = self._route_attempt_key(abs_index, frontier_signature)
            if self.route_attempt_counts.get(key, 0) >= limit:
                blocked.add(abs_index % 4)
        return blocked

    # ========================================================
    # FRONTIER-AWARE DFS DECISION
    # ========================================================

    def plan_direction(self, front_open, left_open, right_open):
        if self.current_node_id is None:
            raise RuntimeError("No current node. Call arrive_at_decision_point()")

        allow_back = self.current_node_id != self.start_node_id or bool(
            self.nodes[self.current_node_id].exits
        )
        candidates = self._physical_candidates(
            front_open,
            left_open,
            right_open,
            allow_back=allow_back,
        )
        self._update_frontier_observations(candidates)

        preference_rank = {
            name: index
            for index, name in enumerate(config.EXPLORATION_PREFERENCE)
        }
        scored = []
        for relative, abs_index in candidates:
            state = self._exit(self.current_node_id, abs_index)
            scored.append((
                state.visits,
                preference_rank.get(relative, 99),
                relative,
                abs_index,
                state,
            ))
        scored.sort(key=lambda item: (item[0], item[1]))

        # 1) Always take a physically confirmed local frontier first.
        local_unvisited = [
            item for item in scored
            if self._is_frontier_state(item[4])
        ]
        if local_unvisited:
            visits, _, relative, abs_index, _ = local_unvisited[0]
            self.completed = False
            return ExplorationDecision(
                direction=relative,
                node_id=self.current_node_id,
                reason="UNVISITED_EXIT",
                visits_before=visits,
                absolute_heading=self.heading_name(abs_index),
            )

        # 1b) IMPORTANT: a known junction can remember an unexplored side branch
        # even when the current stopped Sharp scan misses that opening by a few
        # centimetres.  The V2 planner accidentally ignored a frontier owned by
        # the *current* node unless it appeared in the current sensor candidates,
        # then immediately routed BACK through a known corridor.  That produced
        # exactly the observed "came out of a dead end, then went back the same
        # way instead of taking the branch I had not tried" behaviour.
        #
        # A remembered frontier was already created from a multi-sample stopped
        # scan on an earlier visit.  For LEFT/RIGHT we can safely turn toward it;
        # after the turn the front ToF still prevents driving into a real wall.
        if bool(getattr(config, "ENABLE_REMEMBERED_LOCAL_FRONTIER", True)):
            observed_abs = {abs_index for _, abs_index in candidates}
            min_seen = int(getattr(config, "REMEMBERED_FRONTIER_MIN_SEEN", 1))
            remembered = []
            for abs_index in self.frontier_exits(self.current_node_id):
                if abs_index in observed_abs:
                    continue
                state = self._exit(self.current_node_id, abs_index)
                if state.seen_open_count < min_seen:
                    continue
                relative = self.relative_for_absolute(abs_index)
                # Never force straight ahead against the current front scan.
                # The problematic missed branch on a revisit is normally LEFT
                # or RIGHT after returning from a dead end.
                if relative not in ("LEFT", "RIGHT"):
                    continue
                remembered.append((
                    preference_rank.get(relative, 99),
                    relative,
                    abs_index,
                    state,
                ))

            if remembered:
                remembered.sort(key=lambda item: item[0])
                _, relative, abs_index, state = remembered[0]
                self.completed = False
                return ExplorationDecision(
                    direction=relative,
                    node_id=self.current_node_id,
                    reason="REMEMBERED_LOCAL_FRONTIER",
                    visits_before=state.visits,
                    absolute_heading=self.heading_name(abs_index),
                )

        global_frontiers = self.all_frontiers()

        # 2) No active frontier anywhere -> true global completion.
        if not global_frontiers:
            self.completed = True
            return ExplorationDecision(
                direction="COMPLETE",
                node_id=self.current_node_id,
                reason="ALL_FRONTIERS_EXPLORED",
                visits_before=0,
                absolute_heading=self.heading_name(),
            )

        frontier_signature = tuple(global_frontiers)
        physically_allowed_abs = {abs_index for _, abs_index in candidates}
        repeated_abs = self._repeated_route_abs(candidates, frontier_signature)
        allowed_first_abs = set(physically_allowed_abs) - repeated_abs

        # If exactly the same ROUTE_* departure has already been tried with the
        # same frontier set and made no progress, do not U-turn through it again.
        # Prefer continuing straight through the known junction when physically
        # open; this is the situation seen in the supplied J3/J9 loop log.
        if repeated_abs and front_open:
            front_abs = self.absolute_index("FRONT")
            if front_abs in allowed_first_abs:
                front_state = self._exit(self.current_node_id, front_abs)
                if front_state.visits > 0 or front_state.target is not None:
                    self.completed = False
                    return ExplorationDecision(
                        direction="FRONT",
                        node_id=self.current_node_id,
                        reason="LOOP_BREAK_CONTINUE_FRONT",
                        visits_before=front_state.visits,
                        absolute_heading=self.heading_name(front_abs),
                    )

        # If loop protection temporarily removed every first step, fall back to
        # the physical candidates; the later recovery stage will still avoid an
        # immediate false COMPLETE.
        if not allowed_first_abs:
            allowed_first_abs = set(physically_allowed_abs)

        # 3) Prefer classic DFS backtracking along the current stack toward the
        # nearest ancestor that still owns a frontier.
        stack_target = self._preferred_stack_frontier_target()
        if stack_target is not None:
            if len(self.dfs_stack) >= 2:
                parent = self.dfs_stack[-2]
                abs_to_parent = self._abs_to_target(
                    self.current_node_id,
                    parent,
                    allowed_abs=allowed_first_abs,
                )
                if abs_to_parent is not None:
                    relative = self.relative_for_absolute(abs_to_parent)
                    visits = self._exit(self.current_node_id, abs_to_parent).visits
                    self.completed = False
                    return ExplorationDecision(
                        direction=relative,
                        node_id=self.current_node_id,
                        reason="DFS_BACKTRACK_TO_FRONTIER",
                        visits_before=visits,
                        absolute_heading=self.heading_name(abs_to_parent),
                    )

            path = self._shortest_path(
                self.current_node_id,
                stack_target,
                allowed_first_abs=allowed_first_abs,
            )
            if path and len(path) >= 2:
                abs_index = self._abs_to_target(
                    self.current_node_id,
                    path[1],
                    allowed_abs=allowed_first_abs,
                )
                if abs_index is not None:
                    relative = self.relative_for_absolute(abs_index)
                    visits = self._exit(self.current_node_id, abs_index).visits
                    self.completed = False
                    return ExplorationDecision(
                        direction=relative,
                        node_id=self.current_node_id,
                        reason="ROUTE_TO_DFS_FRONTIER",
                        visits_before=visits,
                        absolute_heading=self.heading_name(abs_index),
                    )

        # 4) Loops / merged topology can leave a frontier outside the active DFS
        # stack. Route to the nearest reachable frontier node in the graph.
        path = self._nearest_reachable_frontier_path(allowed_first_abs)
        if path and len(path) >= 2:
            abs_index = self._abs_to_target(
                self.current_node_id,
                path[1],
                allowed_abs=allowed_first_abs,
            )
            if abs_index is not None:
                relative = self.relative_for_absolute(abs_index)
                visits = self._exit(self.current_node_id, abs_index).visits
                self.completed = False
                return ExplorationDecision(
                    direction=relative,
                    node_id=self.current_node_id,
                    reason="ROUTE_TO_NEAREST_FRONTIER",
                    visits_before=visits,
                    absolute_heading=self.heading_name(abs_index),
                )

        # 5) Frontiers exist but current graph/sensor snapshot cannot route to
        # one. Do not falsely COMPLETE. Use any physically available known edge
        # as a recovery transit; BACK naturally wins preference when it is the
        # only route from a dead end.
        known_transit = []
        for item in scored:
            visits, rank, relative, abs_index, state = item
            if state.target is not None and abs_index in allowed_first_abs:
                known_transit.append((visits, rank, relative, abs_index, state))
        # If every known transit was loop-blocked, allow them as a final safety
        # fallback rather than falsely declaring completion.
        if not known_transit:
            for item in scored:
                visits, rank, relative, abs_index, state = item
                if state.target is not None:
                    known_transit.append((visits, rank, relative, abs_index, state))
        if known_transit:
            # Prefer BACK for recovery, then lower traversal count / preference.
            known_transit.sort(
                key=lambda item: (
                    0 if item[2] == "BACK" else 1,
                    item[0],
                    item[1],
                )
            )
            visits, _, relative, abs_index, _ = known_transit[0]
            self.completed = False
            return ExplorationDecision(
                direction=relative,
                node_id=self.current_node_id,
                reason="FRONTIER_RECOVERY_TRANSIT",
                visits_before=visits,
                absolute_heading=self.heading_name(abs_index),
            )

        # This is a graph-integrity failure, not successful completion.  Keep the
        # reason explicit so logs distinguish it from ALL_FRONTIERS_EXPLORED.
        self.completed = False
        return ExplorationDecision(
            direction="COMPLETE",
            node_id=self.current_node_id,
            reason="FRONTIERS_EXIST_BUT_UNREACHABLE",
            visits_before=0,
            absolute_heading=self.heading_name(),
        )

    def commit_decision(self, decision):
        if decision.direction == "COMPLETE":
            # Only the explicit global-completion reason is a real completion.
            self.completed = decision.reason == "ALL_FRONTIERS_EXPLORED"
            return

        abs_index = self.absolute_index(decision.direction)

        # V4 anti-oscillation: remember directed graph-routing attempts while
        # the frontier set is unchanged. A second visit to the same node will
        # therefore choose an alternative instead of repeating the same U-turn.
        route_reasons = {
            "DFS_BACKTRACK_TO_FRONTIER",
            "ROUTE_TO_DFS_FRONTIER",
            "ROUTE_TO_NEAREST_FRONTIER",
            "FRONTIER_RECOVERY_TRANSIT",
        }
        if decision.reason in route_reasons:
            signature = self._frontier_signature()
            key = self._route_attempt_key(abs_index, signature)
            self.route_attempt_counts[key] = self.route_attempt_counts.get(key, 0) + 1

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
            "frontiers_remaining": len(self.all_frontiers()),
        })
        self.heading_index = abs_index
        self.completed = False

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
            suffix = ""
            if self._is_frontier_state(exit_state):
                suffix = "[FRONTIER]"
            elif exit_state.blocked:
                suffix = "[STALE]"
            parts.append(
                f"{HEADINGS[abs_index]}:{exit_state.visits}->{target}{suffix}"
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
            "frontiers": [
                {"node": node_id, "heading": self.heading_name(abs_index)}
                for node_id, abs_index in self.all_frontiers()
            ],
            "dfs_stack": list(self.dfs_stack),
            "nodes": {},
            "route_history": self.route_history,
            "graph_events": self.graph_events,
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
                        "seen_open_count": exit_state.seen_open_count,
                        "miss_count": exit_state.miss_count,
                        "blocked": exit_state.blocked,
                        "frontier": self._is_frontier_state(exit_state),
                    }
                    for abs_index, exit_state in node.exits.items()
                },
            }

        with open(filepath, "w", encoding="utf-8") as file:
            json.dump(data, file, indent=2, ensure_ascii=False)

"""
RoboMaster Auto Pickup + Manual Keyboard Fallback
==================================================

This standalone test does NOT contain the maze solver. It combines:
    - Automatic ToF approach, pickup, and lift/lower verification.
    - Manual keyboard control that can take over at any time.

Sequence
    1. Reset: move arm to HOME, close gripper, then move to PICK position.
    2. Wait for a valid ToF target and approach it.
    3. Stop at PICKUP_TARGET_CM, open gripper, then close it.
    4. Measure the object at PICK position (D1).
    5. Lift the arm and verify that the object leaves the ToF beam (D2).
    6. Lower the arm and verify that the object returns to the ToF beam (D3).
    7. Success: lift again and enter MANUAL mode while holding the object.
       Failure: open, back away, and automatically retry from Reset.

Mode switching
    - Press M during AUTO to stop the chassis and enter MANUAL.
    - Press M during MANUAL to restart AUTO from Reset.
    - Press SPACE during AUTO for emergency stop + MANUAL takeover.
    - Press ESC to stop and disconnect.

Manual keys
    W/S       forward/backward (hold)
    A/D       strafe left/right (hold)
    Q/E       rotate left/right (hold)
    SPACE     emergency stop
    1/2/3     HOME / PICK / LIFT arm positions
    O/P       open / close gripper
    R         reset arm: HOME -> close -> PICK
    M         return to AUTO
    H         print keyboard help
    ESC       stop and exit

    Forward motion is protected by ToF. If ToF is unavailable, or an object is
    at/below MANUAL_FRONT_STOP_CM, W is blocked. Hold SHIFT+W only when an
    intentional safety override is necessary.

Safety rules
    - AUTO never moves while ToF data is missing/stale/invalid.
    - MANUAL blocks forward motion without ToF unless SHIFT+W is held.
    - A ToF NO_RETURN result is accepted during lift only when callbacks are
      still arriving. A stale sensor is never treated as pickup success.
    - Press Ctrl+C at any time to stop the chassis and disconnect safely.

Requires:
    pip install robomaster pynput
"""

from __future__ import annotations

import math
import statistics
import threading
import time
from collections import Counter, deque
from dataclasses import dataclass
from typing import Deque, Dict, List, Optional, Set, Tuple

try:
    from robomaster import robot
except ModuleNotFoundError as exc:
    raise SystemExit(
        "RoboMaster SDK is not installed. Install it in this Python environment first."
    ) from exc

try:
    from pynput import keyboard as pynput_keyboard
except ImportError as exc:
    raise SystemExit(
        "Could not load pynput keyboard control. Install it with 'pip install "
        "pynput' and run this program in a normal desktop session."
    ) from exc


# ============================================================
# CONFIGURATION - tune these values for your robot
# ============================================================

CONNECTION = "ap"
ENABLE_MOTION = True
START_MODE = "AUTO"  # AUTO or MANUAL

# Arm coordinates confirmed by the supplied arm test.
HOME_POSITION = (0, 0)
PICK_POSITION = (0, 50)
LIFT_POSITION = (0, 120)

# ToF target distance measured from the ToF lens, not from the gripper jaw.
# If 4 cm is not the physical pickup point on your robot, calibrate this value.
PICKUP_TARGET_CM = 4.0
PICKUP_STOP_TOLERANCE_CM = 0.7

# The robot waits safely instead of driving toward a very distant wall.
# Put the test object within this range or increase the value carefully.
OBJECT_DETECT_MAX_CM = 100.0

# Approach speeds (RoboMaster chassis x speed is in m/s).
APPROACH_FAST_SPEED = 0.08
APPROACH_SLOW_SPEED = 0.04
APPROACH_CRAWL_SPEED = 0.02
APPROACH_SLOW_AT_CM = 18.0
APPROACH_CRAWL_AT_CM = 8.0
APPROACH_TIMEOUT_SEC = 25.0
DRIVE_COMMAND_TIMEOUT_SEC = 0.20
CONTROL_LOOP_SEC = 0.05

# ToF configuration.
TOF_FREQUENCY_HZ = 20
TOF_FILTER_SIZE = 5
TOF_MIN_VALID_MM = 20.0
TOF_MAX_VALID_MM = 4000.0
TOF_STALE_SEC = 0.60
TOF_STARTUP_TIMEOUT_SEC = 5.0
TOF_LOST_ABORT_SEC = 2.0
TOF_PRINT_INTERVAL_SEC = 0.25

# Mechanical settling times.
ARM_SETTLE_SEC = 0.35
GRIPPER_OPEN_SEC = 1.5
GRIPPER_CLOSE_SEC = 1.5

# Verification uses only fresh callbacks collected after each arm movement.
VERIFY_WINDOW_SEC = 0.80
VERIFY_MIN_VALID_SAMPLES = 4
VERIFY_MIN_CALLBACKS = 5

# D1 must still be close after the gripper closes.
GRABBED_OBJECT_MAX_CM = 10.0

# D2 must be clearly farther than D1, or most fresh callbacks must report
# NO_RETURN while the callback remains alive.
LIFT_CLEAR_DELTA_CM = 8.0
NO_RETURN_SUCCESS_RATIO = 0.60

# D3 should return close to D1 after lowering the held object.
RETURN_OBJECT_MAX_CM = 10.0
RETURN_MATCH_TOLERANCE_CM = 5.0

# Automatic recovery. Set MAX_PICKUP_ATTEMPTS = 0 for unlimited retries.
MAX_PICKUP_ATTEMPTS = 5
BACKOFF_AFTER_FAILURE_M = 0.05
BACKOFF_SPEED_MPS = 0.05
RETRY_SETTLE_SEC = 0.50

# Keep the object lifted on success. Change to True only if the test should
# release it automatically at the end.
RELEASE_AFTER_SUCCESS = False

# After a successful automatic pickup, stay connected and enter MANUAL so the
# user can move the robot, lower the arm, or release the object.
AUTO_SUCCESS_ENTERS_MANUAL = True
AUTO_FAILURE_ENTERS_MANUAL = True

# Manual chassis settings.
MANUAL_FORWARD_SPEED = 0.08       # m/s
MANUAL_STRAFE_SPEED = 0.08        # m/s
MANUAL_TURN_SPEED = 20.0          # deg/s
MANUAL_Y_SIGN = 1                 # set -1 if A/D are reversed on your robot
MANUAL_Z_SIGN = 1                 # set -1 if Q/E are reversed on your robot
MANUAL_FRONT_STOP_CM = 3.5
MANUAL_BLOCK_FORWARD_WITHOUT_TOF = True
MANUAL_LOOP_SEC = 0.05
MANUAL_PRINT_INTERVAL_SEC = 0.25


# ============================================================
# Keyboard state and mode requests
# ============================================================

class ManualModeRequested(Exception):
    """Raised inside AUTO when M or SPACE requests a safe manual takeover."""


class ProgramExitRequested(Exception):
    """Raised when ESC requests a clean shutdown."""


class KeyboardControls:
    """Thread-safe key state for hold-to-drive and edge-triggered commands."""

    DISCRETE_COMMAND_KEYS = {"1", "2", "3", "o", "p", "r", "h"}

    def __init__(self):
        self._lock = threading.Lock()
        self._pressed: Set[str] = set()
        self._commands: Deque[str] = deque()
        self._mode_toggle = threading.Event()
        self._emergency_stop = threading.Event()
        self._exit = threading.Event()
        self._listener = None

    @staticmethod
    def _key_token(key) -> Optional[str]:
        char = getattr(key, "char", None)
        if char:
            return str(char).lower()

        if key == pynput_keyboard.Key.space:
            return "space"
        if key == pynput_keyboard.Key.esc:
            return "esc"
        if key in (
            pynput_keyboard.Key.shift,
            pynput_keyboard.Key.shift_l,
            pynput_keyboard.Key.shift_r,
        ):
            return "shift"
        return None

    def _on_press(self, key):
        token = self._key_token(key)
        if token is None:
            return

        with self._lock:
            first_press = token not in self._pressed
            self._pressed.add(token)

            # Ignore OS key-repeat for commands that must fire once per press.
            if not first_press:
                return

            if token == "esc":
                self._exit.set()
            elif token == "m":
                self._mode_toggle.set()
            elif token == "space":
                self._emergency_stop.set()
            elif token in self.DISCRETE_COMMAND_KEYS:
                self._commands.append(token)

    def _on_release(self, key):
        token = self._key_token(key)
        if token is None:
            return
        with self._lock:
            self._pressed.discard(token)

    def start(self) -> None:
        try:
            self._listener = pynput_keyboard.Listener(
                on_press=self._on_press,
                on_release=self._on_release,
            )
            self._listener.start()
        except Exception as exc:
            raise RuntimeError(
                "Could not start keyboard listener. Ensure this program is run "
                "in a desktop session where pynput can access the keyboard."
            ) from exc

    def stop(self) -> None:
        listener = self._listener
        self._listener = None
        if listener is None:
            return
        try:
            listener.stop()
            listener.join(timeout=1.0)
        except Exception:
            pass

    def is_pressed(self, token: str) -> bool:
        with self._lock:
            return token in self._pressed

    def pop_command(self) -> Optional[str]:
        with self._lock:
            if not self._commands:
                return None
            return self._commands.popleft()

    def clear_commands(self) -> None:
        with self._lock:
            self._commands.clear()

    def mode_toggle_pending(self) -> bool:
        return self._mode_toggle.is_set()

    def clear_mode_toggle(self) -> None:
        self._mode_toggle.clear()

    def emergency_stop_pending(self) -> bool:
        return self._emergency_stop.is_set()

    def clear_emergency_stop(self) -> None:
        self._emergency_stop.clear()

    def exit_requested(self) -> bool:
        return self._exit.is_set()


def check_auto_keyboard(controls: KeyboardControls) -> None:
    """Abort AUTO safely when the user requests manual control or exit."""
    if controls.exit_requested():
        raise ProgramExitRequested
    if (
        controls.mode_toggle_pending()
        or controls.emergency_stop_pending()
        or controls.is_pressed("space")
    ):
        raise ManualModeRequested


# ============================================================
# ToF reader
# ============================================================

@dataclass(frozen=True)
class ToFEvent:
    sequence: int
    status: str
    distance_cm: Optional[float]
    raw_mm: Optional[float]
    callback_time: Optional[float]


@dataclass
class ToFObservation:
    valid_values_cm: List[float]
    status_counts: Dict[str, int]
    callback_count: int

    @property
    def median_cm(self) -> Optional[float]:
        if not self.valid_values_cm:
            return None
        return float(statistics.median(self.valid_values_cm))

    @property
    def no_return_ratio(self) -> float:
        if self.callback_count <= 0:
            return 0.0
        return self.status_counts.get("NO_RETURN", 0) / self.callback_count


class ToFReader:
    """Thread-safe ToF callback storage with explicit sensor states."""

    def __init__(self):
        self._lock = threading.Lock()
        self._sequence = 0
        self._last_status = "STARTING"
        self._last_distance_cm: Optional[float] = None
        self._last_raw_mm: Optional[float] = None
        self._last_callback_time: Optional[float] = None
        self._recent_valid: Deque[Tuple[float, float]] = deque(
            maxlen=TOF_FILTER_SIZE
        )

    def callback(self, data):
        now = time.monotonic()
        status = "BAD_PACKET"
        distance_cm: Optional[float] = None
        raw_mm: Optional[float] = None

        try:
            if data is None or len(data) < 1 or data[0] is None:
                status = "BAD_PACKET"
            else:
                raw_mm = float(data[0])
                if raw_mm <= 0.0 or raw_mm > TOF_MAX_VALID_MM:
                    status = "NO_RETURN"
                elif raw_mm < TOF_MIN_VALID_MM:
                    status = "TOO_CLOSE"
                else:
                    status = "VALID"
                    distance_cm = raw_mm / 10.0
        except (TypeError, ValueError, IndexError):
            status = "BAD_PACKET"

        with self._lock:
            self._sequence += 1
            self._last_status = status
            self._last_distance_cm = distance_cm
            self._last_raw_mm = raw_mm
            self._last_callback_time = now
            if status == "VALID" and distance_cm is not None:
                self._recent_valid.append((now, distance_cm))

    def latest_event(self) -> ToFEvent:
        with self._lock:
            return ToFEvent(
                sequence=self._sequence,
                status=self._last_status,
                distance_cm=self._last_distance_cm,
                raw_mm=self._last_raw_mm,
                callback_time=self._last_callback_time,
            )

    def filtered_distance(self) -> Tuple[Optional[float], str, Optional[float]]:
        """Return (median_cm, status, raw_mm) using only recent valid data."""
        now = time.monotonic()
        with self._lock:
            if self._last_callback_time is None:
                return None, "STARTING", self._last_raw_mm

            if now - self._last_callback_time > TOF_STALE_SEC:
                return None, "STALE", self._last_raw_mm

            if self._last_status != "VALID":
                return None, self._last_status, self._last_raw_mm

            recent = [
                cm
                for timestamp, cm in self._recent_valid
                if now - timestamp <= TOF_STALE_SEC
            ]
            if not recent:
                return None, "STALE", self._last_raw_mm

            return float(statistics.median(recent)), "VALID", self._last_raw_mm

    def wait_for_first_callback(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self.latest_event().sequence > 0:
                return True
            time.sleep(0.02)
        return False

    def collect_fresh_observation(self, duration_sec: float) -> ToFObservation:
        """Collect only callback events produced after this function starts."""
        started = time.monotonic()
        initial = self.latest_event()
        last_sequence = initial.sequence
        valid_values: List[float] = []
        statuses: Counter = Counter()
        callback_count = 0

        while time.monotonic() - started < duration_sec:
            event = self.latest_event()
            if event.sequence != last_sequence:
                # At 20 Hz this normally advances by one. If the control thread
                # was briefly delayed, count the newest available event once.
                last_sequence = event.sequence
                callback_count += 1
                statuses[event.status] += 1
                if event.status == "VALID" and event.distance_cm is not None:
                    valid_values.append(event.distance_cm)
            time.sleep(0.01)

        return ToFObservation(
            valid_values_cm=valid_values,
            status_counts=dict(statuses),
            callback_count=callback_count,
        )


# ============================================================
# Hardware actions
# ============================================================

def stop_chassis(chassis) -> None:
    if chassis is None or not ENABLE_MOTION:
        return
    try:
        chassis.drive_speed(x=0.0, y=0.0, z=0.0, timeout=0.2)
    except Exception:
        pass


def move_arm(arm, position: Tuple[int, int], label: str) -> None:
    x, y = position
    print(f"[ARM] {label}: x={x}, y={y}")
    action = arm.moveto(x=x, y=y)
    action.wait_for_completed()
    time.sleep(ARM_SETTLE_SEC)


def open_gripper(gripper, label: str = "OPEN") -> None:
    print(f"[GRIPPER] {label}")
    gripper.open()
    time.sleep(GRIPPER_OPEN_SEC)


def close_gripper(gripper, label: str = "CLOSE") -> None:
    print(f"[GRIPPER] {label}")
    gripper.close()
    time.sleep(GRIPPER_CLOSE_SEC)


def reset_pickup_arm(arm, gripper) -> None:
    """Reset requested by the user: arm down, close, then use PICK height."""
    print("\n[RESET] Move down and close gripper")
    move_arm(arm, HOME_POSITION, "HOME / LOWEST")
    close_gripper(gripper, "RESET CLOSE")
    move_arm(arm, PICK_POSITION, "PICK POSITION")


def back_away(chassis) -> None:
    if not ENABLE_MOTION or BACKOFF_AFTER_FAILURE_M <= 0.0:
        return
    stop_chassis(chassis)
    print(f"[RECOVERY] Back away {BACKOFF_AFTER_FAILURE_M * 100:.1f} cm")
    action = chassis.move(
        x=-BACKOFF_AFTER_FAILURE_M,
        y=0.0,
        z=0.0,
        xy_speed=BACKOFF_SPEED_MPS,
    )
    action.wait_for_completed()
    stop_chassis(chassis)


# ============================================================
# Pickup logic
# ============================================================

def approach_speed(distance_cm: float) -> float:
    if distance_cm > APPROACH_SLOW_AT_CM:
        return APPROACH_FAST_SPEED
    if distance_cm > APPROACH_CRAWL_AT_CM:
        return APPROACH_SLOW_SPEED
    return APPROACH_CRAWL_SPEED


def format_raw_mm(raw_mm: Optional[float]) -> str:
    return "---" if raw_mm is None else f"{raw_mm:.0f}"


def approach_object(
    chassis,
    tof: ToFReader,
    controls: KeyboardControls,
) -> Optional[float]:
    """Approach until a stable ToF reading reaches PICKUP_TARGET_CM."""
    print(
        f"[APPROACH] Waiting for object within {OBJECT_DETECT_MAX_CM:.1f} cm; "
        f"target={PICKUP_TARGET_CM:.1f} cm"
    )

    approach_started: Optional[float] = None
    lost_since: Optional[float] = None
    last_print = 0.0

    while True:
        check_auto_keyboard(controls)
        now = time.monotonic()
        distance_cm, status, raw_mm = tof.filtered_distance()

        if status != "VALID" or distance_cm is None:
            stop_chassis(chassis)

            if lost_since is None:
                lost_since = now

            if now - last_print >= TOF_PRINT_INTERVAL_SEC:
                print(
                    f"[WAIT OBJECT] status={status} raw={format_raw_mm(raw_mm)} mm"
                )
                last_print = now

            # Before movement begins, wait safely for the object without using
            # input(). During movement, abort this attempt if ToF is lost.
            if (
                approach_started is not None
                and now - lost_since >= TOF_LOST_ABORT_SEC
            ):
                print("[APPROACH FAILED] ToF was lost while the robot was moving")
                return None

            time.sleep(CONTROL_LOOP_SEC)
            continue

        lost_since = None

        if distance_cm > OBJECT_DETECT_MAX_CM:
            stop_chassis(chassis)
            if now - last_print >= TOF_PRINT_INTERVAL_SEC:
                print(
                    f"[WAIT OBJECT] {distance_cm:.1f} cm is beyond the configured "
                    f"object range ({OBJECT_DETECT_MAX_CM:.1f} cm)"
                )
                last_print = now
            time.sleep(CONTROL_LOOP_SEC)
            continue

        if approach_started is None:
            approach_started = now
            print(f"[OBJECT DETECTED] {distance_cm:.1f} cm")

        if now - approach_started > APPROACH_TIMEOUT_SEC:
            stop_chassis(chassis)
            print("[APPROACH FAILED] Approach timeout")
            return None

        if distance_cm <= PICKUP_TARGET_CM:
            stop_chassis(chassis)
            observation = tof.collect_fresh_observation(0.40)
            check_auto_keyboard(controls)
            stable_cm = observation.median_cm

            if (
                stable_cm is not None
                and len(observation.valid_values_cm) >= 3
                and stable_cm <= PICKUP_TARGET_CM + PICKUP_STOP_TOLERANCE_CM
            ):
                print(
                    f"[TARGET REACHED] median={stable_cm:.1f} cm "
                    f"samples={len(observation.valid_values_cm)}"
                )
                return stable_cm

            print(
                "[TARGET CHECK] Reading was not stable; continuing carefully "
                f"(median={stable_cm}, callbacks={observation.callback_count})"
            )
            time.sleep(CONTROL_LOOP_SEC)
            continue

        speed = approach_speed(distance_cm)
        if ENABLE_MOTION:
            chassis.drive_speed(
                x=speed,
                y=0.0,
                z=0.0,
                timeout=DRIVE_COMMAND_TIMEOUT_SEC,
            )

        if now - last_print >= TOF_PRINT_INTERVAL_SEC:
            print(f"[APPROACH] ToF={distance_cm:5.1f} cm  speed={speed:.3f} m/s")
            last_print = now

        time.sleep(CONTROL_LOOP_SEC)


def measure_down_object(tof: ToFReader, label: str) -> ToFObservation:
    observation = tof.collect_fresh_observation(VERIFY_WINDOW_SEC)
    print(
        f"[{label}] median={observation.median_cm} cm "
        f"valid={len(observation.valid_values_cm)} "
        f"callbacks={observation.callback_count} "
        f"states={observation.status_counts}"
    )
    return observation


def lift_is_clear(before_cm: float, observation: ToFObservation) -> bool:
    if observation.callback_count < VERIFY_MIN_CALLBACKS:
        print("[VERIFY LIFT] FAIL: callback is missing or too slow")
        return False

    lifted_cm = observation.median_cm
    far_enough = (
        lifted_cm is not None
        and len(observation.valid_values_cm) >= VERIFY_MIN_VALID_SAMPLES
        and lifted_cm >= before_cm + LIFT_CLEAR_DELTA_CM
    )

    alive_no_return = (
        observation.no_return_ratio >= NO_RETURN_SUCCESS_RATIO
        and (
            lifted_cm is None
            or lifted_cm >= before_cm + LIFT_CLEAR_DELTA_CM
        )
    )

    if far_enough:
        print(
            f"[VERIFY LIFT] PASS: {before_cm:.1f} -> {lifted_cm:.1f} cm"
        )
        return True

    if alive_no_return:
        print(
            "[VERIFY LIFT] PASS: ToF callback is alive and target is out of range "
            f"(NO_RETURN={observation.no_return_ratio:.0%})"
        )
        return True

    print(
        "[VERIFY LIFT] FAIL: object did not clearly leave the ToF beam "
        f"(before={before_cm:.1f}, after={lifted_cm})"
    )
    return False


def object_returned(before_cm: float, observation: ToFObservation) -> bool:
    after_cm = observation.median_cm
    if observation.callback_count < VERIFY_MIN_CALLBACKS:
        print("[VERIFY RETURN] FAIL: callback is missing or too slow")
        return False

    returned = (
        after_cm is not None
        and len(observation.valid_values_cm) >= VERIFY_MIN_VALID_SAMPLES
        and after_cm <= RETURN_OBJECT_MAX_CM
        and abs(after_cm - before_cm) <= RETURN_MATCH_TOLERANCE_CM
    )

    if returned:
        print(
            f"[VERIFY RETURN] PASS: D1={before_cm:.1f} cm, D3={after_cm:.1f} cm"
        )
        return True

    print(
        "[VERIFY RETURN] FAIL: object did not return near its original distance "
        f"(D1={before_cm:.1f}, D3={after_cm})"
    )
    return False


def perform_grab_and_verify(
    arm,
    gripper,
    tof: ToFReader,
    controls: KeyboardControls,
) -> bool:
    """Open, grab, and run the down -> lift -> down ToF verification."""
    check_auto_keyboard(controls)
    open_gripper(gripper, "OPEN FOR OBJECT")
    check_auto_keyboard(controls)
    close_gripper(gripper, "CLOSE / GRAB")
    check_auto_keyboard(controls)

    down_before = measure_down_object(tof, "D1 AFTER GRAB")
    check_auto_keyboard(controls)
    before_cm = down_before.median_cm
    if (
        before_cm is None
        or len(down_before.valid_values_cm) < VERIFY_MIN_VALID_SAMPLES
        or before_cm > GRABBED_OBJECT_MAX_CM
    ):
        print("[GRAB CHECK] FAIL: no stable close object after closing gripper")
        return False

    move_arm(arm, LIFT_POSITION, "LIFT FOR D2")
    check_auto_keyboard(controls)
    lifted = tof.collect_fresh_observation(VERIFY_WINDOW_SEC)
    check_auto_keyboard(controls)
    print(
        f"[D2 LIFTED] median={lifted.median_cm} cm "
        f"valid={len(lifted.valid_values_cm)} "
        f"callbacks={lifted.callback_count} states={lifted.status_counts}"
    )
    lift_clear = lift_is_clear(before_cm, lifted)

    # Always lower for the paired D1 -> D2 -> D3 check, even when D2 failed.
    move_arm(arm, PICK_POSITION, "LOWER FOR D3")
    check_auto_keyboard(controls)
    down_after = measure_down_object(tof, "D3 LOWERED")
    check_auto_keyboard(controls)
    returned = object_returned(before_cm, down_after)

    return lift_clear and returned


def automatic_recovery(
    chassis,
    arm,
    gripper,
    controls: KeyboardControls,
) -> None:
    """No user input: release, back away, then the loop performs Reset."""
    stop_chassis(chassis)
    check_auto_keyboard(controls)
    print("[RECOVERY] Release object and automatically retry")
    try:
        move_arm(arm, PICK_POSITION, "RECOVERY PICK POSITION")
    except Exception as exc:
        print(f"[RECOVERY WARNING] Could not move arm to PICK: {exc}")
    check_auto_keyboard(controls)
    open_gripper(gripper, "RECOVERY OPEN")
    check_auto_keyboard(controls)
    back_away(chassis)
    check_auto_keyboard(controls)
    time.sleep(RETRY_SETTLE_SEC)


def run_pickup_test(
    chassis,
    arm,
    gripper,
    tof: ToFReader,
    controls: KeyboardControls,
) -> bool:
    attempt = 0

    while MAX_PICKUP_ATTEMPTS == 0 or attempt < MAX_PICKUP_ATTEMPTS:
        check_auto_keyboard(controls)
        attempt += 1
        maximum = "unlimited" if MAX_PICKUP_ATTEMPTS == 0 else str(MAX_PICKUP_ATTEMPTS)
        print("\n" + "=" * 60)
        print(f" PICKUP ATTEMPT {attempt}/{maximum}")
        print("=" * 60)

        stop_chassis(chassis)
        reset_pickup_arm(arm, gripper)
        check_auto_keyboard(controls)
        reached_cm = approach_object(chassis, tof, controls)

        if reached_cm is None:
            check_auto_keyboard(controls)
            automatic_recovery(chassis, arm, gripper, controls)
            check_auto_keyboard(controls)
            continue

        stop_chassis(chassis)
        print(f"[PICKUP] Start grabbing at {reached_cm:.1f} cm")

        if perform_grab_and_verify(arm, gripper, tof, controls):
            print("\n[PICKUP SUCCESS] D1 -> D2 -> D3 verification passed")
            move_arm(arm, LIFT_POSITION, "FINAL CARRY LIFT")
            check_auto_keyboard(controls)

            if RELEASE_AFTER_SUCCESS:
                open_gripper(gripper, "FINAL RELEASE")
            else:
                print("[FINISH] Object remains gripped and lifted")

            return True

        print("\n[PICKUP FAILED] Verification failed")
        check_auto_keyboard(controls)
        automatic_recovery(chassis, arm, gripper, controls)
        check_auto_keyboard(controls)

    print(f"[STOP] Pickup failed after {MAX_PICKUP_ATTEMPTS} attempts")
    return False


# ============================================================
# Manual keyboard fallback
# ============================================================

def print_manual_help() -> None:
    print("\n" + "=" * 66)
    print(" MANUAL KEYBOARD CONTROL")
    print("=" * 66)
    print(" Hold W/S       : forward / backward")
    print(" Hold A/D       : strafe left / right")
    print(" Hold Q/E       : rotate left / right")
    print(" SPACE          : emergency stop")
    print(" 1 / 2 / 3      : HOME / PICK / LIFT arm position")
    print(" O / P          : open / close gripper")
    print(" R              : reset HOME -> close -> PICK")
    print(" SHIFT + W      : intentional ToF safety override")
    print(" M              : stop and restart AUTO pickup")
    print(" H              : show this help")
    print(" ESC            : stop and disconnect")
    print("=" * 66)


def execute_manual_command(command: str, chassis, arm, gripper) -> None:
    """Execute one edge-triggered arm/gripper command with chassis stopped."""
    if command == "h":
        print_manual_help()
        return

    stop_chassis(chassis)

    if command == "1":
        move_arm(arm, HOME_POSITION, "MANUAL HOME")
    elif command == "2":
        move_arm(arm, PICK_POSITION, "MANUAL PICK")
    elif command == "3":
        move_arm(arm, LIFT_POSITION, "MANUAL LIFT")
    elif command == "o":
        open_gripper(gripper, "MANUAL OPEN")
    elif command == "p":
        close_gripper(gripper, "MANUAL CLOSE")
    elif command == "r":
        reset_pickup_arm(arm, gripper)


def manual_motion_command(
    controls: KeyboardControls,
    tof: ToFReader,
) -> Tuple[float, float, float, str, Optional[float], str]:
    """Build a hold-to-drive command and apply the forward ToF guard."""
    forward_axis = int(controls.is_pressed("w")) - int(controls.is_pressed("s"))
    strafe_axis = int(controls.is_pressed("a")) - int(controls.is_pressed("d"))
    turn_axis = int(controls.is_pressed("q")) - int(controls.is_pressed("e"))

    x_cmd = forward_axis * MANUAL_FORWARD_SPEED
    y_cmd = strafe_axis * MANUAL_STRAFE_SPEED * MANUAL_Y_SIGN
    z_cmd = turn_axis * MANUAL_TURN_SPEED * MANUAL_Z_SIGN

    # Do not make diagonal translation faster than a single-axis command.
    translation_magnitude = math.hypot(x_cmd, y_cmd)
    maximum_translation = max(MANUAL_FORWARD_SPEED, MANUAL_STRAFE_SPEED)
    if translation_magnitude > maximum_translation and translation_magnitude > 0.0:
        scale = maximum_translation / translation_magnitude
        x_cmd *= scale
        y_cmd *= scale

    front_cm, tof_status, _ = tof.filtered_distance()
    guard = ""

    if x_cmd > 0.0:
        shift_override = controls.is_pressed("shift")
        blocked_close = (
            tof_status == "VALID"
            and front_cm is not None
            and front_cm <= MANUAL_FRONT_STOP_CM
        )
        blocked_missing = (
            MANUAL_BLOCK_FORWARD_WITHOUT_TOF
            and (tof_status != "VALID" or front_cm is None)
        )

        if (blocked_close or blocked_missing) and not shift_override:
            x_cmd = 0.0
            if blocked_close:
                guard = f"TOF STOP <= {MANUAL_FRONT_STOP_CM:.1f}cm"
            else:
                guard = f"TOF {tof_status}: W BLOCKED"
        elif (blocked_close or blocked_missing) and shift_override:
            guard = "SHIFT OVERRIDE"

    if controls.is_pressed("space"):
        x_cmd = 0.0
        y_cmd = 0.0
        z_cmd = 0.0
        guard = "EMERGENCY STOP"

    return x_cmd, y_cmd, z_cmd, guard, front_cm, tof_status


def manual_control_loop(
    chassis,
    arm,
    gripper,
    tof: ToFReader,
    controls: KeyboardControls,
) -> str:
    """Run until M requests AUTO or ESC requests EXIT."""
    stop_chassis(chassis)
    controls.clear_commands()
    print_manual_help()

    last_print = 0.0
    last_motion = (0.0, 0.0, 0.0)
    emergency_latched = False

    while True:
        if controls.exit_requested():
            stop_chassis(chassis)
            return "EXIT"

        if controls.emergency_stop_pending() or controls.is_pressed("space"):
            stop_chassis(chassis)
            controls.clear_emergency_stop()
            emergency_latched = True
            last_motion = (0.0, 0.0, 0.0)

        if emergency_latched:
            motion_keys_held = any(
                controls.is_pressed(key)
                for key in ("w", "s", "a", "d", "q", "e", "space")
            )
            if motion_keys_held:
                now = time.monotonic()
                if now - last_print >= MANUAL_PRINT_INTERVAL_SEC:
                    print("[MANUAL] EMERGENCY STOP - release all motion keys")
                    last_print = now
                time.sleep(MANUAL_LOOP_SEC)
                continue
            emergency_latched = False
            print("[MANUAL] Emergency latch released")

        if controls.mode_toggle_pending():
            stop_chassis(chassis)
            print("[MODE] MANUAL -> AUTO")
            return "AUTO"

        command = controls.pop_command()
        while command is not None:
            execute_manual_command(command, chassis, arm, gripper)
            if controls.exit_requested():
                stop_chassis(chassis)
                return "EXIT"
            if controls.mode_toggle_pending():
                stop_chassis(chassis)
                print("[MODE] MANUAL -> AUTO")
                return "AUTO"
            command = controls.pop_command()

        x_cmd, y_cmd, z_cmd, guard, front_cm, tof_status = manual_motion_command(
            controls,
            tof,
        )

        motion = (x_cmd, y_cmd, z_cmd)
        if ENABLE_MOTION and motion != (0.0, 0.0, 0.0):
            chassis.drive_speed(
                x=x_cmd,
                y=y_cmd,
                z=z_cmd,
                timeout=DRIVE_COMMAND_TIMEOUT_SEC,
            )
        elif last_motion != (0.0, 0.0, 0.0):
            stop_chassis(chassis)
        last_motion = motion

        now = time.monotonic()
        if now - last_print >= MANUAL_PRINT_INTERVAL_SEC:
            tof_text = "---" if front_cm is None else f"{front_cm:.1f}cm"
            guard_text = f" | {guard}" if guard else ""
            print(
                f"[MANUAL] ToF={tof_text} ({tof_status}) | "
                f"x={x_cmd:+.3f} y={y_cmd:+.3f} z={z_cmd:+.1f}"
                f"{guard_text}"
            )
            last_print = now

        time.sleep(MANUAL_LOOP_SEC)


# ============================================================
# Main
# ============================================================

def main() -> None:
    ep_robot = robot.Robot()
    chassis = None
    tof_sensor = None
    tof_subscribed = False
    controls = KeyboardControls()
    keyboard_started = False

    try:
        print("Connecting to RoboMaster...")
        ep_robot.initialize(conn_type=CONNECTION)
        print("Connected")

        chassis = ep_robot.chassis
        arm = ep_robot.robotic_arm
        gripper = ep_robot.gripper
        tof_sensor = ep_robot.sensor

        tof = ToFReader()
        tof_sensor.sub_distance(freq=TOF_FREQUENCY_HZ, callback=tof.callback)
        tof_subscribed = True

        controls.start()
        keyboard_started = True
        print("Keyboard control ready: M=toggle mode, SPACE=stop, ESC=exit")

        print("Waiting for the first ToF callback...")
        tof_ready = tof.wait_for_first_callback(TOF_STARTUP_TIMEOUT_SEC)
        if tof_ready:
            first = tof.latest_event()
            print(
                f"ToF callback ready: status={first.status}, "
                f"raw={format_raw_mm(first.raw_mm)} mm"
            )
        else:
            print(
                "[TOF WARNING] No callback received. Check sensor connection and "
                "sub_distance(). Entering MANUAL fallback; forward W remains "
                "blocked unless SHIFT+W is intentionally held."
            )

        current_mode = START_MODE.strip().upper()
        if current_mode not in ("AUTO", "MANUAL"):
            raise ValueError("START_MODE must be 'AUTO' or 'MANUAL'")
        if not tof_ready:
            current_mode = "MANUAL"

        controls.clear_mode_toggle()
        controls.clear_commands()

        while not controls.exit_requested():
            if current_mode == "AUTO":
                stop_chassis(chassis)
                print("\n[MODE] AUTO PICKUP")
                print("Press M for MANUAL or SPACE for emergency takeover")

                try:
                    success = run_pickup_test(
                        chassis,
                        arm,
                        gripper,
                        tof,
                        controls,
                    )
                except ManualModeRequested:
                    stop_chassis(chassis)
                    controls.clear_mode_toggle()
                    controls.clear_commands()
                    print("\n[MODE] AUTO stopped safely -> MANUAL")
                    current_mode = "MANUAL"
                    continue
                except ProgramExitRequested:
                    stop_chassis(chassis)
                    break

                if success:
                    print("\nAUTO PICKUP RESULT: SUCCESS")
                    if AUTO_SUCCESS_ENTERS_MANUAL:
                        controls.clear_mode_toggle()
                        controls.clear_commands()
                        print("[MODE] Pickup complete -> MANUAL while holding object")
                        current_mode = "MANUAL"
                        continue
                    break

                print("\nAUTO PICKUP RESULT: FAILED")
                if AUTO_FAILURE_ENTERS_MANUAL:
                    controls.clear_mode_toggle()
                    controls.clear_commands()
                    print("[MODE] Automatic attempts exhausted -> MANUAL")
                    current_mode = "MANUAL"
                    continue
                break

            result = manual_control_loop(
                chassis,
                arm,
                gripper,
                tof,
                controls,
            )
            if result == "AUTO":
                controls.clear_mode_toggle()
                controls.clear_commands()
                current_mode = "AUTO"
                continue
            break

    except KeyboardInterrupt:
        print("\nSTOP REQUESTED BY USER")

    except Exception as exc:
        print(f"\nERROR: {exc}")
        raise

    finally:
        stop_chassis(chassis)

        if keyboard_started:
            controls.stop()

        if tof_sensor is not None and tof_subscribed:
            try:
                tof_sensor.unsub_distance()
            except Exception:
                pass

        try:
            ep_robot.close()
        except Exception:
            pass

        print("Robot stopped and disconnected")


if __name__ == "__main__":
    main()

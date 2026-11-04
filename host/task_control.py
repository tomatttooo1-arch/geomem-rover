"""Bounded find-object task logic, with no ROS, GPU or navigation implementation.

Call ``start`` with a recently acquired frame, deliver navigation results using the
returned command ID, update Memory before ``observe``, and call ``tick`` even when
the camera is silent. All times must share one monotonic acquisition clock (ROS
simulation time is suitable); start a new controller after a clock reset.

An observation planner receives a Memory query row and returns reachable camera
observation poses, NOT the object's coordinates as a driving goal. The adapter is
responsible for collision/FOV checks and real navigation. Search poses are fixed
map coverage poses supplied independently of target ground truth. ``stop`` means
cancel the named navigation action AND command safe stop; it is not a verified
physical stop. A platform watchdog/estop remains necessary.

The explicit action/result/cancel boundary follows Nav2 Simple Commander:
https://api.nav2.org/nav2-humble/html/md_nav2_simple_commander_README.html
This module is new project code and does not reproduce Nav2's navigation logic.
"""

from dataclasses import dataclass
import inspect
import math
import uuid
from typing import Optional


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _takes_time_left(planner):
    """10-29: the search planner's next_goal takes the task's remaining time (BeliefSearch stop sweep)."""
    try:
        return "time_left_s" in inspect.signature(planner.next_goal).parameters
    except (TypeError, ValueError):
        return False


def chain_waypoints(poses, start_xy, start_yaw=None, from_xy=None):
    """10-26 search MVP: greedy nearest-first chain of the poses' positions from ``start_xy``. Poses sharing one
    (x, y) stay together, the yaw nearest the arrival heading first (the travel direction; ``start_yaw`` / the last
    yaw when the car does not move). ``from_xy``: where the car drives from to the first group (default start_xy)."""
    groups = {}
    for p in poses:
        groups.setdefault((p.x, p.y), []).append(p)
    left, out, at, heading = list(groups), [], tuple(start_xy[:2]), start_yaw
    drive_from = tuple(from_xy[:2]) if from_xy is not None else at
    while left:
        key = min(left, key=lambda k: (k[0]-at[0])**2 + (k[1]-at[1])**2)
        left.remove(key)
        dx, dy = key[0]-drive_from[0], key[1]-drive_from[1]
        if math.hypot(dx, dy) > 1e-6:
            heading = math.atan2(dy, dx)
        group = groups[key]
        if heading is not None:
            group = sorted(group, key=lambda p: abs(_wrap(p.yaw-heading)))
        out.extend(group)
        heading = group[-1].yaw
        at = drive_from = key
    return out


@dataclass(frozen=True)
class PoseGoal:
    """Base goal in the planner's map frame, metres and yaw radians."""

    x: float
    y: float
    yaw: float
    frame: str = "map"

    def __post_init__(self):
        if not self.frame or not all(math.isfinite(v) for v in (self.x, self.y, self.yaw)):
            raise ValueError("finite pose and named coordinate frame required")


@dataclass(frozen=True)
class Command:
    kind: str
    command_id: str  # controller session UUID + sequence; never recycled across tasks
    pose: Optional[PoseGoal] = None
    target_id: Optional[str] = None
    reason: str = ""


@dataclass(frozen=True)
class TaskConfig:
    """Reference budgets to calibrate on hardware; no performance claim."""

    task_timeout_s: float = 120.0
    navigation_timeout_s: float = 30.0
    observation_timeout_s: float = 3.0
    stream_timeout_s: float = 2.0
    max_frame_age_s: float = 1.0
    max_visits: int = 12
    # 10-25 find-retry (owner-approved): a failed navigation (Nav2 aborted/rejected, or a goal past its budget)
    # no longer ends the task. The runner stops, clears both costmaps and waits ``retry_pause_s``, then retries the
    # SAME goal ``nav_retries_same_goal`` times before moving on to the next observation pose / search waypoint.
    # ``max_nav_failures`` (10-26, was max_nav_attempts) caps the FAILED navigations of a task (Nav2 aborted /
    # rejected, past the budget, injected); a goal reached costs nothing from it, so the search / the observation
    # poses are walked to their end (max_visits, the task budget and min_attempt_time_s bound the rest). An attempt
    # after the first is not started with less than ``min_attempt_time_s`` of the task budget left, and each
    # attempt's budget is min(navigation_timeout_s, task time left).
    # ``fault_nav_fail_first`` (fault injection, default off) turns the first N successful results into failures.
    nav_retries_same_goal: int = 1
    max_nav_failures: int = 3
    min_attempt_time_s: float = 20.0
    retry_pause_s: float = 1.0
    fault_nav_fail_first: int = 0
    # 10-26 find-near (owner): a fresh confirmation of THIS task's target while driving (navigating, or recovering
    # between attempts) ends the task found/seen_en_route when the car's base is within ``en_route_max_distance_m``
    # (horizontal, base_link to the target's position) -- the active goal is cancelled with the usual stop. Only when
    # observe() is given the car's position (robot_xy); no position = no en-route verdict.
    en_route_max_distance_m: float = 1.2
    # 10-30 owner (car 10-29 23:2x: cup1 seen 12 times at 1.32 m while turning in place, dropped by the 1.2 m cap):
    # the en-route cap for THIS task's target itself (row id == target_id). Search candidates of the same class
    # (the single-candidate path) keep en_route_max_distance_m. Cost: a farther stop (detections at 1.6 m jitter more,
    # the final approach starts farther away).
    en_route_target_max_distance_m: float = 1.6
    # 10-26 find-near: after ``found`` one more command, ``approach`` (the runner drives straight up to the object
    # through the normal navigate path and records the result; it never changes the found verdict).
    final_approach: bool = True
    # 10-26 search MVP (owner): at an observation pose, ``not_seen_frames`` consecutive frames with the target's box
    # in view on valid depth, not occluded (row reason in NOT_SEEN_REASONS) and no fresh confirmation -> search
    # (not_seen_at_last_location). Search goals have their own visit cap ``max_search_visits`` (max_visits now caps
    # the observation poses only): 6 mapping goals x 2 headings = 12. 10-29: 40 -- the probabilistic search with the
    # prior floor looks over the whole searched area (20-30 looks in a 20-40 m2 area); the MVP list has <= 12 goals
    # anyway, so the task budget, not this count, ends a long search.
    not_seen_frames: int = 5
    # 10-30 owner (car 10-29 23:2x: 5 not-seen frames came 2.5 s after arrival, a weak cup missed): at an observation
    # pose the not-seen verdict also needs ``observe_min_s`` since arrival; a fresh match still ends it at once. The
    # observation-pose timeout is max(observation_timeout_s, observe_min_s): at it, enough not-seen frames -> search
    # (not_seen_at_last_location), else the next pose as before (no new wait). Search poses keep observation_timeout_s.
    observe_min_s: float = 6.0
    max_search_visits: int = 40
    # with search goals available, at most this many observation poses before searching (the planner offers up to
    # 37; at ~10 s each they would eat the 180 s car budget when the place gives no not-seen verdict)
    observation_visits_before_search: int = 4
    # 10-26 owner：重成功率，放宽自「任务中一帧位姿无效即 failed:pose_invalid」: True (the car, runner --input-hiccup drop)
    # = such a frame is only dropped (no watchdog refresh, no confirmation); the stream watchdog stays the backstop.
    # start() still refuses a task on an invalid-pose frame (no_recent_frame).
    drop_invalid_pose_frames: bool = False
    # 10-29 owner (边建图边寻物时 Nano CPU 过载，处理帧断了 ~5 s 判 failed:stream_lost): True (the car, runner
    # --input-hiccup drop) = a processed-frame gap of stream_timeout_s no longer ends the task. The task records a
    # warning (events "stream_stalled" / "stream_resumed", ``stream_stall`` while it lasts), keeps the live goal
    # (the car stops by itself: the motion permit and the FPGA drive permit lapse without new frames), does not
    # start a new attempt while the stall lasts, and carries on when frames come back. task_timeout_s still ends it.
    stream_stall_waits: bool = False

    def __post_init__(self):
        times = (self.task_timeout_s, self.navigation_timeout_s,
                 self.observation_timeout_s, self.stream_timeout_s, self.max_frame_age_s)
        if any(not math.isfinite(t) or t <= 0 for t in times):
            raise ValueError("time budgets must be finite and positive")
        if isinstance(self.max_visits, bool) or not isinstance(self.max_visits, int) or self.max_visits <= 0:
            raise ValueError("max_visits must be a positive integer")
        for name, low in (("nav_retries_same_goal", 0), ("max_nav_failures", 1), ("fault_nav_fail_first", 0),
                          ("not_seen_frames", 1), ("max_search_visits", 1),
                          ("observation_visits_before_search", 1)):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value < low:
                raise ValueError("%s must be an integer >= %d" % (name, low))
        for name in ("min_attempt_time_s", "retry_pause_s", "en_route_max_distance_m",
                     "en_route_target_max_distance_m", "observe_min_s"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
                raise ValueError("%s must be finite and >= 0" % name)
        if not isinstance(self.final_approach, bool):
            raise ValueError("final_approach must be bool")
        if not isinstance(self.stream_stall_waits, bool):
            raise ValueError("stream_stall_waits must be bool")


class TaskController:
    """Single-use controller emitting commands for an external executor.

    Memory must offer ``query_memory(goal, after=timestamp)``. ``observe`` accepts
    a Frame-like object with frame_id, timestamp and pose_valid, after that exact
    frame has been processed into Memory. A true navigation result only changes
    state to observing; a query row must be confirmed in a newer acquisition
    frame to finish. ``ambiguous`` exposes ``candidates`` for explicit selection.

    Different track IDs after movement are never silently treated as the same
    object, but (10-26, owner) during re-search exactly one fresh same-class
    candidate ends the task found/search_single_candidate with target_id set to
    it (the event keeps previous_target_id); two or more still require explicit
    selection. Events are in-memory audit records;
    the application should persist them together with commands and source frames.

    ``search_planner`` (10-26, probabilistic search; host/search_belief.BeliefSearch or anything with ``begin``,
    ``next_goal`` and ``mark_failed``): replaces the fixed search-waypoint list once the task searches -- the
    triggers, the single-candidate verdict and the selection for several candidates stay as in the MVP.
    ``begin(target_xy or None)`` when the search starts (the remembered position, or none for an unknown object),
    ``next_goal()`` -> (PoseGoal, info) or (None, info) per search goal (None ends not_found/search_exhausted),
    ``mark_failed(pose)`` after a search goal's navigation failed for good. The planner's belief is fed by the
    runner from processed frames, not through this controller.
    """

    # Retry bookkeeping states/reasons (10-25 find-retry). "recovering" is NOT terminal: the car is stopped, the
    # costmaps are being cleared and the controller waits retry_pause_s before the next navigate.
    RETRY_REASON = "navigation_retry_pending"
    TERMINAL = frozenset(("found", "not_found", "unable_to_confirm", "failed", "cancelled",
                          "needs_manual_approach"))
    # SELECT_SEMANTICS_20261016.md section 1: a known target on a car without a navigation
    # server ends here (the car does not move) instead of failed / unable_to_confirm.
    MANUAL_REASON = "navigation_server_unavailable"
    TARGET_FIELDS = ("id", "label", "color", "position_m", "state", "last_seen")
    # 10-26 search MVP: geometry reasons meaning "the remembered box was fully in view on valid depth, not occluded"
    NOT_SEEN_REASONS = frozenset(("free", "surface_or_uncertain"))
    # no geometry evidence this frame (e.g. an empty geometry dict): neither counts nor resets
    NO_EVIDENCE_REASONS = frozenset(("unknown",))
    # why a remembered target went searching (task.search_trigger; the UI shows them apart)
    TRIGGER_REMOVED = "target_position_invalid"
    TRIGGER_NOT_SEEN = "not_seen_at_last_location"
    # 10-26 (owner): no observation pose could be planned around the remembered position (e.g. against a wall)
    TRIGGER_NO_POSE = "no_observation_pose"
    # 10-29 (owner): processed frames stopped for stream_timeout_s (stream_stall_waits): a warning, not an end
    STALL_REASON = "stream_stalled"
    RESUME_REASON = "stream_resumed"
    # a goal the runner could not send because its input was not ready (held, retried, not a failure)
    HOLD_REASON = "navigation_input_wait"

    def __init__(self, memory, observation_planner, search_waypoints=(), config=None,
                 navigation_available=None, search_planner=None):
        self.memory = memory
        self.observation_planner = observation_planner
        self.search_waypoints = tuple(search_waypoints)
        if any(not isinstance(p, PoseGoal) for p in self.search_waypoints):
            raise TypeError("search waypoints must be PoseGoal values")
        self.config = config or TaskConfig()
        # None = always try to navigate (the behaviour before 2026-10-16); else a callable
        # returning False when no navigation server exists (checked once per selection).
        self.navigation_available = navigation_available
        self.target = None   # summary of the target row, set only for needs_manual_approach
        self.state = "idle"
        self.reason = ""
        self.goal = ""
        self.target_id = None
        self.candidates = []
        self.events = []
        self._now = -math.inf
        self._frame_time = -math.inf
        self._frame_id = None
        self._seen_frames = set()
        self._session = uuid.uuid4().hex
        self._sequence = 0
        self._command_id = self._session + ":0"
        self._active_id = None
        self._visits = 0
        self._search_index = 0
        self._searching = False
        self._poses = []
        self._planned_poses = 0
        self._target_row = None
        self._last_target_row = None   # 10-30: the target's row of the latest not-seen count
        self.attempts = []           # one dict per navigate command (see attempts_snapshot)
        self.attempt_timeout_s = self.config.navigation_timeout_s
        self._pose = None            # goal of the active / last attempt
        self._goal_failures = 0      # consecutive failures on the current goal
        self._retry_pose = None
        self._injections_left = self.config.fault_nav_fail_first
        self._cap_base = 0           # attempts before the latest explicit selection (their failures not counted)
        self.found_row = None        # the confirming Memory row (copy) once found; the runner's approach reads it
        self.en_route = None         # {"distance_m", "frame_id", "robot_xy"} of a seen_en_route verdict
        self.approach = None         # the runner's final-approach record (set_approach); never changes the state
        self.search_trigger = None   # 10-26: TRIGGER_REMOVED / _NOT_SEEN / _NO_POSE once a remembered target goes searching
        self._not_seen = 0           # consecutive not-seen frames at the current observation pose
        self._obs_visits = 0
        self._search_visits = 0
        self._passed_over = set()    # ids offered for selection and not chosen (never auto-found)
        self._robot_xy = None        # last car position given to observe()
        self.search_planner = search_planner   # 10-26 probabilistic search (None = the MVP's waypoint list)
        self._planner_done = False
        # 10-29: {"since": last accepted frame time, "detected": task clock, "n": 1-based} while processed frames
        # have been missing for stream_timeout_s (stream_stall_waits), else None; stream_stalls counts them
        self.stream_stall = None
        self.stream_stalls = 0

    def _clock(self, now):
        if not math.isfinite(now) or now < self._now:
            raise ValueError("task clock must be finite and monotonic")
        self._now = now

    def _state(self, state, reason):
        self.state, self.reason = state, reason
        self.events.append({"time": self._now, "state": state, "reason": reason,
                            "target_id": self.target_id})

    def _finish(self, state, reason):
        if self.attempts and self.attempts[-1]["result"] is None:
            self._close_attempt("interrupted", reason)
        command = Command("stop", self._active_id or self._command_id,
                          target_id=self.target_id, reason=reason)
        self._active_id = None
        self._state(state, reason)
        return [command]

    def _accept_frame(self, frame, in_task=False):
        if not frame.pose_valid:
            if in_task and self.config.drop_invalid_pose_frames:
                return None   # 10-26 owner：重成功率，放宽自 failed:pose_invalid -- dropped like a stale frame
            return self._finish("failed", "pose_invalid")
        if (not frame.frame_id or not math.isfinite(frame.timestamp)
                or frame.timestamp > self._now):
            return self._finish("failed", "invalid_frame_clock")
        if (frame.timestamp <= self._frame_time or frame.frame_id in self._seen_frames
                or self._now - frame.timestamp > self.config.max_frame_age_s):
            return None  # stale frames cannot refresh the watchdog or confirm
        self._frame_time, self._frame_id = frame.timestamp, frame.frame_id
        self._seen_frames.add(frame.frame_id)
        if self.stream_stall is not None:
            self._stall_end(frame)
        return []

    def _stall_begin(self, now):
        """10-29: first watchdog pass with no processed frame for stream_timeout_s -> warning event, no command."""
        if self.stream_stall is not None:
            return
        self.stream_stalls += 1
        self.stream_stall = {"since": self._frame_time, "detected": now, "n": self.stream_stalls}
        self.events.append({"time": now, "state": self.state, "reason": self.STALL_REASON,
                            "target_id": self.target_id,
                            "detail": {"gap_s": round(now - self._frame_time, 3), "n": self.stream_stalls}})

    def _stall_end(self, frame):
        stall, self.stream_stall = self.stream_stall, None
        self.events.append({"time": self._now, "state": self.state, "reason": self.RESUME_REASON,
                            "target_id": self.target_id,
                            "detail": {"gap_s": round(frame.timestamp - stall["since"], 3), "n": stall["n"],
                                       "frame_id": frame.frame_id}})

    def stall_snapshot(self):
        """Task-row field while a stall lasts: {"gap_s", "n"} (gap = task clock minus the last accepted frame),
        else None."""
        if self.stream_stall is None:
            return None
        return {"gap_s": round(self._now - self.stream_stall["since"], 3), "n": self.stream_stall["n"]}

    def start(self, goal, now, frame):
        """Start once using a recent, trusted frame; returns navigation/stop commands."""
        if self.state != "idle":
            raise RuntimeError("controller is single-use")
        if not isinstance(goal, str) or not goal.strip():
            raise ValueError("nonempty query required")
        self._clock(now)
        self.goal, self._started = goal, now
        accepted = self._accept_frame(frame)
        if accepted is None:
            return self._finish("failed", "no_recent_frame")
        if accepted:
            return accepted
        candidates = self.memory.query_memory(goal, after=now)
        if len(candidates) > 1:
            self.candidates = candidates
            self._state("ambiguous", "candidate_selection_required")
            return []
        if candidates:
            return self._select(candidates[0])
        self._searching = True
        self._begin_planner(None)
        return self._next_pose()

    def select_candidate(self, candidate_id, now):
        """Explicit user choice; refreshed query must still contain this ID."""
        commands = self.tick(now)
        if commands or self.state in self.TERMINAL:
            return commands
        if self.state != "ambiguous":
            raise RuntimeError("no candidate selection is pending")
        offered = {row["id"] for row in self.candidates}
        rows = self.memory.query_memory(self.goal, after=self._started)
        chosen = next((r for r in rows if r["id"] == candidate_id and candidate_id in offered), None)
        if chosen is None:
            raise ValueError("candidate is no longer available")
        # an explicit user choice starts a fresh failure count for that target (the task time budget still holds)
        self._cap_base = len(self.attempts)
        self._passed_over |= offered - {candidate_id}
        return self._select(chosen)

    def _manual_approach(self, candidate):
        """Known target, no navigation server: report where it is, stop, do not plan."""
        self.target = {k: candidate.get(k) for k in self.TARGET_FIELDS}
        if candidate.get("number") is not None:          # 10-27 class-local display number, only when known
            self.target["number"] = candidate["number"]
        return self._finish("needs_manual_approach", self.MANUAL_REASON)

    def _select(self, candidate):
        self.target_id = candidate["id"]
        self.candidates = []
        self._searching = False
        self._target_row = candidate
        if candidate["state"] == "position_invalid":
            self._mark_search(self.TRIGGER_REMOVED)
            return self._next_pose()
        if self.navigation_available is not None and not self.navigation_available():
            return self._manual_approach(candidate)
        self._poses = list(self.observation_planner(candidate))
        if any(not isinstance(p, PoseGoal) for p in self._poses):
            raise TypeError("observation planner must return PoseGoal values")
        # Avoid cycling through repeated planner goals without gaining a viewpoint.
        self._poses = list(dict.fromkeys(self._poses))
        self._planned_poses = len(self._poses)
        return self._next_pose()

    def _mark_search(self, trigger, detail=None):
        """10-26 search MVP: the remembered target is not where it was -> search from now on, the remaining goals
        re-chained nearest-first from the target's remembered position (audit row in events, not a state)."""
        self._searching = True
        self._poses = []
        self.search_trigger = trigger
        event = {"time": self._now, "state": self.state, "reason": trigger, "target_id": self.target_id}
        if detail:
            event["detail"] = detail
        self.events.append(event)
        xy = self._row_xy(self._target_row)
        if xy is not None:
            rest = chain_waypoints(self.search_waypoints[self._search_index:], xy, from_xy=self._robot_xy)
            self.search_waypoints = self.search_waypoints[:self._search_index] + tuple(rest)
        self._begin_planner(xy)

    def _begin_planner(self, xy):
        if self.search_planner is not None:
            try:
                self.search_planner.begin(xy)
            except Exception as exc:              # noqa: BLE001 - a planner fault: the MVP list, never a failed task
                self._drop_planner("begin", exc)

    def _drop_planner(self, where, exc):
        """10-26: the probabilistic planner raised -> the MVP's waypoint list from here on (event search_planner_error)."""
        self.search_planner = None
        self.events.append({"time": self._now, "state": self.state, "reason": "search_planner_error",
                            "target_id": self.target_id, "detail": "%s: %r" % (where, exc)})

    def _search_left(self):
        """Search goals still to come (the planner decides per goal; the MVP list by its index)."""
        if self.search_planner is not None:
            return not self._planner_done
        return self._search_index < len(self.search_waypoints)

    @staticmethod
    def _row_xy(row):
        try:
            xy = (float(row["position_m"][0]), float(row["position_m"][1]))
        except (KeyError, TypeError, IndexError, ValueError):
            return None
        return xy if all(math.isfinite(v) for v in xy) else None

    def _next_pose(self):
        if self._searching:
            pose = info = None
            if self.search_planner is not None:
                if self._search_visits >= self.config.max_search_visits:
                    return self._finish("unable_to_confirm", "visit_budget_exhausted")
                try:
                    pose, info = (self.search_planner.next_goal(time_left_s=self._remaining()) if
                                  _takes_time_left(self.search_planner) else self.search_planner.next_goal())
                except Exception as exc:          # noqa: BLE001 - fall back to the MVP list below
                    self._drop_planner("next_goal", exc)
            if self.search_planner is not None:
                if pose is None:
                    self._planner_done = True
                    self.events.append({"time": self._now, "state": self.state, "reason": "search_planner_stop",
                                        "target_id": self.target_id, "detail": dict(info)})
                    return self._finish("not_found", "search_exhausted")
                self.events.append({"time": self._now, "state": self.state, "reason": "search_goal_selected",
                                    "target_id": self.target_id, "detail": dict(info)})
            else:
                if self._search_index >= len(self.search_waypoints):
                    return self._finish("not_found", "search_exhausted")
                if self._search_visits >= self.config.max_search_visits:
                    return self._finish("unable_to_confirm", "visit_budget_exhausted")
                pose = self.search_waypoints[self._search_index]
                self._search_index += 1
            self._search_visits += 1
        else:
            search_left = self._search_left()
            cap = (min(self.config.max_visits, self.config.observation_visits_before_search) if search_left
                   else self.config.max_visits)
            if not self._poses or self._obs_visits >= cap:
                # Zero planned poses (e.g. target outside the known/free map) is not the same
                # cause as "visited every planned pose without a confirming frame".
                end = ("visit_budget_exhausted" if self._obs_visits >= self.config.max_visits else
                       "no_observation_pose_planned" if self._planned_poses == 0 else "observation_positions_exhausted")
                if self._planned_poses and search_left:
                    # 10-26: the poses were seen without a confirmation -> search instead of giving up
                    self._mark_search(self.TRIGGER_NOT_SEEN, end)
                    return self._next_pose()
                if not self._planned_poses and search_left:
                    # 10-26 (owner): nowhere to look from around the remembered place -> search from there
                    self._mark_search(self.TRIGGER_NO_POSE, end)
                    return self._next_pose()
                return self._finish("unable_to_confirm", end)
            pose = self._poses.pop(0)
            self._obs_visits += 1
        self._visits += 1
        self._not_seen = 0
        self._goal_failures = 0
        return self._launch(pose)

    def _failures(self):
        """Failed navigations since the latest explicit selection (succeeded / interrupted / open ones not counted)."""
        return sum(1 for a in self.attempts[self._cap_base:] if a["result"] == "failed")

    def _capped(self):
        return self._failures() >= self.config.max_nav_failures

    def _remaining(self):
        return self.config.task_timeout_s - (self._now - self._started)

    def _launch(self, pose):
        """One navigation attempt (a new goal or a retry of the same goal), within the failure and time caps."""
        if self._capped():   # defensive: _navigation_failed already ends the task on the failure that spends the cap
            return self._finish("failed", "navigation_failed_after_retries")
        remaining = self._remaining()
        # Only attempts after the first: the first navigate of a task is never refused on time.
        if self.attempts and remaining < self.config.min_attempt_time_s:
            return self._finish("failed", "attempts_time_exhausted")
        self.attempt_timeout_s = min(self.config.navigation_timeout_s, remaining)
        self._pose = pose
        self._sequence += 1
        self._command_id = "{}:{}".format(self._session, self._sequence)
        self._active_id = self._command_id
        self._phase_started = self._now
        kind = "search" if self._searching else "reobserve"
        self.attempts.append({"n": len(self.attempts) + 1, "command_id": self._command_id, "kind": kind,
                              "retry": self._goal_failures > 0,
                              "pose": {"x": pose.x, "y": pose.y, "yaw": pose.yaw, "frame": pose.frame},
                              "started": self._now, "budget_s": self.attempt_timeout_s,
                              "result": None, "reason": None, "duration_s": None, "injected": False})
        self._state("navigating", kind)
        return [Command("navigate", self._active_id, pose, self.target_id, self.reason)]

    def _close_attempt(self, result, reason, injected=False):
        if self.attempts and self.attempts[-1]["result"] is None:
            a = self.attempts[-1]
            a.update(result=result, reason=reason, injected=injected,
                     duration_s=round(self._now - a["started"], 3))

    def _navigation_failed(self, reason, injected=False):
        """A failed attempt: stop + clear costmaps (``recover``), then retry the same goal or move on.

        Ends at once (no recover) when no further attempt can follow: failure cap, too little time left, or no
        other pose/waypoint to go to after the same-goal retries are spent."""
        failed_id = self._active_id
        self._active_id = None
        self._close_attempt("failed", reason, injected)
        self._goal_failures += 1
        retry = self._goal_failures <= self.config.nav_retries_same_goal
        self._retry_pose = self._pose if retry else None
        if self._capped():
            return self._finish("failed", "navigation_failed_after_retries")
        if self._remaining() < self.config.min_attempt_time_s:
            return self._finish("failed", "attempts_time_exhausted")
        if not retry and self._searching and self.search_planner is not None:
            try:
                self.search_planner.mark_failed(self._pose)
            except Exception as exc:              # noqa: BLE001
                self._drop_planner("mark_failed", exc)
        if not retry and not self._has_next_pose():
            # nowhere else to go: the usual end words (observation_positions_exhausted / search_exhausted /
            # visit_budget_exhausted), so one reason word keeps one end state
            return self._next_pose()
        self._recover_started = self._now
        self._state("recovering", self.RETRY_REASON)
        return [Command("recover", failed_id, self._pose, self.target_id, reason)]

    def _has_next_pose(self):
        if self._searching:
            return self._search_visits < self.config.max_search_visits and self._search_left()
        if self._poses and self._obs_visits < self.config.max_visits:
            return True
        return bool(self._planned_poses) and self._search_left()

    def attempts_snapshot(self):
        """Task-row summary: count, distinct goals tried (arrived at or attempted), failures since the latest explicit
        selection and every attempt. ``max`` is the failure cap (max_nav_failures), not a cap on ``count``."""
        positions = len({(a["pose"]["x"], a["pose"]["y"], a["pose"]["yaw"], a["pose"]["frame"])
                         for a in self.attempts})
        return {"count": len(self.attempts), "positions": positions, "max": self.config.max_nav_failures,
                "failures": self._failures(), "items": [dict(a) for a in self.attempts]}

    def navigation_result(self, command_id, succeeded, now, reason="navigation_failed"):
        """Accept an actual executor result; ignore results for retired command IDs.

        A failure (``reason`` names it, e.g. navigation_wall_timeout) goes to the retry path, never straight to
        failed. With fault injection on, the first N successful results are judged failed (injected=True)."""
        if not isinstance(succeeded, bool):
            raise TypeError("succeeded must be bool")
        commands = self.tick(now)
        if commands or self.state != "navigating" or command_id != self._active_id:
            return commands
        if succeeded and self._injections_left > 0:
            self._injections_left -= 1
            return self._navigation_failed("injected_navigation_failure", injected=True)
        if not succeeded:
            return self._navigation_failed(reason or "navigation_failed")
        self._active_id = None
        self._close_attempt("succeeded", None)
        self._phase_started = now
        self._arrived = now
        self._state("observing", "fresh_visual_evidence_required")
        return []

    def observe(self, frame, now, robot_xy=None):
        """Consume a processed acquisition frame; no memory mutation is performed.

        ``robot_xy`` (optional): the car base's map-frame (x, y) at this frame's acquisition. Given, a fresh
        confirmation of the task's target while navigating / recovering within en_route_max_distance_m ends the
        task found/seen_en_route (search mode: fresh candidates that near stop the car for explicit selection)."""
        self._clock(now)
        if self.state == "idle" or self.state in self.TERMINAL:
            return []
        commands = self._expired_watchdogs(now)
        if commands:
            if self.state not in self.TERMINAL:
                # a navigation timeout now starts a retry (non-terminal): this frame still refreshes the stream
                accepted = self._accept_frame(frame, in_task=True)
                if accepted:
                    return commands + accepted
            return commands
        accepted = self._accept_frame(frame, in_task=True)
        if accepted:
            return accepted
        # 10-30: the observation timeout waits for this frame (a fresh match at the timeout still counts)
        commands = self.tick(now, defer_observing=True)
        if commands or accepted is None:
            return commands
        deferred = self.state == "observing"
        if robot_xy is not None:
            self._robot_xy = (float(robot_xy[0]), float(robot_xy[1]))
        if self.state in ("navigating", "recovering"):
            return self._en_route(frame, robot_xy)
        if self.state != "observing":
            return []
        if frame.timestamp <= self._arrived:
            return self._observing_timeout(now) if deferred else []
        rows, fresh = self._fresh_rows(frame, self._arrived)
        selected = next((r for r in fresh if r["id"] == self.target_id), None)
        if selected:
            return self._found("fresh_matching_observation", selected)
        if self._searching and fresh:
            single = self._single_candidate(fresh)
            if single is not None:
                return self._take_candidate(single)
            # more than one: identity is not asserted, the user selects
            self.candidates = fresh
            self._state("ambiguous", "search_candidate_selection_required")
            return []
        target = next((r for r in rows if r["id"] == self.target_id), None)
        if target and not self._searching:
            trigger = None
            if target["state"] == "position_invalid":
                trigger = self.TRIGGER_REMOVED
            elif target["state"] == "temporarily_occluded":
                return self._next_pose()
            elif self._count_not_seen(target) and self._observed_long_enough(now):
                trigger = self.TRIGGER_NOT_SEEN
            if trigger:
                self._target_row = target
                self._mark_search(trigger)
                single = self._single_candidate(fresh) if fresh else None
                if single is not None:          # already in view somewhere else
                    return self._take_candidate(single)
                return self._next_pose()
        return self._observing_timeout(now) if deferred and self.state == "observing" else []

    def _observed_long_enough(self, now):
        """10-30: observe_min_s since arrival at this observation pose."""
        return now - self._arrived >= self.config.observe_min_s

    def _observing_limit(self):
        if self._searching:
            return self.config.observation_timeout_s
        return max(self.config.observation_timeout_s, self.config.observe_min_s)

    def _observing_timeout(self, now):
        """Observation timeout: at an observation pose with enough not-seen frames -> search (10-30), else next pose."""
        if self.state != "observing" or now - self._phase_started < self._observing_limit():
            return []
        if now - self._frame_time > self.config.max_frame_age_s:
            return []
        if (not self._searching and self._not_seen >= self.config.not_seen_frames
                and self._last_target_row is not None):
            self._target_row = self._last_target_row
            self._mark_search(self.TRIGGER_NOT_SEEN)
        return self._next_pose()

    def _count_not_seen(self, target):
        """10-26: consecutive frames with the remembered box in view, valid depth, unoccluded, and no confirmation."""
        reason = target.get("reason")
        self._last_target_row = target
        if reason in self.NOT_SEEN_REASONS:
            self._not_seen += 1
        elif reason not in self.NO_EVIDENCE_REASONS:
            self._not_seen = 0
        return self._not_seen >= self.config.not_seen_frames

    def _single_candidate(self, rows):
        """10-26: the only fresh same-class row in search, else None. Owner 10-29: one the user passed over counts
        too (car 10-29 23:2x: the chosen cup was re-identified as the other one and the task waited for a selection)."""
        if len(rows) == 1:
            return rows[0]
        return None

    def _take_candidate(self, row):
        """10-26: search found exactly one same-class object -> it is the target (found, final approach as usual)."""
        previous = self.target_id
        self.target_id = row["id"]
        self.candidates = []
        commands = self._found("search_single_candidate", row)
        self.events[-1]["previous_target_id"] = previous
        self.events[-1]["passed_over"] = row["id"] in self._passed_over
        return commands

    def _fresh_rows(self, frame, after):
        rows = self.memory.query_memory(self.goal, after=after)
        fresh = []
        for r in rows:
            if (r.get("confirmed_after_query") and r["state"] == "current_confirmed"
                    and r["last_seen_frame"] == frame.frame_id and r["last_seen"] == frame.timestamp):
                fresh.append(r)
                continue
            seen = r.get("observed")
            if (r.get("observed_after_query") and isinstance(seen, dict) and seen.get("far") and not seen.get("weak")
                    and seen.get("frame_id") == frame.frame_id and seen.get("timestamp") == frame.timestamp):
                # 10-28: this frame saw the object away from its remembered place (memory not moved yet): the
                # find counts it, and the approach drives to where it was seen (approach_position_m/_bounds_m)
                fresh.append(dict(r, approach_position_m=list(seen["position_m"]),
                                  approach_bounds_m=[list(b) for b in seen["bounds_m"]]))
        return rows, fresh

    def _distance(self, row, robot_xy):
        xy = row.get("approach_position_m") or row.get("position_m")
        try:
            dx = float(xy[0]) - float(robot_xy[0])
            dy = float(xy[1]) - float(robot_xy[1])
        except (KeyError, TypeError, IndexError, ValueError):
            return None
        d = math.hypot(dx, dy)
        return d if math.isfinite(d) else None

    def _en_route(self, frame, robot_xy):
        """10-26 find-near: a fresh, near confirmation while driving (no verdict without a car position)."""
        if robot_xy is None:
            return []
        _, fresh = self._fresh_rows(frame, self._started)
        limit = self.config.en_route_max_distance_m
        target_limit = self.config.en_route_target_max_distance_m      # 10-30: the target itself, 1.6 m
        near = []
        selected = None
        for row in fresh:
            d = self._distance(row, robot_xy)
            if d is None:
                continue
            if selected is None and self.target_id is not None and row["id"] == self.target_id and d <= target_limit:
                selected = (row, d)
            if d <= limit:
                near.append((row, d))
        if selected:
            row, d = selected
            self.en_route = {"distance_m": round(d, 3), "frame_id": frame.frame_id,
                             "robot_xy": [round(float(robot_xy[0]), 3), round(float(robot_xy[1]), 3)]}
            return self._found("seen_en_route", row)
        if self._searching and near:
            single = self._single_candidate([r for r, _ in near])
            if single is not None:
                d = next(d for r, d in near if r is single)
                self.en_route = {"distance_m": round(d, 3), "frame_id": frame.frame_id,
                                 "robot_xy": [round(float(robot_xy[0]), 3), round(float(robot_xy[1]), 3)]}
                return self._take_candidate(single)
            # same rule as at an observation pose: identity is not asserted, the user selects (car stopped first)
            commands = self._halt("search_candidate_selection_required")
            self.candidates = [r for r, _ in near]
            self._state("ambiguous", "search_candidate_selection_required")
            return commands
        return []

    def _halt(self, reason):
        """Cancel the active goal (if any) without ending the task: the usual stop command."""
        if self.attempts and self.attempts[-1]["result"] is None:
            self._close_attempt("interrupted", reason)
        command = Command("stop", self._active_id or self._command_id, target_id=self.target_id, reason=reason)
        self._active_id = None
        self._retry_pose = None
        return [command]

    def _found(self, reason, row):
        commands = self._finish("found", reason)
        self.found_row = dict(row)
        # 10-28: which place the final approach drives to -- this frame's reading when it lay away from the
        # remembered place (see _fresh_rows), else the remembered (fused) place
        far = row.get("approach_position_m") is not None
        self.found_row["approach_source"] = "observation" if far else "memory"
        self.events[-1]["approach_source"] = self.found_row["approach_source"]
        self.events[-1]["approach_position_m"] = list(row["approach_position_m"] if far else row.get("position_m") or [])
        if far:
            self.events[-1]["memory_position_m"] = list(row.get("position_m") or [])
            self.events[-1]["observed_offset_m"] = (row.get("observed") or {}).get("offset_m")
        if self.en_route is not None:
            self.events[-1]["distance_m"] = self.en_route["distance_m"]
        if self.config.final_approach:
            self._sequence += 1
            commands.append(Command("approach", "{}:{}".format(self._session, self._sequence),
                                    target_id=self.target_id, reason=reason))
        return commands

    # 10-29 find while mapping: a live alignment (host/live_align.py) moved the memory -- and the target -- under a
    # running task. Observation poses were planned around the target's OLD position, so they are planned again around
    # the new one when it moved >= REPLAN_MOVE_M; the goal being driven to is swapped for the first new pose when it
    # moved >= RETARGET_MOVE_M (a loop closure: 10-27 run 104727 had one 0.51 m / 20 deg correction), costing no
    # failure (attempt "interrupted"/map_moved). Smaller moves: the en-route / at-pose confirmation absorbs them.
    REPLAN_MOVE_M = 0.15
    RETARGET_MOVE_M = 0.25
    RETARGET_REASON = "map_moved_retarget"

    def map_moved(self, now):
        """The runner moved the memory (live alignment). -> commands (a ``retarget`` = retire the live goal, the
        next pose follows after retry_pause_s like a retry, nothing counted as failed) or []. Search mode and
        terminal states: nothing (the search belief is not moved; the en-route confirmation still works)."""
        self._clock(now)
        if self.state == "idle" or self.state in self.TERMINAL or self._searching or self.target_id is None:
            return []
        if self.state not in ("navigating", "observing", "recovering"):
            return []
        try:
            rows = self.memory.query_memory(self.goal, after=self._started)
        except Exception:                          # noqa: BLE001 - no row now: keep the plan
            return []
        row = next((r for r in rows if r.get("id") == self.target_id), None)
        old, new = self._row_xy(self._target_row), self._row_xy(row)
        if row is None or old is None or new is None:
            return []
        moved = math.hypot(new[0] - old[0], new[1] - old[1])
        if moved < self.REPLAN_MOVE_M:
            return []
        poses = list(dict.fromkeys(self.observation_planner(row)))
        if any(not isinstance(p, PoseGoal) for p in poses):
            raise TypeError("observation planner must return PoseGoal values")
        detail = {"moved_m": round(moved, 3), "poses": len(poses), "from": [round(v, 3) for v in old],
                  "to": [round(v, 3) for v in new]}
        self._target_row = row
        if not poses:                              # nothing plannable around the new place: keep the old plan
            self.events.append({"time": self._now, "state": self.state, "reason": "map_moved_plan_kept",
                                "target_id": self.target_id, "detail": detail})
            return []
        self._poses = poses
        self._planned_poses = max(self._planned_poses, len(poses))
        retarget = (moved >= self.RETARGET_MOVE_M and self.state in ("navigating", "recovering")
                    and self._remaining() >= self.config.min_attempt_time_s + self.config.retry_pause_s + 1.0)
        self.events.append({"time": self._now, "state": self.state,
                            "reason": "map_moved_retarget" if retarget else "map_moved_replanned",
                            "target_id": self.target_id, "detail": detail})
        if not retarget:
            return []
        pose = self._poses.pop(0)
        if self.state == "recovering":             # between attempts: the retry goes to the new pose instead
            self._retry_pose, self._goal_failures = pose, 0
            return []
        retired = self._active_id
        self._active_id = None
        self._close_attempt("interrupted", "map_moved")
        self._retry_pose, self._goal_failures = pose, 0
        self._recover_started = self._now
        self._state("recovering", self.RETARGET_REASON)
        return [Command("retarget", retired, pose, self.target_id, "map_moved")]

    def set_approach(self, record):
        """The runner's final-approach outcome (a dict); the task stays found whatever it says."""
        self.approach = dict(record)

    def _expired_watchdogs(self, now):
        if now - self._started >= self.config.task_timeout_s:
            return self._finish("failed", "task_timeout")
        if now - self._frame_time >= self.config.stream_timeout_s:
            if not self.config.stream_stall_waits:
                return self._finish("failed", "stream_lost")
            self._stall_begin(now)      # 10-29 owner: warn and wait, the task goes on when frames come back
        if self.state == "navigating" and now - self._phase_started >= self.attempt_timeout_s:
            return self._navigation_failed("navigation_timeout")
        return []

    def tick(self, now, defer_observing=False):
        """Watchdog; call regularly even with no input. Returned stop must be executed."""
        self._clock(now)
        if self.state == "idle" or self.state in self.TERMINAL:
            return []
        commands = self._expired_watchdogs(now)
        if commands:
            return commands
        if self.state == "observing" and not defer_observing:
            commands = self._observing_timeout(now)
            if commands:
                return commands
        if self.state == "recovering" and now - self._recover_started >= self.config.retry_pause_s:
            if self.stream_stall is not None:
                return []           # 10-29: no new goal while no frame is processed; sent once frames are back
            if self._retry_pose is not None:
                pose, self._retry_pose = self._retry_pose, None
                return self._launch(pose)
            return self._next_pose()
        return []

    def navigation_held(self, command_id, now, reason=HOLD_REASON):
        """10-29: the runner could not SEND the active goal (its input was not ready, e.g. no processed frame).

        Not a failed navigation: the attempt record is withdrawn (the goal never left), the car stays stopped and
        the same goal is sent again after retry_pause_s (later while a stream stall lasts). Counts against no
        failure cap; task_timeout_s still bounds the wait."""
        commands = self.tick(now)
        if commands or self.state != "navigating" or command_id != self._active_id:
            return commands
        if self.attempts and self.attempts[-1]["command_id"] == command_id and self.attempts[-1]["result"] is None:
            self.attempts.pop()
        self._active_id = None
        self._retry_pose = self._pose
        self._recover_started = now
        self._state("recovering", reason)
        return []

    def cancel(self, now):
        self._clock(now)
        if self.state == "idle" or self.state in self.TERMINAL:
            return []
        return self._finish("cancelled", "user_cancelled")

    def fail(self, reason, now):
        """Adapter fault (e.g. detector unavailable); fail closed with a stop command."""
        self._clock(now)
        if self.state == "idle" or self.state in self.TERMINAL:
            return []
        if (reason == self.MANUAL_REASON and self.target_id is not None and not self._searching
                and self._target_row is not None):
            # the server vanished between the selection and the navigate (same verdict)
            return self._manual_approach(self._target_row)
        return self._finish("failed", reason)

#!/usr/bin/env python3
"""Auto-tune ros_control JointTrajectoryController PID gains in Gazebo.

What it does
- Updates PID gains for /arm_effort_controller (prefer dynamic_reconfigure; fallback to controller reload)
- Runs a repeatable FollowJointTrajectory test
- Scores tracking quality from /joint_states
- Performs random search with progressive narrowing
- Prints + saves the best gains as YAML snippet (ready to paste into ros_controllers.yaml)

Prereqs
- Source your workspace: `source devel/setup.bash`
- Start your sim: `roslaunch fr5_moveit_effort_config demo_gazebo.launch`

Run
- `rosrun fr5_data_collection pid_autotune.py --iters 60`

Notes
- This script intentionally keeps the optimization simple and robust.
- For safety: it only commands small joint deltas by default.
"""

# python3 pid_autotune.py --cem --cem-gens 8 --cem-pop 24 --cem-elite 6 --scenarios 3 --delta 0.12 --apply

from __future__ import annotations

import argparse
import dataclasses
import math
import os
import random
import re
import time
from typing import Dict, List, Optional, Tuple

import rospy
from sensor_msgs.msg import JointState

import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from control_msgs.msg import JointTolerance
from trajectory_msgs.msg import JointTrajectoryPoint


JOINTS_DEFAULT = ["j1", "j2", "j3", "j4", "j5", "j6"]


def decode_fjt_error(code: int) -> str:
    # control_msgs/FollowJointTrajectoryResult
    mapping = {
        0: "SUCCESSFUL",
        -1: "INVALID_GOAL",
        -2: "INVALID_JOINTS",
        -3: "OLD_HEADER_TIMESTAMP",
        -4: "PATH_TOLERANCE_VIOLATED",
        -5: "GOAL_TOLERANCE_VIOLATED",
    }
    return mapping.get(code, f"UNKNOWN({code})")


@dataclasses.dataclass
class JointLimit:
    lower: float
    upper: float


def try_read_joint_limits_from_urdf(joint_names: List[str]) -> Optional[Dict[str, JointLimit]]:
    """Best-effort URDF joint limits from /robot_description.

    Returns None if parsing deps are missing or robot_description is unavailable.
    """

    try:
        urdf_xml = rospy.get_param("/robot_description")
    except Exception:
        return None

    try:
        from urdf_parser_py.urdf import URDF  # type: ignore

        robot = URDF.from_xml_string(urdf_xml)
        limits: Dict[str, JointLimit] = {}
        for jn in joint_names:
            j = robot.joint_map.get(jn)
            if j is None or j.limit is None:
                continue
            limits[jn] = JointLimit(lower=float(j.limit.lower), upper=float(j.limit.upper))
        return limits or None
    except Exception:
        return None


@dataclasses.dataclass
class Gains:
    p: float
    i: float
    d: float
    i_clamp: float


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _now() -> float:
    return rospy.Time.now().to_sec()


class JointStateBuffer:
    def __init__(self, joint_names: List[str]):
        self._joint_names = joint_names
        self._latest: Optional[JointState] = None
        self._name_to_index: Dict[str, int] = {}
        self._sub = rospy.Subscriber("/joint_states", JointState, self._cb, queue_size=5)

    def _cb(self, msg: JointState) -> None:
        self._latest = msg
        if not self._name_to_index:
            self._name_to_index = {name: idx for idx, name in enumerate(msg.name)}

    def wait_for_first(self, timeout_s: float = 5.0) -> None:
        start = _now()
        rate = rospy.Rate(50)
        while not rospy.is_shutdown() and self._latest is None:
            if _now() - start > timeout_s:
                raise RuntimeError("Timeout waiting for /joint_states")
            rate.sleep()

    def positions(self) -> Dict[str, float]:
        if self._latest is None:
            raise RuntimeError("No joint state received yet")
        msg = self._latest
        out: Dict[str, float] = {}
        for name in self._joint_names:
            idx = self._name_to_index.get(name)
            if idx is None or idx >= len(msg.position):
                raise RuntimeError(f"Joint '{name}' not found in /joint_states")
            out[name] = float(msg.position[idx])
        return out


class EffortControllerTuner:
    def __init__(self, controller_ns: str, joint_names: List[str]):
        self.controller_ns = controller_ns.rstrip("/")
        self.joint_names = joint_names

        self._js = JointStateBuffer(joint_names)
        self._js.wait_for_first(timeout_s=10.0)

        self._traj_client = actionlib.SimpleActionClient(
            f"{self.controller_ns}/follow_joint_trajectory", FollowJointTrajectoryAction
        )
        if not self._traj_client.wait_for_server(rospy.Duration(10.0)):
            raise RuntimeError(
                f"Timeout waiting for action server {self.controller_ns}/follow_joint_trajectory"
            )

        # Try dynamic_reconfigure first (fast). If missing / fails, we fall back to reload.
        self._dyn_client = None
        try:
            from dynamic_reconfigure.client import Client as DynClient  # type: ignore

            self._dyn_client = DynClient(self.controller_ns, timeout=5.0)
            _ = self._dyn_client.get_configuration(timeout=5.0)
            rospy.loginfo("dynamic_reconfigure available for %s", self.controller_ns)
        except Exception as ex:
            rospy.logwarn(
                "dynamic_reconfigure not usable (%s). Will fall back to controller reload via controller_manager services.",
                ex,
            )
            self._dyn_client = None

        # Services for reload fallback
        self._srv_load = None
        self._srv_unload = None
        self._srv_switch = None
        try:
            from controller_manager_msgs.srv import LoadController, UnloadController, SwitchController  # type: ignore

            self._srv_load = rospy.ServiceProxy("/controller_manager/load_controller", LoadController)
            self._srv_unload = rospy.ServiceProxy("/controller_manager/unload_controller", UnloadController)
            self._srv_switch = rospy.ServiceProxy("/controller_manager/switch_controller", SwitchController)
        except Exception as ex:
            rospy.logwarn("controller_manager services not available (%s).", ex)

    def read_gains_from_param(self) -> Dict[str, Gains]:
        gains: Dict[str, Gains] = {}
        for j in self.joint_names:
            base = f"{self.controller_ns}/gains/{j}"
            gains[j] = Gains(
                p=float(rospy.get_param(f"{base}/p")),
                i=float(rospy.get_param(f"{base}/i", 0.0)),
                d=float(rospy.get_param(f"{base}/d")),
                i_clamp=float(rospy.get_param(f"{base}/i_clamp", 0.0)),
            )
        return gains

    def _update_gains_dynamic_reconfigure(self, gains: Dict[str, Gains]) -> None:
        assert self._dyn_client is not None
        current = self._dyn_client.get_configuration(timeout=5.0)

        # rqt_reconfigure shows nested groups like gains/j1/p.
        # The flattened keys in the dict depend on controller version.
        # We discover the actual key format from current config and then update accordingly.
        def find_key(joint: str, field: str) -> str:
            candidates = [
                f"gains__{joint}__{field}",
                f"gains_{joint}_{field}",
                f"{joint}_{field}",
                f"gains/{joint}/{field}",
                f"gains.{joint}.{field}",
            ]
            for cand in candidates:
                if cand in current:
                    return cand
            # last resort: best-effort heuristic scan
            for k in current.keys():
                if k.endswith(f"{joint}__{field}") or k.endswith(f"{joint}_{field}"):
                    return k
            raise KeyError(
                f"Cannot find dynamic_reconfigure key for gains {joint}/{field}. Available keys example: {list(current.keys())[:10]}"
            )

        updates: Dict[str, float] = {}
        for j, g in gains.items():
            for field, value in ("p", g.p), ("i", g.i), ("d", g.d), ("i_clamp", g.i_clamp):
                key = find_key(j, field)
                updates[key] = float(value)

        self._dyn_client.update_configuration(updates)

    def _reload_controller_from_param(self, controller_name: str) -> None:
        if self._srv_load is None or self._srv_unload is None or self._srv_switch is None:
            raise RuntimeError("controller_manager services unavailable; cannot reload controller")

        # Stop
        self._srv_switch(start_controllers=[], stop_controllers=[controller_name], strictness=2)
        # Unload + Load (forces param reread)
        self._srv_unload(controller_name)
        self._srv_load(controller_name)
        # Start
        self._srv_switch(start_controllers=[controller_name], stop_controllers=[], strictness=2)

        # The action server restarts with the controller; rebuild client to avoid stale connections.
        self._traj_client = actionlib.SimpleActionClient(
            f"{self.controller_ns}/follow_joint_trajectory", FollowJointTrajectoryAction
        )
        if not self._traj_client.wait_for_server(rospy.Duration(10.0)):
            raise RuntimeError(
                f"Timeout waiting for action server {self.controller_ns}/follow_joint_trajectory after reload"
            )

    def apply_gains(self, gains: Dict[str, Gains], controller_name: Optional[str] = None) -> None:
        if controller_name is None:
            controller_name = self.controller_ns.strip("/").split("/")[-1]
        if self._dyn_client is not None:
            try:
                self._update_gains_dynamic_reconfigure(gains)
                return
            except Exception as ex:
                rospy.logwarn("dynamic_reconfigure update failed (%s), falling back to controller reload", ex)

        # Fallback: set params + reload controller
        for j, g in gains.items():
            base = f"{self.controller_ns}/gains/{j}"
            rospy.set_param(f"{base}/p", float(g.p))
            rospy.set_param(f"{base}/i", float(g.i))
            rospy.set_param(f"{base}/d", float(g.d))
            rospy.set_param(f"{base}/i_clamp", float(g.i_clamp))

        self._reload_controller_from_param(controller_name)

    def send_joint_trajectory(self, positions: Dict[str, float], duration_s: float) -> Tuple[bool, int, str]:
        goal = self._make_goal(positions, duration_s=duration_s)

        self._traj_client.send_goal(goal)
        ok = self._traj_client.wait_for_result(rospy.Duration.from_sec(duration_s + 3.0))
        if not ok:
            self._traj_client.cancel_goal()
            return False, -100, "timeout"
        res = self._traj_client.get_result()
        if res is None:
            return False, -101, "no result"
        code = int(getattr(res, "error_code", 0))
        msg = str(getattr(res, "error_string", ""))
        return code == 0, code, msg

    def evaluate(self, home: Dict[str, float], target: Dict[str, float], move_s: float = 3.0, settle_s: float = 0.25) -> Tuple[float, Dict[str, float]]:
        """Single-scenario score: home -> target -> home.

        Returns (cost, final_abs_error_by_joint). Lower is better.
        """
        # Go home (don't hard-fail tuning if controller aborts; we still score from joint_states)
        ok_home, code_home, msg_home = self.send_joint_trajectory(home, duration_s=move_s)
        if not ok_home:
            rospy.logwarn(
                "Go-home failed (code=%s %s msg=%s); continuing evaluation",
                code_home,
                decode_fjt_error(code_home),
                msg_home,
            )
        rospy.sleep(0.2)

        # Execute target and score
        start_t = _now()
        self._traj_client.send_goal(self._make_goal(target, duration_s=move_s))

        samples: List[Tuple[float, Dict[str, float]]] = []
        rate = rospy.Rate(100)
        end_t = start_t + move_s + float(settle_s)
        while not rospy.is_shutdown() and _now() < end_t:
            t = _now() - start_t
            samples.append((t, self._js.positions()))
            rate.sleep()

        self._traj_client.wait_for_result(rospy.Duration.from_sec(move_s + 3.0))
        res = self._traj_client.get_result()
        exec_code = int(getattr(res, "error_code", -102)) if res is not None else -102
        exec_msg = str(getattr(res, "error_string", "")) if res is not None else ""

        # Final error
        final_pos = self._js.positions()
        final_err = {j: abs(final_pos[j] - target[j]) for j in self.joint_names}

        # Integrated squared error (ISE) using a simple desired linear interpolation (good enough for comparing gains)
        # Desired position is linearly interpolated from home->target over move_s.
        ise = 0.0
        peak_vel = 0.0
        prev_t, prev_pos = samples[0]
        for t, pos in samples[1:]:
            alpha = min(max(t / move_s, 0.0), 1.0)
            for j in self.joint_names:
                desired = home[j] * (1.0 - alpha) + target[j] * alpha
                e = pos[j] - desired
                ise += e * e
            dt = max(t - prev_t, 1e-4)
            # crude peak velocity metric
            vel = 0.0
            for j in self.joint_names:
                vel = max(vel, abs((pos[j] - prev_pos[j]) / dt))
            peak_vel = max(peak_vel, vel)
            prev_t, prev_pos = t, pos

        # Penalize aborted executions but still allow comparisons.
        abort_penalty = 0.0 if exec_code == 0 else 500.0
        # Also penalize failing to return home (often indicates instability / controller limits)
        home_penalty = 0.0 if ok_home else 200.0

        # Weight heavier joints more by default
        weights = {
            "j1": 2.0,
            "j2": 3.0,
            "j3": 3.0,
            "j4": 1.0,
            "j5": 1.0,
            "j6": 1.0,
        }
        weighted_final = sum(weights.get(j, 1.0) * e for j, e in final_err.items())
        max_err = max(final_err.values()) if final_err else 0.0

        # Add a small settling penalty: residual motion at the end implies oscillation.
        # Estimate from last 20 samples.
        settling_pen = 0.0
        if len(samples) >= 25:
            tail = samples[-20:]
            v_sum = 0.0
            v_n = 0
            for (t0, p0), (t1, p1) in zip(tail[:-1], tail[1:]):
                dt = max(t1 - t0, 1e-4)
                for j in self.joint_names:
                    v_sum += abs((p1[j] - p0[j]) / dt)
                    v_n += 1
            settling_pen = (v_sum / max(v_n, 1)) * 0.05

        cost = 80.0 * weighted_final + 20.0 * max_err + 0.5 * ise + 0.3 * peak_vel + settling_pen + abort_penalty + home_penalty

        if exec_code != 0:
            rospy.logdebug("Exec failed (code=%s %s msg=%s)", exec_code, decode_fjt_error(exec_code), exec_msg)

        # Return home to normalize next run (best-effort)
        self.send_joint_trajectory(home, duration_s=move_s)
        rospy.sleep(0.2)

        return float(cost), final_err


def make_targets(
    rng: random.Random,
    home: Dict[str, float],
    joint_names: List[str],
    limits: Optional[Dict[str, JointLimit]],
    delta: float,
    scenarios: int,
) -> List[Dict[str, float]]:
    """Generate multiple small targets around home for more general tuning."""

    targets: List[Dict[str, float]] = []

    # Prefer exciting heavier joints more often
    heavy = ["j1", "j2", "j3"]
    wrist = ["j4", "j5", "j6"]

    for _ in range(scenarios):
        tgt = dict(home)

        # Random signs
        s1 = rng.choice([-1.0, 1.0])
        s2 = rng.choice([-1.0, 1.0])
        s3 = rng.choice([-1.0, 1.0])

        # Always excite shoulder/elbow a bit to include gravity load
        if "j2" in tgt:
            tgt["j2"] = home["j2"] + s2 * delta
        if "j3" in tgt:
            tgt["j3"] = home["j3"] - s2 * delta
        if "j1" in tgt:
            tgt["j1"] = home["j1"] + s1 * (0.6 * delta)

        # Sometimes add a wrist perturbation to avoid ignoring j4-j6
        if rng.random() < 0.6:
            wj = rng.choice([j for j in wrist if j in tgt])
            tgt[wj] = home[wj] + s3 * (0.5 * delta)

        # Clamp to limits if available; else clamp to a conservative range
        for j in joint_names:
            lo, hi = -3.14, 3.14
            if limits and j in limits:
                lo, hi = limits[j].lower, limits[j].upper
            tgt[j] = _clamp(float(tgt[j]), float(lo), float(hi))

        targets.append(tgt)

    return targets


def evaluate_multi(
    tuner: EffortControllerTuner,
    home: Dict[str, float],
    targets: List[Dict[str, float]],
    move_s: float,
    settle_s: float,
) -> Tuple[float, Dict[str, float]]:
    """Aggregate score across scenarios.

    Returns (aggregate_cost, worst_case_final_err_by_joint).
    """

    costs: List[float] = []
    worst: Dict[str, float] = {j: 0.0 for j in tuner.joint_names}
    for tgt in targets:
        c, err = tuner.evaluate(home=home, target=tgt, move_s=move_s, settle_s=settle_s)
        costs.append(c)
        for j, e in err.items():
            worst[j] = max(worst.get(j, 0.0), float(e))

    if not costs:
        return 1e9, {j: 1e6 for j in tuner.joint_names}

    # Combine mean + max to encourage robustness
    mean_c = sum(costs) / len(costs)
    max_c = max(costs)
    agg = mean_c + 0.35 * max_c
    return float(agg), worst

    def _make_goal(self, positions: Dict[str, float], duration_s: float) -> FollowJointTrajectoryGoal:
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = list(self.joint_names)
        goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.05)

        pt = JointTrajectoryPoint()
        pt.positions = [float(positions[j]) for j in self.joint_names]
        # Provide explicit end velocities to help the controller settle.
        pt.velocities = [0.0 for _ in self.joint_names]
        pt.time_from_start = rospy.Duration.from_sec(float(duration_s))
        goal.trajectory.points = [pt]
        goal.goal_time_tolerance = rospy.Duration.from_sec(1.0)

        # Make the controller less likely to abort during tuning; we judge quality via our own cost.
        # (Too-strict tolerances cause constant GOAL_TOLERANCE_VIOLATED and no signal for optimization.)
        goal.goal_tolerance = [JointTolerance(name=j, position=2.0, velocity=2.0, acceleration=0.0) for j in self.joint_names]
        goal.path_tolerance = [JointTolerance(name=j, position=4.0, velocity=4.0, acceleration=0.0) for j in self.joint_names]
        return goal


def _loguniform(rng: random.Random, lo: float, hi: float) -> float:
    lo = max(lo, 1e-9)
    hi = max(hi, lo * 1.000001)
    return math.exp(rng.uniform(math.log(lo), math.log(hi)))


def format_gains_yaml(gains: Dict[str, Gains]) -> str:
    lines = ["gains:"]
    for j in JOINTS_DEFAULT:
        if j not in gains:
            continue
        g = gains[j]
        lines.append(f"  {j}:")
        lines.append(f"    p: {g.p:.6g}")
        lines.append(f"    d: {g.d:.6g}")
        lines.append(f"    i: {g.i:.6g}")
        lines.append(f"    i_clamp: {g.i_clamp:.6g}")
    return "\n".join(lines) + "\n"


def apply_gains_to_ros_controllers_yaml(file_path: str, gains: Dict[str, Gains], controller_key: str = "arm_effort_controller") -> None:
    """Update only the gains fields inside an existing ros_controllers.yaml.

    We avoid external deps and preserve most of the file by doing a targeted text update:
    - Find the controller block
    - Replace the `gains:` mapping for that controller
    """

    with open(file_path, "r", encoding="utf-8") as f:
        text = f.read()

    # Build replacement YAML snippet with correct indentation (2 for controller fields, 4 for joints)
    lines: List[str] = []
    lines.append("  gains:")
    for j in JOINTS_DEFAULT:
        if j not in gains:
            continue
        g = gains[j]
        lines.append(f"    {j}:")
        lines.append(f"      p: {g.p:.6g}")
        lines.append(f"      d: {g.d:.6g}")
        lines.append(f"      i: {g.i:.6g}")
        lines.append(f"      i_clamp: {g.i_clamp:.6g}")
    new_block = "\n".join(lines) + "\n"

    # Locate controller top-level key
    m_ctrl = re.search(rf"^({re.escape(controller_key)}):\s*$", text, flags=re.MULTILINE)
    if not m_ctrl:
        raise RuntimeError(f"Cannot find controller '{controller_key}:' in {file_path}")

    # From controller start, find the gains section and replace it until next same-indent key
    start = m_ctrl.end()
    tail = text[start:]
    m_gains = re.search(r"^\s{2}gains:\s*$", tail, flags=re.MULTILINE)
    if not m_gains:
        raise RuntimeError(f"Cannot find '  gains:' under {controller_key} in {file_path}")

    gains_start = start + m_gains.start()
    after_gains_header = start + m_gains.end()

    # Find end of gains mapping: first line that begins with two spaces and is NOT part of gains mapping.
    # Gains mapping entries are indented >=4 spaces.
    rest = text[after_gains_header:]
    m_end = re.search(r"^\s{2}\S", rest, flags=re.MULTILINE)
    gains_end = after_gains_header + (m_end.start() if m_end else len(rest))

    updated = text[:gains_start] + new_block + text[gains_end:]
    with open(file_path, "w", encoding="utf-8") as f:
        f.write(updated)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--controller", default="/arm_effort_controller", help="Controller namespace (default: /arm_effort_controller)")
    ap.add_argument("--iters", type=int, default=60, help="Number of candidates to evaluate")
    ap.add_argument("--seed", type=int, default=0, help="Random seed (0 => time-based)")
    ap.add_argument("--out", default="/tmp/fr5_pid_best.yaml", help="Where to write best gains YAML")
    ap.add_argument(
        "--apply",
        action="store_true",
        help="Write best gains into fr5_moveit_effort_config/config/ros_controllers.yaml (and reload controller)",
    )
    ap.add_argument(
        "--ros-controllers-yaml",
        default="",
        help="Path to ros_controllers.yaml to patch (defaults to the one in fr5_moveit_effort_config)",
    )
    ap.add_argument("--delta", type=float, default=0.25, help="Test motion delta (rad) applied to a subset of joints")

    ap.add_argument("--scenarios", type=int, default=3, help="How many target poses to test per candidate")
    ap.add_argument("--move-seconds", type=float, default=2.5, help="Seconds per move in each scenario")
    ap.add_argument("--settle-seconds", type=float, default=0.25, help="Extra settling time after reaching target")

    # Stronger optimizer mode
    ap.add_argument("--cem", action="store_true", help="Use Cross-Entropy Method optimizer (recommended)")
    ap.add_argument("--cem-pop", type=int, default=24, help="CEM population size per generation")
    ap.add_argument("--cem-elite", type=int, default=6, help="CEM elite count per generation")
    ap.add_argument("--cem-gens", type=int, default=8, help="CEM generations (total evals ~ pop*gens)")

    # Bounds (kept conservative to avoid instability)
    ap.add_argument("--p_min", type=float, default=50.0)
    ap.add_argument("--p_max", type=float, default=2500.0)
    ap.add_argument("--d_min", type=float, default=0.5)
    ap.add_argument("--d_max", type=float, default=200.0)
    ap.add_argument("--i_min", type=float, default=0.0)
    ap.add_argument("--i_max", type=float, default=2.0)
    ap.add_argument("--i_clamp_min", type=float, default=0.0)
    ap.add_argument("--i_clamp_max", type=float, default=50.0)

    args = ap.parse_args(rospy.myargv()[1:])

    rospy.init_node("fr5_pid_autotune", anonymous=True)

    rng = random.Random()
    rng.seed(time.time() if args.seed == 0 else args.seed)

    tuner = EffortControllerTuner(controller_ns=args.controller, joint_names=JOINTS_DEFAULT)

    limits = try_read_joint_limits_from_urdf(JOINTS_DEFAULT)
    if limits:
        rospy.loginfo("Loaded joint limits from /robot_description")
    else:
        rospy.loginfo("No URDF joint limits found; using conservative [-pi, pi] clamp")

    # Establish a repeatable home pose from current state
    home = tuner._js.positions()

    targets = make_targets(
        rng=rng,
        home=home,
        joint_names=JOINTS_DEFAULT,
        limits=limits,
        delta=float(args.delta),
        scenarios=int(args.scenarios),
    )

    baseline = tuner.read_gains_from_param()

    best_gains = dict(baseline)
    best_cost, best_err = evaluate_multi(
        tuner=tuner,
        home=home,
        targets=targets,
        move_s=float(args.move_seconds),
        settle_s=float(args.settle_seconds),
    )
    rospy.loginfo("Baseline cost=%.3f worst_err=%s", best_cost, best_err)

    # Progressive narrowing around best (random mode) / initial bounds (CEM mode)
    p_lo, p_hi = args.p_min, args.p_max
    d_lo, d_hi = args.d_min, args.d_max
    i_lo, i_hi = args.i_min, args.i_max
    c_lo, c_hi = args.i_clamp_min, args.i_clamp_max

    def sample_candidate_around(best: Dict[str, Gains]) -> Dict[str, Gains]:
        cand = dict(best)
        for j in ["j1", "j2", "j3"]:
            p = _loguniform(rng, p_lo, p_hi)
            d_center = 2.0 * math.sqrt(max(p, 1e-6))
            d = _loguniform(rng, max(d_lo, 0.3 * d_center), min(d_hi, 3.0 * d_center))
            cand[j] = Gains(
                p=p,
                d=d,
                i=_clamp(rng.uniform(i_lo, i_hi), 0.0, args.i_max),
                i_clamp=_clamp(rng.uniform(c_lo, c_hi), 0.0, args.i_clamp_max),
            )
        for j in ["j4", "j5", "j6"]:
            base = best[j]
            cand[j] = Gains(
                p=max(10.0, base.p * _loguniform(rng, 0.7, 1.3)),
                d=max(0.1, base.d * _loguniform(rng, 0.7, 1.3)),
                i=0.0,
                i_clamp=0.0,
            )
        return cand

    if args.cem:
        # Cross-Entropy Method (CEM) over (log P, log D, I, I_clamp) for j1-j3
        pop = int(args.cem_pop)
        elite = int(args.cem_elite)
        gens = int(args.cem_gens)
        if elite <= 0 or elite >= pop:
            raise RuntimeError("cem-elite must be >0 and < cem-pop")

        # Initialize distribution from current best
        # mu/sigma per joint+param
        mu: Dict[Tuple[str, str], float] = {}
        sig: Dict[Tuple[str, str], float] = {}
        for j in ["j1", "j2", "j3"]:
            mu[(j, "logp")] = math.log(max(best_gains[j].p, 1e-6))
            mu[(j, "logd")] = math.log(max(best_gains[j].d, 1e-6))
            mu[(j, "i")] = float(best_gains[j].i)
            mu[(j, "c")] = float(best_gains[j].i_clamp)

            sig[(j, "logp")] = 0.7
            sig[(j, "logd")] = 0.7
            sig[(j, "i")] = 0.4
            sig[(j, "c")] = 15.0

        for g in range(gens):
            scored: List[Tuple[float, Dict[str, Gains], Dict[str, float]]] = []
            for k in range(pop):
                cand = dict(best_gains)
                for j in ["j1", "j2", "j3"]:
                    lp = rng.gauss(mu[(j, "logp")], sig[(j, "logp")])
                    ld = rng.gauss(mu[(j, "logd")], sig[(j, "logd")])
                    p = math.exp(lp)
                    d = math.exp(ld)
                    ii = _clamp(rng.gauss(mu[(j, "i")], sig[(j, "i")]), args.i_min, args.i_max)
                    cc = _clamp(rng.gauss(mu[(j, "c")], sig[(j, "c")]), args.i_clamp_min, args.i_clamp_max)
                    cand[j] = Gains(p=p, d=d, i=ii, i_clamp=cc)

                for j in ["j4", "j5", "j6"]:
                    base = best_gains[j]
                    cand[j] = Gains(
                        p=max(10.0, base.p * _loguniform(rng, 0.85, 1.15)),
                        d=max(0.1, base.d * _loguniform(rng, 0.85, 1.15)),
                        i=0.0,
                        i_clamp=0.0,
                    )

                try:
                    tuner.apply_gains(cand)
                except Exception as ex:
                    rospy.logwarn("Failed to apply candidate gains: %s", ex)
                    continue

                cost, worst_err = evaluate_multi(
                    tuner=tuner,
                    home=home,
                    targets=targets,
                    move_s=float(args.move_seconds),
                    settle_s=float(args.settle_seconds),
                )
                scored.append((cost, cand, worst_err))
                rospy.loginfo(
                    "cem gen %d/%d cand %d/%d cost=%.3f worst_err=%s",
                    g + 1,
                    gens,
                    k + 1,
                    pop,
                    cost,
                    worst_err,
                )

                if cost < best_cost:
                    best_cost = cost
                    best_gains = cand
                    best_err = worst_err
                    rospy.loginfo("New best: cost=%.3f worst_err=%s", best_cost, best_err)

            if not scored:
                rospy.logwarn("CEM generation %d produced no valid candidates", g + 1)
                continue

            scored.sort(key=lambda x: x[0])
            elites = scored[:elite]

            # Update distribution from elites
            for j in ["j1", "j2", "j3"]:
                vals_lp = [math.log(max(cand[j].p, 1e-6)) for _, cand, _ in elites]
                vals_ld = [math.log(max(cand[j].d, 1e-6)) for _, cand, _ in elites]
                vals_i = [cand[j].i for _, cand, _ in elites]
                vals_c = [cand[j].i_clamp for _, cand, _ in elites]

                def mean(xs: List[float]) -> float:
                    return sum(xs) / max(1, len(xs))

                def std(xs: List[float], m: float) -> float:
                    if len(xs) <= 1:
                        return 1e-6
                    return math.sqrt(sum((x - m) ** 2 for x in xs) / (len(xs) - 1))

                mu[(j, "logp")] = mean(vals_lp)
                mu[(j, "logd")] = mean(vals_ld)
                mu[(j, "i")] = mean(vals_i)
                mu[(j, "c")] = mean(vals_c)

                # keep some exploration
                sig[(j, "logp")] = max(0.15, std(vals_lp, mu[(j, "logp")]) * 0.9)
                sig[(j, "logd")] = max(0.15, std(vals_ld, mu[(j, "logd")]) * 0.9)
                sig[(j, "i")] = max(0.05, std(vals_i, mu[(j, "i")]) * 0.9)
                sig[(j, "c")] = max(3.0, std(vals_c, mu[(j, "c")]) * 0.9)

    else:
        for k in range(int(args.iters)):
            cand = sample_candidate_around(best_gains)

            try:
                tuner.apply_gains(cand)
            except Exception as ex:
                rospy.logwarn("Failed to apply candidate gains: %s", ex)
                continue

            cost, worst_err = evaluate_multi(
                tuner=tuner,
                home=home,
                targets=targets,
                move_s=float(args.move_seconds),
                settle_s=float(args.settle_seconds),
            )
            rospy.loginfo("iter %d/%d cost=%.3f worst_err=%s", k + 1, args.iters, cost, worst_err)

            if cost < best_cost:
                best_cost = cost
                best_gains = cand
                best_err = worst_err
                rospy.loginfo("New best: cost=%.3f worst_err=%s", best_cost, best_err)

            if (k + 1) % 10 == 0:
                p_lo, p_hi = max(args.p_min, p_lo * 1.2), max(args.p_min * 1.5, p_hi * 0.85)
                d_lo, d_hi = max(args.d_min, d_lo * 1.2), max(args.d_min * 1.5, d_hi * 0.85)
                i_hi = max(0.2, i_hi * 0.85)
                c_hi = max(5.0, c_hi * 0.85)

    yaml_text = format_gains_yaml(best_gains)
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as f:
        f.write(yaml_text)

    rospy.loginfo("Best cost=%.3f worst_err=%s", best_cost, best_err)
    rospy.loginfo("Wrote %s", args.out)
    print("\n===== BEST GAINS (paste into ros_controllers.yaml) =====\n")
    print(yaml_text)

    if args.apply:
        # Patch the yaml in the MoveIt config so next launch uses it.
        yaml_path = args.ros_controllers_yaml
        if not yaml_path:
            try:
                import rospkg  # type: ignore

                rp = rospkg.RosPack()
                yaml_path = os.path.join(rp.get_path("fr5_moveit_effort_config"), "config", "ros_controllers.yaml")
            except Exception as ex:
                raise RuntimeError(
                    "--apply requested but cannot auto-locate fr5_moveit_effort_config; pass --ros-controllers-yaml explicitly"
                ) from ex

        apply_gains_to_ros_controllers_yaml(yaml_path, best_gains, controller_key="arm_effort_controller")
        rospy.loginfo("Patched %s", yaml_path)

        # Also apply gains live (best-effort)
        try:
            tuner.apply_gains(best_gains)
            rospy.loginfo("Applied best gains live to %s", args.controller)
        except Exception as ex:
            rospy.logwarn("Could not apply gains live: %s", ex)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

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

from __future__ import annotations

import argparse
import dataclasses
import math
import os
import random
import time
from typing import Dict, List, Optional, Tuple

import rospy
from sensor_msgs.msg import JointState

import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint


JOINTS_DEFAULT = ["j1", "j2", "j3", "j4", "j5", "j6"]


@dataclasses.dataclass
class Gains:
    p: float
    i: float
    d: float
    i_clamp: float


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

    def send_joint_trajectory(self, positions: Dict[str, float], duration_s: float) -> bool:
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = list(self.joint_names)
        pt = JointTrajectoryPoint()
        pt.positions = [float(positions[j]) for j in self.joint_names]
        pt.time_from_start = rospy.Duration.from_sec(float(duration_s))
        goal.trajectory.points = [pt]
        goal.goal_time_tolerance = rospy.Duration.from_sec(1.0)

        self._traj_client.send_goal(goal)
        ok = self._traj_client.wait_for_result(rospy.Duration.from_sec(duration_s + 3.0))
        if not ok:
            self._traj_client.cancel_goal()
            return False
        res = self._traj_client.get_result()
        # res is FollowJointTrajectoryResult; treat any non-zero error_code as failure.
        return getattr(res, "error_code", 0) == 0

    def evaluate(self, home: Dict[str, float], target: Dict[str, float], move_s: float = 3.0) -> Tuple[float, Dict[str, float]]:
        """Returns (cost, final_abs_error_by_joint). Lower is better."""
        # Go home
        if not self.send_joint_trajectory(home, duration_s=move_s):
            return 1e9, {j: 1e6 for j in self.joint_names}
        rospy.sleep(0.3)

        # Execute target and score
        start_t = _now()
        self._traj_client.send_goal(self._make_goal(target, duration_s=move_s))

        samples: List[Tuple[float, Dict[str, float]]] = []
        rate = rospy.Rate(100)
        end_t = start_t + move_s + 0.3
        while not rospy.is_shutdown() and _now() < end_t:
            t = _now() - start_t
            samples.append((t, self._js.positions()))
            rate.sleep()

        self._traj_client.wait_for_result(rospy.Duration.from_sec(2.0))

        # Final error
        final_pos = self._js.positions()
        final_err = {j: abs(final_pos[j] - target[j]) for j in self.joint_names}

        # Integrated squared error (ISE) using a simple desired step to target (good enough for comparing gains)
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

        cost = 20.0 * sum(final_err.values()) + 0.5 * ise + 0.2 * peak_vel

        # Return home to normalize next run
        self.send_joint_trajectory(home, duration_s=move_s)
        rospy.sleep(0.2)

        return float(cost), final_err

    def _make_goal(self, positions: Dict[str, float], duration_s: float) -> FollowJointTrajectoryGoal:
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = list(self.joint_names)
        pt = JointTrajectoryPoint()
        pt.positions = [float(positions[j]) for j in self.joint_names]
        pt.time_from_start = rospy.Duration.from_sec(float(duration_s))
        goal.trajectory.points = [pt]
        goal.goal_time_tolerance = rospy.Duration.from_sec(1.0)
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


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--controller", default="/arm_effort_controller", help="Controller namespace (default: /arm_effort_controller)")
    ap.add_argument("--iters", type=int, default=60, help="Number of candidates to evaluate")
    ap.add_argument("--seed", type=int, default=0, help="Random seed (0 => time-based)")
    ap.add_argument("--out", default="/tmp/fr5_pid_best.yaml", help="Where to write best gains YAML")
    ap.add_argument("--delta", type=float, default=0.25, help="Test motion delta (rad) applied to a subset of joints")

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

    # Establish a repeatable home pose from current state
    home = tuner._js.positions()

    # Build a simple reachable target by adding small deltas (avoid limits; user can edit later)
    target = dict(home)
    # Bias the heavier joints slightly (common gravity-load culprits)
    target["j2"] = home["j2"] - float(args.delta)
    target["j3"] = home["j3"] + float(args.delta)
    target["j1"] = home["j1"] + float(args.delta) * 0.5

    baseline = tuner.read_gains_from_param()

    best_gains = dict(baseline)
    best_cost, best_err = tuner.evaluate(home=home, target=target)
    rospy.loginfo("Baseline cost=%.3f final_err=%s", best_cost, best_err)

    # Progressive narrowing around best
    p_lo, p_hi = args.p_min, args.p_max
    d_lo, d_hi = args.d_min, args.d_max
    i_lo, i_hi = args.i_min, args.i_max
    c_lo, c_hi = args.i_clamp_min, args.i_clamp_max

    for k in range(int(args.iters)):
        cand = dict(best_gains)

        # Tune j1-j3 aggressively; keep wrist conservative
        for j in ["j1", "j2", "j3"]:
            cand[j] = Gains(
                p=_loguniform(rng, p_lo, p_hi),
                d=_loguniform(rng, d_lo, d_hi),
                i=rng.uniform(i_lo, i_hi),
                i_clamp=rng.uniform(c_lo, c_hi),
            )

        # keep wrist mostly PD
        for j in ["j4", "j5", "j6"]:
            base = best_gains[j]
            cand[j] = Gains(
                p=max(10.0, base.p * _loguniform(rng, 0.7, 1.3)),
                d=max(0.1, base.d * _loguniform(rng, 0.7, 1.3)),
                i=0.0,
                i_clamp=0.0,
            )

        try:
            tuner.apply_gains(cand)
        except Exception as ex:
            rospy.logwarn("Failed to apply candidate gains: %s", ex)
            continue

        cost, final_err = tuner.evaluate(home=home, target=target)
        rospy.loginfo("iter %d/%d cost=%.3f final_err=%s", k + 1, args.iters, cost, final_err)

        if cost < best_cost:
            best_cost = cost
            best_gains = cand
            best_err = final_err
            rospy.loginfo("New best: cost=%.3f err=%s", best_cost, best_err)

        # shrink bounds every 10 iters
        if (k + 1) % 10 == 0:
            p_lo, p_hi = max(args.p_min, p_lo * 1.2), max(args.p_min * 1.5, p_hi * 0.85)
            d_lo, d_hi = max(args.d_min, d_lo * 1.2), max(args.d_min * 1.5, d_hi * 0.85)
            i_hi = max(0.2, i_hi * 0.85)
            c_hi = max(5.0, c_hi * 0.85)

    yaml_text = format_gains_yaml(best_gains)
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as f:
        f.write(yaml_text)

    rospy.loginfo("Best cost=%.3f final_err=%s", best_cost, best_err)
    rospy.loginfo("Wrote %s", args.out)
    print("\n===== BEST GAINS (paste into ros_controllers.yaml) =====\n")
    print(yaml_text)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

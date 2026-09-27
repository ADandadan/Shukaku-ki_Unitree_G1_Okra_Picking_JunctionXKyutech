"""
g1_kinematics.py - G1 arm kinematics from the robot's own URDF (numpy only). Read-only maths.

  * Chain       - forward kinematics torso_link -> hand tool point for one arm (7 joints)
  * camera_to_torso(p) - head D435i colour OPTICAL frame point (x right, y down, z forward,
                  as okra_vision produces) -> torso_link frame (x forward, y left, z up)
  * Chain.ik    - position-only inverse kinematics (damped least squares, joint limits,
                  pulled towards the current posture so the arm doesn't wander)

Camera and arms both hang off torso_link, so the waist angles don't matter here.
URDF: urdf/g1_29dof_mode_15_with_dex1_1.urdf (copied read-only from the robot's ~/g1_description).
Motor order in rt/lowstate / rt/arm_sdk: left arm 15-21, right arm 22-28 (see arm_test.J).
"""
from __future__ import annotations

import os
import xml.etree.ElementTree as ET
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
URDF = os.path.join(HERE, "urdf", "g1_29dof_mode_15_with_dex1_1.urdf")

# Tool points in the wrist_yaw_link frame (x along the fingers). Toyota-body team measured the plain
# Dex1 fingertip at 18.45 cm; the grasp centre (middle of the finger pads) is a first guess - calibrate.
TOOL_TIP = (0.1845, 0.0, 0.0)
TOOL_GRASP = (0.15, 0.0, 0.0)

ARM_JOINTS = ["shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow",
              "wrist_roll", "wrist_pitch", "wrist_yaw"]
FIRST_MOTOR = {"left": 15, "right": 22}


def rpy_matrix(r: float, p: float, y: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def axis_angle(axis: np.ndarray, a: float) -> np.ndarray:
    x, y, z = axis / np.linalg.norm(axis)
    c, s, C = np.cos(a), np.sin(a), 1 - np.cos(a)
    return np.array([[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
                     [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
                     [z * x * C - y * s, z * y * C + x * s, c + z * z * C]])


def _floats(s: Optional[str], n: int = 3) -> np.ndarray:
    return np.array([float(v) for v in s.split()]) if s else np.zeros(n)


def load_joints(path: str = URDF) -> Dict[str, dict]:
    root = ET.parse(path).getroot()
    out = {}
    for j in root.iter("joint"):
        o, ax, lim = j.find("origin"), j.find("axis"), j.find("limit")
        out[j.get("name")] = dict(
            type=j.get("type"), parent=j.find("parent").get("link"), child=j.find("child").get("link"),
            xyz=_floats(o.get("xyz") if o is not None else None),
            rpy=_floats(o.get("rpy") if o is not None else None),
            axis=_floats(ax.get("xyz")) if ax is not None else np.array([1.0, 0, 0]),
            lower=float(lim.get("lower")) if lim is not None and lim.get("lower") else -np.inf,
            upper=float(lim.get("upper")) if lim is not None and lim.get("upper") else np.inf)
    return out


def _fixed(j: dict) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = rpy_matrix(*j["rpy"])
    T[:3, 3] = j["xyz"]
    return T


class Chain:
    """torso_link -> <side>_wrist_yaw_link (+ tool offset), 7 revolute joints."""

    def __init__(self, side: str = "right", tool: Optional[Sequence[float]] = None, path: str = URDF):
        js = load_joints(path)
        self.side = side
        self.names = [f"{side}_{n}_joint" for n in ARM_JOINTS]
        self.joints = [js[n] for n in self.names]
        if self.joints[0]["parent"] != "torso_link":
            raise ValueError("expected the arm chain to start at torso_link")
        self.motors = list(range(FIRST_MOTOR[side], FIRST_MOTOR[side] + 7))
        self.lower = np.array([j["lower"] for j in self.joints])
        self.upper = np.array([j["upper"] for j in self.joints])
        self.tool = np.asarray(TOOL_GRASP if tool is None else tool, float)  # module value read at call time

    def frames(self, q: Sequence[float]) -> List[np.ndarray]:
        """World (torso) transform of each joint frame after rotation, then the tool frame."""
        T, out = np.eye(4), []
        for j, qi in zip(self.joints, q):
            T = T @ _fixed(j)
            R = np.eye(4)
            R[:3, :3] = axis_angle(j["axis"], qi)
            T = T @ R
            out.append(T.copy())
        tool = np.eye(4)
        tool[:3, 3] = self.tool
        out.append(T @ tool)
        return out

    def fk(self, q: Sequence[float]) -> np.ndarray:
        """Tool point position in torso_link frame [m]."""
        return self.frames(q)[-1][:3, 3]

    def jacobian(self, q: Sequence[float]) -> np.ndarray:
        """3x7 positional Jacobian (geometric, revolute joints)."""
        fr = self.frames(q)
        p = fr[-1][:3, 3]
        J = np.zeros((3, 7))
        for i, (j, T) in enumerate(zip(self.joints, fr[:-1])):
            z = T[:3, :3] @ (j["axis"] / np.linalg.norm(j["axis"]))
            J[:, i] = np.cross(z, p - T[:3, 3])
        return J

    def ik(self, target: Sequence[float], q_start: Sequence[float], iters: int = 200, tol: float = 1e-4,
           damping: float = 0.02, rest_gain: float = 0.05, margin: float = 0.05,
           fixed: Sequence[int] = ()) -> Tuple[np.ndarray, float]:
        """Position-only IK. Returns (q, residual_m). Stays within URDF limits minus `margin`
        and, in the null space, prefers joints near q_start (least motion)."""
        target = np.asarray(target, float)
        q = np.clip(np.asarray(q_start, float), self.lower + margin, self.upper - margin)
        q_rest = q.copy()
        for _ in range(iters):
            err = target - self.fk(q)
            if np.linalg.norm(err) < tol:
                break
            J = self.jacobian(q)
            J[:, list(fixed)] = 0.0   # joints held at their q_start value (e.g. wrist roll = finger orientation)
            JJt = J @ J.T + damping ** 2 * np.eye(3)
            J_pinv = J.T @ np.linalg.inv(JJt)
            dq = J_pinv @ err + (np.eye(7) - J_pinv @ J) @ (rest_gain * (q_rest - q))
            dq[list(fixed)] = 0.0
            step = np.linalg.norm(dq)
            if step > 0.2:  # limit per-iteration change (rad) for stability
                dq *= 0.2 / step
            q = np.clip(q + dq, self.lower + margin, self.upper - margin)
        return q, float(np.linalg.norm(target - self.fk(q)))


# ---- head camera ---------------------------------------------------------------------------
_OPT_TO_LINK = np.array([[0.0, 0.0, 1.0],     # link x (forward) = optical z
                         [-1.0, 0.0, 0.0],    # link y (left)    = -optical x
                         [0.0, -1.0, 0.0]])   # link z (up)      = -optical y


def camera_pose(path: str = URDF) -> np.ndarray:
    """4x4 transform torso_link <- d435_link (URDF d435_joint). Assumes d435_link is a
    REP-103 body frame (x forward, y left, z up) at the colour sensor; the D435's colour
    sensor is ~1.5 cm to the side of its depth origin, so expect ~1-2 cm error until calibrated."""
    return _fixed(load_joints(path)["d435_joint"])


def camera_to_torso(p_opt: Sequence[float], T_cam: Optional[np.ndarray] = None) -> np.ndarray:
    T = camera_pose() if T_cam is None else T_cam
    return T[:3, :3] @ (_OPT_TO_LINK @ np.asarray(p_opt, float)) + T[:3, 3]


def torso_to_camera(p_torso: Sequence[float], T_cam: Optional[np.ndarray] = None) -> np.ndarray:
    T = camera_pose() if T_cam is None else T_cam
    return _OPT_TO_LINK.T @ (T[:3, :3].T @ (np.asarray(p_torso, float) - T[:3, 3]))

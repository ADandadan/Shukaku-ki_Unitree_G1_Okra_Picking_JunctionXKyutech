#!/usr/bin/env python3
"""
reach_test.py - *** MOVES THE ROBOT (with --live) ***  First reach: hand to a HOVER point short of a
target, hold, back. No grasping. Uses the same controller and safety checks as arm_test.py.

Target, one of:
  --target-torso X,Y,Z   torso_link frame [m]: x forward, y left, z up (origin ~ waist, see g1_kinematics)
  --target-cam X,Y,Z     head-camera optical frame [m] (what detect_okra.py prints as xyz)
  --detect               grab frames from `--source robot-depth` (run_depth_server.sh must be running),
                         tiled detection, take the best okra-shaped/sized pod seen consistently in 2 frames
The hand's grasp point goes to target - standoff along torso x (approach from behind), default 10 cm short.

Planning (always, also in the dry run): IK from the current arm pose, then refuse unless
  target in the picking box, IK error < 1 cm, every joint change <= 1.2 rad, joints >= 0.10 rad inside
  their limits, and along the whole path neither grasp point nor elbow enters the torso column
  or goes below z = -0.40 m. Joint speed <= 0.4 rad/s (move time scales with the largest change).
--snapshot: during the hold, grab one camera frame (depth server must run), draw where the kinematics
  says the fingertip / grasp point / target are, save out/reach_snapshot_*.jpg - the hand-eye check.

  python reach_test.py --target-torso 0.40,-0.20,0.00               # dry run: plan + checks only
  python reach_test.py --target-torso 0.40,-0.20,0.00 --live --snapshot
  python reach_test.py --detect --live --snapshot
Robot: motion-control mode (remote L2+Up, then R1+Y). Emergency stop: L2+B.
"""
import argparse
import csv
import os
import signal
import sys
import time

import numpy as np

import arm_test as A
import g1_kinematics as K

HERE = os.path.dirname(os.path.abspath(__file__))
# Conservative picking box, torso frame [m]; mirrored in y for the left arm. The Toyota-body team's
# advice: targets to the side of the reaching arm at chest height; low targets make the arm sweep.
BOX = dict(x=(0.20, 0.50), y=(-0.40, 0.10), z=(-0.30, 0.20))
BODY = dict(x=(-0.15, 0.12), y=(-0.12, 0.12), z=(-0.50, 0.50))   # keep hand + elbow out; shoulder joints are at y=+-0.10
MIN_Z = -0.40
MAX_DQ = 1.2             # rad per joint for these first reaches
MAX_IK_ERR = 0.01        # m
JOINT_SPEED = 0.4        # rad/s (peak speed of the smooth profile is ~1.6x this average)
MIN_T_MOVE = 3.0


def in_box(p, box):
    return all(lo <= v <= hi for v, (lo, hi) in zip(p, box.values()))


def side_box(side):
    if side == "right":
        return BOX
    return dict(x=BOX["x"], y=(-BOX["y"][1], -BOX["y"][0]), z=BOX["z"])


def limit_problems(chain, q, label="goal"):
    out = []
    for j, motor in enumerate(chain.motors):
        lo, hi = A.LIMITS[motor]
        if not (lo + A.LIMIT_MARGIN <= q[j] <= hi - A.LIMIT_MARGIN):
            out.append(f"{K.ARM_JOINTS[j]} {label} {q[j]:+.2f} within {A.LIMIT_MARGIN} rad of its limit")
    return out


def path_problems(chain, q_a, q_b, what=""):
    """Joint-space straight line q_a -> q_b (the controller's path): hand + elbow stay out of the torso
    column and above MIN_Z. Returns (problems, lowest z)."""
    min_z = np.inf
    q_a, q_b = np.asarray(q_a, float), np.asarray(q_b, float)
    for s in np.linspace(0.0, 1.0, 41):
        fr = chain.frames(q_a + s * (q_b - q_a))
        for label, p in (("hand", fr[-1][:3, 3]), ("elbow", fr[3][:3, 3])):
            min_z = min(min_z, p[2])
            if in_box(p, BODY):
                return [f"{label} would pass through the torso column at {np.round(p, 2).tolist()}{what}"], min_z
            if p[2] < MIN_Z:
                return [f"{label} would go below z={MIN_Z} m ({p[2]:.2f}){what}"], min_z
    return [], min_z


ROLL_J = 4                 # wrist_roll index in the 7-joint arm
MAX_ROLL_DQ = 1.7          # rad: allowed wrist-roll change when --finger-roll is used


def plan_reach(q_arm0, target, side="right", standoff=0.10, aim=(0.0, 0.0, 0.0), roll=None):
    """Pure planning. q_arm0: 7 current arm angles. target: torso frame [m] (the real pod position;
    the picking box is checked on it). aim: offset added to what is COMMANDED, to cancel known
    errors (2026-09-27: stretched arm sags ~5 cm below the command at kp 80 -> aim z +0.05).
    Returns dict(ok, problems, hover, q_goal, dq, t_move, ik_err, path_min_z)."""
    chain = K.Chain(side)
    elbow_chain = K.Chain(side)  # elbow position = frame of joint index 3
    real_target = np.asarray(target, float)
    target = real_target + np.asarray(aim, float)
    hover = target - np.array([standoff, 0.0, 0.0])
    problems = []
    if not in_box(real_target, side_box(side)):
        problems.append(f"target {np.round(target, 3).tolist()} outside the picking box {side_box(side)}")
    fixed = ()
    q_start = np.asarray(q_arm0, float).copy()
    if roll is not None:   # turn the fingers: wrist roll set to start + roll and held there by IK
        q_start[ROLL_J] += roll
        fixed = (ROLL_J,)
    q_goal, ik_err = chain.ik(hover, q_start, fixed=fixed)
    if ik_err > MAX_IK_ERR:
        problems.append(f"IK can't reach the hover point (error {ik_err * 100:.1f} cm)")
    dq = q_goal - np.asarray(q_arm0, float)
    dq_check = dq.copy()
    if roll is not None:
        if abs(dq[ROLL_J]) > MAX_ROLL_DQ:
            problems.append(f"wrist roll change {dq[ROLL_J]:+.2f} rad > {MAX_ROLL_DQ}")
        dq_check[ROLL_J] = 0.0
    if np.abs(dq_check).max() > MAX_DQ:
        j = int(np.argmax(np.abs(dq_check)))
        problems.append(f"{K.ARM_JOINTS[j]} would move {dq[j]:+.2f} rad (> {MAX_DQ}) - start from a closer pose")
    problems += limit_problems(chain, q_goal)
    path, min_z = path_problems(elbow_chain, q_arm0, q_goal)
    problems += path
    t_move = max(MIN_T_MOVE, float(np.abs(dq).max()) / JOINT_SPEED)
    return dict(ok=not problems, problems=problems, hover=hover, q_goal=q_goal, dq=dq,
                t_move=t_move, ik_err=ik_err, path_min_z=min_z, fixed=fixed)


APPROACH_SPEED = 0.2     # rad/s: slower for the last 10 cm
MAX_APPROACH_DQ = 0.8    # rad per joint hover -> target (a short, near-straight approach)
T_GRIP = 8.0             # s: slow close until contact (0.3 rad/s x up to 2.2 rad) + squeeze
T_HOLD_LIFT, T_LET_GO = 2.0, 1.5


def plan_grasp(q_arm0, target, side="right", standoff=0.10, lift=0.03, aim=(0.0, 0.0, 0.0), roll=None, pull=False):
    """Hover (plan_reach), then approach so the grasp point reaches the target, close, lift, let go,
    retreat to hover, return. Returns plan_reach's dict + q_target, q_lift, segments_arm (7-vectors)."""
    p = plan_reach(q_arm0, target, side, standoff, aim, roll)
    chain = K.Chain(side)
    target = np.asarray(target, float) + np.asarray(aim, float)
    q_h = p["q_goal"]
    q_t, e_t = chain.ik(target, q_h, fixed=p["fixed"])
    q_l, e_l = chain.ik(target + np.array([0.0, 0.0, lift]), q_t, fixed=p["fixed"])
    problems = list(p["problems"])
    for label, e in (("target", e_t), ("lift point", e_l)):
        if e > MAX_IK_ERR:
            problems.append(f"IK can't reach the {label} (error {e * 100:.1f} cm)")
    if np.abs(q_t - q_h).max() > MAX_APPROACH_DQ:
        problems.append(f"approach needs a {np.abs(q_t - q_h).max():.2f} rad joint change (> {MAX_APPROACH_DQ}) "
                        "- not a short approach")
    problems += limit_problems(chain, q_t, "target") + limit_problems(chain, q_l, "lift")
    for qa, qb, what in ((q_h, q_t, " (approach)"), (q_t, q_l, " (lift)"), (q_l, q_h, " (retreat)")):
        problems += path_problems(chain, qa, qb, what)[0]
    t_app = max(2.5, float(np.abs(q_t - q_h).max()) / APPROACH_SPEED)
    segs = [("to_hover", "move", q_h, p["t_move"]), ("hold", "hold", None, 1.5),
            ("approach", "move", q_t, t_app), ("grip", "hold", None, T_GRIP),
            ("lift", "move", q_l, 2.0), ("hold_lift", "hold", None, T_HOLD_LIFT)]
    if pull:   # harvest motion: back off to the hover point still holding, then let go there
        segs += [("pull", "move", q_h, t_app), ("hold_pulled", "hold", None, 1.5),
                 ("let_go", "hold", None, T_LET_GO)]
    else:
        segs += [("let_go", "hold", None, T_LET_GO), ("retreat", "move", q_h, t_app)]
    segs += [("to_start", "move", np.asarray(q_arm0, float), p["t_move"])]
    p.update(ok=not problems, problems=problems, q_target=q_t, q_lift=q_l, t_approach=t_app, segments_arm=segs)
    return p


class GripperHook:
    """Drives one Dex1 inside the arm loop (Controller tick_hook). q = 0 is fully closed, bigger = open.
    Before 'grip': hold open at the start position. 'grip': close at close_rate until contact (the
    fingers lag the command by > contact_err) or max_travel, then hold contact - squeeze. 'lift',
    'hold_lift': keep. From 'let_go' on: open again. Raises (-> arm abort, gripper goes limp when
    publishing stops) on stale gripper state or |tau_est| > tau_abort."""

    def __init__(self, send, gstate, q_open, close_rate=0.3, max_travel=1.5, squeeze=0.15,
                 contact_err=0.15, tau_abort=3.0, min_q=0.3):
        self.send, self.g, self.q_open = send, gstate, q_open
        self.close_rate, self.max_travel, self.squeeze = close_rate, max_travel, squeeze
        self.contact_err, self.tau_abort, self.min_q = contact_err, tau_abort, min_q
        self.q_cmd, self.t_grip0, self.contact_q, self.lifted_q = q_open, None, None, None
        self.min_cmd = q_open  # deepest close commanded (the summary runs after it reopened)
        self.rows = []

    def __call__(self, t, phase, _ms):
        age = time.time() - self.g["t"]
        if age > 0.2:
            raise RuntimeError(f"gripper state stale ({age:.2f} s)")
        if abs(self.g["tau"]) > self.tau_abort:
            raise RuntimeError(f"gripper torque {self.g['tau']:+.2f} N*m > {self.tau_abort}")
        if phase == "grip":
            if self.t_grip0 is None:
                self.t_grip0 = t
            if self.contact_q is None:
                self.q_cmd = max(self.q_open - self.max_travel, self.min_q,
                                 self.q_open - self.close_rate * (t - self.t_grip0))
                if t - self.t_grip0 > 0.3 and self.g["q"] - self.q_cmd > self.contact_err:
                    self.contact_q = self.g["q"]
                    self.q_cmd = max(self.min_q, self.contact_q - self.squeeze)
        elif phase in ("lift", "hold_lift", "pull", "hold_pulled"):   # keep holding
            if phase in ("hold_lift", "hold_pulled"):
                self.lifted_q = self.g["q"]
        elif phase not in ("engage", "to_hover", "hold", "approach"):
            self.q_cmd = self.q_open          # let_go, retreat, to_start, release
        self.min_cmd = min(self.min_cmd, self.q_cmd)
        self.send(self.q_cmd)
        self.rows.append((round(t, 3), phase, round(self.q_cmd, 4), round(self.g["q"], 4), round(self.g["tau"], 4)))

    def summary(self):
        if self.t_grip0 is None:
            return "gripper: never reached the grip phase"
        if self.contact_q is None:
            return (f"gripper: NO CONTACT - closed {self.q_open - self.min_cmd:.2f} rad on nothing "
                    "(pod missed: check the hand-eye offset / grasp point)")
        held = self.lifted_q is not None and abs(self.lifted_q - self.contact_q) < 0.15
        return (f"gripper: contact at q {self.contact_q:.3f} ({self.q_open - self.contact_q:.2f} rad closed), squeezed to "
                f"{self.contact_q - self.squeeze:.3f}; after lift q {self.lifted_q if self.lifted_q is not None else float('nan'):.3f}"
                f" -> {'still holding' if held else 'probably slipped'}")


def detect_target(n_frames=6, tile=400, tile_imgsz=1280):
    """Best okra-shaped/sized pod with depth, consistent (<= 2 cm) over 2 frames -> camera xyz."""
    from okra_vision import CameraCalib, FrameSource, OkraDetector, locate, okra_sized
    det = OkraDetector(conf=0.3, tile=tile, tile_imgsz=tile_imgsz)
    src, cal, last = FrameSource("robot-depth", timeout_s=10), CameraCalib(), None
    for k, (name, img, depth) in enumerate(src):
        if depth is None:
            sys.exit("--detect needs depth: start run_depth_server.sh (source robot-depth)")
        cal.set_intrinsics(src.intrinsics)
        pods = []
        for o in det.detect(img):
            locate(o, depth, cal)
            o.plausible = okra_sized(o)
            if o.plausible and o.depth_method.startswith("depth"):
                pods.append(o)
        best = pods[0] if pods else None
        print(f"  {name}: " + (f"pod conf {best.conf:.2f} xyz {np.round(best.xyz, 3).tolist()} "
                              f"size {best.length_m * 100:.0f}x{best.width_m * 100:.1f} cm" if best else "no pod"))
        if best is not None and last is not None and np.linalg.norm(np.subtract(best.xyz, last.xyz)) <= 0.02:
            return np.array(best.xyz), (img, best, src.intrinsics)
        last = best
        if k + 1 >= n_frames:
            break
    sys.exit("No pod detected consistently - nothing moved.")


def snapshot(state, side, target_torso, hover, intr_hint=None):
    """Hand-eye check: one camera frame + where the kinematics says fingertip/grasp/target are."""
    import cv2
    from okra_vision import FrameSource
    try:
        src = FrameSource("robot-depth", timeout_s=3)
        name, img, depth = next(iter(src))
        intr = src.intrinsics or intr_hint
        ms = state["msg"].motor_state
        first = K.FIRST_MOTOR[side]
        q = [float(ms[first + j].q) for j in range(7)]
        pts = {"tip (FK)": K.Chain(side, K.TOOL_TIP).fk(q), "grasp (FK)": K.Chain(side, K.TOOL_GRASP).fk(q),
               "hover goal": hover, "target": target_torso}
        colours = {"tip (FK)": (0, 0, 255), "grasp (FK)": (0, 165, 255), "hover goal": (255, 255, 0), "target": (0, 255, 0)}
        lines = []
        for label, p in pts.items():
            c = K.torso_to_camera(p)
            if c[2] <= 0.05:
                lines.append(f"{label}: behind camera")
                continue
            u = intr["fx"] * c[0] / c[2] + intr["cx"]
            v = intr["fy"] * c[1] / c[2] + intr["cy"]
            inside = 0 <= u < img.shape[1] and 0 <= v < img.shape[0]
            meas = ""
            if inside and depth is not None:
                zz = depth[int(v), int(u)]
                meas = f", camera depth at that pixel {zz:.3f} m (predicted {c[2]:.3f})"
            lines.append(f"{label}: pixel ({u:.0f},{v:.0f}){'' if inside else ' OUT OF VIEW'}{meas}")
            if inside:
                cv2.drawMarker(img, (int(u), int(v)), colours[label], cv2.MARKER_CROSS, 24, 2)
                cv2.putText(img, label, (int(u) + 8, int(v) - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, colours[label], 1)
        os.makedirs(os.path.join(HERE, "out"), exist_ok=True)
        path = os.path.join(HERE, "out", time.strftime("reach_snapshot_%Y%m%d_%H%M%S.jpg"))
        cv2.imwrite(path, img)
        print("SNAPSHOT " + path + "\n  " + "\n  ".join(lines))
    except Exception as e:  # noqa: BLE001 - a failed snapshot must never disturb the arm loop
        print(f"SNAPSHOT failed: {e!r} (is run_depth_server.sh running?)")


def parse_xyz(s):
    v = [float(x) for x in s.split(",")]
    if len(v) != 3:
        raise argparse.ArgumentTypeError("need X,Y,Z")
    return np.array(v)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--target-torso", type=parse_xyz)
    g.add_argument("--target-cam", type=parse_xyz)
    g.add_argument("--detect", action="store_true")
    ap.add_argument("--side", choices=("right", "left"), default="right")
    ap.add_argument("--standoff", type=float, default=0.10, help="hover this far short of the target [m], >= 0.05")
    ap.add_argument("--hold", type=float, default=2.0, help="hold time at the hover point [s]")
    ap.add_argument("--snapshot", action="store_true", help="camera frame + FK overlay during the hold")
    ap.add_argument("--grasp", action="store_true",
                    help="after the hover: approach to the target, close the gripper on contact, lift, let go, back")
    ap.add_argument("--lift", type=float, default=0.03, help="--grasp: lift after closing [m], <= 0.05")
    ap.add_argument("--grip-kp", type=float, default=8.0, help="--grasp: gripper stiffness (Toyota team: 8 open, 20 cut)")
    ap.add_argument("--pull", action="store_true", help="--grasp: pull back 'standoff' while holding, then let go")
    ap.add_argument("--grip-travel", type=float, default=1.5,
                    help="--grasp: max closing travel [rad] (2026-09-27: 1.5 ran out before touching a 1.6 cm pod), <= 2.2")
    ap.add_argument("--tool-x", type=float, default=None,
                    help=f"grasp point along the fingers from the wrist [m] (default {K.TOOL_GRASP[0]}; calibrate with --snapshot)")
    ap.add_argument("--aim", type=parse_xyz, default=np.zeros(3),
                    help="DX,DY,DZ added to the commanded point [m], torso frame (e.g. 0,0,0.05 against arm sag); |each| <= 0.08")
    ap.add_argument("--finger-roll", type=float, default=None,
                    help="turn the wrist by this many degrees (e.g. 90: fingers close up-down instead of left-right)")
    ap.add_argument("--live", action="store_true", help="actually publish to rt/arm_sdk (MOVES THE ROBOT)")
    ap.add_argument("--iface", default=None)
    a = ap.parse_args()
    if a.standoff < 0.05:
        sys.exit("--standoff must be >= 0.05 m (the hover point must not touch the target)")
    if np.abs(a.aim).max() > 0.08:
        sys.exit("--aim components must be within +-0.08 m")
    if not 0.0 <= a.lift <= 0.05:
        sys.exit("--lift must be in [0, 0.05] m")
    if not 0.5 <= a.grip_travel <= 2.2:
        sys.exit("--grip-travel must be in [0.5, 2.2] rad")
    if not 0 < a.grip_kp <= 20:
        sys.exit("--grip-kp must be in (0, 20]")
    if a.tool_x is not None:
        if not 0.10 <= a.tool_x <= 0.22:
            sys.exit("--tool-x must be in [0.10, 0.22] m")
        K.TOOL_GRASP = (a.tool_x, 0.0, 0.0)

    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
    from unitree_sdk2py.utils.crc import CRC

    iface = a.iface or A.find_iface()
    if not iface:
        sys.exit("No 192.168.123.x interface (see README step 1).")
    ChannelFactoryInitialize(0, iface)
    st = {"msg": None, "t": 0.0}

    def on_state(m):
        st["msg"], st["t"] = m, time.time()
    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init(on_state, 10)
    t0 = time.time()
    while st["msg"] is None and time.time() - t0 < 3.0:
        time.sleep(0.05)
    if st["msg"] is None:
        sys.exit("No rt/lowstate within 3 s - check the link (python probe_robot.py).")

    if a.detect:
        print("detecting (tiled, ~4 s/frame on CPU)...")
        cam_xyz, _ = detect_target()
        target = K.camera_to_torso(cam_xyz)
    elif a.target_cam is not None:
        target = K.camera_to_torso(a.target_cam)
    else:
        target = a.target_torso

    first = K.FIRST_MOTOR[a.side]
    ms = st["msg"].motor_state
    q_arm0 = [float(ms[first + j].q) for j in range(7)]
    grasp_now = K.Chain(a.side).fk(q_arm0)
    roll = None if a.finger_roll is None else float(np.radians(a.finger_roll))
    p = (plan_grasp(q_arm0, target, a.side, a.standoff, a.lift, a.aim, roll, a.pull) if a.grasp
         else plan_reach(q_arm0, target, a.side, a.standoff, a.aim, roll))
    if np.abs(a.aim).max() > 0:
        print(f"aim offset {np.round(a.aim, 3).tolist()} m -> commanded point {np.round(np.asarray(target) + a.aim, 3).tolist()}")
    print(f"target (torso) {np.round(target, 3).tolist()} m; hover {np.round(p['hover'], 3).tolist()} "
          f"({a.standoff * 100:.0f} cm short); grasp point now {np.round(grasp_now, 3).tolist()} "
          f"(tool {K.TOOL_GRASP[0]:.3f} m)")
    print(f"IK error {p['ik_err'] * 1000:.1f} mm; move {p['t_move']:.1f} s; lowest point on path z={p['path_min_z']:.2f} m")
    for j, name in enumerate(K.ARM_JOINTS):
        extra = (f"   target {p['q_target'][j]:+.3f}  lift {p['q_lift'][j]:+.3f}" if a.grasp else "")
        print(f"  {name:15s} {q_arm0[j]:+.3f} -> hover {p['q_goal'][j]:+.3f}  ({p['dq'][j]:+.3f} rad){extra}")
    if a.grasp:
        total = A.T_ENGAGE + sum(seg[3] for seg in p["segments_arm"]) + A.T_RELEASE
        print("grasp sequence: " + " -> ".join(f"{n} {d:.1f}s" for n, _, _, d in p["segments_arm"]) + f"  (~{total:.0f} s)")

    # Robot-side checks, as in arm_test.py
    import json
    fsm_id = None
    try:
        from unitree_sdk2py.g1.loco.g1_loco_api import ROBOT_API_ID_LOCO_GET_FSM_ID
        from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
        lc = LocoClient()
        lc.SetTimeout(3.0)
        lc.Init()
        code, data = lc._Call(ROBOT_API_ID_LOCO_GET_FSM_ID, json.dumps({}))
        fsm_id = int(json.loads(data).get("data")) if code == 0 else None
    except Exception:  # noqa: BLE001
        pass
    readers = A.remote_readers("rt/arm_sdk")
    mm = int(st["msg"].mode_machine)
    hot = max(int(ms[i].temperature[0]) for i in A.HELD)
    print(f"locomotion FSM {fsm_id} ({A.WALK_FSM.get(fsm_id, 'not walk-capable')}); rt/arm_sdk subscribers "
          f"{readers or 'NONE'}; mode_machine {mm}; hottest held motor {hot} C")
    problems = list(p["problems"])
    if fsm_id not in A.WALK_FSM:
        problems.append("robot not in motion-control mode - remote: L2+Up, then R1+Y")
    if not readers:
        problems.append("nothing subscribes to rt/arm_sdk")
    if hot >= A.MAX_TEMP_C:
        problems.append(f"motor temperature {hot} C")
    gstate = {"q": None, "tau": 0.0, "t": 0.0}
    if a.grasp:
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorStates_

        def on_grip(m):
            gstate.update(q=float(m.states[0].q), tau=float(m.states[0].tau_est), t=time.time())
        gsub = ChannelSubscriber(f"rt/dex1/{a.side}/state", MotorStates_)
        gsub.Init(on_grip, 10)
        t0 = time.time()
        while gstate["q"] is None and time.time() - t0 < 2.0:
            time.sleep(0.05)
        others = A.remote_endpoints(f"rt/dex1/{a.side}/cmd", "writer", wait_s=1.5)
        if gstate["q"] is None:
            problems.append(f"no rt/dex1/{a.side}/state (gripper)")
        else:
            print(f"gripper {a.side}: q {gstate['q']:.3f} (open start); will close <= {a.grip_travel} rad at 0.3 rad/s until "
                  f"contact (never below q 0.3), kp {a.grip_kp}")
            if gstate["q"] - min(a.grip_travel, 1.5) < 0.3:
                problems.append(f"gripper q {gstate['q']:.3f} too close to closed - open it first "
                                "(python gripper_test.py --side ... --delta 0.3)")
        if others:
            problems.append(f"someone else commands the {a.side} gripper: {others}")
    for pr in problems:
        print("  PROBLEM:", pr)

    if not a.live:
        print("\nDRY RUN - nothing was published. Add --live to move the robot.")
        return
    if problems:
        sys.exit("Not moving: fix the problems above first.")

    if a.grasp:
        print(f"\n*** THIS WILL MOVE THE {a.side.upper()} ARM AND GRIPPER *** hover, reach the target, close on it, "
              f"lift {a.lift * 100:.0f} cm, let go, back")
    else:
        print(f"\n*** THIS WILL MOVE THE {a.side.upper()} ARM *** to {a.standoff * 100:.0f} cm short of the target "
              f"and back, ~{2 * p['t_move'] + a.hold + A.T_ENGAGE + A.T_RELEASE:.0f} s")
    print("Confirm: area clear (nobody within arm reach); remote in hand, emergency stop L2+B;")
    print("robot in motion-control mode (L2+Up, R1+Y); no other command source running.")
    if input("Type 'move' to go, anything else aborts: ").strip() != "move":
        sys.exit("Aborted by user - nothing was published.")

    crc, cmd = CRC(), unitree_hg_msg_dds__LowCmd_()
    cmd.mode_pr = 0
    cmd.mode_machine = mm

    def gains(i):
        if i in A.WAIST:
            return A.WAIST_KP, A.WAIST_KD
        if i in A.WRIST:
            return A.WRIST_KP, A.WRIST_KD
        return A.ARM_KP_MAX, 3.0   # kp 80 / kd 3: best tracking on this robot (arm_test gain sweep)

    def build(weight, q):
        cmd.motor_cmd[A.ARM_SDK_WEIGHT].q = weight
        for i in A.HELD:
            mc = cmd.motor_cmd[i]
            mc.mode = 1
            mc.q, mc.dq, mc.tau = q[i], 0.0, 0.0
            mc.kp, mc.kd = gains(i)
        cmd.crc = crc.Crc(cmd)
        return cmd

    pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
    pub.Init()
    ms = st["msg"].motor_state
    q0 = {i: float(ms[i].q) for i in A.HELD}
    q_arm_now = [q0[first + j] for j in range(7)]
    if np.abs(np.subtract(q_arm_now, q_arm0)).max() > 0.05 or int(st["msg"].mode_machine) != mm:
        sys.exit("Arm moved or mode changed since planning - run again. Nothing was published.")
    goal = {first + j: float(p["dq"][j]) for j in range(7)}

    def send(w, q):
        if not pub.Write(build(w, q)):
            raise RuntimeError("rt/arm_sdk write failed")
    on_hold = (lambda: snapshot(st, a.side, target, p["hover"])) if a.snapshot else None
    hook = None
    if a.grasp:
        from unitree_sdk2py.idl.default import unitree_go_msg_dds__MotorCmd_
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_
        gpub = ChannelPublisher(f"rt/dex1/{a.side}/cmd", MotorCmds_)
        gpub.Init()
        gcmd = MotorCmds_()
        gcmd.cmds = [unitree_go_msg_dds__MotorCmd_()]
        gc = gcmd.cmds[0]
        gc.dq, gc.tau, gc.kp, gc.kd = 0.0, 0.0, a.grip_kp, 0.05

        def gsend(q):
            gc.q = float(q)
            if not gpub.Write(gcmd):
                raise RuntimeError("gripper write failed")
        hook = GripperHook(gsend, gstate, gstate["q"], max_travel=a.grip_travel)
        segs = [(n, k, None if qv is None else {first + j: float(qv[j]) for j in range(7)}, d)
                for n, k, qv, d in p["segments_arm"]]
        ctl = A.Controller(q0, goal, st, send, mm, on_hold=on_hold, planner=A.Waypoints(q0, segs),
                           tick_hook=hook, hold_phase="hold")
    else:
        ctl = A.Controller(q0, goal, st, send, mm, t_move=p["t_move"], t_hold=a.hold, on_hold=on_hold)

    def on_signal(_signum, _frame):
        raise KeyboardInterrupt
    for sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, on_signal)
    os.makedirs(os.path.join(HERE, "logs"), exist_ok=True)
    log_path = os.path.join(HERE, "logs", time.strftime("reach_test_%Y%m%d_%H%M%S.csv"))
    with open(log_path, "w", newline="") as fh:
        print("moving... (Ctrl-C = abort and release)")
        reason = ctl.run(csv.writer(fh))
    print(("aborted: " + reason) if reason else "done", "- arm_sdk released. log:", log_path)
    A.print_joint_stats(ctl.stats, ctl.joint)
    if hook is not None:
        try:
            gpub.Close()   # stop publishing: the gripper goes limp (as before the run)
        except Exception as e:  # noqa: BLE001 - never lose the summary/log over this
            print(f"(gripper publisher close failed: {e!r})")
        print(hook.summary())
        with open(log_path.replace(".csv", "_gripper.csv"), "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["t", "phase", "q_cmd", "q_meas", "tau_est"])
            w.writerows(hook.rows)


if __name__ == "__main__":
    main()

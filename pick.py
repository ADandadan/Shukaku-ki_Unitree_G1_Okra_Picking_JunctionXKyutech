#!/usr/bin/env python3
"""
pick.py - okra picking loop. *** MOVES THE ROBOT only with --live, and only after you type 'move' ***

Each round:
  1. MEASURE fresh (never reuse a target - 2026-09-27: both failed picks reused stale coordinates):
     tiled detection + depth over several frames; keep pods inside an arm's picking box, seen in >= 2 frames
     within 2 cm. Pods seen end-on (pointing at the camera) look short, so inside the reachable region the
     shape rule is relaxed.
  2. POD AXIS in 3D: principal direction of the mask's depth points (camera -> torso frame).
  3. ARM + FINGER ROLL: for each arm and wrist-roll candidate, plan the grasp (reach_test.plan_grasp, all
     its safety checks) and pick the plan whose fingers close ACROSS the pod (closing axis perpendicular to
     the pod), preferring up-down closing (our height error is the largest - that is what made the grab work).
  4. RUN reach_test.py with the fresh target, as a separate process (the robot-tested code, unchanged):
     --grasp --pull --tool-x 0.19 --grip-travel 2.0 --finger-roll <chosen>. Without --live it is a dry run;
     with --live, reach_test.py itself asks you to type 'move'.
  5. Next round (re-measure) or stop.

  python pick.py                 # measure + plan + dry run, one round
  python pick.py --live          # pick; after each attempt asks whether to continue
Needs: run_depth_server.sh running; robot in motion-control mode (L2+Up, R1+Y); see preflight.py.
"""
import argparse
import os
import subprocess
import sys
import time
from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np

import g1_kinematics as K
import reach_test as R

HERE = os.path.dirname(os.path.abspath(__file__))
TOOL_X = 0.19            # grasp point along the fingers, from today's hand-eye check
GRIP_TRAVEL = 2.0        # rad; 1.5 ran out before touching a 1.6 cm pod
ROLLS_DEG = list(range(-150, 151, 15))
MIN_HITS = 2
CLUSTER_M = 0.02
END_ON_MIN_ELONGATION = 1.3   # inside the picking box only: pods pointing at the camera look short


@dataclass
class Pod:
    cam: np.ndarray                      # median position, camera optical frame [m]
    torso: np.ndarray
    conf: float
    hits: int
    length_m: float
    axis: Optional[np.ndarray] = None    # unit vector, torso frame (sign arbitrary)
    samples: List[tuple] = field(default_factory=list)


def pod_axis_cam(o, depth, cal, min_points=30):
    """Principal direction of the pod's 3D points (camera frame), or None."""
    from okra_vision import okra_mask
    m = okra_mask(o, depth.shape, erode_px=2)
    vs, us = np.nonzero(m)
    z = depth[vs, us]
    ok = (z > 0.1) & (z < 2.0)
    if ok.sum() < min_points:
        return None
    vs, us, z = vs[ok], us[ok], z[ok]
    keep = np.abs(z - np.median(z)) < 0.03          # drop background leaking in at the edges
    if keep.sum() < min_points:
        return None
    vs, us, z = vs[keep], us[keep], z[keep]
    fx = cal.f()
    fy = cal.fy or fx
    cx, cy = cal.pp()
    pts = np.stack([(us - cx) * z / fx, (vs - cy) * z / fy, z], axis=1)
    pts -= pts.mean(axis=0)
    _, s, vt = np.linalg.svd(pts, full_matrices=False)
    if s[0] < 1.5 * s[1]:                             # no clear long direction
        return None
    return vt[0]


def cam_axis_to_torso(p_cam, a_cam):
    a = K.camera_to_torso(np.asarray(p_cam) + 0.05 * np.asarray(a_cam)) - K.camera_to_torso(p_cam)
    return a / np.linalg.norm(a)


def in_any_box(t):
    return R.in_box(t, R.side_box("left")) or R.in_box(t, R.side_box("right"))


# Where each arm grasps best (torso frame): ~42 cm ahead, 20 cm to its side (today's successful pick: 0.43, 0.22)
SWEET = {"left": np.array([0.42, 0.20]), "right": np.array([0.42, -0.20])}
MAX_TURN_DEG, MAX_WALK_M = 30.0, 0.40


def suggest_walk(torso):
    """Turn + forward step that brings a pod at `torso` (x fwd, y left) to the nearer arm's sweet spot.
    Returns (side, turn_deg (+ = left), walk_m, note). Rough: in the gantry the robot under-delivers and
    drifts, so re-measure after every move."""
    x, y, z = torso
    side = "left" if y >= 0 else "right"
    sx, sy = SWEET[side]
    turn = np.degrees(np.arctan2(y, x) - np.arctan2(sy, sx))
    walk = np.hypot(x, y) - np.hypot(sx, sy)
    note = ""
    if not (R.BOX["z"][0] <= z <= R.BOX["z"][1]):
        note = f"pod height z={z:+.2f} m is outside {R.BOX['z']} - walking can't fix that; move the okra"
    return side, float(np.clip(turn, -MAX_TURN_DEG, MAX_TURN_DEG)), float(np.clip(walk, -0.2, MAX_WALK_M)), note


def measure(n_frames=5, tile=400, tile_imgsz=1280, conf=0.4, verbose=True, anywhere=False):
    """Fresh pods within reach (anywhere=True: within 2 m), consistent over frames. Returns list of Pod, best first."""
    from okra_vision import CameraCalib, FrameSource, OkraDetector, locate, okra_sized, pod_width_px
    det = OkraDetector(conf=conf, tile=tile, tile_imgsz=tile_imgsz)
    src, cal = FrameSource("robot-depth", timeout_s=10), CameraCalib()
    raw, last_img = [], None
    for k, (name, img, depth) in enumerate(src):
        last_img = img
        if depth is None:
            sys.exit("Needs depth: start run_depth_server.sh (source robot-depth).")
        cal.set_intrinsics(src.intrinsics)
        for o in det.detect(img):
            locate(o, depth, cal)
            if not (o.xyz and o.depth_method.startswith("depth")):
                continue
            t = K.camera_to_torso(o.xyz)
            if not (in_any_box(t) or (anywhere and o.xyz[2] < 2.0)):
                continue
            o.plausible = okra_sized(o)
            end_on = o.length_px >= END_ON_MIN_ELONGATION * max(1.0, pod_width_px(o)) and o.width_m <= 0.05 \
                and o.length_m <= 0.25
            if not (o.plausible or end_on):
                continue
            ax = pod_axis_cam(o, depth, cal)
            raw.append((o.conf, np.array(o.xyz), t, o.length_m, ax, o.center))
        if k + 1 >= n_frames:
            break
    pods: List[Pod] = []
    for c, x, t, L, ax, uv in sorted(raw, key=lambda r: -r[0]):
        for p in pods:
            if np.linalg.norm(p.cam - x) < CLUSTER_M:
                p.samples.append((c, x, t, L, ax, uv))
                break
        else:
            pods.append(Pod(x, t, c, 1, L, None, [(c, x, t, L, ax, uv)]))
    out = []
    for p in pods:
        if len(p.samples) < MIN_HITS:
            continue
        p.hits = len(p.samples)
        p.cam = np.median([s[1] for s in p.samples], axis=0)
        p.torso = K.camera_to_torso(p.cam)
        p.conf = max(s[0] for s in p.samples)
        p.length_m = float(np.median([s[3] for s in p.samples]))
        axes = [s[4] for s in p.samples if s[4] is not None]
        if axes:
            ref = axes[0]
            a = np.median([v if np.dot(v, ref) >= 0 else -v for v in axes], axis=0)
            p.axis = cam_axis_to_torso(p.cam, a / np.linalg.norm(a))
        out.append(p)
    out.sort(key=lambda p: -(p.conf * p.hits))
    if verbose:
        for i, p in enumerate(out):
            ax = "unknown" if p.axis is None else np.round(p.axis, 2).tolist()
            print(f"  [{i}] pod: conf {p.conf:.2f}, {p.hits}/{n_frames} frames, ~{p.length_m * 100:.0f} cm, torso "
                  f"{np.round(p.torso, 3).tolist()}, axis {ax}")
    if out and last_img is not None:
        save_candidates(last_img, out)
    return out


def save_candidates(img, pods, path=os.path.join(HERE, "out", "pick_candidates.jpg")):
    """Numbered candidates on the last frame, so a person can pick the real pod (green stakes also get
    detected as okra - 2026-09-27)."""
    import cv2
    vis = img.copy()
    for i, p in enumerate(pods):
        u, v = np.median([s[5] for s in p.samples], axis=0)
        cv2.circle(vis, (int(u), int(v)), 28, (0, 0, 255), 3)
        cv2.putText(vis, f"[{i}] {p.conf:.2f}", (int(u) + 32, int(v) + 8), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), 5)
        cv2.putText(vis, f"[{i}] {p.conf:.2f}", (int(u) + 32, int(v) + 8), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    cv2.imwrite(path, vis)
    print(f"  candidates image: {path}")


def closing_axis(side, q):
    """Unit vector (torso frame) along which the Dex1 fingers close (tool-frame y, checked against the
    robot on 2026-09-27: roll 0 -> left-right, roll -90 -> up-down)."""
    return K.Chain(side, (TOOL_X, 0.0, 0.0)).frames(q)[-1][:3, 1]


def choose_grasp(pod, q_arms, aim=(0.0, 0.0, 0.0), verbose=True):
    """Try both arms x wrist-roll candidates; return (score, side, roll_deg, plan) of the best safe plan."""
    best = None
    K.TOOL_GRASP = (TOOL_X, 0.0, 0.0)
    for side in ("left", "right"):
        if not R.in_box(pod.torso, R.side_box(side)):
            continue
        for deg in ROLLS_DEG:
            p = R.plan_grasp(q_arms[side], pod.torso, side, aim=aim, roll=np.radians(deg), pull=True)
            if not p["ok"]:
                continue
            c = closing_axis(side, p["q_target"])
            across = abs(float(np.dot(c, pod.axis))) if pod.axis is not None else 0.0   # 0 = perfectly across
            not_vertical = 1.0 - abs(float(c[2]))                                        # 0 = up-down closing
            effort = float(np.abs(p["dq"]).max()) / 3.0                                  # prefer smaller moves
            score = 1.0 * across + 0.3 * not_vertical + 0.1 * effort
            if best is None or score < best[0]:
                best = (score, side, deg, p, c)
    if verbose and best:
        s, side, deg, p, c = best
        print(f"  choice: {side} arm, finger roll {deg:+d} deg, fingers close along {np.round(c, 2).tolist()}"
              + (f" (|cos| to pod axis {abs(np.dot(c, pod.axis)):.2f})" if pod.axis is not None else " (pod axis unknown)"))
    return best


def arm_reader(timeout=3.0):
    """Initialise DDS once; returns a function giving the current {side: 7 arm angles}."""
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
    import arm_test as A
    iface = A.find_iface()
    if not iface:
        sys.exit("No 192.168.123.x interface.")
    ChannelFactoryInitialize(0, iface)
    st = {}
    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init(lambda m: st.update(m=m), 10)
    t0 = time.time()
    while "m" not in st and time.time() - t0 < timeout:
        time.sleep(0.05)
    if "m" not in st:
        sys.exit("No rt/lowstate.")
    def arms():
        ms = st["m"].motor_state
        return {side: [float(ms[K.FIRST_MOTOR[side] + j].q) for j in range(7)] for side in ("left", "right")}
    arms.sub = sub   # keep the subscriber alive
    return arms


def reach_cmd(pod, side, roll_deg, live, aim):
    x, y, z = pod.cam
    cmd = [sys.executable, os.path.join(HERE, "reach_test.py"), f"--target-cam={x:.3f},{y:.3f},{z:.3f}",
           "--side", side, "--grasp", "--pull", "--tool-x", f"{TOOL_X}", "--finger-roll", f"{roll_deg}",
           "--grip-travel", f"{GRIP_TRAVEL}"]
    if any(aim):
        cmd += [f"--aim={aim[0]},{aim[1]},{aim[2]}"]
    if live:
        cmd.append("--live")
    return cmd


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--live", action="store_true", help="pass --live to reach_test.py (it asks 'move' each time)")
    ap.add_argument("--frames", type=int, default=5)
    ap.add_argument("--aim", type=R.parse_xyz, default=np.zeros(3), help="DX,DY,DZ correction [m] (default 0)")
    ap.add_argument("--auto", action="store_true", help="don't ask which candidate; take the best-scoring one")
    a = ap.parse_args()
    arms = arm_reader()
    rnd = 0
    while True:
        rnd += 1
        print(f"\n=== round {rnd}: measuring ({a.frames} frames, ~{1.4 * a.frames:.0f} s on GPU) ===")
        pods = measure(a.frames)
        if not pods:
            print("No pod within reach. Looking further away to suggest a walk...")
            far = measure(a.frames, anywhere=True, verbose=True)
            far = [p for p in far if p.length_m and 0.03 <= p.length_m <= 0.35]
            if not far:
                print("No okra seen within 2 m either. Move the okra into view (in front, 0.5-1.5 m), then run again.")
                return
            if len(far) > 1 and not a.auto:
                ans = input(f"Which candidate is the okra? (see out/pick_candidates.jpg) [0-{len(far) - 1}, Enter = 0]: ").strip()
                if ans.isdigit() and int(ans) < len(far):
                    far = [far[int(ans)]]
            side, turn, walk, note = suggest_walk(far[0].torso)
            print(f"\nSUGGESTED MOVE to bring it to the {side} arm (rough - re-run pick.py after each move):")
            if note:
                print("  " + note)
            if abs(turn) >= 5:
                print(f"  python walk_test.py --live --pattern turn-{'left' if turn > 0 else 'right'} --angle {abs(turn):.0f}")
            if walk >= 0.05:
                print(f"  python walk_test.py --live --pattern forward --distance {walk:.2f}")
                print("  (check the feet will still clear the rig/box before walking forward)")
            elif walk <= -0.05:
                print(f"  too close by {-walk:.2f} m: python walk_test.py --live --pattern fwd-back is not one-way - "
                      "move the okra back instead")
            if abs(turn) < 5 and abs(walk) < 0.05 and not note:
                print("  already about right - the pod may be just outside the picking box; move it a few cm toward the arm")
            return
        q_arms = arms()   # current pose: the previous pick may have left the arm elsewhere
        if len(pods) > 1 and not a.auto:
            ans = input(f"Which candidate is the okra? (see out/pick_candidates.jpg) [0-{len(pods) - 1}, "
                        "Enter = 0, s = skip round]: ").strip().lower()
            if ans == "s":
                return
            if ans:
                if not ans.isdigit() or int(ans) >= len(pods):
                    sys.exit("Not a candidate number - nothing moved.")
                pods = [pods[int(ans)]]
        choice = None
        for pod in pods:
            choice = choose_grasp(pod, q_arms, a.aim)
            if choice:
                break
            print("  no safe grasp plan for this pod - trying the next one")
        if not choice:
            print("No safe grasp plan for any pod in reach. Walk/turn closer and run again.")
            return
        _, side, deg, _, _ = choice
        cmd = reach_cmd(pod, side, deg, a.live, a.aim)
        print("  running: " + " ".join(cmd[1:]))
        rc = subprocess.run(cmd).returncode
        if not a.live:
            print("\nDry run done (nothing moved). Add --live to pick.")
            return
        if rc != 0:
            print(f"reach_test.py exited with {rc} - stopping.")
            return
        if input("\nAnother round (re-measure and pick)? [y/N] ").strip().lower() != "y":
            return


if __name__ == "__main__":
    main()

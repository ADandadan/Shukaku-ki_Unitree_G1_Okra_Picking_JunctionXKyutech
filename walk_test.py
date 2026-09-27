#!/usr/bin/env python3
"""
walk_test.py - *** MAKES THE ROBOT WALK (with --live) ***  First walking test: small, slow, short.

Without --live this is a DRY RUN: it reads the locomotion state (GetFsmId), IMU and robot state,
prints the plan. Nothing is sent.

With --live: Unitree's own walking controller (LocoClient.SetVelocity, service "sport") does the
stepping; we only ask for a velocity for a few seconds. Default pattern "fwd-back":
  forward 0.15 m/s for 2 s (~30 cm) -> stop, settle 2 s -> backward 0.15 m/s for 2 s -> stop
Patterns: fwd-back | turn (0.3 rad/s left 2 s, then right 2 s) | side (0.1 m/s left 2 s, then right)
Caps: |vx| <= 0.3 m/s, |vy| <= 0.2 m/s, |omega| <= 0.4 rad/s, each step <= 3 s.
StopMove() is sent after every step, on Ctrl-C / SIGTERM / SIGHUP, on any error, and at the end.
Aborts (StopMove) if |roll| or |pitch| > 0.35 rad, robot state goes stale, the FSM leaves a
walk-capable state, or a velocity command is rejected.

Robot: standing in motion-control mode (remote L2+Up, then R1+Y), gantry slack enough to let it
step but ready to catch it, 1.5 m clear around it. Emergency stop: L2+B (it goes limp).
  python walk_test.py                  # dry run
  python walk_test.py --live           # fwd-back, 0.15 m/s
  python walk_test.py --live --pattern turn
"""
import argparse
import csv
import math
import json
import os
import signal
import sys
import time

import arm_test as A

HERE = os.path.dirname(os.path.abspath(__file__))
MAX_VX, MAX_VY, MAX_W, MAX_STEP_S = 0.3, 0.2, 0.4, 3.0
MAX_TILT = 0.35          # rad roll/pitch
SETTLE_S = 2.0


def pattern_steps(name, speed, secs):
    if name == "forward":   # one step towards a target (e.g. bring a pod into arm reach)
        return [("forward", (speed, 0.0, 0.0), secs)]
    if name == "turn-left":  # one-way turn (e.g. face a pod again)
        return [("turn left", (0.0, 0.0, speed), secs)]
    if name == "turn-right":
        return [("turn right", (0.0, 0.0, -speed), secs)]
    if name == "fwd-back":
        return [("forward", (speed, 0.0, 0.0), secs), ("backward", (-speed, 0.0, 0.0), secs)]
    if name == "turn":
        return [("turn left", (0.0, 0.0, speed), secs), ("turn right", (0.0, 0.0, -speed), secs)]
    if name == "side":
        return [("step left", (0.0, speed, 0.0), secs), ("step right", (0.0, -speed, 0.0), secs)]
    raise ValueError(name)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--live", action="store_true", help="actually walk")
    ap.add_argument("--pattern", choices=("fwd-back", "forward", "turn", "turn-left", "turn-right", "side"),
                    default="fwd-back")
    ap.add_argument("--angle", type=float, default=None,
                    help="turn-left/turn-right: degrees to turn (sets --secs = angle/speed, max 3 s)")
    ap.add_argument("--distance", type=float, default=None,
                    help="forward pattern: metres to walk (sets --secs = distance/speed, max 3 s)")
    ap.add_argument("--speed", type=float, default=None, help="m/s (fwd-back, side) or rad/s (turn)")
    ap.add_argument("--secs", type=float, default=2.0, help=f"per step, <= {MAX_STEP_S}")
    ap.add_argument("--iface", default=None)
    a = ap.parse_args()
    speed = a.speed if a.speed is not None else {"fwd-back": 0.15, "forward": 0.15, "turn": 0.3, "turn-left": 0.3,
                                                 "turn-right": 0.3, "side": 0.1}[a.pattern]
    cap = {"fwd-back": MAX_VX, "forward": MAX_VX, "turn": MAX_W, "turn-left": MAX_W, "turn-right": MAX_W,
           "side": MAX_VY}[a.pattern]
    if a.angle is not None:
        if a.pattern not in ("turn-left", "turn-right") or not 0 < a.angle <= 45:
            sys.exit("--angle needs --pattern turn-left/turn-right and 0 < angle <= 45 deg")
        a.secs = math.radians(a.angle) / speed
    if a.distance is not None:
        if a.pattern != "forward" or not 0 < a.distance <= 0.5:
            sys.exit("--distance needs --pattern forward and 0 < distance <= 0.5 m")
        a.secs = a.distance / speed
    if not 0 < speed <= cap:
        sys.exit(f"--speed must be in (0, {cap}] for {a.pattern}")
    if not 0 < a.secs <= MAX_STEP_S:
        sys.exit(f"--secs must be in (0, {MAX_STEP_S}]")
    steps = pattern_steps(a.pattern, speed, a.secs)

    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.g1.loco.g1_loco_api import ROBOT_API_ID_LOCO_GET_FSM_ID
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

    iface = a.iface or A.find_iface()
    if not iface:
        sys.exit("No 192.168.123.x interface (see README step 1).")
    ChannelFactoryInitialize(0, iface)
    st = {"msg": None, "t": 0.0}

    def on_state(m):
        st["msg"], st["t"] = m, time.time()
    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init(on_state, 10)
    lc = LocoClient()
    lc.SetTimeout(3.0)
    lc.Init()
    t0 = time.time()
    while st["msg"] is None and time.time() - t0 < 3.0:
        time.sleep(0.05)
    if st["msg"] is None:
        sys.exit("No rt/lowstate within 3 s.")

    def fsm():
        code, data = lc._Call(ROBOT_API_ID_LOCO_GET_FSM_ID, json.dumps({}))
        return int(json.loads(data).get("data")) if code == 0 else None

    def tilt():
        rpy = st["msg"].imu_state.rpy
        return float(rpy[0]), float(rpy[1])

    f0 = fsm()
    r0, p0 = tilt()
    print(f"locomotion FSM {f0} ({A.WALK_FSM.get(f0, 'NOT walk-capable')}); IMU roll {r0:+.3f} pitch {p0:+.3f} rad; "
          f"mode_machine {int(st['msg'].mode_machine)}")
    total = sum(s for _, _, s in steps) + SETTLE_S * len(steps)
    print(f"plan ({a.pattern}, ~{total:.0f} s):")
    for label, (vx, vy, w), secs in steps:
        dist = f"~{abs(vx or vy) * secs * 100:.0f} cm" if (vx or vy) else f"~{abs(w) * secs * 57.3:.0f} deg"
        print(f"  {label:10s} vx {vx:+.2f} vy {vy:+.2f} omega {w:+.2f} for {secs:.1f} s ({dist}), then stop + {SETTLE_S:.0f} s settle")
    problems = []
    if f0 not in A.WALK_FSM:
        problems.append("not in a walk-capable state - remote: L2+Up (stand), wait, then R1+Y")
    if max(abs(r0), abs(p0)) > 0.15:
        problems.append(f"robot not upright (roll {r0:+.2f}, pitch {p0:+.2f})")
    for pr in problems:
        print("  PROBLEM:", pr)
    if not a.live:
        print("\nDRY RUN - nothing was sent. Add --live to walk.")
        return
    if problems:
        sys.exit("Not walking: fix the problems above first.")
    print("\n*** THE ROBOT WILL WALK ***")
    print("Confirm: 1.5 m clear around it; gantry slack but ready to catch; remote in hand (L2+B = limp);")
    print("nobody touching the joysticks; no other program sending commands.")
    if input("Type 'walk' to go, anything else aborts: ").strip() != "walk":
        sys.exit("Aborted by user - nothing was sent.")

    def on_signal(_signum, _frame):
        raise KeyboardInterrupt
    for sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, on_signal)

    def stop(why=""):
        for _ in range(3):  # redundant: a dropped request must not leave it walking
            try:
                lc.StopMove()
            except Exception:  # noqa: BLE001
                pass
            time.sleep(0.05)
        if why:
            print(f"STOP: {why}")

    os.makedirs(os.path.join(HERE, "logs"), exist_ok=True)
    log_path = os.path.join(HERE, "logs", time.strftime("walk_test_%Y%m%d_%H%M%S.csv"))
    reason = None
    start = time.time()
    with open(log_path, "w", newline="") as fh:
        log = csv.writer(fh)
        log.writerow(["t", "step", "vx", "vy", "omega", "roll", "pitch", "state_age"])
        try:
            for label, (vx, vy, w), secs in steps:
                print(f"{label}...")
                code = lc.SetVelocity(vx, vy, w, secs)
                if code != 0:
                    reason = f"SetVelocity rejected (code {code})"
                    break
                t_end = time.time() + secs + SETTLE_S
                moving_until = time.time() + secs
                while time.time() < t_end:
                    if time.time() >= moving_until and moving_until > 0:
                        lc.StopMove()
                        moving_until = 0
                    r, p = tilt()
                    age = time.time() - st["t"]
                    log.writerow([f"{time.time() - start:.3f}", label, vx, vy, w, f"{r:.4f}", f"{p:.4f}", f"{age:.3f}"])
                    if max(abs(r), abs(p)) > MAX_TILT:
                        reason = f"tilt roll {r:+.2f} pitch {p:+.2f} rad"
                    elif age > 0.2:
                        reason = f"robot state stale ({age:.2f} s)"
                    if reason:
                        break
                    time.sleep(0.02)
                if reason:
                    break
                f = fsm()
                if f not in A.WALK_FSM:
                    reason = f"locomotion FSM changed to {f}"
                    break
        except KeyboardInterrupt:
            reason = "Ctrl-C"
        except Exception as e:  # noqa: BLE001
            reason = f"error: {e!r}"
        finally:
            stop(reason or "")
    print(("aborted: " + reason) if reason else "done", "- StopMove sent. log:", log_path)


if __name__ == "__main__":
    main()

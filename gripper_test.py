#!/usr/bin/env python3
"""
gripper_test.py - *** MOVES A GRIPPER (with --live) ***  First Dex1-1 test: one side, small, gentle.

Without --live this is a DRY RUN: it reads rt/dex1/{left,right}/state, lists anyone else
commanding the grippers, prints the plan. Nothing is published.

With --live --side left|right (rt/dex1/<side>/cmd, unitree_go MotorCmds_ with one MotorCmd_):
  1. move    2 s  q0 -> q0 + delta (smooth). delta < 0 closes, > 0 opens (q = 0 is fully closed)
  2. hold    1 s
  3. return  2 s  back to q0
  4. stop publishing -> the gripper goes limp again (its state before the test)
Gentle: kp 5 (Toyota team: 8 to open, 20 to cut), |delta| <= 0.5 rad, never below q = 0.3.

Aborts (hold the measured position 0.3 s, then stop publishing) on: Ctrl-C / SIGTERM / SIGHUP,
state older than 0.1 s, |tau_est| above --tau-max (blocked / pinching something),
tracking error > 0.4 rad, gripper not following (< 50% of delta by the end of "hold").

  python gripper_test.py                                 # dry run (safe)
  python gripper_test.py --live --side left              # close 0.3 rad and back
  python gripper_test.py --live --side left --delta 0.3  # open 0.3 rad and back
Writes logs/gripper_test_<side>_<time>.csv on the laptop.
"""
import argparse
import csv
import os
import signal
import sys
import time

from arm_test import find_iface, remote_endpoints, smooth

HERE = os.path.dirname(os.path.abspath(__file__))
SIDES = ("left", "right")
DT = 0.02                     # 50 Hz
T_MOVE, T_HOLD, T_RETURN, T_ABORT_HOLD = 2.0, 1.0, 2.0, 0.3
MAX_DELTA = 0.5               # rad
MIN_Q = 0.3                   # never command closer than this to the fully-closed stop (q = 0)
KP_MAX = 20.0                 # Toyota team's cutting force; this test defaults to 5
MAX_TRACK_ERR = 0.4           # rad
MAX_STATE_AGE = 0.1           # s (state arrives at 500 Hz)
MIN_FOLLOW = 0.5


def plan(t, q0, delta, no_return=False):
    if no_return:   # --open-to: move and stay (then publishing stops; the gripper stays roughly there)
        if t < T_MOVE:
            return "move", q0 + delta * smooth(t / T_MOVE)
        if t < T_MOVE + T_HOLD:
            return "hold", q0 + delta
        return "done", q0 + delta
    if t < T_MOVE:
        return "move", q0 + delta * smooth(t / T_MOVE)
    if t < T_MOVE + T_HOLD:
        return "hold", q0 + delta
    if t < T_MOVE + T_HOLD + T_RETURN:
        return "return", q0 + delta * (1.0 - smooth((t - T_MOVE - T_HOLD) / T_RETURN))
    return "done", q0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--live", action="store_true", help="actually publish (MOVES THE GRIPPER)")
    ap.add_argument("--side", choices=SIDES, help="required with --live")
    ap.add_argument("--delta", type=float, default=-0.3, help=f"rad, |delta| <= {MAX_DELTA}; < 0 closes")
    ap.add_argument("--kp", type=float, default=5.0, help=f"(0, {KP_MAX:.0f}]")
    ap.add_argument("--kd", type=float, default=0.05, help="Toyota team uses 0.05")
    ap.add_argument("--tau-max", type=float, default=1.0, help="abort above this |tau_est| [N*m]")
    ap.add_argument("--open-to", type=float, default=None,
                    help="OPEN to this q and stay (no return), e.g. after a grasp left it closed; opening only, <= 3.0")
    ap.add_argument("--iface", default=None)
    a = ap.parse_args()
    if a.live and not a.side:
        sys.exit("--live needs --side left|right")
    if a.open_to is None and abs(a.delta) > MAX_DELTA:
        sys.exit(f"--delta {a.delta} exceeds the {MAX_DELTA} rad cap")
    if not 0 < a.kp <= KP_MAX:
        sys.exit(f"--kp must be in (0, {KP_MAX:.0f}]")

    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
    from unitree_sdk2py.idl.default import unitree_go_msg_dds__MotorCmd_
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_, MotorStates_

    iface = a.iface or find_iface()
    if not iface:
        sys.exit("No 192.168.123.x interface (see README step 1).")
    ChannelFactoryInitialize(0, iface)

    st = {s: {"q": None, "dq": 0.0, "tau": 0.0, "t": 0.0} for s in SIDES}

    def on_state(side):
        def cb(m):
            s = m.states[0]
            st[side].update(q=float(s.q), dq=float(s.dq), tau=float(s.tau_est), t=time.time())
        return cb

    subs = []
    for side in SIDES:
        sub = ChannelSubscriber(f"rt/dex1/{side}/state", MotorStates_)
        sub.Init(on_state(side), 10)
        subs.append(sub)
    t0 = time.time()
    while any(st[s]["q"] is None for s in SIDES) and time.time() - t0 < 3.0:
        time.sleep(0.05)

    for side in SIDES:
        d = st[side]
        if d["q"] is None:
            print(f"{side:5s}: NO STATE (dex1_1_service not running, or nothing on this side)")
            continue
        others = remote_endpoints(f"rt/dex1/{side}/cmd", "writer", wait_s=1.5)
        d["others"] = others
        print(f"{side:5s}: q {d['q']:.3f} rad  dq {d['dq']:+.3f}  tau {d['tau']:+.3f} N*m   "
              f"other commanders: {others or 'none'}")

    sides = [a.side] if a.side else [s for s in SIDES if st[s]["q"] is not None]
    problems = []
    for side in sides:
        d = st[side]
        if d["q"] is None:
            problems.append(f"{side}: no state")
            continue
        if a.open_to is not None:
            if not d["q"] < a.open_to <= 3.0:
                problems.append(f"{side}: --open-to must be above the current q {d['q']:.3f} and <= 3.0 (opening only)")
            a.delta = a.open_to - d["q"]
        target = d["q"] + a.delta
        word = "close" if a.delta < 0 else "open"
        back = "stay open" if a.open_to is not None else f"{d['q']:.3f}"
        print(f"plan {side}: {word} {abs(a.delta):.2f} rad: {d['q']:.3f} -> {target:.3f} -> {back}"
              f"  (kp {a.kp}, kd {a.kd}, abort above |tau| {a.tau_max})")
        if target < MIN_Q:
            problems.append(f"{side}: target {target:.3f} below {MIN_Q} (too close to the fully-closed stop)")
        if d.get("others"):
            problems.append(f"{side}: someone else is commanding this gripper: {d['others']}")
        if abs(d["tau"]) > 0.5 * a.tau_max:
            problems.append(f"{side}: already loaded (tau {d['tau']:+.3f}) - holding something?")
    for p in problems:
        print("  PROBLEM:", p)

    if not a.live:
        print("\nDRY RUN - nothing was published. Add --live --side left|right to move a gripper.")
        return
    if problems:
        sys.exit("Not moving: fix the problems above first.")

    side, d = a.side, st[a.side]
    print(f"\n*** THIS WILL MOVE THE {side.upper()} GRIPPER ***  ~{T_MOVE + T_HOLD + T_RETURN:.0f} s")
    print("Confirm: nothing and nobody's fingers between or near the gripper fingers; remote in hand (L2+B);")
    print("no other program commanding the grippers (Toyota team apps, teleop).")
    if input("Type 'move' to go, anything else aborts: ").strip() != "move":
        sys.exit("Aborted by user - nothing was published.")

    pub = ChannelPublisher(f"rt/dex1/{side}/cmd", MotorCmds_)
    pub.Init()
    cmd = MotorCmds_()
    cmd.cmds = [unitree_go_msg_dds__MotorCmd_()]
    mc = cmd.cmds[0]
    mc.dq, mc.tau, mc.kp, mc.kd = 0.0, 0.0, a.kp, a.kd

    def send(q):
        mc.q = float(q)
        if not pub.Write(cmd):
            raise RuntimeError(f"rt/dex1/{side}/cmd write failed")

    def on_signal(_signum, _frame):
        raise KeyboardInterrupt
    for sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, on_signal)

    q0 = d["q"]  # re-read right before moving
    os.makedirs(os.path.join(HERE, "logs"), exist_ok=True)
    log_path = os.path.join(HERE, "logs", time.strftime(f"gripper_test_{side}_%Y%m%d_%H%M%S.csv"))
    reason, hold_q, abort_t = None, None, None
    start, k = time.perf_counter(), 0
    with open(log_path, "w", newline="") as fh:
        log = csv.writer(fh)
        log.writerow(["t", "phase", "q_cmd", "q_meas", "dq", "tau_est", "state_age"])
        print("moving... (Ctrl-C = abort)")
        while True:
            t = time.perf_counter() - start
            try:
                if reason is None:
                    phase, qc = plan(t, q0, a.delta, a.open_to is not None)
                    if phase == "done":
                        break
                    age = time.time() - d["t"]
                    if age > MAX_STATE_AGE:
                        reason = f"gripper state stale ({age:.2f} s)"
                    elif abs(d["tau"]) > a.tau_max:
                        reason = f"torque {d['tau']:+.2f} N*m > {a.tau_max} (blocked / pinching?)"
                    elif abs(d["q"] - qc) > MAX_TRACK_ERR:
                        reason = f"tracking error {abs(d['q'] - qc):.2f} rad"
                    elif phase == "hold" and t >= T_MOVE + T_HOLD - 2 * DT and (d["q"] - q0) / a.delta < MIN_FOLLOW:
                        reason = f"gripper not following: moved {d['q'] - q0:+.3f} of {a.delta:+.3f} rad"
                    if reason:
                        print(f"ABORT: {reason} -> holding measured position, then releasing")
                        hold_q, abort_t = d["q"], t
                    else:
                        send(qc)
                        log.writerow([f"{t:.3f}", phase, f"{qc:.4f}", f"{d['q']:.4f}", f"{d['dq']:.4f}",
                                      f"{d['tau']:.4f}", f"{age:.3f}"])
                if reason is not None:
                    if t - abort_t >= T_ABORT_HOLD:
                        break
                    send(hold_q)  # stop pushing: hold where it actually is, briefly
                k += 1
                time.sleep(max(0.0, start + k * DT - time.perf_counter()))
            except KeyboardInterrupt:
                if reason is None:
                    reason, hold_q, abort_t = "Ctrl-C", d["q"], t
                    print("ABORT: Ctrl-C -> holding measured position, then releasing")
            except Exception as e:  # noqa: BLE001
                if reason is None:
                    reason, hold_q, abort_t = f"error: {e!r}", d["q"], t
                    print(f"ABORT: {reason}")
                time.sleep(DT)
                if t - abort_t > T_ABORT_HOLD + 1.0:
                    break
    pub.Close()
    time.sleep(0.2)
    print(("aborted: " + reason) if reason else "done",
          f"- stopped publishing (gripper limp). final q {d['q']:.3f} (start {q0:.3f}). log: {log_path}")


if __name__ == "__main__":
    main()

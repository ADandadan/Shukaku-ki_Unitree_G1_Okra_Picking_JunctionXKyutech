#!/usr/bin/env python3
"""
arm_test.py - *** MOVES THE ROBOT (with --live) ***  First arm_sdk test: ONE joint, small, slow.

Without --live this is a DRY RUN: it only reads rt/lowstate, prints the plan and builds
(but never publishes) the commands. Nothing is sent to the robot.

With --live, via rt/arm_sdk (high-level arm override on top of Unitree's balance controller).
The robot must be in motion-control mode first - remote: L2+Up, then R1+Y. Otherwise nothing
subscribes to rt/arm_sdk and the motors ignore it (our 2026-09-26 run; also the Toyota-body
team's FARM_QUICKSTART.md, same robot). The dry run checks this.
  1. engage   2 s  arm_sdk weight 0 -> 1, all arm + waist joints held at their CURRENT pose
  2. move     3 s  one joint q0 -> q0 + delta (smooth), default right elbow +0.15 rad
  3. hold     1 s
  4. return   3 s  back to q0
  5. release  2 s  weight 1 -> 0 (Unitree's controller takes the arms back)
Every other arm/waist joint is commanded to hold its start pose the whole time: at weight 1
a joint left at kp=0 would go limp (incl. the waist while balancing). Gains as in the
Toyota-body team's proven G1ArmSdkConnection: waist kp 300 / kd 3, wrists 40 / 1.5;
the arm joints use --kp/--kd (start low). Each LowCmd echoes mode_machine and sets mode=1.

Aborts (-> hold current command, weight -> 0 over 1.5 s) on: Ctrl-C / SIGTERM / SIGHUP, rt/lowstate older than
0.1 s, tracking error > 0.35 rad on any commanded joint, motor temperature >= 70 C,
mode_machine changing mid-run (robot left motion-control mode), joint not following.
Emergency stop on the remote: L2 + B held 5 s.

  python arm_test.py                                  # dry run (safe)
  python arm_test.py --live                           # right elbow +0.15 rad and back
  python arm_test.py --live --joint LeftElbow --delta -0.1
Writes logs/arm_test_<time>.csv (commanded vs measured) on the laptop.
"""
import argparse
import csv
import math
import os
import re
import signal
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))

# G1 29-dof motor indices (unitree_sdk2_python example/g1/high_level/g1_arm7_sdk_dds_example.py)
J = dict(WaistYaw=12, WaistRoll=13, WaistPitch=14,
         LeftShoulderPitch=15, LeftShoulderRoll=16, LeftShoulderYaw=17, LeftElbow=18,
         LeftWristRoll=19, LeftWristPitch=20, LeftWristYaw=21,
         RightShoulderPitch=22, RightShoulderRoll=23, RightShoulderYaw=24, RightElbow=25,
         RightWristRoll=26, RightWristPitch=27, RightWristYaw=28)
ARM_SDK_WEIGHT = 29          # motor_cmd[29].q = arm_sdk weight (0 = Unitree controller, 1 = us)
HELD = list(J.values())      # all joints arm_sdk takes over
WAIST = {12, 13, 14}
WRIST = {19, 20, 21, 26, 27, 28}
WAIST_KP, WAIST_KD = 300.0, 3.0   # Toyota-body G1ArmSdkConnection defaults (field-proven)
WRIST_KP, WRIST_KD = 40.0, 1.5
ARM_KP_MAX = 80.0                 # their proven arm kp
WALK_FSM = {500: "Walk Motion", 501: "Walk Motion 3Dof-waist", 801: "Run", 802: "Run (ai_sport)"}

# Limits from the robot's ~/g1_description/g1_29dof_rev_1_0.urdf (rad)
LIMITS = {12: (-2.618, 2.618), 13: (-0.520, 0.520), 14: (-0.520, 0.520),
          15: (-3.089, 2.670), 16: (-1.588, 2.252), 17: (-2.618, 2.618), 18: (-1.047, 2.094),
          19: (-1.972, 1.972), 20: (-1.614, 1.614), 21: (-1.614, 1.614),
          22: (-3.089, 2.670), 23: (-2.252, 1.588), 24: (-2.618, 2.618), 25: (-1.047, 2.094),
          26: (-1.972, 1.972), 27: (-1.614, 1.614), 28: (-1.614, 1.614)}
LIMIT_MARGIN = 0.10
MAX_DELTA = 0.30             # rad, hard cap for this first test
DT = 0.02                    # 50 Hz, as in Unitree's example
T_ENGAGE, T_MOVE, T_HOLD, T_RETURN, T_RELEASE, T_ABORT = 2.0, 3.0, 1.0, 3.0, 2.0, 1.5
MAX_TRACK_ERR = 0.35         # rad
MAX_TEMP_C = 70
MAX_STATE_AGE = 0.1          # s
MIN_FOLLOW = 0.5             # by the end of "hold" the joint must cover >= 50% of delta


def remote_endpoints(topic, kind="reader", wait_s=3.0):
    """Listen-only: DDS discovery data -> ["process@host", ...] for every remote reader
    (kind="reader") or writer (kind="writer") of `topic`.
    On 2026-09-26 nothing read rt/arm_sdk, so the first live test silently did nothing."""
    from cyclonedds.builtin import (BuiltinDataReader, BuiltinTopicDcpsParticipant,
                                    BuiltinTopicDcpsPublication, BuiltinTopicDcpsSubscription)
    from unitree_sdk2py.core.channel import ChannelFactory
    dp = ChannelFactory()._ChannelFactory__participant
    rp = BuiltinDataReader(dp, BuiltinTopicDcpsParticipant)
    re_ = BuiltinDataReader(dp, BuiltinTopicDcpsSubscription if kind == "reader" else BuiltinTopicDcpsPublication)
    time.sleep(wait_s)
    names = {}
    for p in rp.take(N=1000):
        props = {}
        try:
            props = {v["key"]: v["value"] for v in p.qos.asdict().values() if isinstance(v, dict) and "key" in v}
        except Exception:  # noqa: BLE001
            pass
        host = props.get("__NetworkAddresses", "").split("udp/")[-1].split(":")[0]
        names[str(p.key)] = f"{props.get('__ProcessName', '?')}@{host}"
    return [names.get(str(e.participant_key), str(e.participant_key)[:8])
            for e in re_.take(N=5000) if e.topic_name == topic and str(e.participant_key) != str(dp.guid)]


def remote_readers(topic, wait_s=3.0):
    return remote_endpoints(topic, "reader", wait_s)


def smooth(x):
    """0..1 -> 0..1 with zero velocity at both ends."""
    x = min(max(x, 0.0), 1.0)
    return 0.5 - 0.5 * math.cos(math.pi * x)


def plan(t, q0, goal, t_move=T_MOVE, t_hold=T_HOLD):
    """Pure trajectory: time since start -> (phase, weight, {motor: q}).
    goal = {motor: delta}: those joints go q0 -> q0 + delta and back together (same smooth
    profile); every other held joint stays at q0. Return takes as long as the move."""
    q = dict(q0)
    t1 = T_ENGAGE
    t2 = t1 + t_move
    t3 = t2 + t_hold
    t4 = t3 + t_move
    t5 = t4 + T_RELEASE
    if t < t1:
        return "engage", smooth(t / T_ENGAGE), q
    if t < t2:
        s = smooth((t - t1) / t_move)
    elif t < t3:
        s = 1.0
    elif t < t4:
        s = 1.0 - smooth((t - t3) / t_move)
    elif t < t5:
        return "release", 1.0 - smooth((t - t4) / T_RELEASE), q
    else:
        return "done", 0.0, q
    for j, d in goal.items():
        q[j] = q0[j] + d * s
    return ("move" if t < t2 else "hold" if t < t3 else "return"), 1.0, q


FOLLOW_TOL = 0.10            # waypoint mode: every moving joint within this of its target at the end of a hold


class Waypoints:
    """Multi-segment plan for Controller(planner=...): engage, then segments, then release.
    segments: list of (name, kind, q_target, duration); kind "move" = smooth from the previous
    target to q_target ({motor: q} for any HELD motors), "hold" = stay (q_target ignored).
    at(t) -> (phase, weight, q, check); check is True on the last ticks of each hold, where the
    Controller verifies the arm actually got there."""

    def __init__(self, q0, segments):
        self.q0 = dict(q0)
        self.segs, t, q_prev = [], T_ENGAGE, dict(q0)
        self.moving = set()
        for name, kind, q_target, dur in segments:
            q_to = dict(q_prev)
            if kind == "move":
                q_to.update(q_target)
                self.moving |= {j for j in q_target if abs(q_target[j] - q_prev[j]) > 1e-6}
            self.segs.append((name, kind, t, t + dur, dict(q_prev), q_to))
            t, q_prev = t + dur, q_to
        self.q_end = q_prev
        self.t_release = t
        self.total = t + T_RELEASE

    def at(self, t):
        if t < T_ENGAGE:
            return "engage", smooth(t / T_ENGAGE), dict(self.q0), False
        for name, kind, t0, t1, q_from, q_to in self.segs:
            if t < t1:
                if kind == "hold":
                    return name, 1.0, dict(q_to), t >= t1 - 2 * DT
                s = smooth((t - t0) / (t1 - t0))
                return name, 1.0, {j: q_from[j] + (q_to[j] - q_from[j]) * s for j in q_to}, False
        if t < self.total:
            return "release", 1.0 - smooth((t - self.t_release) / T_RELEASE), dict(self.q_end), False
        return "done", 0.0, dict(self.q_end), False


def find_iface():
    out = subprocess.run(["ip", "-o", "-4", "addr"], capture_output=True, text=True).stdout
    for line in out.splitlines():
        m = re.match(r"\d+:\s+(\S+)\s+inet\s+(192\.168\.123\.\d+)/", line)
        if m:
            return m.group(1)
    return None


def checklist(joint_name, delta, mode_name):
    print("\n*** THIS WILL MOVE THE ROBOT ***")
    print(f"    {joint_name} by {delta:+.2f} rad ({math.degrees(delta):+.0f} deg) and back, "
          f"~{T_ENGAGE + T_MOVE + T_HOLD + T_RETURN + T_RELEASE:.0f} s. Controller mode: '{mode_name}'.")
    print("Confirm ALL of these:")
    print("  - area around the robot and its arms is clear of people and objects")
    print("  - someone holds the remote and knows the emergency stop: L2 + B held 5 s")
    print("  - robot is secured in the frame/gantry as agreed with the mentor")
    print("  - robot is in motion-control mode (remote: L2+Up, then R1+Y)")
    print("  - NO other command source is running (Unitree app teleop, DimOS, other people's scripts)")
    return input("Type 'move' to go, anything else aborts: ").strip() == "move"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--live", action="store_true", help="actually publish to rt/arm_sdk (MOVES THE ROBOT)")
    ap.add_argument("--joint", default="RightElbow", choices=[k for k in J if not k.startswith("Waist")])
    ap.add_argument("--delta", type=float, default=0.15, help=f"rad, |delta| <= {MAX_DELTA}")
    # 2026-09-26 on this robot (elbow +0.15 rad): kp40/kd1.5 return lag 0.037 rad, kp60/kd2 0.028,
    # kp80/kd3 0.021 with 0.0015 overshoot; shoulder-roll gravity sag 0.049 -> 0.034 -> 0.025 rad.
    ap.add_argument("--kp", type=float, default=80.0, help="arm joints (80 = best on this robot, max 80)")
    ap.add_argument("--kd", type=float, default=3.0, help="arm joints (3.0 at kp 80)")
    ap.add_argument("--iface", default=None, help="network interface on 192.168.123.x (default: auto)")
    a = ap.parse_args()

    if abs(a.delta) > MAX_DELTA:
        sys.exit(f"--delta {a.delta} exceeds the {MAX_DELTA} rad cap for this first test")
    if not 0 < a.kp <= ARM_KP_MAX:
        sys.exit(f"--kp must be in (0, {ARM_KP_MAX:.0f}]")

    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
    from unitree_sdk2py.utils.crc import CRC

    iface = a.iface or find_iface()
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

    try:
        from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
        msc = MotionSwitcherClient()
        msc.SetTimeout(3.0)
        msc.Init()
        code, res = msc.CheckMode()
        mode_name = (res or {}).get("name", "") if code == 0 else "?"
    except Exception as e:  # noqa: BLE001
        mode_name = f"? ({e})"

    fsm_id, fsm_err = None, None
    try:  # read-only query: which locomotion state is the onboard controller in?
        import json
        from unitree_sdk2py.g1.loco.g1_loco_api import ROBOT_API_ID_LOCO_GET_FSM_ID
        from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
        lc = LocoClient()
        lc.SetTimeout(3.0)
        lc.Init()
        code, data = lc._Call(ROBOT_API_ID_LOCO_GET_FSM_ID, json.dumps({}))
        if code == 0:
            fsm_id = int(json.loads(data).get("data"))
        else:
            fsm_err = f"code {code}"
    except Exception as e:  # noqa: BLE001
        fsm_err = repr(e)

    readers = remote_readers("rt/arm_sdk")

    ms = st["msg"].motor_state
    q0 = {i: float(ms[i].q) for i in HELD}
    mm = int(st["msg"].mode_machine)
    joint = J[a.joint]
    lo, hi = LIMITS[joint]
    target = q0[joint] + a.delta
    temps = {i: int(ms[i].temperature[0]) for i in HELD}

    print(f"interface {iface}; controller mode '{mode_name}'; mode_machine {mm}")
    print(f"locomotion FSM: {fsm_id} ({WALK_FSM.get(fsm_id, 'not walk-capable')})" if fsm_id is not None
          else f"locomotion FSM: service not reachable ({fsm_err})")
    print(f"rt/arm_sdk subscribers on the robot: {readers or 'NONE'}")
    print(f"{a.joint} (motor {joint}): now {q0[joint]:+.3f} rad -> target {target:+.3f} rad "
          f"(URDF limits {lo:+.3f}..{hi:+.3f}, margin {LIMIT_MARGIN})")
    print(f"gains: arm kp {a.kp} kd {a.kd} | wrist {WRIST_KP}/{WRIST_KD} | waist {WAIST_KP}/{WAIST_KD}"
          f"   hottest held motor {max(temps.values())} C")

    problems = []
    if not (lo + LIMIT_MARGIN <= target <= hi - LIMIT_MARGIN):
        problems.append("target too close to / beyond the joint limit")
    if max(temps.values()) >= MAX_TEMP_C:
        problems.append(f"motor temperature >= {MAX_TEMP_C} C")
    if mode_name in ("", "?") or mode_name.startswith("?"):
        problems.append("no high-level controller active (arm_sdk needs Unitree's controller, e.g. mode 'ai')")
    if fsm_id not in WALK_FSM:
        problems.append("robot not in motion-control mode - on the remote: L2+Up, then R1+Y "
                        "(arm_sdk is ignored otherwise; see ARM_STATUS.md)")
    if not readers:
        problems.append("nothing subscribes to rt/arm_sdk - commands would be silently dropped "
                        "(same fix: L2+Up, then R1+Y)")
    for p in problems:
        print("  PROBLEM:", p)

    crc, cmd = CRC(), unitree_hg_msg_dds__LowCmd_()
    cmd.mode_pr = 0
    cmd.mode_machine = mm

    def gains(i):
        if i in WAIST:
            return WAIST_KP, WAIST_KD
        if i in WRIST:
            return WRIST_KP, WRIST_KD
        return a.kp, a.kd

    def build(weight, q):
        cmd.motor_cmd[ARM_SDK_WEIGHT].q = weight
        for i in HELD:
            mc = cmd.motor_cmd[i]
            mc.mode = 1
            mc.q, mc.dq, mc.tau = q[i], 0.0, 0.0
            mc.kp, mc.kd = gains(i)
        cmd.crc = crc.Crc(cmd)
        return cmd

    # Build every command of the plan (catches errors) without publishing.
    t_total = T_ENGAGE + T_MOVE + T_HOLD + T_RETURN + T_RELEASE
    for k in range(int(t_total / DT) + 2):
        phase, w, q = plan(k * DT, q0, {joint: a.delta})
        build(w, q)
        if k % 25 == 0 or phase == "done":
            print(f"  t={k * DT:5.2f}s {phase:8s} weight={w:.2f} q[{a.joint}]={q[joint]:+.3f}")

    if not a.live:
        print("\nDRY RUN - nothing was published. Add --live to move the robot.")
        return
    if problems:
        sys.exit("Not moving: fix the problems above first.")
    if not checklist(a.joint, a.delta, mode_name):
        sys.exit("Aborted by user - nothing was published.")

    os.makedirs(os.path.join(HERE, "logs"), exist_ok=True)
    log_path = os.path.join(HERE, "logs", time.strftime("arm_test_%Y%m%d_%H%M%S.csv"))
    pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
    pub.Init()

    # re-capture the start pose right before engaging (the arm may have moved while we waited)
    ms = st["msg"].motor_state
    q0 = {i: float(ms[i].q) for i in HELD}
    if int(st["msg"].mode_machine) != mm:
        sys.exit("mode_machine changed while waiting - redo L2+Up, R1+Y and run again. Nothing was published.")
    def send(w, q):
        if not pub.Write(build(w, q)):
            raise RuntimeError("rt/arm_sdk write failed")
    ctl = Controller(q0, {joint: a.delta}, st, send, mm)

    def on_signal(signum, _frame):  # kill / closed terminal: same abort-and-release path as Ctrl-C
        raise KeyboardInterrupt
    for sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, on_signal)
    with open(log_path, "w", newline="") as fh:
        print("moving... (Ctrl-C = abort and release)")
        reason = ctl.run(csv.writer(fh))
    print(("aborted: " + reason) if reason else "done", "- arm_sdk released. log:", log_path)
    print_joint_stats(ctl.stats, joint)


def print_joint_stats(stats, moved):
    """Per-joint tracking at weight 1 (measured - commanded), worst first."""
    if not stats:
        return
    names = {v: k for k, v in J.items()}
    print("per-joint tracking while under arm_sdk (measured - commanded):")
    for i, (mx, sm, n) in sorted(stats.items(), key=lambda kv: -kv[1][0])[:8]:
        tag = "  <- moved joint" if i == moved else ""
        print(f"  {names[i]:20s} max |err| {mx:.4f} rad ({math.degrees(mx):4.1f} deg)"
              f"   mean {sm / n:+.4f} rad{tag}")


class Controller:
    """50 Hz loop. Any abort (incl. Ctrl-C anywhere in the loop) holds the last
    commanded pose and fades the arm_sdk weight to 0; that part needs no robot state.
    goal = {motor: delta}. on_hold() (optional) is started once in a background thread
    when the hold phase begins (e.g. a camera snapshot) - it never blocks the loop."""

    def __init__(self, q0, goal, state, send, mode_machine=None, t_move=T_MOVE, t_hold=T_HOLD, on_hold=None,
                 planner=None, tick_hook=None, hold_phase="hold"):
        # planner (Waypoints) replaces the single there-and-back goal; tick_hook(t, phase, ms) runs every
        # normal tick after the arm command is sent (e.g. drive a gripper) - if it raises, the arm aborts.
        self.q0, self.goal, self.st, self.send = q0, dict(goal), state, send
        self.joint = max(self.goal, key=lambda j: abs(self.goal[j]))  # logged in q_cmd/q_meas
        self.t_move, self.t_hold, self.on_hold, self._hold_started = t_move, t_hold, on_hold, False
        self.planner, self.tick_hook, self.hold_phase = planner, tick_hook, hold_phase
        self.mm = mode_machine
        self.reason = None
        self.abort_t = self.abort_w = None
        self.last_q, self.last_w = dict(q0), 0.0
        self.stats = {}  # motor -> [max |meas - cmd|, sum(meas - cmd), n] at weight 1

    def _abort(self, reason, t):
        if self.reason is None:
            self.reason, self.abort_t, self.abort_w = reason, t, self.last_w
            print(f"ABORT: {reason} -> releasing")

    def _check(self):
        age = time.time() - self.st["t"]
        ms = self.st["msg"].motor_state
        err = max(abs(float(ms[i].q) - self.last_q[i]) for i in HELD) if self.last_w > 0.5 else 0.0
        hot = max(int(ms[i].temperature[0]) for i in HELD)
        if age > MAX_STATE_AGE:
            return f"rt/lowstate stale ({age:.2f} s)", age, err, ms
        mm = getattr(self.st["msg"], "mode_machine", self.mm)
        if self.mm is not None and int(mm) != self.mm:
            return f"mode_machine changed {self.mm} -> {mm} (robot left motion-control mode)", age, err, ms
        if err > MAX_TRACK_ERR:
            return f"tracking error {err:.2f} rad", age, err, ms
        if hot >= MAX_TEMP_C:
            return f"motor temperature {hot} C", age, err, ms
        return None, age, err, ms

    def step(self, t, log):
        """One tick. Returns False when finished (weight back at 0)."""
        if self.reason is None:
            if self.planner is not None:
                phase, w, q, check = self.planner.at(t)
            else:
                phase, w, q = plan(t, self.q0, self.goal, self.t_move, self.t_hold)
                check = False
            if phase == "done":
                self.send(0.0, q)
                return False
            reason, age, err, ms = self._check()
            if not reason and check:
                for j in sorted(self.planner.moving):
                    off = float(ms[j].q) - q[j]
                    if abs(off) > FOLLOW_TOL:
                        name = {v: k for k, v in J.items()}[j]
                        reason = (f"arm not following at '{phase}': {name} is {off:+.3f} rad from its target "
                                  "(blocked, or commands not accepted)")
                        break
            if not reason and self.planner is None and phase == "hold" and t >= T_ENGAGE + self.t_move + self.t_hold - 2 * DT:
                for j, d in self.goal.items():
                    moved = float(ms[j].q) - self.q0[j]
                    # both: under half-way AND clearly off. Small moves are dominated by gravity sag/friction
                    # (2026-09-27: shoulder roll 0.067 rad move, 45% done, 0.037 rad sag -> false abort).
                    if abs(d) >= 0.02 and moved / d < MIN_FOLLOW and abs(float(ms[j].q) - q[j]) > FOLLOW_TOL:
                        name = {v: k for k, v in J.items()}[j]
                        reason = (f"arm not following: {name} moved {moved:+.3f} of {d:+.3f} rad "
                                  "(commands not reaching / not accepted by the controller)")
                        break
            if not reason and phase == self.hold_phase and self.on_hold and not self._hold_started:
                self._hold_started = True
                threading.Thread(target=self.on_hold, daemon=True).start()
            if reason:
                self._abort(reason, t)
            else:
                self.send(w, q)
                self.last_q, self.last_w = q, w
                if self.tick_hook is not None:
                    self.tick_hook(t, phase, ms)
                meas = {i: float(ms[i].q) for i in HELD}
                log.writerow([f"{t:.3f}", phase, f"{w:.3f}", f"{q[self.joint]:.4f}",
                              f"{meas[self.joint]:.4f}", f"{err:.4f}", f"{age:.3f}"]
                             + [f"{q[i]:.4f}" for i in HELD] + [f"{meas[i]:.4f}" for i in HELD])
                if w >= 0.999:  # per-joint error stats while we fully own the joints
                    for i in HELD:
                        e = meas[i] - q[i]
                        st = self.stats.setdefault(i, [0.0, 0.0, 0])
                        st[0] = max(st[0], abs(e))
                        st[1] += e
                        st[2] += 1
                return True
        w = self.abort_w * (1.0 - smooth((t - self.abort_t) / T_ABORT))
        self.send(w, self.last_q)
        return w > 0.0

    def run(self, log):
        names = {v: k for k, v in J.items()}
        log.writerow(["t", "phase", "weight", "q_cmd", "q_meas", "max_err", "state_age"]
                     + [f"cmd_{names[i]}" for i in HELD] + [f"meas_{names[i]}" for i in HELD])
        start, k, running = time.perf_counter(), 0, True
        while running:
            try:
                running = self.step(time.perf_counter() - start, log)
                k += 1
                time.sleep(max(0.0, start + k * DT - time.perf_counter()))
            except KeyboardInterrupt:
                self._abort("Ctrl-C", time.perf_counter() - start)
            except Exception as e:  # noqa: BLE001 - any bug: still release the arms
                self._abort(f"error: {e!r}", time.perf_counter() - start)
                time.sleep(DT)
                if time.perf_counter() - start - self.abort_t > T_ABORT + 1.0:
                    print("could not finish releasing - use the remote (L2 + B)")
                    break
        return self.reason

if __name__ == "__main__":
    main()

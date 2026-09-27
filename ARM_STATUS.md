# Arm control (rt/arm_sdk): status (2026-09-26)

**Status: WORKING (2026-09-26 20:52).** First real arm motion: `arm_test.py --live`, right elbow
+0.150 rad commanded → +0.150 rad measured, no overshoot, clean handover and release
(log `logs/arm_test_20260926_205220.csv`). Pre-checks showed FSM 501 (Walk Motion 3Dof-waist)
and `humanoid@192.168.123.161` subscribed to rt/arm_sdk.
Cause of the earlier failure: the robot was not in motion-control mode.
Fix, on the remote: **L2+↑, then R1+Y** (emergency stop / damping: L2+B, recover with L2+↑).
Not development mode (L2+R2): that turns the controller off; leave it only by restarting.

### Tracking at kp 40 / kd 1.5 (elbow)
- handover drift 0.011 rad; bending tracked within 0.015 rad; hold exactly on target
- **return lagged up to 0.037 rad** and settled 0.0035 rad from start → gains low vs friction
  (Toyota team uses kp 80 and warns low authority stalls short of target)
- one other held joint sat ~0.049 rad off target all run (log didn't record which)

### Gain sweep (same move, 20:52-20:56) — now default kp 80 / kd 3
| max error (rad) | kp 40/kd 1.5 | kp 60/kd 2 | **kp 80/kd 3** |
|---|---|---|---|
| handover drift | 0.0110 | 0.0085 | 0.0079 |
| elbow bending | 0.0148 | 0.0086 | 0.0080 |
| elbow returning | 0.0371 | 0.0277 | 0.0207 |
| overshoot at hold | 0 | 0.0043 | 0.0015 |
| L / R shoulder-roll sag | ~0.049 | -0.034/+0.032 | -0.025/+0.018 |
The steady off-target joints are both shoulder rolls, mirror-signed: gravity sag, scaling ~1/kp.
For accurate reaching this needs gravity compensation (the Toyota team feeds URDF gravity torque as tau).

### Next
1. ~~Log every held joint, gain sweep~~ done.
2. Dex1 gripper test (left side, `rt/dex1/left`; see CLAUDE.md for the protocol).
3. Camera → robot-frame transform (URDF `d435_joint`), then IK reach.

Source: the Toyota-body team's repo (github.com/Orboh/DimOS_base_G1-TOYOTA-BODY-RESEARCH, `oda/`),
which picks okra on **this same robot** (same D435i serial 347622073233). Their
`oda/FARM_QUICKSTART.md`: "if the arm doesn't move at all although commands are sent → remote
L2+↑ → R1+Y (motion-control mode, confirmed on this machine). Without it the motors ignore
commands. The manuals' R1+X is for different firmware." That is exactly our symptom.

Their field-proven arm_sdk details, now in `arm_test.py`:
- each LowCmd: `mode_pr = 0`, `mode_machine` = value latched from rt/lowstate, `motor_cmd[i].mode = 1`
- gains: arms kp 80 / kd 3, wrists 40 / 1.5, **waist 300 / 3** (held at start pose); legs untouched
  (the onboard controller keeps balancing)
- if `mode_machine` changes mid-run, the robot stops honouring arm_sdk → abort
- Ctrl-C of their DimOS app does NOT ramp the weight down; they use `oda/arm_release.py`
  (hold measured pose, weight 1→0 over 2 s)
- read the locomotion FSM with `LocoClient` GetFsmId (7001): 500/501 walk, 801/802 run = arm_sdk OK;
  0 zero-torque, 1 damping, 4 lock-standing = not

## First run (before the fix)

### What happened
`arm_test.py --live` (right elbow +0.15 rad) ran all 11 s without an abort
(log `logs/arm_test_20260926_184243.csv`). The measured elbow stayed at 1.4285 rad the whole time.
Robot state was live: tick counter +1 per message at 1 kHz, joints varying by ~1e-5 rad.
The SDK's `Write()` succeeds even with no subscriber, so the commands were silently dropped.

### What DDS discovery showed (listen-only, laptop)
| Finding | Evidence |
|---|---|
| Nothing reads `rt/arm_sdk` | no DCPSSubscription on that topic |
| Unitree's locomotion service (`sport`) is not reachable | 6 processes *write* `rt/api/sport/request`, none reads it; `LocoClient.GetFsmId()` → error 3102; nobody publishes `rt/sportmodestate` |
| The only `rt/lowcmd` writer is `emergency_stop` (pid 2624, .161) | DCPSPublication |
| `python3` pid 2642 on .161 reads `rt/armsdk`, writes `rt/arm_sdk`, serves `rt/api/arm/*` | most likely Unitree's arm-action service (SDK error 7400: "The topic rt/armsdk is occupied.") |
| motion switcher reports mode `'ai'` | `CheckMode` |

Interpretation at the time: the walking/balance controller that normally consumes
`rt/arm_sdk` isn't running. The robot is probably held by `emergency_stop` + the gantry,
**not actively balancing**. Explained by the missing R1+Y: the motion-control mode (and its
arm_sdk subscriber) was never started.

## Procedure (each session)
1. Remote: L2+↑, then R1+Y. `python arm_test.py` (dry run) must show a walk-capable FSM and
   a subscriber on rt/arm_sdk; otherwise it refuses `--live`.
2. The user runs `python arm_test.py --live` in their own terminal; Claude reads `logs/`.

## Safeguards added to arm_test.py
- Reads the locomotion FSM (GetFsmId, read-only) and refuses `--live` unless walk-capable.
- Lists `rt/arm_sdk` subscribers (listen-only) and refuses `--live` if there are none.
- Aborts and releases if `mode_machine` changes mid-run.
- Aborts and releases if `Write()` fails.
- Aborts and releases if the joint hasn't covered ≥ 50% of the step by the end of "hold".
- SIGTERM / SIGHUP abort and release like Ctrl-C.

## Also learned
- `rt/lowstate` runs at ~1 kHz (not 500 Hz).

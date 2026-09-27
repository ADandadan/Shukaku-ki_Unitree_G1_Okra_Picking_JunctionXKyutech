#!/usr/bin/env python3
"""
preflight.py - READ-ONLY session check. Sends no commands, changes nothing on the robot.

Checks, in order, and ends with the exact next thing to do:
  link        192.168.123.x interface, ping motion PC (.161) + dev PC (.164)
  state       rt/lowstate rate, mode_machine, hottest motor
  mode        motion switcher mode, locomotion FSM (walk-capable = arm commands accepted),
              who subscribes to rt/arm_sdk, anyone else publishing arm/gripper commands
  grippers    rt/dex1/{left,right}/state
  camera      (ssh, read-only) master_service, videohub_pc4 on the RealSense, depth server on :55570
  laptop      torch GPU

  python preflight.py            # everything
  python preflight.py --no-ssh   # skip the ssh camera check
"""
import argparse
import json
import os
import socket
import subprocess
import sys
import tempfile
import time

import arm_test as A

DEV_PC, MOTION_PC = "192.168.123.164", "192.168.123.161"
# Exact, anchored command line: an unanchored pattern also matches the remote shell/sudo running it.
HUB = "^/unitree/module/video_hub_pc4/videohub_pc4 /dev/video4$"
FREE_CAMERA = ("ssh -t unitree@192.168.123.164 'sudo systemctl stop master_service; "
               f"sudo kill $(pgrep -f \"{HUB}\")'")
RESTORE = "ssh -t unitree@192.168.123.164 'sudo systemctl start master_service'   (or power the robot off)"

todo = []


def ok(msg):
    print(f"  \033[32mOK\033[0m    {msg}")


def bad(msg, fix=None):
    print(f"  \033[31mFIX\033[0m   {msg}")
    if fix:
        todo.append(fix)


def info(msg):
    print(f"  ..    {msg}")


def ping(ip):
    return subprocess.run(["ping", "-c", "1", "-W", "1", ip], capture_output=True).returncode == 0


def ssh_read(cmd, password):
    """Read-only remote command; password via a throwaway askpass script on the laptop."""
    with tempfile.NamedTemporaryFile("w", suffix=".sh", delete=False) as fh:
        fh.write(f"#!/bin/sh\necho '{password}'\n")
        askpass = fh.name
    os.chmod(askpass, 0o700)
    try:
        env = dict(os.environ, SSH_ASKPASS=askpass, SSH_ASKPASS_REQUIRE="force", DISPLAY=":0")
        r = subprocess.run(["setsid", "ssh", "-o", "ConnectTimeout=5", "-o", "StrictHostKeyChecking=accept-new",
                            f"unitree@{DEV_PC}", cmd], capture_output=True, text=True, timeout=30,
                           env=env, stdin=subprocess.DEVNULL)
        return r.stdout
    finally:
        os.unlink(askpass)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--no-ssh", action="store_true")
    ap.add_argument("--password", default=os.getenv("G1_PW", "123"))
    a = ap.parse_args()
    print("G1 preflight - READ ONLY, nothing is sent to the robot.\n")

    print("[link]")
    iface = A.find_iface()
    if not iface:
        bad("no 192.168.123.x address on this laptop",
            "Plug the robot LAN cable in; README step 1 (Windows static IP + WSL mirrored networking)")
        return finish()
    ok(f"interface {iface}")
    for name, ip in (("motion PC", MOTION_PC), ("dev PC", DEV_PC)):
        up = ping(ip)
        (ok if up else bad)(f"{name} {ip} " + ("answers" if up else "does not answer"))

    print("\n[robot state]")
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorStates_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
    ChannelFactoryInitialize(0, iface)
    st = {"n": 0, "msg": None}
    grip = {"left": None, "right": None}

    def on_low(m):
        st["n"] += 1
        st["msg"] = m
    subs = [ChannelSubscriber("rt/lowstate", LowState_)]
    subs[0].Init(on_low, 10)
    for side in grip:
        s = ChannelSubscriber(f"rt/dex1/{side}/state", MotorStates_)
        s.Init(lambda m, side=side: grip.__setitem__(side, float(m.states[0].q)), 10)
        subs.append(s)
    time.sleep(2.0)
    if st["msg"] is None:
        bad("no rt/lowstate", "Robot powered? Firewall/WSL networking (README troubleshooting)")
        return finish()
    ms = st["msg"].motor_state
    hot = max(int(ms[i].temperature[0]) for i in A.HELD)
    ok(f"rt/lowstate {st['n'] / 2.0:.0f} Hz, mode_machine {int(st['msg'].mode_machine)}")
    (ok if hot < 60 else bad)(f"hottest arm/waist motor {hot} C" + ("" if hot < 60 else " - let it cool (abort at 70)"))

    print("\n[mode]")
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
    if fsm_id in A.WALK_FSM:
        ok(f"locomotion FSM {fsm_id} ({A.WALK_FSM[fsm_id]}) - arm commands accepted")
    else:
        bad(f"locomotion FSM {fsm_id if fsm_id is not None else 'not answering'} - not motion-control mode",
            "Remote: L2+Up (stand), wait, then R1+Y. Feet on the floor in the gantry. (Not L2+R2.)")
    readers = A.remote_readers("rt/arm_sdk", wait_s=2.0)
    (ok if readers else bad)(f"rt/arm_sdk subscribers: {readers or 'NONE'}")
    others = {t: A.remote_endpoints(t, "writer", wait_s=1.0) for t in
              ("rt/arm_sdk", "rt/lowcmd", "rt/dex1/left/cmd", "rt/dex1/right/cmd")}
    for t, w in others.items():
        if t == "rt/lowcmd":
            info(f"{t} writers: {w or 'none'} (Unitree's own controller writes this - normal)")
        elif t == "rt/arm_sdk" and w and all(x == f"python3@{MOTION_PC}" for x in w):
            info(f"{t} writer {w}: Unitree's arm-action (gesture) service - idle unless a gesture runs; don't use app gestures")
        elif w:
            bad(f"someone else publishes {t}: {w}", f"Stop the other program publishing {t} (one command source only)")
        else:
            ok(f"nobody else publishes {t}")

    print("\n[grippers]")
    for side, q in grip.items():
        (ok if q is not None else bad)(f"{side}: " + (f"q {q:.3f} rad" if q is not None else "no state (cable/service?)"))

    print("\n[camera]")
    server = socket.socket()
    server.settimeout(0.7)
    depth_up = server.connect_ex((DEV_PC, 55570)) == 0
    server.close()
    if depth_up:
        ok("depth server running (port 55570) - `--source robot-depth` works")
    elif a.no_ssh:
        info("depth server not running; --no-ssh: camera state not checked")
        todo.append("bash run_depth_server.sh   # in its own terminal; Ctrl-C stops it")
    else:
        out = ssh_read(f"systemctl is-active master_service; pgrep -af '{HUB}' || true; "
                       "pgrep -af ik_camera_standalone | grep -v pgrep || true", a.password)
        lines = [ln for ln in out.splitlines() if ln.strip()]
        if not lines:
            bad("ssh to the dev PC failed", "Check ssh unitree@192.168.123.164 (password 123)")
        else:
            svc = lines[0].strip()
            hub = [ln for ln in lines[1:] if "videohub_pc4 /dev/video4" in ln]
            relay = [ln for ln in lines[1:] if "ik_camera_standalone" in ln]
            (ok if svc != "active" else bad)(f"master_service {svc}")
            (ok if not hub else bad)("videohub_pc4 " + ("holds the RealSense (pid " + hub[0].split()[0] + ")" if hub else "not running"))
            if relay:
                bad("the other team's camera relay is running: " + relay[0], "Ask the other team before taking the camera")
            if svc == "active" or hub:
                todo.append("Free the camera (one line, password 123):\n      " + FREE_CAMERA)
            todo.append("bash run_depth_server.sh   # in its own terminal; Ctrl-C stops it")

    print("\n[laptop]")
    try:
        import torch
        (ok if torch.cuda.is_available() else info)(
            "torch " + torch.__version__ + (" GPU " + torch.cuda.get_device_name(0) if torch.cuda.is_available() else " CPU only (tiling ~4.6 s/frame)"))
    except Exception as e:  # noqa: BLE001
        info(f"torch not importable: {e!r}")
    return finish()


def finish():
    print("\n" + ("=" * 70))
    if todo:
        print("TO DO, in this order:")
        for k, t in enumerate(todo, 1):
            print(f"  {k}. {t}")
    else:
        print("ALL GREEN. Next: dry runs, e.g.  python reach_test.py --target-torso 0.40,-0.20,0.00")
    print(f"\nEnd of day: {RESTORE}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

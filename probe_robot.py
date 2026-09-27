#!/usr/bin/env python3
"""
probe_robot.py - READ-ONLY health check of the laptop <-> G1 link.

It never publishes a command. It only:
  1. finds your 192.168.123.x network interface
  2. pings the G1's computers
  3. checks which service ports are open on the dev PC (.164)
  4. asks teleimager for its camera config and grabs one frame per camera
  5. listens to DDS for ~3 s: rt/lowstate (IMU, joints), Dex1 gripper state,
     and asks the motion switcher which high-level mode is active

Run:  python probe_robot.py            (auto-detect interface)
      python probe_robot.py --iface eth1
Writes probe_report.json and probe_<camera>.jpg next to this file.
"""
import argparse
import json
import os
import re
import socket
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
DEV_PC = "192.168.123.164"
HOSTS = {"motion control PC (.161)": "192.168.123.161", "dev PC (.164)": DEV_PC}
PORTS = {22: "ssh", 55555: "teleimager head cam", 55556: "teleimager L wrist cam",
         55557: "teleimager R wrist cam", 60000: "teleimager config", 60001: "teleimager WebRTC"}

report = {"time": time.strftime("%Y-%m-%d %H:%M:%S")}


def ok(msg):
    print(f"  \033[32mOK\033[0m   {msg}")


def bad(msg):
    print(f"  \033[31mFAIL\033[0m {msg}")


def info(msg):
    print(f"  ..   {msg}")


# 1 ------------------------------------------------------------------------- #
def find_iface(forced=None):
    print("\n[1] Network interface")
    try:
        out = subprocess.run(["ip", "-o", "-4", "addr"], capture_output=True, text=True).stdout
    except FileNotFoundError:
        out = ""
    found = None
    for line in out.splitlines():
        m = re.match(r"\d+:\s+(\S+)\s+inet\s+([\d.]+)/(\d+)", line)
        if m and m.group(2).startswith("192.168.123."):
            found = (m.group(1), m.group(2))
    if forced:
        found = (forced, found[1] if found else "?")
    if found:
        ok(f"interface {found[0]} has IP {found[1]}")
        if found[1] in ("192.168.123.161", "192.168.123.164"):
            bad("your laptop is using an IP that belongs to the robot! Pick e.g. 192.168.123.222")
    else:
        bad("no 192.168.123.x address found. Set a static IP 192.168.123.222/24 on the Windows "
            "Ethernet adapter and make sure WSL uses mirrored networking (README step 1-2).")
    report["iface"] = found
    return found[0] if found else None


# 2-3 ----------------------------------------------------------------------- #
def ping_and_ports():
    print("\n[2] Ping")
    report["ping"] = {}
    for name, ip in HOSTS.items():
        try:
            r = subprocess.run(["ping", "-c", "1", "-W", "1", ip], capture_output=True, text=True)
            alive = r.returncode == 0
        except FileNotFoundError:  # no ping binary: fall back to a TCP ssh check
            s = socket.socket()
            s.settimeout(1.0)
            alive = s.connect_ex((ip, 22)) == 0
            s.close()
        report["ping"][ip] = alive
        (ok if alive else bad)(f"{name} {ip}")
    print(f"\n[3] Ports on {DEV_PC}")
    report["ports"] = {}
    for p, what in PORTS.items():
        s = socket.socket()
        s.settimeout(0.7)
        is_open = s.connect_ex((DEV_PC, p)) == 0
        s.close()
        report["ports"][p] = is_open
        (ok if is_open else info)(f"{p:5d} {what}: {'open' if is_open else 'closed'}")
    return report["ports"]


# 4 ------------------------------------------------------------------------- #
def cameras(ports):
    print("\n[4] Cameras (teleimager)")
    try:
        import zmq
        import cv2
        from okra_vision import decode_jpeg
    except ImportError as e:
        bad(f"missing python package: {e}")
        return
    ctx = zmq.Context.instance()
    cams = {}
    if ports.get(60000):
        req = ctx.socket(zmq.REQ)
        req.setsockopt(zmq.LINGER, 0)
        req.setsockopt(zmq.RCVTIMEO, 1500)
        req.connect(f"tcp://{DEV_PC}:60000")
        try:
            req.send(b"GET_DATA")
            cfg = req.recv_json()
            report["teleimager_config"] = cfg
            for name, c in (cfg.get("camera") or cfg).items():
                if isinstance(c, dict) and c.get("enable_zmq"):
                    cams[name] = c
                    info(f"{name}: type={c.get('type')} shape={c.get('image_shape')} "
                         f"binocular={c.get('binocular')} port={c.get('zmq_port')}")
        except Exception as e:
            bad(f"config request failed: {e}")
        finally:
            req.close()
    else:
        info("teleimager is not running (port 60000 closed). Start it on the robot - README step 5.")
    for p in (55555, 55556, 55557):
        if ports.get(p) and not any(c.get("zmq_port") == p for c in cams.values()):
            cams[f"port{p}"] = {"zmq_port": p}
    report["frames"] = {}
    for name, c in cams.items():
        sub = ctx.socket(zmq.SUB)
        sub.setsockopt(zmq.CONFLATE, 1)
        sub.setsockopt(zmq.LINGER, 0)
        sub.connect(f"tcp://{DEV_PC}:{c['zmq_port']}")
        sub.setsockopt_string(zmq.SUBSCRIBE, "")
        if sub.poll(3000):
            img = decode_jpeg(sub.recv())
            if img is not None:
                path = os.path.join(HERE, f"probe_{name}.jpg")
                cv2.imwrite(path, img)
                report["frames"][name] = list(img.shape)
                ok(f"{name}: got frame {img.shape[1]}x{img.shape[0]} -> {os.path.basename(path)}")
            else:
                bad(f"{name}: data received but not a JPEG")
        else:
            bad(f"{name}: no frame within 3 s")
        sub.close()


# 5 ------------------------------------------------------------------------- #
def dds(iface):
    print("\n[5] DDS (unitree_sdk2py) - listening only")
    try:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorStates_
    except ImportError as e:
        bad(f"unitree_sdk2py not installed ({e}). Run setup_wsl.sh.")
        return
    if not iface:
        bad("no interface - skipping DDS")
        return
    ChannelFactoryInitialize(0, iface)
    got = {"low": [], "dex1_left": [], "dex1_right": []}

    def on_low(m):
        got["low"].append(m)

    subs = []
    s = ChannelSubscriber("rt/lowstate", LowState_)
    s.Init(on_low, 10)
    subs.append(s)
    for side in ("left", "right"):
        s = ChannelSubscriber(f"rt/dex1/{side}/state", MotorStates_)
        s.Init(lambda m, side=side: got[f"dex1_{side}"].append(m), 10)
        subs.append(s)
    time.sleep(3.0)

    n = len(got["low"])
    report["lowstate_hz"] = n / 3.0
    if n:
        m = got["low"][-1]
        rpy = [round(float(x), 3) for x in m.imu_state.rpy]
        q = [round(float(ms.q), 3) for ms in m.motor_state[:29]]
        temps = [int(ms.temperature[0]) if hasattr(ms.temperature, "__len__") else int(ms.temperature)
                 for ms in m.motor_state[:29]]
        report.update(mode_machine=int(m.mode_machine), imu_rpy=rpy, joint_q=q, motor_temp=temps)
        ok(f"rt/lowstate {n / 3:.0f} Hz, mode_machine={m.mode_machine}, IMU rpy={rpy}")
        info(f"left arm q (15-21):  {q[15:22]}")
        info(f"right arm q (22-28): {q[22:29]}")
        info(f"max motor temp: {max(temps)} C")
    else:
        bad("no rt/lowstate. Check interface/IP, firewall, and WSL mirrored networking (README).")

    for side in ("left", "right"):
        msgs = got[f"dex1_{side}"]
        if msgs and len(msgs[-1].states):
            q = float(msgs[-1].states[0].q)
            report[f"dex1_{side}_q"] = q
            ok(f"Dex1 {side} gripper q={q:.3f} rad")
        else:
            info(f"Dex1 {side}: no state (dex1_1_service not running, or no gripper on this side)")

    try:
        from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
        msc = MotionSwitcherClient()
        msc.SetTimeout(3.0)
        msc.Init()
        code, res = msc.CheckMode()
        report["motion_mode"] = res
        if code == 0:
            name = (res or {}).get("name", "")
            ok(f"motion switcher: active mode = '{name}'"
               + ("  (no high-level controller -> low-level/debug)" if not name else ""))
        else:
            info(f"motion switcher CheckMode returned code {code}")
    except Exception as e:
        info(f"motion switcher query skipped: {e}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--iface", help="network interface on 192.168.123.x (default: auto)")
    ap.add_argument("--skip-dds", action="store_true")
    a = ap.parse_args()
    print("G1 probe - READ ONLY, no commands are sent to the robot.")
    iface = find_iface(a.iface)
    ports = ping_and_ports()
    cameras(ports)
    if not a.skip_dds:
        dds(iface)
    path = os.path.join(HERE, "probe_report.json")
    with open(path, "w") as fh:
        json.dump(report, fh, indent=2, default=str)
    print(f"\nReport written to {path}")


if __name__ == "__main__":
    sys.path.insert(0, HERE)
    main()

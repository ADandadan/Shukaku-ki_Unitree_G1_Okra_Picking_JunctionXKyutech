#!/usr/bin/env python3
"""
robot_depth_server.py - READ-ONLY RealSense colour + depth publisher for the G1 head camera.

  *** Moves nothing. Sends no robot commands (no DDS at all). Writes NO files. ***

Runs ON the G1 dev PC (192.168.123.164), but is never copied there: start it from
the laptop with `bash run_depth_server.sh`, which sends this file over ssh and runs it
from memory (python -B: no .pyc). It only reads the camera and publishes on ZMQ.
It stops when the ssh session ends (Ctrl-C on the laptop): stdin EOF -> exit.

Must stay self-contained and Python 3.10 compatible (robot env:
~/miniconda3/envs/teleimager: pyrealsense2 2.50, pyzmq, opencv, numpy).

Wire format: ZMQ PUB, one single-part message per frame (so subscribers can CONFLATE):
  b"OKRD" | uint32 LE header_len | header JSON | colour JPEG | depth PNG (uint16, raw units)
  header = {v, seq, t, serial, depth_scale (m per unit), jpeg_len, depth_len,
            intr: {fx, fy, cx, cy, width, height}}   # colour intrinsics; depth is aligned to colour
Decoder: okra_vision.decode_depth_msg (laptop).

Laptop-side test without the robot:  python robot_depth_server.py --playback rec.bag
"""
import sys

sys.dont_write_bytecode = True  # belt and braces with python -B: never write .pyc on the robot

import resource  # noqa: E402

# No core dumps, ever: the G1's core_pattern is /tmp/core.%e.%p (would be a file write).
resource.setrlimit(resource.RLIMIT_CORE, (0, 0))

import argparse  # noqa: E402
import json  # noqa: E402
import os  # noqa: E402
import signal  # noqa: E402
import struct  # noqa: E402
import threading  # noqa: E402
import time  # noqa: E402

MAGIC = b"OKRD"
CAMERA_USERS = ("videohub_pc4", "teleimager", "image_server", "realsense-viewer")


def encode_msg(seq, color_bgr, depth_u16, depth_scale, intr, serial, jpeg_quality=90, png_level=1):
    import cv2
    ok1, jpg = cv2.imencode(".jpg", color_bgr, [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality])
    ok2, png = cv2.imencode(".png", depth_u16, [cv2.IMWRITE_PNG_COMPRESSION, png_level])
    if not (ok1 and ok2):
        return None
    jpg, png = jpg.tobytes(), png.tobytes()
    header = json.dumps(dict(v=1, seq=seq, t=time.time(), serial=serial, depth_scale=depth_scale,
                             jpeg_len=len(jpg), depth_len=len(png), intr=intr)).encode()
    return MAGIC + struct.pack("<I", len(header)) + header + jpg + png


def other_camera_users():
    """Read-only scan of /proc for processes that may hold the RealSense."""
    found = []
    for pid in os.listdir("/proc"):
        if not pid.isdigit() or int(pid) == os.getpid():
            continue
        try:
            with open(f"/proc/{pid}/cmdline", "rb") as fh:
                cmd = fh.read().replace(b"\0", b" ").decode(errors="replace").strip()
        except OSError:
            continue
        if any(u in cmd for u in CAMERA_USERS):
            found.append(f"{pid} {cmd}")
    return found


def watch_stdin(stop):
    """ssh session gone (laptop Ctrl-C / network drop) -> stdin EOF -> stop.
    Raw os.read, not sys.stdin: a daemon thread blocked in the buffered reader
    makes Python abort at shutdown ("could not acquire lock for <stdin>")."""
    try:
        while os.read(0, 4096):
            pass
    except Exception:
        pass
    stop.set()


def main():
    ap = argparse.ArgumentParser(description="READ-ONLY RealSense colour+depth ZMQ publisher")
    ap.add_argument("--port", type=int, default=55570)
    ap.add_argument("--width", type=int, default=1280, help="colour width (depth is aligned to colour)")
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--depth-width", type=int, default=848, help="848x480 = best D435 depth quality")
    ap.add_argument("--depth-height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=15, choices=(6, 15, 30))
    ap.add_argument("--jpeg-quality", type=int, default=90)
    ap.add_argument("--serial", default=None, help="RealSense serial (default: first device)")
    ap.add_argument("--playback", default=None, help="publish from a .bag/.db3 recording (testing, on the laptop)")
    ap.add_argument("--no-stdin-watch", action="store_true", help="don't exit on stdin EOF (local testing)")
    a = ap.parse_args()

    import numpy as np
    import pyrealsense2 as rs
    import zmq

    print("robot_depth_server: READ-ONLY (camera -> ZMQ). No robot commands, no files written.", flush=True)
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda *_: stop.set())
    if not a.no_stdin_watch:
        threading.Thread(target=watch_stdin, args=(stop,), daemon=True).start()

    if not a.playback:
        users = other_camera_users()
        if users:
            print("WARNING: these processes may already hold the RealSense:", flush=True)
            for u in users:
                print("   ", u, flush=True)

    pipe, cfg = rs.pipeline(), rs.config()
    if a.playback:
        cfg.enable_device_from_file(a.playback, repeat_playback=True)
    else:
        if a.serial:
            cfg.enable_device(a.serial)
        cfg.enable_stream(rs.stream.color, a.width, a.height, rs.format.bgr8, a.fps)
        cfg.enable_stream(rs.stream.depth, a.depth_width, a.depth_height, rs.format.z16, a.fps)
    try:
        profile = pipe.start(cfg)
    except RuntimeError as e:
        print(f"ERROR: could not start the RealSense: {e}", flush=True)
        if "busy" in str(e).lower():
            print("The camera is in use by another process (see WARNING above - on the G1 usually "
                  "Unitree's videohub_pc4 or teleimager). Stopping that is a mentor decision; "
                  "this script will not do it.", flush=True)
        sys.exit(2)

    dev = profile.get_device()
    serial = dev.get_info(rs.camera_info.serial_number) if dev.supports(rs.camera_info.serial_number) else ""
    depth_scale = float(dev.first_depth_sensor().get_depth_scale())
    cp = profile.get_stream(rs.stream.color).as_video_stream_profile()
    ci = cp.get_intrinsics()
    intr = dict(fx=ci.fx, fy=ci.fy, cx=ci.ppx, cy=ci.ppy, width=ci.width, height=ci.height)
    color_is_rgb = cp.format() == rs.format.rgb8  # recordings may store rgb8
    align = rs.align(rs.stream.color)

    ctx = zmq.Context.instance()
    pub = ctx.socket(zmq.PUB)
    pub.setsockopt(zmq.SNDHWM, 2)  # slow subscriber: drop frames, don't queue
    pub.setsockopt(zmq.LINGER, 0)
    pub.bind(f"tcp://*:{a.port}")
    print(f"publishing {serial} colour {ci.width}x{ci.height} + aligned depth (scale {depth_scale:g} m) "
          f"on tcp://*:{a.port}  fx={ci.fx:.1f} fy={ci.fy:.1f} cx={ci.ppx:.1f} cy={ci.ppy:.1f}", flush=True)

    seq, t_last, n_last, period = 0, time.time(), 0, 1.0 / a.fps
    try:
        while not stop.is_set():
            ok, frames = pipe.try_wait_for_frames(2000)
            if not ok:
                print("no frames for 2 s", flush=True)
                continue
            frames = align.process(frames)
            c, d = frames.get_color_frame(), frames.get_depth_frame()
            if not c or not d:
                continue
            img = np.asanyarray(c.get_data())
            if color_is_rgb:
                img = img[:, :, ::-1]
            msg = encode_msg(seq, np.ascontiguousarray(img), np.asanyarray(d.get_data()),
                             depth_scale, intr, serial, a.jpeg_quality)
            if msg:
                pub.send(msg, copy=False)
                seq += 1
            if a.playback:
                time.sleep(period)
            now = time.time()
            if now - t_last >= 10:
                print(f"{(seq - n_last) / (now - t_last):.1f} fps, {len(msg or b'') / 1024:.0f} KiB/frame", flush=True)
                t_last, n_last = now, seq
    finally:
        pipe.stop()
        pub.close()
        print(f"stopped after {seq} frames; camera released.", flush=True)


if __name__ == "__main__":
    try:
        main()
        code = 0
    except SystemExit as e:
        code = e.code if isinstance(e.code, int) else 1
    except BaseException:
        import traceback
        traceback.print_exc()
        code = 1
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)  # skip interpreter finalization (the stdin watchdog thread may still be blocked)

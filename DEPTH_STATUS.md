# Live depth from the G1 head camera: status (2026-09-26)

**Status: WORKING (19:30).** After the user ran `sudo systemctl stop master_service` and
`sudo kill <videohub_pc4 /dev/video4 pid>` on .164 (stopping the service does NOT kill its running
children), `run_depth_server.sh` streamed 1280×720 colour + aligned depth: ~6.6 fps, ~409 KiB/frame,
92% valid depth pixels, intrinsics fx 911.0 fy 910.8 cx 645.6 cy 388.6. Stopped cleanly; nothing left
or written on the robot. Restore Unitree's video: `sudo systemctl start master_service` or reboot.

Per session: after every robot reboot, repeat the stop + kill (master_service comes back at boot).

**Next problem: detection, not the camera.** On the live stream the model marked the whole green
carpet as okra (conf up to 0.87, ~1.3 m wide) and missed a real pod hanging by the stakes.
Depth enables a metric size check (pod ≈ 5–20 cm long) to reject such false positives.

**Heads-up:** the other team's relay `ik_camera_standalone.py` starts at boot and tries to grab the
RealSense (its log `~/ik_cam_standalone.log` was written 2 min after the 19:22 boot, "busy").
Only one program can hold the camera. Another machine (.100) was logged in earlier today.

## History (before it worked)

## What's ready
- `okra_vision.py` uses RealSense depth when a source provides it (`locate()`: median depth
  inside the eroded okra mask, exact colour intrinsics). Falls back to the size prior.
- `robot_depth_server.py` + `run_depth_server.sh`: read-only colour + aligned-depth publisher
  that runs on the G1 **from memory** (nothing written on the robot). Laptop side:
  `python detect_okra.py --source robot-depth`.
- Tested end-to-end on the laptop with a synthetic RealSense recording: depth within 1 mm,
  intrinsics delivered, clean shutdown when ssh closes.

## Why it doesn't work on the robot yet
```
ERROR: could not start the RealSense: xioctl(VIDIOC_S_FMT) failed Last Error: Device or resource busy
```
Unitree's own `/unitree/module/video_hub_pc4/videohub_pc4 /dev/video4` (runs as root) holds the
RealSense colour node. Re-tested 19:24 right after a reboot: it's already running 1 min after boot and
its PID changed between two checks, so `master_service` starts it at boot and respawns it.
teleimager would hit the same conflict.

## What the Toyota-body team does (same robot, `oda/RUN_HEAD_IK.md`)
- Killing `videohub_pc4` doesn't help: **`master_service` respawns it within seconds**. They run
  `sudo systemctl stop master_service` on the dev PC (.164). It only stops the two video hubs there;
  arm control is on .161 and unaffected. Undo: `sudo systemctl start master_service` (reboot also restores it).
- Their camera relay `~/run_ik_camera.sh` → `ik_camera_standalone.py` (repo `~/workSpace/dimos_min`,
  teleimager env Python 3.10) publishes over LCM multicast 239.255.76.67:7667 at 640×480@15,
  K=[607.3, 607.2, 323.7, 259.1]. It **writes `~/ik_cam_standalone.log` on the robot** (breaks our
  no-writes rule) and needs sudo multicast routes on both the laptop and the NX (the NX sends
  multicast over WiFi otherwise). Our `run_depth_server.sh` is plain TCP to .164 and writes nothing.

## To unblock (mentor decision)
1. Ask a mentor whether we may run `sudo systemctl stop master_service` on .164 (not a file write,
   undone by `systemctl start` or a reboot; the other team does this routinely).
2. Once the camera is free: `bash run_depth_server.sh`, then `python detect_okra.py --source robot-depth`.
   No code changes needed.

## Robot-side safety notes
- Mentor rule: anything may run on the G1 as long as its files are only read.
- The G1's `core_pattern` is `/tmp/core.%e.%p` and apport is active, so a crash could write a file.
  The server sets `RLIMIT_CORE=0` and exits via `os._exit`. The first test run aborted before
  this fix, but a check confirmed nothing was written (ssh sessions have `ulimit -c 0`).

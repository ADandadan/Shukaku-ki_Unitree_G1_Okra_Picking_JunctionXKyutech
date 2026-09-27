# okra-g1: okra detection for the Unitree G1

**Live demo? Follow [DEMO.md](DEMO.md)** (setup → find → walk → pick → end, with fallbacks).

Stage 1: see okra and know where they are. **Nothing here moves the robot.**
Every script only reads (camera frames, joint states, gripper state).

```
G1 head camera ──teleimager (ZMQ, JPEG)──►  laptop (WSL 2)
                                             ├─ YOLO11n-seg okra model  → masks
                                             ├─ RealSense depth (or size prior) → distance
                                             └─ 3D point (camera frame, metres)
G1 DDS (rt/lowstate, rt/dex1/*/state) ─────► probe_robot.py (listen only)
```

| File | What it does |
|---|---|
| `windows_network_setup.ps1` | static IP 192.168.123.222 + WSL mirrored networking (run once, as Admin) |
| `setup_wsl.sh` | installs Python env, YOLO, CycloneDDS, unitree_sdk2_python, downloads the model |
| `robot_inventory.sh` | SSH into the robot and **list** cameras/processes/config (no changes) |
| `preflight.py` | **start here each session**: read-only check of everything + the fixes to do, in order |
| `probe_robot.py` | checks ping, ports, camera stream, DDS joint + gripper state |
| `run_depth_server.sh` + `robot_depth_server.py` | **read-only** RealSense colour + depth stream from the robot (nothing written on the robot) |
| `detect_okra.py` | live okra detection + 3D position (`--tile` for small/far pods) |
| `arm_test.py` | **MOVES the arm (`--live`)**: one joint, small step and back (first arm_sdk test) |
| `gripper_test.py` | **MOVES a gripper (`--live --side`)**: small close/open and back |
| `reach_test.py` | **MOVES the arm (`--live`)**: hover / grasp / pull a target or detected pod |
| `pick.py` | picking loop: fresh measurement, auto arm + wrist angle, runs `reach_test.py` (`--live` to move) |
| `walk_test.py` | **WALKS (`--live`)**: small forward/back/turn steps via Unitree's walk controller |
| `g1_kinematics.py` | arm FK/IK + head-camera → torso transform from the robot's URDF (`urdf/`) |
| `okra_vision.py` | the library behind it |

---

## 1. Windows network (once)
Cable in the robot's top LAN port. For the read-only scripts any mode is fine.
**Arm/walking (`arm_test.py`) needs motion-control mode:** feet on the floor in the gantry, remote **L2+↑** (stand), then **R1+Y**. Do **not** use Development Mode (L2 + R2): it shuts down the controller that executes arm commands (it's only for low-level `rt/lowcmd`, robot hanging), and you leave it only by restarting the robot. Emergency stop: **L2+B** (joints go limp - the gantry must hold it).

Admin PowerShell, in this folder:
```powershell
powershell -ExecutionPolicy Bypass -File windows_network_setup.ps1
```
If it can't pick the adapter: `-Adapter "Ethernet 2"` (name from the table it prints).

## 2. WSL setup (once, ~10-20 min)
Open Ubuntu (WSL):
```bash
cd "/mnt/c/Users/aqilq/Documents/okra-g1"
bash setup_wsl.sh
```
Afterwards, a new terminal + `okra` puts you in the venv in this folder.

## 3. Check the link (read-only)
```bash
okra
ping -c 2 192.168.123.164
bash robot_inventory.sh        # password 123 -> robot_inventory.txt
python probe_robot.py          # -> probe_report.json, probe_*.jpg
```
You want to see: `rt/lowstate ~1000 Hz`, arm joint angles, and (if running) the Dex1 gripper q.

## 4. Test the detector without the robot
```bash
python detect_okra.py --source some_okra_photo.jpg
python detect_okra.py --source webcam:0          # if WSL sees a webcam
```

## 5. Robot camera stream
`probe_robot.py` tells you if **teleimager** is already streaming (port 55555 open).
If not: the G1 PC already has teleimager 1.5.0 installed. **Ask a mentor before
starting it** (the guide says don't change robot software; starting an installed
server is normally fine). Then, in an SSH session on the robot:
```bash
teleimager-server            # add --rs if the head camera is a RealSense
```
Then on the laptop:
```bash
python detect_okra.py --source robot
```

## 6. Distance (RealSense depth)
The G1 head camera is an Intel RealSense D435i, so distance comes from its depth
image: the median depth inside the okra mask, back-projected with the colour
intrinsics. No calibration is needed.

teleimager only streams colour JPEGs (it can read depth but never sends it), so
`--source robot` falls back to the size-based estimate (okra length ~10 cm, ±30%).
For live depth, use our own read-only publisher instead of teleimager:
```bash
bash run_depth_server.sh                          # terminal 1, password 123; Ctrl-C stops it
python detect_okra.py --source robot-depth        # terminal 2
```
`run_depth_server.sh` sends `robot_depth_server.py` inside the ssh command and runs it
from memory (`python -B`, in the robot's existing `teleimager` conda env).
**Nothing is written on the robot.** It only reads the camera and publishes on ZMQ port 55570.
It sends no robot commands. Mentor rule: anything is OK as long as robot files are only read.

Only one program can use the RealSense. If the server says the camera is busy, another
program holds it: usually Unitree's `videohub_pc4` (it was on the colour node `/dev/video4`)
or teleimager. Stopping those is a **mentor decision**; the script never does it.

Other depth sources:
```bash
python detect_okra.py --source realsense          # RealSense plugged into this machine
python detect_okra.py --source recording.bag      # RealSense recording (.bag or .db3)
```
Depth is invalid closer than ~0.2 m (D435 minimum range); the okra then falls back to the size estimate.

Output (`--json`) per okra: `center` (px), `xyz` (m, camera frame: x right, y down, z forward),
`angle_deg` (pod orientation, useful for gripper roll), `conf`, `depth_method` (`depth n=<pixels>` or `size-prior`).

## 7. Moving the robot (arm, gripper, reach)
Every script is a **dry run by default** (reads state, plans, prints checks, publishes nothing).
`--live` moves the robot after a typed `move`; run it yourself in your own terminal, watching the robot.
1. Robot standing in the gantry, feet on the floor. Remote: **L2+↑**, then **R1+Y** (motion-control mode).
   Not Development Mode (L2+R2). Emergency stop: **L2+B**.
2. `python arm_test.py` (dry run) must show a walk-capable FSM (500/501/801/802) and a `rt/arm_sdk` subscriber.
3. `python arm_test.py --live` — elbow ~9° and back (defaults kp 80 / kd 3, tested on this robot).
4. `python gripper_test.py --live --side left` — close 0.3 rad and back, gentle (kp 5).
5. `python reach_test.py --target-torso 0.40,-0.20,0.00 [--live --snapshot]` — hover reach; `--snapshot`
   (needs `run_depth_server.sh`) saves the camera view with where the kinematics thinks the hand is.
   `python reach_test.py --detect` plans the reach to a detected pod instead.
6. `python reach_test.py --detect --grasp [--live]` — hover, approach, close the gripper on contact, lift 3 cm,
   let go, back (~28 s). Only after the hover reach lands within ~1-2 cm of the pod.
Logs: `logs/*.csv`. Status/details: `ARM_STATUS.md`, `DEPTH_STATUS.md`.

---

## Safety (from the G1 guide)
- One command source at a time. These scripts send **no** commands.
- The robot's files are only ever read: `run_depth_server.sh` runs its script from memory.
- Anything that moves joints later (arm/gripper tests): robot **suspended, feet off the floor**, remote in hand, emergency stop = **L2 + B for 5 s**.
- Nothing is installed or written on the robot's internal PC. Everything else runs on this laptop.

## Troubleshooting
- **No `192.168.123.x` in WSL**: `wsl --shutdown`, reopen; check `%UserProfile%\.wslconfig` has `networkingMode=mirrored` (needs WSL ≥ 2.0: `wsl --update`).
- **Ping works but no `rt/lowstate`**: Windows Defender Firewall → allow inbound UDP on the Ethernet profile, or temporarily turn it off for that network; re-run the Hyper-V line in the .ps1.
- **`cv2.imshow` error**: use `--no-show --save out/`.
- **Slow on CPU**: fine for yolo11n (~10-20 fps); add `--device 0` if WSL sees an NVIDIA GPU.

Model: [Kota0612/okra11n-seg-v5](https://huggingface.co/Kota0612/okra11n-seg-v5) (AGPL-3.0).

# okra-g1 — hackathon: Unitree G1 detects and picks okra (walking if needed)

## Setup
- Laptop: Windows + WSL 2 (Ubuntu). Code runs in WSL from `/mnt/c/Users/aqilq/Documents/okra-g1`.
- Python env: `~/okra-venv` = Python 3.12 via uv (created by `setup_wsl.sh`). Must stay <= 3.12: cyclonedds 0.10.2 breaks on 3.13+
  (Ubuntu 26.04 ships 3.14). NOW HAS CUDA torch 2.14.0+cu126 (installed 2026-09-26; venv 7 GB; RTX 3050 Ti, driver CUDA 12.7).
  "Out of disk" is NOT the disk (945 GB free): /tmp is a 1.9 GB tmpfs and pip unpacks there -> big installs need
  `TMPDIR=~/.pip-tmp PIP_NO_CACHE_DIR=1 pip install ...` (setup_wsl.sh does this). That was the original setup failure.
- Robot LAN: laptop 192.168.123.222/24 (Windows static IP, WSL `networkingMode=mirrored`, via `windows_network_setup.ps1`).
  G1 motion PC 192.168.123.161, G1 dev PC 192.168.123.164 (ssh unitree@…164, pw 123).
- Robot has on board: unitree_sdk2 (C++), unitree_sdk2_python, dex1_1_service (Dex1-1 gripper, DDS `rt/dex1/{left,right}/{cmd,state}`, unitree_go MotorCmds_/MotorStates_),
  teleimager 1.5.0 (camera server: ZMQ PUB raw JPEG on 55555 head / 55556-7 wrists, config REQ/REP on 60000, send b"GET_DATA" → JSON), librealsense 2.50, ZED SDK 4.2, Livox-SDK2.
- Reference: github.com/Orboh/DimOS_base_G1-TOYOTA-BODY-RESEARCH `oda/` = another team's okra picking on THIS robot
  (D435i head cam via NX relay, rt/arm_sdk arm control, Dex1 on rt/dex1/left: close q~1.8, open 3.7 with blade, kp 8-20).
  Their docs are in Japanese. Gripper: MotorCmds_ with one MotorCmd_ (q, kp, kd), stops holding when publishing stops.
- Mentor-recommended tools: DimOS (dimensionalOS/dimos, G1 = beta; `dimos --simulation run unitree-g1-sim`), unitree_sdk2, unitree_ros2, YOLO.
- Model: HF `Kota0612/okra11n-seg-v5` (YOLO11n-seg, 1 class okra, weights `output/okra_finetune_v5/weights/best.pt`, AGPL-3.0).

## Files
- `okra_vision.py` – FrameSource yields (name, bgr, depth_m|None) from file/folder/video/webcam/robot ZMQ (colour only)/realsense/.bag/.db3 (pyrealsense2, aligned depth + exact intrinsics), OkraDetector, `locate()` = median RealSense depth in eroded mask, size-prior fallback; camera-frame xyz (x right, y down, z forward). CameraCalib defaults = D435i colour 1280x720.
- `detect_okra.py` – CLI (`--source robot|realsense|x.bag`, `--depth auto|size|none`, `--json`, `--no-show --save out/`).
- `probe_robot.py` – read-only link check (ping, ports, camera frames, rt/lowstate, Dex1 state, motion-switcher mode) → probe_report.json.
- `robot_inventory.sh` – read-only SSH inventory of the robot → robot_inventory.txt.
- `robot_depth_server.py` – READ-ONLY RealSense colour+aligned-depth ZMQ PUB (port 55570; msg = b"OKRD"+u32 hdr len+JSON hdr w/ intrinsics+JPEG+16-bit PNG; decoder `okra_vision.decode_depth_msg`). Must stay self-contained + Python 3.10 (robot env `~/miniconda3/envs/teleimager`, pyrealsense2 2.50). `--playback x.db3` for laptop tests.
- `arm_test.py` – *** MOVES ROBOT with --live *** first rt/arm_sdk test: one joint, |delta|<=0.3 rad, kp<=60 (default 40), 50 Hz,
  weight ramp 0->1->0, holds ALL 17 arm+waist joints at start pose (weight 1 + kp 0 = limp), aborts (fade weight to 0) on
  Ctrl-C / stale lowstate / tracking err >0.35 / temp >=70 C. Default = dry run. Logs logs/arm_test_*.csv. Offline-tested with a fake robot.
- `gripper_test.py` – *** MOVES A GRIPPER with --live --side *** Dex1-1 small move (|delta|<=0.5, q>=0.3, kp 5, kd 0.05) and back,
  then stops publishing (gripper goes limp). Aborts on |tau_est|>1.0, stale state, not following. Refuses if another
  process writes rt/dex1/<side>/cmd. Offline-tested with a fake SDK; dry run OK on robot (left q 2.515, right q 5.354).
  User: normal fingers, both wide open at those q (sides have different zero points), both cabled.
  LEFT live 2026-09-26 21:03: close -0.300 cmd -> -0.267 measured (89%), q down = closing (seen), tau max 0.42,
  lag ~160 ms, friction ~0.19 N*m at kp 5.
  RIGHT live 21:04: -0.300 cmd -> -0.277 (92%), tau max 0.28, lag ~180 ms (direction not yet confirmed by eye).
  Step 2 done. Next step 3: depth size filter (carpet false positive) -> head-camera->robot transform (URDF d435_joint) -> IK reach.
- `g1_kinematics.py` – numpy FK/IK from `urdf/g1_29dof_mode_15_with_dex1_1.urdf` (copied read-only from robot ~/g1_description).
  Chain torso_link -> <side>_wrist_yaw_link + tool (grasp 0.15 m / tip 0.1845 m along x, tip per Toyota team; grasp = guess).
  camera_to_torso(): D435 colour optical -> torso via URDF d435_joint xyz (0.0576, 0.0175, 0.4299), pitch 0.8308 rad down;
  assumes d435_link = REP-103 frame at the colour sensor (~1-2 cm uncertainty). Offline-checked: logged hanging pose ->
  hands at (0.04, -/+0.25, -0.25) symmetric; IK 200 targets <0.3 mm; centre-pixel floor depth consistent with standing height.
  Hands hanging are OUT of the head camera's view; forearm forward (elbow ~0) brings them into view (for a hand-eye check).
- okra_vision: detections carry length_m/width_m/plausible; detect_okra drops non-pods unless --keep-implausible:
  shape (length >= 3x width, always) + metric size from depth (0.03-0.35 m long, <=0.08 m wide).
  LIVE 2026-09-26: carpet "okra" 1.65x0.66 m dropped every frame.
- MODEL LIMIT: yolo11n okra model only finds a pod when it fills much of its input. Held pod ~93 px at ~1 m: missed on
  full frame; found with tiled inference `--tile 400 --tile-imgsz 1280` (conf 0.71, 87x20 px) + 8 blob false positives
  that the shape check removes. ~4.6 s/frame on CPU, ~1.4 s on GPU (default device picks it). GPU is ~50 ms/tile regardless
  of imgsz (WSL GPU overhead / laptop GPU in P3 at 11 W); batching and FP16 did not help (tested, reverted).
- `reach_test.py` – *** MOVES ROBOT with --live *** hand grasp point to a HOVER point (default 10 cm short along torso -x) of a
  target (--target-torso / --target-cam / --detect via robot-depth + tiling), hold, back. Pure planner plan_reach(): picking box
  x .20-.50 y -.40-.10 z -.30-.20 (right; mirrored left), IK err <1 cm, |dq|<=1.2 rad, 0.10 rad limit margin, hand+elbow path
  outside torso column |y|<.12 and above z -.40, speed 0.4 rad/s. kp 80/kd 3. --snapshot = hand-eye check (FK tip/grasp/target
  projected on a camera frame + depth at those pixels). Offline: planner tests from the real logged pose + full main() run
  with fake SDK (OK). NOT yet run on the robot. Right-arm targets across the midline are refused (elbow swings in).
- `DEMO.md` – live demo runbook chaining preflight → depth server → pick.py (dry) → walk_test → pick.py --live, fallbacks.
- `pick.py` – picking loop (MOVES only with --live, via reach_test.py which asks 'move'): measure FRESH each round (tiled det +
  depth, >=2/N frames within 2 cm, inside a picking box; end-on pods allowed at elongation >= 1.3), pod 3D axis = PCA of mask
  depth points, choose arm + finger roll over -150..150 deg by FK (tool y = finger closing axis; checked vs robot: roll 0 ->
  left-right, -90 -> up-down) scoring |closing . pod axis| + 0.3*(not vertical) + 0.1*effort, then runs reach_test.py
  --grasp --pull --tool-x 0.19 --grip-travel 2.0 --finger-roll <chosen> as a subprocess. Stakes still get detected ->
  numbered out/pick_candidates.jpg + "which candidate?" prompt (--auto skips). Offline-tested on the saved pole frame:
  picked left arm roll -90 for the sideways pod (|cos| 0.03). NOT yet run on the robot. Next: visual correction at hover.
- `walk_test.py` – *** WALKS with --live *** LocoClient.SetVelocity (Unitree's walk controller), patterns fwd-back/turn/side,
  caps vx<=.3 vy<=.2 w<=.4, <=3 s/step, StopMove after each step/on every exit (x3), abort on |roll|/|pitch|>.35, stale state,
  FSM leaving 500/501/801/802. LIVE 2026-09-27 11:07: fwd-back 0.15 m/s x 2 s each OK, gait sway 1.0 Hz, max tilt 0.075 rad.
- `preflight.py` – READ-ONLY session check + ordered fix list. Camera free: `sudo systemctl stop master_service; sudo kill
  $(pgrep -f "^/unitree/module/video_hub_pc4/videohub_pc4 /dev/video4$")` (ANCHORED - unanchored pgrep/pkill matches its own shell).
  Tested with fakes (not-ready + all-green paths).
- reach_test `--grasp` (+ --lift 0.03, --grip-kp 8, --tool-x): arm_test.Waypoints plan to_hover, hold, approach (<=0.8 rad, 0.2 rad/s),
  grip (GripperHook: close 0.3 rad/s up to 1.5 rad until the fingers lag > 0.15 rad = contact, then contact-0.15 squeeze), lift,
  hold_lift, let_go (reopen), retreat, to_start (~28 s). Aborts: gripper stale/tau>3 -> arm release + gripper limp; FOLLOW_TOL 0.10
  at hold ends. Offline: GripperHook sims (pod/no pod/stale/spike), plan_grasp from real pose, full main() --grasp with fake SDK.
  NOT run on the robot. Chest-height targets plan OK; low/far refused.
- g1_kinematics.Chain(tool=None) reads module TOOL_GRASP at construction (so --tool-x works).
- arm_test.Controller now takes goal={motor: delta} (+ t_move, t_hold, on_hold thread); plan() verified bit-identical to the
  proven single-joint version (1150 samples) and all controller safety tests re-run OK.
- `DEPTH_STATUS.md` – why live depth is parked (videohub_pc4 holds the RealSense).
- `ARM_STATUS.md` – why arm control is blocked (nothing subscribes to rt/arm_sdk; sport service not running).
- `run_depth_server.sh` – runs it on the G1 from memory (base64 in the ssh command, `python -B`); exits on ssh close (stdin-EOF watchdog). Laptop: `--source robot-depth`.

## RESULTS 2026-09-27 robot hour (for the pitch)
- WALKING works: fwd-back, turn, one-way turn/forward (walk_test.py). In the gantry, commanded distances under-deliver and it
  drifts/turns (35 cm asked -> ~4 cm + 16 deg yaw); re-detect after every walk. Max tilt seen 0.075 rad.
- Right gripper direction confirmed (q down = close), like the left.
- Detection: green stakes/poles are detected as okra (conf up to 0.8); area-based pod width (mask area/length) fixed curved
  pods being dropped. Pods pointing AT the camera are not detected (end-on) -> measured by pixel + depth instead.
- Hand-eye (snapshot): real Dex1 fingers ~4 cm longer than the model -> --tool-x 0.19. Stretched arm sags ~3-5 cm (kp 80).
  Aim offsets are pose-dependent: +5 cm was right at x 0.48, 5 cm too high at x 0.41 -> used --aim 0.
- GRASP: 113919 5 cm too high (aim +5 too much at x .41); 114052 reached but pushed the pod (fingers closed left-right).
  FIX that worked: --finger-roll -90 (fingers close up-down; vertical error is our worst) -> 114534 grabbed (no contact
  logged: 1.5 rad travel ran out), --grip-travel 2.0 -> 114856 contact q .757, held 1.2 N*m; --pull (retreat while holding)
  -> 115125 PULLED THE OKRA OFF. 115312/115404 failed only because they REUSED an old --target-cam after the pod moved.
  Winning command: reach_test.py --target-cam=<fresh> --side left --grasp --pull --tool-x 0.19 --finger-roll -90 --grip-travel 2.0
  (pod ~0.43 m ahead, 0.22 left, waist height; left arm; fresh measurement each attempt).
- NEXT (offline first): pick.py loop = measure fresh each attempt (reachable-region search incl. end-on pods), auto
  finger-roll + approach from pod 3D axis (mask + depth), visual correction at the hover (gripper + pod both in view),
  gravity compensation.
Fixes next: gravity compensation (tau feed-forward, as the Toyota team) instead of --aim; control wrist orientation so
fingers straddle the pod; slower last cm + close earlier; detector retrain with stake negatives or tap-to-select.
arm_test follow check: now needs <50% AND >0.10 rad off (small moves false-aborted on gravity sag).

## Status / next steps
1. DONE 2026-09-26: setup ok, DDS works in WSL (rt/lowstate, Dex1 L+R state, motion mode 'ai').
   Head camera = Intel RealSense D435i (USB 8086:0b3a) -> real depth available; stereo-disparity path not needed.
   Unitree `videohub_pc4` currently holds the RealSense (/dev/video4). teleimager 1.5.0 is in conda env
   `~/miniconda3/envs/teleimager` (not on PATH), not running. G1 URDFs in robot `~/g1_description`. Robot is shared (others' code in ~).
2. Run detector on live stream. Depth code done (RealSense). BLOCKER: teleimager 1.5.0 never publishes depth
   (get_depth_frame() unused, ZMQ = colour JPEG only) -> robot stream uses size prior. Live depth: run_depth_server.sh
   WORKS since 2026-09-26 19:30 once the user stops master_service AND kills videohub_pc4 (/dev/video4) - see DEPTH_STATUS.md.
   Detection problem found: green carpet = "okra" (conf 0.87) -> add a metric size filter using depth.
   (tested end-to-end on laptop with a synthetic recording). Earlier on the robot: "Device or resource busy" -
   Unitree's videohub_pc4 (root, PID 8862 then) holds the RealSense colour node /dev/video4. Stopping it = mentor decision.
   Robot has core_pattern /tmp/core.%e.%p + apport: server sets RLIMIT_CORE=0 and exits via os._exit to never write files.
   pyrealsense2 needs apt libusb-1.0-0 (in setup_wsl.sh).
3. Head-camera → robot-frame transform (use G1 URDF / measure).
4. (IN PROGRESS) G1 = 29 dof, waist unlocked, Dex1-1 on both arms (URDF g1_29dof_mode_15_with_dex1_1). arm_test.py dry run OK
   on robot 2026-09-26. First --live run (user ran it): arm did NOT move - nothing subscribes to rt/arm_sdk,
   sport/loco service unreachable (GetFsmId 3102), only rt/lowcmd writer = `emergency_stop`.
   CAUSE (per Toyota-body repo, same robot): not in motion-control mode -> remote L2+Up then R1+Y. arm_test.py now
   sets mode=1/mode_machine/waist kp300 and checks FSM. WORKS 2026-09-26 20:52: elbow +0.150 cmd -> +0.150 measured,
   FSM 501, subscriber humanoid@.161. Gain sweep 40/60/80: kp80/kd3 best (return lag 0.021, shoulder-roll gravity sag
   ~0.025 rad) -> now arm_test default. Step 1 done; next step 2 = Dex1 gripper test. See ARM_STATUS.md.
   Auto-mode denies Claude running --live motion commands: the user runs them in their own terminal, Claude reads logs/.
   Useful read-only diagnostic: cyclonedds.builtin DCPSParticipant/Publication/Subscription (__ProcessName, host per endpoint).
   Next after unblock: gripper test (read ~/dex1_1_service/README.md on robot), then IK.
   Arm reach: high-level `rt/arm_sdk` (see unitree_sdk2_python/example/g1/high_level/g1_arm7_sdk_dds_example.py) + IK; gripper via rt/dex1/*/cmd.
5. Walking to okra: LocoClient or DimOS.

## SAFETY RULES — must follow (from the G1 safety guide)
- NEVER send any motion command (arm_sdk, lowcmd, loco, gripper cmd) without first asking the user to confirm:
  robot suspended in the frame with feet off the floor (for debug/low-level code), area clear, remote in hand.
  Emergency stop: L2 + B held 5 s. Start slow (low gains / small steps) and ramp up.
- Only ONE command source at a time (not remote + SDK + DimOS together).
- Do NOT install or modify anything on the G1's internal PC. Mentor (2026-09-26): running anything is OK as long as robot files are only READ, never written (no copying scripts there, no .pyc, no logs) -> run code from memory like run_depth_server.sh. Stopping Unitree services (e.g. videohub_pc4) is still a mentor decision.
- Dex1 power only from top port #2 (DC 24 V). Never change the remote pairing.
- Default to read-only scripts; label any script that moves the robot clearly.

# Live demo runbook: find → walk → reach → grab → pull

All commands run in WSL: `okra` (activates the venv in this folder).
Every movement script is a **dry run unless `--live`**, and asks you to type `move` / `walk` before anything moves.
Emergency stop: **L2 + B** on the remote (joints go limp, so the gantry must hold the robot). Ctrl-C in a script stops it cleanly.

## 0. Setup (~5 min)
1. Robot in the gantry, feet on the floor, area clear. Power on and wait ~2 min.
2. Remote: **L2 + ↑** (stand), wait, then **R1 + Y** (motion-control mode). Not L2 + R2.
3. `python preflight.py` and do its fixes in the order it lists them:
   - free the camera (one line, password 123):
     `ssh -t unitree@192.168.123.164 'sudo systemctl stop master_service; sudo kill $(pgrep -f "^/unitree/module/video_hub_pc4/videohub_pc4 /dev/video4$")'`
   - second terminal: `bash run_depth_server.sh` (leave it running)
4. Re-run `python preflight.py` until **ALL GREEN**.
5. If a gripper was left closed: `python gripper_test.py --live --side left --open-to 2.4` (same for right).

## 1. Scene
- Okra taped to a stick/pole so it sticks out ≥ 5 cm, **pointing at the robot**, good light.
- Best spot: ~40–45 cm in front of the waist, in front of one shoulder (~20 cm to that side), 0–15 cm above the waist.
- Keep other green poles out of the direct path if you can (the detector sometimes calls them okra).

## 2. Find the okra and plan (dry run, nothing moves)
`python pick.py`
- It measures fresh, lists the candidates, and saves `out/pick_candidates.jpg` with numbered circles.
  Type the number that is the okra.
- It prints the chosen arm, finger roll and the `reach_test.py` command.
- **"No pod within reach"** → go to step 3 (walk), or move the okra closer.

## 3. Walk closer (only if needed)
When no okra is in reach, `python pick.py` looks up to 2 m away and **prints the exact commands**, e.g.
```
SUGGESTED MOVE to bring it to the left arm (rough - re-run pick.py after each move):
  python walk_test.py --live --pattern turn-right --angle 22
  python walk_test.py --live --pattern forward --distance 0.29
```
How it's computed: each arm grasps best at ~42 cm ahead and 20 cm to its side (bearing ~25°, distance ~47 cm).
turn = pod bearing − sweet-spot bearing (capped at 30°); walk = pod distance − 47 cm (capped at 40 cm).
It picks the arm on the pod's side. If the pod is too high/low it says so: walking can't change height, so move the okra.
- In the gantry the robot covers less than commanded and drifts (2026-09-27: 35 cm asked → ~4 cm + 16° turn), so
  **run the turn, then `python pick.py` again, then the walk it suggests next.** Usually 2–3 rounds.
- Check the box/rig clearance to the feet before every forward walk.

## 4. Pick (the demo moment, ~33 s)
`python pick.py --live`: choose the candidate → it runs the grasp → type `move`.
Sequence: hover 10 cm short → slide in → close until contact → lift 3 cm → **pull back 10 cm holding** → let go → return.
- Success line: `gripper: contact at q … -> still holding`.
- Proven fallback (the exact settings that pulled the okra off on 2026-09-27; put in the fresh target from step 2):
  `python reach_test.py --target-cam=X,Y,Z --side left --grasp --pull --tool-x 0.19 --finger-roll -90 --grip-travel 2.0 --live`
- **Never reuse an old `--target-cam`.** Both failed attempts on 2026-09-27 did that. Re-measure after anything moves.

## 5. Shorter demos if time or conditions are bad
- Walking: `python walk_test.py --live` (forward + back, ~8 s)
- Arm reach only, no grab: the pick command without `--grasp` (add `--snapshot` to save the camera view)
- Detection: `python detect_okra.py --source robot-depth --save out/ --tile 400 --tile-imgsz 1280`

## 6. End
- Ctrl-C the depth server.
- `ssh -t unitree@192.168.123.164 'sudo systemctl start master_service'`, or power the robot off.

## If something goes wrong
| Symptom | Fix |
|---|---|
| preflight: FSM not walk-capable | remote L2 + ↑, then R1 + Y |
| preflight: no gripper state | wait 1–2 min after boot (the Dex1 service starts late) |
| camera "busy" | the free-camera line again (needed after every reboot) |
| arm doesn't move / "arm not following" | not in motion-control mode, or another program is sending arm commands |
| gripper: NO CONTACT | target stale or okra moved → re-run `pick.py`; the fingers passed beside the okra |
| hand ~5 cm too high/low | add `--aim 0,0,-0.03` (lower) or `0,0,0.03` (higher) to the reach command |

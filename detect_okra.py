#!/usr/bin/env python3
"""
Detect okra and estimate where they are (camera frame). Read-only: never
sends anything to the robot.

Examples
  python detect_okra.py --source test_images/                 # photos
  python detect_okra.py --source webcam:0                     # laptop webcam
  python detect_okra.py --source robot                        # G1 head camera (teleimager, colour only)
  python detect_okra.py --source robot-depth                  # G1 head camera + depth (run_depth_server.sh)
  python detect_okra.py --source realsense                    # RealSense on this machine: real depth
  python detect_okra.py --source recording.bag                # RealSense recording (.bag/.db3): real depth
  python detect_okra.py --source robot --no-show --json       # headless, print JSON

Keys in the window: q = quit, s = save snapshot.
"""
import argparse
import json
import os
import time

import cv2

from okra_vision import CameraCalib, FPS, FrameSource, OkraDetector, draw, locate, locate_size, okra_sized


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", default="robot",
                    help="image | folder | video | webcam:N | robot[:host[:port]] | robot-depth[:host[:port]] | realsense[:serial] | file.bag|.db3")
    ap.add_argument("--weights", default=None, help="path to .pt (default: download okra11n-seg-v5)")
    ap.add_argument("--conf", type=float, default=0.35)
    ap.add_argument("--device", default=None, help="cpu, 0 (GPU) ... default: auto")
    ap.add_argument("--tile", type=int, default=0,
                    help="tiled inference: tile size px (e.g. 320-480; 0 = off). Needed for small/far pods; ~100 ms/tile on CPU")
    ap.add_argument("--tile-imgsz", type=int, default=960)
    ap.add_argument("--depth", choices=["auto", "size", "none"], default="auto",
                    help="auto = depth image if the source has one (RealSense), else size prior")
    ap.add_argument("--hfov", type=float,
                    help="set horizontal field of view (deg) in camera_calib.json (non-RealSense sources)")
    ap.add_argument("--keep-implausible", action="store_true",
                    help="keep detections whose depth-measured size isn't okra-like (default: drop them)")
    ap.add_argument("--no-show", action="store_true")
    ap.add_argument("--save", default=None, help="folder to write annotated frames")
    ap.add_argument("--json", action="store_true", help="print detections as JSON lines")
    ap.add_argument("--max-frames", type=int, default=0)
    args = ap.parse_args()

    cal = CameraCalib.load()
    if args.hfov:
        cal.hfov_deg, cal.fx = args.hfov, None
        cal.save()
        print(f"[calib] hfov set to {args.hfov} deg")

    det = OkraDetector(args.weights, conf=args.conf, device=args.device,
                       tile=args.tile, tile_imgsz=args.tile_imgsz)
    src = FrameSource(args.source)
    if args.save:
        os.makedirs(args.save, exist_ok=True)

    fps = FPS()
    n = 0
    for name, view, depth in src:
        if src.intrinsics:
            cal.set_intrinsics(src.intrinsics)
        elif (cal.width, cal.height) != (view.shape[1], view.shape[0]):
            cal.width, cal.height = view.shape[1], view.shape[0]
            cal.fx = cal.fy = cal.cx = cal.cy = None  # saved intrinsics were for another size

        mode = args.depth
        if mode == "auto":
            mode = "depth" if depth is not None else "size"
        okras = det.detect(view)
        for o in okras:
            if mode == "depth":
                locate(o, depth, cal)
            elif mode == "size":
                locate_size(o, cal)
        for o in okras:
            o.plausible = okra_sized(o)  # shape always; metric size when depth is known
        dropped = [o for o in okras if o.plausible is False]
        if not args.keep_implausible:
            okras = [o for o in okras if o.plausible is not False]

        f = fps.tick()
        if args.json:
            print(json.dumps({"frame": name, "t": time.time(),
                              "okra": [o.to_dict() for o in okras]}), flush=True)
        elif not src.is_live or n % 15 == 0:
            desc = ", ".join(
                f"#{k} {o.conf:.2f}" + (f" xyz=({o.xyz[0]:+.2f},{o.xyz[1]:+.2f},{o.xyz[2]:.2f})m [{o.depth_method}]"
                                        if o.xyz else f" [{o.depth_method or 'no depth'}]")
                + (f" size {o.length_m * 100:.0f}x{o.width_m * 100:.1f}cm" if o.length_m is not None else "")
                + (" IMPLAUSIBLE" if o.plausible is False else "")
                for k, o in enumerate(okras)) or "none"
            gone = (f"  [dropped {len(dropped)} not okra-shaped/sized: "
                    + ", ".join(f"{o.conf:.2f} " + (f"{o.length_m:.2f}x{o.width_m:.2f}m" if o.length_m is not None
                                                     else f"{o.length_px:.0f}x{o.width_px:.0f}px") for o in dropped)
                    + "]") if dropped and not args.keep_implausible else ""
            print(f"{name}: {len(okras)} okra  {desc}{gone}   ({f:.1f} fps)")

        vis = draw(view, okras)
        cv2.putText(vis, f"{f:.1f} fps  depth={mode}", (8, vis.shape[0] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1, cv2.LINE_AA)
        if args.save and (not src.is_live or n % 10 == 0):
            cv2.imwrite(os.path.join(args.save, f"{os.path.splitext(name)[0]}_det.jpg"), vis)
        if not args.no_show:
            cv2.imshow("okra", vis)
            k = cv2.waitKey(1 if src.is_live else 0) & 0xFF
            if k == ord("q"):
                break
            if k == ord("s"):
                p = f"snap_{int(time.time())}.jpg"
                cv2.imwrite(p, vis)
                print("saved", p)
        n += 1
        if args.max_frames and n >= args.max_frames:
            break
    if not args.no_show:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

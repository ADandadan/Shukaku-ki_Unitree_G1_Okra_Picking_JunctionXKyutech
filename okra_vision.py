"""
okra_vision.py - okra detection + 3D localisation for the Unitree G1.

Pieces:
  * FrameSource   - frames from an image, folder, video, webcam, or the G1's
                    teleimager ZMQ stream (raw JPEG over ZMQ PUB/SUB).
  * OkraDetector  - YOLO11n-seg model Kota0612/okra11n-seg-v5 (1 class: okra).
  * Depth         - the G1 head camera is a RealSense D435i:
      - "depth": median of the aligned depth image inside the okra mask, then
        back-projected with the colour intrinsics. Needs a source that carries
        depth: `realsense` (pyrealsense2, camera on this machine) or a recording (`.bag`, or `.db3` from librealsense >= 2.56).
        The robot's teleimager stream is colour-only (it never publishes depth).
      - "size":  fallback when there's no depth (photos, teleimager, holes in
        the depth map): monocular guess from okra length (~10 cm). Rough (+-30%).
All 3D points are in the CAMERA frame: x right, y down, z forward (metres).
"""
from __future__ import annotations

import glob
import json
import os
import time
from dataclasses import dataclass, field, asdict
from typing import Iterator, List, Optional, Tuple

import cv2
import numpy as np

MODEL_REPO = "Kota0612/okra11n-seg-v5"
MODEL_FILE = "output/okra_finetune_v5/weights/best.pt"
HERE = os.path.dirname(os.path.abspath(__file__))
CALIB_PATH = os.path.join(HERE, "camera_calib.json")

IMG_EXT = (".jpg", ".jpeg", ".png", ".bmp", ".webp")
VID_EXT = (".mp4", ".avi", ".mov", ".mkv")
RS_EXT = (".bag", ".db3")          # RealSense recordings (colour + depth)


# --------------------------------------------------------------------------- #
# Frame sources
# --------------------------------------------------------------------------- #
class FrameSource:
    """Iterate (name, bgr, depth) tuples. `depth` is float32 metres aligned to
    the colour image (0 = no data), or None if the source has no depth.
    `spec` examples:
         okra.jpg | photos/ | clip.mp4 | webcam:0 | robot | robot:192.168.123.164:55555
         robot-depth | robot-depth:192.168.123.164:55570   (run_depth_server.sh on the G1)
         realsense | realsense:<serial> | recording.bag | recording.db3
    Depth sources fill `self.intrinsics` (colour: fx, fy, cx, cy, width, height).
    """

    def __init__(self, spec: str, timeout_s: float = 5.0):
        self.spec = spec
        self.timeout_s = timeout_s
        self.is_live = spec.startswith(("webcam", "robot", "realsense"))
        self.intrinsics: Optional[dict] = None

    def __iter__(self) -> Iterator[Tuple[str, np.ndarray, Optional[np.ndarray]]]:
        s = self.spec
        if s.startswith("realsense") or s.lower().endswith(RS_EXT):
            yield from self._realsense()
        elif s.startswith("robot-depth"):
            yield from self._zmq(55570, "Is run_depth_server.sh running? (README step 6)", self._depth_msg)
        elif s.startswith("robot"):
            yield from self._zmq(55555, "Is teleimager-server running on the robot? (README step 5)",
                                 lambda buf: (decode_jpeg(buf), None))
        elif s.startswith("webcam"):
            idx = int(s.split(":")[1]) if ":" in s else 0
            yield from self._video(idx)
        elif os.path.isdir(s):
            files = sorted(f for f in glob.glob(os.path.join(s, "*")) if f.lower().endswith(IMG_EXT))
            if not files:
                raise FileNotFoundError(f"No images in {s}")
            for f in files:
                img = cv2.imread(f)
                if img is not None:
                    yield os.path.basename(f), img, None
        elif s.lower().endswith(IMG_EXT):
            img = cv2.imread(s)
            if img is None:
                raise FileNotFoundError(s)
            yield os.path.basename(s), img, None
        elif s.lower().endswith(VID_EXT):
            yield from self._video(s)
        else:
            raise ValueError(f"Don't know how to read source '{s}'")

    def _video(self, src):
        cap = cv2.VideoCapture(src)
        if not cap.isOpened():
            raise RuntimeError(f"Cannot open {src}")
        i = 0
        try:
            while True:
                ok, frame = cap.read()
                if not ok:
                    break
                yield f"frame_{i:05d}", frame, None
                i += 1
        finally:
            cap.release()

    def _depth_msg(self, buf: bytes):
        img, depth, intr = decode_depth_msg(buf)
        if intr:
            self.intrinsics = intr
        return img, depth

    def _zmq(self, default_port: int, hint: str, decode):
        import zmq  # only needed for the robot streams

        parts = self.spec.split(":")
        host = parts[1] if len(parts) > 1 and parts[1] else "192.168.123.164"
        port = int(parts[2]) if len(parts) > 2 else default_port
        ctx = zmq.Context.instance()
        sock = ctx.socket(zmq.SUB)
        sock.setsockopt(zmq.CONFLATE, 1)  # always the newest frame
        sock.setsockopt(zmq.RCVHWM, 1)
        sock.setsockopt(zmq.LINGER, 0)
        sock.connect(f"tcp://{host}:{port}")
        sock.setsockopt_string(zmq.SUBSCRIBE, "")
        poller = zmq.Poller()
        poller.register(sock, zmq.POLLIN)
        i = 0
        try:
            while True:
                if not dict(poller.poll(int(self.timeout_s * 1000))):
                    raise TimeoutError(
                        f"No frames from tcp://{host}:{port} in {self.timeout_s}s. {hint}")
                frame, depth = decode(sock.recv())
                if frame is None:
                    continue
                yield f"robot_{i:05d}", frame, depth
                i += 1
        finally:
            sock.close()

    def _realsense(self, width: int = 1280, height: int = 720, fps: int = 30):
        """RealSense colour + depth aligned to colour (same settings as teleimager)."""
        try:
            import pyrealsense2 as rs
        except ImportError:
            raise RuntimeError("pyrealsense2 not installed: pip install pyrealsense2")
        pipe, cfg = rs.pipeline(), rs.config()
        if self.spec.lower().endswith(RS_EXT):
            cfg.enable_device_from_file(self.spec, repeat_playback=False)
        else:
            if ":" in self.spec:
                cfg.enable_device(self.spec.split(":", 1)[1])
            cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
            cfg.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        profile = pipe.start(cfg)
        if not self.is_live:  # bag: deliver every frame, don't drop to keep real time
            profile.get_device().as_playback().set_real_time(False)
        scale = profile.get_device().first_depth_sensor().get_depth_scale()
        ci = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.intrinsics = dict(fx=ci.fx, fy=ci.fy, cx=ci.ppx, cy=ci.ppy, width=ci.width, height=ci.height)
        align = rs.align(rs.stream.color)
        i = 0
        try:
            while True:
                ok, frames = pipe.try_wait_for_frames(int(self.timeout_s * 1000))
                if not ok:
                    if self.is_live:
                        raise TimeoutError(f"No frames from RealSense in {self.timeout_s}s")
                    break  # end of bag
                frames = align.process(frames)
                color, depth = frames.get_color_frame(), frames.get_depth_frame()
                if not color:
                    continue
                img = np.asanyarray(color.get_data())
                if color.get_profile().format() == rs.format.rgb8:  # bags often store rgb8
                    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                z = np.asanyarray(depth.get_data()).astype(np.float32) * scale if depth else None
                yield f"rs_{i:05d}", img, z
                i += 1
        finally:
            pipe.stop()


def decode_depth_msg(buf: bytes) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[dict]]:
    """robot_depth_server.py message -> (bgr, depth metres float32, colour intrinsics).
    Format: b"OKRD" | uint32 LE header_len | header JSON | JPEG | 16-bit PNG."""
    if buf[:4] != b"OKRD" or len(buf) < 8:
        return None, None, None
    hlen = int.from_bytes(buf[4:8], "little")
    h = json.loads(buf[8:8 + hlen])
    a = 8 + hlen
    b = a + h["jpeg_len"]
    img = cv2.imdecode(np.frombuffer(buf[a:b], np.uint8), cv2.IMREAD_COLOR)
    raw = cv2.imdecode(np.frombuffer(buf[b:b + h["depth_len"]], np.uint8), cv2.IMREAD_UNCHANGED)
    depth = raw.astype(np.float32) * h["depth_scale"] if raw is not None else None
    return img, depth, h.get("intr")


def decode_jpeg(buf: bytes) -> Optional[np.ndarray]:
    """Decode JPEG bytes; tolerates a small header before the JPEG start marker."""
    start = buf.find(b"\xff\xd8")
    if start < 0:
        return None
    return cv2.imdecode(np.frombuffer(buf[start:], np.uint8), cv2.IMREAD_COLOR)


# --------------------------------------------------------------------------- #
# Detection
# --------------------------------------------------------------------------- #
@dataclass
class Okra:
    conf: float
    box: Tuple[float, float, float, float]      # x1,y1,x2,y2 px
    center: Tuple[float, float]                  # mask centroid px (u,v)
    length_px: float                             # long side of min-area rect
    width_px: float
    angle_deg: float                             # orientation of long axis in image
    area_px: int
    polygon: np.ndarray = field(repr=False, default=None)
    xyz: Optional[Tuple[float, float, float]] = None   # camera frame, metres
    depth_method: str = ""
    length_m: Optional[float] = None             # metric size from depth (None without depth)
    width_m: Optional[float] = None
    plausible: Optional[bool] = None             # okra-sized? (None = can't tell without depth)

    def to_dict(self):
        d = asdict(self)
        d.pop("polygon", None)
        return d


def download_model() -> str:
    local = os.path.join(HERE, "models", "okra11n-seg-v5.pt")
    if os.path.exists(local):
        return local
    from huggingface_hub import hf_hub_download
    path = hf_hub_download(repo_id=MODEL_REPO, filename=MODEL_FILE)
    os.makedirs(os.path.dirname(local), exist_ok=True)
    import shutil
    shutil.copy(path, local)
    return local


class OkraDetector:
    """YOLO okra segmentation. `tile` > 0 adds tiled inference: overlapping tile x tile px crops,
    each run at `tile_imgsz`. On the G1 head camera the model only finds a pod when it fills a
    good part of its input (2026-09-26: a pod ~93 px long at ~1 m was missed on the full frame,
    found at conf 0.45-0.64 on a 320 px crop at imgsz 960-1280). 12 tiles of 400 px @1280 + full frame:
    ~4.6 s/frame on CPU, ~1.4 s on the laptop GPU (device None = GPU if torch sees one)."""

    def __init__(self, weights: Optional[str] = None, conf: float = 0.35,
                 imgsz: int = 640, device: Optional[str] = None,
                 tile: int = 0, tile_imgsz: int = 960, tile_overlap: float = 0.25):
        from ultralytics import YOLO
        self.model = YOLO(weights or download_model())
        self.conf = conf
        self.imgsz = imgsz
        self.device = device
        self.tile, self.tile_imgsz, self.tile_overlap = tile, tile_imgsz, tile_overlap

    def _run(self, img: np.ndarray, imgsz: int, off=(0, 0)) -> List[Okra]:
        r = self.model.predict(img, imgsz=imgsz, conf=self.conf,
                               device=self.device, verbose=False)[0]
        return self._parse(r, off)

    def _parse(self, r, off=(0, 0)) -> List[Okra]:
        out: List[Okra] = []
        if r.boxes is None or len(r.boxes) == 0:
            return out
        boxes = r.boxes.xyxy.cpu().numpy() + np.array([off[0], off[1], off[0], off[1]])
        confs = r.boxes.conf.cpu().numpy()
        polys = r.masks.xy if r.masks is not None else [None] * len(boxes)
        for box, c, poly in zip(boxes, confs, polys):
            if poly is not None and len(poly):
                poly = np.asarray(poly, np.float32) + np.array(off, np.float32)
            out.append(_okra_from(box, float(c), poly))
        return out

    def tiles(self, h: int, w: int) -> List[Tuple[int, int]]:
        t = self.tile
        step = max(1, int(t * (1.0 - self.tile_overlap)))
        xs = list(range(0, max(1, w - t) + 1, step))
        ys = list(range(0, max(1, h - t) + 1, step))
        if xs[-1] + t < w:
            xs.append(w - t)
        if ys[-1] + t < h:
            ys.append(h - t)
        return [(x, y) for y in ys for x in xs]

    def detect(self, img: np.ndarray) -> List[Okra]:
        out = self._run(img, self.imgsz)
        if self.tile > 0:
            h, w = img.shape[:2]
            # One call per tile. Batching all tiles gave no gain on the GPU (1.47 vs 1.42 s/frame)
            # and was 3x slower on the CPU (14 s vs 4.6 s), 2026-09-26.
            for x, y in self.tiles(h, w):
                out += self._run(img[y:y + self.tile, x:x + self.tile], self.tile_imgsz, (x, y))
            out = _merge(out)
        out.sort(key=lambda o: -o.conf)
        return out


def _iou(a, b) -> float:
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0 else 0.0


def _merge(dets: List[Okra], iou: float = 0.3) -> List[Okra]:
    """Greedy NMS across tiles; a detection whose centre lies inside a kept, higher-confidence
    box of similar size also counts as a duplicate (pods cut at tile borders)."""
    keep: List[Okra] = []
    for o in sorted(dets, key=lambda o: -o.conf):
        dup = False
        for k in keep:
            same_size = 0.5 <= o.length_px / max(1.0, k.length_px) <= 2.0
            inside = k.box[0] <= o.center[0] <= k.box[2] and k.box[1] <= o.center[1] <= k.box[3]
            if _iou(o.box, k.box) > iou or (inside and same_size):
                dup = True
                break
        if not dup:
            keep.append(o)
    return keep


def _okra_from(box, conf, poly) -> Okra:
    x1, y1, x2, y2 = [float(v) for v in box]
    if poly is not None and len(poly) >= 3:
        pts = np.asarray(poly, dtype=np.float32)
        m = cv2.moments(pts)
        if m["m00"] > 1e-3:
            cx, cy = m["m10"] / m["m00"], m["m01"] / m["m00"]
        else:
            cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
        (_, _), (rw, rh), ang = cv2.minAreaRect(pts)
        area = int(abs(cv2.contourArea(pts)))
    else:
        pts = None
        cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
        rw, rh, ang = x2 - x1, y2 - y1, 0.0
        area = int(rw * rh)
    if rw >= rh:
        length, width, angle = rw, rh, ang
    else:
        length, width, angle = rh, rw, ang + 90.0
    return Okra(conf, (x1, y1, x2, y2), (cx, cy), float(length), float(width),
                float(angle), area, pts)


# --------------------------------------------------------------------------- #
# Camera model + depth
# --------------------------------------------------------------------------- #
@dataclass
class CameraCalib:
    """Pinhole model of the colour camera. Defaults = RealSense D435i colour at
    1280x720 (HFOV ~69 deg) until real intrinsics are known: a RealSense source
    supplies them (set_intrinsics), or set fx/fy/cx/cy in camera_calib.json."""
    width: int = 1280
    height: int = 720
    hfov_deg: float = 69.4            # D435(i) colour sensor spec
    fx: Optional[float] = None
    fy: Optional[float] = None
    cx: Optional[float] = None
    cy: Optional[float] = None
    okra_length_m: float = 0.10       # for monocular size-based depth

    def f(self) -> float:
        if self.fx:
            return self.fx
        return (self.width / 2.0) / np.tan(np.radians(self.hfov_deg) / 2.0)

    def pp(self) -> Tuple[float, float]:
        return (self.cx if self.cx is not None else self.width / 2.0,
                self.cy if self.cy is not None else self.height / 2.0)

    def backproject(self, u: float, v: float, z: float) -> Tuple[float, float, float]:
        fx = self.f()
        fy = self.fy or fx
        cx, cy = self.pp()
        return ((u - cx) * z / fx, (v - cy) * z / fy, z)

    def set_intrinsics(self, intr: dict):
        for k in ("fx", "fy", "cx", "cy", "width", "height"):
            setattr(self, k, intr[k])

    @staticmethod
    def load(path: str = CALIB_PATH) -> "CameraCalib":
        if os.path.exists(path):
            with open(path) as fh:
                d = json.load(fh)
            known = CameraCalib.__dataclass_fields__
            return CameraCalib(**{k: v for k, v in d.items() if k in known})
        return CameraCalib()

    def save(self, path: str = CALIB_PATH):
        with open(path, "w") as fh:
            json.dump(asdict(self), fh, indent=2)


def okra_mask(o: Okra, shape: Tuple[int, int], erode_px: int = 3) -> np.ndarray:
    """Filled segmentation mask (or box), eroded so edge pixels, whose depth
    mixes okra and background, are left out."""
    m = np.zeros(shape[:2], np.uint8)
    if o.polygon is not None:
        cv2.fillPoly(m, [o.polygon.astype(np.int32)], 1)
    else:
        x1, y1, x2, y2 = [int(round(v)) for v in o.box]
        m[max(0, y1):y2, max(0, x1):x2] = 1
    if erode_px > 0:
        eroded = cv2.erode(m, np.ones((2 * erode_px + 1,) * 2, np.uint8))
        if eroded.sum() >= 10:  # thin/far okra: keep the un-eroded mask
            m = eroded
    return m.astype(bool)


def locate_depth(o: Okra, depth: np.ndarray, cal: CameraCalib,
                 min_valid: int = 10, z_range: Tuple[float, float] = (0.1, 4.0)) -> bool:
    """Set o.xyz from the median depth inside the okra mask. Returns False (and
    leaves o untouched) when too few valid depth pixels, e.g. too close for the
    D435 (< ~0.2 m) or a hole in the depth map."""
    z = depth[okra_mask(o, depth.shape)]
    z = z[(z > z_range[0]) & (z < z_range[1])]
    if z.size < min_valid:
        return False
    if o.polygon is not None:
        zm = float(np.median(z))
    else:  # box only: mostly background behind a diagonal pod -> take the near end
        zm = float(np.percentile(z, 10))
    o.xyz = cal.backproject(o.center[0], o.center[1], zm)
    o.depth_method = f"depth n={z.size}" + ("" if o.polygon is not None else " (box)")
    # Metric size in the image plane (a pod tilted towards the camera looks shorter, never longer)
    o.length_m = o.length_px * zm / cal.f()
    o.width_m = pod_width_px(o) * zm / cal.f()
    o.plausible = okra_sized(o)
    return True


# Okra pods: ~5-20 cm long, 1-4 cm wide. Generous bounds: the point is to reject things like a
# 1.8 m "okra" (green carpet, 2026-09-26), not to judge borderline pods.
OKRA_LENGTH_M = (0.03, 0.35)
OKRA_WIDTH_M = (0.004, 0.08)
# Pods are long and thin (real pod on the G1 camera: 87x20 px = 4.4:1); the tiled false positives
# on jeans/carpet/floor were blobs of 1:1 to 2.5:1. Works without depth too.
OKRA_MIN_ELONGATION = 3.0


def pod_width_px(o: Okra) -> float:
    """Mean thickness = mask area / length. A curved pod's bounding rectangle is much wider than the
    pod (2026-09-27: hanging pod 119x50 px rect = 2.4:1, but area/length = 18 px -> 6.6:1)."""
    if o.polygon is not None and o.area_px > 0:
        return min(o.width_px, o.area_px / max(1.0, o.length_px))
    return o.width_px


def okra_sized(o: Okra) -> Optional[bool]:
    """False = not a pod (wrong shape, or wrong metric size when depth is known);
    None = shape OK but no depth to check the size; True = both OK."""
    if o.length_px < OKRA_MIN_ELONGATION * max(1.0, pod_width_px(o)):
        return False
    if o.length_m is None:
        return None
    return (OKRA_LENGTH_M[0] <= o.length_m <= OKRA_LENGTH_M[1]
            and OKRA_WIDTH_M[0] <= o.width_m <= OKRA_WIDTH_M[1])


def locate(o: Okra, depth: Optional[np.ndarray], cal: CameraCalib) -> Okra:
    """Depth if available and valid, else the size prior."""
    if depth is not None and locate_depth(o, depth, cal):
        return o
    locate_size(o, cal)
    if depth is not None and o.depth_method:
        o.depth_method = "size-prior (no valid depth)"
    return o


def locate_size(o: Okra, cal: CameraCalib) -> Okra:
    if o.length_px > 5:
        z = cal.f() * cal.okra_length_m / o.length_px
        o.xyz = cal.backproject(o.center[0], o.center[1], z)
        o.depth_method = "size-prior"
    return o


# --------------------------------------------------------------------------- #
# Drawing
# --------------------------------------------------------------------------- #
def draw(img: np.ndarray, okras: List[Okra]) -> np.ndarray:
    vis = img.copy()
    overlay = vis.copy()
    for o in okras:
        if o.polygon is not None:
            cv2.fillPoly(overlay, [o.polygon.astype(np.int32)], (0, 200, 0))
    vis = cv2.addWeighted(overlay, 0.35, vis, 0.65, 0)
    for k, o in enumerate(okras):
        x1, y1, x2, y2 = [int(v) for v in o.box]
        cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cx, cy = int(o.center[0]), int(o.center[1])
        cv2.drawMarker(vis, (cx, cy), (0, 0, 255), cv2.MARKER_CROSS, 14, 2)
        label = f"#{k} okra {o.conf:.2f}"
        if o.xyz is not None:
            label += f" z={o.xyz[2]:.2f}m"
        cv2.putText(vis, label, (x1, max(15, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(vis, label, (x1, max(15, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (255, 255, 255), 1, cv2.LINE_AA)
    return vis


class FPS:
    def __init__(self):
        self.t, self.v = time.time(), 0.0

    def tick(self) -> float:
        now = time.time()
        dt, self.t = now - self.t, now
        self.v = 0.9 * self.v + 0.1 * (1.0 / dt if dt > 0 else 0)
        return self.v

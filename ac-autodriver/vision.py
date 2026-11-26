"""
vision.py - fast forward perception from the screen (mss + OpenCV).

Pipeline per frame (on a ~480 px wide ROI, a few ms):
    grab ROI -> downscale
    lanes:     white/yellow mask & Canny -> HoughLinesP -> nearest line each side
               -> lane offset / heading / confidence (smoothed, survives one-sided loss)
    obstacles: pixels that differ from the road colour sampled just ahead of the hood
               -> morphology (removes thin lane paint) -> contours -> edge-density filter
               -> distance from the blob's bottom row (perspective model)
               -> zone (left/center/right) using the detected lane lines at that row,
                  falling back to fixed ROI splits

Obstacle interface: anything with poll() -> list[Obstacle] is an ObstacleSource.
VisionThread is one; a future CSP Lua UDP feed just needs to emit Obstacle objects,
and summarize() folds any mix of sources into per-zone nearest distances.

    python vision.py --selftest           # synthetic road scenes, no screen needed
    python vision.py --calibrate          # screenshot with ROI/zone overlay -> calibration.png
    python vision.py --image shot.png     # run the pipeline on a saved full-screen shot
    python vision.py --live               # live capture + debug window + stats
"""

from __future__ import annotations

import argparse
import math
import sys
import threading
import time
from dataclasses import dataclass, replace
from typing import Iterable, Protocol

import cv2
import numpy as np

from config import CONFIG, VisionConfig

ZONES = ("left", "center", "right")


# --------------------------------------------------------------------------- #
# Data types
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class Obstacle:
    zone: str                                    # "left" | "center" | "right"
    distance_m: float                            # estimated forward distance
    source: str = "vision"                       # "vision", "csp", ...
    bbox: tuple[int, int, int, int] | None = None  # x, y, w, h in processed-ROI px (vision)
    lateral_m: float | None = None               # for position-based feeds (CSP)


class ObstacleSource(Protocol):
    def poll(self) -> list[Obstacle]: ...


@dataclass
class ZoneState:
    nearest_m: float = math.inf
    count: int = 0

    def clear_for(self, distance_m: float) -> bool:
        return self.nearest_m > distance_m


def summarize(obstacles: Iterable[Obstacle]) -> dict[str, ZoneState]:
    zones = {z: ZoneState() for z in ZONES}
    for o in obstacles:
        z = zones[o.zone]
        z.count += 1
        z.nearest_m = min(z.nearest_m, o.distance_m)
    return zones


Line = tuple[float, float]  # (x at bottom row, dx/dy): x(y) = xb + a * (y - h)


@dataclass
class LaneEstimate:
    offset: float | None = None   # car vs lane centre in half-lane widths: <0 car left of centre
    heading: float | None = None  # >0 lane runs off to the right ahead (dx/dy, unitless)
    confidence: float = 0.0       # 0..1 (1 = both lines seen this frame)
    left: Line | None = None
    right: Line | None = None


@dataclass
class Perception:
    t: float
    lanes: LaneEstimate
    obstacles: list[Obstacle]
    zones: dict[str, ZoneState]
    shape: tuple[int, int]        # processed ROI (h, w)
    process_ms: float = 0.0


def _x_at(line: Line, y: float, h: int) -> float:
    return line[0] + line[1] * (y - h)


# --------------------------------------------------------------------------- #
# Lane detection
# --------------------------------------------------------------------------- #
class LaneDetector:
    SMOOTH = 0.5     # EMA weight of a new measurement
    DECAY = 0.85     # confidence multiplier per frame with no lines
    MIN_CONF = 0.15  # below this, offset/heading are reported as None

    def __init__(self, cfg: VisionConfig) -> None:
        self.cfg = cfg
        self.width: float | None = None   # remembered lane width at the bottom row (px)
        self._est = LaneEstimate()

    def detect(self, gray: np.ndarray, hsv: np.ndarray) -> LaneEstimate:
        c, (h, w) = self.cfg, gray.shape
        cx = c.car_center_x * w
        paint = (gray > c.white_thresh) | (cv2.inRange(hsv, c.yellow_hsv_lo, c.yellow_hsv_hi) > 0)
        paint = cv2.dilate(paint.astype(np.uint8), np.ones((3, 3), np.uint8))
        edges = cv2.Canny(cv2.GaussianBlur(gray, (5, 5), 0), c.canny_lo, c.canny_hi) & (paint * 255)
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, c.hough_threshold,
                                minLineLength=c.hough_min_len, maxLineGap=c.hough_max_gap)

        left_c, right_c = [], []
        # (N,1,4) in OpenCV 4.x, (N,4) in 5.x
        for x1, y1, x2, y2 in ([] if lines is None else lines.reshape(-1, 4).astype(float)):
            if y1 == y2:
                continue
            a = (x2 - x1) / (y2 - y1)
            if abs(a) > 2.5:                          # near-horizontal: not a lane line
                continue
            seg = (x1 + a * (h - y1), a, math.hypot(x2 - x1, y2 - y1))
            if seg[0] < cx and a < 0.3:
                left_c.append(seg)
            elif seg[0] >= cx and a > -0.3:
                right_c.append(seg)
        left, right = self._nearest(left_c, cx, w), self._nearest(right_c, cx, w)
        return self._update(left, right, cx, w)

    def _nearest(self, segs: list, cx: float, w: int) -> Line | None:
        """Length-weighted line of the segment cluster closest to the car (needs real support)."""
        segs = sorted(segs, key=lambda s: abs(s[0] - cx))
        tol, min_len = 0.06 * w, 1.5 * self.cfg.hough_min_len
        while segs:
            grp = [s for s in segs if abs(s[0] - segs[0][0]) <= tol]
            total = sum(s[2] for s in grp)
            if total >= min_len:
                return (float(sum(s[0] * s[2] for s in grp) / total),
                        float(sum(s[1] * s[2] for s in grp) / total))
            segs = segs[1:]  # lone fragment nearest the car - drop it and try the next
        return None

    def _update(self, left: Line | None, right: Line | None, cx: float, w: int) -> LaneEstimate:
        e = self._est
        if left and right and 0.15 * w < right[0] - left[0] < 2.0 * w:
            width = right[0] - left[0]
            self.width = width if self.width is None else self.width + 0.2 * (width - self.width)
            conf = 1.0
        elif (left or right) and self.width:
            # one line visible: place the other at the remembered width, mirrored slope
            if left:
                right = (left[0] + self.width, e.right[1] if e.right else -left[1])
            else:
                left = (right[0] - self.width, e.left[1] if e.left else -right[1])
            conf = 0.6
        else:
            left = right = None
            conf = 0.0

        if left and right:
            half = (right[0] - left[0]) / 2
            off = max(-1.5, min(1.5, (cx - (left[0] + right[0]) / 2) / half))
            head = -(left[1] + right[1]) / 2
            k = self.SMOOTH
            e.offset = off if e.offset is None else e.offset + k * (off - e.offset)
            e.heading = head if e.heading is None else e.heading + k * (head - e.heading)
            e.left, e.right, e.confidence = left, right, conf
        else:
            e.confidence *= self.DECAY
            if e.confidence < self.MIN_CONF:
                e.offset = e.heading = e.left = e.right = None
        return replace(e)


# --------------------------------------------------------------------------- #
# Obstacle detection
# --------------------------------------------------------------------------- #
class ObstacleDetector:
    def __init__(self, cfg: VisionConfig) -> None:
        self.cfg = cfg

    def distance(self, y_bottom: float, h: int) -> float:
        """Flat-road perspective d = k / (y - y0), pinned at near (y = h) and far (y = 0)."""
        dn, df = self.cfg.near_distance_m, self.cfg.far_distance_m
        y0 = dn * h / (dn - df)          # horizon row (above the ROI, so negative)
        return max(dn, min(df, -df * y0 / max(y_bottom - y0, 1e-6)))

    def zone_bounds(self, y: float, lanes: LaneEstimate, w: int, h: int) -> tuple[float, float]:
        """x where the center zone starts/ends at row y."""
        if lanes.left and lanes.right and lanes.confidence >= 0.3:
            return _x_at(lanes.left, y, h), _x_at(lanes.right, y, h)
        a, b = self.cfg.zone_split
        return a * w, b * w

    def detect(self, gray: np.ndarray, hsv: np.ndarray, edges: np.ndarray,
               lanes: LaneEstimate) -> list[Obstacle]:
        c, (h, w) = self.cfg, gray.shape
        cx = int(c.car_center_x * w)
        # road reference: just ahead of the hood, centre of the lane (median is robust to paint)
        patch = hsv[int(h * 0.85):, max(0, cx - w // 10): cx + w // 10]
        road_v, road_s = float(np.median(patch[..., 2])), float(np.median(patch[..., 1]))

        v, s = hsv[..., 2].astype(np.int16), hsv[..., 1].astype(np.int16)
        mask = ((np.abs(v - road_v) > c.obstacle_value_delta)
                | (s - road_s > c.obstacle_sat_delta)).astype(np.uint8)
        k = max(5, (w // 60) | 1)          # wider than lane paint, narrower than a distant car
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((k, k), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((k + 4, k + 4), np.uint8))

        found: list[Obstacle] = []
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            x, y, bw, bh = cv2.boundingRect(cnt)
            if bw * bh < c.obstacle_min_area_frac * w * h or bw > 0.9 * w:
                continue
            if not 0.3 <= bw / bh <= 6.0:
                continue
            # sides lying on the ROI border can't show an edge, so leave them out
            perim = (bw * ((y > 0) + (y + bh < h)) + bh * ((x > 0) + (x + bw < w))) or 1
            if np.count_nonzero(edges[y:y + bh, x:x + bw]) / perim < c.obstacle_edge_score:
                continue  # smooth blob: shadow, tar patch, sky
            yb = y + bh
            dist = self.distance(yb, h)
            lo, hi = self.zone_bounds(yb, lanes, w, h)
            # report in every zone the blob covers >= 30 % of its width (straddling trucks)
            for zone, (z0, z1) in zip(ZONES, ((-math.inf, lo), (lo, hi), (hi, math.inf))):
                if max(0.0, min(x + bw, z1) - max(x, z0)) >= 0.3 * bw:
                    found.append(Obstacle(zone, dist, bbox=(x, y, bw, bh)))
        return found


# --------------------------------------------------------------------------- #
# Frame processor + overlay
# --------------------------------------------------------------------------- #
class VisionProcessor:
    def __init__(self, cfg: VisionConfig | None = None) -> None:
        self.cfg = cfg or CONFIG.vision
        self.lanes = LaneDetector(self.cfg)
        self.obstacles = ObstacleDetector(self.cfg)

    def prepare(self, roi_bgr: np.ndarray) -> np.ndarray:
        h, w = roi_bgr.shape[:2]
        if w != self.cfg.downscale_width:
            nw = self.cfg.downscale_width
            roi_bgr = cv2.resize(roi_bgr, (nw, max(1, round(h * nw / w))), interpolation=cv2.INTER_AREA)
        return roi_bgr

    def process(self, roi_bgr: np.ndarray, t: float | None = None) -> tuple[Perception, np.ndarray]:
        t0 = time.perf_counter()
        img = self.prepare(roi_bgr)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lanes = self.lanes.detect(gray, hsv)
        edges = cv2.Canny(gray, self.cfg.canny_lo, self.cfg.canny_hi)
        obs = self.obstacles.detect(gray, hsv, edges, lanes)
        p = Perception(t=t0 if t is None else t, lanes=lanes, obstacles=obs, zones=summarize(obs),
                       shape=gray.shape, process_ms=(time.perf_counter() - t0) * 1000)
        return p, img


def draw_overlay(img: np.ndarray, p: Perception, cfg: VisionConfig) -> np.ndarray:
    out, (h, w) = img.copy(), p.shape
    det = ObstacleDetector(cfg)
    for y in range(0, h, 4):                               # zone boundaries
        for x in det.zone_bounds(y, p.lanes, w, h):
            cv2.circle(out, (int(x), y), 1, (255, 200, 0), -1)
    for line in (p.lanes.left, p.lanes.right):
        if line:
            cv2.line(out, (int(_x_at(line, h, h)), h), (int(_x_at(line, 0, h)), 0), (0, 255, 0), 2)
    for o in p.obstacles:
        x, y, bw, bh = o.bbox
        cv2.rectangle(out, (x, y), (x + bw, y + bh), (0, 0, 255), 2)
        cv2.putText(out, f"{o.zone[0].upper()} {o.distance_m:.0f}m", (x + 2, y - 3 if y > 12 else y + 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
    cx = int(cfg.car_center_x * w)
    cv2.line(out, (cx, h - 12), (cx, h), (255, 255, 255), 2)
    off = "--" if p.lanes.offset is None else f"{p.lanes.offset:+.2f}"
    cv2.putText(out, f"off {off} conf {p.lanes.confidence:.2f} {p.process_ms:.1f}ms",
                (4, 12), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
    return out


# --------------------------------------------------------------------------- #
# Screen capture + background thread
# --------------------------------------------------------------------------- #
def roi_rect(monitor: dict, cfg: VisionConfig) -> dict:
    """Pixel rect of the ROI on screen, from monitor geometry + fractional config."""
    l, t, w, h = monitor["left"], monitor["top"], monitor["width"], monitor["height"]
    if cfg.capture_region:
        x0, y0, x1, y1 = cfg.capture_region
        l, t, w, h = l + x0 * w, t + y0 * h, (x1 - x0) * w, (y1 - y0) * h
    x0, y0, x1, y1 = cfg.roi
    return {"left": int(l + x0 * w), "top": int(t + y0 * h),
            "width": max(1, int((x1 - x0) * w)), "height": max(1, int((y1 - y0) * h))}


class ScreenCapture:
    """mss (GDI) grabber for just the ROI: works everywhere, ~5-15 ms a grab, and may show a
    black / frozen image of a game in exclusive fullscreen. Create it on the thread that uses it."""
    name = "mss"

    def __init__(self, cfg: VisionConfig) -> None:
        import mss
        self._sct = mss.mss()
        self.monitor = self._sct.monitors[cfg.monitor_index]
        self.rect = roi_rect(self.monitor, cfg)

    def grab(self, rect: dict | None = None) -> np.ndarray:
        # convert the whole BGRA buffer: a [..., :3] view is strided, which OpenCV dislikes
        return cv2.cvtColor(np.asarray(self._sct.grab(rect or self.rect)), cv2.COLOR_BGRA2BGR)

    def grab_full(self) -> np.ndarray:
        return self.grab(self.monitor)

    def close(self) -> None:
        self._sct.close()


class DXCamCapture:
    """DXGI Desktop Duplication (dxcam): frames straight from the GPU, ~1-2 ms a grab, 100+ fps,
    and sees games that GDI can't. grab() returns None when the screen hasn't changed since
    the last grab, so we never re-process an identical frame. Windows only."""
    name = "dxcam"

    def __init__(self, cfg: VisionConfig) -> None:
        if sys.platform != "win32":
            raise RuntimeError("dxcam is Windows-only")
        import dxcam
        cam = dxcam.create(output_idx=max(cfg.monitor_index - 1, 0), output_color="BGR")
        if cam is None:
            raise RuntimeError(f"no DXGI output for monitor {cfg.monitor_index}")
        self._cam = cam
        self.monitor = {"left": 0, "top": 0, "width": cam.width, "height": cam.height}
        self.rect = roi_rect(self.monitor, cfg)

    def _region(self, r: dict) -> tuple[int, int, int, int]:
        w, h = self.monitor["width"], self.monitor["height"]
        x0, y0 = min(max(r["left"], 0), w - 1), min(max(r["top"], 0), h - 1)
        return x0, y0, min(x0 + r["width"], w), min(y0 + r["height"], h)

    def grab(self, rect: dict | None = None) -> np.ndarray | None:
        return self._cam.grab(region=self._region(rect or self.rect))

    def grab_full(self) -> np.ndarray | None:
        return self.grab(self.monitor)

    def close(self) -> None:
        try:
            self._cam.release()
        except Exception:
            pass


def make_capture(cfg: VisionConfig):
    """Fastest capture available: dxcam, else mss. The chosen one's .note says why."""
    if cfg.capture_backend in ("auto", "dxcam"):
        try:
            cap = DXCamCapture(cfg)
            cap.note = "GPU desktop duplication"
            return cap
        except Exception as e:
            if cfg.capture_backend == "dxcam":
                raise
            reason = f"dxcam unavailable ({type(e).__name__}: {e})"
    else:
        reason = "chosen in config"
    cap = ScreenCapture(cfg)
    cap.note = reason
    return cap


class VisionThread:
    """Captures + processes continuously; the control loop reads latest() / poll()."""

    def __init__(self, cfg: VisionConfig | None = None, capture_factory=make_capture) -> None:
        self.cfg = cfg or CONFIG.vision
        self._factory = capture_factory
        self._lock = threading.Lock()
        self._latest: tuple[Perception, np.ndarray] | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.error: BaseException | None = None
        self.fps = 0.0
        self.grab_ms = self.proc_ms = 0.0     # smoothed timings, for the HUD
        self.backend = "-"

    def start(self) -> "VisionThread":
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="vision", daemon=True)
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2)

    def _run(self) -> None:
        try:
            cap = self._factory(self.cfg)
        except BaseException as e:      # e.g. bad monitor index
            self.error = e
            return
        self.backend = f"{getattr(cap, 'name', '?')} ({getattr(cap, 'note', '')})"
        proc, period = VisionProcessor(self.cfg), 1.0 / max(self.cfg.max_fps, 1.0)
        n, t_fps = 0, time.perf_counter()
        try:
            while not self._stop.is_set():
                t0 = time.perf_counter()
                try:
                    frame = cap.grab()
                    if frame is None:               # screen unchanged since last grab: wait for a new one
                        self._stop.wait(0.001)
                        continue
                    t1 = time.perf_counter()
                    result = proc.process(frame, t=t0)
                    self.error = None
                except Exception as e:  # keep running; latest() goes stale and planner reacts
                    self.error = e
                    time.sleep(0.1)
                    continue
                self.grab_ms += 0.1 * ((t1 - t0) * 1000 - self.grab_ms)
                self.proc_ms += 0.1 * (result[0].process_ms - self.proc_ms)
                with self._lock:
                    self._latest = result
                n += 1
                if t0 - t_fps >= 1.0:
                    self.fps, n, t_fps = n / (t0 - t_fps), 0, t0
                if self.cfg.debug_window:
                    cv2.imshow("ac-autodriver vision", draw_overlay(result[1], result[0], self.cfg))
                    cv2.waitKey(1)
                self._stop.wait(max(0.0, period - (time.perf_counter() - t0)))
        finally:
            cap.close()
            if self.cfg.debug_window:
                cv2.destroyAllWindows()

    def latest(self, max_age_s: float = 0.2) -> Perception | None:
        """Most recent perception, or None if it is older than max_age_s (vision stalled)."""
        with self._lock:
            r = self._latest
        return r[0] if r and time.perf_counter() - r[0].t <= max_age_s else None

    def frame(self) -> np.ndarray | None:
        with self._lock:
            return None if self._latest is None else self._latest[1]

    def poll(self) -> list[Obstacle]:
        p = self.latest()
        return p.obstacles if p else []


# --------------------------------------------------------------------------- #
# Synthetic scenes + self-test
# --------------------------------------------------------------------------- #
def synth_road(w: int = 480, h: int = 220, shift: float = 0.0, cars: Iterable = (),
               shadow: bool = False, seed: int = 0) -> np.ndarray:
    """Perspective 3-lane highway. shift: lane centre offset at the bottom (px, + = right).
    cars: (lane -1/0/+1, y_bottom) tuples. Returns BGR."""
    rng = np.random.default_rng(seed)
    img = np.clip(rng.normal(92, 6, (h, w, 3)), 0, 255).astype(np.uint8)
    vx, vy, half = w / 2 + shift * 0.3, -180.0, 0.36 * w          # vanishing point, half-lane px
    xb = lambda k: w / 2 + shift + k * half                       # lane line k at bottom row
    x_at = lambda k, y: vx + (xb(k) - vx) * (y - vy) / (h - vy)
    if shadow:
        ov = img.copy()
        cv2.ellipse(ov, (int(w * 0.5), int(h * 0.45)), (70, 22), 0, 0, 360, (55, 55, 55), -1)
        img = cv2.GaussianBlur(cv2.addWeighted(ov, 1, img, 0, 0), (41, 41), 0)
    for k in (-3, -1, 1, 3):
        for y in range(0, h, 26):                                 # dashed lines
            y2 = min(h, y + 16)
            t = max(2, int(2 + 5 * y2 / h))
            cv2.line(img, (int(x_at(k, y)), y), (int(x_at(k, y2)), y2), (235, 235, 235), t)
    for lane, yb in cars:
        cxl = (x_at(2 * lane - 1, yb) + x_at(2 * lane + 1, yb)) / 2
        cw = 0.75 * (x_at(1, yb) - x_at(-1, yb))
        ch = 0.6 * cw
        x0, y0 = int(cxl - cw / 2), int(yb - ch)
        cv2.rectangle(img, (x0, y0), (int(x0 + cw), int(yb)), (40, 30, 30), -1)        # body
        cv2.rectangle(img, (int(x0 + cw * .15), int(y0 + ch * .1)),
                      (int(x0 + cw * .85), int(y0 + ch * .45)), (120, 110, 100), -1)    # rear glass
        for lx in (x0 + cw * .05, x0 + cw * .75):                                       # tail lights
            cv2.rectangle(img, (int(lx), int(y0 + ch * .55)), (int(lx + cw * .2), int(y0 + ch * .7)),
                          (20, 20, 210), -1)
        cv2.rectangle(img, (int(x0 + cw * .35), int(y0 + ch * .75)),
                      (int(x0 + cw * .65), int(y0 + ch * .88)), (200, 200, 200), -1)    # plate
    return img


def selftest() -> int:
    fails: list[str] = []
    check = lambda ok, name: ok or fails.append(name)  # noqa: E731
    cfg = replace(CONFIG.vision, debug_window=False)

    def run(n: int = 6, **scene) -> Perception:
        proc = VisionProcessor(cfg)
        for i in range(n):
            p, _ = proc.process(synth_road(seed=i, **scene), t=float(i))
        return p

    p = run()
    check(p.lanes.confidence > 0.9 and abs(p.lanes.offset) < 0.1, f"centred: {p.lanes}")
    check(not p.obstacles, f"empty road false positives: {p.obstacles}")

    p = run(shift=40)        # lane centre right of car -> car is left of centre -> offset < 0
    exp = -40 / (0.36 * 480)
    check(p.lanes.offset is not None and abs(p.lanes.offset - exp) < 0.08,
          f"shift right: got {p.lanes.offset}, want ~{exp:.2f}")
    p = run(shift=-40)
    check(p.lanes.offset is not None and p.lanes.offset > 0.1, f"shift left: {p.lanes.offset}")

    far, near = run(cars=[(0, 90)]), run(cars=[(0, 180)])
    for name, q in (("far", far), ("near", near)):
        check([o.zone for o in q.obstacles] == ["center"], f"{name} center car zones: {q.obstacles}")
    if far.obstacles and near.obstacles:
        check(near.zones["center"].nearest_m < far.zones["center"].nearest_m, "distance ordering")
        check(cfg.near_distance_m <= near.zones["center"].nearest_m < 40, "near distance plausible")

    q = run(cars=[(-1, 120), (1, 150)])
    check(sorted(o.zone for o in q.obstacles) == ["left", "right"], f"side cars: {q.obstacles}")
    check(q.zones["center"].clear_for(1e9), "center should be clear")

    q = run(shadow=True)
    check(not q.obstacles, f"soft shadow false positive: {q.obstacles}")
    band = synth_road()
    band[60:100, 100:380] = 50  # hard-edged dark patch (bridge shadow / tarmac repair)
    q, _ = VisionProcessor(cfg).process(band, t=0.0)
    check(not q.obstacles, f"hard shadow false positive: {q.obstacles}")

    # background thread with a fake capture: produces, goes stale when frames stop, stops cleanly
    class FakeCap:
        def __init__(self, _cfg): self.n = 0
        def grab(self):
            self.n += 1
            if self.n > 8:
                raise RuntimeError("capture lost")
            return synth_road(cars=[(0, 150)], seed=self.n)
        def close(self): pass
    vt = VisionThread(replace(cfg, max_fps=200), capture_factory=FakeCap).start()
    deadline = time.perf_counter() + 2
    while vt.latest() is None and time.perf_counter() < deadline:
        time.sleep(0.01)
    got = vt.latest()
    check(got is not None and [o.zone for o in vt.poll()] == ["center"], "thread produces")
    time.sleep(0.35)
    check(vt.latest() is None and vt.poll() == [] and isinstance(vt.error, RuntimeError),
          "thread goes stale + reports error")
    vt.stop()
    check(not vt._thread.is_alive(), "thread stops")

    # one line lost for a few frames -> estimate survives on remembered width
    proc = VisionProcessor(cfg)
    for i in range(5):
        proc.process(synth_road(seed=i), t=float(i))
    img = synth_road(seed=9)
    img[:, 240:] = 92  # erase the right side
    p, _ = proc.process(img, t=9.0)
    check(p.lanes.offset is not None and abs(p.lanes.offset) < 0.15 and 0.5 <= p.lanes.confidence < 1,
          f"one-sided: {p.lanes}")
    blank = np.full((220, 480, 3), 92, np.uint8)
    for i in range(20):
        p, _ = proc.process(blank, t=10.0 + i)
    check(p.lanes.offset is None and p.lanes.confidence < LaneDetector.MIN_CONF, "decays to unknown")

    # timing on a full-res 1920x1080 ROI crop (cfg.roi of a 1080p screen)
    big = cv2.resize(synth_road(cars=[(0, 120)]), (1152, 292))
    proc, times = VisionProcessor(cfg), []
    for i in range(30):
        times.append(proc.process(big, t=float(i))[0].process_ms)
    ms = sorted(times)[len(times) // 2]
    check(ms < 12, f"too slow: {ms:.1f} ms")

    # dxcam backend with a fake driver: region maths, None = no new frame, thread + release
    import types
    calls: list = []

    class FakeCam:
        width, height = 1920, 1080

        def __init__(self):
            self.n = 0

        def grab(self, region):
            calls.append(region)
            self.n += 1
            if self.n % 2 == 0:
                return None                      # screen unchanged
            x0, y0, x1, y1 = region
            return cv2.resize(synth_road(cars=[(0, 150)], seed=self.n), (x1 - x0, y1 - y0))

        def release(self):
            calls.append("released")

    saved = sys.platform, sys.modules.get("dxcam"), globals()["ScreenCapture"]
    sys.platform = "win32"
    sys.modules["dxcam"] = types.SimpleNamespace(create=lambda output_idx, output_color: FakeCam())
    try:
        cap = make_capture(cfg)
        check(cap.name == "dxcam" and cap.rect == {"left": 384, "top": 486, "width": 1152, "height": 291},
              f"dxcam rect {getattr(cap, 'rect', None)}")
        f1, f2 = cap.grab(), cap.grab()
        check(f1 is not None and f1.shape == (291, 1152, 3) and f2 is None, "dxcam frame / no-new-frame")
        check(calls[0] == (384, 486, 1536, 777), f"dxcam region {calls[0]}")
        cap.close()
        calls.clear()
        vt = VisionThread(replace(cfg, max_fps=500)).start()
        deadline = time.perf_counter() + 2
        while vt.latest() is None and time.perf_counter() < deadline:
            time.sleep(0.01)
        time.sleep(0.1)
        check([o.zone for o in vt.poll()] == ["center"] and vt.backend.startswith("dxcam"), "thread on dxcam")
        n_grabs = sum(1 for c in calls if c != "released")
        check(vt.fps <= n_grabs, "None frames are not processed")
        vt.stop()
        check("released" in calls, "dxcam released on stop")

        # dxcam missing -> falls back to mss and says why; forced dxcam -> error
        class FakeMss:
            name = "mss"
            def __init__(self, cfg): pass
        globals()["ScreenCapture"] = FakeMss
        sys.modules["dxcam"] = None              # import dxcam -> ImportError
        cap = make_capture(cfg)
        check(cap.name == "mss" and "dxcam unavailable" in cap.note, f"fallback: {getattr(cap, 'note', '')}")
        try:
            make_capture(replace(cfg, capture_backend="dxcam"))
            fails.append("forced dxcam should raise when missing")
        except ImportError:
            pass
    finally:
        sys.platform = saved[0]
        if saved[1] is None:
            sys.modules.pop("dxcam", None)
        else:
            sys.modules["dxcam"] = saved[1]
        globals()["ScreenCapture"] = saved[2]

    # summarize merges multiple sources; roi_rect math
    z = summarize([Obstacle("left", 50), Obstacle("left", 30, source="csp"), Obstacle("right", 80)])
    check(z["left"].nearest_m == 30 and z["left"].count == 2 and z["center"].count == 0, "summarize")
    r = roi_rect({"left": 1920, "top": 0, "width": 1920, "height": 1080}, cfg)
    check(r == {"left": 1920 + 384, "top": 486, "width": 1152, "height": 291}, f"roi_rect {r}")

    print(f"median process time {ms:.1f} ms (1152x292 ROI -> {cfg.downscale_width} px)")
    print("SELFTEST", "FAILED:\n  " + "\n  ".join(fails) if fails else "OK")
    return 1 if fails else 0


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #
def _crop_roi(full: np.ndarray, cfg: VisionConfig) -> np.ndarray:
    h, w = full.shape[:2]
    r = roi_rect({"left": 0, "top": 0, "width": w, "height": h}, cfg)
    return full[r["top"]:r["top"] + r["height"], r["left"]:r["left"] + r["width"]]


def calibrate(cfg: VisionConfig, path: str = "calibration.png") -> int:
    cap = make_capture(cfg)
    full, t_end = None, time.perf_counter() + 2.0
    while full is None and time.perf_counter() < t_end:   # dxcam: None until the screen changes
        full = cap.grab_full()
        if full is None:
            time.sleep(0.01)
    cap.close()
    if full is None:
        print("No screen frame in 2 s - is the game running and visible?")
        return 1
    full = full.copy()
    print(f"captured with {cap.name} ({cap.note})")
    r = roi_rect({"left": 0, "top": 0, "width": full.shape[1], "height": full.shape[0]}, cfg)
    x, y, w, h = r["left"], r["top"], r["width"], r["height"]
    cv2.rectangle(full, (x, y), (x + w, y + h), (0, 0, 255), 3)
    for f in cfg.zone_split:
        cv2.line(full, (int(x + f * w), y), (int(x + f * w), y + h), (255, 200, 0), 2)
    cx = int(x + cfg.car_center_x * w)
    cv2.line(full, (cx, y + h - 40), (cx, y + h), (255, 255, 255), 3)
    cv2.imwrite(path, full)
    print(f"wrote {path}: red = ROI {cfg.roi}, blue = zone_split, white = car_center_x.\n"
          "ROI should cover road ahead only: bottom edge just above the hood, top edge below the horizon.")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="ac-autodriver vision")
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--selftest", action="store_true")
    g.add_argument("--calibrate", action="store_true")
    g.add_argument("--image", metavar="PNG", help="full-screen screenshot to analyse")
    g.add_argument("--live", action="store_true")
    ap.add_argument("--out", default="vision_debug.png", help="--image output path")
    a = ap.parse_args()
    cfg = CONFIG.vision

    if a.selftest:
        return selftest()
    if a.calibrate:
        return calibrate(cfg)
    if a.image:
        full = cv2.imread(a.image)
        if full is None:
            print(f"cannot read {a.image}")
            return 1
        p, img = VisionProcessor(cfg).process(_crop_roi(full, cfg))
        cv2.imwrite(a.out, draw_overlay(img, p, cfg))
        print(p.lanes, *p.obstacles, sep="\n")
        print(f"wrote {a.out} ({p.process_ms:.1f} ms)")
        return 0

    vt = VisionThread(replace(cfg, debug_window=True)).start()
    print("live vision - Ctrl+C to stop (debug window shows the processed ROI)")
    try:
        while True:
            time.sleep(0.2)
            p = vt.latest()
            if vt.error:
                print(f"\nvision error: {vt.error!r}")
            if p is None:
                continue
            zs = " ".join(f"{z[0].upper()}:{'--' if math.isinf(s.nearest_m) else f'{s.nearest_m:4.0f}m'}"
                          for z, s in p.zones.items())
            off = "  --" if p.lanes.offset is None else f"{p.lanes.offset:+.2f}"
            print(f"\r{vt.backend.split()[0]} {vt.fps:4.0f} fps grab {vt.grab_ms:4.1f} ms proc {vt.proc_ms:4.1f} ms | lane off {off} conf "
                  f"{p.lanes.confidence:.2f} | {zs}   ", end="", flush=True)
    except KeyboardInterrupt:
        print()
        return 0
    finally:
        vt.stop()


if __name__ == "__main__":
    sys.exit(main())

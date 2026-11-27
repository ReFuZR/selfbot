"""
drive.py - steering and shifting from AC's own data.

Motion       heading and how hard the car is actually turning (curvature, 1/m), measured from
             AC's world velocity - no vision involved.
SteerModel   learns how much the car turns per unit of steering axis (curvature = k x steer),
             from YOUR driving while paused (pass-through), then keeps refining while the bot
             drives. No steering-ratio / wheel-degrees guessing, and an inverted binding just
             shows up as a negative k. Saved per car.
Steering     turns the planner's "turn this much" into a steering axis value with that model,
             and holds the current curve when vision briefly loses the lane lines.
Path         optional (F7): record your lane on a road you drive a lot; the bot then knows its
             exact place in the lane from AC's coordinates and reads curves ahead.
GearBox      shifts with two vJoy buttons when AC's automatic gearbox is off.
"""

from __future__ import annotations

import json
import math
import re
from collections import deque
from pathlib import Path as FsPath

from config import Config, DriveConfig
from vision import LaneEstimate

# curvature (1/m) per unit of planner steer the planner's gains were tuned for (sim car:
# 0.30 rad of front wheel at full command, 2.6 m wheelbase)
K_REF = 0.30 / 2.6


def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def heading_of(dx: float, dz: float) -> float:
    """World heading in AC's horizontal plane (x, z); turning toward +curvature raises it."""
    return math.atan2(dx, dz)


# --------------------------------------------------------------------------- #
class Motion:
    """How the car is actually moving, from AC's world velocity."""

    def __init__(self) -> None:
        self.psi: float | None = None
        self.kappa = 0.0          # measured curvature, 1/m (+ = heading increasing)
        self.v = 0.0

    def update(self, velocity, dt: float) -> None:
        vx, _, vz = velocity
        self.v = math.hypot(vx, vz)
        if self.v < 3.0:
            self.psi = None
            return
        psi = heading_of(vx, vz)
        if self.psi is not None and dt > 0:
            k = wrap(psi - self.psi) / (self.v * dt)
            a = 1 - 0.7 ** (dt * 60)                    # light smoothing, same at any rate
            self.kappa += a * (k - self.kappa)
        self.psi = psi


# --------------------------------------------------------------------------- #
class SteerModel:
    """curvature = k x steer_axis, learned by recursive least squares with slow forgetting.
    The steer used is from DELAY_S ago: the car takes a moment to respond."""
    DELAY_S = 0.12
    FORGET = 0.999
    KI = 1.5                  # integral on curvature error (1/s): corrects model error in a curve

    def __init__(self, k: float = 0.0, seconds: float = 0.0, learn_seconds: float = 3.0) -> None:
        self.k, self.seconds, self.learn_seconds = k, seconds, learn_seconds
        # estimate uncertainty. Steering inputs at speed are tiny (~0.003), so an unknown k
        # needs a huge starting value or it barely moves
        self.P = 1e3 if seconds >= learn_seconds else 1e6
        self.hist: deque = deque()
        self.i = 0.0
        self._k_1s: deque = deque()             # recent estimates: is it still moving?
        self.stable = seconds >= learn_seconds
        self._ready = False

    @property
    def ready(self) -> bool:
        # once learned it stays learned (and keeps refining): it must not drop out mid-corner
        if not self._ready and self.seconds >= self.learn_seconds and abs(self.k) > 1e-3 and self.stable:
            self._ready = True
        return self._ready

    @property
    def right(self) -> int:
        """+1 if steering right raises the world heading, -1 if it lowers it."""
        return 1 if self.k >= 0 else -1

    def observe(self, now: float, dt: float, steer: float, kappa: float, v: float, slip: float) -> None:
        self.hist.append((now, steer))
        while self.hist and now - self.hist[0][0] > 1.0:
            self.hist.popleft()
        x = next((s for t, s in reversed(self.hist) if now - t >= self.DELAY_S), None)
        if x is None or v < 8.0 or abs(x) < 0.0015 or abs(kappa) > 0.1 or slip > 3.0:
            return                                   # no information (or sliding)
        g = self.P * x / (self.FORGET + self.P * x * x)
        self.k += g * (kappa - self.k * x)
        self.P = (self.P - g * x * self.P) / self.FORGET
        self.seconds += dt
        self._k_1s.append((now, self.k))
        while self._k_1s and now - self._k_1s[0][0] > 1.0:
            self._k_1s.popleft()
        old = self._k_1s[0][1]
        # settled = moved less than 10 % over the last second
        self.stable = now - self._k_1s[0][0] > 0.5 and abs(self.k - old) < 0.1 * abs(self.k)

    def command(self, kappa_target: float, kappa_meas: float, dt: float) -> float:
        """Steering axis value for a target curvature."""
        if not self.ready:
            return 0.0
        self.i = clamp(self.i + self.KI * (kappa_target - kappa_meas) * dt, -0.01, 0.01)
        return clamp((kappa_target + self.i) / self.k, -1.0, 1.0)

    def reset_integral(self) -> None:
        self.i = 0.0

    # persistence, per car
    @staticmethod
    def load(path: FsPath, car: str, learn_seconds: float) -> "SteerModel":
        try:
            d = json.loads(path.read_text()).get(car)
        except (OSError, ValueError):
            d = None
        return SteerModel(d["k"], d["seconds"], learn_seconds) if d else SteerModel(learn_seconds=learn_seconds)

    def save(self, path: FsPath, car: str) -> None:
        if not self.ready or not car:
            return
        try:
            data = json.loads(path.read_text())
        except (OSError, ValueError):
            data = {}
        data[car] = {"k": self.k, "seconds": min(self.seconds, 600.0)}
        path.write_text(json.dumps(data, indent=2))


# --------------------------------------------------------------------------- #
class Path:
    """A recorded lane: points in AC's world (x, z), every couple of metres."""

    def __init__(self, pts: list[tuple[float, float]], loop: bool = False, name: str = "") -> None:
        self.x = [p[0] for p in pts]
        self.z = [p[1] for p in pts]
        self.loop, self.name = loop, name
        self.s = [0.0]
        for i in range(1, len(pts)):
            self.s.append(self.s[-1] + math.hypot(self.x[i] - self.x[i - 1], self.z[i] - self.z[i - 1]))
        self.length = self.s[-1]
        self._hint: int | None = None

    def __len__(self) -> int:
        return len(self.x)

    def _seg(self, i: int) -> tuple[int, int]:
        n = len(self.x)
        return (i % n, (i + 1) % n) if self.loop else (min(i, n - 2), min(i, n - 2) + 1)

    def locate(self, x: float, z: float) -> tuple[int, float, float]:
        """(segment, lateral offset in m (+ = toward rising heading), distance along)."""
        n = len(self.x)
        def d2(i): return (self.x[i % n] - x) ** 2 + (self.z[i % n] - z) ** 2
        if self._hint is None:
            cand = range(n)
        else:
            lo, hi = self._hint - 30, self._hint + 60
            cand = range(lo, hi) if self.loop else range(max(lo, 0), min(hi, n))
        i = min(cand, key=d2) % n
        if d2(i) > 20.0 ** 2 and self._hint is not None:
            i = min(range(n), key=d2)                # lost: search everything
        self._hint = i
        a, b = self._seg(i)
        if not self.loop and i > 0 and a == i:       # project onto the segment that contains us
            pa, pb = self._seg(i - 1)
            tx, tz = self.x[pb] - self.x[pa], self.z[pb] - self.z[pa]
            if (x - self.x[a]) * tx + (z - self.z[a]) * tz < 0:
                a, b = pa, pb
        tx, tz = self.x[b] - self.x[a], self.z[b] - self.z[a]
        L = math.hypot(tx, tz) or 1e-9
        tx, tz = tx / L, tz / L
        dx, dz = x - self.x[a], z - self.z[a]
        return a, dx * tz - dz * tx, self.s[a] + dx * tx + dz * tz

    def _index_at(self, s: float) -> int:
        if self.loop:
            s %= self.length
        lo, hi = 0, len(self.s) - 1
        while lo < hi:
            mid = (lo + hi) // 2
            if self.s[mid] < s:
                lo = mid + 1
            else:
                hi = mid
        return lo

    def curvature_at(self, s: float, span: float = 8.0) -> float:
        def psi(i):
            a, b = self._seg(i)
            return heading_of(self.x[b] - self.x[a], self.z[b] - self.z[a])
        i0, i1 = self._index_at(s - span), self._index_at(s + span)
        ds = (self.s[i1] - self.s[i0]) % self.length if self.loop else self.s[i1] - self.s[i0]
        return wrap(psi(i1) - psi(i0)) / ds if ds > 1.0 else 0.0

    def ended(self, s: float) -> bool:
        return not self.loop and s > self.length - 20.0

    # persistence
    @staticmethod
    def file_for(cfg: DriveConfig, base: FsPath, track: str) -> FsPath:
        return base / cfg.paths_dir / (re.sub(r"[^\w.-]+", "_", track or "unknown") + ".json")

    def save(self, file: FsPath) -> None:
        file.parent.mkdir(parents=True, exist_ok=True)
        file.write_text(json.dumps({"name": self.name, "loop": self.loop,
                                    "points": [[round(a, 2), round(b, 2)] for a, b in zip(self.x, self.z)]}))

    @staticmethod
    def load(file: FsPath) -> "Path | None":
        try:
            d = json.loads(file.read_text())
            return Path([tuple(p) for p in d["points"]], d.get("loop", False), d.get("name", ""))
        except (OSError, ValueError, KeyError):
            return None


class Recorder:
    def __init__(self, spacing: float = 2.0) -> None:
        self.spacing, self.pts = spacing, []

    def add(self, x: float, z: float) -> None:
        if not self.pts or math.hypot(x - self.pts[-1][0], z - self.pts[-1][1]) >= self.spacing:
            self.pts.append((x, z))

    @property
    def length(self) -> float:
        return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(self.pts, self.pts[1:]))

    def finish(self, name: str) -> Path | None:
        if len(self.pts) < 50:
            return None
        p = Path(self.pts, name=name)
        close = math.hypot(self.pts[-1][0] - self.pts[0][0], self.pts[-1][1] - self.pts[0][1])
        return Path(self.pts, loop=p.length > 500 and close < 30, name=name)


# --------------------------------------------------------------------------- #
class Steering:
    """Planner command -> steering axis, through the learned model."""
    HOLD_DECAY_S = 1.5        # without lanes, keep the current curve, fading over this long

    def __init__(self, cfg: Config, model: SteerModel, path: Path | None = None) -> None:
        self.cfg, self.model, self.path = cfg, model, path
        self.kappa_ff = 0.0
        self.on_path = False
        self.path_e = 0.0
        self._last_target = 0.0

    def lanes_from_path(self, snap, v_mps: float) -> LaneEstimate | None:
        """Lane offset from the recorded path (exact), in the same terms vision reports."""
        self.on_path, self.kappa_ff = False, 0.0
        if self.path is None or not self.model.ready:
            return None
        x, _, z = snap.car_xyz
        _, e, s = self.path.locate(x, z)
        w = self.cfg.drive.lane_width_m
        if abs(e) > 3 * w or self.path.ended(s):
            return None
        self.on_path, self.path_e = True, e
        self.kappa_ff = self.path.curvature_at(s + v_mps * self.cfg.drive.curve_lookahead_s)
        y = e * self.model.right / (w / 2)          # half-lane widths, + = right
        lane = round(y / 2)
        return LaneEstimate(y - 2 * lane, None, 1.0)

    def axis(self, plan_steer: float, blind: bool, motion: Motion, dt: float) -> float:
        """plan_steer: the planner's steer (its units: + = right, K_REF curvature per unit)."""
        if blind:
            # no lane info: keep turning the way we're turning, fading out
            self._last_target *= math.exp(-dt / self.HOLD_DECAY_S)
            target = self._last_target
        else:
            target = plan_steer * K_REF * self.model.right + self.kappa_ff
            self._last_target = target
        return self.model.command(target, motion.kappa, dt)


# --------------------------------------------------------------------------- #
class GearBox:
    """Shifts with vJoy buttons when AC's automatic gearbox is off."""

    def __init__(self, cfg: DriveConfig) -> None:
        self.cfg = cfg
        self.last_shift = -math.inf
        self.pressed: tuple[bool, bool] = (False, False)
        self.release_at = 0.0
        self.seen_max_rpm = 0
        self.note = ""

    def update(self, now: float, snap, throttle: float) -> tuple[bool, bool]:
        """(upshift, downshift) button states for this tick."""
        if any(self.pressed):
            if now < self.release_at:
                return self.pressed
            self.pressed = (False, False)
        if snap is None or snap.auto_shifter:
            self.note = "AC automatic gearbox"
            return self.pressed
        self.seen_max_rpm = max(self.seen_max_rpm, snap.rpm)
        max_rpm = snap.max_rpm if snap.max_rpm > 1000 else max(self.seen_max_rpm, 6000)
        self.note = f"bot shifts (gear {snap.gear_label})"
        if now - self.last_shift < self.cfg.shift_interval_s:
            return self.pressed
        g, rpm, c = snap.gear_raw, snap.rpm, self.cfg
        up = (g <= 1 and throttle > 0.1) or (g >= 2 and rpm > c.upshift_rpm_frac * max_rpm and throttle > 0.3)
        down = g >= 3 and rpm < c.downshift_rpm_frac * max_rpm
        if up or down:
            self.pressed, self.last_shift, self.release_at = (up, not up and down), now, now + c.shift_press_s
        return self.pressed


# --------------------------------------------------------------------------- #
def selftest() -> int:
    from dataclasses import replace
    from types import SimpleNamespace

    from config import CONFIG
    from planner import Planner
    from vision import Perception, summarize

    fails: list[str] = []
    check = lambda ok, name: ok or fails.append(name)  # noqa: E731
    dt = 1 / 60

    def road(points_every: float = 2.0) -> Path:
        """~9 km of highway: straights and 700-1200 m radius bends both ways."""
        segs = [(400, 0), (900, 1 / 800), (300, 0), (1100, -1 / 1000), (500, 0), (800, 1 / 700),
                (400, 0), (1200, -1 / 1200), (600, 0), (900, 1 / 900), (1000, 0)]
        x = z = psi = 0.0
        pts = [(x, z)]
        for length, kap in segs:
            for _ in range(int(length / points_every)):
                psi += kap * points_every
                x += points_every * math.sin(psi)
                z += points_every * math.cos(psi)
                pts.append((x, z))
        return Path(pts, name="test")

    def run(k_sign: int, wheel_rad: float, kmh: float, lag_s: float = 0.08) -> dict:
        """Kinematic car on the test road: 12 s of a 'human' driving (the model learns),
        then the bot keeps lane through the bends and does a lane change left at ~25 s."""
        cfg = replace(CONFIG, mode="simple", modes=dict(CONFIG.modes))
        path = road()
        model = SteerModel(learn_seconds=cfg.drive.learn_seconds)
        steering, motion, planner = Steering(cfg, model, path), Motion(), Planner(cfg)
        x, z, psi = path.x[0], path.z[0], heading_of(path.x[1] - path.x[0], path.z[1] - path.z[0])
        v, delta, t = kmh / 3.6, 0.0, 0.0
        out = {"k_true": k_sign * math.tan(wheel_rad) / 2.6, "max_e": 0.0, "lc_e": None, "trace": []}
        human = SteerModel(out["k_true"], 99, 0)     # the 'human' knows their car
        hpath = Path(list(zip(path.x, path.z)))
        changed = False
        for _ in range(int(40 / dt)):
            t += dt
            snap = SimpleNamespace(speed_kmh=v * 3.6, max_wheel_slip=0.0, damage=0.0, is_fresh=True,
                                   auto_shifter=True, car_xyz=(x, 0.0, z),
                                   velocity=(v * math.sin(psi), 0.0, v * math.cos(psi)))
            motion.update(snap.velocity, dt)
            if t < 12:                               # human drives: looks ahead, weaves a little
                _, e, s = hpath.locate(x, z)
                ld = 25 + 0.6 * v
                i = hpath._index_at(s + ld)
                dx, dz = hpath.x[i] - x, hpath.z[i] - z
                lat = dx * math.cos(psi) - dz * math.sin(psi)     # toward rising heading
                kt = 2 * lat / (dx * dx + dz * dz) + 0.4 / (v * v) * math.sin(t * 1.3)   # ~0.4 m/s2 weave
                steer = human.command(kt, motion.kappa, dt)
            else:
                lanes = steering.lanes_from_path(snap, v)
                perc = Perception(t, lanes, [], summarize([]), (0, 0)) if lanes else None
                if t > 25 and not changed and planner.state == "CRUISE":
                    planner.state, planner.change_dir, planner.change_t0, planner.k = "CHANGE", -1, t, 0
                    changed = True
                cmd = planner.update(snap, perc, t)
                steer = steering.axis(cmd.steer, cmd.state == "BLIND", motion, dt)
                if 16 < t < 24:                      # after the handover settles
                    out["max_e"] = max(out["max_e"], abs(steering.path_e))
                if t > 37:
                    out["lc_e"] = steering.path_e * model.right
            model.observe(t, dt, steer, motion.kappa, v, 0.0)
            _, e_true, _ = hpath.locate(x, z)
            out["trace"].append((round(t, 2), round(e_true, 2), round(steer, 4), round(motion.kappa, 5),
                                 round(model.k, 4), planner.state if t >= 12 else "HUMAN"))
            delta += (steer * wheel_rad * k_sign - delta) * min(1.0, dt / lag_s)
            psi += v * math.tan(delta) / 2.6 * dt
            x += v * math.sin(psi) * dt
            z += v * math.cos(psi) * dt
        out["k"], out["ready"] = model.k, model.ready
        return out

    for k_sign, wheel, kmh in ((1, 0.30, 180), (1, 0.55, 240), (-1, 0.45, 200)):
        r = run(k_sign, wheel, kmh)
        tag = f"k_sign {k_sign:+d} wheel {wheel} rad {kmh} km/h"
        check(r["ready"] and abs(r["k"] / r["k_true"] - 1) < 0.25, f"{tag}: learned k {r['k']:.4f} vs {r['k_true']:.4f}")
        check(r["max_e"] < 0.5, f"{tag}: lane keeping error {r['max_e']:.2f} m")
        check(r["lc_e"] is not None and abs(r["lc_e"] + CONFIG.drive.lane_width_m) < 0.5,
              f"{tag}: lane change ended at {r['lc_e']} m (want {-CONFIG.drive.lane_width_m})")

    # recorder / loop detection / persistence
    rec = Recorder(2.0)
    for i in range(400):
        a = i / 400 * 2 * math.pi
        rec.add(200 * math.sin(a), 200 * math.cos(a))
    p = rec.finish("ring")
    check(p is not None and p.loop and abs(p.curvature_at(300) - 1 / 200) < 2e-4, "ring path")
    import tempfile
    with tempfile.TemporaryDirectory() as d:
        f = FsPath(d) / "x.json"
        p.save(f)
        q = Path.load(f)
        check(q is not None and q.loop and len(q) == len(p), "path save/load")
        m = SteerModel(0.2, 5.0, 3.0)
        m.save(FsPath(d) / "m.json", "car")
        check(SteerModel.load(FsPath(d) / "m.json", "car", 3.0).ready, "model save/load")
        check(not SteerModel.load(FsPath(d) / "m.json", "other", 3.0).ready, "model per car")

    # gearbox
    gb = GearBox(CONFIG.drive)
    def s(**kw):
        base = dict(auto_shifter=False, gear_raw=3, gear_label="2", rpm=5000, max_rpm=8000)
        return SimpleNamespace(**{**base, **kw})
    check(gb.update(0.0, s(rpm=7600), 1.0) == (True, False), "upshift near redline")
    check(gb.update(0.05, s(rpm=7600), 1.0) == (True, False), "held for the press time")
    check(gb.update(0.2, s(rpm=7600), 1.0) == (False, False), "released, then waits")
    check(gb.update(1.0, s(gear_raw=4, rpm=3000), 0.0) == (False, True), "downshift when slow")
    check(gb.update(2.0, s(gear_raw=1, rpm=900), 0.5) == (True, False), "neutral -> first")
    check(gb.update(3.0, s(auto_shifter=True, rpm=7900), 1.0) == (False, False), "auto gearbox: hands off")

    print("SELFTEST", "FAILED:\n  " + "\n  ".join(fails) if fails else "OK")
    return 1 if fails else 0


if __name__ == "__main__":
    import sys
    sys.exit(selftest() if "--selftest" in sys.argv else print("usage: python drive.py --selftest") or 0)

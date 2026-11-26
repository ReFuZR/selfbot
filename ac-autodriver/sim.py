"""
sim.py - tiny 3-lane highway to try planner modes offline (no AC, no screen).

Feeds the planner the same Perception / snapshot shapes the real modules produce,
with vision-like limits: only sees 6-120 m ahead in the left/center/right zones,
4 % distance noise, dropped frames, nothing beside the car (blind spot).
The car is a bicycle model with a grip limit; going over it is a spin (= crash).
Output goes through the real VirtualController smoothing.

    python sim.py                              # compare all modes, 3 seeds x 3 min
    python sim.py --mode madmax --seconds 600  # one mode, longer
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass, replace
from types import SimpleNamespace

import numpy as np

from config import CONFIG, Config
from controller import NullBackend, VirtualController
from planner import Planner
from vision import LaneEstimate, Obstacle, Perception, summarize

LANE_W, LANES, CAR_L, CAR_W = 3.7, 3, 4.6, 1.9
WHEELBASE, MAX_WHEEL_RAD, GRIP = 2.6, 0.30, 1.25 * 9.81
A_MAX, B_MAX, V_TOP = 9.0, 12.0, 330 / 3.6
SEE_NEAR, SEE_FAR = 6.0, 120.0


@dataclass
class Car:
    x: float
    lane: int
    v: float
    cruise: float = 0.0

    def __post_init__(self) -> None:
        self.cruise = self.cruise or self.v

    @property
    def y(self) -> float:
        return (self.lane + 0.5) * LANE_W


class Sim:
    def __init__(self, cfg: Config, seed: int = 0, density: float = 1.0) -> None:
        self.rng = np.random.default_rng(seed)
        self.cfg, self.density = cfg, density
        self.planner = Planner(cfg)
        # unscaled output: the car model below already is "planner steer 1.0 = 0.30 rad"
        self.pad = VirtualController(replace(cfg.controller, wake_on_connect=False, output="xbox"),
                                     NullBackend())
        self.x, self.y, self.yaw, self.v = 0.0, 1.5 * LANE_W, 0.0, 150 / 3.6
        self.damage, self.crashes, self.true_passes, self.t = 0.0, 0, 0, 0.0
        self.slip, self.dist, self.max_lat_g = 0.0, 0.0, 0.0
        self.traffic: list[Car] = []
        x = 40.0
        while x < 900:
            self.traffic.append(self._new_car(x))
            x += self.rng.exponential(45 / density) + 8
        self.prev_perc: Perception | None = None
        self.crash_log: list = []
        self.pass_gaps: list = []
        self.last_cmd = None

    def _new_car(self, x: float) -> Car:
        return Car(x, int(self.rng.integers(LANES)), self.rng.uniform(85, 130) / 3.6)

    def perceive(self) -> Perception | None:
        if self.rng.random() < 0.02:
            return None                                     # dropped frame
        lane = int(clamp(self.y // LANE_W, 0, LANES - 1))
        off = (self.y - (lane + 0.5) * LANE_W) / (LANE_W / 2) + self.rng.normal(0, 0.03)
        obs = []
        for c in self.traffic:
            d = c.x - self.x - CAR_L
            rel = c.lane - lane
            if SEE_NEAR < d < SEE_FAR and abs(rel) <= 1:
                obs.append(Obstacle(("left", "center", "right")[rel + 1], d * (1 + self.rng.normal(0, 0.04))))
        if lane == 0:
            obs.append(Obstacle("left", 3.0, source="barrier"))
        if lane == LANES - 1:
            obs.append(Obstacle("right", 3.0, source="barrier"))
        return Perception(self.t, LaneEstimate(off, 0.0, 1.0), obs, summarize(obs), (0, 0))

    def step(self, dt: float) -> None:
        snap = SimpleNamespace(speed_kmh=self.v * 3.6, max_wheel_slip=self.slip,
                               damage=self.damage, is_fresh=True)
        cmd = self.planner.update(snap, self.prev_perc, self.t)    # one frame of vision latency
        self.last_cmd = cmd
        self.prev_perc = self.perceive()
        self.pad.set_targets(cmd.steer, cmd.throttle, cmd.brake)
        steer, thr, brk = self.pad.update(self.t)

        x_old = self.x
        acc = thr * A_MAX * max(0.0, 1 - self.v / V_TOP) - brk * B_MAX - 0.0004 * self.v ** 2
        self.v = max(0.0, self.v + acc * dt)
        yaw_rate = self.v * math.tan(steer * MAX_WHEEL_RAD) / WHEELBASE
        lat = self.v * yaw_rate
        self.max_lat_g = max(self.max_lat_g, abs(lat) / 9.81)
        self.slip = 4.0 * abs(lat) / GRIP
        if abs(lat) > GRIP:                                 # over the limit: spin
            self._crash("spin")
            yaw_rate, self.yaw = 0.0, 0.0
        self.yaw += yaw_rate * dt
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.dist += self.v * dt
        if not CAR_W / 2 < self.y < LANES * LANE_W - CAR_W / 2:
            self._crash("barrier")
            self.y = clamp(self.y, CAR_W / 2 + 0.1, LANES * LANE_W - CAR_W / 2 - 0.1)
            self.yaw = 0.0

        ego_lane = int(clamp(self.y // LANE_W, 0, LANES - 1))
        for c in list(self.traffic):
            before = c.x > x_old       # compare with ego position before this step
            # traffic keeps its cruise speed but brakes for anything slower close ahead in its lane
            lead = [(o.x, o.v) for o in self.traffic if o.lane == c.lane and 0 < o.x - c.x < 35]
            if c.lane == ego_lane and 0 < self.x - c.x < 35:
                lead.append((self.x, self.v))
            if lead:
                lx, lv = min(lead)
                c.v = max(0.0, min(c.v, lv + (lx - c.x - 15) * 0.2))
            elif c.v < c.cruise:
                c.v = min(c.cruise, c.v + 1.5 * dt)
            c.x += c.v * dt
            if abs(c.x - self.x) < CAR_L and abs(c.y - self.y) < CAR_W:
                self._crash("car")
                self.v = max(0.0, c.v - 8)
                self.traffic.remove(c)
                continue
            if before and c.x <= self.x:
                self.true_passes += 1
                if abs(c.lane - ego_lane) == 1:
                    self.pass_gaps.append(abs(c.y - self.y) - CAR_W)   # true side gap
            if c.x < self.x - 150:                           # recycle far ahead
                self.traffic.remove(c)
                self.traffic.append(self._new_car(self.x + self.rng.uniform(600, 1000)))
        self.t += dt

    def _crash(self, why: str) -> None:
        c = self.last_cmd
        self.crash_log.append((round(self.t, 1), why, round(self.v * 3.6), c and c.state, c and c.note))
        self.crashes += 1
        self.damage += 5.0
        self.v *= 0.6

    def run(self, seconds: float, hz: float = 60.0) -> dict:
        for _ in range(int(seconds * hz)):
            self.step(1 / hz)
        s = self.planner.score
        return dict(score=s.total, best=s.best, passes=s.passes, near=s.near_misses,
                    cuts=s.cuts, swerves=s.swerves,
                    gap=float(np.median(self.pass_gaps)) if self.pass_gaps else 0.0,
                    crashes=self.crashes, true_passes=self.true_passes,
                    avg_kmh=self.dist / self.t * 3.6, lat_g=self.max_lat_g)


def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def run_mode(mode: str, seconds: float, seeds: list[int], density: float = 1.0,
             verbose: bool = False) -> dict:
    tot: dict = {}
    for seed in seeds:
        cfg = replace(CONFIG, mode=mode, modes=dict(CONFIG.modes))
        sim = Sim(cfg, seed=seed, density=density)
        r = sim.run(seconds)
        if verbose:
            for t, why, kmh, state, note in sim.crash_log:
                print(f"  {mode} seed {seed}: crash ({why}) at {t}s, {kmh} km/h, {state} {note}")
        for k, v in r.items():
            tot[k] = max(tot.get(k, 0), v) if k == "lat_g" else tot.get(k, 0) + v / len(seeds)
    return tot


def selftest() -> int:
    """Regression check: modes rank as intended and crashes stay rare."""
    r = {m: run_mode(m, 150, [10, 11, 12]) for m in ("cruise", "simple", "madmax")}
    fails = []
    if not r["madmax"]["score"] > r["simple"]["score"] > r["cruise"]["score"]:
        fails.append("score ranking madmax > simple > cruise")
    if not r["madmax"]["avg_kmh"] > r["cruise"]["avg_kmh"]:
        fails.append("madmax should be faster than cruise")
    if sum(x["crashes"] for x in r.values()) * 3 > 1:
        fails.append("more than 1 crash in 9 runs")
    if r["madmax"]["near"] < 0.5 * r["madmax"]["passes"]:
        fails.append("madmax should farm near misses")
    if r["cruise"]["lat_g"] > 0.8:
        fails.append("cruise too violent")
    for m, x in r.items():
        print(f"  {m:<7} score {x['score']:7.0f}  passes {x['passes']:4.1f}  crashes {x['crashes']:.2f}  "
              f"{x['avg_kmh']:5.1f} km/h  {x['lat_g']:.2f} g")
    print("SELFTEST", "FAILED: " + "; ".join(fails) if fails else "OK")
    return 1 if fails else 0


def main() -> int:
    ap = argparse.ArgumentParser(description="offline planner simulator")
    ap.add_argument("--mode", choices=list(CONFIG.modes), help="default: compare all")
    ap.add_argument("--seconds", type=float, default=180)
    ap.add_argument("--seeds", type=int, default=3)
    ap.add_argument("--density", type=float, default=1.0, help="traffic density multiplier")
    ap.add_argument("--verbose", action="store_true", help="print every crash")
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        return selftest()
    modes = [a.mode] if a.mode else list(CONFIG.modes)
    print(f"{a.seeds} seed(s) x {a.seconds:.0f} s, density {a.density}  (averages per run)")
    print(f"{'mode':<8} {'score':>8} {'passes':>7} {'near':>5} {'cuts':>5} {'swerve':>6} {'side gap':>8} "
          f"{'crash':>6} {'avg km/h':>9} {'max lat g':>9}")
    for mode in modes:
        r = run_mode(mode, a.seconds, list(range(a.seeds)), a.density, a.verbose)
        print(f"{mode:<8} {r['score']:8.0f} {r['passes']:7.1f} {r['near']:5.1f} {r['cuts']:5.1f} "
              f"{r['swerves']:6.1f} {r['gap']:7.2f}m {r['crashes']:6.2f} {r['avg_kmh']:9.1f} {r['lat_g']:9.2f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

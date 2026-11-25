"""Tiny kinematic highway simulator for tuning/testing the planner offline."""
import math
import random
from planner import Planner, Config


def run(seed=0, seconds=120, density=1 / 45.0, cfg=None, steer_scale=1.0, hz=50, verbose=False):
    rng = random.Random(seed)
    cfg = cfg or Config()
    lanes = cfg.lane_centers
    dt = 1.0 / hz
    # ego
    ef, el, psi, v = 0.0, lanes[1], 0.0, 30.0
    # traffic: [id, f, lane_l, speed]
    cars, nid = [], 0
    f = 60.0
    while f < 1500:
        if rng.random() < density * 45 * 0.5:
            cars.append([nid, f, rng.choice(lanes), rng.uniform(22, 30)]); nid += 1
        f += 45
    p = Planner(cfg)
    t, crashes, passed, dist0 = 0.0, 0, 0, ef
    min_clear = 99.0
    maxv = 0.0
    colliding = set()
    while t < seconds:
        # world coords: forward = +z, right = +x  (so axis=(0,1), lateral = x)
        traffic = [(c[0], c[2], c[1]) for c in cars]
        ctl = p.step(t, (el, ef), (v * math.sin(psi), v * math.cos(psi)), traffic)
        a = 0.8 * ctl.throttle * (1 - v / 85.0) * 12 - ctl.brake * 10.0 - 0.0008 * v * v
        v = max(0.0, v + a * dt)
        delta = ctl.steer * 0.09 * steer_scale / (1.0 + v / 25.0)
        psi += v / 2.7 * math.tan(delta) * dt
        psi = max(-0.6, min(0.6, psi))
        ef += v * math.cos(psi) * dt
        el += v * math.sin(psi) * dt
        for c in cars:
            c[1] += c[3] * dt
        for c in cars:
            df, dl = abs(c[1] - ef), abs(c[2] - el)
            if df < 4.7 and dl < 2.0:
                if c[0] not in colliding:
                    crashes += 1; colliding.add(c[0])
            if df < 6 and dl < 6:
                min_clear = min(min_clear, dl)
        # recycle
        for c in cars:
            if c[1] < ef - 150:
                c[1] = ef + 700 + rng.uniform(0, 300)
                c[2] = rng.choice(lanes); c[3] = rng.uniform(22, 30)
        if abs(el) > lanes[-1] + 3.0:
            crashes += 1; el = max(min(el, lanes[-1]), lanes[0]); psi = 0
        maxv = max(maxv, v)
        t += dt
    return {"seed": seed, "crashes": crashes, "dist_m": round(ef), "avg_kmh": round(ef / seconds * 3.6),
            "max_kmh": round(maxv * 3.6)}


if __name__ == "__main__":
    for s in range(6):
        print(run(seed=s))

"""Traffic-weaving planner. Pure Python, no game/OS dependencies (so it can be simulated).

Road frame: forward = direction of travel when the bot started, lateral = metres to the
RIGHT of the start line. Works for straight-ish highways, which is what traffic servers are.
"""
import math
from dataclasses import dataclass


@dataclass
class Config:
    lane_centers: tuple = (-5.25, -1.75, 1.75, 5.25)   # lateral metres, relative to start
    car_width: float = 2.0
    car_length: float = 4.7
    margin: float = 0.55          # extra side clearance (m). Lower = closer passes = "daring"
    cruise_kmh: float = 180.0
    lookahead_s: float = 4.5
    min_lookahead: float = 45.0
    brake_decel: float = 9.0      # m/s^2 the planner assumes it can brake at
    switch_penalty: float = 4.0   # metres of gap a lane change must beat
    hysteresis: float = 10.0
    steer_gain: float = 1.6
    max_steer: float = 0.7
    steer_rate: float = 2.5       # max change of steer command per second
    ref_resp: float = 1.5         # yaw response the gains were tuned for (see sim.py)
    steer_sign: float = 1.0       # flip to -1 if the car steers the wrong way in game


@dataclass
class Control:
    steer: float = 0.0     # -1 (left) .. 1 (right)
    throttle: float = 0.0  # 0..1
    brake: float = 0.0     # 0..1


def _clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


class Planner:
    def __init__(self, cfg=None):
        self.cfg = cfg or Config()
        self.origin = None
        self.axis = None
        self.target_lane = None
        self.steer = 0.0
        self._prev = {}      # traffic id -> (forward, t, smoothed forward speed)
        self.last_gap = float("inf")
        self.resp = None        # measured yaw response per unit steer (adaptive)
        self._psi = None
        self._steer_cmd = 0.0

    # -- frame ----------------------------------------------------------------
    def _init_frame(self, x, z, vx, vz):
        n = math.hypot(vx, vz)
        self.origin = (x, z)
        self.axis = (vx / n, vz / n)

    def to_road(self, x, z):
        dx, dz = x - self.origin[0], z - self.origin[1]
        fx, fz = self.axis
        return dx * fx + dz * fz, dx * fz - dz * fx   # forward, lateral(right)

    def _vel_road(self, vx, vz):
        fx, fz = self.axis
        return vx * fx + vz * fz, vx * fz - vz * fx

    # -- main step --------------------------------------------------------------
    def step(self, t, own_xz, own_vxz, traffic):
        """traffic: iterable of (id, x, z) in world coords. Returns Control."""
        c = self.cfg
        x, z = own_xz
        vx, vz = own_vxz
        if self.origin is None:
            if math.hypot(vx, vz) < 5.0:      # wait until we're actually moving
                return Control(0.0, 1.0, 0.0)
            self._init_frame(x, z, vx, vz)
        ef, el = self.to_road(x, z)
        vf, vl = self._vel_road(vx, vz)
        speed = max(vf, 0.1)
        psi = math.atan2(vl, speed)           # heading relative to road, + = toward right

        cars = []
        for tid, tx, tz in traffic:
            f, l = self.to_road(tx, tz)
            pv = self._prev.get(tid)
            cv = 0.0   # unknown speed on first sighting: assume stopped (conservative)
            if pv is not None and t > pv[1]:
                raw = (f - pv[0]) / (t - pv[1])
                cv = 0.7 * pv[2] + 0.3 * raw if abs(raw) < 120 else pv[2]
            self._prev[tid] = (f, t, cv)
            cars.append((f - ef, l, cv))      # relative forward, lateral, forward speed

        D = max(c.min_lookahead, speed * c.lookahead_s)
        clear = (c.car_width + 2.0) / 2 + c.margin
        trans_d = max(18.0, speed * 0.9)

        cur_i = min(range(len(c.lane_centers)), key=lambda i: abs(c.lane_centers[i] - el))
        results = {}
        for i, L in enumerate(c.lane_centers):
            lo, hi = min(L, el) - clear, max(L, el) + clear
            gap, lead_v, valid = float("inf"), None, True
            for rf, l, cv in cars:
                if abs(l - L) < clear and -c.car_length * 1.2 < rf < D:
                    g = rf - c.car_length
                    if g < gap:
                        gap, lead_v = g, cv
                if i != cur_i and lo < l < hi and -c.car_length * 1.6 < rf < trans_d:
                    # cars in our own lane ahead don't veto leaving it
                    if not (abs(l - el) < clear and rf > c.car_length * 0.5):
                        valid = False
            results[i] = (valid, gap, lead_v)

        def score(i):
            valid, gap, _ = results[i]
            if not valid:
                return -1e9
            return min(gap, D) - c.switch_penalty * abs(c.lane_centers[i] - el)

        if self.target_lane is None:
            self.target_lane = cur_i
        n = len(c.lane_centers)
        # only ever move one lane at a time, and let the car settle first
        near_ok = [i for i in (self.target_lane - 1, self.target_lane, self.target_lane + 1) if 0 <= i < n]
        best = max(near_ok, key=score)
        settled = abs(c.lane_centers[self.target_lane] - el) < 1.2
        if not results[self.target_lane][0] and best != self.target_lane:
            self.target_lane = best          # emergency: current path is vetoed
        elif best != self.target_lane and settled and score(best) > score(self.target_lane) + c.hysteresis:
            self.target_lane = best
        _, gap, lead_v = results[self.target_lane]
        self.last_gap = gap

        # --- speed
        v_cruise = c.cruise_kmh / 3.6
        v_tgt = v_cruise
        if gap != float("inf"):
            lv = lead_v if lead_v is not None else 0.0
            safe = max(gap - 9.0, 0.0)
            v_tgt = min(v_cruise, math.sqrt(max(lv * lv + 2 * c.brake_decel * safe, 0.0)))
        err = v_tgt - speed
        throttle = _clamp(err * 0.25, 0.0, 1.0)
        brake = _clamp((-err - 1.5) * 0.25, 0.0, 1.0)

        # --- steering
        e = c.lane_centers[self.target_lane] - el
        look = max(14.0, speed * 0.7)
        des = _clamp(math.atan2(e, look), -0.4, 0.4)
        dt = 0.02
        if hasattr(self, "_t"):
            dt = max(t - self._t, 1e-3)
        self._t = t
        # adaptive gain: compare achieved yaw rate with what we commanded last step
        if self._psi is not None and abs(self._steer_cmd) > 0.12 and speed > 10:
            r = (psi - self._psi) / dt * (1.0 + speed / 18.0) / self._steer_cmd
            if r > 0:
                self.resp = r if self.resp is None else 0.98 * self.resp + 0.02 * r
        self._psi = psi
        mult = 1.0
        if self.resp is not None:
            mult = _clamp(c.ref_resp / self.resp, 0.4, 3.0)
        raw = c.steer_gain * mult * (des - psi) * 8.0 / (1.0 + speed / 18.0)
        raw = _clamp(raw, -c.max_steer, c.max_steer)
        mx = c.steer_rate * dt
        self.steer += _clamp(raw - self.steer, -mx, mx)
        self._steer_cmd = self.steer
        return Control(self.steer * c.steer_sign, throttle, brake)

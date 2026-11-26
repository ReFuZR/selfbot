"""
planner.py - turns telemetry + perception into (steer, throttle, brake).

    CRUISE   hold lane (PD on vision lane offset), PID to the mode's target speed
    CHANGE   lateral target slides one lane over (closed loop, so it works at any speed)
    FOLLOW   boxed in: lift, brake on the deceleration actually needed to not hit the car ahead
    BLIND / WAIT   no vision / stale telemetry: lift off, steer straight

Vision only sees 6-120 m ahead, never beside us. Cars that drop out of view up close are
kept as Hidden memories (dead-reckoned from their last sighting and our own speed) so we
neither drive into a car right ahead nor change lanes into one beside us.

Modes (config.MODES) set how hard it drives. ScoreModel is our own score:
passes x speed x combo, near-miss multiplier for leaning toward the car you pass,
speed points, crash penalty. "madmax" is tuned to farm exactly that.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field

from config import CONFIG, Config, ScoreConfig
from vision import Perception

SIDES = (("left", -1), ("right", 1))
LANES3 = ("left", "center", "right")


def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def approach(cur: float, target: float, max_step: float) -> float:
    return cur + clamp(target - cur, -max_step, max_step)


class PID:
    def __init__(self, kp: float, ki: float, kd: float, i_limit: float) -> None:
        self.kp, self.ki, self.kd, self.i_limit = kp, ki, kd, i_limit
        self.reset()

    def reset(self) -> None:
        self.i, self.prev = 0.0, None

    def step(self, err: float, dt: float, freeze_i: bool = False) -> float:
        if not freeze_i:
            self.i = clamp(self.i + self.ki * err * dt, -self.i_limit, self.i_limit)
        d = 0.0 if self.prev is None else (err - self.prev) / dt
        self.prev = err
        return self.kp * err + self.i + self.kd * d


class Tracker:
    """Nearest-obstacle distance and closing speed (m/s, + = getting closer) for one zone.
    Closing speed is the slope of a line fit over the last 0.5 s - frame differencing would
    be buried in vision noise. Tells a car we're passing (closing) from a barrier (constant)."""
    WINDOW_S = 0.5

    def __init__(self) -> None:
        self.samples: deque = deque()
        self.d, self.closing = math.inf, 0.0

    def update(self, now: float, d: float) -> None:
        if not math.isfinite(d) or (self.samples and abs(d - self.samples[-1][1]) > 12):
            self.samples.clear()             # gone, or the nearest object changed identity
        self.d = d
        if math.isfinite(d):
            self.samples.append((now, d))
        while self.samples and now - self.samples[0][0] > self.WINDOW_S:
            self.samples.popleft()
        if len(self.samples) < 6:
            self.closing = 0.0
            return
        tm = sum(t for t, _ in self.samples) / len(self.samples)
        dm = sum(x for _, x in self.samples) / len(self.samples)
        var = sum((t - tm) ** 2 for t, _ in self.samples) or 1e-9
        self.closing = -sum((t - tm) * (x - dm) for t, x in self.samples) / var

    def approaching(self, min_mps: float = 2.0) -> bool:
        return self.closing >= min_mps


@dataclass
class Hidden:
    """A car we can't see right now but know is there, dead-reckoned from its last sighting.
    d: metres ahead of us (negative = behind). Its own speed is assumed constant, so its
    closing speed changes exactly as much as our speed does."""
    d: float
    closing0: float
    v0_kmh: float
    t0: float

    def closing(self, v_kmh: float) -> float:
        return self.closing0 - (self.v0_kmh - v_kmh) / 3.6

    def advance(self, dt: float, v_kmh: float) -> None:
        self.d -= self.closing(v_kmh) * dt


# --------------------------------------------------------------------------- #
# Score model
# --------------------------------------------------------------------------- #
@dataclass
class ScoreModel:
    cfg: ScoreConfig
    total: float = 0.0
    best: float = 0.0
    combo: int = 1
    passes: int = 0
    near_misses: int = 0
    crashes: int = 0
    events: deque = field(default_factory=lambda: deque(maxlen=6))   # (t, text) for the HUD
    _combo_until: float = 0.0
    _armed: dict = field(default_factory=lambda: {"left": None, "right": None})
    _speeds: deque = field(default_factory=lambda: deque(maxlen=64))
    _damage: float | None = None
    _crash_quiet_until: float = 0.0

    def combo_left(self, now: float) -> float:
        return max(0.0, self._combo_until - now) if self.combo > 1 else 0.0

    def update(self, now: float, dt: float, speed_kmh: float, damage: float,
               trackers: dict[str, Tracker], lean: float) -> None:
        c = self.cfg
        if speed_kmh > c.speed_floor_kmh:
            self.total += dt * c.speed_points_per_s * (speed_kmh - c.speed_floor_kmh) / 100
        if self.combo > 1 and now > self._combo_until:
            self.combo = 1
            self.events.append((now, "combo lost"))

        # pass = side car closed to within pass_trigger_m, then vanished off the bottom of the view
        for side, sign in SIDES:
            tr, arm = trackers[side], self._armed[side]
            if tr.d < c.pass_trigger_m and (arm is not None or tr.approaching()):
                if arm is None:
                    self._armed[side] = [tr.d, lean * sign]
                else:
                    arm[0], arm[1] = min(arm[0], tr.d), max(arm[1], lean * sign)
            elif arm is not None and tr.d < 2 * c.pass_trigger_m:
                if tr.d > arm[0] + 4:
                    self._armed[side] = None       # it pulled away from us: not a pass
            elif arm is not None:
                self._armed[side] = None
                self._on_pass(now, speed_kmh, near=arm[1] >= c.near_miss_lean)

        self._check_crash(now, speed_kmh, damage)
        self.best = max(self.best, self.total)

    def _on_pass(self, now: float, speed_kmh: float, near: bool) -> None:
        c = self.cfg
        pts = c.pass_points * max(speed_kmh, 0) / 100 * self.combo * (c.near_miss_mult if near else 1)
        self.total += pts
        self.passes += 1
        self.near_misses += near
        self.events.append((now, f"{'NEAR MISS' if near else 'pass'} +{pts:.0f} x{self.combo}"))
        self.combo = min(self.combo + 1, c.combo_max)
        self._combo_until = now + c.combo_window_s

    def _check_crash(self, now: float, speed_kmh: float, damage: float) -> None:
        self._speeds.append((now, speed_kmh))
        hit = self._damage is not None and damage > self._damage + 1e-3
        self._damage = damage
        old = next((v for t, v in self._speeds if now - t <= 0.3), speed_kmh)
        hit |= old - speed_kmh > 30          # >2.8 g deceleration: brakes can't do that
        if hit and now >= self._crash_quiet_until:
            c = self.cfg
            self.crashes += 1
            self.combo = 1
            self.total = 0.0 if c.crash_resets_score else max(0.0, self.total - c.crash_penalty)
            self.events.append((now, "CRASH"))
            self._crash_quiet_until = now + 1.0


# --------------------------------------------------------------------------- #
# Planner
# --------------------------------------------------------------------------- #
@dataclass
class Command:
    steer: float
    throttle: float
    brake: float
    state: str
    note: str = ""


class Planner:
    HIDE_NEAR_M = {"left": 20.0, "center": 15.0, "right": 20.0}  # remember cars last seen this close
    VISIBLE_FROM_M = 12.0    # a hidden car estimated beyond this should be visible again
    HIDDEN_MAX_S = 15.0
    HUG_RANGE_M = 30.0       # lean toward side cars closer than this (mode.hug)
    LOST_LANES_S = 0.4       # dead-reckon lateral position this long without lane lines

    def __init__(self, cfg: Config | None = None) -> None:
        self.cfg = cfg or CONFIG
        sc = self.cfg.speed
        self.speed_pid = PID(sc.kp, sc.ki, sc.kd, sc.integral_limit)
        self.score = ScoreModel(self.cfg.score)
        self.reset()

    @property
    def mode(self):
        return self.cfg.active

    def set_mode(self, name: str) -> None:
        self.cfg.mode = name
        _ = self.cfg.active  # validates

    def reset(self) -> None:
        self.state = "WAIT"
        self.steer = self.throttle = self.brake = self.trim = 0.0
        self.change_dir, self.change_t0, self.k = 0, 0.0, 0
        self.last_change = -math.inf
        self.y = self.y_rate = 0.0
        self.prev_off: float | None = None
        self.lanes_t = -math.inf
        self.last_t: float | None = None
        self.last_perc: Perception | None = None
        self.trackers = {z: Tracker() for z in LANES3}
        self.hidden: dict[str, Hidden | None] = {z: None for z in LANES3}
        self.speed_pid.reset()

    # ----------------------------------------------------------------------- #
    def update(self, snap, perc: Perception | None, now: float) -> Command:
        """snap: telemetry.Snapshot (or anything with speed_kmh, max_wheel_slip, damage, is_fresh)."""
        dt = 1 / 60 if self.last_t is None else clamp(now - self.last_t, 1e-3, 0.1)
        self.last_t = now
        if snap is None or not snap.is_fresh:
            self.reset()
            self.last_t = now
            return Command(0.0, 0.0, 0.0, "WAIT", "telemetry stale")

        m, v = self.mode, snap.speed_kmh
        if perc is None and now - self.lanes_t <= self.LOST_LANES_S:
            perc = self.last_perc               # one dropped frame: reuse the last view
        self.last_perc = perc
        if perc:
            for z, tr in self.trackers.items():
                tr.update(now, perc.zones[z].nearest_m)
        self.score.update(now, dt, v, snap.damage, self.trackers, self.y if self.state != "CHANGE" else 0.0)
        self._track_lateral(perc, now, dt, v)
        self._track_hidden(now, dt, v)
        if perc is None or now - self.lanes_t > self.LOST_LANES_S:
            return self._output(0.0, 0.0, 0.25 if v > 120 else 0.0, dt, v, snap, "BLIND", "no lanes")

        decel = self.cfg.speed.brake_decel_mps2
        d_brake = m.follow_base_m + m.follow_per_kmh_m * v
        d_over = d_brake * m.overtake_factor
        ahead, closing = self._ahead(v)
        note = ""

        # ---- lane decision --------------------------------------------------
        if self.state == "CHANGE":
            note = self._run_change(now, m)
        if self.state != "CHANGE" and ahead < d_over and now - self.last_change >= m.cooldown_s:
            options = []
            for side, sign in SIDES:
                tr = self.trackers[side]
                c = max(tr.closing, 0.0)
                # where that lane's car will be once we're in it, and the room we'd need to
                # brake down to its speed from there
                gap = tr.d - c * m.lane_change_s
                need = max(m.min_gap_m, ahead + 5, m.follow_base_m + c * c / (2 * 0.6 * decel))
                if gap >= need and self.hidden[side] is None:
                    options.append((gap, side, sign))
            if options:
                _, side, sign = max(options)            # most open lane
                self.state, self.change_dir, self.change_t0, self.k = "CHANGE", sign, now, 0
                note = f"overtake {side}"

        # ---- lateral target -------------------------------------------------
        if self.state == "CHANGE":
            s = clamp((now - self.change_t0) / m.lane_change_s, 0, 1)
            target = 2 * self.change_dir * s * s * (3 - 2 * s)   # smoothstep over one lane
        else:
            target = 0.0
            near = [(self.trackers[s].d, sign) for s, sign in SIDES  # only cars we're passing,
                    if self.trackers[s].d < self.HUG_RANGE_M and self.trackers[s].approaching()]
            if near and m.hug:                                     # not barriers
                target = m.hug * min(near)[1]                      # lean toward the closest one

        # ---- longitudinal -----------------------------------------------------
        target_v, brake = m.target_kmh, 0.0
        changing = self.state == "CHANGE"
        standoff = max(4.0, 0.5 * m.follow_base_m)
        if closing > 0 and ahead < 200:
            # brake on the deceleration actually needed to match the car ahead before standoff
            need = closing * closing / (2 * max(ahead - standoff, 0.5))
            if need > 0.25 * decel or ahead < standoff:
                brake = clamp(1.25 * need / decel, 0.15, 1.0)
        if not changing and ahead < d_brake and closing > -1:
            brake = max(brake, 0.5 * clamp((d_brake - ahead) / d_brake, 0, 1))
        if brake > 0:
            self.state, note = ("CHANGE" if changing else "FOLLOW"), note or f"brake {ahead:.0f}m"
        elif ahead < d_over and not changing:
            target_v = min(target_v, v - 8)                    # lift and wait for a gap
            self.state, note = "FOLLOW", note or "boxed in"
        elif not changing:
            self.state = "CRUISE"

        # ---- steering -------------------------------------------------------
        st = self.cfg.steer
        raw = st.kp * (target - self.y) - st.kd * self.y_rate
        if perc.lanes.heading is not None:
            raw += st.k_heading * perc.lanes.heading
        scale = max(st.min_steer_scale, 1 / (1 + (max(v, 0) / st.damp_ref_kmh) ** st.damp_exponent))
        steer = raw * scale * m.steer_gain

        # ---- speed PID --------------------------------------------------------
        err = target_v - v
        u = self.speed_pid.step(err, dt, freeze_i=brake > 0)
        throttle = clamp(u, 0, 1) if brake == 0 else 0.0
        if brake == 0 and err < -self.cfg.speed.brake_deadband_kmh:
            brake, throttle = clamp(-u * 0.5, 0, 1), 0.0
        return self._output(steer, throttle, brake, dt, v, snap, self.state, note)

    # ----------------------------------------------------------------------- #
    def _ahead(self, v_kmh: float) -> tuple[float, float]:
        """(distance, closing speed) of whatever is nearest ahead in our lane, seen or hidden."""
        tr, h = self.trackers["center"], self.hidden["center"]
        if h and h.d < tr.d:
            return h.d, h.closing(v_kmh)
        return tr.d, tr.closing

    def _track_hidden(self, now: float, dt: float, v_kmh: float) -> None:
        """Remember cars that drop out of view up close (vision is blind beside us and in the
        last few metres ahead). Forget them once they must have fallen behind us, once they
        should be visible again but aren't, or after HIDDEN_MAX_S."""
        m = self.mode
        for z, tr in self.trackers.items():
            h = self.hidden[z]
            if math.isfinite(tr.d) and tr.d < self.HIDE_NEAR_M[z]:
                closing = max(tr.closing, 0.0) if z == "center" else tr.closing
                self.hidden[z] = Hidden(tr.d, closing, v_kmh, now)
                continue
            if h is None:
                continue
            h.advance(dt, v_kmh)
            if z == "center":
                h.d = max(h.d, 0.5)               # it's in our lane: can't be behind us
            reappeared = math.isfinite(tr.d) and abs(tr.d - h.d) < 6
            # dead reckoning drifts (~2 m per second unseen), so demand more clearance the
            # longer we've been guessing; a car pacing us stays blocked until provably behind
            slack = 10 + m.blind_spot_s * max(h.closing(v_kmh), 0) + 2.0 * (now - h.t0)
            behind = z != "center" and h.d < -slack
            if reappeared or behind or h.d > self.VISIBLE_FROM_M or now - h.t0 > self.HIDDEN_MAX_S:
                self.hidden[z] = None

    def _cross(self, direction: int, now: float, v_kmh: float) -> None:
        """We just crossed into the next lane: shift everything we know one lane over."""
        back, toward = ("left", "right") if direction > 0 else ("right", "left")
        old = self.hidden["center"]
        tr = self.trackers["center"]
        if old is None and tr.d < 40:
            old = Hidden(tr.d, tr.closing, v_kmh, now)
        self.hidden[back], self.hidden["center"], self.hidden[toward] = old, self.hidden[toward], None
        for t in self.trackers.values():
            t.samples.clear()

    def _track_lateral(self, perc: Perception | None, now: float, dt: float, v_kmh: float) -> None:
        """Lateral position in half-lane widths of the lane we started in. During a lane
        change the vision offset wraps (+1 -> -1) as we cross the line; unwrap it."""
        off = perc.lanes.offset if perc else None
        if off is None:
            self.y += self.y_rate * dt                  # brief dead reckoning
            self.y_rate *= 0.9
            return
        if self.prev_off is not None and self.state == "CHANGE" and abs(off - self.prev_off) > 1.2:
            step = round((off - self.prev_off) / 2)
            self.k -= step
            self._cross(-step, now, v_kmh)
        self.prev_off = off
        y = off + 2 * self.k
        if now - self.lanes_t > self.LOST_LANES_S:      # re-acquired: no derivative kick
            self.y, self.y_rate = y, 0.0
        else:
            self.y_rate += 0.35 * ((y - self.y) / dt - self.y_rate)
            self.y = y
        self.lanes_t = now

    def _run_change(self, now: float, m) -> str:
        side = "left" if self.change_dir < 0 else "right"
        if self.k == 0 and abs(self.y) < 1.0 and (self.trackers[side].d < m.min_gap_m * 0.5
                                                  or self.hidden[side] is not None):
            self.state, self.last_change = "CRUISE", now          # target lane closed: abort
            return "abort"
        done = (now - self.change_t0 >= m.lane_change_s and abs(self.y - 2 * self.change_dir) < 0.35
                and abs(self.y_rate) < 0.6)
        if done or now - self.change_t0 > 2.5 * m.lane_change_s:
            self.y -= 2 * self.k                         # re-base on the new lane
            self.k, self.state, self.last_change = 0, "CRUISE", now
            return "lane change done"
        return f"changing {side}"

    def _output(self, steer: float, throttle: float, brake: float, dt: float, v: float,
                snap, state: str, note: str) -> Command:
        m, st, sb, sc = self.mode, self.cfg.steer, self.cfg.stability, self.cfg.speed
        over = snap.max_wheel_slip - m.slip_threshold
        if over > 0:
            self.trim = min(sb.max_trim, self.trim + sb.slip_trim_gain * over)
            steer *= 1 - min(0.5, sb.steer_trim_gain * over)
        else:
            self.trim = max(0.0, self.trim - sb.recover_per_s * dt)
        throttle *= 1 - self.trim

        hi = clamp(v / (2 * st.damp_ref_kmh), 0, 1)
        steer_slew = st.slew_per_s + (st.slew_high_speed_per_s - st.slew_per_s) * hi
        self.steer = approach(self.steer, clamp(steer, -st.max_steer, st.max_steer), steer_slew * dt)
        self.throttle = approach(self.throttle, throttle, sc.throttle_slew_per_s * dt)
        self.brake = approach(self.brake, brake, sc.brake_slew_per_s * dt)
        if self.brake > 0.05:
            self.throttle = 0.0
        return Command(self.steer, self.throttle, self.brake, state, note)


# --------------------------------------------------------------------------- #
def selftest() -> int:
    from dataclasses import replace
    fails: list[str] = []
    check = lambda ok, name: ok or fails.append(name)  # noqa: E731
    dt = 1 / 60

    def drive(sm: ScoreModel, t: float, dists: list[float], side: str = "right",
              lean: float = 0.0, v: float = 200.0, damage: float = 0.0) -> float:
        trs = {z: Tracker() for z in LANES3}
        for d in dists:
            for z, tr in trs.items():
                tr.update(t, d if z == side else math.inf)
            sm.update(t, dt, v, damage, trs, lean)
            t += dt
        return t

    closing = [30 - 10 * i * dt for i in range(int(2.5 / dt))]     # 30 m -> 5 m at 10 m/s
    gone = [math.inf] * 10
    cfg = replace(CONFIG.score, speed_points_per_s=0.0)

    sm = ScoreModel(cfg)
    t = drive(sm, 0.0, closing + gone)
    check(sm.passes == 1 and abs(sm.total - 200) < 1e-6, f"plain pass: {sm.passes} {sm.total}")
    drive(sm, t, closing + gone, lean=0.4)                          # 2nd pass: combo x2, near miss
    check(sm.near_misses == 1 and abs(sm.total - (200 + 200 * 2 * cfg.near_miss_mult)) < 1e-6,
          f"near miss + combo: {sm.total}")
    check(sm.combo == 3, "combo grows")
    sm.update(t + 60, dt, 200, 0, {z: Tracker() for z in LANES3}, 0)
    check(sm.combo == 1, "combo expires")

    sm = ScoreModel(cfg)
    drive(sm, 0.0, [3.0] * 120 + gone)                              # barrier: constant distance
    check(sm.passes == 0, "barrier is not a pass")
    drive(sm, 10.0, [10 + 10 * i * dt for i in range(90)])          # car pulling away from us
    check(sm.passes == 0, "receding car is not a pass")

    sm = ScoreModel(replace(cfg, crash_penalty=150))
    t = drive(sm, 0.0, closing + gone)
    drive(sm, t, [math.inf] * 5, damage=0.0)
    drive(sm, t + 1, [math.inf] * 5, damage=2.0)                    # damage jump
    check(sm.crashes == 1 and abs(sm.total - 50) < 1e-6 and sm.combo == 1, f"damage crash: {sm.total}")
    sm2 = ScoreModel(replace(cfg, crash_resets_score=True))
    t = drive(sm2, 0.0, closing + gone)
    for i in range(20):                                             # 200 -> 150 km/h in 0.33 s
        sm2.update(t + i * dt, dt, 200 - 2.5 * i, 0, {z: Tracker() for z in LANES3}, 0)
    check(sm2.crashes == 1 and sm2.total == 0 and sm2.best == 200, "impact crash resets score")

    sm = ScoreModel(replace(CONFIG.score, speed_points_per_s=10, speed_floor_kmh=150))
    drive(sm, 0.0, [math.inf] * 60, v=250)
    check(abs(sm.total - 10) < 0.2, f"speed points: {sm.total}")

    # tracker: closing speed through 4 % noise
    rng, tr = __import__("random").Random(1), Tracker()
    for i in range(60):
        tr.update(i * dt, (50 - 12 * i * dt) * (1 + rng.gauss(0, 0.04)))
    check(abs(tr.closing - 12) < 4, f"tracker closing {tr.closing:.1f}")

    # hidden car: dead reckoning follows our speed changes
    h = Hidden(10.0, closing0=5.0, v0_kmh=150, t0=0.0)
    check(abs(h.closing(150 - 18) - 0.0) < 1e-9, "hidden closing tracks our braking")

    print("SELFTEST", "FAILED:\n  " + "\n  ".join(fails) if fails else "OK")
    return 1 if fails else 0


if __name__ == "__main__":
    import sys
    sys.exit(selftest() if "--selftest" in sys.argv else print("usage: python planner.py --selftest") or 0)

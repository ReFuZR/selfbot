"""
main.py - the driver: telemetry + vision + planner + controller at 60 Hz.

    F8   engage / pause        F9   cycle mode        F12  kill: neutral inputs, quit
    F7   start / stop recording your lane as a path for this track (optional)

Engaged: the bot drives. Touch your wheel or a pedal and it hands control back.
Paused:  your Logitech drives the car through the virtual wheel (pass-through), 1:1.
It also pauses by itself when the game is paused, after a crash, when vision is lost,
or when the car is stuck.

    python main.py                  # mode from config.py
    python main.py --mode madmax
    python main.py --dry-run        # no output device: watch what it *would* do
    python main.py --log run.csv    # per-tick log for tuning (send it to me)
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys
import time
from pathlib import Path

from config import CONFIG, Config
from controller import ControllerError, NullBackend, PhysicalWheel, VirtualController
from drive import GearBox, Motion, Path as LanePath, Recorder, SteerModel, Steering
from planner import Planner
from telemetry import ACTelemetry, TelemetryUnavailable
from vision import Perception, summarize

BASE = Path(__file__).parent


class Driver:
    VISION_LOST_S = 1.0
    STUCK_S = 4.0

    def __init__(self, cfg: Config, telemetry, vision, pad: VirtualController,
                 wheel: PhysicalWheel | None = None, wheel_note: str = "", log=None,
                 base: Path = BASE) -> None:
        self.cfg, self.tm, self.vision, self.pad = cfg, telemetry, vision, pad
        self.wheel, self.wheel_note, self.log, self.base = wheel, wheel_note, log, base
        self.motion, self.gearbox = Motion(), GearBox(cfg.drive)
        self.car = ""
        self.model = SteerModel(learn_seconds=cfg.drive.learn_seconds)
        self.steering = Steering(cfg, self.model)
        self.recorder: Recorder | None = None
        self._record = False
        self._last_tick: float | None = None
        self._saved_at = 0.0
        self.planner = Planner(cfg)
        self.engaged, self.stop = False, False
        self.status = "paused - press F8 to engage"
        self._toggle = self._cycle = False
        self._neutral = False
        self._steer0 = 0.0
        self._perc_t = -math.inf
        self._stuck_since: float | None = None
        self._crashes = 0
        self.snap = self.perc = self.cmd = self.phys = None
        self.out = (0.0, 0.0, 0.0)
        self._bind_err = [0.0, 0.0]
        self.binding_warning = ""
        self.tick_hz, self._ticks, self._hz_t = 0.0, 0, None

    # -- hotkeys (called from the listener thread) -------------------------- #
    def request_toggle(self) -> None:
        self._toggle = True

    def request_mode(self) -> None:
        self._cycle = True

    def request_record(self) -> None:
        self._record = True

    def request_kill(self) -> None:
        self.pad.kill()               # neutral immediately, from this thread
        self.engaged, self.stop = False, True
        self.status = "KILLED"

    # -- engage / pause -------------------------------------------------------- #
    def engage_problem(self) -> str | None:
        s, p = self.snap, self.perc
        if s is None or not s.is_fresh:
            return "AC isn't live - get in the car and unpause"
        if not self.model.ready:
            return (f"drive yourself for a bit (a few bends / lane changes) so it learns your steering - "
                    f"{self.model.seconds:.1f}/{self.model.learn_seconds:.0f} s of steering so far")
        if p is None:
            return "vision has no frames yet"
        if p.lanes.confidence < 0.3 and not self.steering.on_path:
            return "no lane lines visible - check camera view / ROI (run.bat -> calibrate), or record a path (F7)"
        if self.pad.killed:
            return "killed - restart main.py"
        return None

    def engage(self, now: float) -> None:
        problem = self.engage_problem()
        if problem:
            self.status = f"can't engage: {problem}"
            return
        self.planner.reset()
        self.model.reset_integral()
        self._steer0 = self.phys[0] if self.phys else 0.0
        self._stuck_since, self._crashes = None, self.planner.score.crashes
        self.engaged, self.status = True, "ENGAGED"

    def pause(self, why: str) -> None:
        self.engaged = False
        self.status = f"paused: {why} - F8 to re-engage"
        self.planner.reset()
        self._neutral = False

    def pause_reason(self, now: float) -> str | None:
        s, w = self.snap, self.cfg.wheel
        if s is None or not s.is_fresh:
            return "game paused / not live"
        if now - self._perc_t > self.VISION_LOST_S:
            return "vision lost"
        if self.phys and w.takeover:
            st, gas, brake = self.phys
            if abs(st - self._steer0) > w.takeover_steer or gas > w.takeover_pedal or brake > w.takeover_pedal:
                return "driver takeover"
        if self.planner.score.crashes > self._crashes:
            return "crash"
        if s.speed_kmh < 3 and self.cmd and self.cmd.throttle > 0.3:
            self._stuck_since = self._stuck_since or now
            if now - self._stuck_since > self.STUCK_S:
                return "car stuck - reset it (Esc -> back to pits / restart)"
        else:
            self._stuck_since = None
        return None

    # -- one control tick ------------------------------------------------------ #
    def _car_changed(self, s) -> None:
        """New car / track: load its learned steering and recorded path."""
        self.save()
        self.car = s.car_model
        d = self.cfg.drive
        self.model = SteerModel.load(self.base / d.steer_models_file, self.car, d.learn_seconds)
        self.steering = Steering(self.cfg, self.model, LanePath.load(LanePath.file_for(d, self.base, s.track)))

    def save(self) -> None:
        if self.car:
            self.model.save(self.base / self.cfg.drive.steer_models_file, self.car)

    def _toggle_recording(self) -> None:
        s = self.snap
        if self.recorder is None:
            self.recorder = Recorder(self.cfg.drive.record_spacing_m)
            self.status = "RECORDING your lane - drive it centred, F7 again to stop"
            return
        rec, self.recorder = self.recorder, None
        path = rec.finish(s.track if s else "")
        if path is None:
            self.status = f"recording too short ({rec.length:.0f} m) - not saved"
            return
        f = LanePath.file_for(self.cfg.drive, self.base, s.track if s else "")
        path.save(f)
        self.steering.path = path
        self.status = f"saved path {f.name}: {path.length:.0f} m{' (loop)' if path.loop else ''}"

    def tick(self, now: float) -> None:
        dt = 1 / 60 if self._last_tick is None else min(max(now - self._last_tick, 1e-3), 0.1)
        self._last_tick = now
        try:
            self.snap = self.tm.read()
        except TelemetryUnavailable:
            self.snap = None
            self.tm.try_connect()
        s = self.snap
        if s is not None and s.is_fresh:
            if s.car_model != self.car:
                self._car_changed(s)
            self.motion.update(s.velocity, dt)
            if self.recorder is not None:
                self.recorder.add(s.car_xyz[0], s.car_xyz[2])
        if self._record:
            self._record = False
            self._toggle_recording()
        self.perc = self.vision.latest() if self.vision else None
        if self.perc is not None:
            self._perc_t = now
        if self.wheel:
            try:
                self.phys = self.wheel.read()
            except Exception as e:                       # wheel unplugged etc.
                self.wheel, self.phys, self.wheel_note = None, None, f"wheel lost ({e})"

        if self._cycle:
            self._cycle = False
            names = list(self.cfg.modes)
            self.planner.set_mode(names[(names.index(self.cfg.mode) + 1) % len(names)])
        if self._toggle:
            self._toggle = False
            self.pause("F8") if self.engaged else self.engage(now)

        if self.engaged:
            why = self.pause_reason(now)
            if why:
                self.pause(why)
        # lane position for the planner: from the recorded path when we're on it (exact),
        # otherwise from vision's lane lines
        perc = self.perc
        lanes = self.steering.lanes_from_path(s, self.motion.v) if s is not None and s.is_fresh else None
        if lanes is not None:
            perc = Perception(now, lanes, perc.obstacles if perc else [],
                              perc.zones if perc else summarize([]), perc.shape if perc else (0, 0))

        if self.engaged:
            self.cmd = c = self.planner.update(s, perc, now)
            if self.model.ready:                     # learned steering: real axis values
                steer = self.steering.axis(c.steer, c.state == "BLIND", self.motion, dt)
                self.pad.set_targets(steer, c.throttle, c.brake, scaled=False)
            else:
                self.pad.set_targets(c.steer, c.throttle, c.brake)
            self.out = self.pad.update(now)
            self.pad.buttons(*self.gearbox.update(now, s, c.throttle))
            self._neutral = False
        else:
            self.cmd = None
            if (self.phys and self.cfg.wheel.passthrough and self.cfg.controller.output == "wheel"
                    and not self.pad.killed):
                self.pad.direct(*self.phys)              # you drive, through the virtual wheel
                self.pad.buttons(*self.wheel.read_buttons())
                self.out = self.phys
            elif not self._neutral:
                self.pad.neutralize()
                self.out, self._neutral = (0.0, 0.0, 0.0), True
        # learn how the car steers from whatever steering is being sent (yours or the bot's)
        if s is not None and s.is_fresh and not self.pad.killed:
            self.model.observe(now, dt, self.out[0], self.motion.kappa, self.motion.v, s.max_wheel_slip)
            if self.model.ready and now - self._saved_at > 30:
                self._saved_at = now
                self.save()
        self._check_binding()
        self._log(now)
        self._ticks += 1
        if self._hz_t is None:
            self._hz_t = now
        elif now - self._hz_t >= 1.0:
            self.tick_hz, self._ticks, self._hz_t = self._ticks / (now - self._hz_t), 0, now

    def _check_binding(self) -> None:
        """AC reports the throttle / brake it actually receives. If that keeps disagreeing with
        what we send, Content Manager's binding is wrong (wrong axis, inverted, or still your
        Logitech pedal) - say so instead of silently driving badly."""
        s = self.snap
        if s is None or not s.is_fresh or self.pad.killed:
            return
        for i, (name, got) in enumerate((("throttle", s.gas), ("brake", s.brake))):
            diff = abs(got - self.out[i + 1])
            self._bind_err[i] += 0.03 * (diff - self._bind_err[i])      # ~0.5 s average
        bad = [n for n, e in zip(("throttle", "brake"), self._bind_err) if e > 0.35]
        self.binding_warning = (f"AC's {' and '.join(bad)} doesn't follow ours - check the Content "
                                "Manager binding (vJoy axis, invert). run.bat -> 0 diagnoses it" if bad else "")

    def _log(self, now: float) -> None:
        if not self.log:
            return
        s, p, c = self.snap, self.perc, self.cmd
        z = p.zones if p else None
        self.log.writerow([
            f"{now:.3f}", int(self.engaged), self.cfg.mode, s and f"{s.speed_kmh:.1f}",
            c and c.state, c and c.note, *(f"{x:.3f}" for x in self.out),
            p and p.lanes.offset is not None and f"{p.lanes.offset:.3f}", p and f"{p.lanes.confidence:.2f}",
            *((z and f"{z[k].nearest_m:.1f}") for k in ("left", "center", "right")),
            f"{self.planner.score.total:.0f}", self.status,
            *((s and f"{x:.3f}") for x in ((s.gas, s.brake, s.steer_angle) if s else (0, 0, 0))),
            self.planner.why, int(bool(self.binding_warning)),
            f"{self.motion.kappa:.5f}", f"{self.model.k:.4f}", int(self.model.ready),
            int(self.steering.on_path), f"{self.steering.path_e:.2f}", s and s.gear_label])

    # -- HUD ------------------------------------------------------------------- #
    def hud(self) -> str:
        s, p, c, sc, now = self.snap, self.perc, self.cmd, self.planner.score, time.perf_counter()
        hk = self.cfg.hotkeys
        lines = [f"ac-autodriver   mode {self.cfg.mode.upper():<7}  {hk.toggle.upper()} engage/pause  "
                 f"{hk.mode.upper()} mode  {hk.kill.upper()} kill      loop {self.tick_hz:4.0f} Hz",
                 f"  {self.status}"]
        if s:
            lines.append(f"car     {s.speed_kmh:5.0f} km/h  gear {s.gear_label:>2}  rpm {s.rpm:5d}  "
                         f"slip {s.max_wheel_slip:4.1f}  {'auto' if s.auto_shifter else 'MANUAL'} gearbox  "
                         f"{s.car_model} @ {s.track}")
        else:
            lines.append("car     waiting for Assetto Corsa...")
        st, thr, brk = self.out
        lines.append(f"plan    {(c.state if c else '-'):<7} {(c.note if c else ''):<22} "
                     f"sending: steer {st:+.2f}  gas {thr:.2f}  brake {brk:.2f}")
        if s:
            lines.append(f"AC gets {'':30} steer {s.steer_angle:+.2f}  gas {s.gas:.2f}  brake {s.brake:.2f}"
                         + (f"   !! {self.binding_warning}" if self.binding_warning else ""))
        if self.engaged:
            why = self.planner.why or ("lane ahead clear" if c and c.state == "CRUISE" else "")
            lines.append(f"lanes   {why}")
        if p:
            zs = "  ".join(f"{k[0].upper()} {'--' if math.isinf(v.nearest_m) else f'{v.nearest_m:3.0f}m'}"
                           for k, v in p.zones.items())
            off = "--" if p.lanes.offset is None else f"{p.lanes.offset:+.2f}"
            fps = getattr(self.vision, "fps", 0.0)
            age = (time.perf_counter() - p.t) * 1000
            lines.append(f"vision  {fps:3.0f} fps  grab {getattr(self.vision, 'grab_ms', 0):3.1f}ms "
                         f"proc {getattr(self.vision, 'proc_ms', 0):3.1f}ms  frame age {age:3.0f}ms  "
                         f"lane {off} conf {p.lanes.confidence:.2f}   {zs}")
        else:
            err = getattr(self.vision, "error", None)
            lines.append(f"vision  no frames{f' ({err})' if err else ''}")
        lines.append(f"capture {getattr(self.vision, 'backend', '-')}")
        lines.append(f"score   {sc.total:9,.0f}  combo x{sc.combo} ({sc.combo_left(now):3.1f}s)  "
                     f"passes {sc.passes}  near {sc.near_misses}  cuts {sc.cuts}  swerves {sc.swerves}  "
                     f"crashes {sc.crashes}  best {sc.best:,.0f}")
        mode = "pass-through" if self.phys and self.cfg.wheel.passthrough and not self.engaged else ""
        ph = (f"steer {self.phys[0]:+.2f} gas {self.phys[1]:.2f} brake {self.phys[2]:.2f}"
              if self.phys else "")
        lines.append(f"wheel   {self.wheel_note}  {ph}  {mode}")
        md = self.model
        lines.append("steer   " + (f"learned (k {md.k:+.4f}){'  INVERTED binding?' if md.k < 0 else ''}" if md.ready else
                                   f"learning {md.seconds:.1f}/{md.learn_seconds:.0f} s - drive a few bends yourself"))
        p_ = self.steering.path
        if self.recorder is not None:
            lines.append(f"path    RECORDING {self.recorder.length:.0f} m - F7 to stop and save")
        elif p_ is not None:
            where = (f"on it, {self.steering.path_e:+.1f} m from its centre" if self.steering.on_path else "not on it")
            lines.append(f"path    {p_.name or 'recorded'} {p_.length:.0f} m{' loop' if p_.loop else ''} - {where}")
        else:
            lines.append("path    none for this track (optional: F7 to record your lane)")
        lines.append(f"gears   {self.gearbox.note or ('AC automatic' if s and s.auto_shifter else '-')}"
                     f"{'   paddles pass through' if self.wheel and not self.engaged else ''}")
        lines.append("events  " + "   ".join(t for _, t in list(sc.events)[-4:]))
        return "\n".join(lines)

    # -- loop -------------------------------------------------------------------- #
    def run(self, hud=True) -> None:
        period = 1.0 / self.cfg.loop.hz
        hud_period = 1.0 / self.cfg.loop.hud_hz
        next_t = last_hud = time.perf_counter()
        while not self.stop:
            now = time.perf_counter()
            self.tick(now)
            if hud and now - last_hud >= hud_period:
                last_hud = now
                sys.stdout.write("\x1b[H" + "\n".join(l.ljust(118)[:118] for l in self.hud().split("\n"))
                                 + "\x1b[J")
                sys.stdout.flush()
            next_t += period
            delay = next_t - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
            elif delay < -5 * period:
                next_t = time.perf_counter()             # fell behind (hitch): don't burst


def start_hotkeys(cfg: Config, driver: Driver):
    try:
        from pynput import keyboard
    except ImportError:
        print("pynput missing - hotkeys off (Ctrl+C still quits). Re-run install.bat.")
        return None
    keys = {getattr(keyboard.Key, cfg.hotkeys.toggle): driver.request_toggle,
            getattr(keyboard.Key, cfg.hotkeys.mode): driver.request_mode,
            getattr(keyboard.Key, cfg.hotkeys.kill): driver.request_kill,
            getattr(keyboard.Key, cfg.hotkeys.record): driver.request_record}

    def on_press(key):
        fn = keys.get(key)
        if fn:
            fn()
    listener = keyboard.Listener(on_press=on_press)
    listener.daemon = True
    listener.start()
    return listener


def main() -> int:
    ap = argparse.ArgumentParser(description="ac-autodriver")
    ap.add_argument("--mode", choices=list(CONFIG.modes), help="driving mode (default: config.py)")
    ap.add_argument("--dry-run", action="store_true", help="don't touch any input device")
    ap.add_argument("--log", metavar="CSV", help="write a per-tick log")
    ap.add_argument("--no-wheel", action="store_true", help="ignore the physical wheel")
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        return selftest()
    cfg = CONFIG
    if a.mode:
        cfg.mode = a.mode
    if os.name == "nt":
        os.system("")                                    # enable ANSI colours / cursor moves

    try:
        pad = VirtualController(cfg.controller, NullBackend()) if a.dry_run else VirtualController(cfg.controller)
    except ControllerError as e:
        print(f"Output device: {e}")
        return 1
    wheel, note = (None, "ignored (--no-wheel)") if a.no_wheel else PhysicalWheel.load(cfg.wheel, BASE)
    if cfg.controller.output != "wheel":
        note += "  (output is xbox: no pass-through)"

    tm = ACTelemetry()
    print("Waiting for Assetto Corsa (get into a session)... Ctrl+C to quit", flush=True)
    try:
        tm.connect()
    except KeyboardInterrupt:
        pad.close()
        return 0

    from vision import VisionThread                     # imports OpenCV: only when needed
    vision = VisionThread(cfg.vision).start()
    log_file = open(a.log, "w", newline="") if a.log else None
    log = csv.writer(log_file) if log_file else None
    if log:
        log.writerow(["t", "engaged", "mode", "kmh", "state", "note", "steer", "gas", "brake",
                      "lane_off", "lane_conf", "left_m", "center_m", "right_m", "score", "status",
                      "ac_gas", "ac_brake", "ac_steer", "why_no_lane_change", "binding_warning",
                      "curvature", "steer_k", "steer_learned", "on_path", "path_offset_m", "gear"])
    driver = Driver(cfg, tm, vision, pad, wheel, note, log)
    hotkeys = start_hotkeys(cfg, driver)
    print("\x1b[2J", end="")
    try:
        driver.run()
    except KeyboardInterrupt:
        pass
    finally:
        driver.save()
        pad.close()
        vision.stop()
        tm.close()
        if hotkeys:
            hotkeys.stop()
        if log_file:
            log_file.close()
        print(f"\n\nInputs neutral. Final score {driver.planner.score.total:,.0f} "
              f"(best {driver.planner.score.best:,.0f}).")
    return 0


# --------------------------------------------------------------------------- #
def selftest() -> int:
    """Driver logic with fake telemetry / vision / wheel - no game, no devices."""
    import tempfile
    from dataclasses import replace
    from types import SimpleNamespace

    from vision import LaneEstimate

    fails: list[str] = []
    check = lambda ok, name: ok or fails.append(name)  # noqa: E731
    K_TRUE = 0.09                                   # this fake car's curvature per unit of steering

    class FakeTM:
        """A car moving on a flat world: steering we send curves it (K_TRUE), AC reports back
        the throttle / brake it receives (correct binding, or inverted)."""
        def __init__(self):
            self.snap = SimpleNamespace(
                speed_kmh=150.0, max_wheel_slip=0.0, damage=0.0, is_fresh=True, auto_shifter=True,
                gear_raw=6, gear_label="5", rpm=6000, max_rpm=8000, car_model="car", track="track",
                gas=0.0, brake=0.0, steer_angle=0.0, velocity=(0.0, 0.0, 0.0), car_xyz=(0.0, 0.0, 0.0))
            self.driver, self.inverted, self.psi, self.x, self.z = None, False, 0.0, 0.0, 0.0
        def read(self):
            s, dt = self.snap, 1 / 60
            if self.driver:
                g, b = self.driver.out[1], self.driver.out[2]
                s.gas, s.brake = (1 - g, 1 - b) if self.inverted else (g, b)
                v = s.speed_kmh / 3.6
                self.psi += v * K_TRUE * self.driver.out[0] * dt
                self.x += v * math.sin(self.psi) * dt
                self.z += v * math.cos(self.psi) * dt
                s.velocity, s.car_xyz = (v * math.sin(self.psi), 0.0, v * math.cos(self.psi)), (self.x, 0.0, self.z)
            return s
        def try_connect(self): return True

    class FakeVision:
        fps, error = 60.0, None
        def __init__(self): self.perc, self.obs = True, []
        def latest(self):
            if not self.perc:
                return None
            return Perception(0.0, LaneEstimate(0.0, 0.0, 1.0), self.obs, summarize(self.obs), (0, 0))

    class FakeWheel:
        def __init__(self): self.v, self.btn = (0.0, 0.0, 0.0), (False, False)
        def read(self): return self.v
        def read_buttons(self): return self.btn

    tmp = Path(tempfile.mkdtemp())

    def make(**ctl):
        cfg = replace(CONFIG, mode="simple", modes=dict(CONFIG.modes),
                      controller=replace(CONFIG.controller, output="wheel", **ctl))
        nb = NullBackend()
        d = Driver(cfg, FakeTM(), FakeVision(), VirtualController(cfg.controller, nb), FakeWheel(), "fake",
                   base=tmp)
        d.tm.driver = d
        return d, nb

    d, nb = make()
    t = 0.0

    def run(n: int, wheel=None) -> None:
        nonlocal t
        for _ in range(n):
            t += 1 / 60
            if wheel:
                d.wheel.v = wheel(t)
            d.tick(t)

    # won't drive until it has learned the steering from you
    run(2)
    d.request_toggle(); run(1)
    check(not d.engaged and "learns your steering" in d.status, f"steering gate: {d.status}")

    # you drive (pass-through) a few bends: it learns k, and the HUD says so
    run(60 * 6, wheel=lambda t: (0.08 * math.sin(1.7 * t), 0.4, 0.0))
    check(d.model.ready and abs(d.model.k / K_TRUE - 1) < 0.25, f"learned k {d.model.k:.4f} vs {K_TRUE}")
    check("learned" in d.hud(), "hud shows learned steering")

    # paused: your wheel and paddles drive through
    d.wheel.v, d.wheel.btn = (-0.3, 0.4, 0.0), (True, False)
    run(3)
    check(nb.last == (-0.3, 0.4, 0.0) and nb.btn == (True, False), f"pass-through: {nb.last} {nb.btn}")
    d.wheel.btn = (False, False)

    # engage: bot drives (empty road, below target -> throttle), steering through the model
    d.wheel.v = (0.0, 0.0, 0.0)
    d.request_toggle(); run(60)
    check(d.engaged and nb.last[1] > 0.2, f"engaged + throttle: {d.status} {nb.last}")

    # manual gearbox: the bot shifts out of neutral with the vJoy button
    d.tm.snap.auto_shifter, d.tm.snap.gear_raw = False, 1
    pressed = []
    for _ in range(30):
        run(1)
        pressed.append(nb.btn)
    check((True, False) in pressed and pressed[-1] == (False, False), "bot shifts N -> 1 and releases")
    d.tm.snap.auto_shifter, d.tm.snap.gear_raw = True, 6

    # takeover: turn your wheel -> pause, then pass-through again
    d.wheel.v = (0.2, 0.0, 0.0)
    run(2)
    check(not d.engaged and "takeover" in d.status and nb.last == (0.2, 0.0, 0.0), f"takeover: {d.status}")
    d.wheel.v = (0.0, 0.0, 0.0)
    d.request_toggle(); run(2)
    d.wheel.v = (0.0, 0.0, 0.5)                       # brake pedal
    run(2)
    check(not d.engaged and "takeover" in d.status, "brake takeover")
    d.wheel.v = (0.0, 0.0, 0.0)                       # feet off again

    # auto-pauses: game paused, vision lost, crash
    d.request_toggle(); run(2)
    d.tm.snap.is_fresh = False; run(2)
    check(not d.engaged and "not live" in d.status, f"game paused: {d.status}")
    d.tm.snap.is_fresh = True
    d.request_toggle(); run(2)
    d.vision.perc = False; run(int(60 * 1.2))
    check(not d.engaged and "vision lost" in d.status, f"vision lost: {d.status}")
    d.vision.perc = True
    d.request_toggle(); run(5)
    d.tm.snap.damage = 5.0; run(2)
    check(not d.engaged and "crash" in d.status, f"crash: {d.status}")

    # stuck: no speed while giving gas
    d.tm.snap.speed_kmh, d.tm.snap.damage = 0.0, 5.0
    d.request_toggle(); run(int(60 * 5))
    check(not d.engaged and "stuck" in d.status, f"stuck: {d.status}")
    d.tm.snap.speed_kmh = 150.0

    # F9 cycles mode
    m0 = d.cfg.mode
    d.request_mode(); run(1)
    check(d.cfg.mode != m0, "mode cycle")

    # F7: record your lane -> saved for this track, used straight away, reloaded next time
    d.request_record(); run(60 * 4)
    d.request_record(); run(1)
    f = tmp / CONFIG.drive.paths_dir / "track.json"
    check(f.exists() and d.steering.path is not None and "saved path" in d.status, f"record: {d.status}")
    p = d.steering.path                               # back to where the recording started,
    tx, tz = p.x[1] - p.x[0], p.z[1] - p.z[0]         # a metre to one side of it
    n = math.hypot(tx, tz)
    d.tm.psi = math.atan2(tx, tz)
    d.tm.x, d.tm.z = p.x[0] + tz / n, p.z[0] - tx / n
    run(5)
    check(d.steering.on_path and 0.5 < abs(d.steering.path_e) < 1.5, f"on recorded path: e={d.steering.path_e:.2f}")
    d.save()
    d5, _ = make()
    d5.tick(1 / 60)
    check(d5.model.ready and d5.steering.path is not None, "learned steering + path reload for this car / track")

    # without pass-through: paused = neutral
    d2, nb2 = make()
    d2.wheel = None
    d2.model = d2.steering.model = d.model
    d2.request_toggle(); [d2.tick(i / 60) for i in range(1, 30)]
    d2.request_toggle(); d2.tick(1.0)
    check(nb2.last == (0.0, 0.0, 0.0) and nb2.btn == (False, False), "paused without wheel = neutral")

    # F12 kill: neutral, latched, loop stops, pass-through stays off
    d.request_toggle(); run(10)
    d.request_kill()
    d.wheel.v = (0.5, 0.5, 0.0); run(3)
    check(d.stop and d.pad.killed and nb.last == (0.0, 0.0, 0.0), f"kill: {nb.last}")

    # Content Manager binding check: correct binding stays quiet, inverted one is flagged
    d4, _ = make()
    for i in range(1, 120):
        d4.tick(i / 60)
    check(d4.binding_warning == "", f"correct binding flagged: {d4.binding_warning}")
    d4.tm.inverted = True
    for i in range(120, 240):
        d4.tick(i / 60)
    check("throttle" in d4.binding_warning and "AC gets" in d4.hud(), f"inverted binding: {d4.binding_warning!r}")

    # HUD renders in every state; run() exits on stop
    check(any(k in d.hud() for k in ("SIMPLE", "CRUISE", "MADMAX")), "hud")
    d3, _ = make()
    d3.stop = True
    d3.run(hud=False)

    print("SELFTEST", "FAILED:\n  " + "\n  ".join(fails) if fails else "OK")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())

"""
main.py - the driver: telemetry + vision + planner + controller at 60 Hz.

    F8   engage / pause        F9   cycle mode        F12  kill: neutral inputs, quit

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
from planner import Planner
from telemetry import ACTelemetry, TelemetryUnavailable

BASE = Path(__file__).parent


class Driver:
    VISION_LOST_S = 1.0
    STUCK_S = 4.0

    def __init__(self, cfg: Config, telemetry, vision, pad: VirtualController,
                 wheel: PhysicalWheel | None = None, wheel_note: str = "", log=None) -> None:
        self.cfg, self.tm, self.vision, self.pad = cfg, telemetry, vision, pad
        self.wheel, self.wheel_note, self.log = wheel, wheel_note, log
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
        self.tick_hz, self._ticks, self._hz_t = 0.0, 0, None

    # -- hotkeys (called from the listener thread) -------------------------- #
    def request_toggle(self) -> None:
        self._toggle = True

    def request_mode(self) -> None:
        self._cycle = True

    def request_kill(self) -> None:
        self.pad.kill()               # neutral immediately, from this thread
        self.engaged, self.stop = False, True
        self.status = "KILLED"

    # -- engage / pause -------------------------------------------------------- #
    def engage_problem(self) -> str | None:
        s, p = self.snap, self.perc
        if s is None or not s.is_fresh:
            return "AC isn't live - get in the car and unpause"
        if not s.auto_shifter:
            return "turn on the automatic gearbox (AC assists) - the bot can't shift"
        if p is None:
            return "vision has no frames yet"
        if p.lanes.confidence < 0.3:
            return "no lane lines visible - check camera view / ROI (run.bat -> calibrate)"
        if self.pad.killed:
            return "killed - restart main.py"
        return None

    def engage(self, now: float) -> None:
        problem = self.engage_problem()
        if problem:
            self.status = f"can't engage: {problem}"
            return
        self.planner.reset()
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
    def tick(self, now: float) -> None:
        try:
            self.snap = self.tm.read()
        except TelemetryUnavailable:
            self.snap = None
            self.tm.try_connect()
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
        if self.engaged:
            self.cmd = self.planner.update(self.snap, self.perc, now)
            self.pad.set_targets(self.cmd.steer, self.cmd.throttle, self.cmd.brake)
            self.out = self.pad.update(now)
            self._neutral = False
        else:
            self.cmd = None
            if (self.phys and self.cfg.wheel.passthrough and self.cfg.controller.output == "wheel"
                    and not self.pad.killed):
                self.pad.direct(*self.phys)              # you drive, through the virtual wheel
                self.out = self.phys
            elif not self._neutral:
                self.pad.neutralize()
                self.out, self._neutral = (0.0, 0.0, 0.0), True
        self._log(now)
        self._ticks += 1
        if self._hz_t is None:
            self._hz_t = now
        elif now - self._hz_t >= 1.0:
            self.tick_hz, self._ticks, self._hz_t = self._ticks / (now - self._hz_t), 0, now

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
            f"{self.planner.score.total:.0f}", self.status])

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
                     f"steer {st:+.2f}  gas {thr:.2f}  brake {brk:.2f}")
        if p:
            zs = "  ".join(f"{k[0].upper()} {'--' if math.isinf(v.nearest_m) else f'{v.nearest_m:3.0f}m'}"
                           for k, v in p.zones.items())
            off = "--" if p.lanes.offset is None else f"{p.lanes.offset:+.2f}"
            fps = getattr(self.vision, "fps", 0.0)
            lines.append(f"vision  {fps:3.0f} fps  lane {off} conf {p.lanes.confidence:.2f}   {zs}")
        else:
            err = getattr(self.vision, "error", None)
            lines.append(f"vision  no frames{f' ({err})' if err else ''}")
        lines.append(f"score   {sc.total:9,.0f}  combo x{sc.combo} ({sc.combo_left(now):3.1f}s)  "
                     f"passes {sc.passes}  near {sc.near_misses}  cuts {sc.cuts}  swerves {sc.swerves}  "
                     f"crashes {sc.crashes}  best {sc.best:,.0f}")
        mode = "pass-through" if self.phys and self.cfg.wheel.passthrough and not self.engaged else ""
        ph = (f"steer {self.phys[0]:+.2f} gas {self.phys[1]:.2f} brake {self.phys[2]:.2f}"
              if self.phys else "")
        lines.append(f"wheel   {self.wheel_note}  {ph}  {mode}")
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
            getattr(keyboard.Key, cfg.hotkeys.kill): driver.request_kill}

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
                      "lane_off", "lane_conf", "left_m", "center_m", "right_m", "score", "status"])
    driver = Driver(cfg, tm, vision, pad, wheel, note, log)
    hotkeys = start_hotkeys(cfg, driver)
    print("\x1b[2J", end="")
    try:
        driver.run()
    except KeyboardInterrupt:
        pass
    finally:
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
    from dataclasses import replace
    from types import SimpleNamespace

    from vision import LaneEstimate, Perception, summarize

    fails: list[str] = []
    check = lambda ok, name: ok or fails.append(name)  # noqa: E731

    class FakeTM:
        def __init__(self): self.snap = SimpleNamespace(
            speed_kmh=150.0, max_wheel_slip=0.0, damage=0.0, is_fresh=True, auto_shifter=True,
            gear_label="5", rpm=6000, car_model="car", track="track")
        def read(self): return self.snap
        def try_connect(self): return True

    class FakeVision:
        fps, error = 60.0, None
        def __init__(self): self.perc, self.obs = True, []
        def latest(self):
            if not self.perc:
                return None
            return Perception(0.0, LaneEstimate(0.0, 0.0, 1.0), self.obs, summarize(self.obs), (0, 0))

    class FakeWheel:
        def __init__(self): self.v = (0.0, 0.0, 0.0)
        def read(self): return self.v

    def make(**ctl):
        cfg = replace(CONFIG, mode="simple", modes=dict(CONFIG.modes),
                      controller=replace(CONFIG.controller, output="wheel", **ctl))
        nb = NullBackend()
        d = Driver(cfg, FakeTM(), FakeVision(), VirtualController(cfg.controller, nb), FakeWheel(), "fake")
        return d, nb

    d, nb = make()
    t = 0.0

    def run(n: int) -> None:
        nonlocal t
        for _ in range(n):
            t += 1 / 60
            d.tick(t)

    # refuses without automatic gearbox
    d.tm.snap.auto_shifter = False
    d.request_toggle(); run(1)
    check(not d.engaged and "gearbox" in d.status, f"auto-shifter gate: {d.status}")
    d.tm.snap.auto_shifter = True

    # paused: your wheel drives through
    d.wheel.v = (-0.3, 0.4, 0.0)
    run(3)
    check(nb.last == (-0.3, 0.4, 0.0), f"pass-through: {nb.last}")

    # engage: bot drives (empty road, below target -> throttle)
    d.wheel.v = (0.0, 0.0, 0.0)
    d.request_toggle(); run(60)
    check(d.engaged and nb.last[1] > 0.2, f"engaged + throttle: {d.status} {nb.last}")

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

    # F9 cycles mode
    m0 = d.cfg.mode
    d.request_mode(); run(1)
    check(d.cfg.mode != m0, "mode cycle")

    # without pass-through: paused = neutral
    d2, nb2 = make()
    d2.wheel = None
    d2.request_toggle(); [d2.tick(i / 60) for i in range(1, 30)]
    d2.request_toggle(); d2.tick(1.0)
    check(nb2.last == (0.0, 0.0, 0.0), "paused without wheel = neutral")

    # F12 kill: neutral, latched, loop stops, pass-through stays off
    d.tm.snap.speed_kmh = 150.0
    d.request_toggle(); run(10)
    d.request_kill()
    d.wheel.v = (0.5, 0.5, 0.0); run(3)
    check(d.stop and d.pad.killed and nb.last == (0.0, 0.0, 0.0), f"kill: {nb.last}")

    # HUD renders in every state; run() exits on stop
    check("SIMPLE" in d.hud() or "CRUISE" in d.hud() or "MADMAX" in d.hud(), "hud")
    d3, _ = make()
    d3.stop = True
    d3.run(hud=False)

    print("SELFTEST", "FAILED:\n  " + "\n  ".join(fails) if fails else "OK")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())

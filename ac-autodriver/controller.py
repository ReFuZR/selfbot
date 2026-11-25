"""
controller.py - virtual Xbox 360 pad output (vgamepad / ViGEmBus).

Axis mapping (matches AC's default gamepad bindings):
    steer     -> left stick X        [-1 .. +1]  (+ = right)
    throttle  -> right trigger       [ 0 ..  1]
    brake     -> left trigger        [ 0 ..  1]

The planner calls set_targets() with what it wants; update() runs once per
control tick and applies, in order:
    clamp -> brake-over-throttle interlock -> time-based EMA smoothing
    -> deadzone snap / game-deadzone rescale -> send.

Safety:
    * neutralize()  zero everything now (used on pause). Can be re-engaged.
    * kill()        zero everything and latch: nothing is sent afterwards.
    * atexit + close() always zero the pad, so a crash can't leave a stuck input.

    python controller.py --selftest     # offline logic check, no ViGEmBus needed
    python controller.py --bind-test    # slowly sweeps each axis (for CM binding)
"""

from __future__ import annotations

import argparse
import atexit
import math
import sys
import threading
import time
from typing import Protocol

from config import CONFIG, ControllerConfig


class ControllerError(RuntimeError):
    """Virtual pad could not be created (missing package or ViGEmBus driver)."""


# --------------------------------------------------------------------------- #
# Backends
# --------------------------------------------------------------------------- #
class PadBackend(Protocol):
    def send(self, steer: float, throttle: float, brake: float) -> None: ...
    def reset(self) -> None: ...
    def wake(self) -> None: ...
    def close(self) -> None: ...


class VGamepadBackend:
    """Real virtual Xbox 360 controller."""

    def __init__(self) -> None:
        try:
            import vgamepad as vg
        except ImportError as e:
            raise ControllerError("vgamepad is not installed: pip install vgamepad") from e
        try:
            self._vg = vg
            self._pad = vg.VX360Gamepad()
        except Exception as e:  # vgamepad raises its own error types
            raise ControllerError(
                "Could not create the virtual pad - is the ViGEmBus driver installed? "
                f"({e})") from e

    def send(self, steer: float, throttle: float, brake: float) -> None:
        p = self._pad
        p.left_joystick_float(x_value_float=steer, y_value_float=0.0)
        p.right_trigger_float(value_float=throttle)
        p.left_trigger_float(value_float=brake)
        p.update()

    def reset(self) -> None:
        self._pad.reset()
        self._pad.update()

    def wake(self) -> None:
        # Windows hides a new virtual pad from games until it reports some input.
        btn = self._vg.XUSB_BUTTON.XUSB_GAMEPAD_A
        self._pad.press_button(button=btn)
        self._pad.update()
        time.sleep(0.05)
        self._pad.release_button(button=btn)
        self._pad.update()

    def close(self) -> None:
        try:
            self.reset()
        finally:
            self._pad = None  # drop the reference -> ViGEm target is unplugged


class NullBackend:
    """Records what would be sent. Used by --selftest and dry runs."""

    def __init__(self, echo: bool = False) -> None:
        self.echo = echo
        self.last = (0.0, 0.0, 0.0)
        self.sent = 0
        self.resets = 0

    def send(self, steer: float, throttle: float, brake: float) -> None:
        self.last = (steer, throttle, brake)
        self.sent += 1
        if self.echo:
            print(f"steer {steer:+.3f}  thr {throttle:.3f}  brk {brake:.3f}")

    def reset(self) -> None:
        self.last = (0.0, 0.0, 0.0)
        self.resets += 1

    def wake(self) -> None:
        pass

    def close(self) -> None:
        self.reset()


# --------------------------------------------------------------------------- #
# Controller
# --------------------------------------------------------------------------- #
def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def _finite(x: float) -> float:
    return x if math.isfinite(x) else 0.0


class VirtualController:
    # A brake command above this suppresses throttle entirely.
    BRAKE_INTERLOCK = 0.05

    def __init__(self, cfg: ControllerConfig | None = None,
                 backend: PadBackend | None = None) -> None:
        self.cfg = cfg or CONFIG.controller
        self._backend: PadBackend | None = backend if backend is not None else VGamepadBackend()
        self._lock = threading.RLock()
        self._killed = False

        # raw targets from the planner, and smoothed (pre-deadzone) filter state
        self._tgt = [0.0, 0.0, 0.0]        # steer, throttle, brake
        self._flt = [0.0, 0.0, 0.0]
        self._out = (0.0, 0.0, 0.0)        # last values actually sent
        self._last_t: float | None = None

        atexit.register(self.close)
        if self.cfg.wake_on_connect:
            self._backend.wake()
        self._send_zero()

    # -- planner-facing API -------------------------------------------------- #
    def set_targets(self, steer: float, throttle: float, brake: float) -> None:
        """Latest desired inputs. steer in [-1,1]; throttle, brake in [0,1]."""
        steer, throttle, brake = _finite(steer), _finite(throttle), _finite(brake)
        if self.cfg.invert_steer:
            steer = -steer
        brake = _clamp(brake, 0.0, 1.0)
        throttle = _clamp(throttle, 0.0, 1.0)
        if brake > self.BRAKE_INTERLOCK:
            throttle = 0.0
        with self._lock:
            self._tgt = [_clamp(steer, -1.0, 1.0), throttle, brake]

    def update(self, now: float | None = None) -> tuple[float, float, float]:
        """Run once per control tick. Returns the (steer, throttle, brake) sent."""
        with self._lock:
            if self._killed or self._backend is None:
                return (0.0, 0.0, 0.0)
            now = time.perf_counter() if now is None else now
            dt = 1 / 60 if self._last_t is None else _clamp(now - self._last_t, 1e-3, 0.1)
            self._last_t = now

            c = self.cfg
            for i, alpha60 in enumerate((c.steer_smoothing, c.throttle_smoothing,
                                         c.brake_smoothing)):
                # alpha is specified per 60 Hz frame; make it frame-rate independent
                a = 1.0 - (1.0 - _clamp(alpha60, 0.0, 1.0)) ** (dt * 60.0)
                self._flt[i] += a * (self._tgt[i] - self._flt[i])

            steer = self._shape_steer(self._flt[0])
            throttle = self._shape_trigger(self._flt[1])
            brake = self._shape_trigger(self._flt[2])
            if brake > 0.0:
                throttle = 0.0
            self._out = (steer, throttle, brake)
            self._backend.send(*self._out)
            return self._out

    # -- shaping -------------------------------------------------------------- #
    def _shape_steer(self, x: float) -> float:
        c = self.cfg
        ax = abs(x)
        if ax < c.steer_deadzone:
            return 0.0
        gd = _clamp(c.steer_game_deadzone, 0.0, 0.9)
        # rescale (0,1] -> (gd,1] so small commands clear AC's own deadzone
        return math.copysign(gd + (1.0 - gd) * min(ax, 1.0), x)

    def _shape_trigger(self, x: float) -> float:
        return 0.0 if x < self.cfg.trigger_deadzone else min(x, 1.0)

    # -- safety ----------------------------------------------------------------- #
    def _send_zero(self) -> None:
        self._tgt = [0.0, 0.0, 0.0]
        self._flt = [0.0, 0.0, 0.0]
        self._out = (0.0, 0.0, 0.0)
        if self._backend is not None:
            self._backend.reset()

    def neutralize(self) -> None:
        """Zero all axes immediately and drop filter state (e.g. on pause). Re-usable."""
        with self._lock:
            self._send_zero()
            self._last_t = None

    def kill(self) -> None:
        """Emergency stop: zero all axes and refuse to send anything further."""
        with self._lock:
            self._killed = True
            self._send_zero()

    @property
    def killed(self) -> bool:
        return self._killed

    @property
    def output(self) -> tuple[float, float, float]:
        return self._out

    def close(self) -> None:
        with self._lock:
            if self._backend is None:
                return
            self._killed = True
            try:
                self._send_zero()
            finally:
                try:
                    self._backend.close()
                finally:
                    self._backend = None

    def __enter__(self) -> "VirtualController":
        return self

    def __exit__(self, *exc) -> None:
        self.close()


# --------------------------------------------------------------------------- #
# Self-test / bind-test
# --------------------------------------------------------------------------- #
def selftest() -> int:
    from dataclasses import replace
    fails: list[str] = []

    def check(ok: bool, name: str) -> None:
        if not ok:
            fails.append(name)

    cfg = replace(CONFIG.controller, wake_on_connect=False, steer_game_deadzone=0.0,
                  invert_steer=False)
    nb = NullBackend()
    vc = VirtualController(cfg, nb)

    # smoothing converges, and is monotonic (no overshoot)
    vc.set_targets(0.5, 0.8, 0.0)
    t, prev = 0.0, -1.0
    for _ in range(120):
        t += 1 / 60
        s, th, b = vc.update(t)
        check(s >= prev - 1e-9, "steer monotonic")
        prev = s
    check(abs(s - 0.5) < 0.01 and abs(th - 0.8) < 0.01 and b == 0.0, "converges")

    # frame-rate independence: 30 Hz and 120 Hz reach about the same value after 0.25 s
    def run(hz: int) -> float:
        v = VirtualController(cfg, NullBackend())
        v.set_targets(1.0, 0.0, 0.0)
        tt, out = 0.0, 0.0
        v.update(tt)
        for _ in range(int(hz * 0.25)):
            tt += 1 / hz
            out = v.update(tt)[0]
        v.close()
        return out
    check(abs(run(30) - run(120)) < 0.02, "frame-rate independent")

    # deadzone snap
    vc.neutralize()
    vc.set_targets(cfg.steer_deadzone * 0.5, cfg.trigger_deadzone * 0.5, 0.0)
    for i in range(300):
        vc.update(i / 60)
    check(vc.output == (0.0, 0.0, 0.0), "deadzone snap")

    # game-deadzone rescale
    cfg_gd = replace(cfg, steer_game_deadzone=0.1, steer_smoothing=1.0)
    v2 = VirtualController(cfg_gd, NullBackend())
    v2.set_targets(0.05, 0.0, 0.0)
    check(abs(v2.update(0.0)[0] - (0.1 + 0.9 * 0.05)) < 1e-6, "game deadzone rescale")
    v2.set_targets(1.0, 0.0, 0.0)
    check(abs(v2.update(0.02)[0] - 1.0) < 1e-6, "full scale preserved")
    v2.close()

    # brake interlock
    vc.neutralize()
    vc.set_targets(0.0, 1.0, 0.6)
    for i in range(120):
        out = vc.update(i / 60)
    check(out[1] == 0.0 and out[2] > 0.5, "brake overrides throttle")

    # NaN / out-of-range input
    vc.set_targets(float("nan"), 9.0, -3.0)
    check(vc._tgt == [0.0, 1.0, 0.0], "sanitise input")

    # invert
    v3 = VirtualController(replace(cfg, invert_steer=True, steer_smoothing=1.0), NullBackend())
    v3.set_targets(0.5, 0.0, 0.0)
    check(v3.update(0.0)[0] < 0, "invert steer")
    v3.close()

    # kill latches and zeroes
    vc.set_targets(1.0, 1.0, 0.0)
    vc.update(10.0)
    vc.kill()
    check(nb.last == (0.0, 0.0, 0.0), "kill zeroes backend")
    sent = nb.sent
    vc.set_targets(1.0, 1.0, 0.0)
    check(vc.update(11.0) == (0.0, 0.0, 0.0) and nb.sent == sent, "kill latches")

    # close zeroes + releases
    vc.close()
    check(vc._backend is None, "close releases backend")

    print("SELFTEST", "FAILED: " + ", ".join(fails) if fails else "OK")
    return 1 if fails else 0


def bind_test() -> int:
    """Sweep each axis slowly so Content Manager / joy.cpl can see and bind it."""
    cfg = CONFIG.controller
    try:
        vc = VirtualController(cfg)
    except ControllerError as e:
        print(f"ERROR: {e}")
        return 1
    print("Virtual Xbox 360 pad is live. Sweeping axes - bind them in AC now. Ctrl+C to stop.")
    try:
        with vc:
            t0 = time.perf_counter()
            while True:
                t = time.perf_counter() - t0
                phase = int(t // 6) % 3
                x = math.sin((t % 6) / 6 * 2 * math.pi)
                if phase == 0:
                    vc.set_targets(x, 0.0, 0.0);              label = "STEER   (left stick X)"
                elif phase == 1:
                    vc.set_targets(0.0, abs(x), 0.0);         label = "THROTTLE (right trigger)"
                else:
                    vc.set_targets(0.0, 0.0, abs(x));         label = "BRAKE   (left trigger)"
                s, th, b = vc.update()
                print(f"\r{label:<26} steer {s:+.2f} thr {th:.2f} brk {b:.2f}   ",
                      end="", flush=True)
                time.sleep(1 / 60)
    except KeyboardInterrupt:
        print("\naxes zeroed, pad removed")
        return 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Virtual controller tools")
    ap.add_argument("--selftest", action="store_true", help="offline logic check")
    ap.add_argument("--bind-test", action="store_true", help="sweep axes for binding")
    a = ap.parse_args()
    if a.selftest:
        sys.exit(selftest())
    if a.bind_test:
        sys.exit(bind_test())
    ap.print_help()

"""
controller.py - virtual Xbox 360 pad output (vgamepad / ViGEmBus).

Axis mapping (AC's default gamepad bindings):
    steer     -> left stick X   [-1 .. +1]  (+ = right)
    throttle  -> right trigger  [ 0 ..  1]
    brake     -> left trigger   [ 0 ..  1]

The planner calls set_targets(); update() runs once per tick and applies
clamp -> brake-over-throttle interlock -> time-based EMA smoothing
-> deadzone snap / game-deadzone rescale -> send.

Safety: neutralize() zeroes now (pause, re-usable); kill() zeroes and latches;
atexit/close() always zero the pad so a crash can't leave a stuck input.

    python controller.py --selftest     # offline logic check, no ViGEmBus needed
    python controller.py --bind-test    # sweeps each axis (for Content Manager binding)
"""

from __future__ import annotations

import argparse
import atexit
import math
import sys
import threading
import time
from dataclasses import replace

from config import CONFIG, ControllerConfig


class ControllerError(RuntimeError):
    """Virtual pad could not be created (missing package or ViGEmBus driver)."""


class VGamepadBackend:
    """Real virtual Xbox 360 controller."""

    def __init__(self) -> None:
        try:
            import vgamepad as vg
        except ImportError as e:
            raise ControllerError("vgamepad is not installed: pip install vgamepad") from e
        try:
            self._vg, self._pad = vg, vg.VX360Gamepad()
        except Exception as e:  # vgamepad raises its own error types
            raise ControllerError(
                f"Could not create the virtual pad - is the ViGEmBus driver installed? ({e})") from e

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
        """Windows hides a new virtual pad from games until it reports some input."""
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
    """Records what would be sent; for --selftest and driver-less dry runs."""

    def __init__(self) -> None:
        self.last, self.sent = (0.0, 0.0, 0.0), 0

    def send(self, steer: float, throttle: float, brake: float) -> None:
        self.last, self.sent = (steer, throttle, brake), self.sent + 1

    def reset(self) -> None:
        self.last = (0.0, 0.0, 0.0)

    def wake(self) -> None:
        pass

    close = reset


def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def _finite(x: float) -> float:
    return x if math.isfinite(x) else 0.0


class VirtualController:
    BRAKE_INTERLOCK = 0.05  # a brake command above this suppresses throttle entirely

    def __init__(self, cfg: ControllerConfig | None = None, backend=None) -> None:
        self.cfg = cfg or CONFIG.controller
        self._backend = backend if backend is not None else VGamepadBackend()
        self._lock = threading.RLock()
        self._killed = False
        self._tgt = [0.0, 0.0, 0.0]   # planner targets: steer, throttle, brake
        self._flt = [0.0, 0.0, 0.0]   # smoothed (pre-deadzone) filter state
        self._out = (0.0, 0.0, 0.0)   # last values actually sent
        self._last_t: float | None = None
        atexit.register(self.close)
        if self.cfg.wake_on_connect:
            self._backend.wake()
        self.neutralize()

    # -- planner-facing API -------------------------------------------------- #
    def set_targets(self, steer: float, throttle: float, brake: float) -> None:
        """Latest desired inputs. steer in [-1,1]; throttle, brake in [0,1]."""
        steer = -_finite(steer) if self.cfg.invert_steer else _finite(steer)
        throttle, brake = _clamp(_finite(throttle), 0, 1), _clamp(_finite(brake), 0, 1)
        if brake > self.BRAKE_INTERLOCK:
            throttle = 0.0
        with self._lock:
            self._tgt = [_clamp(steer, -1, 1), throttle, brake]

    def update(self, now: float | None = None) -> tuple[float, float, float]:
        """Run once per control tick. Returns the (steer, throttle, brake) sent."""
        with self._lock:
            if self._killed or self._backend is None:
                return (0.0, 0.0, 0.0)
            now = time.perf_counter() if now is None else now
            dt = 1 / 60 if self._last_t is None else _clamp(now - self._last_t, 1e-3, 0.1)
            self._last_t = now

            c = self.cfg
            for i, alpha60 in enumerate((c.steer_smoothing, c.throttle_smoothing, c.brake_smoothing)):
                a = 1.0 - (1.0 - _clamp(alpha60, 0, 1)) ** (dt * 60.0)  # frame-rate independent
                self._flt[i] += a * (self._tgt[i] - self._flt[i])

            steer = self._shape_steer(self._flt[0])
            throttle, brake = self._shape_trigger(self._flt[1]), self._shape_trigger(self._flt[2])
            self._out = (steer, 0.0 if brake > 0.0 else throttle, brake)
            self._backend.send(*self._out)
            return self._out

    def _shape_steer(self, x: float) -> float:
        c = self.cfg
        if abs(x) < c.steer_deadzone:
            return 0.0
        gd = _clamp(c.steer_game_deadzone, 0.0, 0.9)  # rescale (0,1] -> (gd,1] past AC's deadzone
        return math.copysign(gd + (1.0 - gd) * min(abs(x), 1.0), x)

    def _shape_trigger(self, x: float) -> float:
        return 0.0 if x < self.cfg.trigger_deadzone else min(x, 1.0)

    # -- safety -------------------------------------------------------------- #
    def neutralize(self) -> None:
        """Zero all axes immediately and drop filter state (e.g. on pause). Re-usable."""
        with self._lock:
            self._tgt, self._flt, self._out = [0.0] * 3, [0.0] * 3, (0.0, 0.0, 0.0)
            self._last_t = None
            if self._backend is not None:
                self._backend.reset()

    def kill(self) -> None:
        """Emergency stop: zero all axes and refuse to send anything further."""
        with self._lock:
            self._killed = True
            self.neutralize()

    def close(self) -> None:
        """Kill, then release the virtual pad. Idempotent."""
        with self._lock:
            if self._backend is None:
                return
            self.kill()
            backend, self._backend = self._backend, None
            backend.close()

    @property
    def killed(self) -> bool:
        return self._killed

    @property
    def output(self) -> tuple[float, float, float]:
        return self._out

    def __enter__(self) -> "VirtualController":
        return self

    def __exit__(self, *exc) -> None:
        self.close()


# --------------------------------------------------------------------------- #
def selftest() -> int:
    fails: list[str] = []
    check = lambda ok, name: ok or fails.append(name)  # noqa: E731

    cfg = replace(CONFIG.controller, wake_on_connect=False, steer_game_deadzone=0.0,
                  invert_steer=False)
    nb = NullBackend()
    vc = VirtualController(cfg, nb)

    # smoothing converges monotonically (no overshoot)
    vc.set_targets(0.5, 0.8, 0.0)
    prev = -1.0
    for i in range(1, 121):
        s, th, b = vc.update(i / 60)
        check(s >= prev - 1e-9, "steer monotonic")
        prev = s
    check(abs(s - 0.5) < 0.01 and abs(th - 0.8) < 0.01 and b == 0.0, "converges")

    # frame-rate independence: 30 Hz vs 120 Hz after 0.25 s
    def run(hz: int) -> float:
        v = VirtualController(cfg, NullBackend())
        v.set_targets(1.0, 0.0, 0.0)
        v.update(0.0)
        out = [v.update(i / hz)[0] for i in range(1, int(hz * 0.25) + 1)][-1]
        v.close()
        return out
    check(abs(run(30) - run(120)) < 0.02, "frame-rate independent")

    # deadzone snap
    vc.neutralize()
    vc.set_targets(cfg.steer_deadzone * 0.5, cfg.trigger_deadzone * 0.5, 0.0)
    for i in range(300):
        vc.update(i / 60)
    check(vc.output == (0.0, 0.0, 0.0), "deadzone snap")

    # game-deadzone rescale; invert
    v2 = VirtualController(replace(cfg, steer_game_deadzone=0.1, steer_smoothing=1.0,
                                   invert_steer=True), NullBackend())
    v2.set_targets(-0.05, 0.0, 0.0)
    check(abs(v2.update(0.0)[0] - (0.1 + 0.9 * 0.05)) < 1e-6, "game deadzone rescale + invert")
    v2.set_targets(-1.0, 0.0, 0.0)
    check(abs(v2.update(0.02)[0] - 1.0) < 1e-6, "full scale preserved")
    v2.close()

    # brake interlock; NaN / out-of-range sanitising
    vc.neutralize()
    vc.set_targets(0.0, 1.0, 0.6)
    for i in range(120):
        out = vc.update(i / 60)
    check(out[1] == 0.0 and out[2] > 0.5, "brake overrides throttle")
    vc.set_targets(float("nan"), 9.0, -3.0)
    check(vc._tgt == [0.0, 1.0, 0.0], "sanitise input")

    # kill zeroes and latches; close releases
    vc.set_targets(1.0, 1.0, 0.0)
    vc.update(10.0)
    vc.kill()
    sent = nb.sent
    vc.set_targets(1.0, 1.0, 0.0)
    check(nb.last == (0.0, 0.0, 0.0), "kill zeroes backend")
    check(vc.update(11.0) == (0.0, 0.0, 0.0) and nb.sent == sent, "kill latches")
    vc.close()
    check(vc._backend is None, "close releases backend")
    vc.close()  # idempotent

    print("SELFTEST", "FAILED: " + ", ".join(fails) if fails else "OK")
    return 1 if fails else 0


def bind_test() -> int:
    """Sweep each axis slowly so Content Manager / joy.cpl can see and bind it."""
    try:
        vc = VirtualController(replace(CONFIG.controller, wake_on_connect=True))
    except ControllerError as e:
        print(f"ERROR: {e}")
        return 1
    print("Virtual Xbox 360 pad is live. Sweeping axes - bind them in AC now. Ctrl+C to stop.")
    labels = ("STEER (left stick X)", "THROTTLE (right trigger)", "BRAKE (left trigger)")
    try:
        with vc:
            t0 = time.perf_counter()
            while True:
                t = time.perf_counter() - t0
                phase, x = int(t // 6) % 3, math.sin((t % 6) / 6 * 2 * math.pi)
                tgt = [0.0, 0.0, 0.0]
                tgt[phase] = x if phase == 0 else abs(x)
                vc.set_targets(*tgt)
                s, th, b = vc.update()
                print(f"\r{labels[phase]:<26} steer {s:+.2f} thr {th:.2f} brk {b:.2f}   ",
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

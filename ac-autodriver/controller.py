"""
controller.py - drives the game's inputs.

Outputs (config.controller.output):
    "wheel"  vJoy virtual wheel: steer -> X axis, gas -> Y, brake -> Z. Bind those in
             Content Manager and keep your Logitech for buttons / paddles.
    "xbox"   virtual Xbox 360 pad: steer -> left stick X, gas -> RT, brake -> LT.

The planner calls set_targets(); update() runs once per tick and applies
clamp -> brake-over-throttle interlock -> time-based EMA smoothing
-> deadzone snap / game-deadzone rescale -> send.
direct() is pass-through: your real wheel driving the car through vJoy, 1:1.

Your physical Logitech is read with pygame (PhysicalWheel) using a calibration made
once with --wheel-setup.

Safety: neutralize() centres the wheel and releases the pedals (pause, re-usable);
kill() does that and latches; atexit/close() always do it, so a crash can't leave a
stuck input.

    python controller.py --selftest      # offline logic check, no drivers needed
    python controller.py --wheel-setup   # learn your Logitech's axes (once)
    python controller.py --bind-test     # sweeps each axis (for Content Manager binding)
"""

from __future__ import annotations

import argparse
import atexit
import ctypes
import json
import math
import os
import sys
import threading
import time
from dataclasses import replace
from pathlib import Path

from config import CONFIG, ControllerConfig, WheelConfig


class ControllerError(RuntimeError):
    """Virtual device could not be created (missing package or driver)."""


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

    def buttons(self, up: bool, down: bool) -> None:
        B = self._vg.XUSB_BUTTON
        for on, btn in ((up, B.XUSB_GAMEPAD_RIGHT_SHOULDER), (down, B.XUSB_GAMEPAD_LEFT_SHOULDER)):
            (self._pad.press_button if on else self._pad.release_button)(button=btn)
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


HID_X, HID_Y, HID_Z = 0x30, 0x31, 0x32
VJD_OWN, VJD_FREE, VJD_BUSY, VJD_MISS = 0, 1, 2, 3


def find_vjoy_dll() -> str | None:
    pf = os.environ.get("ProgramFiles", r"C:\Program Files")
    cands = [os.path.join(pf, "vJoy", "x64", "vJoyInterface.dll"),
             os.path.join(pf, "vJoy", "vJoyInterface.dll")]
    try:
        import winreg
        key = r"SOFTWARE\Microsoft\Windows\CurrentVersion\Uninstall\{8E31F76F-74C3-47F1-9550-E041EEDC5FBB}_is1"
        with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, key) as k:
            loc = winreg.QueryValueEx(k, "InstallLocation")[0]
            cands.insert(0, os.path.join(loc, "x64", "vJoyInterface.dll"))
    except Exception:
        pass
    return next((c for c in cands if os.path.isfile(c)), None)


class VJoyBackend:
    """vJoy virtual wheel. Axes run 1..0x8000; X centre = straight ahead, Y/Z low = pedal up."""
    AXIS_MAX = 0x8000

    def __init__(self, device: int = 1, dll=None, up_button: int = 1, down_button: int = 2) -> None:
        if dll is None:
            if sys.platform != "win32":
                raise ControllerError("vJoy is Windows-only")
            path = find_vjoy_dll()
            if not path:
                raise ControllerError("vJoy driver not found - run install.bat")
            dll = ctypes.WinDLL(path)
            dll.SetAxis.argtypes = [ctypes.c_long, ctypes.c_uint, ctypes.c_uint]
            for f in ("AcquireVJD", "RelinquishVJD", "GetVJDStatus"):
                getattr(dll, f).argtypes = [ctypes.c_uint]
            dll.GetVJDAxisExist.argtypes = [ctypes.c_uint, ctypes.c_uint]
            dll.SetBtn.argtypes = [ctypes.c_int, ctypes.c_uint, ctypes.c_ubyte]
            dll.GetVJDButtonNumber.argtypes = [ctypes.c_uint]
        self.dll, self.dev = dll, device
        if not dll.vJoyEnabled():
            raise ControllerError("vJoy is installed but disabled - enable it in 'Configure vJoy'")
        st = dll.GetVJDStatus(device)
        if st == VJD_BUSY:
            raise ControllerError(f"vJoy device {device} is in use by another program - close it")
        if st == VJD_MISS:
            raise ControllerError(f"vJoy device {device} doesn't exist - add it in 'Configure vJoy'")
        if st != VJD_OWN and not dll.AcquireVJD(device):
            raise ControllerError(f"could not take vJoy device {device}")
        missing = [n for n, a in (("X", HID_X), ("Y", HID_Y), ("Z", HID_Z))
                   if not dll.GetVJDAxisExist(device, a)]
        if missing:
            dll.RelinquishVJD(device)
            raise ControllerError(f"vJoy device {device} has no {'/'.join(missing)} axis - "
                                  "tick X, Y and Z in 'Configure vJoy'")
        self.up_button, self.down_button = up_button, down_button
        # gear buttons need at least that many buttons on the device (Configure vJoy)
        self.has_buttons = dll.GetVJDButtonNumber(device) >= max(up_button, down_button)
        self.reset()

    def _ax(self, frac: float) -> int:
        return int(round(1 + min(max(frac, 0.0), 1.0) * (self.AXIS_MAX - 1)))

    def send(self, steer: float, throttle: float, brake: float) -> None:
        d, n = self.dll, self.dev
        d.SetAxis(self._ax((steer + 1) / 2), n, HID_X)
        d.SetAxis(self._ax(throttle), n, HID_Y)
        d.SetAxis(self._ax(brake), n, HID_Z)

    def reset(self) -> None:
        # explicit neutral: vJoy's own ResetVJD would put X at 0 = full lock left
        self.send(0.0, 0.0, 0.0)
        self.buttons(False, False)

    def buttons(self, up: bool, down: bool) -> None:
        if self.has_buttons:
            self.dll.SetBtn(int(up), self.dev, self.up_button)
            self.dll.SetBtn(int(down), self.dev, self.down_button)

    def wake(self) -> None:
        pass

    def close(self) -> None:
        try:
            self.reset()
        finally:
            self.dll.RelinquishVJD(self.dev)


def make_backend(cfg: ControllerConfig):
    if cfg.output == "wheel":
        return VJoyBackend(cfg.vjoy_device, up_button=CONFIG.drive.upshift_button,
                           down_button=CONFIG.drive.downshift_button)
    if cfg.output == "xbox":
        return VGamepadBackend()
    raise ControllerError(f"unknown controller output {cfg.output!r} (use 'wheel' or 'xbox')")


class NullBackend:
    """Records what would be sent; for --selftest and driver-less dry runs."""

    def __init__(self) -> None:
        self.last, self.sent = (0.0, 0.0, 0.0), 0
        self.btn = (False, False)

    def send(self, steer: float, throttle: float, brake: float) -> None:
        self.last, self.sent = (steer, throttle, brake), self.sent + 1

    def reset(self) -> None:
        self.last, self.btn = (0.0, 0.0, 0.0), (False, False)

    def buttons(self, up: bool, down: bool) -> None:
        self.btn = (up, down)

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
        self._backend = backend if backend is not None else make_backend(self.cfg)
        # planner steer -> axis (see config.steer_output_scale); pass-through is never scaled
        self.steer_scale = self.cfg.steer_output_scale if self.cfg.output == "wheel" else 1.0
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
    def set_targets(self, steer: float, throttle: float, brake: float, scaled: bool = True) -> None:
        """Latest desired inputs. steer in [-1,1]; throttle, brake in [0,1].
        scaled=False: steer is already a real axis value (the learned steering model)."""
        steer = _finite(steer) * (self.steer_scale if scaled else 1.0) * (-1 if self.cfg.invert_steer else 1)
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

    def direct(self, steer: float, throttle: float, brake: float) -> None:
        """Pass-through: send exactly this (your real wheel), no scaling or smoothing.
        The filter follows, so engaging the bot later starts from where you are."""
        with self._lock:
            if self._killed or self._backend is None:
                return
            out = (_clamp(_finite(steer), -1, 1), _clamp(_finite(throttle), 0, 1),
                   _clamp(_finite(brake), 0, 1))
            self._tgt, self._flt, self._out, self._last_t = list(out), list(out), out, None
            self._backend.send(*out)

    def buttons(self, up: bool, down: bool) -> None:
        """Gear buttons (held while True)."""
        with self._lock:
            if not self._killed and self._backend is not None and hasattr(self._backend, "buttons"):
                self._backend.buttons(up, down)

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
# Your physical wheel (pygame)
# --------------------------------------------------------------------------- #
def _pygame():
    os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")  # keep reading while AC has focus
    os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
    import pygame
    pygame.init()
    pygame.joystick.init()
    return pygame


def calibration_from(rest: list, left: list, right: list, gas: list, brake: list, name: str) -> dict:
    """Work out which axis is which from held positions (raw pygame values, -1..1)."""
    def pick(a: list, b: list, exclude: set) -> int:
        diffs = [(abs(x - y), i) for i, (x, y) in enumerate(zip(a, b)) if i not in exclude]
        d, i = max(diffs)
        if d < 0.3:
            raise ControllerError("no axis moved enough - check the wheel is connected and try again")
        return i
    st = pick(left, right, set())
    g = pick(rest, gas, {st})
    b = pick(rest, brake, {st, g})
    return {"name": name, "steer_axis": st, "steer_left": left[st], "steer_center": rest[st],
            "steer_right": right[st], "gas_axis": g, "gas_rest": rest[g], "gas_full": gas[g],
            "brake_axis": b, "brake_rest": rest[b], "brake_full": brake[b]}


class PhysicalWheel:
    """Steering / gas / brake from your real wheel, normalised with the saved calibration:
    steer -1 (full left) .. +1 (full right), pedals 0 (up) .. 1 (floored)."""

    def __init__(self, cal: dict, joystick, pump=lambda: None) -> None:
        self.cal, self.js, self.pump = cal, joystick, pump

    @classmethod
    def load(cls, cfg: WheelConfig, base: Path) -> tuple["PhysicalWheel | None", str]:
        path = base / cfg.calibration_file
        if not path.exists():
            return None, "no wheel calibration yet (run.bat -> wheel setup)"
        cal = json.loads(path.read_text())
        try:
            pg = _pygame()
        except ImportError:
            return None, "pygame missing (re-run install.bat)"
        for i in range(pg.joystick.get_count()):
            js = pg.joystick.Joystick(i)
            if js.get_name() == cal["name"]:
                js.init()
                return cls(cal, js, pg.event.pump), cal["name"]
        return None, f"{cal['name']} not connected"

    def raw(self) -> list[float]:
        self.pump()
        return [self.js.get_axis(i) for i in range(self.js.get_numaxes())]

    def read(self) -> tuple[float, float, float]:
        a, c = self.raw(), self.cal
        x = a[c["steer_axis"]] - c["steer_center"]
        right = c["steer_right"] - c["steer_center"]
        left = c["steer_left"] - c["steer_center"]
        if right and x * right > 0:
            steer = x / right                  # same side as "full right"
        else:
            steer = -x / left if left else 0.0
        def pedal(axis: str) -> float:
            span = c[f"{axis}_full"] - c[f"{axis}_rest"]
            return _clamp((a[c[f"{axis}_axis"]] - c[f"{axis}_rest"]) / span, 0, 1) if span else 0.0
        return _clamp(steer, -1, 1), pedal("gas"), pedal("brake")

    def read_buttons(self) -> tuple[bool, bool]:
        """(upshift, downshift) paddles, if wheel setup learned them."""
        c, n = self.cal, self.js.get_numbuttons()
        def b(key):
            i = c.get(key, -1)
            return 0 <= i < n and bool(self.js.get_button(i))
        return b("up_button"), b("down_button")


def wheel_setup(cfg: WheelConfig, base: Path) -> int:
    """Interactive: learn which axes your wheel and pedals use. Takes ~30 s."""
    try:
        pg = _pygame()
    except ImportError:
        print("pygame is missing - re-run install.bat")
        return 1
    sticks = [pg.joystick.Joystick(i) for i in range(pg.joystick.get_count())]
    if not sticks:
        print("No wheel / joystick found. Plug the wheel in (and turn it on) and try again.")
        return 1
    for i, js in enumerate(sticks):
        print(f"  {i}: {js.get_name()}  ({js.get_numaxes()} axes)")
    guess = [i for i, js in enumerate(sticks)
             if any(k in js.get_name().lower() for k in ("logitech", "g29", "g920", "g923", "driving force", "g27", "g25"))
             and "vjoy" not in js.get_name().lower()]
    if len(guess) == 1:
        idx = guess[0]
        print(f"Using {sticks[idx].get_name()}")
    else:
        idx = int(input("Which one is your wheel? number: ").strip())
    js = sticks[idx]
    js.init()

    def hold(prompt: str, secs: float = 4.0) -> list[float]:
        print(f"\n{prompt}")
        end, samples = time.perf_counter() + secs, []
        while time.perf_counter() < end:
            pg.event.pump()
            samples.append([js.get_axis(i) for i in range(js.get_numaxes())])
            print(f"\r  {end - time.perf_counter():3.1f} s ", end="", flush=True)
            time.sleep(0.02)
        tail = samples[-max(1, len(samples) // 4):]          # average the last quarter: held still
        return [sum(s[i] for s in tail) / len(tail) for i in range(js.get_numaxes())]

    rest = hold("Centre the wheel and take your feet OFF the pedals.", 3)
    left = hold("Turn the wheel fully LEFT and hold it there.")
    right = hold("Now fully RIGHT and hold.")
    gas = hold("Centre the wheel. Press the GAS pedal all the way and hold.")
    brake = hold("Release the gas. Press the BRAKE pedal all the way and hold.")
    try:
        cal = calibration_from(rest, left, right, gas, brake, js.get_name())
    except ControllerError as e:
        print(f"\n{e}")
        return 1

    def button(prompt: str, exclude: set) -> int:
        print(f"\n{prompt} (or wait to skip)")
        end, seen = time.perf_counter() + 4.0, []
        while time.perf_counter() < end:
            pg.event.pump()
            seen += [i for i in range(js.get_numbuttons()) if js.get_button(i) and i not in exclude]
            print(f"\r  {end - time.perf_counter():3.1f} s ", end="", flush=True)
            time.sleep(0.02)
        return max(set(seen), key=seen.count) if seen else -1   # most-pressed button, or none
    cal["up_button"] = button("Pull your UPSHIFT paddle (right) a few times.", set())
    cal["down_button"] = button("Pull your DOWNSHIFT paddle (left) a few times.", {cal["up_button"]})
    print("\n  paddles: " + ("not set - your paddles won't pass through" if cal["up_button"] < 0 else
                             f"up = button {cal['up_button']}, down = button {cal['down_button']}"))
    (base / cfg.calibration_file).write_text(json.dumps(cal, indent=2))
    print(f"\n\nSaved {cfg.calibration_file}. Check it - move the wheel and pedals (5 s):")
    w, end = PhysicalWheel(cal, js, pg.event.pump), time.perf_counter() + 5
    while time.perf_counter() < end:
        s, g, b = w.read()
        print(f"\r  steer {s:+.2f}   gas {g:.2f}   brake {b:.2f}   ", end="", flush=True)
        time.sleep(0.03)
    print("\nLeft should read negative, right positive, pedals 0 -> 1. If not, run setup again.")
    return 0


# --------------------------------------------------------------------------- #
def selftest() -> int:
    fails: list[str] = []
    check = lambda ok, name: ok or fails.append(name)  # noqa: E731

    cfg = replace(CONFIG.controller, wake_on_connect=False, steer_game_deadzone=0.0,
                  invert_steer=False, output="xbox")      # xbox = unscaled steer
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

    # vJoy backend against a fake driver DLL
    class FakeVJoy:
        def __init__(self, status=VJD_FREE, axes=(HID_X, HID_Y, HID_Z)):
            self.status, self.axes, self.set, self.released = status, set(axes), {}, False
        def vJoyEnabled(self): return True
        def GetVJDStatus(self, n): return self.status
        def AcquireVJD(self, n): return True
        def GetVJDAxisExist(self, n, a): return a in self.axes
        def SetAxis(self, v, n, a): self.set[a] = v; return True
        def RelinquishVJD(self, n): self.released = True
        def GetVJDButtonNumber(self, n): return 8
        def SetBtn(self, v, n, b): self.btns = getattr(self, "btns", {}); self.btns[b] = v; return True
    fv = FakeVJoy()
    vb = VJoyBackend(1, dll=fv)
    check(fv.set == {HID_X: 16384, HID_Y: 1, HID_Z: 1}, f"vjoy neutral on open: {fv.set}")
    vb.send(1.0, 1.0, 0.0)
    check(fv.set == {HID_X: 0x8000, HID_Y: 0x8000, HID_Z: 1}, "vjoy full right + full gas")
    vb.send(-1.0, 0.0, 0.5)
    check(fv.set[HID_X] == 1 and abs(fv.set[HID_Z] - 16384) <= 1, "vjoy full left + half brake")
    vb.buttons(True, False)
    check(fv.btns == {1: 1, 2: 0}, f"vjoy gear buttons {fv.btns}")
    vb.close()
    check(fv.btns == {1: 0, 2: 0}, "vjoy close releases buttons")
    check(fv.released and fv.set[HID_X] == 16384 and fv.set[HID_Y] == 1, "vjoy close: neutral + released")
    for bad, why in ((FakeVJoy(status=VJD_BUSY), "busy"), (FakeVJoy(status=VJD_MISS), "missing"),
                     (FakeVJoy(axes=(HID_X, HID_Y)), "no Z axis")):
        try:
            VJoyBackend(1, dll=bad)
            fails.append(f"vjoy should refuse: {why}")
        except ControllerError:
            pass

    # wheel output scales planner steer; pass-through is exact
    wcfg = replace(cfg, output="wheel", steer_output_scale=0.5, steer_smoothing=1.0)
    w = VirtualController(wcfg, NullBackend())
    w.set_targets(1.0, 0.0, 0.0)
    check(abs(w.update(0.0)[0] - 0.5) < 1e-9, "wheel steer scale")
    w.direct(-0.8, 0.3, 0.0)
    check(w.output == (-0.8, 0.3, 0.0) and w._backend.last == (-0.8, 0.3, 0.0), "pass-through exact")
    w.set_targets(0.3, 0.0, 0.0, scaled=False)
    check(abs(w.update(1.0)[0] - 0.3) < 1e-9, "unscaled steer (learned model) passes as-is")
    w.buttons(True, False)
    check(w._backend.btn == (True, False), "gear button pressed")
    w.kill()
    check(w._backend.btn == (False, False), "kill releases gear buttons")
    w.buttons(True, False)
    check(w._backend.btn == (False, False), "no gear buttons after kill")
    w.direct(1.0, 1.0, 0.0)
    check(w._backend.last == (0.0, 0.0, 0.0), "no pass-through after kill")
    w.close()

    # physical wheel: axis detection + normalisation, incl. a wheel that reads left as +
    rest, left, right = [0.0, -1.0, -1.0, 0.0], [0.9, -1.0, -1.0, 0.0], [-0.9, -1.0, -1.0, 0.0]
    gas, brake = [0.0, 1.0, -1.0, 0.0], [0.0, -1.0, 0.6, 0.0]
    cal = calibration_from(rest, left, right, gas, brake, "Logitech G29 Driving Force Racing Wheel")
    check((cal["steer_axis"], cal["gas_axis"], cal["brake_axis"]) == (0, 1, 2), f"axis detect {cal}")
    class FakeStick:
        def __init__(self, axes): self.axes, self.btns = axes, [0] * 12
        def get_numaxes(self): return len(self.axes)
        def get_axis(self, i): return self.axes[i]
        def get_numbuttons(self): return len(self.btns)
        def get_button(self, i): return self.btns[i]
    stick = FakeStick(list(rest))
    pw = PhysicalWheel(cal, stick)
    check(pw.read() == (0.0, 0.0, 0.0), "wheel at rest")
    stick.axes = [0.45, 0.0, -0.2, 0.0]          # half left, half gas, half brake
    s, g, b = pw.read()
    check(abs(s + 0.5) < 1e-9 and abs(g - 0.5) < 1e-9 and abs(b - 0.5) < 1e-9, f"wheel half: {(s, g, b)}")
    stick.axes = [-0.9, 1.0, 0.6, 0.0]
    check(pw.read() == (1.0, 1.0, 1.0), "wheel full right, pedals floored")
    check(pw.read_buttons() == (False, False), "no paddles learned = none pressed")
    pw.cal.update(up_button=4, down_button=5)
    stick.btns[5] = 1
    check(pw.read_buttons() == (False, True), "downshift paddle")
    try:
        calibration_from(rest, rest, rest, gas, brake, "x")
        fails.append("calibration should reject an unmoved wheel")
    except ControllerError:
        pass

    print("SELFTEST", "FAILED: " + ", ".join(fails) if fails else "OK")
    return 1 if fails else 0


def bind_test() -> int:
    """Sweep each axis slowly so Content Manager / joy.cpl can see and bind it."""
    try:
        vc = VirtualController(replace(CONFIG.controller, wake_on_connect=True))
    except ControllerError as e:
        print(f"ERROR: {e}")
        return 1
    print(f"Virtual {CONFIG.controller.output} is live. Cycling steer / gas / brake / gear up / gear down, "
          "6 s each - bind them in Content Manager now. Ctrl+C to stop.")
    labels = ("STEER", "THROTTLE", "BRAKE", "GEAR UP (vJoy button 1)", "GEAR DOWN (vJoy button 2)")
    try:
        with vc:
            t0 = time.perf_counter()
            while True:
                t = time.perf_counter() - t0
                phase, x = int(t // 6) % 5, math.sin((t % 6) / 6 * 2 * math.pi)
                tgt = [0.0, 0.0, 0.0]
                if phase < 3:
                    tgt[phase] = x if phase == 0 else abs(x)
                pulse = (t % 1.0) < 0.3                  # buttons: pressed 0.3 s every second
                vc.buttons(phase == 3 and pulse, phase == 4 and pulse)
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
    ap.add_argument("--wheel-setup", action="store_true", help="learn your physical wheel's axes")
    a = ap.parse_args()
    if a.selftest:
        sys.exit(selftest())
    if a.wheel_setup:
        sys.exit(wheel_setup(CONFIG.wheel, Path(__file__).parent))
    if a.bind_test:
        sys.exit(bind_test())
    ap.print_help()

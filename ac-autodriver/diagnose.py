"""
diagnose.py - find out why it isn't driving right, and package the evidence.

Run with Assetto Corsa in a session, car stopped (hard shoulder or pit lane):

    python diagnose.py            (or run.bat -> 0)

1. Bindings (5 s): moves the virtual wheel's steering, gas and brake one at a time and
   reads back what AC actually receives. Says if an axis isn't bound, is inverted, or
   is crossed with another.
2. Capture (30 s): drive along the highway past traffic (your wheel works through
   pass-through) - or play a replay of highway driving. Saves what vision sees, and what
   the bot would have decided and why.

Writes diagnostics/ (summary.txt, capture.csv, screen.png, frames/) and diagnostics.zip.
Send the zip.
"""

from __future__ import annotations

import argparse
import csv
import math
import shutil
import statistics
import sys
import time
from collections import Counter
from pathlib import Path

from config import CONFIG, Config

BASE = Path(__file__).parent
OUT = BASE / "diagnostics"


def classify(r0: float, r_mid: float, r1: float) -> str:
    """Readback of an axis we drove at 0, 0.5, 1."""
    if max(r0, r_mid, r1) - min(r0, r_mid, r1) < 0.1:
        return "NOT RESPONDING - not bound to vJoy (still on your Logitech pedal?)"
    if r0 > r1:
        return "INVERTED - tick 'invert' for it in Content Manager"
    if r0 > 0.1:
        return f"never fully releases (reads {r0:.2f} at 0) - check deadzone / calibration in Content Manager"
    if r1 < 0.9:
        return f"never reaches full (reads {r1:.2f} at 1) - check range / gamma in Content Manager"
    if abs(r_mid - 0.5) > 0.2:
        return f"OK but non-linear (0.5 reads {r_mid:.2f}) - set gamma 1 in Content Manager"
    return "OK"


def _read(tm, settle: float, sample: float, sleep) -> tuple[float, float, float]:
    sleep(settle)
    t_end, rows = time.perf_counter() + sample, []
    while True:
        s = tm.read()
        rows.append((s.steer_angle, s.gas, s.brake))
        if time.perf_counter() >= t_end:
            break
        sleep(0.01)
    return tuple(sum(r[i] for r in rows) / len(rows) for i in range(3))


def check_bindings(tm, pad, settle: float = 0.35, sample: float = 0.15, sleep=time.sleep) -> list[str]:
    """Drive each virtual axis and read back what AC receives. Brake is held during the
    steering / gas tests so the car stays put."""
    def at(steer, gas, brake):
        pad.direct(steer, gas, brake)
        return _read(tm, settle, sample, sleep)

    lines = []
    s_m, s_0, s_p = (at(x, 0.0, 1.0)[0] for x in (-0.5, 0.0, 0.5))
    if abs(s_p - s_m) < 0.02:
        lines.append(f"steering: NOT RESPONDING - bind Steering to the vJoy X axis "
                     f"(AC read {s_m:+.2f} / {s_0:+.2f} / {s_p:+.2f})")
    else:
        lines.append(f"steering: responds  (sent -0.5 / 0 / +0.5 -> AC {s_m:+.2f} / {s_0:+.2f} / {s_p:+.2f}). "
                     "If the car steered the wrong way when engaged, tick invert for steering.")
    gas = [at(0.0, g, 1.0) for g in (0.0, 0.5, 1.0)]
    lines.append("throttle: " + classify(*(r[1] for r in gas)))
    if max(r[2] for r in gas) - min(r[2] for r in gas) > 0.3:
        lines.append("  !! the brake moved while we only moved the gas: throttle and brake are crossed")
    brk = [at(0.0, 0.0, b) for b in (0.0, 0.5, 1.0)]
    lines.append("brake:    " + classify(*(r[2] for r in brk)))
    if max(r[1] for r in brk) - min(r[1] for r in brk) > 0.3:
        lines.append("  !! the throttle moved while we only moved the brake: throttle and brake are crossed")
    pad.direct(0.0, 0.0, 1.0)

    s = tm.read()
    if getattr(s, "auto_shifter", True):
        lines.append("gears:    AC automatic gearbox is on - fine, the bot doesn't need to shift")
    else:
        def press(up: bool) -> int:
            pad.buttons(up, not up)
            sleep(0.15)
            tm.read()                                # (AC sees the press while it's held)
            pad.buttons(False, False)
            sleep(settle + 0.3)
            return tm.read().gear_raw
        g0 = s.gear_raw
        g1 = press(True)
        g2 = press(False)
        if g0 == g1 == g2:
            lines.append("gears:    NOT RESPONDING - bind Gear Up / Gear Down to vJoy buttons 1 / 2 in Content "
                         "Manager (bind test cycles them), or turn on AC's automatic gearbox")
        elif g1 < g0:
            lines.append("gears:    SWAPPED - Gear Up is bound to vJoy button 2 and Down to 1")
        else:
            lines.append("gears:    OK (shifted up and back down)")
    return lines


def summarize_capture(rows: list[dict], why: Counter, states: Counter) -> list[str]:
    if not rows:
        return ["capture: no data (AC not live?)"]
    n = len(rows)
    lanes = [r for r in rows if r["conf"] is not None]
    good = [r for r in lanes if r["conf"] >= 0.5]
    with_obs = [r for r in rows if r["n_obs"]]
    speeds = [r["kmh"] for r in rows if r["kmh"] is not None]
    out = [f"capture: {n} ticks, vision {statistics.mean(r['fps'] for r in rows):.0f} fps, "
           f"{statistics.mean(r['ms'] for r in lanes) if lanes else 0:.1f} ms/frame",
           f"lanes:   found (conf >= 0.5) in {100 * len(good) / n:.0f}% of ticks, "
           f"mean conf {statistics.mean(r['conf'] for r in lanes) if lanes else 0:.2f}"
           + ("   << LOW: check the ROI covers the road (screen.png)" if len(good) < 0.6 * n else ""),
           f"cars:    seen in {100 * len(with_obs) / n:.0f}% of ticks"
           + ("   << NONE: vision isn't detecting traffic - send the frames" if not with_obs else "")]
    for z in ("left", "center", "right"):
        d = [r[z] for r in rows if r[z] is not None and math.isfinite(r[z])]
        out.append(f"  {z:<6} {100 * len(d) / n:3.0f}% of ticks"
                   + (f", nearest median {statistics.median(d):.0f} m, closest {min(d):.0f} m" if d else ""))
    if speeds:
        out.append(f"speed:   {min(speeds):.0f} - {max(speeds):.0f} km/h (avg {statistics.mean(speeds):.0f})")
    tot = sum(states.values()) or 1
    out.append("bot would be: " + ", ".join(f"{k} {100 * v / tot:.0f}%" for k, v in states.most_common()))
    if why:
        out.append("not changing lanes because:")
        out += [f"  {100 * v / tot:3.0f}%  {k}" for k, v in why.most_common(6)]
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description="ac-autodriver diagnostics")
    ap.add_argument("--seconds", type=float, default=30.0, help="capture length")
    ap.add_argument("--skip-bindings", action="store_true")
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        return selftest()

    import cv2
    from controller import ControllerError, PhysicalWheel, VirtualController
    from planner import Planner
    from telemetry import ACTelemetry
    from vision import VisionThread, calibrate, draw_overlay

    cfg: Config = CONFIG
    if OUT.exists():
        shutil.rmtree(OUT)
    (OUT / "frames").mkdir(parents=True)
    summary = [f"ac-autodriver diagnostics {time.strftime('%Y-%m-%d %H:%M')}",
               f"mode {cfg.mode}  output {cfg.controller.output}  steer_output_scale "
               f"{cfg.controller.steer_output_scale}  roi {cfg.vision.roi}  monitor {cfg.vision.monitor_index}"]

    try:
        pad = VirtualController(cfg.controller)
    except ControllerError as e:
        print(f"Output device: {e}")
        return 1
    wheel, note = PhysicalWheel.load(cfg.wheel, BASE)
    summary.append(f"wheel: {note}")
    tm = ACTelemetry()
    print("Waiting for Assetto Corsa (get into a session)...", flush=True)
    try:
        tm.connect()
        while not tm.read().is_fresh:
            print("\r  AC is connected but not live - get in the car and unpause   ", end="", flush=True)
            time.sleep(0.5)
        print()
        s = tm.read()
        summary.append(f"car: {s.car_model} @ {s.track}, automatic gearbox {'ON' if s.auto_shifter else 'OFF  << turn it on'}")

        # ---- 1. bindings ---------------------------------------------------------
        if not a.skip_bindings:
            if cfg.controller.output != "wheel":
                summary.append("bindings: skipped (output is xbox)")
            else:
                t0 = time.perf_counter()
                while tm.read().speed_kmh > 5 and time.perf_counter() - t0 < 30:
                    print(f"\r  Stop the car to test the bindings ({tm.read().speed_kmh:.0f} km/h)   ", end="", flush=True)
                    if wheel:
                        pad.direct(*wheel.read())
                    time.sleep(0.05)
                print("\nTesting bindings - hands off the wheel and pedals for 5 seconds...", flush=True)
                lines = check_bindings(tm, pad)
                print("\n".join("  " + x for x in lines))
                summary += ["", "BINDINGS"] + lines

        # ---- 2. capture ------------------------------------------------------------
        calibrate(cfg.vision, str(OUT / "screen.png"))
        print(f"\nCapturing {a.seconds:.0f} s. Drive along the highway past traffic now"
              f"{'' if wheel else ' (no wheel calibration: play a replay instead)'}.", flush=True)
        vt = VisionThread(cfg.vision).start()
        planner = Planner(cfg)                    # deciding, not driving: tells us what it *would* do
        rows, why, states = [], Counter(), Counter()
        t0 = last_frame = time.perf_counter()
        n_frame = 0
        with open(OUT / "capture.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "kmh", "ac_gas", "ac_brake", "ac_steer", "lane_off", "lane_conf",
                        "left_m", "center_m", "right_m", "n_obs", "vision_fps", "bot_state", "bot_note", "why"])
            while (now := time.perf_counter()) - t0 < a.seconds:
                s, p = tm.read(), vt.latest()
                if wheel:
                    pad.direct(*wheel.read())
                cmd = planner.update(s, p, now)
                states[cmd.state] += 1
                if planner.why:
                    for part in planner.why.split("; "):
                        why[part.split(" (")[0]] += 1
                z = p.zones if p else None
                row = dict(kmh=s.speed_kmh if s.is_fresh else None, conf=p.lanes.confidence if p else None,
                           ms=p.process_ms if p else 0.0, fps=vt.fps, n_obs=len(p.obstacles) if p else 0,
                           **{k: (z[k].nearest_m if z else None) for k in ("left", "center", "right")})
                rows.append(row)
                w.writerow([f"{now - t0:.2f}", row["kmh"], f"{s.gas:.2f}", f"{s.brake:.2f}", f"{s.steer_angle:.2f}",
                            p and p.lanes.offset, row["conf"], row["left"], row["center"], row["right"],
                            row["n_obs"], f"{vt.fps:.0f}", cmd.state, cmd.note, planner.why])
                if now - last_frame >= 1.0 and p is not None and vt.frame() is not None:
                    last_frame = now
                    cv2.imwrite(str(OUT / "frames" / f"{n_frame:02d}.png"), draw_overlay(vt.frame(), p, cfg.vision))
                    n_frame += 1
                print(f"\r  {now - t0:4.1f}s  {s.speed_kmh:5.0f} km/h  lanes {row['conf'] or 0:.2f}  "
                      f"cars {row['n_obs']}  bot: {cmd.state} {planner.why[:50]:<50}", end="", flush=True)
                time.sleep(1 / 60)
        vt.stop()
        summary.append(f"capture: {vt.backend}, grab {vt.grab_ms:.1f} ms, "
                       f"process {vt.proc_ms:.1f} ms")
        if vt.error:
            summary.append(f"vision error: {vt.error!r}")
        summary += ["", "CAPTURE"] + summarize_capture(rows, why, states)
    except KeyboardInterrupt:
        summary.append("(interrupted)")
    finally:
        pad.close()
        tm.close()

    (OUT / "summary.txt").write_text("\n".join(summary))
    zip_path = shutil.make_archive(str(BASE / "diagnostics"), "zip", OUT)
    print("\n\n" + "\n".join(summary))
    print(f"\nSaved {zip_path} - send that file.")
    return 0


# --------------------------------------------------------------------------- #
def selftest() -> int:
    from types import SimpleNamespace

    from controller import NullBackend, VirtualController

    fails: list[str] = []
    check = lambda ok, name: ok or fails.append(name)  # noqa: E731

    check(classify(0.0, 0.5, 1.0) == "OK", "classify ok")
    check(classify(1.0, 0.5, 0.0).startswith("INVERTED"), "classify inverted")
    check(classify(0.0, 0.0, 0.0).startswith("NOT RESPONDING"), "classify unbound")
    check(classify(0.0, 0.5, 0.7).startswith("never reaches"), "classify short range")

    class FakeAC:
        """AC with a configurable Content Manager binding, reading our vJoy output."""
        def __init__(self, nb, gas="ok", brake="ok", steer="ok", gears="ok"):
            self.nb, self.gas, self.brake, self.steer, self.gears = nb, gas, brake, steer, gears
            self.gear, self.prev_btn = 1, (False, False)
        def read(self):
            up, dn = self.nb.btn
            if self.gears != "none" and (up, dn) != self.prev_btn:
                if self.gears == "swap":
                    up, dn = dn, up
                self.gear += (up and not self.prev_btn[0]) - (dn and not self.prev_btn[1])
            self.prev_btn = self.nb.btn
            st, g, b = self.nb.last
            m = {"ok": lambda x: x, "inv": lambda x: 1 - x, "none": lambda x: 0.0, "swap": None}
            gas = b if self.gas == "swap" else m[self.gas](g)
            brk = g if self.brake == "swap" else m[self.brake](b)
            return SimpleNamespace(steer_angle=0.0 if self.steer == "none" else st * 0.8, gas=gas, brake=brk,
                                   gear_raw=self.gear, auto_shifter=False)

    def run(**binding) -> list[str]:
        nb = NullBackend()
        pad = VirtualController(CONFIG.controller, nb)
        return check_bindings(FakeAC(nb, **binding), pad, settle=0, sample=0, sleep=lambda s: None)

    r = run()
    check(r[0].startswith("steering: responds") and r[1] == "throttle: OK" and r[2] == "brake:    OK"
          and "OK" in r[-1], f"all ok: {r}")
    check("NOT RESPONDING" in run(gears="none")[-1], "gears unbound")
    check("SWAPPED" in run(gears="swap")[-1], "gears swapped")
    r = run(gas="inv")
    check("INVERTED" in r[1] and r[2] == "brake:    OK", f"inverted gas: {r}")
    r = run(brake="none", steer="none")
    check("NOT RESPONDING" in r[0] and "NOT RESPONDING" in r[2], f"unbound: {r}")
    r = run(gas="swap", brake="swap")
    check(any("crossed" in x for x in r), f"crossed: {r}")

    rows = [dict(kmh=200.0, conf=0.9, ms=3.0, fps=60.0, n_obs=1, left=None, center=50.0, right=None)] * 10
    lines = summarize_capture(rows, Counter({"L: car beside": 5}), Counter({"FOLLOW": 10}))
    check(any("center" in x and "50 m" in x for x in lines) and any("car beside" in x for x in lines), f"summary: {lines}")
    lines = summarize_capture([dict(rows[0], n_obs=0, center=None)] * 10, Counter(), Counter({"CRUISE": 10}))
    check(any("NONE" in x for x in lines), "summary flags no traffic seen")

    print("SELFTEST", "FAILED:\n  " + "\n  ".join(fails) if fails else "OK")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())

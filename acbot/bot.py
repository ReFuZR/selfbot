"""Auto-driver for Assetto Corsa traffic sessions. Run on the Windows PC that runs the game.

    py bot.py              # drive
    py bot.py --dry-run    # read telemetry + print decisions, send nothing

Stop it with Ctrl+C in this window (or --max-seconds). Only use it on a private session you
and your friends agreed to; public servers (including nohesi.gg) ban automation.
"""
import argparse
import time

from planner import Config, Planner
from telemetry import ACMemory
from gamepad import make_pad


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cruise", type=float, default=180.0, help="target top speed km/h")
    ap.add_argument("--margin", type=float, default=0.55, help="side clearance m (lower = closer passes)")
    ap.add_argument("--steer-sign", type=float, default=1.0, choices=[-1.0, 1.0])
    ap.add_argument("--hz", type=float, default=60.0)
    ap.add_argument("--max-seconds", type=float, default=0.0)
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args()

    cfg = Config(cruise_kmh=a.cruise, margin=a.margin, steer_sign=a.steer_sign)
    planner, pad, mem = Planner(cfg), make_pad(a.dry_run), ACMemory()
    t0 = time.perf_counter()
    period, n = 1.0 / a.hz, 0
    print("Driving. Ctrl+C to stop.")
    try:
        while True:
            t = time.perf_counter() - t0
            if a.max_seconds and t > a.max_seconds:
                break
            s = mem.read()
            if s["live"]:
                c = planner.step(t, s["own_xz"], s["own_vxz"], s["traffic"])
                pad.send(c.steer, c.throttle, c.brake)
                n += 1
                if n % int(a.hz) == 0:
                    print(f"{s['speed_kmh']:6.1f} km/h  cars={len(s['traffic']):2d}  lane={planner.target_lane}  "
                          f"gap={planner.last_gap:6.1f}  steer={c.steer:+.2f} thr={c.throttle:.2f} brk={c.brake:.2f}")
            else:
                pad.send(0.0, 0.0, 0.0)
            time.sleep(max(0.0, period - ((time.perf_counter() - t0) - t)))
    except KeyboardInterrupt:
        pass
    finally:
        pad.close()
        print("Stopped.")


if __name__ == "__main__":
    main()

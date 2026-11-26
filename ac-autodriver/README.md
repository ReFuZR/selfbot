# ac-autodriver

Autonomous highway driver for Assetto Corsa, for private / offline traffic sessions.
It reads AC's telemetry, watches the road through a screen capture, and drives
a virtual Xbox 360 controller.

> Status: telemetry, controller, vision, planner and simulator are done and self-tested.
> `main.py` (the live driving loop with F8 / F12 hotkeys) is the next step.

## Install (once)

1. Put this folder anywhere, e.g. `C:\ac-autodriver`.
2. Double-click **`install.bat`**.

It installs everything it doesn't find:

| Step | What |
|---|---|
| Python 3.11+ | Python 3.12, through `winget` (or python.org if winget is missing) |
| ViGEmBus | the virtual gamepad driver. Windows asks for admin: click **Yes** |
| Packages | numpy, opencv-python, mss, vgamepad, pynput, in a private `.venv` folder |
| Self-tests | every module checks itself; no Assetto Corsa needed |

It's safe to run again; finished steps are skipped. If something fails, send the window's text.

## Use

Double-click **`run.bat`** for a menu:

1. **Telemetry monitor.** Sit in an AC practice session; speed, RPM and gear should match the game.
2. **Controller bind test.** Sweeps steering, throttle and brake so you can bind them (below).
3. **Vision: calibrate.** Writes `calibration.png` showing the road area it watches.
4. **Vision: live view.** Debug window, lane offset, distance to cars per lane.
5. **Simulator.** Compares the driving modes on a fake highway.
6. **Self-tests.**

## One-time game setup

**Start the virtual pad before the session.** AC detects controllers when a session
loads, so start option 2 (later, `main.py`) first, then launch the session.

**Bind it.** In Content Manager: Settings, Assetto Corsa, Controls, pick the Xbox 360
controller. With option 2 running, bind:

- steering to the left stick X axis
- throttle to the right trigger
- brake to the left trigger

**Gamepad settings.** In the same screen, set steering speed sensitivity and steering
filter to 0, gamma to 1, deadzone to 0. Otherwise AC adds its own damping on top
of the planner's. Automatic gearbox: on.

**Calibrate vision.** Use a hood or bumper camera. Run option 3 and open
`calibration.png`. The red box (`roi` in `config.py`) should cover only road:
bottom edge just above the hood, top edge just below the horizon. Edit the
fractions and repeat until it does, then check option 4 on a real highway.

## Modes

Change one line at the top of `config.py`:

```python
MODE = "simple"   # "cruise" | "simple" | "madmax"
```

| Mode | Drives like |
|---|---|
| `cruise` | 180 km/h, long gaps, only overtakes when blocked |
| `simple` | 250 km/h, clean overtakes for pass and speed points |
| `madmax` | 300 km/h, tight gaps, quick lane changes, leans toward cars it passes for near-miss multipliers |

Score (our own model; AC can't read a server's score): passes x speed x combo,
x2.5 for near misses, points per second above 150 km/h, -1000 per crash
(`crash_resets_score = True` makes a crash wipe the run).

## Tuning

Everything is in `config.py`, with comments. The ones you'll most likely touch:

- `roi`: the road area vision watches
- `steer_gain` (per mode): steering too lazy or too twitchy
- `slip_threshold` (per mode): drive hard into a corner with option 1 running, note
  the `Slip` reading, and set it a bit below that
- `obstacle_edge_score`: raise if vision sees phantom cars, lower if it misses real ones

Try changes in the simulator first: `run.bat`, option 5.

## Known limits

- **No rear view.** Vision only sees ahead (6-120 m). The planner remembers cars
  that drop out of view beside it, but a faster car coming from behind is invisible.
  This shows up when the car has been slowed below about 100 km/h and then pulls
  out. Reading AC's virtual mirror as a second view would fix it.
- Distances from vision are rough until calibrated (`near_distance_m`, `far_distance_m`).

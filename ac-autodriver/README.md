# ac-autodriver

Autonomous highway driver for Assetto Corsa, for private / offline traffic sessions.
It reads AC's telemetry, watches the road through a screen capture, and drives a
**virtual wheel (vJoy)** that sits next to your Logitech:

- **Engaged (F8):** the bot drives. Touch your wheel or a pedal and it hands control back.
- **Paused:** your Logitech drives the car through the virtual wheel, 1:1, so you don't
  have to switch control presets.

## 1. Install (once)

Put this folder anywhere (e.g. `C:\ac-autodriver`) and double-click **`install.bat`**.
It installs what's missing and skips what's there:

| Step | What |
|---|---|
| Python 3.12 | via `winget`, or python.org if winget is missing |
| vJoy | the virtual wheel driver. Windows asks for admin and to trust the driver: **Yes / Install** |
| Packages | numpy, opencv-python, mss, pynput, pygame, in a private `.venv` folder |
| Self-tests | every module checks itself; no game needed |
| Wheel setup | plug in your Logitech; it asks you to turn the wheel and press each pedal (~30 s) |

## 2. Content Manager setup (once)

Make a **new controls preset** so your normal one stays untouched.

1. Content Manager: Settings, Assetto Corsa, Controls. Input method: **Wheel**.
2. Double-click `run.bat` and pick **3 (Bind test)**. It moves the virtual wheel's
   steering, then gas, then brake, 6 seconds each, over and over.
3. In Content Manager click **Steering**, and wait for the STEER phase so vJoy's axis gets picked.
   Do the same for **Throttle** (THROTTLE phase) and **Brake** (BRAKE phase).
   Ctrl+C the bind test when done.
4. Leave **gear shifting and all buttons on your Logitech** (paddles still work).
5. Set the steering **degrees of rotation to the same as your Logitech** (usually 900), so
   driving through pass-through feels the same as normal.
6. **Automatic gearbox: on** (AC assists). The bot can't shift and won't engage without it.
7. Save the preset, e.g. `autodriver`. Load your old preset to drive without the bot.

Force feedback goes to the steering device, which is now vJoy, so your Logitech will
feel limp in this preset. That's expected.

## 3. Calibrate vision (once)

Use a hood or bumper camera. `run.bat` → **5** writes `calibration.png`. The red box
(`roi` in `config.py`) must cover only road: bottom edge just above the hood, top edge
just below the horizon. Edit the fractions, repeat, then check **6 (live view)** on a
highway: lane offset goes negative when you're left of centre, and cars ahead show a
distance.

## 4. Drive

1. Load a highway with traffic in AC (practice / offline) using the `autodriver` preset.
2. `run.bat` → **1**, pick a mode.
3. Drive onto the highway yourself (pass-through), get into a lane, press **F8**.

| Key | |
|---|---|
| **F8** | engage / pause |
| **F9** | cycle mode: cruise, simple, madmax |
| **F12** | kill: wheel centred, pedals up, program exits |

It pauses by itself when you touch the wheel or pedals, the game is paused, it
crashes, vision loses the lanes, or the car is stuck. Every drive is logged to
`logs\run-*.csv`; send me one if it misbehaves.

If F8 does nothing while AC has focus, AC is probably running as administrator: run
`run.bat` as administrator too.

## Modes

Pick in the menu, with F9, or with one line in `config.py` (`MODE = "simple"`).

| Mode | Drives like |
|---|---|
| `cruise` | 180 km/h, long gaps, only overtakes when blocked |
| `simple` | 250 km/h, clean overtakes for pass and speed points |
| `madmax` | 300 km/h, tight gaps, quick lane changes, leans toward cars it passes for near-miss multipliers |

Score (our own model; AC can't read a server's score): passes x speed x combo,
x2.5 for near misses, points per second above 150 km/h, -1000 per crash
(`crash_resets_score = True` makes a crash wipe the run).

## First-drive tuning

Start in **cruise**. All settings are in `config.py`, with comments.

| Symptom | Change |
|---|---|
| turns lazily, drifts in the lane | raise `steer_output_scale` (0.55), or the mode's `steer_gain` |
| weaves / oscillates | lower them |
| hands control back by itself | raise `takeover_steer` / `takeover_pedal` (`WheelConfig`) |
| phantom cars | raise `obstacle_edge_score`; missed cars: lower `obstacle_value_delta` |
| slides in corners | lower the mode's `slip_threshold` (watch `slip` in the HUD in a hard corner) |

Try planner changes in the simulator first (`run.bat` → 7).

## Gamepad instead of a wheel

Set `output = "xbox"` in `config.py` and run `install.bat -Xbox` (installs the ViGEmBus
driver and vgamepad). Bind the Xbox 360 controller in a Gamepad preset; no pass-through.

## Known limits

- **No rear view.** Vision only sees ahead (6-120 m). The planner remembers cars that
  drop out of view beside it, but a faster car coming from behind is invisible. This
  shows up when the car has been slowed below about 100 km/h and then pulls out.
- Distances from vision are rough until calibrated (`near_distance_m`, `far_distance_m`).
- Nothing has run in the real game yet; expect the first drives to need the tuning above.

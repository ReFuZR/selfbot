# acbot: traffic-weaving auto-driver for Assetto Corsa

Drives through AI traffic on a **private** Assetto Corsa session for you and your friends.
Don't point it at public servers. nohesi.gg and similar communities ban automation and it ruins their leaderboards.

## How it works
- `telemetry.py` reads AC's shared memory (own speed/velocity and every car's world position).
- `planner.py` builds a road frame from the direction you were heading when it started, then picks a
  lane one step at a time, matches speed to the car ahead, and steers with a self-calibrating gain.
- `gamepad.py` sends the result through a virtual Xbox 360 pad (left stick = steering, triggers = gas/brake).
- `sim.py` / `test_bot.py` test the planner offline against simulated highway traffic. The tests run on any OS.

## Setup (Windows, on the PC running the game)
1. Install Python 3.10+ and [ViGEmBus](https://github.com/nefarius/ViGEmBus/releases).
2. `pip install -r requirements.txt`
3. In AC's controls menu, choose **Gamepad** and set steering/throttle/brake to the Xbox pad.
4. Start a session with traffic, get up to speed on a straight road, then run `py bot.py`.

Try `py bot.py --dry-run` first. It prints what the bot would do without sending any input.

## Tuning
| Flag | Meaning |
|---|---|
| `--cruise 220` | top speed target in km/h |
| `--margin 0.3` | side clearance in metres. Lower means closer, scarier passes |
| `--steer-sign -1` | use this if the car steers the wrong way |

Lane centres in `planner.Config.lane_centers` assume a 4-lane road with 3.5 m lanes, relative to where you
started. Edit them for your road. Start in the middle of the road, pointing along it.

## Known limits
- It's tested only against a simple simulator, not the real game. Expect to tune steering and `--steer-sign`.
- The road frame is a straight line, so it is built for highways, not twisty roads.
- Press Ctrl+C in the console to stop. It then holds the brake.

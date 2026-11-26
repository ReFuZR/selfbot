"""
config.py - every tunable in one place.

Most people only change MODE. Each mode is a preset of how hard the car drives;
tweak a preset below or add your own. Everything else is plumbing you tune once.
Values are starting points: try them in sim.py, then on an empty road in AC.
"""

from __future__ import annotations

from dataclasses import dataclass, field

MODE = "simple"   # "cruise" | "simple" | "madmax"   (override with  python main.py --mode madmax)


@dataclass
class Mode:
    description: str
    target_kmh: float            # cruise speed with a clear lane
    follow_base_m: float         # braking distance = follow_base_m + follow_per_kmh_m * speed
    follow_per_kmh_m: float
    overtake_factor: float       # start a lane change at this x braking distance
    min_gap_m: float             # target lane must be clear at least this far ahead
    blind_spot_s: float          # extra seconds of gap a passed car must have behind us before we cut in
    lane_change_s: float         # time to move one lane over
    cooldown_s: float            # minimum time between lane changes
    hug: float                   # lean toward cars being passed (half-lane widths) - near-miss farming
    steer_gain: float            # overall steering aggressiveness
    slip_threshold: float        # wheelSlip above this trims throttle / steering (calibrate in AC)


MODES: dict[str, Mode] = {
    "cruise": Mode("Relaxed, safe gaps, only changes lane when blocked. Scores, but doesn't chase.",
                   target_kmh=180, follow_base_m=30, follow_per_kmh_m=0.70, overtake_factor=1.6,
                   min_gap_m=45, blind_spot_s=2.5, lane_change_s=3.0, cooldown_s=6.0,
                   hug=0.0, steer_gain=0.9, slip_threshold=2.5),
    "simple": Mode("Simple scoring: high speed and clean overtakes for pass + speed points.",
                   target_kmh=250, follow_base_m=22, follow_per_kmh_m=0.50, overtake_factor=1.4,
                   min_gap_m=30, blind_spot_s=1.6, lane_change_s=2.2, cooldown_s=2.5,
                   hug=0.0, steer_gain=1.0, slip_threshold=3.0),
    "madmax": Mode("Max out the score model: flat out, late swerves, tight gaps, hugs traffic "
                   "for near-miss multipliers, keeps the combo alive.",
                   target_kmh=300, follow_base_m=20, follow_per_kmh_m=0.36, overtake_factor=1.2,
                   min_gap_m=18, blind_spot_s=0.9, lane_change_s=2.0, cooldown_s=1.5,
                   hug=0.4, steer_gain=1.2, slip_threshold=3.4),
}


@dataclass
class ScoreConfig:
    """Our own score model (AC's shared memory can't read a server/CSP script score)."""
    pass_points: float = 100.0       # per car passed, x (speed / 100 km/h) x combo
    near_miss_lean: float = 0.25     # leaning this far toward the passed car = near miss
    near_miss_mult: float = 2.5
    pass_trigger_m: float = 15.0     # a side car must get this close to count as being passed
    speed_floor_kmh: float = 150.0   # speed points accrue above this
    speed_points_per_s: float = 10.0 # per second, per 100 km/h above the floor
    combo_window_s: float = 6.0      # pass again within this to grow the combo
    combo_max: int = 10
    crash_penalty: float = 1000.0
    crash_resets_score: bool = False # True = No-Hesi style: a crash wipes the run


@dataclass
class LoopConfig:
    hz: float = 60.0                 # control loop rate
    hud_hz: float = 10.0             # terminal HUD refresh rate


@dataclass
class SpeedConfig:
    """Cruise PID on speed error (km/h). Output in [-1, 1]: >0 throttle, <0 brake."""
    kp: float = 0.045
    ki: float = 0.006
    kd: float = 0.010
    integral_limit: float = 0.35     # anti-windup clamp on the I term (output units)
    brake_deadband_kmh: float = 6.0  # no PID braking until this far OVER target (coast instead)
    throttle_slew_per_s: float = 2.5 # max throttle change per second
    brake_slew_per_s: float = 5.0
    brake_decel_mps2: float = 10.0  # what full brake achieves at speed (GT cars ~12-15). Used to
                                    # brake on time-to-impact and size safe overtaking gaps.


@dataclass
class SteerConfig:
    """Lateral PD on lane offset (half-lane widths; vision reports it)."""
    kp: float = 0.12                 # per half-lane of position error
    kd: float = 0.09                 # per half-lane/s of lateral speed
    k_heading: float = 0.0           # feed-forward on vision lane heading (curvy roads: try 0.2)
    max_steer: float = 1.0
    # speed damping: steer *= 1 / (1 + (speed / ref) ** exponent); exponent 2 keeps the
    # car's lateral response roughly constant above ref (lateral accel grows with v^2)
    damp_ref_kmh: float = 90.0
    damp_exponent: float = 2.0
    min_steer_scale: float = 0.03
    slew_per_s: float = 3.0          # max steer change per second at low speed...
    slew_high_speed_per_s: float = 1.0   # ...and at/above 2x damp_ref_kmh


@dataclass
class StabilityConfig:
    """Throttle/steer trim when wheelSlip exceeds the mode's slip_threshold."""
    slip_trim_gain: float = 0.12     # throttle removed per unit of slip over threshold
    max_trim: float = 0.60
    steer_trim_gain: float = 0.08    # steer reduction per unit of slip over threshold
    recover_per_s: float = 1.5


@dataclass
class VisionConfig:
    """mss + OpenCV. Rectangles are fractions (left, top, right, bottom) so they survive
    resolution changes. Calibrate with the README procedure."""
    monitor_index: int = 1           # mss monitor (1 = primary)
    capture_region: tuple[float, float, float, float] | None = None  # None = full monitor
    roi: tuple[float, float, float, float] = (0.20, 0.45, 0.80, 0.72)  # road ahead of the hood
    zone_split: tuple[float, float] = (0.33, 0.67)  # ROI x-fractions: left | center | right
    downscale_width: int = 480       # ROI is resized to this width before processing

    # Lane lines
    white_thresh: int = 200
    yellow_hsv_lo: tuple[int, int, int] = (15, 80, 120)
    yellow_hsv_hi: tuple[int, int, int] = (35, 255, 255)
    canny_lo: int = 60
    canny_hi: int = 150
    hough_threshold: int = 25
    hough_min_len: int = 20
    hough_max_gap: int = 12

    car_center_x: float = 0.5        # where the car's centreline sits across the ROI (0..1)

    # Obstacles: blobs that differ from the road colour sampled just ahead of the hood
    obstacle_value_delta: int = 40          # brightness difference from road to count
    obstacle_sat_delta: int = 60            # saturation above road (coloured paint, tail lights)
    obstacle_min_area_frac: float = 0.004   # of ROI area
    # Edge pixels inside a blob's box per pixel of box perimeter (sides on the ROI border
    # excluded). Scale-free: a crisp outline alone scores ~1, cars with glass / lights /
    # plates score 1.3+, shadows 0-0.4. Raise if you get phantom obstacles.
    obstacle_edge_score: float = 0.7
    # Perspective distance: d = k / (y - horizon); pinned by these two points
    near_distance_m: float = 8.0            # at the bottom row of the ROI (uncalibrated)
    far_distance_m: float = 120.0           # at the top row of the ROI
    max_fps: float = 60.0                   # cap capture rate so AC keeps its CPU
    debug_window: bool = False              # cv2.imshow overlay (costs a few ms)


@dataclass
class ExternalObstacleConfig:
    """Optional feed from a Custom Shaders Patch Lua script over UDP (later)."""
    enabled: bool = False
    host: str = "127.0.0.1"
    port: int = 9996
    max_age_s: float = 0.25          # drop obstacle packets older than this


@dataclass
class ControllerConfig:
    # |steer| below this is snapped to 0. Keep it tiny: at 250 km/h the speed-damped
    # corrections are only 0.005-0.02, and a bigger deadzone deletes them.
    steer_deadzone: float = 0.002
    trigger_deadzone: float = 0.02   # same for throttle / brake
    # AC's own gamepad deadzone (Content Manager -> Controls -> steering "deadzone"). Non-zero
    # steer output is rescaled to start just past it. Set to the same value you set in-game.
    steer_game_deadzone: float = 0.0
    # Tap the A button once on connect. Only for binding screens that ignore an idle pad;
    # off by default because A may be bound to something in-game.
    wake_on_connect: bool = False
    steer_smoothing: float = 0.35    # EMA alpha per 60 Hz frame (1 = no smoothing)
    throttle_smoothing: float = 0.50
    brake_smoothing: float = 0.50
    invert_steer: bool = False


@dataclass
class HotkeyConfig:
    toggle: str = "f8"               # engage / pause   (pynput key names)
    kill: str = "f12"                # zero all axes and exit immediately


@dataclass
class Config:
    mode: str = MODE
    modes: dict[str, Mode] = field(default_factory=lambda: dict(MODES))
    score: ScoreConfig = field(default_factory=ScoreConfig)
    loop: LoopConfig = field(default_factory=LoopConfig)
    speed: SpeedConfig = field(default_factory=SpeedConfig)
    steer: SteerConfig = field(default_factory=SteerConfig)
    stability: StabilityConfig = field(default_factory=StabilityConfig)
    vision: VisionConfig = field(default_factory=VisionConfig)
    external: ExternalObstacleConfig = field(default_factory=ExternalObstacleConfig)
    controller: ControllerConfig = field(default_factory=ControllerConfig)
    hotkeys: HotkeyConfig = field(default_factory=HotkeyConfig)

    @property
    def active(self) -> Mode:
        if self.mode not in self.modes:
            raise ValueError(f"unknown mode {self.mode!r}; choose from {', '.join(self.modes)}")
        return self.modes[self.mode]


CONFIG = Config()

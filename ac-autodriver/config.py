"""
config.py - every tunable in one place.

All values are starting points. Tune them in a practice session on an empty
road before trusting them at speed. Units are noted per field.
"""

from __future__ import annotations

from dataclasses import dataclass, field


# --------------------------------------------------------------------------- #
# Main loop
# --------------------------------------------------------------------------- #
@dataclass
class LoopConfig:
    hz: float = 60.0                 # control loop rate
    hud_hz: float = 10.0             # terminal HUD refresh rate
    telemetry_wait_s: float = 2.0    # retry interval while waiting for AC to start


# --------------------------------------------------------------------------- #
# Speed / cruise control (PID on speed error, km/h)
# --------------------------------------------------------------------------- #
@dataclass
class SpeedConfig:
    target_kmh: float = 240.0        # cruise speed when lane is clear (220-280 typical)
    min_kmh: float = 60.0            # never plan below this when merely following

    # PID gains. Output is in [-1, 1]: >0 = throttle, <0 = brake.
    kp: float = 0.045
    ki: float = 0.006
    kd: float = 0.010
    integral_limit: float = 0.35     # anti-windup clamp on the I term (output units)
    max_throttle: float = 1.0
    max_brake: float = 1.0
    brake_deadband_kmh: float = 6.0  # don't touch brake until this far OVER target
                                     # (coast/engine-brake instead of dabbing)
    throttle_slew_per_s: float = 2.5 # max throttle change per second (smoothness)
    brake_slew_per_s: float = 4.0


# --------------------------------------------------------------------------- #
# Steering
# --------------------------------------------------------------------------- #
@dataclass
class SteerConfig:
    # Lane keeping PID: input is lateral offset error in [-1, 1]
    # (-1 = lane line far left of centre, +1 = far right).
    kp: float = 0.55
    ki: float = 0.00
    kd: float = 0.12

    max_steer: float = 1.0           # absolute clamp on the stick axis [-1, 1]

    # Speed damping: steer *= 1 / (1 + (speed / ref_kmh) ** exponent)
    # Higher exponent -> sharper fall-off past ref_kmh.
    damp_ref_kmh: float = 90.0
    damp_exponent: float = 1.6
    min_steer_scale: float = 0.06    # floor so the car can still change lane at 280

    # Lane-change shaping
    lane_change_steer: float = 0.22  # peak steer command for a lane change (pre-damping)
    lane_change_duration_s: float = 2.2
    slew_per_s: float = 3.0          # max steer change per second at low speed
    slew_high_speed_per_s: float = 0.9   # ...and at/above damp_ref_kmh * 2


# --------------------------------------------------------------------------- #
# Stability guard (uses wheelSlip from physics)
# --------------------------------------------------------------------------- #
@dataclass
class StabilityConfig:
    slip_threshold: float = 4.0      # AC wheelSlip units; spike above this -> trim
    slip_trim_gain: float = 0.12     # throttle removed per unit of slip over threshold
    max_trim: float = 0.60           # never remove more than this fraction of throttle
    steer_trim_gain: float = 0.08    # extra steer reduction per unit of slip over threshold
    recover_per_s: float = 1.5       # how fast the trim relaxes once slip is back to normal


# --------------------------------------------------------------------------- #
# Vision (mss + OpenCV)
# ROI values are fractions of the captured monitor/window: (left, top, right, bottom)
# so they survive resolution changes. Calibrate with the README procedure.
# --------------------------------------------------------------------------- #
@dataclass
class VisionConfig:
    monitor_index: int = 1           # mss monitor (1 = primary)
    capture_region: tuple[float, float, float, float] | None = None
    # ^ None = full monitor. Or fractions to capture only the AC window area.

    # Forward region of interest (road ahead of the hood), fractions of the capture.
    roi: tuple[float, float, float, float] = (0.20, 0.45, 0.80, 0.72)

    # Split ROI horizontally into Left / Center / Right zones by these x-fractions
    # *of the ROI width*: [0, a) = left, [a, b) = center, [b, 1] = right.
    zone_split: tuple[float, float] = (0.33, 0.67)

    downscale_width: int = 480       # ROI is resized to this width before processing

    # Lane-line detection
    white_thresh: int = 200          # V/L threshold for white lines
    yellow_hsv_lo: tuple[int, int, int] = (15, 80, 120)
    yellow_hsv_hi: tuple[int, int, int] = (35, 255, 255)
    canny_lo: int = 60
    canny_hi: int = 150
    hough_threshold: int = 25
    hough_min_len: int = 20
    hough_max_gap: int = 12

    # Obstacle detection (dark/edge-dense blobs on the road surface)
    obstacle_min_area_frac: float = 0.004   # of ROI area
    obstacle_edge_density: float = 0.10     # min edge density in a zone to count as blocked
    # Rough distance mapping: bottom of ROI = near, top = far (metres, uncalibrated)
    near_distance_m: float = 8.0
    far_distance_m: float = 120.0

    debug_window: bool = False       # cv2.imshow overlay (costs a few ms)


# --------------------------------------------------------------------------- #
# Planner
# --------------------------------------------------------------------------- #
@dataclass
class PlannerConfig:
    # Dynamic braking distance (metres) = base + speed_kmh * per_kmh
    brake_dist_base_m: float = 25.0
    brake_dist_per_kmh_m: float = 0.55   # ~140 m at 210 km/h
    overtake_dist_factor: float = 1.4    # start evaluating a lane change at factor * brake dist
    min_gap_m: float = 30.0              # adjacent lane must be clear at least this far
    lane_change_cooldown_s: float = 4.0
    progressive_brake_gain: float = 0.012   # brake per metre inside the braking distance


# --------------------------------------------------------------------------- #
# Optional external obstacle feed (Custom Shaders Patch Lua -> UDP, later)
# --------------------------------------------------------------------------- #
@dataclass
class ExternalObstacleConfig:
    enabled: bool = False
    host: str = "127.0.0.1"
    port: int = 9996
    max_age_s: float = 0.25          # drop obstacle packets older than this


# --------------------------------------------------------------------------- #
# Virtual controller
# --------------------------------------------------------------------------- #
@dataclass
class ControllerConfig:
    steer_deadzone: float = 0.015    # |steer| below this is snapped to 0 (kills stick jitter)
    trigger_deadzone: float = 0.02   # same for throttle / brake
    # AC's own gamepad deadzone (Content Manager -> Controls -> steering "deadzone").
    # Non-zero output is rescaled to start just past it so small corrections aren't swallowed.
    # Set this to the same value you configure in-game (0.0 if you set it to 0).
    steer_game_deadzone: float = 0.0
    wake_on_connect: bool = True     # tap a button once so Windows/AC notice the new pad
    steer_smoothing: float = 0.35    # EMA alpha (1 = no smoothing)
    throttle_smoothing: float = 0.50
    brake_smoothing: float = 0.50
    invert_steer: bool = False


# --------------------------------------------------------------------------- #
# Hotkeys (pynput key names; see pynput.keyboard.Key)
# --------------------------------------------------------------------------- #
@dataclass
class HotkeyConfig:
    toggle: str = "f8"               # engage / pause
    kill: str = "f12"                # zero all axes and exit immediately


# --------------------------------------------------------------------------- #
# Aggregate
# --------------------------------------------------------------------------- #
@dataclass
class Config:
    loop: LoopConfig = field(default_factory=LoopConfig)
    speed: SpeedConfig = field(default_factory=SpeedConfig)
    steer: SteerConfig = field(default_factory=SteerConfig)
    stability: StabilityConfig = field(default_factory=StabilityConfig)
    vision: VisionConfig = field(default_factory=VisionConfig)
    planner: PlannerConfig = field(default_factory=PlannerConfig)
    external: ExternalObstacleConfig = field(default_factory=ExternalObstacleConfig)
    controller: ControllerConfig = field(default_factory=ControllerConfig)
    hotkeys: HotkeyConfig = field(default_factory=HotkeyConfig)


CONFIG = Config()

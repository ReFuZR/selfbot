"""
telemetry.py - Assetto Corsa shared-memory reader (ctypes only, no dependencies).

AC publishes three memory-mapped pages while the sim is running:

    Local\\acpmf_physics   ~333 Hz   car dynamics (speed, rpm, gear, slip, inputs)
    Local\\acpmf_graphics  ~60 Hz    session / lap / position info
    Local\\acpmf_static    once      car + track names, max rpm, etc.

Layouts follow Kunos' published SharedFileOut.h, compiled with `#pragma pack(4)`.
Strings are UTF-16 (wchar_t is 2 bytes on Windows); they are declared here as
uint16 arrays so the struct layout is identical on any OS (handy for testing).

The structs deliberately stop at the last field that exists in every AC 1.x
build.  We map the whole section (length 0), so newer, longer pages are fine -
we just ignore the tail.

Usage:
    python telemetry.py              # live speed/RPM/gear monitor
    python telemetry.py --once       # one JSON snapshot
    python telemetry.py --selftest   # offline layout check (works without AC)
"""

from __future__ import annotations

import argparse
import ctypes
import json
import sys
import time
from dataclasses import asdict, dataclass

# --------------------------------------------------------------------------- #
# Enums (plain ints as stored in memory)
# --------------------------------------------------------------------------- #
AC_OFF, AC_REPLAY, AC_LIVE, AC_PAUSE = 0, 1, 2, 3
STATUS_NAMES = {AC_OFF: "OFF", AC_REPLAY: "REPLAY", AC_LIVE: "LIVE", AC_PAUSE: "PAUSE"}
SESSION_NAMES = {
    -1: "UNKNOWN", 0: "PRACTICE", 1: "QUALIFY", 2: "RACE",
    3: "HOTLAP", 4: "TIME_ATTACK", 5: "DRIFT", 6: "DRAG",
}

_F = ctypes.c_float
_I = ctypes.c_int32
_W = ctypes.c_uint16  # one UTF-16 code unit


# --------------------------------------------------------------------------- #
# Structs
# --------------------------------------------------------------------------- #
class SPageFilePhysics(ctypes.Structure):
    _pack_ = 4
    _fields_ = [
        ("packetId", _I),
        ("gas", _F),
        ("brake", _F),
        ("fuel", _F),
        ("gear", _I),                    # 0 = R, 1 = N, 2 = 1st, ...
        ("rpms", _I),
        ("steerAngle", _F),
        ("speedKmh", _F),
        ("velocity", _F * 3),
        ("accG", _F * 3),
        ("wheelSlip", _F * 4),           # FL FR RL RR
        ("wheelLoad", _F * 4),
        ("wheelsPressure", _F * 4),
        ("wheelAngularSpeed", _F * 4),
        ("tyreWear", _F * 4),
        ("tyreDirtyLevel", _F * 4),
        ("tyreCoreTemperature", _F * 4),
        ("camberRAD", _F * 4),
        ("suspensionTravel", _F * 4),
        ("drs", _F),
        ("tc", _F),
        ("heading", _F),
        ("pitch", _F),
        ("roll", _F),
        ("cgHeight", _F),
        ("carDamage", _F * 5),
        ("numberOfTyresOut", _I),
        ("pitLimiterOn", _I),
        ("abs", _F),
        ("kersCharge", _F),
        ("kersInput", _F),
        ("autoShifterOn", _I),
        ("rideHeight", _F * 2),
        ("turboBoost", _F),
        ("ballast", _F),
        ("airDensity", _F),
        ("airTemp", _F),
        ("roadTemp", _F),
        ("localAngularVel", _F * 3),
        ("finalFF", _F),
        ("performanceMeter", _F),
        ("engineBrake", _I),
        ("ersRecoveryLevel", _I),
        ("ersPowerLevel", _I),
        ("ersHeatCharging", _I),
        ("ersIsCharging", _I),
        ("kersCurrentKJ", _F),
        ("drsAvailable", _I),
        ("drsEnabled", _I),
        ("brakeTemp", _F * 4),
        ("clutch", _F),
        ("tyreTempI", _F * 4),
        ("tyreTempM", _F * 4),
        ("tyreTempO", _F * 4),
        ("isAIControlled", _I),
        ("tyreContactPoint", (_F * 3) * 4),
        ("tyreContactNormal", (_F * 3) * 4),
        ("tyreContactHeading", (_F * 3) * 4),
        ("brakeBias", _F),
        ("localVelocity", _F * 3),
    ]


class SPageFileGraphic(ctypes.Structure):
    _pack_ = 4
    _fields_ = [
        ("packetId", _I),
        ("status", _I),                  # AC_STATUS
        ("session", _I),                 # AC_SESSION_TYPE
        ("currentTime", _W * 15),
        ("lastTime", _W * 15),
        ("bestTime", _W * 15),
        ("split", _W * 15),
        ("completedLaps", _I),
        ("position", _I),
        ("iCurrentTime", _I),            # ms
        ("iLastTime", _I),
        ("iBestTime", _I),
        ("sessionTimeLeft", _F),
        ("distanceTraveled", _F),
        ("isInPit", _I),
        ("currentSectorIndex", _I),
        ("lastSectorTime", _I),
        ("numberOfLaps", _I),
        ("tyreCompound", _W * 33),
        ("replayTimeMultiplier", _F),
        ("normalizedCarPosition", _F),   # 0..1 around the lap
        ("carCoordinates", _F * 3),      # world X, Y, Z of the player car
        ("penaltyTime", _F),
        ("flag", _I),
        ("idealLineOn", _I),
        ("isInPitLane", _I),
        ("surfaceGrip", _F),
        ("mandatoryPitDone", _I),
        ("windSpeed", _F),
        ("windDirection", _F),
    ]


class SPageFileStatic(ctypes.Structure):
    _pack_ = 4
    _fields_ = [
        ("smVersion", _W * 15),
        ("acVersion", _W * 15),
        ("numberOfSessions", _I),
        ("numCars", _I),
        ("carModel", _W * 33),
        ("track", _W * 33),
        ("playerName", _W * 33),
        ("playerSurname", _W * 33),
        ("playerNick", _W * 33),
        ("sectorCount", _I),
        ("maxTorque", _F),
        ("maxPower", _F),
        ("maxRpm", _I),
        ("maxFuel", _F),
        ("suspensionMaxTravel", _F * 4),
        ("tyreRadius", _F * 4),
        ("maxTurboBoost", _F),
    ]


def _wstr(arr) -> str:
    """UTF-16 code-unit array -> str (stops at first NUL)."""
    return bytes(arr).decode("utf-16-le", errors="ignore").split("\0", 1)[0]


# --------------------------------------------------------------------------- #
# Flattened snapshot handed to the rest of the program
# --------------------------------------------------------------------------- #
@dataclass
class Snapshot:
    # timing / validity
    t: float                     # time.perf_counter() when read
    physics_packet: int
    graphics_packet: int
    status: int                  # AC_OFF / REPLAY / LIVE / PAUSE
    session: int

    # driving state
    speed_kmh: float
    rpm: int
    gear_raw: int                # AC encoding: 0=R, 1=N, 2=1st ...
    gas: float
    brake: float
    steer_angle: float
    clutch: float
    heading: float               # rad
    velocity: tuple[float, float, float]
    local_velocity: tuple[float, float, float]
    acc_g: tuple[float, float, float]   # (lateral, vertical, longitudinal) in AC
    wheel_slip: tuple[float, float, float, float]
    tyres_out: int
    is_ai_controlled: bool

    # position
    normalized_pos: float
    car_xyz: tuple[float, float, float]
    completed_laps: int
    in_pit_lane: bool

    # static (cached)
    car_model: str
    track: str
    max_rpm: int

    @property
    def gear(self) -> int:
        """Human gear: -1 = reverse, 0 = neutral, 1..n = forward gears."""
        return self.gear_raw - 1

    @property
    def gear_label(self) -> str:
        g = self.gear
        return "R" if g < 0 else "N" if g == 0 else str(g)

    @property
    def is_live(self) -> bool:
        return self.status == AC_LIVE

    @property
    def max_wheel_slip(self) -> float:
        return max(abs(s) for s in self.wheel_slip)

    @property
    def status_name(self) -> str:
        return STATUS_NAMES.get(self.status, f"?{self.status}")

    @property
    def session_name(self) -> str:
        return SESSION_NAMES.get(self.session, f"?{self.session}")


def build_snapshot(p: SPageFilePhysics, g: SPageFileGraphic, s: SPageFileStatic,
                   t: float | None = None) -> Snapshot:
    return Snapshot(
        t=time.perf_counter() if t is None else t,
        physics_packet=p.packetId,
        graphics_packet=g.packetId,
        status=g.status,
        session=g.session,
        speed_kmh=p.speedKmh,
        rpm=p.rpms,
        gear_raw=p.gear,
        gas=p.gas,
        brake=p.brake,
        steer_angle=p.steerAngle,
        clutch=p.clutch,
        heading=p.heading,
        velocity=tuple(p.velocity),
        local_velocity=tuple(p.localVelocity),
        acc_g=tuple(p.accG),
        wheel_slip=tuple(p.wheelSlip),
        tyres_out=p.numberOfTyresOut,
        is_ai_controlled=bool(p.isAIControlled),
        normalized_pos=g.normalizedCarPosition,
        car_xyz=tuple(g.carCoordinates),
        completed_laps=g.completedLaps,
        in_pit_lane=bool(g.isInPitLane),
        car_model=_wstr(s.carModel),
        track=_wstr(s.track),
        max_rpm=s.maxRpm,
    )


# --------------------------------------------------------------------------- #
# Windows shared-memory access
# --------------------------------------------------------------------------- #
class TelemetryUnavailable(RuntimeError):
    """AC is not running / pages not present / not on Windows."""


class _Page:
    """Read-only view of one named AC file mapping.

    Uses OpenFileMappingW (fails if the mapping does not exist) instead of
    mmap.mmap(tagname=...), which would silently *create* an empty zero-filled
    mapping when AC is not running and make every reading look like 0.
    """

    FILE_MAP_READ = 0x0004

    def __init__(self, name: str, struct_type: type[ctypes.Structure]):
        if sys.platform != "win32":
            raise TelemetryUnavailable("AC shared memory is only available on Windows")
        from ctypes import wintypes

        k32 = ctypes.WinDLL("kernel32", use_last_error=True)
        k32.OpenFileMappingW.argtypes = [wintypes.DWORD, wintypes.BOOL, wintypes.LPCWSTR]
        k32.OpenFileMappingW.restype = wintypes.HANDLE
        k32.MapViewOfFile.argtypes = [wintypes.HANDLE, wintypes.DWORD, wintypes.DWORD,
                                      wintypes.DWORD, ctypes.c_size_t]
        k32.MapViewOfFile.restype = ctypes.c_void_p
        k32.UnmapViewOfFile.argtypes = [ctypes.c_void_p]
        k32.CloseHandle.argtypes = [wintypes.HANDLE]
        self._k32 = k32

        self.name = name
        self.type = struct_type
        self._size = ctypes.sizeof(struct_type)
        self._handle = k32.OpenFileMappingW(self.FILE_MAP_READ, False, name)
        if not self._handle:
            raise TelemetryUnavailable(
                f"{name} not found (error {ctypes.get_last_error()}) - is Assetto Corsa running?")
        # length 0 -> map the entire section, so a longer real page is fine
        self._view = k32.MapViewOfFile(self._handle, self.FILE_MAP_READ, 0, 0, 0)
        if not self._view:
            err = ctypes.get_last_error()
            k32.CloseHandle(self._handle)
            self._handle = None
            raise TelemetryUnavailable(f"MapViewOfFile({name}) failed (error {err})")

    def read(self):
        # string_at copies, so we never hold a pointer into live memory
        return self.type.from_buffer_copy(ctypes.string_at(self._view, self._size))

    def close(self) -> None:
        if getattr(self, "_view", None):
            self._k32.UnmapViewOfFile(self._view)
            self._view = None
        if getattr(self, "_handle", None):
            self._k32.CloseHandle(self._handle)
            self._handle = None


class ACTelemetry:
    """Reader for all three AC pages.

        with ACTelemetry() as tm:
            tm.connect(timeout=30)
            snap = tm.read()
    """

    PREFIX = "Local\\"

    def __init__(self) -> None:
        self._phys: _Page | None = None
        self._gfx: _Page | None = None
        self._stat: _Page | None = None
        self._static: SPageFileStatic | None = None

    # -- lifecycle ---------------------------------------------------------- #
    @property
    def connected(self) -> bool:
        return self._phys is not None

    def try_connect(self) -> bool:
        if self.connected:
            return True
        pages: list[_Page] = []
        try:
            for suffix, typ in (("physics", SPageFilePhysics),
                                ("graphics", SPageFileGraphic),
                                ("static", SPageFileStatic)):
                pages.append(_Page(f"{self.PREFIX}acpmf_{suffix}", typ))
        except TelemetryUnavailable:
            for p in pages:
                p.close()
            return False
        self._phys, self._gfx, self._stat = pages
        self._static = self._stat.read()
        return True

    def connect(self, timeout: float | None = None, poll_s: float = 1.0) -> bool:
        """Wait (up to `timeout` seconds, forever if None) for AC to publish memory."""
        deadline = None if timeout is None else time.monotonic() + timeout
        while not self.try_connect():
            if deadline is not None and time.monotonic() >= deadline:
                return False
            time.sleep(poll_s)
        return True

    def close(self) -> None:
        for p in (self._phys, self._gfx, self._stat):
            if p is not None:
                p.close()
        self._phys = self._gfx = self._stat = self._static = None

    def __enter__(self) -> "ACTelemetry":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    # -- reading ------------------------------------------------------------ #
    def refresh_static(self) -> None:
        """Re-read the static page (car/track change between sessions)."""
        if self._stat is not None:
            self._static = self._stat.read()

    def read(self) -> Snapshot:
        if not self.connected:
            raise TelemetryUnavailable("not connected")
        assert self._phys and self._gfx and self._static
        return build_snapshot(self._phys.read(), self._gfx.read(), self._static)


# --------------------------------------------------------------------------- #
# Offline self-test (no AC / no Windows needed)
# --------------------------------------------------------------------------- #
def selftest() -> int:
    """Verify struct offsets against the published SharedFileOut.h layout and
    round-trip a synthetic buffer through build_snapshot()."""
    P, G, S = SPageFilePhysics, SPageFileGraphic, SPageFileStatic
    expected = {
        # physics
        (P, "packetId"): 0, (P, "gas"): 4, (P, "brake"): 8, (P, "fuel"): 12,
        (P, "gear"): 16, (P, "rpms"): 20, (P, "steerAngle"): 24, (P, "speedKmh"): 28,
        (P, "velocity"): 32, (P, "accG"): 44, (P, "wheelSlip"): 56,
        (P, "wheelLoad"): 72, (P, "tyreWear"): 120, (P, "drs"): 200,
        (P, "heading"): 208, (P, "carDamage"): 224, (P, "numberOfTyresOut"): 244,
        (P, "airTemp"): 288, (P, "clutch"): 364, (P, "isAIControlled"): 416,
        (P, "brakeBias"): 564, (P, "localVelocity"): 568,
        # graphics
        (G, "status"): 4, (G, "session"): 8, (G, "currentTime"): 12,
        (G, "completedLaps"): 132, (G, "iCurrentTime"): 140,
        (G, "numberOfLaps"): 172, (G, "tyreCompound"): 176,
        (G, "replayTimeMultiplier"): 244, (G, "normalizedCarPosition"): 248,
        (G, "carCoordinates"): 252, (G, "windDirection"): 292,
        # static
        (S, "numberOfSessions"): 60, (S, "carModel"): 68, (S, "track"): 134,
        (S, "sectorCount"): 400, (S, "maxRpm"): 412, (S, "maxTurboBoost"): 452,
    }
    bad = 0
    for (typ, field), want in expected.items():
        got = getattr(typ, field).offset
        if got != want:
            bad += 1
            print(f"  MISMATCH {typ.__name__}.{field}: offset {got}, expected {want}")
    print(f"sizeof: physics={ctypes.sizeof(P)}  graphics={ctypes.sizeof(G)}  "
          f"static={ctypes.sizeof(S)}")

    # round trip
    p, g, s = P(), G(), S()
    p.packetId, p.speedKmh, p.rpms, p.gear = 42, 251.5, 7250, 5
    p.wheelSlip[2] = 6.5
    g.status, g.session = AC_LIVE, 0
    car = "ks_porsche_911_gt3_rs".encode("utf-16-le")
    ctypes.memmove(s.carModel, car, len(car))
    s.maxRpm = 9000
    snap = build_snapshot(P.from_buffer_copy(bytes(p)), G.from_buffer_copy(bytes(g)),
                          S.from_buffer_copy(bytes(s)))
    checks = [
        (snap.speed_kmh == 251.5, "speed"), (snap.rpm == 7250, "rpm"),
        (snap.gear == 4 and snap.gear_label == "4", "gear"),
        (snap.max_wheel_slip == 6.5, "slip"), (snap.is_live, "status"),
        (snap.car_model == "ks_porsche_911_gt3_rs", "wstr"), (snap.max_rpm == 9000, "max_rpm"),
    ]
    for ok, name in checks:
        if not ok:
            bad += 1
            print(f"  ROUND-TRIP FAIL: {name}")

    print("SELFTEST", "FAILED" if bad else "OK")
    return 1 if bad else 0


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #
def _hud_line(s: Snapshot, rate: float) -> str:
    return (f"{s.status_name:<6} {s.session_name:<8} | "
            f"{s.speed_kmh:6.1f} km/h | RPM {s.rpm:5d}/{s.max_rpm:<5d} | Gear {s.gear_label:>2} | "
            f"Gas {s.gas:4.2f} Brk {s.brake:4.2f} Steer {s.steer_angle:+6.2f} | "
            f"Slip {s.max_wheel_slip:5.2f} | pkt {s.physics_packet} ({rate:3.0f}/s)")


def main() -> int:
    ap = argparse.ArgumentParser(description="Assetto Corsa shared-memory telemetry monitor")
    ap.add_argument("--hz", type=float, default=10.0, help="display refresh rate (default 10)")
    ap.add_argument("--once", action="store_true", help="print one JSON snapshot and exit")
    ap.add_argument("--selftest", action="store_true", help="offline struct-layout check")
    args = ap.parse_args()

    if args.selftest:
        return selftest()

    if sys.platform != "win32":
        print("This must run on Windows with Assetto Corsa. Use --selftest to check layouts here.")
        return 2

    tm = ACTelemetry()
    print("Waiting for Assetto Corsa shared memory (Ctrl+C to quit)...")
    try:
        if not tm.connect(timeout=None):
            return 1
        snap = tm.read()
        print(f"Connected: {snap.car_model or '?'} @ {snap.track or '?'}")
        if args.once:
            print(json.dumps(asdict(snap), indent=2))
            return 0

        period = 1.0 / max(args.hz, 0.5)
        last_pkt, last_t, rate = snap.physics_packet, time.perf_counter(), 0.0
        while True:
            snap = tm.read()
            now = time.perf_counter()
            if now - last_t >= 0.5:  # packets/sec, should be ~333 while LIVE
                rate = (snap.physics_packet - last_pkt) / (now - last_t)
                last_pkt, last_t = snap.physics_packet, now
            print("\r" + _hud_line(snap, rate).ljust(118), end="", flush=True)
            time.sleep(period)
    except KeyboardInterrupt:
        print("\nbye")
        return 0
    finally:
        tm.close()


if __name__ == "__main__":
    sys.exit(main())

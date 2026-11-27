"""
telemetry.py - Assetto Corsa shared-memory reader (ctypes only, no dependencies).

AC publishes three memory-mapped pages while running:
    Local\\acpmf_physics   ~333 Hz  car dynamics (speed, rpm, gear, slip, inputs)
    Local\\acpmf_graphics  ~60 Hz   session / lap / position
    Local\\acpmf_static    once     car + track names, max rpm

Layouts follow Kunos' SharedFileOut.h (`#pragma pack(4)`). Strings are UTF-16 and
declared as uint16 arrays so the layout is identical on any OS (testable off-Windows).
Structs stop at the last field present in every AC 1.x build; the whole section is
mapped, so newer, longer pages are fine.

    python telemetry.py              # live speed/RPM/gear monitor
    python telemetry.py --once       # one JSON snapshot
    python telemetry.py --selftest   # offline layout check
"""

from __future__ import annotations

import argparse
import ctypes
import json
import sys
import time
from dataclasses import dataclass

AC_OFF, AC_REPLAY, AC_LIVE, AC_PAUSE = 0, 1, 2, 3
STATUS_NAMES = {AC_OFF: "OFF", AC_REPLAY: "REPLAY", AC_LIVE: "LIVE", AC_PAUSE: "PAUSE"}
SESSION_NAMES = {-1: "UNKNOWN", 0: "PRACTICE", 1: "QUALIFY", 2: "RACE",
                 3: "HOTLAP", 4: "TIME_ATTACK", 5: "DRIFT", 6: "DRAG"}

_F, _I, _W = ctypes.c_float, ctypes.c_int32, ctypes.c_uint16  # float, int, UTF-16 unit
F2, F3, F4, F5 = _F * 2, _F * 3, _F * 4, _F * 5


class SPageFilePhysics(ctypes.Structure):
    _pack_ = 4
    _fields_ = [
        ("packetId", _I), ("gas", _F), ("brake", _F), ("fuel", _F),
        ("gear", _I),                    # 0 = R, 1 = N, 2 = 1st, ...
        ("rpms", _I), ("steerAngle", _F), ("speedKmh", _F),
        ("velocity", F3), ("accG", F3),
        ("wheelSlip", F4),               # FL FR RL RR
        ("wheelLoad", F4), ("wheelsPressure", F4), ("wheelAngularSpeed", F4),
        ("tyreWear", F4), ("tyreDirtyLevel", F4), ("tyreCoreTemperature", F4),
        ("camberRAD", F4), ("suspensionTravel", F4),
        ("drs", _F), ("tc", _F), ("heading", _F), ("pitch", _F), ("roll", _F),
        ("cgHeight", _F), ("carDamage", F5), ("numberOfTyresOut", _I),
        ("pitLimiterOn", _I), ("abs", _F), ("kersCharge", _F), ("kersInput", _F),
        ("autoShifterOn", _I), ("rideHeight", F2), ("turboBoost", _F), ("ballast", _F),
        ("airDensity", _F), ("airTemp", _F), ("roadTemp", _F), ("localAngularVel", F3),
        ("finalFF", _F), ("performanceMeter", _F), ("engineBrake", _I),
        ("ersRecoveryLevel", _I), ("ersPowerLevel", _I), ("ersHeatCharging", _I),
        ("ersIsCharging", _I), ("kersCurrentKJ", _F), ("drsAvailable", _I),
        ("drsEnabled", _I), ("brakeTemp", F4), ("clutch", _F),
        ("tyreTempI", F4), ("tyreTempM", F4), ("tyreTempO", F4),
        ("isAIControlled", _I), ("tyreContactPoint", F3 * 4),
        ("tyreContactNormal", F3 * 4), ("tyreContactHeading", F3 * 4),
        ("brakeBias", _F), ("localVelocity", F3),
    ]


class SPageFileGraphic(ctypes.Structure):
    _pack_ = 4
    _fields_ = [
        ("packetId", _I), ("status", _I), ("session", _I),
        ("currentTime", _W * 15), ("lastTime", _W * 15),
        ("bestTime", _W * 15), ("split", _W * 15),
        ("completedLaps", _I), ("position", _I),
        ("iCurrentTime", _I), ("iLastTime", _I), ("iBestTime", _I),   # ms
        ("sessionTimeLeft", _F), ("distanceTraveled", _F), ("isInPit", _I),
        ("currentSectorIndex", _I), ("lastSectorTime", _I), ("numberOfLaps", _I),
        ("tyreCompound", _W * 33), ("replayTimeMultiplier", _F),
        ("normalizedCarPosition", _F),   # 0..1 around the lap
        ("carCoordinates", F3),          # world X, Y, Z of the player car
        ("penaltyTime", _F), ("flag", _I), ("idealLineOn", _I), ("isInPitLane", _I),
        ("surfaceGrip", _F), ("mandatoryPitDone", _I),
        ("windSpeed", _F), ("windDirection", _F),
    ]


class SPageFileStatic(ctypes.Structure):
    _pack_ = 4
    _fields_ = [
        ("smVersion", _W * 15), ("acVersion", _W * 15),
        ("numberOfSessions", _I), ("numCars", _I),
        ("carModel", _W * 33), ("track", _W * 33), ("playerName", _W * 33),
        ("playerSurname", _W * 33), ("playerNick", _W * 33),
        ("sectorCount", _I), ("maxTorque", _F), ("maxPower", _F), ("maxRpm", _I),
        ("maxFuel", _F), ("suspensionMaxTravel", F4), ("tyreRadius", F4),
        ("maxTurboBoost", _F),
    ]


def _wstr(arr) -> str:
    """UTF-16 code-unit array -> str (stops at first NUL)."""
    return bytes(arr).decode("utf-16-le", errors="ignore").split("\0", 1)[0]


@dataclass
class Snapshot:
    """Flattened, immutable-by-convention view handed to the rest of the program."""
    t: float                         # time.perf_counter() at read
    physics_packet: int
    status: int                      # AC_OFF / REPLAY / LIVE / PAUSE
    session: int
    speed_kmh: float
    rpm: int
    gear_raw: int                    # AC encoding: 0=R, 1=N, 2=1st ...
    gas: float
    brake: float
    steer_angle: float
    heading: float                   # rad
    velocity: tuple[float, float, float]        # world frame, m/s (x, y up, z)
    local_velocity: tuple[float, float, float]
    acc_g: tuple[float, float, float]
    wheel_slip: tuple[float, float, float, float]
    damage: float                    # sum of carDamage[5]; jumps on contact (if damage is on)
    auto_shifter: bool               # AC automatic gearbox assist on (the bot can't shift)
    normalized_pos: float            # 0..1 around the lap
    car_xyz: tuple[float, float, float]
    completed_laps: int
    car_model: str
    track: str
    max_rpm: int
    age_s: float = 0.0               # seconds since physics packetId last changed (set by reader)

    @classmethod
    def from_pages(cls, p: SPageFilePhysics, g: SPageFileGraphic, s: SPageFileStatic,
                   t: float | None = None) -> "Snapshot":
        # keywords, not positions: most fields are floats, so a reorder would fail silently
        return cls(
            t=time.perf_counter() if t is None else t, physics_packet=p.packetId,
            status=g.status, session=g.session, speed_kmh=p.speedKmh, rpm=p.rpms,
            gear_raw=p.gear, gas=p.gas, brake=p.brake, steer_angle=p.steerAngle,
            heading=p.heading, velocity=tuple(p.velocity), local_velocity=tuple(p.localVelocity), acc_g=tuple(p.accG),
            wheel_slip=tuple(p.wheelSlip), damage=float(sum(p.carDamage)), auto_shifter=bool(p.autoShifterOn), normalized_pos=g.normalizedCarPosition,
            car_xyz=tuple(g.carCoordinates), completed_laps=g.completedLaps,
            car_model=_wstr(s.carModel), track=_wstr(s.track), max_rpm=s.maxRpm)

    STALE_S = 0.1  # physics runs ~333 Hz; no new packet for 100 ms = paused / frozen / AC gone

    @property
    def is_fresh(self) -> bool:
        """LIVE and physics actually advancing. Control code must not drive otherwise."""
        return self.is_live and self.age_s < self.STALE_S

    @property
    def gear(self) -> int:           # -1 = reverse, 0 = neutral, 1..n
        return self.gear_raw - 1

    @property
    def gear_label(self) -> str:
        return "R" if self.gear < 0 else "N" if self.gear == 0 else str(self.gear)

    @property
    def is_live(self) -> bool:
        return self.status == AC_LIVE

    @property
    def max_wheel_slip(self) -> float:
        return max(abs(x) for x in self.wheel_slip)

    @property
    def status_name(self) -> str:
        return STATUS_NAMES.get(self.status, f"?{self.status}")

    @property
    def session_name(self) -> str:
        return SESSION_NAMES.get(self.session, f"?{self.session}")


# --------------------------------------------------------------------------- #
# Windows shared-memory access
# --------------------------------------------------------------------------- #
class TelemetryUnavailable(RuntimeError):
    """AC is not running / pages not present / not on Windows."""


class _Page:
    """Read-only view of one named AC file mapping.

    Uses OpenFileMappingW (fails if absent) rather than mmap.mmap(tagname=...), which
    silently *creates* an empty zero-filled mapping when AC isn't running, making
    every reading look like 0.
    """
    FILE_MAP_READ = 0x0004

    def __init__(self, name: str, struct_type: type[ctypes.Structure]):
        if sys.platform != "win32":
            raise TelemetryUnavailable("AC shared memory is only available on Windows")
        from ctypes import wintypes as w
        k = self._k32 = ctypes.WinDLL("kernel32", use_last_error=True)
        k.OpenFileMappingW.argtypes, k.OpenFileMappingW.restype = [w.DWORD, w.BOOL, w.LPCWSTR], w.HANDLE
        k.MapViewOfFile.argtypes = [w.HANDLE, w.DWORD, w.DWORD, w.DWORD, ctypes.c_size_t]
        k.MapViewOfFile.restype = ctypes.c_void_p
        k.UnmapViewOfFile.argtypes = [ctypes.c_void_p]
        k.CloseHandle.argtypes = [w.HANDLE]

        self.type, self._size = struct_type, ctypes.sizeof(struct_type)
        self._handle = k.OpenFileMappingW(self.FILE_MAP_READ, False, name)
        if not self._handle:
            raise TelemetryUnavailable(
                f"{name} not found (error {ctypes.get_last_error()}) - is Assetto Corsa running?")
        # length 0 -> map the whole section, so a longer real page is fine
        self._view = k.MapViewOfFile(self._handle, self.FILE_MAP_READ, 0, 0, 0)
        if not self._view:
            err = ctypes.get_last_error()
            self.close()
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
    PAGES = (("physics", SPageFilePhysics), ("graphics", SPageFileGraphic),
             ("static", SPageFileStatic))

    def __init__(self) -> None:
        self._pages: list[_Page] = []
        self._static: SPageFileStatic | None = None
        self._last_pkt: int | None = None
        self._pkt_t = 0.0

    @property
    def connected(self) -> bool:
        return bool(self._pages)

    def try_connect(self) -> bool:
        if self.connected:
            return True
        opened: list[_Page] = []
        try:
            for name, typ in self.PAGES:
                opened.append(_Page(f"Local\\acpmf_{name}", typ))
        except TelemetryUnavailable:
            for p in opened:
                p.close()
            return False
        self._pages, self._static = opened, opened[2].read()
        return True

    def connect(self, timeout: float | None = None, poll_s: float = 1.0) -> bool:
        """Wait up to `timeout` s (forever if None) for AC to publish memory."""
        deadline = None if timeout is None else time.monotonic() + timeout
        while not self.try_connect():
            if deadline is not None and time.monotonic() >= deadline:
                return False
            time.sleep(poll_s)
        return True

    def read(self) -> Snapshot:
        if not self.connected:
            raise TelemetryUnavailable("not connected")
        phys, gfx, stat = self._pages
        if self._static.maxRpm == 0:   # connected mid-load: static page not filled yet
            self._static = stat.read()
        snap = Snapshot.from_pages(phys.read(), gfx.read(), self._static)
        # We hold the mapping open, so it outlives acs.exe: after AC quits (or while
        # paused) the page just freezes. Track packet progress so callers can tell.
        if snap.physics_packet != self._last_pkt:
            self._last_pkt, self._pkt_t = snap.physics_packet, snap.t
        snap.age_s = snap.t - self._pkt_t
        return snap

    def close(self) -> None:
        for p in self._pages:
            p.close()
        self._pages, self._static, self._last_pkt = [], None, None

    def __enter__(self) -> "ACTelemetry":
        return self

    def __exit__(self, *exc) -> None:
        self.close()


# --------------------------------------------------------------------------- #
# Offline self-test (no AC / no Windows needed)
# --------------------------------------------------------------------------- #
def selftest() -> int:
    """Check struct offsets against SharedFileOut.h, then round-trip a synthetic buffer."""
    P, G, S = SPageFilePhysics, SPageFileGraphic, SPageFileStatic
    expected = {
        (P, "gas"): 4, (P, "gear"): 16, (P, "rpms"): 20, (P, "speedKmh"): 28,
        (P, "accG"): 44, (P, "wheelSlip"): 56, (P, "heading"): 208,
        (P, "numberOfTyresOut"): 244, (P, "clutch"): 364, (P, "isAIControlled"): 416,
        (P, "localVelocity"): 568,
        (G, "status"): 4, (G, "currentTime"): 12, (G, "completedLaps"): 132,
        (G, "tyreCompound"): 176, (G, "normalizedCarPosition"): 248,
        (G, "carCoordinates"): 252, (G, "windDirection"): 292,
        (S, "carModel"): 68, (S, "track"): 134, (S, "sectorCount"): 400, (S, "maxRpm"): 412,
    }
    fails = [f"{t.__name__}.{f}: offset {getattr(t, f).offset}, expected {w}"
             for (t, f), w in expected.items() if getattr(t, f).offset != w]

    p, g, s = P(), G(), S()
    p.speedKmh, p.rpms, p.gear, p.wheelSlip[2] = 251.5, 7250, 5, -6.5
    g.status, s.maxRpm = AC_LIVE, 9000
    name = "ks_porsche_911_gt3_rs".encode("utf-16-le")
    ctypes.memmove(s.carModel, name, len(name))
    n = Snapshot.from_pages(P.from_buffer_copy(bytes(p)), G.from_buffer_copy(bytes(g)),
                            S.from_buffer_copy(bytes(s)))
    for ok, what in [(n.speed_kmh == 251.5, "speed"), (n.rpm == 7250, "rpm"),
                     (n.gear == 4 and n.gear_label == "4", "gear"),
                     (n.max_wheel_slip == 6.5, "slip"), (n.is_live, "status"),
                     (n.car_model == "ks_porsche_911_gt3_rs", "wstr"), (n.max_rpm == 9000, "maxrpm")]:
        if not ok:
            fails.append(f"round-trip: {what}")

    # every Snapshot field must come from the right source field: give each a unique value
    src = {"physics_packet": (p, "packetId"), "status": (g, "status"),
           "session": (g, "session"), "speed_kmh": (p, "speedKmh"), "rpm": (p, "rpms"),
           "gear_raw": (p, "gear"), "gas": (p, "gas"), "brake": (p, "brake"),
           "steer_angle": (p, "steerAngle"), "heading": (p, "heading"),
           "normalized_pos": (g, "normalizedCarPosition"),
           "completed_laps": (g, "completedLaps"), "max_rpm": (s, "maxRpm")}
    for i, (obj, attr) in enumerate(src.values(), start=11):
        setattr(obj, attr, i)
    for i, arr in enumerate((p.localVelocity, p.accG, p.wheelSlip, g.carCoordinates, p.velocity)):
        for j in range(len(arr)):
            arr[j] = 100 + 10 * i + j
    p.carDamage[1], p.carDamage[4] = 0.25, 0.5
    p.autoShifterOn = 1
    n = Snapshot.from_pages(p, g, s, t=0.0)
    if n.damage != 0.75:
        fails.append("mapping: damage")
    if n.auto_shifter is not True:
        fails.append("mapping: auto_shifter")
    fails += [f"mapping: {k}" for i, k in enumerate(src, start=11) if getattr(n, k) != i]
    for i, k in enumerate(("local_velocity", "acc_g", "wheel_slip", "car_xyz", "velocity")):
        if getattr(n, k) != tuple(float(100 + 10 * i + j) for j in range(len(getattr(n, k)))):
            fails.append(f"mapping: {k}")

    print(f"sizeof: physics={ctypes.sizeof(P)} graphics={ctypes.sizeof(G)} static={ctypes.sizeof(S)}")
    print("SELFTEST", "FAILED:\n  " + "\n  ".join(fails) if fails else "OK")
    return 1 if fails else 0


def _hud_line(s: Snapshot, rate: float) -> str:
    state = s.status_name if s.age_s < s.STALE_S else "FROZEN"
    return (f"{state:<6} {s.session_name:<8} | {s.speed_kmh:6.1f} km/h | "
            f"RPM {s.rpm:5d}/{s.max_rpm:<5d} | Gear {s.gear_label:>2} | "
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

    print("Waiting for Assetto Corsa shared memory (Ctrl+C to quit)...")
    try:
        with ACTelemetry() as tm:
            tm.connect()
            snap = tm.read()
            print(f"Connected: {snap.car_model or '?'} @ {snap.track or '?'}")
            if args.once:
                print(json.dumps({**snap.__dict__, "gear": snap.gear_label}, indent=2))
                return 0
            last_pkt, last_t, rate = snap.physics_packet, time.perf_counter(), 0.0
            while True:
                snap, now = tm.read(), time.perf_counter()
                if now - last_t >= 0.5:  # packets/sec: ~333 while LIVE
                    rate = (snap.physics_packet - last_pkt) / (now - last_t)
                    last_pkt, last_t = snap.physics_packet, now
                print("\r" + _hud_line(snap, rate).ljust(118), end="", flush=True)
                time.sleep(1.0 / max(args.hz, 0.5))
    except KeyboardInterrupt:
        print("\nbye")
        return 0


if __name__ == "__main__":
    sys.exit(main())

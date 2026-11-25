"""Assetto Corsa shared-memory reader (Windows). Layouts follow AC's SPageFilePhysics / SPageFileGraphic.
Only the leading fields we need are declared; ctypes structs are packed to 4 bytes like the game's.
wchar fields are declared as uint16 arrays so the layout is identical on any OS (for tests)."""
import ctypes as C
import mmap
import sys

MAX_CARS = 60


class Physics(C.Structure):
    _pack_ = 4
    _fields_ = [
        ("packetId", C.c_int), ("gas", C.c_float), ("brake", C.c_float), ("fuel", C.c_float),
        ("gear", C.c_int), ("rpms", C.c_int), ("steerAngle", C.c_float), ("speedKmh", C.c_float),
        ("velocity", C.c_float * 3), ("accG", C.c_float * 3),
    ]


class Graphics(C.Structure):
    _pack_ = 4
    _fields_ = [
        ("packetId", C.c_int), ("status", C.c_int), ("session", C.c_int),
        ("currentTime", C.c_uint16 * 15), ("lastTime", C.c_uint16 * 15),
        ("bestTime", C.c_uint16 * 15), ("split", C.c_uint16 * 15),
        ("completedLaps", C.c_int), ("position", C.c_int),
        ("iCurrentTime", C.c_int), ("iLastTime", C.c_int), ("iBestTime", C.c_int),
        ("sessionTimeLeft", C.c_float), ("distanceTraveled", C.c_float),
        ("isInPit", C.c_int), ("currentSectorIndex", C.c_int), ("lastSectorTime", C.c_int),
        ("numberOfLaps", C.c_int), ("tyreCompound", C.c_uint16 * 33),
        ("replayTimeMultiplier", C.c_float), ("normalizedCarPosition", C.c_float),
        ("activeCars", C.c_int), ("carCoordinates", (C.c_float * 3) * MAX_CARS),
        ("carID", C.c_int * MAX_CARS), ("playerCarID", C.c_int),
    ]


STATUS_LIVE = 2


class ACMemory:
    def __init__(self):
        if sys.platform != "win32":
            raise RuntimeError("Assetto Corsa shared memory is only available on Windows")
        self._p = mmap.mmap(0, C.sizeof(Physics), tagname="Local\\acpmf_physics")
        self._g = mmap.mmap(0, C.sizeof(Graphics), tagname="Local\\acpmf_graphics")

    def read(self):
        return parse(self._p[:], self._g[:])


def parse(physics_bytes, graphics_bytes):
    """-> dict(live, speed_kmh, own_xz, own_vxz, traffic=[(id,x,z)...])"""
    p = Physics.from_buffer_copy(physics_bytes[:C.sizeof(Physics)])
    g = Graphics.from_buffer_copy(graphics_bytes[:C.sizeof(Graphics)])
    n = max(0, min(g.activeCars, MAX_CARS))
    me = None
    for i in range(n):
        if g.carID[i] == g.playerCarID:
            me = i
            break
    if me is None:
        me = 0
    traffic = [(g.carID[i], g.carCoordinates[i][0], g.carCoordinates[i][2])
               for i in range(n) if i != me]
    return {
        "live": g.status == STATUS_LIVE and n > 0,
        "speed_kmh": p.speedKmh,
        "own_xz": (g.carCoordinates[me][0], g.carCoordinates[me][2]) if n else (0.0, 0.0),
        "own_vxz": (p.velocity[0], p.velocity[2]),
        "traffic": traffic,
    }

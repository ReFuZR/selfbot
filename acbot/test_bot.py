import ctypes as C
import unittest

import sim
import telemetry as T
from planner import Config, Planner


class Tests(unittest.TestCase):
    def test_no_crashes_across_steering_responses(self):
        for scale in (0.4, 1.0, 2.5):
            for seed in range(4):
                self.assertEqual(sim.run(seed=seed, steer_scale=scale, seconds=60)["crashes"], 0, (scale, seed))

    def test_dense_traffic(self):
        for seed in range(4):
            self.assertEqual(sim.run(seed=seed, density=1 / 20, seconds=60)["crashes"], 0, seed)

    def test_brakes_for_stopped_car(self):
        p = Planner(Config())
        p.step(0.0, (0, 0), (0, 40), [])
        c = p.step(0.02, (0, 0.8), (0, 40), [(1, -5.25, 40), (2, -1.75, 40), (3, 1.75, 40), (4, 5.25, 40)])
        self.assertGreater(c.brake, 0.0)

    def test_telemetry_parse(self):
        g = T.Graphics()
        g.status, g.activeCars, g.playerCarID = 2, 3, 7
        for i, (cid, x, z) in enumerate([(5, 1.0, 2.0), (7, 10.0, 20.0), (9, -3.0, 4.0)]):
            g.carID[i] = cid
            g.carCoordinates[i][0], g.carCoordinates[i][2] = x, z
        ph = T.Physics()
        ph.speedKmh = 100.0
        ph.velocity[0], ph.velocity[2] = 1.0, 27.0
        s = T.parse(bytes(ph), bytes(g))
        self.assertTrue(s["live"])
        self.assertEqual(s["own_xz"], (10.0, 20.0))
        self.assertEqual([t[0] for t in s["traffic"]], [5, 9])
        self.assertEqual(s["own_vxz"], (1.0, 27.0))

    def test_struct_offsets_match_ac(self):
        # Offsets taken from AC's documented SPageFileGraphic (wchar = 2 bytes on Windows).
        self.assertEqual(T.Graphics.activeCars.offset, 252)
        self.assertEqual(T.Graphics.carCoordinates.offset, 256)
        self.assertEqual(T.Graphics.playerCarID.offset, 256 + 720 + 240)
        self.assertEqual(T.Physics.velocity.offset, 32)


if __name__ == "__main__":
    unittest.main()

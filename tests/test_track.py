"""Measurement must follow a bend and handle finite track ends."""
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                       'simulation/ros_ws/src/robotrio_sim/src'))
from robotrio_sim.track import nearest_track


class TrackMeasurementTests(unittest.TestCase):
    def test_lateral_world_displacement_on_a_bend_is_not_error(self):
        track = [((0, 0), (1, 0)), ((1, 0), (2, 1))]
        distance, progress, heading = nearest_track(1.5, 0.5, track)
        self.assertAlmostEqual(distance, 0)
        self.assertAlmostEqual(progress, 1 + math.sqrt(0.5))
        self.assertAlmostEqual(heading, 45)

    def test_distance_is_perpendicular_and_clamped_at_ends(self):
        track = [((1, 2), (2, 3))]
        self.assertAlmostEqual(nearest_track(1, 3, track)[0], math.sqrt(0.5))
        self.assertEqual(nearest_track(0, 2, track)[:2], (1, 0))
        self.assertAlmostEqual(nearest_track(3, 3, track)[0], 1)

    def test_missing_or_degenerate_track_is_rejected(self):
        for track in ([], [((0, 0), (0, 0))]):
            with self.assertRaises(ValueError):
                nearest_track(0, 0, track)


if __name__ == '__main__':
    unittest.main()

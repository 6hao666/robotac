#!/usr/bin/env python3

import math
import unittest

from robotac_examples.geometry import distance3
from robotac_examples.geometry import relative_point


class GeometryTest(unittest.TestCase):
    def test_distance(self):
        self.assertAlmostEqual(distance3([0, 0, 0], [3, 4, 0]), 5.0)

    def test_relative_point_uses_start_yaw(self):
        point = relative_point([2.0, 3.0, 0.0], math.pi / 2.0,
                               [1.0, 0.0, 0.5])
        self.assertAlmostEqual(point[0], 2.0)
        self.assertAlmostEqual(point[1], 4.0)
        self.assertAlmostEqual(point[2], 0.5)


if __name__ == "__main__":
    unittest.main()

#!/usr/bin/env python3

import unittest

from robotac_localization.validation import distance3
from robotac_localization.validation import values_are_finite


class ValidationTest(unittest.TestCase):
    def test_distance(self):
        self.assertAlmostEqual(distance3([0, 0, 0], [3, 4, 0]), 5.0)

    def test_nonfinite_value(self):
        self.assertFalse(values_are_finite([0.0, float("nan")]))
        self.assertTrue(values_are_finite([0.0, 1.0]))


if __name__ == "__main__":
    unittest.main()

#!/usr/bin/env python3

import os
import tempfile
import unittest

from robotac_examples.route import load_route


class RouteTest(unittest.TestCase):
    def _write(self, text):
        handle, path = tempfile.mkstemp(suffix=".yaml")
        with os.fdopen(handle, "w", encoding="utf-8") as stream:
            stream.write(text)
        self.addCleanup(os.unlink, path)
        return path

    def test_valid_route(self):
        path = self._write("waypoints:\n  - {x: 0, y: 0, z: 0.5, hold: 2}\n")
        route = load_route(path)
        self.assertEqual(route[0][0], [0.0, 0.0, 0.5])

    def test_unknown_field_is_rejected(self):
        path = self._write(
            "waypoints:\n  - {x: 0, y: 0, z: 0.5, hold: 2, arm: true}\n")
        with self.assertRaises(ValueError):
            load_route(path)


if __name__ == "__main__":
    unittest.main()

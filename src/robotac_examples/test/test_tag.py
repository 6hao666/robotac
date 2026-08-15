#!/usr/bin/env python3

import unittest

from robotac_examples.tag_filter import find_detection, StableTag


class Detection(object):
    def __init__(self, ids):
        self.id = ids


class StableTagTest(unittest.TestCase):
    def test_requires_multiple_samples(self):
        tracker = StableTag(sample_count=3, jump_limit=0.2)
        self.assertIsNone(tracker.add([1.0, 2.0, 0.0]))
        self.assertIsNone(tracker.add([1.0, 2.0, 0.0]))
        self.assertEqual(tracker.add([1.0, 2.0, 0.0]), [1.0, 2.0, 0.0])

    def test_position_jump_resets_window(self):
        tracker = StableTag(sample_count=2, jump_limit=0.1)
        tracker.add([0.0, 0.0, 0.0])
        self.assertIsNone(tracker.add([1.0, 0.0, 0.0]))

    def test_reset_requires_new_stable_samples(self):
        tracker = StableTag(sample_count=2, jump_limit=0.1)
        tracker.add([0.0, 0.0, 0.0])
        tracker.add([0.0, 0.0, 0.0])
        tracker.reset()
        self.assertIsNone(tracker.add([0.0, 0.0, 0.0]))

    def test_detection_is_selected_by_id(self):
        first = Detection([1])
        second = Detection([0])
        self.assertIs(find_detection([first, second], 0), second)
        self.assertIsNone(find_detection([first], 0))


if __name__ == "__main__":
    unittest.main()

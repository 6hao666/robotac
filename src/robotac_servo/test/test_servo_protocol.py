#!/usr/bin/env python3

import unittest

from robotac_servo.protocol import ServoCalibration
from robotac_servo.protocol import duty_for_angle
from robotac_servo.protocol import make_frame
from robotac_servo.protocol import pulse_width_us


class ServoProtocolTest(unittest.TestCase):
    def test_frame_uses_controller_wire_format(self):
        self.assertEqual(make_frame(1, 50, 5), bytes((0x5A, 1, 0, 50, 5)))

    def test_angle_conversion_is_integer(self):
        self.assertEqual(duty_for_angle(0), 3)
        self.assertEqual(duty_for_angle(180), 12)

    def test_pulse_width(self):
        self.assertEqual(pulse_width_us(50, 5), 1000)

    def test_default_positions_match_aircraft_calibration(self):
        calibration = ServoCalibration()
        calibration.validate()
        self.assertEqual(calibration.blocked_angle, 0.0)
        self.assertEqual(calibration.released_angle, 45.0)
        self.assertEqual(calibration.blocked_duty, 3)
        self.assertEqual(calibration.released_duty, 5)

    def test_soft_limit_rejects_angle(self):
        calibration = ServoCalibration()
        with self.assertRaises(ValueError):
            calibration.duty_for_angle(-1.0)
        with self.assertRaises(ValueError):
            calibration.duty_for_angle(71.0)


if __name__ == "__main__":
    unittest.main()

#!/usr/bin/env python3
"""示例 07：起飞后完成一次相对位移、返回并降落。"""

import rospy

from robotac_examples.flight import FlightController
from robotac_examples.geometry import relative_point


def main():
    rospy.init_node("move_relative_example")
    height = float(rospy.get_param("~height", 0.6))
    forward = float(rospy.get_param("~forward", 0.5))
    controller = FlightController()
    if not controller.wait_for_start() or not controller.begin(height):
        return
    move = relative_point(controller.origin, controller.origin_yaw,
                          [forward, 0.0, height])
    home = relative_point(controller.origin, controller.origin_yaw,
                          [0.0, 0.0, height])
    controller._publish_state("MOVE", True)
    if controller.wait_target(move, 20.0, 2.0):
        controller._publish_state("RETURN", True)
        if controller.wait_target(home, 20.0, 2.0):
            controller.finish()


if __name__ == "__main__":
    main()

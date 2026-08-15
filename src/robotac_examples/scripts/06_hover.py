#!/usr/bin/env python3
"""示例 06：起飞、定高悬停并降落。"""

import rospy

from robotac_examples.flight import FlightController


def main():
    rospy.init_node("hover_example")
    height = float(rospy.get_param("~height", 0.6))
    hold = float(rospy.get_param("~hold", 3.0))
    controller = FlightController()
    if not controller.wait_for_start():
        return
    if not controller.begin(height):
        return
    controller._publish_state("HOVER", True)
    if controller.wait_target(controller.target, hold + 10.0, hold):
        controller.finish()


if __name__ == "__main__":
    main()

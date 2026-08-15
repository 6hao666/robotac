#!/usr/bin/env python3
"""示例 08：依次执行 YAML 中的简短相对航点。"""

import rospy

from robotac_examples.flight import FlightController
from robotac_examples.geometry import relative_point
from robotac_examples.route import load_route


def main():
    rospy.init_node("waypoints_example")
    route_file = rospy.get_param("~route_file")
    route = load_route(route_file)
    controller = FlightController()
    if not controller.wait_for_start():
        return
    first_height = route[0][0][2]
    if not controller.begin(first_height):
        return
    for index, item in enumerate(route):
        point, hold = item
        target = relative_point(controller.origin, controller.origin_yaw, point)
        controller._publish_state("WAYPOINT_%d" % (index + 1), True)
        if not controller.wait_target(target, 30.0, hold):
            return
    controller.finish()


if __name__ == "__main__":
    main()

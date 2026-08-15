#!/usr/bin/env python3
"""示例 10：起飞、对准 ID 0、稳定悬停并降落。"""

import math
import time

import rospy

from robotac_examples.flight import FlightController
from robotac_examples.tag import TagTracker


def main():
    rospy.init_node("tag_centering_flight_example")
    height = float(rospy.get_param("~height", 0.6))
    hold_seconds = float(rospy.get_param("~hold", 3.0))
    max_step = float(rospy.get_param("~max_step", 0.20))
    timeout = float(rospy.get_param("~timeout", 30.0))
    tracker = TagTracker(int(rospy.get_param("~tag_id", 0)), "map")
    controller = FlightController()
    if not controller.wait_for_start():
        return
    if not tracker.fresh():
        controller.fail("启动时没有稳定的 AprilTag")
        return
    if not controller.begin(height):
        return
    controller._publish_state("TAG_CENTERING", True)
    start = time.monotonic()
    reached_since = None
    rate = rospy.Rate(controller.rate_hz)
    while not rospy.is_shutdown():
        issue = controller.health_issue()
        if issue is not None:
            controller.fail(issue)
            return
        if not tracker.fresh():
            controller.fail("AprilTag 数据中断")
            return
        current = controller.pose.pose.position
        tag = tracker.pose.pose.position
        dx = tag.x - current.x
        dy = tag.y - current.y
        length = math.hypot(dx, dy)
        scale = 1.0
        if length > max_step:
            scale = max_step / length
        target = [current.x + dx * scale, current.y + dy * scale,
                  controller.origin[2] + height, controller.origin_yaw]
        try:
            controller.publish_target(target)
        except ValueError as error:
            controller.fail(str(error))
            return
        if length <= controller.position_tolerance:
            if reached_since is None:
                reached_since = time.monotonic()
            if time.monotonic() - reached_since >= hold_seconds:
                break
        else:
            reached_since = None
        if time.monotonic() - start > timeout:
            controller.fail("AprilTag 对准超时")
            return
        rate.sleep()
    controller.finish()


if __name__ == "__main__":
    main()

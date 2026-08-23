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
    # 2026-08-23: 支持"先起飞后找 tag"——飞机起飞前 tag 可能不在视野
    # （例如飞机在地面、tag 也在低处，只有升空后相机才能看到）。
    # acquire_timeout: 起飞后悬停等待 tag 出现的最长时间；超时中止。
    # require_tag_before: 设为 true 恢复旧行为（起飞前必须已有稳定 tag）。
    acquire_timeout = float(rospy.get_param("~acquire_timeout", 20.0))
    require_tag_before = bool(rospy.get_param("~require_tag_before", False))
    # 2026-08-23 事故后新增：tag 合理性。tag 距飞机超过 tag_max_range 判为
    # 幻影/异常（常见于定位漂移），悬停等待而不是追出去（防止追进墙里）。
    tag_max_range = float(rospy.get_param("~tag_max_range", 1.5))
    tracker = TagTracker(int(rospy.get_param("~tag_id", 0)), "map")
    controller = FlightController()
    if not controller.wait_for_start():
        return
    if require_tag_before and not tracker.fresh():
        controller.fail("启动时没有稳定的 AprilTag")
        return
    if not controller.begin(height):
        return
    controller._publish_state("TAG_CENTERING", True)
    start = time.monotonic()
    reached_since = None
    acquired = False
    previous = None
    previous_time = None
    rate = rospy.Rate(controller.rate_hz)
    while not rospy.is_shutdown():
        issue = controller.health_issue()
        if issue is not None:
            controller.fail(issue)
            return
        # 保险：位置估计速度异常（定位跳变/漂移带飞）。
        now = time.monotonic()
        if previous is not None and previous_time is not None:
            if controller.safety.too_fast(previous, now - previous_time):
                controller.fail("位置估计异常快速移动")
                return
        if controller.pose is not None:
            previous = [controller.pose.pose.position.x,
                        controller.pose.pose.position.y]
        previous_time = now
        if not tracker.fresh():
            if not acquired:
                # 起飞后 tag 尚未出现：在起飞点悬停等待获取。
                if time.monotonic() - start > acquire_timeout:
                    controller.fail("AprilTag 获取超时")
                    return
                hover = [controller.origin[0], controller.origin[1],
                         controller.origin[2] + height, controller.origin_yaw]
                try:
                    controller.publish_target(hover)
                except ValueError as error:
                    controller.fail(str(error))
                    return
                rate.sleep()
                continue
            controller.fail("AprilTag 数据中断")
            return
        acquired = True
        current = controller.pose.pose.position
        tag = tracker.pose.pose.position
        dx = tag.x - current.x
        dy = tag.y - current.y
        length = math.hypot(dx, dy)
        # 保险：tag 位置不合理（离飞机太远，常见于定位漂移/幻影）→ 悬停不追。
        if length > tag_max_range:
            hover = [controller.origin[0], controller.origin[1],
                     controller.origin[2] + height, controller.origin_yaw]
            try:
                controller.publish_target(hover)
            except ValueError as error:
                controller.fail(str(error))
                return
            rate.sleep()
            continue
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

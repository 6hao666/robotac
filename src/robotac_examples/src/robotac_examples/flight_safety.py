"""示例飞行共用的事故后安全保险（2026-08-23 从 FlightController 拆出）。

原 FlightController 加入这些保护后超过 check_source 250 行红线，故独立成模块。
通过组合持有 controller 引用，读取其 pose / origin 等飞行状态：

- check_ground_stable：起飞前位置估计稳定性检查（漂移则拒绝起飞）。
- too_far：飞行中距起飞点水平距离保护（防撞墙/追幻影目标）。
- too_fast：位置估计速度保护（防漂移定位带飞）。
"""

import math
import time

import rospy


class FlightSafety(object):
    def __init__(self, controller):
        self.controller = controller
        # 2026-08-23 事故后新增保险：
        # max_distance：飞行中距起飞点水平距离超过该值立即中止。
        # max_velocity：位置估计每帧变化超过该值立即中止。
        # stability_*：起飞前位置估计稳定性检查（漂移则拒绝起飞）。
        self.max_distance = float(rospy.get_param("~max_distance", 1.5))
        self.max_velocity = float(rospy.get_param("~max_velocity", 3.0))
        self.stability_seconds = float(
            rospy.get_param("~stability_seconds", 2.0))
        self.stability_tolerance = float(
            rospy.get_param("~stability_tolerance", 0.15))

    def check_ground_stable(self, seconds=None, tolerance=None):
        """起飞前保险：飞机静止时，位置估计不应漂移。

        采样 stability_seconds 秒，水平位移超过 stability_tolerance 即判定位不稳定。
        返回 True 表示稳定可起飞，False 表示定位漂移应拒绝起飞。
        """
        if seconds is None:
            seconds = self.stability_seconds
        if tolerance is None:
            tolerance = self.stability_tolerance
        first = None
        last = None
        rate = rospy.Rate(10)
        start = time.monotonic()
        while time.monotonic() - start < seconds:
            if self.controller.pose is not None:
                point = [self.controller.pose.pose.position.x,
                         self.controller.pose.pose.position.y]
                if first is None:
                    first = point
                last = point
            rate.sleep()
        if first is None or last is None:
            return False
        drift = math.hypot(last[0] - first[0], last[1] - first[1])
        rospy.loginfo("起飞前位置漂移: %.3f m (阈值 %.3f)", drift, tolerance)
        return drift <= tolerance

    def too_far(self):
        """飞行中距起飞点水平距离保护。"""
        controller = self.controller
        if controller.pose is None or controller.origin is None:
            return False
        dx = controller.pose.pose.position.x - controller.origin[0]
        dy = controller.pose.pose.position.y - controller.origin[1]
        return math.hypot(dx, dy) > self.max_distance

    def too_fast(self, previous, dt):
        """位置估计速度保护：每帧位移超过 max_velocity*dt 判为异常。"""
        if previous is None or self.controller.pose is None:
            return False
        dx = self.controller.pose.pose.position.x - previous[0]
        dy = self.controller.pose.pose.position.y - previous[1]
        if dt <= 0.0:
            return False
        speed = math.hypot(dx, dy) / dt
        return speed > self.max_velocity

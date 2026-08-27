#!/usr/bin/env python3
"""示例 12：起飞后按机头方向做 前/后/左/右 水平移动排查（不降落中途，仅水平位移）。

用途：排查当前水平移动/定位问题。每次 start 后：
  1. 起飞到 height 高度（默认 0.3m）
  2. 依次飞到 前方、后方、左方、右方 各 step 距离（相对起点，按机头朝向）
  3. 每个方向到达后打印实际位置与期望的偏差，供排查
  4. 全部走完返回起点并 AUTO.LAND

参数:
  ~height: 飞行高度 m（默认 0.3）
  ~step:   每方向水平位移 m（默认 0.5）
  ~timeout: 每方向到达超时 s（默认 20.0）
  ~hold:    每方向稳定保持 s（默认 2.0）
"""

import rospy

from robotac_examples.flight import FlightController
from robotac_examples.geometry import distance3, relative_point


def _report(controller, label, target):
    """打印期望位置 vs 实际位置的偏差，供排查水平移动问题。"""
    pose = controller.pose
    if pose is None:
        rospy.logwarn("[%s] 无本地位姿，无法核对", label)
        return
    pos = pose.pose.position
    err = distance3([pos.x, pos.y, pos.z], target[:3])
    rospy.loginfo("[%s] 期望 (%.3f,%.3f,%.3f) 实际 (%.3f,%.3f,%.3f) 偏差 %.3f m",
                  label, target[0], target[1], target[2],
                  pos.x, pos.y, pos.z, err)


def main():
    rospy.init_node("move_cardinal_example")
    height = float(rospy.get_param("~height", 0.3))
    step = float(rospy.get_param("~step", 0.5))
    timeout = float(rospy.get_param("~timeout", 20.0))
    hold = float(rospy.get_param("~hold", 2.0))

    controller = FlightController()
    if not controller.wait_for_start() or not controller.begin(height):
        rospy.logerr("起飞失败，中止")
        return

    origin = controller.origin
    yaw = controller.origin_yaw
    # 前后左右相对起点（保持同一高度，仅水平位移）
    points = {
        "前": relative_point(origin, yaw, [step, 0.0, 0.0]),
        "后": relative_point(origin, yaw, [-step, 0.0, 0.0]),
        "左": relative_point(origin, yaw, [0.0, step, 0.0]),
        "右": relative_point(origin, yaw, [0.0, -step, 0.0]),
    }

    for label, target in points.items():
        controller._publish_state("MOVE_" + label, True)
        rospy.loginfo(">>> 向%s 移动 %.2f m", label, step)
        if not controller.wait_target(target, timeout, hold):
            rospy.logerr("[%s] 移动失败/超时", label)
            break
        _report(controller, label, target)

    # 回到起点并降落
    home = relative_point(origin, yaw, [0.0, 0.0, 0.0])
    rospy.loginfo(">>> 返回起点并降落")
    controller._publish_state("RETURN", True)
    if controller.wait_target(home, timeout, hold):
        controller.finish()
    else:
        rospy.logerr("返回起点失败/超时")


if __name__ == "__main__":
    main()

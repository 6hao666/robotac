#!/usr/bin/env python3
"""地面标定工具：显式发送一个角度，然后立即释放 PWM。"""

import rospy

from robotac_servo.protocol import duty_for_angle, make_frame
from robotac_servo.transport import SerialPort, find_port


def main():
    rospy.init_node("servo_angle_tuner")
    if not bool(rospy.get_param("~enabled", False)):
        rospy.logfatal("拒绝动作：必须显式设置 enabled:=true")
        return 2
    port_setting = str(rospy.get_param("~port", "/dev/robotac_servo"))
    candidates = rospy.get_param("~port_candidates", ["/dev/robotac_servo"])
    angle = float(rospy.get_param("~angle", 15.0))
    minimum = float(rospy.get_param("~min_angle", 10.0))
    maximum = float(rospy.get_param("~max_angle", 70.0))
    if angle < minimum or angle > maximum:
        rospy.logfatal("角度 %.1f 超出软件限位 %.1f 至 %.1f", angle,
                       minimum, maximum)
        return 2
    path = find_port(port_setting, candidates)
    if path is None:
        rospy.logfatal("未找到舵机串口")
        return 2
    duty = duty_for_angle(angle,
                          int(rospy.get_param("~pwm_min_duty", 3)),
                          int(rospy.get_param("~pwm_max_duty", 12)))
    channel = int(rospy.get_param("~channel", 1))
    frequency = int(rospy.get_param("~frequency_hz", 50))
    drive_seconds = float(rospy.get_param("~drive_seconds", 0.30))
    idle_duty = int(rospy.get_param("~idle_duty", 0))
    port = None
    try:
        port = SerialPort(path, int(rospy.get_param("~baudrate", 115200)))
        port.write(make_frame(channel, frequency, duty))
        rospy.sleep(drive_seconds)
        port.write(make_frame(channel, frequency, idle_duty))
    except (OSError, RuntimeError, ValueError) as error:
        rospy.logfatal("标定动作失败: %s", error)
        return 2
    finally:
        if port is not None:
            port.close()
    rospy.loginfo("标定动作完成: angle=%.1f duty=%d", angle, duty)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""通过 SetBool 服务控制投放机构的阻挡和释放位置。"""

import threading

import rospy
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, SetBoolResponse

from robotac_servo.protocol import ServoCalibration, make_frame
from robotac_servo.transport import SerialPort, find_port


DEFAULT_PORTS = ["/dev/robotac_servo"]


class ServoNode(object):
    def __init__(self):
        self.port_setting = str(
            rospy.get_param("~port", "/dev/robotac_servo")).strip()
        self.port_candidates = rospy.get_param("~port_candidates", DEFAULT_PORTS)
        self.baudrate = int(rospy.get_param("~baudrate", 115200))
        self.drive_seconds = float(rospy.get_param("~drive_seconds", 0.30))
        self.idle_after_move = bool(rospy.get_param("~idle_after_move", True))
        self.allow_repeat = bool(rospy.get_param("~allow_repeat", False))
        self.calibration = self._load_calibration()
        self.calibration.validate()
        if self.drive_seconds < 0.0 or self.drive_seconds > 5.0:
            raise ValueError("drive_seconds 必须在 0 至 5 秒之间")

        self.lock = threading.Lock()
        self.port = None
        self.last_state = "unknown"
        self.connected_pub = rospy.Publisher("~connected", Bool, queue_size=1,
                                             latch=True)
        self.state_pub = rospy.Publisher("~state", String, queue_size=1,
                                         latch=True)
        self.command_ok_pub = rospy.Publisher("~command_ok", Bool, queue_size=1,
                                              latch=True)
        self.service = rospy.Service("~set_released", SetBool,
                                     self._set_released)
        self.timer = rospy.Timer(rospy.Duration(1.0), self._health_check)
        rospy.on_shutdown(self._shutdown)
        connected = self._connect()
        self._publish(connected, "unknown", False)
        rospy.loginfo("舵机节点已启动，启动过程不会发送动作指令")

    def _load_calibration(self):
        return ServoCalibration(
            channel=int(rospy.get_param("~channel", 1)),
            frequency_hz=int(rospy.get_param("~frequency_hz", 50)),
            pwm_min_duty=int(rospy.get_param("~pwm_min_duty", 3)),
            pwm_max_duty=int(rospy.get_param("~pwm_max_duty", 12)),
            min_command_angle=float(rospy.get_param("~min_command_angle", 0.0)),
            max_command_angle=float(rospy.get_param("~max_command_angle", 70.0)),
            blocked_angle=float(rospy.get_param("~blocked_angle", 0.0)),
            released_angle=float(rospy.get_param("~released_angle", 45.0)),
            idle_duty=int(rospy.get_param("~idle_duty", 0)))

    def _publish(self, connected, state, command_ok):
        self.connected_pub.publish(Bool(data=connected))
        self.state_pub.publish(String(data=state))
        self.command_ok_pub.publish(Bool(data=command_ok))

    def _connect(self):
        if self.port is not None and self.port.is_open():
            return True
        self._close()
        path = find_port(self.port_setting, self.port_candidates)
        if path is None:
            return False
        try:
            self.port = SerialPort(path, self.baudrate)
            self.port.probe()
        except (OSError, RuntimeError, ValueError) as error:
            rospy.logerr("无法打开舵机串口 %s: %s", path, error)
            self._close()
            return False
        rospy.loginfo("舵机串口已连接: %s", path)
        return True

    def _close(self):
        if self.port is not None:
            try:
                self.port.close()
            except OSError:
                pass
        self.port = None

    def _write_duty(self, duty):
        frame = make_frame(self.calibration.channel,
                           self.calibration.frequency_hz, duty)
        self.port.write(frame)

    def _set_released(self, request):
        requested_state = "released" if request.data else "blocked"
        with self.lock:
            if requested_state == self.last_state and not self.allow_repeat:
                self._publish(self.port is not None, self.last_state, False)
                return SetBoolResponse(False, "拒绝重复动作")
            if not self._connect():
                self._publish(False, "error", False)
                return SetBoolResponse(False, "舵机串口不可用")
            duty = self.calibration.released_duty
            if not request.data:
                duty = self.calibration.blocked_duty
            try:
                self._write_duty(duty)
                rospy.sleep(self.drive_seconds)
                self.port.probe()
                if self.idle_after_move:
                    self._write_duty(self.calibration.idle_duty)
            except (OSError, RuntimeError, ValueError) as error:
                rospy.logerr("舵机动作失败: %s", error)
                self._close()
                self._publish(False, "error", False)
                return SetBoolResponse(False, "舵机动作失败")
            self.last_state = requested_state
            self._publish(True, requested_state, True)
            rospy.loginfo("舵机动作完成: %s", requested_state)
            return SetBoolResponse(True, requested_state)

    def _health_check(self, unused_event):
        del unused_event
        with self.lock:
            if self.port is None:
                connected = self._connect()
                self._publish(connected, self.last_state, False)
                return
            try:
                self.port.probe()
            except OSError:
                rospy.logerr("舵机串口连接中断，不会自动重放上一条指令")
                self._close()
                self._publish(False, "error", False)

    def _shutdown(self):
        with self.lock:
            if self.port is not None:
                try:
                    self._write_duty(self.calibration.idle_duty)
                except (OSError, RuntimeError, ValueError):
                    pass
            self._close()


def main():
    rospy.init_node("robotac_servo")
    try:
        ServoNode()
    except (RuntimeError, ValueError) as error:
        rospy.logfatal("舵机节点配置无效: %s", error)
        return 2
    rospy.spin()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

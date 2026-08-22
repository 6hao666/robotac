"""外部接口薄封装：MAVROS 控制、舵机释放、目标发布。

M2 飞行轮转真：真实调用 OFFBOARD / 解锁 / setpoint 流 / AUTO.LAND / 舵机服务，
全部按 06 已验证模式（预流 → OFFBOARD → arm → 20Hz setpoint → AUTO.LAND）。
dry_run=true 时只记录"将要做的动作"不真发（G1 / 拆桨地面联调用）；真飞必须 false。

本模块导入 rospy（控制层）；纯逻辑放 guards / coordinates / state_machine，
便于单元测试。单次调用、无隐含重试（RELEASE 重试次数由调用方按 payload.retry_count）。
"""

import math

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode
from std_srvs.srv import SetBool


class MissionInterfaces(object):
    def __init__(self, dry_run=True, target_sink=None):
        self.dry_run = bool(dry_run)
        self._target_sink = target_sink
        self.pending_actions = []
        self.setpoint_pub = rospy.Publisher(
            "/mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.mode_service = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self.arm_service = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.servo_service = rospy.ServiceProxy(
            "/robotac_servo/set_released", SetBool)

    def log_action(self, action):
        """记录一个将要执行的动作。dry_run 时返回 False（未真正执行）。"""
        self.pending_actions.append(action)
        return not self.dry_run

    def set_mode(self, custom_mode):
        """请求模式（OFFBOARD / AUTO.LAND）。返回 (ok, reason)。"""
        if not self.log_action("set_mode:%s" % custom_mode):
            return True, "dry_run：模拟 %s" % custom_mode
        try:
            response = self.mode_service(
                base_mode=0, custom_mode=custom_mode)
        except rospy.ServiceException as exc:
            return False, "模式请求失败: %s" % exc
        if not response.mode_sent:
            return False, "模式请求被拒绝: %s" % custom_mode
        return True, ""

    def arm(self, value):
        """解锁/上锁。返回 (ok, reason)。"""
        if not self.log_action("arm:%s" % bool(value)):
            return True, "dry_run：模拟解锁"
        try:
            response = self.arm_service(value=bool(value))
        except rospy.ServiceException as exc:
            return False, "解锁请求失败: %s" % exc
        if not response.success:
            return False, "解锁被拒绝"
        return True, ""

    def send_position(self, position, yaw, frame_id):
        """发布位置 setpoint（map 系）并转发预览目标。dry_run 只记录。"""
        message = PoseStamped()
        message.header.frame_id = frame_id
        message.header.stamp = rospy.Time.now()
        message.pose.position.x = float(position[0])
        message.pose.position.y = float(position[1])
        message.pose.position.z = float(position[2])
        half = float(yaw) * 0.5
        message.pose.orientation.z = math.sin(half)
        message.pose.orientation.w = math.cos(half)
        if self.dry_run:
            self.pending_actions.append(
                "setpoint:%.3f,%.3f,%.3f" % tuple(position))
            # dry_run 仍转发预览目标（/robotac_mission/target，地面观测），
            # 但绝不发布 /mavros/setpoint_position/local。
            if self._target_sink is not None:
                self._target_sink(message)
            return False
        self.setpoint_pub.publish(message)
        if self._target_sink is not None:
            self._target_sink(message)
        return True

    def release_payload(self, released=True):
        """调用舵机释放服务，单次。返回 (ok, reason)。"""
        if not self.log_action("release_payload:%s" % bool(released)):
            return True, "dry_run：模拟释放"
        try:
            response = self.servo_service(data=bool(released))
        except rospy.ServiceException as exc:
            return False, "舵机服务调用失败: %s" % exc
        if not response.success:
            return False, "舵机释放被拒绝"
        return True, ""

    def publish_target(self, pose_stamped):
        """发布当前目标（供地面预览与日志回放）。"""
        if self._target_sink is not None:
            self._target_sink(pose_stamped)

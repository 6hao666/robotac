"""外部接口薄封装：MAVROS、Tag、雷达/里程计、舵机。

骨架轮（dry_run=true）：只记录"将要做的动作"，不发送任何控制。
飞行轮在此实现真实调用（OFFBOARD / 解锁 / 降落 / 舵机），并继续受 dry_run 门控。

本模块不 import rospy：目标发布通过注入的回调（target_sink）完成，便于单元测试。
"""


class MissionInterfaces(object):
    def __init__(self, dry_run=True, target_sink=None):
        self.dry_run = bool(dry_run)
        self._target_sink = target_sink
        self.pending_actions = []

    def log_action(self, action):
        """记录一个将要执行的动作。dry_run 时返回 False（未真正执行）。"""
        self.pending_actions.append(action)
        return not self.dry_run

    def arm(self):
        return self.log_action("arm")

    def set_mode(self, mode):
        return self.log_action("set_mode:%s" % mode)

    def setpoint(self):
        return self.log_action("setpoint")

    def release_payload(self):
        return self.log_action("release_payload")

    def publish_target(self, pose_stamped):
        """发布当前目标（供地面预览与日志回放）。"""
        if self._target_sink is not None:
            self._target_sink(pose_stamped)

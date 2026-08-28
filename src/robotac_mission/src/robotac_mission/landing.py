"""降落确认门（纯 Python）。

只接受 AUTO.LAND 请求之后的新 extended_state 样本；样本还必须新鲜、已上锁、
位置低于允许上限且垂直速度稳定，连续满足指定次数才可宣告落地。
"""


class LandingConfirmation(object):
    def __init__(self, required_samples):
        self.required_samples = int(required_samples)
        self.reset()

    def reset(self):
        self._started = False
        self._minimum_sequence = 0
        self._last_sequence = 0
        self._consecutive = 0

    def begin(self, extended_sequence):
        """从本次 AUTO.LAND 请求开始重新取证，拒绝此前缓存的 ON_GROUND。"""
        self._started = True
        self._minimum_sequence = int(extended_sequence)
        self._last_sequence = int(extended_sequence)
        self._consecutive = 0

    def observe(self, extended_sequence, on_ground, extended_age,
                extended_timeout, armed, pose_z, pose_age, pose_timeout,
                vertical_speed, velocity_age, velocity_timeout,
                max_height, max_vertical_speed):
        """记录一个新的落地状态样本，返回是否已连续确认落地。"""
        if not self._started:
            return False
        sequence = int(extended_sequence)
        if sequence <= self._minimum_sequence or sequence <= self._last_sequence:
            return False
        self._last_sequence = sequence
        valid = (
            bool(on_ground) and
            0.0 <= extended_age <= extended_timeout and
            not bool(armed) and
            # 2026-08-28: 移除 pose_z <= max_height 判定。FAST-LIO z 漂移不可靠，
            # 落地确认只信飞控 on_ground + 未上锁 + 垂直速度稳定。
            vertical_speed is not None and
            0.0 <= velocity_age <= velocity_timeout and
            abs(float(vertical_speed)) <= max_vertical_speed)
        if valid:
            self._consecutive += 1
        else:
            self._consecutive = 0
        return self._consecutive >= self.required_samples

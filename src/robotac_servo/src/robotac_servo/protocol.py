"""舵机控制器协议和标定计算。"""

import math


def round_half_up(value):
    return int(math.floor(value + 0.5))


def pulse_width_us(frequency_hz, duty):
    if frequency_hz <= 0:
        raise ValueError("frequency_hz 必须大于 0")
    if duty < 0 or duty > 100:
        raise ValueError("duty 必须在 0 至 100 之间")
    return round_half_up(1000000.0 * duty / (frequency_hz * 100.0))


def make_frame(channel, frequency_hz, duty):
    if channel < 0 or channel > 255:
        raise ValueError("channel 必须能够写入一个字节")
    if frequency_hz < 1 or frequency_hz > 65535:
        raise ValueError("frequency_hz 必须在 1 至 65535 之间")
    if duty < 0 or duty > 100:
        raise ValueError("duty 必须在 0 至 100 之间")
    return bytes((0x5A, channel, frequency_hz >> 8,
                  frequency_hz & 0xFF, duty))


def duty_for_angle(angle, pwm_min_duty=3, pwm_max_duty=12):
    if angle < 0.0 or angle > 180.0:
        raise ValueError("angle 必须在 0 至 180 度之间")
    if pwm_min_duty < 0 or pwm_min_duty >= pwm_max_duty:
        raise ValueError("PWM 占空比上下限无效")
    if pwm_max_duty > 100:
        raise ValueError("PWM 占空比不得大于 100")
    duty = pwm_min_duty
    duty += (pwm_max_duty - pwm_min_duty) * angle / 180.0
    return round_half_up(duty)


class ServoCalibration(object):
    """保存经过地面标定的舵机参数。"""

    def __init__(self, channel=1, frequency_hz=50, pwm_min_duty=3,
                 pwm_max_duty=12, min_command_angle=0.0,
                 max_command_angle=70.0, blocked_angle=0.0,
                 released_angle=45.0, idle_duty=0):
        self.channel = channel
        self.frequency_hz = frequency_hz
        self.pwm_min_duty = pwm_min_duty
        self.pwm_max_duty = pwm_max_duty
        self.min_command_angle = min_command_angle
        self.max_command_angle = max_command_angle
        self.blocked_angle = blocked_angle
        self.released_angle = released_angle
        self.idle_duty = idle_duty

    def duty_for_angle(self, angle):
        if angle < self.min_command_angle or angle > self.max_command_angle:
            raise ValueError("舵机角度超出软件限位")
        return duty_for_angle(angle, self.pwm_min_duty, self.pwm_max_duty)

    @property
    def blocked_duty(self):
        return self.duty_for_angle(self.blocked_angle)

    @property
    def released_duty(self):
        return self.duty_for_angle(self.released_angle)

    def validate(self):
        make_frame(self.channel, self.frequency_hz, self.idle_duty)
        if self.min_command_angle < 0.0:
            raise ValueError("min_command_angle 不得小于 0")
        if self.max_command_angle > 180.0:
            raise ValueError("max_command_angle 不得大于 180")
        if self.min_command_angle >= self.max_command_angle:
            raise ValueError("舵机角度限位顺序错误")
        self.duty_for_angle(self.blocked_angle)
        self.duty_for_angle(self.released_angle)
        if self.blocked_duty == self.released_duty:
            raise ValueError("阻挡和释放位置换算为相同占空比")

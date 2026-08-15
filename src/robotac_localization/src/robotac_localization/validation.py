"""外部视觉数据检查使用的纯 Python 函数。"""

import math


def pose_values(pose):
    return [pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w]


def values_are_finite(values):
    for value in values:
        if not math.isfinite(float(value)):
            return False
    return True


def distance3(first, second):
    dx = float(first[0]) - float(second[0])
    dy = float(first[1]) - float(second[1])
    dz = float(first[2]) - float(second[2])
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def quaternion_norm(pose):
    q = pose.orientation
    return math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)

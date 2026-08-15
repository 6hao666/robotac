"""教学示例使用的纯坐标计算。"""

import math


def yaw_from_quaternion(quaternion):
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w
    return math.atan2(2.0 * (w * z + x * y),
                      1.0 - 2.0 * (y * y + z * z))


def quaternion_from_yaw(yaw):
    half = yaw * 0.5
    return [0.0, 0.0, math.sin(half), math.cos(half)]


def distance3(first, second):
    dx = first[0] - second[0]
    dy = first[1] - second[1]
    dz = first[2] - second[2]
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def relative_point(origin, yaw, point):
    forward = point[0]
    left = point[1]
    x = origin[0] + math.cos(yaw) * forward - math.sin(yaw) * left
    y = origin[1] + math.sin(yaw) * forward + math.cos(yaw) * left
    z = origin[2] + point[2]
    return [x, y, z, yaw]

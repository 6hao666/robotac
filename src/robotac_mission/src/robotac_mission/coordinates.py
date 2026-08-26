"""坐标系变换：起飞点局部归零（方案 §16.2/§16.10）。

FAST-LIO map 原点随启动漂移（实测 -5.23,-8.06），场地几何（起降台/投放台/障碍）
是场地坐标。本模块把 home（起飞时刻、停在起降台桌面的 map 位姿）作为 map→field
变换锚点：home_field = 起降台圆心（waypoints.takeoff 的 xy）+ 桌面高；并使用现场
标定的 field_yaw 旋转场地 xy 轴。

- field_to_map：场地航点 -> map setpoint（发布 /mavros/setpoint_position/local 用）。
- map_to_field：实时 map 位姿 -> 场地坐标（场地边界 / 桌区 / 到达判定用）。
二者互逆（平移），纯函数无 rospy，便于单元测试。
- limit_step：setpoint 前速度限幅（M5）。
"""

import math


def limit_step(current, target, max_step):
    """把 target 朝 current 限幅：单步位移 ≤ max_step，返回受限目标（M5）。

    setpoint 前按 max_speed/rate 限制单 tick 位移，令 limits.max_speed 生效，
    防止 TF/外参错误时 ALIGN 以 max_step×rate（理论 4 m/s）冲墙。"""
    dx = target[0] - current[0]
    dy = target[1] - current[1]
    dz = target[2] - current[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= max_step:
        return target
    scale = max_step / length
    return (current[0] + dx * scale, current[1] + dy * scale,
            current[2] + dz * scale)


class Coordinates(object):
    def __init__(self):
        self.home_map = None    # (x, y, z) 起飞时刻 map 位姿
        self.home_field = None  # (x, y, z) home 对应场地坐标（起降台桌面）
        self.home_yaw = 0.0     # 起飞时刻偏航（全程保持，示例模式）
        self.field_yaw = 0.0    # 场地 +x 相对 map +x 的已标定偏航

    @property
    def ready(self):
        return self.home_map is not None and self.home_field is not None

    def capture_home(self, pose_xyz, home_field_xy, yaw=0.0,
                     field_yaw=0.0, home_field_z=0.0):
        """捕获 map 位姿与已标定的场地坐标锚点。

        ``field_yaw`` 是场地 +x 轴相对于 map +x 轴的偏航，必须在现场标定；它和
        无人机起飞朝向 ``yaw`` 是两个不同概念。``home_field_z`` 为起降台桌面
        高度，因此场地 z 始终表示距地面的几何中心高度。
        """
        if len(pose_xyz) != 3 or len(home_field_xy) != 2:
            raise ValueError("pose_xyz 长度 3，home_field_xy 长度 2")
        self.home_map = tuple(float(value) for value in pose_xyz)
        self.home_field = (float(home_field_xy[0]),
                           float(home_field_xy[1]), float(home_field_z))
        self.home_yaw = float(yaw)
        self.field_yaw = float(field_yaw)

    def map_to_field(self, point):
        """map -> 场地坐标。home 未捕获抛 ValueError。"""
        if not self.ready:
            raise ValueError("home 未捕获，无法变换")
        dx = point[0] - self.home_map[0]
        dy = point[1] - self.home_map[1]
        cosine = math.cos(self.field_yaw)
        sine = math.sin(self.field_yaw)
        return (dx * cosine + dy * sine + self.home_field[0],
                -dx * sine + dy * cosine + self.home_field[1],
                point[2] - self.home_map[2] + self.home_field[2])

    def field_to_map(self, point):
        """场地坐标 -> map（发布 setpoint）。home 未捕获抛 ValueError。"""
        if not self.ready:
            raise ValueError("home 未捕获，无法变换")
        dx = point[0] - self.home_field[0]
        dy = point[1] - self.home_field[1]
        cosine = math.cos(self.field_yaw)
        sine = math.sin(self.field_yaw)
        return (dx * cosine - dy * sine + self.home_map[0],
                dx * sine + dy * cosine + self.home_map[1],
                point[2] + self.home_map[2] - self.home_field[2])

"""坐标系变换：起飞点局部归零（方案 §16.2/§16.10）。

FAST-LIO map 原点随启动漂移（实测 -5.23,-8.06），场地几何（起降台/投放台/障碍）
是场地坐标。本模块把 home（起飞时刻的 map 位姿）作为 map→field 变换锚点：
home_field = 起降台圆心（waypoints.takeoff 的 xy）+ 地面 z=0。

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
    def __init__(self, field_yaw=0.0):
        self.home_map = None    # (x, y, z) 起飞时刻 map 位姿
        self.home_field = None  # (x, y, z) home 对应场地坐标（地面 z=0）
        self.home_yaw = 0.0     # 起飞时刻 map 系偏航
        self.field_yaw = float(field_yaw)  # 起飞时机头在场地系朝向（rad；机头朝场地+y=pi/2）

    @property
    def ready(self):
        return self.home_map is not None and self.home_field is not None

    def capture_home(self, pose_xyz, home_field_xy, yaw=0.0):
        """start 时捕获：map 位姿 (x,y,z) + home 场地 xy（起降台圆心）。

        home_field_z 固定 0（地面）；yaw 记录起飞朝向。重复调用以最后一次为准。"""
        if len(pose_xyz) != 3 or len(home_field_xy) != 2:
            raise ValueError("pose_xyz 长度 3，home_field_xy 长度 2")
        self.home_map = tuple(float(value) for value in pose_xyz)
        self.home_field = (float(home_field_xy[0]),
                           float(home_field_xy[1]), 0.0)
        self.home_yaw = float(yaw)

    def _rot_xy(self, dx, dy, theta):
        """绕 z 旋转 (dx,dy) 角度 theta（rad），map↔field 用。"""
        c, s = math.cos(theta), math.sin(theta)
        return (c * dx - s * dy, s * dx + c * dy)

    def map_to_field(self, point):
        """map -> 场地坐标。home 未捕获抛 ValueError。"""
        if not self.ready:
            raise ValueError("home 未捕获，无法变换")
        dx = point[0] - self.home_map[0]
        dy = point[1] - self.home_map[1]
        rx, ry = self._rot_xy(dx, dy, self.field_yaw - self.home_yaw)
        return (self.home_field[0] + rx,
                self.home_field[1] + ry,
                self.home_field[2] + (point[2] - self.home_map[2]))

    def field_to_map(self, point):
        """场地坐标 -> map（发布 setpoint）。home 未捕获抛 ValueError。"""
        if not self.ready:
            raise ValueError("home 未捕获，无法变换")
        dx = point[0] - self.home_field[0]
        dy = point[1] - self.home_field[1]
        rx, ry = self._rot_xy(dx, dy, self.home_yaw - self.field_yaw)
        return (self.home_map[0] + rx,
                self.home_map[1] + ry,
                self.home_map[2] + (point[2] - self.home_field[2]))

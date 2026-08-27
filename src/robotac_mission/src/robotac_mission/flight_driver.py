"""飞行驱动：20Hz 逐态推进 C1-C5（TAKEOFF→LAND）。

读节点（ctx）快照，只前向推进（stage_done），健康门/超时 -> abort。策略全走
YAML（占位），ALIGN/RELEASE 依赖 C3 TF 链 / C4 舵机（硬依赖）。rospy 层不单测。
"""

import math
import time

from robotac_mission import guards
from robotac_mission._flight_stages import FlightStageMixin
from robotac_mission.coordinates import limit_step
from robotac_mission.flight_health import check as health_check
from robotac_mission.tag_tracker import TagTracker

class FlightDriver(FlightStageMixin):
    def __init__(self, ctx, machine, interfaces, coord, config):
        self.ctx = ctx
        self.machine = machine
        self.interfaces = interfaces
        self.coord = coord
        self.config = config
        self.takeoff_target = config["waypoints"]["takeoff"]
        self.mission_point = config["waypoints"]["mission"][0]
        self.return_point = config["waypoints"]["return"][0]
        self.tag = TagTracker(config, ctx.tf_buffer)
        self._health = {"soft_bad": 0, "prev_pose": None}
        self._last_target = None
        self._reset_stage()

    def start(self, home_map_xyz, home_yaw):
        """start 接受后调用：捕获 home（起飞点局部归零，§16.2）。"""
        self.coord.capture_home(
            home_map_xyz, self.config["waypoints"]["takeoff"][:2],
            yaw=home_yaw)
        self._reset_stage()

    def tick(self):
        """一个 20Hz 控制步。返回当前机器状态。"""
        if (self.ctx.fcu_state is not None and
                guards.manual_mode_active(self.ctx.fcu_state.mode,
                                          self.ctx.fcu_state.armed) and
                not self.awaiting_offboard_confirmation()):
            self.machine.confirm_manual_takeover()
            self.ctx.cancel_landing()
            self.ctx.publish_all()
            return self.machine.state
        issue = health_check(self.ctx, self.coord, self.config, self._health)
        if issue is not None:
            self._abort(issue)
            return self.machine.state
        stage = self.machine.state
        # 借自 agent-fix：C1 已完成后若飞控意外上锁，继续发航点既不安全也不符合自主飞行语义。
        if stage not in ("TAKEOFF", "LAND") and not self.ctx.fcu_state.armed:
            self._abort("飞控意外上锁")
            return self.machine.state
        if stage == "TAKEOFF":
            self._stage_takeoff()
        elif stage == "TRANSIT":
            self._follow(self._routing_points("obstacle_routing"),
                         "transit", "绕障")
        elif stage == "SEARCH_TAG":
            self._stage_search()
        elif stage == "ALIGN_TAG":
            self._stage_align()
        elif stage == "RELEASE":
            self._stage_release()
        elif stage == "RETURN":
            self._follow(self._return_points(), "return", "返航")
        elif stage == "LAND":
            self._stage_land()
        return self.machine.state

    def _routing_points(self, name):
        routing = self.config["waypoints"][name]
        return [routing[key] for key in ("approach", "gap_enter",
                                         "gap_cross", "resume")]

    def _return_points(self):
        return self._routing_points("return_routing") + [self.return_point]

    def _follow(self, points, timeout_key, label):
        index = self._index
        if index >= len(points):
            return
        self._send(points[index])
        if self._arrived(points[index],
                         self.config["timing"]["waypoint_hold"]):
            self._index = index + 1
            if self._index >= len(points):
                self._advance()
        elif self._stage_timeout(
                self.config["timing"]["stage_timeout"][timeout_key]):
            self._abort("%s超时" % label)

    def _field_pose(self):
        if self.ctx.pose is None:
            return None
        return self.coord.map_to_field(
            (self.ctx.pose.pose.position.x, self.ctx.pose.pose.position.y,
             self.ctx.pose.pose.position.z))

    def _send(self, target_field, is_map=False):
        # 速度限幅：单 tick 位移 ≤ max_speed/rate（M5）。
        #
        # 必须从“上一次发布的 setpoint”推进，不能从实时 pose 推进：后者会把
        # 位置误差永远钳在一个 tick 的距离（例如 0.6 / 20 = 3cm），PX4 几乎
        # 不会加速，表现为悬停并被估计漂移带走。
        max_step = (self.config["limits"]["max_speed"] /
                    self.config["control"]["rate_hz"])
        if is_map:
            # 直接 map 坐标（相对爬升等，不走 field_to_map——home_map_z 会漂移）
            target_map = tuple(target_field)
        else:
            target_map = self.coord.field_to_map(target_field)
            if self._last_target is not None:
                target_map = limit_step(self._last_target[0], target_map,
                                        max_step)
        self.interfaces.send_position(
            target_map, self.coord.home_yaw,
            self.config["frames"]["mission_frame"])
        self._last_target = (target_map, self.coord.home_yaw)

    def _arrived_map(self, target_map, hold):
        """相对 map 目标到达判定（用于相对爬升起飞）。"""
        if self.ctx.pose is None:
            return False
        current = (self.ctx.pose.pose.position.x,
                   self.ctx.pose.pose.position.y,
                   self.ctx.pose.pose.position.z)
        tolerance = self.config["control"]["position_tolerance"]
        if math.dist(current, target_map[:3]) <= tolerance:
            if self._reached_since is None:
                self._reached_since = self._now()
            if self._now() - self._reached_since >= hold:
                return True
        else:
            self._reached_since = None
        return False

    def _arrived(self, target_field, hold):
        pose_field = self._field_pose()
        if pose_field is None:
            return False
        tolerance = self.config["control"]["position_tolerance"]
        if math.dist(pose_field, target_field) <= tolerance:
            if self._reached_since is None:
                self._reached_since = self._now()
            return self._now() - self._reached_since >= hold
        self._reached_since = None
        return False

    def _advance(self):
        if (self.machine.state == "SEARCH_TAG" and
                self.config["mission"].get("route_only", False)):
            ok, message = self.machine.skip_to_return()
        else:
            ok, message = self.machine.stage_done()
        if not ok:
            self._abort(message)
            return
        if self.machine.state == "LAND":
            self.ctx.begin_landing(self._last_target)
        self._reset_stage()
        self.ctx.publish_all()

    def _abort(self, reason):
        self.machine.abort(reason)
        if self.machine.state == "ABORT_LAND":
            self.ctx.begin_landing(self._last_target)
        self.ctx.publish_all()

    def _reset_stage(self):
        self._takeoff_armed = False
        self._takeoff_offboard_requested = False
        self._takeoff_offboard_confirmed = False
        self._takeoff_arm_requested = False
        self._takeoff_map = None
        self._servo_called = False
        self._land_requested = False
        self._index = 0
        # _last_target 是速度限幅的积分起点，跨阶段保留才能避免新航点首帧跳变。
        # 新 FlightDriver 在 start 前创建，初始值仍为 None。
        self._reached_since = None
        self._stage_start = self._now()

    def _stage_timeout(self, seconds):
        return self._now() - self._stage_start > seconds

    def awaiting_offboard_confirmation(self):
        """OFFBOARD 请求已发出但飞控尚未确认的短暂窗口。"""
        return (self.machine.state == "TAKEOFF" and
                self._takeoff_offboard_requested and
                not self._takeoff_offboard_confirmed)

    @staticmethod
    def _now():
        return time.monotonic()

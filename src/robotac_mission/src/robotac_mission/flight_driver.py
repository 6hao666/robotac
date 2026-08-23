"""飞行驱动：20Hz 逐态推进 C1-C5（TAKEOFF→LAND）。

读节点（ctx）快照，只前向推进（stage_done），健康门/超时 -> abort。策略全走
YAML（占位），ALIGN/RELEASE 依赖 C3 TF 链 / C4 舵机（硬依赖）。rospy 层不单测。
"""

import math
import time

from mavros_msgs.msg import ExtendedState

from robotac_mission.coordinates import limit_step
from robotac_mission.flight_health import check as health_check
from robotac_mission.tag_tracker import TagTracker

class FlightDriver(object):
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
        self._reset_stage()

    def start(self, home_map_xyz, home_yaw):
        """start 接受后调用：捕获 home（起飞点局部归零，§16.2）。"""
        self.coord.capture_home(
            home_map_xyz, self.config["waypoints"]["takeoff"][:2],
            yaw=home_yaw)
        self._reset_stage()

    def tick(self):
        """一个 20Hz 控制步。返回当前机器状态。"""
        issue = health_check(self.ctx, self.coord, self.config, self._health)
        if issue is not None:
            self._abort(issue)
            return self.machine.state
        stage = self.machine.state
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

    def _stage_takeoff(self):
        timing = self.config["timing"]
        control = self.config["control"]
        # 起飞目标捕获一次固定（相对当前 z + 高度），防止每 tick 重算导致目标
        # 跟着飞机一起涨、永远追不上（2026-08-23 事故：一直爬升不停）。
        if self._takeoff_map is None:
            self._takeoff_map = self._relative_takeoff_target()
        takeoff = self._takeoff_map
        if not self._takeoff_armed:
            if self._now() - self._stage_start < control["prestream_seconds"]:
                self._send(takeoff, is_map=True)
                return
            ok, reason = self.interfaces.set_mode("OFFBOARD")
            if ok:
                ok, reason = self.interfaces.arm(True)
            if not ok:
                self._abort(reason)
                return
            self._takeoff_armed = True
            self._stage_start = self._now()
        self._send(takeoff, is_map=True)
        if self._arrived_map(takeoff, timing["takeoff_hold"]):
            self._advance()
        elif self._stage_timeout(timing["stage_timeout"]["takeoff"]):
            self._abort("起飞超时")

    def _relative_takeoff_target(self):
        """起飞目标：相对当前 FC 位置爬升（跟示例 begin(height) 一致，抗 z 漂移）。

        2026-08-23 修复：FC z 估计漂移使 home_map_z 失效，绝对场地坐标的起飞
        setpoint z 偏低（实测最高 0.15m，应为 1.0m）→ 飞控只看到小爬升误差 →
        油门不够不升空 → 起飞超时。改为每 tick 按当前 FC z + 起飞高度发目标。"""
        target = list(self.takeoff_target)  # [x, y, z] 场地坐标（x/y=起飞点）
        if self.ctx.pose is not None:
            # 场地 z 映射为 map z：当前 FC z + 起飞高度（相对爬升）
            target[0] = self.ctx.pose.pose.position.x
            target[1] = self.ctx.pose.pose.position.y
            target[2] = self.ctx.pose.pose.position.z + target[2]
        return target

    def _stage_search(self):
        timing = self.config["timing"]
        self.tag.update(self.ctx.tag, self.ctx.topic_age("tag"))
        self._send(self.mission_point)   # 投放台上方悬停扫描
        if self.tag.fresh:
            tag_field = self.coord.map_to_field(self.tag.pose_map)
            center = self.config["tables"]["delivery_center"]
            radius = self.config["tables"]["search_radius"]
            if math.dist(tag_field[:2], center) <= radius:
                self._advance()
                return
        if self._stage_timeout(timing["stage_timeout"]["search"]):
            self._abort("搜索 Tag 超时")

    def _stage_align(self):
        timing = self.config["timing"]
        control = self.config["control"]
        self.tag.update(self.ctx.tag, self.ctx.topic_age("tag"))
        if not self.tag.fresh:
            if self._last_target is not None:   # 保持目标悬停，超时中止
                self.interfaces.send_position(
                    self._last_target[0], self._last_target[1],
                    self.config["frames"]["mission_frame"])
            if self._stage_timeout(timing["stage_timeout"]["align"]):
                self._abort("对准阶段 Tag 丢失")
            return
        pose_field = self._field_pose()
        if pose_field is None:
            self._abort("本地位姿失效")
            return
        tag_field = self.coord.map_to_field(self.tag.pose_map)
        dx = tag_field[0] - pose_field[0]
        dy = tag_field[1] - pose_field[1]
        length = math.hypot(dx, dy)
        scale = 1.0
        if length > control["max_step"]:
            scale = control["max_step"] / length
        self._send([pose_field[0] + dx * scale,
                    pose_field[1] + dy * scale, self.mission_point[2]])
        if length <= control["position_tolerance"]:
            if self._reached_since is None:
                self._reached_since = self._now()
            if self._now() - self._reached_since >= timing["align_hold"]:
                self._advance()
        else:
            self._reached_since = None
        if self._stage_timeout(timing["stage_timeout"]["align"]):
            self._abort("对准超时")

    def _stage_release(self):
        timing = self.config["timing"]
        if not self._servo_called:
            self._servo_called = True
            if self.config["payload"]["enable"]:
                attempts = 0
                ok = False
                reason = "舵机释放失败"
                while attempts <= self.config["payload"]["retry_count"]:
                    ok, reason = self.interfaces.release_payload(True)
                    if ok:
                        break
                    attempts += 1
                if not ok:
                    self._abort(reason)
                    return
            else:
                self.interfaces.log_action("release_skipped")   # 空投（M2）
        self._send(self.mission_point)   # 释放后保持，等待货物脱离
        if self._arrived(self.mission_point, timing["release_hold"]):
            self._advance()
        elif self._stage_timeout(timing["stage_timeout"]["release"]):
            self._abort("投放等待超时")

    def _stage_land(self):
        timing = self.config["timing"]
        if not self._land_requested:
            self._land_requested = True
            ok, reason = self.interfaces.set_mode("AUTO.LAND")
            if not ok:
                self._abort(reason)
                return
        extended = self.ctx.extended_state
        if extended is not None and (
                extended.landed_state == ExtendedState.LANDED_STATE_ON_GROUND):
            self._advance()
        elif self._stage_timeout(timing["land_confirm"]):
            self._abort("降落超时")

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
        # 速度限幅：单 tick 位移 ≤ max_speed/rate（M5），令 limits.max_speed 生效
        max_step = (self.config["limits"]["max_speed"] /
                    self.config["control"]["rate_hz"])
        target = target_field
        pose_field = self._field_pose()
        if pose_field is not None and not is_map:
            target = limit_step(pose_field, target_field, max_step)
        if is_map:
            # 直接 map 坐标（相对爬升等，不走 field_to_map——home_map_z 会漂移）
            target_map = tuple(target)
        else:
            target_map = self.coord.field_to_map(target)
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
        ok, message = self.machine.stage_done()
        if not ok:
            self._abort(message)
            return
        self._reset_stage()
        self.ctx.publish_all()

    def _abort(self, reason):
        self.machine.abort(reason)
        self.ctx.publish_all()

    def _reset_stage(self):
        self._takeoff_armed = False
        self._takeoff_map = None
        self._servo_called = False
        self._land_requested = False
        self._index = 0
        self._reached_since = self._last_target = None
        self._stage_start = self._now()

    def _stage_timeout(self, seconds):
        return self._now() - self._stage_start > seconds

    @staticmethod
    def _now():
        return time.monotonic()

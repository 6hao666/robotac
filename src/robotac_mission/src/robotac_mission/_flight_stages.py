"""飞行阶段策略：C1-C5 各阶段的"策略"方法（mixin，供 FlightDriver 组合）。

与 flight_driver.FlightDriver 拆分（2026-08-23 为过 check_source 250 行红线）：
FlightDriver 保留 tick 调度、_send/_arrived 坐标推进、_advance/_abort 骨架；
本模块只含各阶段的"做什么"策略，通过 self 访问 driver 的推进/坐标/接口。
纯 rospy 逻辑不单测。
"""

import math

from mavros_msgs.msg import ExtendedState


class FlightStageMixin(object):
    def _stage_takeoff(self):
        timing = self.config["timing"]
        control = self.config["control"]
        # 起飞目标捕获一次固定（相对当前 z + 高度），防止每 tick 重算导致目标
        # 跟着飞机一起涨、永远追不上（2026-08-23 事故：一直爬升不停）。
        if self._takeoff_map is None:
            self._takeoff_map = self._relative_takeoff_target()
            # 同步地面参考 z：field_to_map 用一致的起飞地面高度（防 home_map_z 陈旧
            # 导致 routing/SEARCH 等航点 z 偏低，飞机飞完起飞就降到地面）
            if self.ctx.pose is not None:
                self.coord.home_map = (self.coord.home_map[0],
                                       self.coord.home_map[1],
                                       self.ctx.pose.pose.position.z)
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
            if not self.config["payload"]["enable"]:
                self._abort("payload 未启用，禁止跳过投放")
                return
            self._release_sequence = self.ctx.payload_release_sequence
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
        release = self._release_target()   # 投放点 = mission_point - 舵机偏移（补偿）
        self._send(release)                # 释放后保持，等待货物脱离
        if not self.ctx.payload_confirmed_after(self._release_sequence):
            if self._stage_timeout(timing["stage_timeout"]["release"]):
                self._abort("投放后未收到货物完全脱离确认")
            return
        if self._arrived(release, timing["release_hold"]):
            self._advance()
        elif self._stage_timeout(timing["stage_timeout"]["release"]):
            self._abort("投放等待超时")

    def _release_target(self):
        """投放目标：舵机偏移补偿。

        mission_point 是投放台中心正上方（飞机几何中心对准）。舵机相对 body
        有水平偏移 payload.offset（2026-08-26 实测：机尾 -0.04m），若飞机几何
        中心对准投放中心，舵机落点会偏 4cm。补偿：飞机往反方向偏 offset 的水平
        分量，使舵机正好在投放中心正上方（C4 落点精度）。
        """
        offset = self.config["payload"].get("offset", [0.0, 0.0, 0.0])
        return [self.mission_point[0] - offset[0],
                self.mission_point[1] - offset[1],
                self.mission_point[2]]

    def _stage_land(self):
        self.ctx.begin_landing(self._last_target)

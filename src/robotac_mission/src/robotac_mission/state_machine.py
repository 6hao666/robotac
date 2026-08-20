"""robotac_mission 状态机核心（纯 Python，不依赖 rospy 时间）。

骨架轮状态：BOOT / WAIT_READY / WAIT_START（预启动），ABORT_LAND / COMPLETE /
ERROR（终态）。飞行状态（TAKEOFF / TRANSIT / SEARCH_TAG / ALIGN_TAG / RELEASE /
RETURN / LAND）留待下一轮，本文件仅定义终态转移接口，供飞行轮复用。

设计要点：
- 不依赖 rospy 时间：时钟由调用方注入（默认 time.monotonic），便于单元测试。
- 只记录第一个根因作为 mission result，后续派生错误作为附加事件，避免覆盖。
- 转移优先级：操作员 stop / 飞控断连 > 安全门 > 任务推进（骨架轮仅 stop 生效）。
- BOOT 可重入：ERROR -> reset -> BOOT 后由节点重新从磁盘加载参数（本文件只复位
  状态，参数重读在 mission_node 完成）。
"""

import time


class MissionState(object):
    BOOT = "BOOT"
    WAIT_READY = "WAIT_READY"
    WAIT_START = "WAIT_START"
    ABORT_LAND = "ABORT_LAND"
    COMPLETE = "COMPLETE"
    ERROR = "ERROR"

    PRE_START = (BOOT, WAIT_READY, WAIT_START)
    TERMINAL = (ABORT_LAND, COMPLETE, ERROR)
    # 飞行活动态（下一轮）：TAKEOFF / TRANSIT / SEARCH_TAG / ALIGN_TAG /
    # RELEASE / RETURN / LAND，接在 WAIT_START 之后，stop/安全门 -> ABORT_LAND。


class MissionResult(object):
    """首个根因结果值；空串表示尚未产生终态结果。"""
    PARAM_INVALID = "参数无效"
    SAFE_LANDING = "安全降落"
    MANUAL_TAKEOVER = "人工接管"


class MissionStateMachine(object):
    def __init__(self, clock=None):
        self._clock = clock if clock is not None else time.monotonic
        self.state = MissionState.BOOT
        self.reason = ""
        self.result = ""
        self.active = False
        self.transitions = []
        self._preconditions_ok = False
        self._max_log = 200

    # ---- 事件处理（由 mission_node 把 ROS 消息翻译成事件后调用）----

    def handle_boot_params(self, ok, reason=""):
        """BOOT：参数读取/校验结果。ok=True -> WAIT_READY；False -> ERROR。"""
        self._log("BOOT_PARAMS", self.state, reason or "参数校验")
        if ok:
            self._enter(MissionState.WAIT_READY, reason or "参数校验通过，等待就绪")
        else:
            self._enter(MissionState.ERROR, reason or "参数校验失败")
            self._set_result(MissionResult.PARAM_INVALID)
        return self.state

    def handle_preconditions(self, ok, reason=""):
        """安全门更新。WAIT_READY 前置满足 -> WAIT_START；WAIT_START 前置失效
        -> 回 WAIT_READY（不启动，无中止语义）。其他状态忽略。"""
        self._preconditions_ok = ok
        if self.state == MissionState.WAIT_READY:
            if ok:
                self._enter(MissionState.WAIT_START,
                            reason or "前置条件满足，等待 start")
            else:
                self.reason = reason or "就绪条件不满足"
        elif self.state == MissionState.WAIT_START:
            if not ok:
                self._enter(MissionState.WAIT_READY,
                            reason or "前置条件失效，回退等待")
        return self.state

    def request_start(self):
        """start 服务。仅在 WAIT_START 且前置条件通过时接受。

        骨架轮语义：接受（返回 success）但停留 WAIT_START，不产生任何控制输出；
        active 保持 False。飞行轮 start 才触发 TAKEOFF。
        """
        if self.state != MissionState.WAIT_START:
            return False, "当前状态不可启动：" + self.state
        if not self._preconditions_ok:
            return False, "前置条件不满足，等待就绪"
        self._log("START", self.state, "飞行态未启用（骨架）")
        self.reason = "飞行态未启用（骨架）"
        self.active = False
        return True, "飞行态未启用（骨架）"

    def request_stop(self):
        """stop 服务。预启动态不启动任务；终态幂等；飞行活动态 -> ABORT_LAND。"""
        if self.state == MissionState.BOOT:
            return True, "引导中，stop 无效果"
        if self.state == MissionState.WAIT_READY:
            return True, "已就绪等待，stop 无效果"
        if self.state == MissionState.WAIT_START:
            self._enter(MissionState.WAIT_READY, "操作员 stop，回退等待就绪")
            return True, "已回退到 WAIT_READY"
        if self.state in MissionState.TERMINAL:
            return True, "任务已停止，stop 幂等"
        self.abort("操作员 stop")
        return True, "已进入 ABORT_LAND"

    def request_reset(self):
        """reset 服务。终态 -> 新一轮：COMPLETE/ABORT_LAND -> WAIT_READY；
        ERROR -> BOOT（由节点重读参数）；预启动态幂等。"""
        if self.state == MissionState.BOOT:
            return True, "引导中，reset 幂等"
        if self.state in (MissionState.WAIT_READY, MissionState.WAIT_START):
            return True, "任务尚未开始，reset 幂等"
        if self.state in (MissionState.COMPLETE, MissionState.ABORT_LAND):
            self._clear_result()
            self._enter(MissionState.WAIT_READY, "mission_reset，准备下一次飞行")
            return True, "已重置到 WAIT_READY"
        if self.state == MissionState.ERROR:
            self._clear_result()
            self._enter(MissionState.BOOT, "mission_reset，重读参数")
            return True, "已重置到 BOOT，将重读参数"
        # 兜底：当前 6 态均被上面分支覆盖，此分支只对未来的飞行活动态可达。飞行中
        # reset 须先 stop（方案 §5.2），必须 fail-safe 拒绝，不得静默成功（R5-1）。
        return False, "任务进行中，请先 stop 再 reset"

    # ---- 飞行轮入口（骨架轮定义接口：通过 ROS 流程不可达，单测可直接调用）----

    def abort(self, reason=""):
        """飞行活动态收到 stop 或安全门触发 -> ABORT_LAND。"""
        if self.state in (MissionState.ABORT_LAND,) + MissionState.TERMINAL:
            return self.state
        self._enter(MissionState.ABORT_LAND, reason or "中止请求")
        return self.state

    def confirm_landed(self):
        """ABORT_LAND 落地确认 -> COMPLETE（结果：安全降落）。"""
        if self.state != MissionState.ABORT_LAND:
            return self.state
        self._enter(MissionState.COMPLETE, "安全降落")
        self._set_result(MissionResult.SAFE_LANDING)
        return self.state

    def confirm_manual_takeover(self):
        """ABORT_LAND 人工接管 -> COMPLETE（结果：人工接管）。"""
        if self.state != MissionState.ABORT_LAND:
            return self.state
        self._enter(MissionState.COMPLETE, "人工接管")
        self._set_result(MissionResult.MANUAL_TAKEOVER)
        return self.state

    # ---- 内部工具 ----

    def _enter(self, state, reason):
        self._log("TRANSITION", state, reason)
        self.state = state
        self.reason = reason

    def _log(self, event, to, detail=""):
        self.transitions.append({
            "at": self._clock(),
            "from": self.state,
            "to": to,
            "event": event,
            "detail": detail,
        })
        if len(self.transitions) > self._max_log:
            del self.transitions[:len(self.transitions) - self._max_log]

    def _set_result(self, result):
        if not self.result:
            self.result = result

    def _clear_result(self):
        self.result = ""

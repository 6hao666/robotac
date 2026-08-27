#!/usr/bin/env python3
"""state_machine 单元测试（纯 Python，不依赖 rospy 时间）。

覆盖方案 §9.2：正常流程、启动保护、定位异常回退、操作员 stop、幂等性、
reset 语义、飞行轮入口接口、可观测性。
"""

import unittest

from robotac_mission.state_machine import (
    MissionResult, MissionState, MissionStateMachine)


class FakeClock(object):
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now


class BootFlowTest(unittest.TestCase):
    def test_normal_flow_to_wait_start(self):
        machine = MissionStateMachine()
        machine.handle_boot_params(True)
        self.assertEqual(machine.state, MissionState.WAIT_READY)
        machine.handle_preconditions(True)
        self.assertEqual(machine.state, MissionState.WAIT_START)
        self.assertFalse(machine.active)

    def test_boot_param_failure_enters_error(self):
        machine = MissionStateMachine()
        machine.handle_boot_params(False, "参数无效：缺少 limits")
        self.assertEqual(machine.state, MissionState.ERROR)
        self.assertEqual(machine.result, MissionResult.PARAM_INVALID)

    def test_error_reset_reboots_and_clears_result(self):
        machine = MissionStateMachine()
        machine.handle_boot_params(False, "参数无效")
        machine.request_reset()
        self.assertEqual(machine.state, MissionState.BOOT)
        self.assertEqual(machine.result, "")
        machine.handle_boot_params(True)
        self.assertEqual(machine.state, MissionState.WAIT_READY)


class StartStopResetTest(unittest.TestCase):
    def _ready(self):
        machine = MissionStateMachine()
        machine.handle_boot_params(True)
        machine.handle_preconditions(True)
        return machine

    def test_start_placeholder_stays_in_wait_start(self):
        machine = self._ready()
        success, message = machine.request_start()
        self.assertTrue(success)
        self.assertIn("飞行态未启用", message)
        self.assertEqual(machine.state, MissionState.WAIT_START)
        self.assertFalse(machine.active)
        # 幂等：连续调用不产生第二次启动语义
        success, _ = machine.request_start()
        self.assertTrue(success)

    def test_start_rejected_when_not_in_wait_start(self):
        machine = self._ready()
        machine.handle_preconditions(False)
        self.assertEqual(machine.state, MissionState.WAIT_READY)
        success, message = machine.request_start()
        self.assertFalse(success)
        self.assertIn("当前状态不可启动", message)

    def test_start_accepted_after_preconditions_recover(self):
        machine = self._ready()
        machine.handle_preconditions(False)
        machine.handle_preconditions(True)
        self.assertEqual(machine.state, MissionState.WAIT_START)
        success, _ = machine.request_start()
        self.assertTrue(success)

    def test_stop_from_wait_start_returns_to_wait_ready(self):
        machine = self._ready()
        machine.request_stop()
        self.assertEqual(machine.state, MissionState.WAIT_READY)

    def test_stop_idempotent_in_prestart(self):
        machine = self._ready()
        machine.request_stop()
        machine.request_stop()
        self.assertEqual(machine.state, MissionState.WAIT_READY)

    def test_stop_keeps_wait_ready_idle(self):
        machine = self._ready()
        machine.handle_preconditions(False)
        self.assertEqual(machine.state, MissionState.WAIT_READY)
        machine.request_stop()
        self.assertEqual(machine.state, MissionState.WAIT_READY)

    def test_stop_idempotent_in_terminal(self):
        machine = self._ready()
        machine.abort("安全门")
        machine.confirm_landed()
        machine.request_stop()
        self.assertEqual(machine.state, MissionState.COMPLETE)

    def test_reset_idempotent_before_start(self):
        machine = self._ready()
        success, _ = machine.request_reset()
        self.assertTrue(success)
        self.assertEqual(machine.state, MissionState.WAIT_START)

    def test_reset_from_complete_returns_to_wait_ready(self):
        machine = self._ready()
        machine.abort("安全门")
        machine.confirm_landed()
        self.assertEqual(machine.state, MissionState.COMPLETE)
        self.assertEqual(machine.result, MissionResult.SAFE_LANDING)
        machine.request_reset()
        self.assertEqual(machine.state, MissionState.WAIT_READY)
        self.assertEqual(machine.result, "")

    def test_reset_rejected_while_abort_landing_is_active(self):
        machine = self._ready()
        machine.request_start(flight_enabled=True)
        machine.request_stop()
        self.assertEqual(machine.state, MissionState.ABORT_LAND)
        success, message = machine.request_reset()
        self.assertFalse(success)
        self.assertIn("安全降落", message)
        self.assertTrue(machine.active)

    def test_reset_from_manual_takeover_returns_to_wait_ready(self):
        machine = self._ready()
        machine.abort("飞控断连")
        machine.confirm_manual_takeover()
        self.assertEqual(machine.state, MissionState.COMPLETE)
        self.assertEqual(machine.result, MissionResult.MANUAL_TAKEOVER)
        machine.request_reset()
        self.assertEqual(machine.state, MissionState.WAIT_READY)

    def test_reset_rejected_in_future_flight_state(self):
        # R5-1：当前状态全被显式分支覆盖，兜底仅对未来的飞行活动态生效——直接注入
        # 飞行态验证 reset 必须拒绝（方案 §5.2：飞行中须先 stop，不得静默成功）。
        machine = self._ready()
        machine.state = "TAKEOFF"
        success, message = machine.request_reset()
        self.assertFalse(success)
        self.assertIn("stop", message)


class FlightRoundEntryTest(unittest.TestCase):
    """骨架轮不可达、由单测直接驱动的飞行轮入口接口。"""

    def test_abort_to_landed_complete(self):
        machine = MissionStateMachine()
        machine.handle_boot_params(True)
        machine.handle_preconditions(True)
        machine.abort("操作员 stop")
        self.assertEqual(machine.state, MissionState.ABORT_LAND)
        machine.confirm_landed()
        self.assertEqual(machine.state, MissionState.COMPLETE)
        self.assertEqual(machine.result, MissionResult.SAFE_LANDING)

    def test_abort_is_idempotent(self):
        machine = MissionStateMachine()
        machine.handle_boot_params(True)
        machine.abort("安全门")
        machine.abort("重复触发")
        self.assertEqual(machine.state, MissionState.ABORT_LAND)

    def test_terminal_state_ignores_abort(self):
        machine = MissionStateMachine()
        machine.handle_boot_params(False)
        machine.abort("不应离开 ERROR")
        self.assertEqual(machine.state, MissionState.ERROR)


class PreconditionTest(unittest.TestCase):
    def test_precondition_dropout_from_wait_start(self):
        machine = MissionStateMachine()
        machine.handle_boot_params(True)
        machine.handle_preconditions(True)
        machine.handle_preconditions(False, "本地位置过期")
        self.assertEqual(machine.state, MissionState.WAIT_READY)

    def test_observability_logs_transition(self):
        machine = MissionStateMachine(clock=FakeClock())
        machine.handle_boot_params(True)
        machine.handle_preconditions(True)
        self.assertTrue(machine.transitions)
        last = machine.transitions[-1]
        self.assertEqual(last["from"], MissionState.WAIT_READY)
        self.assertEqual(last["to"], MissionState.WAIT_START)
        self.assertIn("at", last)
        self.assertIn("event", last)


class FlightStateMachineTest(unittest.TestCase):
    """M2 飞行轮：完整 C1-C5 七态转移（表驱动 stage_done）。"""

    def _fly(self):
        machine = MissionStateMachine()
        machine.handle_boot_params(True)
        machine.handle_preconditions(True)
        success, _ = machine.request_start(flight_enabled=True)
        self.assertTrue(success)
        self.assertEqual(machine.state, MissionState.TAKEOFF)
        self.assertTrue(machine.active)
        return machine

    def test_start_flight_enabled_enters_takeoff(self):
        machine = self._fly()

    def test_placeholder_when_flight_disabled(self):
        machine = MissionStateMachine()
        machine.handle_boot_params(True)
        machine.handle_preconditions(True)
        success, message = machine.request_start(flight_enabled=False)
        self.assertTrue(success)
        self.assertIn("飞行态未启用", message)
        self.assertEqual(machine.state, MissionState.WAIT_START)
        self.assertFalse(machine.active)

    def test_full_cycle_takeoff_to_complete(self):
        machine = self._fly()
        expected = [MissionState.TRANSIT, MissionState.SEARCH_TAG,
                    MissionState.ALIGN_TAG, MissionState.RELEASE,
                    MissionState.RETURN, MissionState.LAND]
        for state in expected:
            ok, _ = machine.stage_done()
            self.assertTrue(ok)
            self.assertEqual(machine.state, state)
            self.assertTrue(machine.active)
        ok, _ = machine.stage_done()   # LAND -> COMPLETE
        self.assertTrue(ok)
        self.assertEqual(machine.state, MissionState.COMPLETE)
        self.assertEqual(machine.result, MissionResult.SAFE_LANDING)
        self.assertFalse(machine.active)

    def test_stage_done_invalid_outside_flight(self):
        machine = MissionStateMachine()
        machine.handle_boot_params(True)
        ok, _ = machine.stage_done()
        self.assertFalse(ok)

    def test_abort_during_flight_records_root_cause(self):
        machine = self._fly()
        machine.stage_done()   # TRANSIT
        machine.abort("绕障超时")
        self.assertEqual(machine.state, MissionState.ABORT_LAND)
        self.assertIn("绕障超时", machine.result)
        machine.confirm_landed()
        self.assertEqual(machine.state, MissionState.COMPLETE)
        self.assertIn("绕障超时", machine.result)   # 首个根因保留

    def test_stop_during_flight_enters_abort_land(self):
        machine = self._fly()
        success, _ = machine.request_stop()
        self.assertTrue(success)
        self.assertEqual(machine.state, MissionState.ABORT_LAND)

    def test_reset_rejected_during_flight(self):
        machine = self._fly()
        success, message = machine.request_reset()
        self.assertFalse(success)
        self.assertIn("stop", message)

    def test_confirm_landed_clears_active(self):
        machine = self._fly()
        machine.abort("安全门")
        machine.confirm_landed()
        self.assertFalse(machine.active)

    def test_manual_takeover_can_stop_any_flight_stage(self):
        machine = self._fly()
        machine.confirm_manual_takeover()
        self.assertEqual(machine.state, MissionState.COMPLETE)
        self.assertEqual(machine.result, MissionResult.MANUAL_TAKEOVER)
        self.assertFalse(machine.active)


if __name__ == "__main__":
    unittest.main()

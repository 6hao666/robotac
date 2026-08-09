#!/usr/bin/env python3
"""Read-only check of PX4 external-vision EKF parameters through MAVROS."""

import sys
import math

import rospy
from mavros_msgs.srv import ParamGet


# PX4 v1.10/v1.11 EKF2_AID_MASK: bit 3 is vision position, bit 4 is vision yaw.
LEGACY_AID_MASK_VISION_POSITION = 1 << 3
LEGACY_AID_MASK_VISION_YAW = 1 << 4


def as_bool(value):
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in ("true", "1", "yes", "on"):
            return True
        if normalized in ("false", "0", "no", "off", ""):
            return False
        raise ValueError("invalid Boolean value: %s" % value)
    return bool(value)


def parse_int_list(value):
    if value is None:
        return []
    if isinstance(value, (list, tuple)):
        raw_items = value
    else:
        raw_items = str(value).replace(";", ",").split(",")
    result = []
    for item in raw_items:
        text = str(item).strip()
        if text:
            result.append(int(text, 0))
    return result


def get_param(proxy, name):
    response = proxy(param_id=name)
    if not response.success:
        return None
    if response.value.integer != 0:
        return int(response.value.integer)
    return float(response.value.real)


def main():
    rospy.init_node("check_px4_vision_config", anonymous=True)
    service_name = rospy.get_param("~param_get_service", "/mavros/param/get")
    require_yaw = as_bool(rospy.get_param("~require_yaw_fusion", False))
    require_ev_offsets_zero = as_bool(rospy.get_param("~require_ev_offsets_zero", True))
    ev_offset_tolerance_m = float(rospy.get_param("~ev_offset_tolerance_m", 0.01))
    require_ev_delay = as_bool(rospy.get_param("~require_ev_delay", False))
    expected_ev_delay_ms = float(rospy.get_param("~expected_ev_delay_ms", 0.0))
    ev_delay_tolerance_ms = float(rospy.get_param("~ev_delay_tolerance_ms", 20.0))
    check_offboard_failsafe = as_bool(
        rospy.get_param("~check_px4_offboard_failsafe_params", False))
    min_offboard_loss_timeout_s = float(
        rospy.get_param("~min_offboard_loss_timeout_s", 0.30))
    max_offboard_loss_timeout_s = float(
        rospy.get_param("~max_offboard_loss_timeout_s", 5.00))
    require_offboard_loss_action_param = as_bool(
        rospy.get_param("~require_offboard_loss_action_param", True))
    allowed_offboard_loss_actions = parse_int_list(
        rospy.get_param("~allowed_offboard_loss_actions", ""))
    if not math.isfinite(ev_offset_tolerance_m) or ev_offset_tolerance_m < 0.0:
        print("FAIL: ev_offset_tolerance_m must be finite and non-negative")
        return 3
    if (not math.isfinite(expected_ev_delay_ms) or
            not math.isfinite(ev_delay_tolerance_ms) or ev_delay_tolerance_ms < 0.0):
        print("FAIL: expected_ev_delay_ms and ev_delay_tolerance_ms must be finite; tolerance must be non-negative")
        return 3
    if (not math.isfinite(min_offboard_loss_timeout_s) or
            not math.isfinite(max_offboard_loss_timeout_s) or
            min_offboard_loss_timeout_s < 0.0 or
            max_offboard_loss_timeout_s < min_offboard_loss_timeout_s):
        print("FAIL: offboard loss timeout bounds must be finite, non-negative, and ordered")
        return 3
    if any(action < 0 for action in allowed_offboard_loss_actions):
        print("FAIL: allowed_offboard_loss_actions must contain non-negative integers")
        return 3
    try:
        rospy.wait_for_service(service_name, timeout=5.0)
        get = rospy.ServiceProxy(service_name, ParamGet)
        ev_ctrl = get_param(get, "EKF2_EV_CTRL")
        if ev_ctrl is not None:
            value = int(ev_ctrl)
            required = 0x03 | (0x08 if require_yaw else 0x00)
            print("PX4 EKF2_EV_CTRL=%d required_mask=0x%02x" % (value, required))
            if value & required != required:
                print("FAIL: external-vision horizontal and vertical position fusion is not enabled")
                return 2
        else:
            # Query the legacy parameter only when the current PX4 parameter
            # is genuinely unavailable, avoiding an expected MAVROS warning
            # on recent firmware.
            aid_mask = get_param(get, "EKF2_AID_MASK")
            if aid_mask is None:
                print("FAIL: neither EKF2_EV_CTRL nor EKF2_AID_MASK is available")
                return 2
            value = int(aid_mask)
            required = (LEGACY_AID_MASK_VISION_POSITION |
                        (LEGACY_AID_MASK_VISION_YAW if require_yaw else 0))
            print("PX4 legacy EKF2_AID_MASK=%d required_mask=0x%02x" % (value, required))
            if value & required != required:
                print("FAIL: legacy vision-position fusion is not enabled")
                return 2

        for name in ("EKF2_EV_DELAY", "EKF2_EV_POS_X", "EKF2_EV_POS_Y", "EKF2_EV_POS_Z"):
            value = get_param(get, name)
            print("%s=%s" % (name, "unavailable" if value is None else value))
        if require_ev_delay:
            value = get_param(get, "EKF2_EV_DELAY")
            if value is None:
                print("FAIL: EKF2_EV_DELAY is unavailable; cannot verify external-vision delay")
                return 2
            actual = float(value)
            if not math.isfinite(actual):
                print("FAIL: EKF2_EV_DELAY must be finite")
                return 2
            if abs(actual - expected_ev_delay_ms) > ev_delay_tolerance_ms:
                print("FAIL: EKF2_EV_DELAY %.3f ms is outside expected %.3f +/- %.3f ms" %
                      (actual, expected_ev_delay_ms, ev_delay_tolerance_ms))
                return 2
        if require_ev_offsets_zero:
            offsets = []
            for name in ("EKF2_EV_POS_X", "EKF2_EV_POS_Y", "EKF2_EV_POS_Z"):
                value = get_param(get, name)
                if value is None:
                    print("FAIL: %s is unavailable; cannot verify zero PX4 EV lever arm" % name)
                    return 2
                offsets.append(float(value))
            if any(not math.isfinite(value) for value in offsets):
                print("FAIL: PX4 EV_POS offsets must be finite; offsets=%s" %
                      ",".join(str(value) for value in offsets))
                return 2
            if any(abs(value) > ev_offset_tolerance_m for value in offsets):
                print("FAIL: PX4 EV_POS offsets must be zero when Robotac bridge outputs base_link pose; offsets=%s tolerance=%.3f" %
                      (",".join("%.4f" % value for value in offsets), ev_offset_tolerance_m))
                return 2
        if check_offboard_failsafe:
            loss_timeout = get_param(get, "COM_OF_LOSS_T")
            print("COM_OF_LOSS_T=%s" % ("unavailable" if loss_timeout is None else loss_timeout))
            if loss_timeout is None:
                print("FAIL: COM_OF_LOSS_T is unavailable; cannot verify OFFBOARD loss timeout")
                return 2
            actual_timeout = float(loss_timeout)
            if not math.isfinite(actual_timeout):
                print("FAIL: COM_OF_LOSS_T must be finite")
                return 2
            if actual_timeout < min_offboard_loss_timeout_s or actual_timeout > max_offboard_loss_timeout_s:
                print("FAIL: COM_OF_LOSS_T %.3f s outside %.3f..%.3f s" %
                      (actual_timeout, min_offboard_loss_timeout_s, max_offboard_loss_timeout_s))
                return 2
            action_name = None
            action_value = None
            for name in ("COM_OBL_RC_ACT", "COM_OBL_ACT"):
                value = get_param(get, name)
                print("%s=%s" % (name, "unavailable" if value is None else value))
                if value is not None and action_value is None:
                    action_name = name
                    action_value = int(value)
            if action_value is None and require_offboard_loss_action_param:
                print("FAIL: neither COM_OBL_RC_ACT nor COM_OBL_ACT is available")
                return 2
            if action_value is not None and action_value < 0:
                print("FAIL: %s invalid value %d" % (action_name, action_value))
                return 2
            if (action_value is not None and allowed_offboard_loss_actions and
                    action_value not in allowed_offboard_loss_actions):
                print("FAIL: %s value %d is not in allowed set %s" %
                      (action_name, action_value, allowed_offboard_loss_actions))
                return 2
        print("PASS: PX4 external-vision%s parameters are enabled (read-only check)" %
              (" and OFFBOARD failsafe" if check_offboard_failsafe else " fusion"))
        return 0
    except (rospy.ROSException, rospy.ServiceException) as exc:
        print("FAIL: %s" % exc)
        return 3


if __name__ == "__main__":
    sys.exit(main())

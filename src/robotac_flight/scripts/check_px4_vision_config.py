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
    if not math.isfinite(ev_offset_tolerance_m) or ev_offset_tolerance_m < 0.0:
        print("FAIL: ev_offset_tolerance_m must be finite and non-negative")
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
        if require_ev_offsets_zero:
            offsets = []
            for name in ("EKF2_EV_POS_X", "EKF2_EV_POS_Y", "EKF2_EV_POS_Z"):
                value = get_param(get, name)
                if value is None:
                    print("FAIL: %s is unavailable; cannot verify zero PX4 EV lever arm" % name)
                    return 2
                offsets.append(float(value))
            if any(abs(value) > ev_offset_tolerance_m for value in offsets):
                print("FAIL: PX4 EV_POS offsets must be zero when Robotac bridge outputs base_link pose; offsets=%s tolerance=%.3f" %
                      (",".join("%.4f" % value for value in offsets), ev_offset_tolerance_m))
                return 2
        print("PASS: PX4 external-vision fusion parameters are enabled (read-only check)")
        return 0
    except (rospy.ROSException, rospy.ServiceException) as exc:
        print("FAIL: %s" % exc)
        return 3


if __name__ == "__main__":
    sys.exit(main())

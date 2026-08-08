#!/usr/bin/env python3
"""Preview a Robotac local-flight route without starting ROS.

The flight controller captures the MAVROS local ENU pose at /start, then turns
configured relative waypoints into fixed local ENU setpoints. This helper uses
the same route math offline so an operator can check the intended route before
arming: no ROS master, MAVROS node, FCU connection, setpoints, services, or
hardware access are used.
"""

import argparse
import math
import sys

import yaml


def _finite_float(value, name):
    try:
        number = float(value)
    except (TypeError, ValueError):
        raise ValueError("%s must be numeric" % name)
    if not math.isfinite(number):
        raise ValueError("%s must be finite" % name)
    return number


def _angle_wrap(value):
    return math.atan2(math.sin(value), math.cos(value))


def _load_yaml(path):
    with open(path, "r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if data is None:
        raise ValueError("route YAML is empty")
    if isinstance(data, dict) and "local_waypoint_flight" in data:
        data = data["local_waypoint_flight"]
    if not isinstance(data, dict):
        raise ValueError("route YAML must contain a local_waypoint_flight mapping")
    return data


def _parse_waypoint(item, index, default_hold, default_payload_settle):
    if not isinstance(item, dict):
        raise ValueError("waypoint %d must be a mapping" % index)
    yaw_keys = [key for key in ("yaw", "yaw_deg") if key in item]
    if len(yaw_keys) > 1:
        raise ValueError("waypoint %d must use yaw or yaw_deg, not both" % index)
    if "yaw_deg" in item:
        yaw = math.radians(_finite_float(item["yaw_deg"], "waypoint %d yaw_deg" % index))
    else:
        yaw = _finite_float(item.get("yaw", 0.0), "waypoint %d yaw" % index)
    action = str(item.get("payload_action", "none")).strip().lower()
    return {
        "name": "wp%d" % index,
        "x": _finite_float(item.get("x", 0.0), "waypoint %d x" % index),
        "y": _finite_float(item.get("y", 0.0), "waypoint %d y" % index),
        "z": _finite_float(item.get("z", 0.0), "waypoint %d z" % index),
        "yaw": yaw,
        "hold": _finite_float(item.get("hold", default_hold), "waypoint %d hold" % index),
        "payload_action": action,
        "payload_settle": _finite_float(item.get("payload_settle", default_payload_settle),
                                        "waypoint %d payload_settle" % index),
    }


def _parse_route(path):
    section = _load_yaml(path)
    frame = str(section.get("waypoint_frame", "robotac_start_body")).strip()
    if frame not in ("robotac_start_body", "robotac_local_enu"):
        raise ValueError("unsupported waypoint_frame: %s" % frame)
    raw_waypoints = section.get("waypoints")
    if not isinstance(raw_waypoints, list) or not raw_waypoints:
        raise ValueError("waypoints must be a non-empty list")
    hold = _finite_float(section.get("waypoint_hold_seconds", 2.0), "waypoint_hold_seconds")
    payload_settle = _finite_float(section.get("payload_default_settle_seconds", 2.0),
                                   "payload_default_settle_seconds")
    waypoints = [_parse_waypoint(item, index, hold, payload_settle)
                 for index, item in enumerate(raw_waypoints)]
    return section, frame, waypoints


def _absolute_target(origin, origin_yaw, frame, relative):
    if frame == "robotac_start_body":
        cos_yaw = math.cos(origin_yaw)
        sin_yaw = math.sin(origin_yaw)
        offset_x = cos_yaw * relative["x"] - sin_yaw * relative["y"]
        offset_y = sin_yaw * relative["x"] + cos_yaw * relative["y"]
    else:
        offset_x = relative["x"]
        offset_y = relative["y"]
    return (
        origin[0] + offset_x,
        origin[1] + offset_y,
        origin[2] + relative["z"],
        _angle_wrap(origin_yaw + relative["yaw"]),
    )


def _mavlink_ned_target(enu_target):
    # Mirrors MAVROS SetpointRawPlugin::local_cb for FRAME_LOCAL_NED:
    # ROS ENU position/yaw fields are transformed to MAVLink local NED.
    east, north, up, yaw_enu = enu_target
    return (north, east, -up, _angle_wrap(math.pi / 2.0 - yaw_enu))


def _validate(section, waypoints):
    max_xy = _finite_float(section.get("max_waypoint_xy", 20.0), "max_waypoint_xy")
    max_z = _finite_float(section.get("max_waypoint_z", 5.0), "max_waypoint_z")
    waypoint_timeout = _finite_float(section.get("waypoint_timeout", 45.0), "waypoint_timeout")
    takeoff_height = _finite_float(section.get("takeoff_height", 1.0), "takeoff_height")
    if max_xy <= 0.0 or max_z <= 0.0 or waypoint_timeout <= 0.0 or takeoff_height <= 0.1:
        raise ValueError("route limits and takeoff height must be positive")
    for waypoint in waypoints:
        if math.hypot(waypoint["x"], waypoint["y"]) > max_xy:
            raise ValueError("%s exceeds max_waypoint_xy" % waypoint["name"])
        if waypoint["z"] < -0.1 or waypoint["z"] > max_z:
            raise ValueError("%s z is outside allowed range" % waypoint["name"])
        if waypoint["hold"] < 0.0 or waypoint["hold"] > waypoint_timeout:
            raise ValueError("%s hold is outside allowed range" % waypoint["name"])
        if waypoint["payload_action"] not in ("none", "open", "close"):
            raise ValueError("%s has invalid payload_action" % waypoint["name"])
        if waypoint["payload_settle"] < 0.0 or waypoint["payload_settle"] > 30.0:
            raise ValueError("%s payload_settle is outside allowed range" % waypoint["name"])


def _build_targets(section, frame, waypoints, origin, origin_yaw, include_takeoff):
    targets = []
    if include_takeoff:
        takeoff_height = _finite_float(section.get("takeoff_height", 1.0), "takeoff_height")
        targets.append({
            "name": "takeoff",
            "relative": {"x": 0.0, "y": 0.0, "z": takeoff_height, "yaw": 0.0},
            "hold": _finite_float(section.get("waypoint_hold_seconds", 2.0), "waypoint_hold_seconds"),
            "payload_action": "none",
            "payload_settle": 0.0,
            "target": (origin[0], origin[1], origin[2] + takeoff_height, origin_yaw),
        })
    for waypoint in waypoints:
        targets.append({
            "name": waypoint["name"],
            "relative": waypoint,
            "hold": waypoint["hold"],
            "payload_action": waypoint["payload_action"],
            "payload_settle": waypoint["payload_settle"],
            "target": _absolute_target(origin, origin_yaw, frame, waypoint),
        })
    return targets


def _format_tuple(values):
    return "(" + ",".join("%.3f" % value for value in values) + ")"


def _print_targets(section, frame, targets, origin, origin_yaw):
    print("route_file_frame=%s" % frame)
    print("origin_enu=%s origin_yaw_deg=%.3f" % (_format_tuple(origin), math.degrees(origin_yaw)))
    print("takeoff_height=%.3f waypoint_count=%d auto_land_required=%s" % (
        _finite_float(section.get("takeoff_height", 1.0), "takeoff_height"),
        len([target for target in targets if target["name"].startswith("wp")]),
        section.get("require_auto_land", "unknown")))
    print("index name rel_flu_or_enu target_enu yaw_enu_deg mavlink_ned yaw_ned_deg hold payload settle")
    for index, item in enumerate(targets):
        relative = item["relative"]
        target = item["target"]
        mavlink = _mavlink_ned_target(target)
        print("%02d %s %s %s %.3f %s %.3f %.3f %s %.3f" % (
            index,
            item["name"],
            _format_tuple((relative["x"], relative["y"], relative["z"])),
            _format_tuple(target[:3]),
            math.degrees(target[3]),
            _format_tuple(mavlink[:3]),
            math.degrees(mavlink[3]),
            item["hold"],
            item["payload_action"],
            item["payload_settle"],
        ))
    print("target_enu_route=%s" % "->".join(_format_tuple(item["target"][:3]) for item in targets))
    print("mavlink_ned_route=%s" % "->".join(_format_tuple(_mavlink_ned_target(item["target"])[:3])
                                             for item in targets))
    payload_events = ["%s:%s@%s" % (item["name"], item["payload_action"],
                                     _format_tuple(item["target"][:3]))
                      for item in targets if item["payload_action"] != "none"]
    print("payload_events=%s" % ("none" if not payload_events else ",".join(payload_events)))


def _build_parser():
    parser = argparse.ArgumentParser(
        description="Preview Robotac local waypoint targets without ROS or hardware access.")
    parser.add_argument("--file", required=True, help="YAML file with local_waypoint_flight route")
    parser.add_argument("--origin-x", type=float, default=0.0)
    parser.add_argument("--origin-y", type=float, default=0.0)
    parser.add_argument("--origin-z", type=float, default=0.0)
    parser.add_argument("--origin-yaw", type=float, default=None,
                        help="Captured start yaw in radians")
    parser.add_argument("--origin-yaw-deg", type=float, default=0.0,
                        help="Captured start yaw in degrees; ignored if --origin-yaw is set")
    parser.add_argument("--no-takeoff", action="store_true",
                        help="Do not prepend the separate takeoff target")
    return parser


def main():
    args = _build_parser().parse_args()
    origin = (
        _finite_float(args.origin_x, "origin_x"),
        _finite_float(args.origin_y, "origin_y"),
        _finite_float(args.origin_z, "origin_z"),
    )
    if args.origin_yaw is None:
        origin_yaw = math.radians(_finite_float(args.origin_yaw_deg, "origin_yaw_deg"))
    else:
        origin_yaw = _finite_float(args.origin_yaw, "origin_yaw")
    section, frame, waypoints = _parse_route(args.file)
    _validate(section, waypoints)
    targets = _build_targets(section, frame, waypoints, origin, origin_yaw,
                             include_takeoff=not args.no_takeoff)
    _print_targets(section, frame, targets, origin, origin_yaw)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        print("ERROR: %s" % exc, file=sys.stderr)
        sys.exit(1)

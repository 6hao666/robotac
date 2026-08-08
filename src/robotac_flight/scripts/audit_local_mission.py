#!/usr/bin/env python3
"""Offline audit for Robotac local waypoint missions.

This script validates the mission file used by ``local_waypoint_flight.py`` and
summarizes the route contract before an operator goes near MAVROS, PX4, or the
aircraft. It starts no ROS node, publishes no setpoints, opens no serial device,
and calls no services.
"""

import argparse
import json
import math
import pathlib
import sys

import yaml

import preview_local_route as preview


FORBIDDEN_GLOBAL_KEYS = {
    "lat",
    "lon",
    "latitude",
    "longitude",
    "altitude_amsl",
    "altitude_wgs84",
    "global_frame",
    "global_position",
    "gps",
    "gps_fix",
    "frame_global",
    "command_tol",
    "commandtol",
    "mission_item",
    "mission_items",
}


def _load_yaml(path):
    with open(path, "r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if data is None:
        raise ValueError("mission YAML is empty")
    return data


def _as_bool(value, name):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in ("true", "1", "yes", "on"):
            return True
        if normalized in ("false", "0", "no", "off", ""):
            return False
    raise ValueError("%s must be a boolean" % name)


def _global_key_violations(value, prefix=""):
    violations = []
    if isinstance(value, dict):
        for key, child in value.items():
            key_text = str(key)
            key_path = "%s.%s" % (prefix, key_text) if prefix else key_text
            if key_text.strip().lower() in FORBIDDEN_GLOBAL_KEYS:
                violations.append(key_path)
            violations.extend(_global_key_violations(child, key_path))
    elif isinstance(value, list):
        for index, child in enumerate(value):
            key_path = "%s[%d]" % (prefix, index) if prefix else "[%d]" % index
            violations.extend(_global_key_violations(child, key_path))
    return violations


def _distance(a, b):
    return math.sqrt(sum((float(a[index]) - float(b[index])) ** 2 for index in range(3)))


def _format_tuple(values):
    return "(" + ",".join("%.3f" % float(value) for value in values) + ")"


def _build_summary(args):
    data = _load_yaml(args.file)
    violations = _global_key_violations(data)
    if violations:
        raise ValueError("global/GPS mission keys are not allowed: %s" % ",".join(violations))

    section, frame, waypoints = preview._parse_route(args.file)
    preview._validate(section, waypoints)
    origin_yaw = (preview._finite_float(args.origin_yaw, "origin_yaw")
                  if args.origin_yaw is not None else
                  math.radians(preview._finite_float(args.origin_yaw_deg, "origin_yaw_deg")))
    origin = (
        preview._finite_float(args.origin_x, "origin_x"),
        preview._finite_float(args.origin_y, "origin_y"),
        preview._finite_float(args.origin_z, "origin_z"),
    )
    targets = preview._build_targets(section, frame, waypoints, origin, origin_yaw,
                                     include_takeoff=not args.no_takeoff)

    auto_land_required = _as_bool(section.get("require_auto_land", False), "require_auto_land")
    if args.require_auto_land and not auto_land_required:
        raise ValueError("require_auto_land must be true for the staged flight mission")

    payload_events = [
        {
            "name": item["name"],
            "action": item["payload_action"],
            "target_enu": item["target"][:3],
        }
        for item in targets if item["payload_action"] != "none"
    ]
    if args.require_payload_open and not any(event["action"] == "open" for event in payload_events):
        raise ValueError("payload open action is required but absent")

    target_positions = [target["target"][:3] for target in targets]
    path_distance = sum(_distance(a, b) for a, b in zip(target_positions, target_positions[1:]))
    max_horizontal_radius = max(
        math.hypot(position[0] - origin[0], position[1] - origin[1])
        for position in target_positions)
    max_relative_altitude = max(position[2] - origin[2] for position in target_positions)
    min_relative_altitude = min(position[2] - origin[2] for position in target_positions)
    mavlink_targets = [preview._mavlink_ned_target(target["target"]) for target in targets]

    return {
        "file": str(pathlib.Path(args.file)),
        "frame": frame,
        "origin_enu": origin,
        "origin_yaw_deg": math.degrees(origin_yaw),
        "waypoints": len(waypoints),
        "targets_with_takeoff": len(targets),
        "takeoff_height_m": preview._finite_float(section.get("takeoff_height", 1.0), "takeoff_height"),
        "auto_land_required": auto_land_required,
        "land_mode": str(section.get("land_mode", "AUTO.LAND")),
        "path_distance_m": path_distance,
        "max_horizontal_radius_m": max_horizontal_radius,
        "min_relative_altitude_m": min_relative_altitude,
        "max_relative_altitude_m": max_relative_altitude,
        "first_target_enu": target_positions[0],
        "last_target_enu": target_positions[-1],
        "payload_events": payload_events,
        "target_enu_route": target_positions,
        "mavlink_ned_route": [target[:3] for target in mavlink_targets],
        "local_contract": "robotac_start_body/robotac_local_enu -> ROS ENU -> MAVROS setpoint_raw/local",
    }


def _print_text(summary):
    payload_events = [
        "%s:%s@%s" % (event["name"], event["action"], _format_tuple(event["target_enu"]))
        for event in summary["payload_events"]
    ]
    print("MISSION_AUDIT_PASS file=%s" % summary["file"])
    print("frame=%s waypoints=%d targets_with_takeoff=%d" % (
        summary["frame"], summary["waypoints"], summary["targets_with_takeoff"]))
    print("origin_enu=%s origin_yaw_deg=%.3f" % (
        _format_tuple(summary["origin_enu"]), summary["origin_yaw_deg"]))
    print("takeoff_height_m=%.3f auto_land_required=%s land_mode=%s" % (
        summary["takeoff_height_m"], summary["auto_land_required"], summary["land_mode"]))
    print("path_distance_m=%.3f max_horizontal_radius_m=%.3f altitude_range_m=%.3f..%.3f" % (
        summary["path_distance_m"], summary["max_horizontal_radius_m"],
        summary["min_relative_altitude_m"], summary["max_relative_altitude_m"]))
    print("first_target_enu=%s last_target_enu=%s" % (
        _format_tuple(summary["first_target_enu"]), _format_tuple(summary["last_target_enu"])))
    print("payload_events=%s" % ("none" if not payload_events else ",".join(payload_events)))
    print("local_contract=%s" % summary["local_contract"])
    print("global_coordinates=forbidden")


def _build_parser():
    parser = argparse.ArgumentParser(
        description="Audit a Robotac local waypoint mission without ROS or hardware access.")
    parser.add_argument("--file", required=True, help="YAML file with local_waypoint_flight route")
    parser.add_argument("--origin-x", type=float, default=0.0)
    parser.add_argument("--origin-y", type=float, default=0.0)
    parser.add_argument("--origin-z", type=float, default=0.0)
    parser.add_argument("--origin-yaw", type=float, default=None,
                        help="Captured start yaw in radians")
    parser.add_argument("--origin-yaw-deg", type=float, default=0.0,
                        help="Captured start yaw in degrees; ignored if --origin-yaw is set")
    parser.add_argument("--no-takeoff", action="store_true",
                        help="Do not include the separate takeoff target in route metrics")
    parser.add_argument("--require-auto-land", action="store_true", default=True,
                        help="Require require_auto_land: true in the mission file")
    parser.add_argument("--allow-no-auto-land", dest="require_auto_land", action="store_false",
                        help="Allow a position-only or hover-ended route audit")
    parser.add_argument("--require-payload-open", action="store_true",
                        help="Require at least one payload_action: open waypoint")
    parser.add_argument("--json", action="store_true", help="Print machine-readable JSON")
    return parser


def main():
    args = _build_parser().parse_args()
    summary = _build_summary(args)
    if args.json:
        print(json.dumps(summary, indent=2, sort_keys=True))
    else:
        _print_text(summary)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        print("ERROR: %s" % exc, file=sys.stderr)
        sys.exit(1)

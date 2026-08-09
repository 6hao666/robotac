#!/usr/bin/env python3
"""Create a full Robotac local-flight route file from simple local waypoints.

This is an offline helper for field preparation.  It starts no ROS node, opens
no serial/network device, publishes no setpoints, calls no MAVROS services, and
does not arm or change mode.  Its only side effect is writing the requested YAML
file, or printing the YAML when ``--dry-run`` / ``--output -`` is used.
"""

import argparse
import json
import math
import pathlib
import sys

import yaml


FORBIDDEN_GLOBAL_KEYS = {
    "lat",
    "lon",
    "latitude",
    "longitude",
    "altitude",
    "altitude_amsl",
    "altitude_wgs84",
    "global",
    "global_frame",
    "global_position",
    "gps",
    "gps_fix",
    "frame_global",
    "command_tol",
    "commandtol",
    "mission",
    "mission_item",
    "mission_items",
}


BASE_ROUTE_DEFAULTS = {
    "waypoint_frame": "robotac_start_body",
    "takeoff_height": 1.0,
    "require_auto_land": True,
    "waypoints": [],
    "position_tolerance": 0.25,
    "yaw_tolerance_deg": 12.0,
    "waypoint_timeout": 45.0,
    "mission_timeout": 600.0,
    "control_rate_hz": 20.0,
    "prestream_seconds": 5.0,
    "require_vision": True,
    "require_vision_output": True,
    "require_estimator_status": True,
    "require_horizontal_relative": True,
    "require_vertical_estimate": True,
    "max_waypoint_xy": 20.0,
    "max_waypoint_z": 5.0,
    "local_pose_timeout": 0.30,
    "local_stamp_timeout": 0.50,
    "local_stamp_future_tolerance": 0.10,
    "strict_local_frames": True,
    "expected_local_parent": "map",
    "expected_local_child": "base_link",
    "max_local_position_speed": 6.0,
    "max_local_yaw_rate": 6.0,
    "state_timeout": 1.0,
    "extended_state_timeout": 1.0,
    "vision_timeout": 0.50,
    "vision_status_timeout": 0.50,
    "vision_output_timeout": 0.50,
    "vision_output_stamp_timeout": 0.30,
    "vision_output_stamp_future_tolerance": 0.10,
    "vision_output_parent": "odom",
    "require_vision_output_consumer": True,
    "vision_output_consumer_node": "/mavros",
    "setpoint_topic": "/mavros/setpoint_raw/local",
    "require_setpoint_consumer": True,
    "setpoint_consumer_node": "/mavros",
    "consumer_check_interval": 0.50,
    "require_timesync": True,
    "timesync_timeout": 1.0,
    "max_timesync_rtt_ms": 20.0,
    "estimator_timeout": 1.0,
    "offboard_timeout": 15.0,
    "arming_timeout": 15.0,
    "takeoff_timeout": 45.0,
    "critical_fault_action": "release",
    "operator_abort_action": "release",
    "abort_land_mode_timeout": 8.0,
    "auto_land_mode_timeout": 8.0,
    "landing_timeout": 60.0,
    "land_mode": "AUTO.LAND",
    "land_descent_rate": 0.30,
    "land_switch_height": 0.50,
}


PAYLOAD_ROUTE_DEFAULTS = {
    "payload_topic": "/robotac_servo/control",
    "payload_status_topic": "/robotac_servo/status",
    "payload_required_connection": True,
    "payload_require_ack": True,
    "payload_ack_timeout": 1.0,
    "payload_preflight_close": True,
    "payload_default_settle_seconds": 2.0,
}


LOCAL_SECTION_KEYS = set(BASE_ROUTE_DEFAULTS) | set(PAYLOAD_ROUTE_DEFAULTS) | {
    "waypoint_hold_seconds",
}


def _finite_float(value, name):
    try:
        number = float(value)
    except (TypeError, ValueError):
        raise ValueError("%s must be numeric" % name)
    if not math.isfinite(number):
        raise ValueError("%s must be finite" % name)
    return number


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


def _load_input(path):
    input_path = pathlib.Path(path).expanduser()
    text = input_path.read_text(encoding="utf-8")
    if input_path.suffix.lower() == ".json":
        data = json.loads(text)
    else:
        data = yaml.safe_load(text)
    if data is None:
        raise ValueError("input waypoint file is empty")
    return data


def _section_and_waypoints(data):
    if isinstance(data, list):
        return {}, data
    if not isinstance(data, dict):
        raise ValueError("input must be a waypoint list or a mapping")
    if "local_waypoint_flight" in data:
        section = data["local_waypoint_flight"]
        if not isinstance(section, dict):
            raise ValueError("local_waypoint_flight must be a mapping")
    else:
        section = data
    if "waypoints" in section:
        waypoints = section["waypoints"]
    elif "points" in section:
        waypoints = section["points"]
    else:
        raise ValueError("input mapping must contain waypoints or points")
    if not isinstance(waypoints, list) or not waypoints:
        raise ValueError("waypoints must be a non-empty list")
    return section, waypoints


def _parse_point_text(text, index):
    fields = [item.strip() for item in text.split(",")]
    if len(fields) not in (3, 4):
        raise ValueError("--point %d must be x,y,z or x,y,z,yaw_deg" % index)
    result = {
        "x": _finite_float(fields[0], "point %d x" % index),
        "y": _finite_float(fields[1], "point %d y" % index),
        "z": _finite_float(fields[2], "point %d z" % index),
    }
    if len(fields) == 4:
        result["yaw_deg"] = _finite_float(fields[3], "point %d yaw_deg" % index)
    return result


def _raw_waypoints_from_args(args):
    if args.input and args.point:
        raise ValueError("use either --input or repeated --point, not both")
    if args.input:
        data = _load_input(args.input)
        violations = _global_key_violations(data)
        if violations:
            raise ValueError("global/GPS mission keys are not allowed: %s" % ",".join(violations))
        return _section_and_waypoints(data)
    if args.point:
        return {}, [_parse_point_text(text, index) for index, text in enumerate(args.point)]
    raise ValueError("provide --input or at least one --point")


def _yaw_degrees_from_mapping(item, index, default_yaw_deg):
    yaw_keys = [key for key in ("yaw", "yaw_deg") if key in item]
    if len(yaw_keys) > 1:
        raise ValueError("waypoint %d must use yaw or yaw_deg, not both" % index)
    if "yaw" in item:
        return math.degrees(_finite_float(item["yaw"], "waypoint %d yaw" % index))
    return _finite_float(item.get("yaw_deg", default_yaw_deg), "waypoint %d yaw_deg" % index)


def _parse_waypoint(item, index, default_hold, default_yaw_deg, default_payload_settle):
    if isinstance(item, (list, tuple)):
        if len(item) not in (3, 4):
            raise ValueError("waypoint %d list must be [x, y, z] or [x, y, z, yaw_deg]" % index)
        item = {
            "x": item[0],
            "y": item[1],
            "z": item[2],
            "yaw_deg": item[3] if len(item) == 4 else default_yaw_deg,
        }
    if not isinstance(item, dict):
        raise ValueError("waypoint %d must be a mapping or list" % index)
    for key in ("x", "y", "z"):
        if key not in item:
            raise ValueError("waypoint %d missing %s" % (index, key))
    action = str(item.get("payload_action", "none")).strip().lower()
    if action not in ("none", "open", "close"):
        raise ValueError("waypoint %d payload_action must be none/open/close" % index)
    waypoint = {
        "x": _finite_float(item["x"], "waypoint %d x" % index),
        "y": _finite_float(item["y"], "waypoint %d y" % index),
        "z": _finite_float(item["z"], "waypoint %d z" % index),
        "yaw_deg": _yaw_degrees_from_mapping(item, index, default_yaw_deg),
        "hold": _finite_float(item.get("hold", default_hold), "waypoint %d hold" % index),
    }
    if action != "none":
        waypoint["payload_action"] = action
        waypoint["payload_settle"] = _finite_float(
            item.get("payload_settle", default_payload_settle),
            "waypoint %d payload_settle" % index)
    return waypoint


def _apply_payload_index(waypoints, args, default_payload_settle):
    if args.payload_open_last and args.payload_open_index is not None:
        raise ValueError("use either --payload-open-last or --payload-open-index, not both")
    if args.payload_open_last:
        index = len(waypoints) - 1
    else:
        index = args.payload_open_index
    if index is None:
        return
    if index < 0 or index >= len(waypoints):
        raise ValueError("payload-open index %d is outside waypoint range 0..%d" % (
            index, len(waypoints) - 1))
    waypoints[index]["payload_action"] = "open"
    waypoints[index]["payload_settle"] = _finite_float(
        default_payload_settle if args.payload_settle is None else args.payload_settle,
        "payload_settle")


def _append_return_home(waypoints, takeoff_height, default_hold, default_yaw_deg):
    last = waypoints[-1]
    if (abs(float(last["x"])) <= 1.0e-9 and
            abs(float(last["y"])) <= 1.0e-9 and
            abs(float(last["z"]) - float(takeoff_height)) <= 1.0e-9):
        return
    waypoints.append({
        "x": 0.0,
        "y": 0.0,
        "z": float(takeoff_height),
        "yaw_deg": float(default_yaw_deg),
        "hold": float(default_hold),
    })


def _validate_route(section):
    frame = str(section.get("waypoint_frame", "")).strip()
    if frame not in ("robotac_start_body", "robotac_local_enu"):
        raise ValueError("waypoint_frame must be robotac_start_body or robotac_local_enu")
    takeoff_height = _finite_float(section.get("takeoff_height"), "takeoff_height")
    if takeoff_height <= 0.1:
        raise ValueError("takeoff_height must be greater than 0.1 m")
    max_xy = _finite_float(section.get("max_waypoint_xy"), "max_waypoint_xy")
    max_z = _finite_float(section.get("max_waypoint_z"), "max_waypoint_z")
    if max_xy <= 0.0 or max_z <= 0.0:
        raise ValueError("max_waypoint_xy and max_waypoint_z must be positive")
    if section.get("require_auto_land") is not True:
        raise ValueError("generated route must keep require_auto_land: true")
    for index, waypoint in enumerate(section["waypoints"]):
        radius = math.hypot(float(waypoint["x"]), float(waypoint["y"]))
        if radius > max_xy:
            raise ValueError("waypoint %d exceeds max_waypoint_xy" % index)
        z_value = float(waypoint["z"])
        if z_value < -0.1 or z_value > max_z:
            raise ValueError("waypoint %d z is outside allowed range" % index)
        if float(waypoint["hold"]) < 0.0:
            raise ValueError("waypoint %d hold must be non-negative" % index)
        if waypoint.get("payload_action", "none") not in ("none", "open", "close"):
            raise ValueError("waypoint %d payload_action must be none/open/close" % index)


def _build_route(args):
    input_section, raw_waypoints = _raw_waypoints_from_args(args)
    route = dict(BASE_ROUTE_DEFAULTS)
    for key in LOCAL_SECTION_KEYS:
        if key in input_section and key != "waypoints":
            route[key] = input_section[key]

    if args.frame is not None:
        route["waypoint_frame"] = args.frame
    if args.takeoff_height is not None:
        route["takeoff_height"] = _finite_float(args.takeoff_height, "takeoff_height")
    if args.max_waypoint_xy is not None:
        route["max_waypoint_xy"] = _finite_float(args.max_waypoint_xy, "max_waypoint_xy")
    if args.max_waypoint_z is not None:
        route["max_waypoint_z"] = _finite_float(args.max_waypoint_z, "max_waypoint_z")

    default_hold = _finite_float(
        route.get("waypoint_hold_seconds", 2.0) if args.hold is None else args.hold,
        "hold")
    default_yaw_deg = _finite_float(0.0 if args.yaw_deg is None else args.yaw_deg, "yaw_deg")
    default_payload_settle = _finite_float(
        route.get("payload_default_settle_seconds", 2.0)
        if args.payload_settle is None else args.payload_settle,
        "payload_settle")
    waypoints = [
        _parse_waypoint(item, index, default_hold, default_yaw_deg, default_payload_settle)
        for index, item in enumerate(raw_waypoints)
    ]
    _apply_payload_index(waypoints, args, default_payload_settle)
    if args.append_return_home:
        _append_return_home(waypoints, route["takeoff_height"], default_hold, default_yaw_deg)

    has_payload = any(item.get("payload_action", "none") != "none" for item in waypoints)
    if has_payload:
        for key, value in PAYLOAD_ROUTE_DEFAULTS.items():
            route.setdefault(key, value)
    route["waypoints"] = waypoints
    _validate_route(route)
    output = {"local_waypoint_flight": route}
    violations = _global_key_violations(output)
    if violations:
        raise ValueError("generated route contains forbidden global/GPS keys: %s" % ",".join(violations))
    return output


def _dump_yaml(data):
    return yaml.safe_dump(data, default_flow_style=False, sort_keys=False, allow_unicode=True)


def _write_output(args, text):
    if args.dry_run or args.output == "-":
        sys.stdout.write(text)
        return
    if not args.output:
        raise ValueError("--output is required unless --dry-run is used")
    path = pathlib.Path(args.output).expanduser()
    if path.exists() and not args.force:
        raise ValueError("output already exists; pass --force to overwrite: %s" % path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")
    print("WROTE_ROUTE_FILE %s" % path)


def _build_parser():
    parser = argparse.ArgumentParser(
        description="Generate a full Robotac local MAVROS route YAML without ROS or hardware access.")
    source = parser.add_argument_group("waypoint source")
    source.add_argument("--input", default="",
                        help="YAML/JSON with waypoints, points, or local_waypoint_flight.waypoints")
    source.add_argument("--point", action="append", default=[],
                        help="Repeatable local point x,y,z[,yaw_deg], e.g. --point 1,0,1,0")

    output = parser.add_argument_group("output")
    output.add_argument("--output", default="",
                        help="Generated YAML path; use '-' for stdout")
    output.add_argument("--force", action="store_true", help="Overwrite an existing output file")
    output.add_argument("--dry-run", action="store_true", help="Print generated YAML and write no file")

    route = parser.add_argument_group("route defaults")
    route.add_argument("--frame", default=None,
                       choices=("robotac_start_body", "robotac_local_enu"))
    route.add_argument("--takeoff-height", type=float, default=None)
    route.add_argument("--hold", type=float, default=None,
                       help="Default per-waypoint hold time in seconds")
    route.add_argument("--yaw-deg", type=float, default=None,
                       help="Default yaw for points without yaw/yaw_deg")
    route.add_argument("--max-waypoint-xy", type=float, default=None)
    route.add_argument("--max-waypoint-z", type=float, default=None)
    route.add_argument("--append-return-home", action="store_true",
                       help="Append 0,0,takeoff_height if the route does not already end there")

    payload = parser.add_argument_group("payload")
    payload.add_argument("--payload-open-index", type=int, default=None,
                         help="Zero-based waypoint index that should open the payload servo")
    payload.add_argument("--payload-open-last", action="store_true",
                         help="Mark the last waypoint as payload_action: open")
    payload.add_argument("--payload-settle", type=float, default=None,
                         help="Payload settle seconds when adding an open action")
    return parser


def main():
    args = _build_parser().parse_args()
    route = _build_route(args)
    _write_output(args, _dump_yaml(route))
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        print("ERROR: %s" % exc, file=sys.stderr)
        sys.exit(1)

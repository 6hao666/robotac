#!/usr/bin/env python3
"""Publish a position-only local waypoint list to the flight controller.

This helper is intentionally passive with respect to the aircraft. It publishes
only ``/robotac/flight/waypoints`` and never calls ``/robotac/flight/start``,
MAVROS services, arming, mode changes, or setpoint topics.
"""

import argparse
import math
import sys
import time

import yaml


ALLOWED_WAYPOINT_KEYS = {"x", "y", "z", "yaw", "yaw_deg"}
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


def _finite_float(value, name):
    try:
        number = float(value)
    except (TypeError, ValueError):
        raise ValueError("%s must be numeric" % name)
    if not math.isfinite(number):
        raise ValueError("%s must be finite" % name)
    return number


def _load_yaml(path):
    with open(path, "r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if data is None:
        raise ValueError("waypoint YAML is empty")
    return data


def _route_section(data):
    if isinstance(data, dict) and "local_waypoint_flight" in data:
        section = data["local_waypoint_flight"]
    else:
        section = data
    if isinstance(section, list):
        section = {"waypoints": section}
    if not isinstance(section, dict):
        raise ValueError("waypoint YAML must be a mapping or list")
    return section


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


def parse_waypoint_file(path, frame_override=None, allow_metadata_drop=False):
    data = _load_yaml(path)
    violations = _global_key_violations(data)
    if violations:
        raise ValueError("global/GPS waypoint keys are not allowed: %s" % ",".join(violations))
    section = _route_section(data)
    frame_id = frame_override or str(section.get("waypoint_frame", "robotac_start_body"))
    frame_id = frame_id.strip()
    if not frame_id:
        raise ValueError("waypoint frame must be non-empty")
    if frame_id not in ("robotac_start_body", "robotac_local_enu"):
        raise ValueError("unsupported waypoint frame: %s" % frame_id)
    raw_waypoints = section.get("waypoints")
    if not isinstance(raw_waypoints, list) or not raw_waypoints:
        raise ValueError("waypoints must be a non-empty list")

    waypoints = []
    for index, item in enumerate(raw_waypoints):
        if not isinstance(item, dict):
            raise ValueError("waypoint %d must be a mapping" % index)
        extra_keys = set(item) - ALLOWED_WAYPOINT_KEYS
        if extra_keys and not allow_metadata_drop:
            raise ValueError(
                "waypoint %d contains fields that PoseArray cannot carry: %s" %
                (index, ",".join(sorted(extra_keys))))
        yaw_keys = [key for key in ("yaw", "yaw_deg") if key in item]
        if len(yaw_keys) > 1:
            raise ValueError("waypoint %d must use yaw or yaw_deg, not both" % index)
        yaw = _finite_float(item.get("yaw", 0.0), "waypoint %d yaw" % index)
        if "yaw_deg" in item:
            yaw = math.radians(_finite_float(item["yaw_deg"], "waypoint %d yaw_deg" % index))
        waypoints.append({
            "x": _finite_float(item.get("x", 0.0), "waypoint %d x" % index),
            "y": _finite_float(item.get("y", 0.0), "waypoint %d y" % index),
            "z": _finite_float(item.get("z", 0.0), "waypoint %d z" % index),
            "yaw": yaw,
        })
    return frame_id, waypoints


def _print_route(frame_id, waypoints):
    print("frame_id=%s waypoints=%d" % (frame_id, len(waypoints)))
    for index, waypoint in enumerate(waypoints):
        print("%02d x=%.3f y=%.3f z=%.3f yaw_deg=%.1f" % (
            index, waypoint["x"], waypoint["y"], waypoint["z"],
            math.degrees(waypoint["yaw"])))


def _publish_route(args, frame_id, waypoints):
    import rospy
    from geometry_msgs.msg import Pose, PoseArray
    from tf.transformations import quaternion_from_euler

    rospy.init_node("robotac_waypoint_file_publisher", anonymous=True)
    publisher = rospy.Publisher(args.topic, PoseArray, queue_size=1, latch=True)
    deadline = time.monotonic() + args.wait_seconds
    rate = rospy.Rate(20)
    while not rospy.is_shutdown() and publisher.get_num_connections() < 1:
        if time.monotonic() >= deadline:
            raise RuntimeError("no waypoint-controller subscriber on %s" % args.topic)
        rate.sleep()

    message = PoseArray()
    message.header.frame_id = frame_id
    for waypoint in waypoints:
        pose = Pose()
        pose.position.x = waypoint["x"]
        pose.position.y = waypoint["y"]
        pose.position.z = waypoint["z"]
        quaternion = quaternion_from_euler(0.0, 0.0, waypoint["yaw"])
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]
        message.poses.append(pose)

    publish_rate = rospy.Rate(1.0 / args.period)
    for _ in range(args.publish_count):
        if rospy.is_shutdown():
            break
        message.header.stamp = rospy.Time.now()
        publisher.publish(message)
        publish_rate.sleep()
    print("Published %d %s waypoints to %s" % (len(waypoints), frame_id, args.topic))


def _build_parser():
    parser = argparse.ArgumentParser(
        description="Publish a local position-only waypoint YAML as geometry_msgs/PoseArray.")
    parser.add_argument("--file", required=True, help="YAML file containing waypoint_frame and waypoints")
    parser.add_argument("--topic", default="/robotac/flight/waypoints")
    parser.add_argument("--frame-id", default=None,
                        help="Override YAML frame: robotac_start_body or robotac_local_enu")
    parser.add_argument("--publish-count", type=int, default=5)
    parser.add_argument("--period", type=float, default=0.10,
                        help="Seconds between repeated latched publishes")
    parser.add_argument("--wait-seconds", type=float, default=5.0,
                        help="Wait for the flight controller subscriber")
    parser.add_argument("--allow-metadata-drop", action="store_true",
                        help="Ignore YAML fields PoseArray cannot carry, such as hold or payload_action")
    parser.add_argument("--dry-run", action="store_true",
                        help="Validate and print the route without starting ROS")
    return parser


def main():
    args = _build_parser().parse_args()
    if args.publish_count < 1:
        raise ValueError("publish-count must be positive")
    if args.period <= 0.0 or args.wait_seconds <= 0.0:
        raise ValueError("period and wait-seconds must be positive")
    frame_id, waypoints = parse_waypoint_file(
        args.file, frame_override=args.frame_id,
        allow_metadata_drop=args.allow_metadata_drop)
    if args.dry_run:
        _print_route(frame_id, waypoints)
    else:
        _publish_route(args, frame_id, waypoints)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        print("ERROR: %s" % exc, file=sys.stderr)
        sys.exit(1)

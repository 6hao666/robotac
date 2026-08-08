#!/usr/bin/env python3
"""Offline readiness report for Robotac local MAVROS flight.

The report maps the project configuration to the staged evidence required for:
FAST-LIO -> MAVROS vision output, active local waypoint flight, and payload
flight. It reads only YAML files; it never starts ROS, opens an FCU link,
publishes setpoints, changes modes, arms, or calls services.
"""

import argparse
import json
import pathlib
import sys
from types import SimpleNamespace

import yaml

import audit_local_mission


VISION_GATES = (
    "lidar_network_configured",
    "lidar_imu_extrinsics_calibrated",
    "lidar_imu_time_checked",
    "stable_fcu_device_configured",
    "fastlio_airframe_extrinsics_validated",
    "fastlio_axes_validated",
    "px4_external_vision_configured",
)

FLIGHT_GATES = VISION_GATES + (
    "px4_offboard_failsafe_configured",
    "local_flight_ground_tested",
)

PAYLOAD_GATES = FLIGHT_GATES + (
    "stable_servo_device_configured",
)

MAVROS_REQUIRED_WHITELIST = {
    "command",
    "imu",
    "local_position",
    "param",
    "setpoint_raw",
    "sys_status",
    "sys_time",
    "vision_pose_estimate",
}

MAVROS_REQUIRED_BLACKLIST = {
    "global_position",
    "gps_status",
    "waypoint",
}


def _load_yaml(path):
    with open(path, "r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    if not isinstance(data, dict):
        raise ValueError("%s must be a YAML mapping" % path)
    return data


def _phase(name, ready, missing=None, notes=None):
    return {
        "name": name,
        "ready": bool(ready),
        "missing": list(missing or []),
        "notes": list(notes or []),
    }


def _missing(deployment, keys):
    return [key for key in keys if deployment.get(key) is not True]


def _plugin_report(config_root):
    path = config_root / "mavros" / "px4_pluginlists.yaml"
    data = _load_yaml(path)
    whitelist = set(data.get("plugin_whitelist") or [])
    blacklist = set(data.get("plugin_blacklist") or [])
    missing = []
    notes = []
    for name in sorted(MAVROS_REQUIRED_WHITELIST):
        if name not in whitelist:
            missing.append("mavros_plugin_whitelist:%s" % name)
    for name in sorted(MAVROS_REQUIRED_BLACKLIST):
        if name in whitelist:
            missing.append("mavros_plugin_not_local_only:%s" % name)
        if name not in blacklist:
            missing.append("mavros_plugin_blacklist:%s" % name)
    if not missing:
        notes.append("local-only MAVROS plugins configured")
    return _phase("mavros_local_only", not missing, missing, notes)


def _mission_report(config_root, args):
    route_file = (pathlib.Path(args.route_file).expanduser().resolve()
                  if args.route_file else config_root / "flight" / "local_waypoints.yaml")
    mission_args = SimpleNamespace(
        file=str(route_file),
        origin_x=args.origin_x,
        origin_y=args.origin_y,
        origin_z=args.origin_z,
        origin_yaw=args.origin_yaw,
        origin_yaw_deg=args.origin_yaw_deg,
        no_takeoff=False,
        require_auto_land=True,
        require_payload_open=False,
        json=False,
    )
    summary = audit_local_mission._build_summary(mission_args)
    notes = [
        "waypoints=%d" % summary["waypoints"],
        "targets_with_takeoff=%d" % summary["targets_with_takeoff"],
        "path_distance_m=%.3f" % summary["path_distance_m"],
        "max_radius_m=%.3f" % summary["max_horizontal_radius_m"],
    ]
    if summary["payload_events"]:
        notes.append("payload_events=%d" % len(summary["payload_events"]))
    return _phase("local_mission_file", True, [], notes), summary


def _vision_bridge_report(config_root):
    path = config_root / "fastlio" / "vision_bridge.yaml"
    data = _load_yaml(path).get("fastlio_vision_bridge", {})
    if not isinstance(data, dict):
        raise ValueError("fastlio/vision_bridge.yaml must contain fastlio_vision_bridge mapping")
    missing = []
    notes = []
    if data.get("frame_alignment_approved") is not True:
        missing.append("frame_alignment_approved")
    for key, expected in (
            ("expected_input_parent", "camera_init"),
            ("expected_input_child", "body"),
            ("output_parent_frame", "odom")):
        if data.get(key) != expected:
            missing.append("vision_bridge_%s" % key)
    if data.get("strict_input_frames") is not True:
        missing.append("vision_bridge_strict_input_frames")
    if data.get("frame_alignment_approved") is True:
        notes.append("FAST-LIO frame alignment approved")
    else:
        notes.append("FAST-LIO frame alignment still requires aircraft evidence")
    return _phase("fastlio_vision_bridge_config", not missing, missing, notes)


def build_report(args):
    config_root = pathlib.Path(args.config_root).expanduser().resolve()
    deployment_data = _load_yaml(config_root / "deployment.yaml")
    deployment = deployment_data.get("deployment", {})
    if not isinstance(deployment, dict):
        raise ValueError("deployment.yaml must contain a deployment mapping")

    phases = []
    mission_phase, mission_summary = _mission_report(config_root, args)
    mavros_phase = _plugin_report(config_root)
    vision_bridge_phase = _vision_bridge_report(config_root)
    phases.extend([mission_phase, mavros_phase, vision_bridge_phase])

    vision_missing = _missing(deployment, VISION_GATES)
    if not vision_bridge_phase["ready"]:
        vision_missing.extend(vision_bridge_phase["missing"])
    if not mavros_phase["ready"]:
        vision_missing.extend(mavros_phase["missing"])
    phases.append(_phase(
        "vision_output",
        not vision_missing,
        vision_missing,
        ["FAST-LIO /Odometry -> /mavros/vision_pose/pose_cov"] if not vision_missing else []))

    flight_missing = _missing(deployment, FLIGHT_GATES)
    if not mission_phase["ready"]:
        flight_missing.extend(mission_phase["missing"])
    if not mavros_phase["ready"]:
        flight_missing.extend(mavros_phase["missing"])
    if not vision_bridge_phase["ready"]:
        flight_missing.extend(vision_bridge_phase["missing"])
    phases.append(_phase(
        "active_local_flight",
        not flight_missing,
        flight_missing,
        ["/mavros/setpoint_raw/local + OFFBOARD + AUTO.LAND"] if not flight_missing else []))

    payload_missing = _missing(deployment, PAYLOAD_GATES)
    if not mission_summary["payload_events"]:
        payload_missing.append("mission_payload_action")
    if flight_missing:
        payload_missing.append("active_local_flight_blocked")
    phases.append(_phase(
        "payload_local_flight",
        not payload_missing,
        payload_missing,
        ["payload action present and servo gate confirmed"] if not payload_missing else []))

    return {
        "config_root": str(config_root),
        "route_file": str(pathlib.Path(args.route_file).expanduser().resolve()
                          if args.route_file else config_root / "flight" / "local_waypoints.yaml"),
        "overall_ready": all(phase["ready"] for phase in phases),
        "required_phase": args.require_phase,
        "required_phase_ready": next((phase["ready"] for phase in phases
                                      if phase["name"] == args.require_phase), None),
        "phases": phases,
        "mission": mission_summary,
    }


def _print_text(report):
    print("LOCAL_FLIGHT_READINESS config_root=%s" % report["config_root"])
    print("route_file=%s" % report["route_file"])
    for phase in report["phases"]:
        status = "READY" if phase["ready"] else "BLOCKED"
        print("%s=%s" % (phase["name"], status))
        if phase["missing"]:
            print("  missing=%s" % ",".join(phase["missing"]))
        if phase["notes"]:
            print("  notes=%s" % "; ".join(phase["notes"]))
    print("overall_ready=%s" % report["overall_ready"])


def _build_parser():
    parser = argparse.ArgumentParser(
        description="Report offline readiness for Robotac FAST-LIO/MAVROS local flight.")
    parser.add_argument("--config-root", default="config",
                        help="Robotac config directory, default: ./config")
    parser.add_argument("--route-file", default="",
                        help="Local waypoint YAML; default: CONFIG_ROOT/flight/local_waypoints.yaml")
    parser.add_argument("--origin-x", type=float, default=0.0)
    parser.add_argument("--origin-y", type=float, default=0.0)
    parser.add_argument("--origin-z", type=float, default=0.0)
    parser.add_argument("--origin-yaw", type=float, default=None)
    parser.add_argument("--origin-yaw-deg", type=float, default=0.0)
    parser.add_argument("--require-phase", default="offline",
                        choices=("offline", "vision_output", "active_local_flight", "payload_local_flight"),
                        help="Exit nonzero if the named phase is blocked; offline only validates files")
    parser.add_argument("--json", action="store_true")
    return parser


def main():
    args = _build_parser().parse_args()
    report = build_report(args)
    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        _print_text(report)
    if args.require_phase != "offline" and report["required_phase_ready"] is not True:
        return 2
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        print("ERROR: %s" % exc, file=sys.stderr)
        sys.exit(1)

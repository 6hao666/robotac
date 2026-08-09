#!/usr/bin/env python3
"""Offline contract check for Robotac local MAVROS flight.

This script verifies the source/configuration contract for the current flight
goal: local relative MAVROS waypoint flight with takeoff/landing and FAST-LIO
as MAVROS external-vision pose input. It is intentionally read-only: it never
starts ROS, opens serial/network devices, publishes topics, calls services,
changes modes, arms, lands, or sends setpoints.
"""

import argparse
import json
import pathlib
import sys
import tempfile
from types import SimpleNamespace

import yaml


SCRIPT_DIR = pathlib.Path(__file__).resolve().parent
WORKSPACE_DIR = SCRIPT_DIR.parent
FLIGHT_SCRIPT_DIR = WORKSPACE_DIR / "src" / "robotac_flight" / "scripts"
if str(FLIGHT_SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(FLIGHT_SCRIPT_DIR))

import audit_local_mission  # noqa: E402
import create_route_file  # noqa: E402


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

MAVROS_FORBIDDEN_GLOBAL_PLUGINS = {
    "fake_gps",
    "global_position",
    "gps_input",
    "gps_status",
    "waypoint",
}

FORBIDDEN_FLIGHT_SOURCE_TOKENS = {
    "CommandTOL",
    "CommandTOLLocal",
    "WaypointPush",
    "WaypointPull",
    "WaypointSetCurrent",
    "/mavros/global_position",
    "/mavros/mission",
    "/mavros/cmd/takeoff",
    "/mavros/cmd/land",
}


def _load_yaml(path):
    with open(path, "r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    if not isinstance(data, dict):
        raise ValueError("%s must be a YAML mapping" % path)
    return data


def _read(path):
    return pathlib.Path(path).read_text(encoding="utf-8")


def _phase(name, ready, missing=None, notes=None):
    return {
        "name": name,
        "ready": bool(ready),
        "missing": list(missing or []),
        "notes": list(notes or []),
    }


def _contains_all(source, tokens):
    return [token for token in tokens if token not in source]


def _check_mavros_local_only(root):
    missing = []
    notes = []
    plugin_path = root / "config" / "mavros" / "px4_pluginlists.yaml"
    px4_path = root / "config" / "mavros" / "px4.yaml"
    launch_path = root / "src" / "robotac_bringup" / "launch" / "mavros_px4.launch"

    plugins = _load_yaml(plugin_path)
    whitelist = set(plugins.get("plugin_whitelist") or [])
    blacklist = set(plugins.get("plugin_blacklist") or [])
    for name in sorted(MAVROS_REQUIRED_WHITELIST):
        if name not in whitelist:
            missing.append("mavros_plugin_whitelist:%s" % name)
    for name in sorted(MAVROS_FORBIDDEN_GLOBAL_PLUGINS):
        if name in whitelist:
            missing.append("mavros_global_plugin_whitelisted:%s" % name)
        if name not in blacklist:
            missing.append("mavros_global_plugin_not_blacklisted:%s" % name)

    px4 = _load_yaml(px4_path)
    local_position = px4.get("local_position", {})
    if not isinstance(local_position, dict):
        missing.append("mavros_px4_local_position_mapping")
    else:
        if local_position.get("frame_id") != "map":
            missing.append("mavros_local_position_frame_id")
        local_tf = local_position.get("tf", {}) or {}
        if not isinstance(local_tf, dict) or local_tf.get("child_frame_id") != "base_link":
            missing.append("mavros_local_position_child_frame_id")
    vision_pose = px4.get("vision_pose", {})
    if not isinstance(vision_pose, dict):
        missing.append("mavros_px4_vision_pose_mapping")
    else:
        tf_cfg = vision_pose.get("tf", {}) or {}
        if not isinstance(tf_cfg, dict):
            missing.append("mavros_vision_pose_tf_mapping")
        elif tf_cfg.get("listen") not in (False, "false", "False", 0):
            missing.append("mavros_vision_pose_tf_listen_disabled")

    launch = _read(launch_path)
    if '<arg name="check_geographiclib" default="false" />' not in launch:
        missing.append("mavros_geographiclib_optional_for_local_only")
    if "serial:///dev/px4_fcu:921600" not in launch:
        missing.append("mavros_stable_fcu_default")
    if not missing:
        notes.append("MAVROS surface is local-only: setpoint_raw + vision_pose_estimate, no GPS/global mission")
    return _phase("mavros_local_only_contract", not missing, missing, notes)


def _check_local_route(root):
    missing = []
    notes = []
    route_file = root / "config" / "flight" / "local_waypoints.yaml"
    route = _load_yaml(route_file).get("local_waypoint_flight", {})
    if not isinstance(route, dict):
        return _phase("local_relative_route_contract", False, ["local_waypoint_flight_mapping"])

    audit_args = SimpleNamespace(
        file=str(route_file),
        origin_x=0.0,
        origin_y=0.0,
        origin_z=0.0,
        origin_yaw=None,
        origin_yaw_deg=0.0,
        no_takeoff=False,
        require_auto_land=True,
        require_payload_open=True,
        json=False,
    )
    try:
        summary = audit_local_mission._build_summary(audit_args)
    except Exception as exc:  # pragma: no cover - surfaced in script output
        return _phase("local_relative_route_contract", False, ["mission_audit:%s" % exc])

    if summary.get("frame") != "robotac_start_body":
        missing.append("route_frame_robotac_start_body")
    if abs(float(summary.get("takeoff_height_m", 0.0)) - 1.0) > 1.0e-6:
        missing.append("takeoff_height_1m")
    if summary.get("auto_land_required") is not True:
        missing.append("require_auto_land_true")
    if summary.get("land_mode") != "AUTO.LAND":
        missing.append("land_mode_AUTO.LAND")
    if int(summary.get("targets_with_takeoff", 0)) < 2:
        missing.append("route_targets_with_takeoff")
    if "setpoint_topic" in route and route.get("setpoint_topic") != "/mavros/setpoint_raw/local":
        missing.append("route_setpoint_topic")
    for key in (
            "require_vision",
            "require_vision_output",
            "require_estimator_status",
            "require_horizontal_relative",
            "require_vertical_estimate",
            "require_vision_output_consumer",
            "require_setpoint_consumer",
            "require_timesync"):
        if route.get(key) is not True:
            missing.append("route_%s" % key)

    if not missing:
        notes.extend([
            "route is local relative to captured start heading",
            "takeoff=1.0m targets=%d payload_events=%d" % (
                summary["targets_with_takeoff"], len(summary["payload_events"])),
        ])
    return _phase("local_relative_route_contract", not missing, missing, notes)


def _check_controller_source(root):
    missing = []
    notes = []
    path = root / "src" / "robotac_flight" / "scripts" / "local_waypoint_flight.py"
    launch_path = root / "src" / "robotac_flight" / "launch" / "local_waypoint_flight.launch"
    source = _read(path)
    launch = _read(launch_path)

    forbidden_present = [token for token in sorted(FORBIDDEN_FLIGHT_SOURCE_TOKENS) if token in source]
    if forbidden_present:
        missing.extend("controller_forbidden_token:%s" % token for token in forbidden_present)
    missing.extend("controller_missing:%s" % token for token in _contains_all(source, (
        "PositionTarget.FRAME_LOCAL_NED",
        "msg.position.x, msg.position.y, msg.position.z = target[:3]",
        "if self.enable_control and self.control_tx_enabled and self.setpoint_pub is not None:",
        "self.setpoint_pub = (rospy.Publisher(self.setpoint_topic, PositionTarget, queue_size=10)",
        "if self.enable_control else None)",
        "rospy.ServiceProxy(\"/mavros/set_mode\", SetMode)",
        "rospy.ServiceProxy(\"/mavros/cmd/arming\", CommandBool)",
        "auto_land_required_for_this_route",
        "route_revision=",
        "route_fingerprint=",
        "mavros_setpoint_raw_consumer_unavailable",
        "mavros_vision_pose_consumer_unavailable",
        "mavros_setpoint_raw_consumer_lost",
        "mavros_vision_pose_consumer_lost",
        "waypoints_empty",
    )))
    missing.extend("launch_missing:%s" % token for token in _contains_all(launch, (
        '<arg name="route_file" default="$(arg config_root)/flight/local_waypoints.yaml" />',
        '<arg name="enable_control" default="false" />',
        '<arg name="auto_mode" default="false" />',
        '<arg name="auto_arm" default="false" />',
        '<arg name="auto_land" default="false" />',
        '<arg name="enable_payload" default="false" />',
    )))
    if not missing:
        notes.append("controller publishes MAVROS raw setpoints only after explicit enable_control and /start")
    return _phase("waypoint_controller_contract", not missing, missing, notes)


def _check_local_takeoff_landing(root):
    """Verify takeoff/landing remain local-setpoint based, not global mission based."""
    missing = []
    notes = []
    source_path = root / "src" / "robotac_flight" / "scripts" / "local_waypoint_flight.py"
    route_path = root / "config" / "flight" / "local_waypoints.yaml"
    source = _read(source_path)
    route = _load_yaml(route_path).get("local_waypoint_flight", {})
    if not isinstance(route, dict):
        return _phase("local_takeoff_landing_contract", False, ["local_waypoint_flight_mapping"])

    missing.extend("takeoff_land_source_missing:%s" % token for token in _contains_all(source, (
        "elif self.state == self.WAIT_OFFBOARD:",
        "elif self.auto_mode:",
        "self._request_mode(\"OFFBOARD\")",
        "elif self.state == self.WAIT_ARMED:",
        "elif self.auto_arm:",
        "self._request_arm()",
        "elif self.state == self.TAKEOFF:",
        "self.target = (self.origin[0], self.origin[1], self.origin[2] + self.takeoff_height, self.origin_yaw)",
        "self._publish_setpoint(self.target)",
        "self._enter(self.WAYPOINTS if self.waypoints else self.WAIT_LAND)",
        "elif self.state == self.LANDING:",
        "switch_z = self.origin[2] + self.land_switch_height",
        "self.landing_target[2] - self.land_descent_rate / max(1.0, self.control_rate)",
        "self._request_mode(self.land_mode)",
        "ExtendedState.LANDED_STATE_ON_GROUND",
        "not self.fcu.armed",
        "return TriggerResponse(False, \"auto_land_required_for_this_route\")",
        "return TriggerResponse(False, \"refuse_start_while_armed\")",
        "return TriggerResponse(False, \"vehicle_not_reported_on_ground\")",
    )))
    for forbidden in (
            "/mavros/cmd/takeoff",
            "/mavros/cmd/land",
            "CommandTOL",
            "WaypointPush",
            "WaypointPull",
            "WaypointSetCurrent"):
        if forbidden in source:
            missing.append("takeoff_land_forbidden_global_or_tol:%s" % forbidden)

    expected_route_values = {
        "takeoff_height": 1.0,
        "require_auto_land": True,
        "land_mode": "AUTO.LAND",
        "land_descent_rate": 0.30,
        "land_switch_height": 0.50,
        "waypoint_frame": "robotac_start_body",
    }
    for key, expected in expected_route_values.items():
        if route.get(key) != expected:
            missing.append("takeoff_land_route_%s" % key)
    if route.get("setpoint_topic") != "/mavros/setpoint_raw/local":
        missing.append("takeoff_land_route_setpoint_topic")
    if not missing:
        notes.append("takeoff climbs on local raw setpoints; landing descends locally then requests AUTO.LAND")
    return _phase("local_takeoff_landing_contract", not missing, missing, notes)


def _check_route_generator(root):
    missing = []
    notes = []
    path = root / "src" / "robotac_flight" / "scripts" / "create_route_file.py"
    if not path.exists():
        return _phase("route_generator_contract", False, ["missing:create_route_file.py"])
    source = _read(path)
    missing.extend("route_generator_missing:%s" % token for token in _contains_all(source, (
        "FORBIDDEN_GLOBAL_KEYS",
        "BASE_ROUTE_DEFAULTS",
        "PAYLOAD_ROUTE_DEFAULTS",
        "require_auto_land",
        "require_vision_output_consumer",
        "require_setpoint_consumer",
        "require_timesync",
        "--point",
        "--payload-open-index",
        "--append-return-home",
        "starts no ROS node",
    )))
    for forbidden in (
            "import rospy",
            "rospy.Publisher",
            "rospy.ServiceProxy",
            "CommandBool",
            "SetMode",
            "/mavros/cmd/arming",
            "/mavros/set_mode",
            "/robotac/flight/start"):
        if forbidden in source:
            missing.append("route_generator_forbidden_token:%s" % forbidden)

    args = SimpleNamespace(
        input="",
        point=[
            "1,0,1,0",
            "0,0,1,0",
            "0,1,1,0",
            "0,0,1,0",
            "0,-1,1,0",
            "0,0,1,0",
            "-1,0,1,0",
        ],
        output="",
        force=False,
        dry_run=True,
        frame="robotac_start_body",
        takeoff_height=1.0,
        hold=2.0,
        yaw_deg=0.0,
        max_waypoint_xy=20.0,
        max_waypoint_z=5.0,
        append_return_home=True,
        payload_open_index=6,
        payload_open_last=False,
        payload_settle=2.0,
    )
    try:
        generated = create_route_file._build_route(args)
        section = generated.get("local_waypoint_flight", {})
        if section.get("waypoint_frame") != "robotac_start_body":
            missing.append("generated_route_frame")
        if section.get("require_auto_land") is not True:
            missing.append("generated_route_require_auto_land")
        waypoints = section.get("waypoints") or []
        if len(waypoints) != 8:
            missing.append("generated_route_waypoint_count")
        if not waypoints or waypoints[6].get("payload_action") != "open":
            missing.append("generated_route_payload_open_index")
        with tempfile.NamedTemporaryFile("w", suffix=".yaml", encoding="utf-8") as stream:
            yaml.safe_dump(generated, stream, default_flow_style=False, sort_keys=False)
            stream.flush()
            audit_args = SimpleNamespace(
                file=stream.name,
                origin_x=3.0,
                origin_y=-2.0,
                origin_z=0.0,
                origin_yaw=None,
                origin_yaw_deg=90.0,
                no_takeoff=False,
                require_auto_land=True,
                require_payload_open=True,
                json=False,
            )
            summary = audit_local_mission._build_summary(audit_args)
        if summary.get("targets_with_takeoff") != 9:
            missing.append("generated_route_targets_with_takeoff")
        payload_events = summary.get("payload_events") or []
        if not payload_events or payload_events[0].get("action") != "open":
            missing.append("generated_route_payload_audit")
    except Exception as exc:  # pragma: no cover - surfaced in script output
        missing.append("route_generator_execution:%s" % exc)

    if not missing:
        notes.append("offline generator expands simple local points into audited full route files")
    return _phase("route_generator_contract", not missing, missing, notes)


def _check_fastlio_vision_bridge(root):
    missing = []
    notes = []
    config_path = root / "config" / "fastlio" / "vision_bridge.yaml"
    launch_path = root / "src" / "robotac_flight" / "launch" / "fastlio_vision_bridge.launch"
    source_path = root / "src" / "robotac_flight" / "scripts" / "fastlio_vision_bridge.py"
    config = _load_yaml(config_path).get("fastlio_vision_bridge", {})
    if not isinstance(config, dict):
        return _phase("fastlio_vision_pose_contract", False, ["fastlio_vision_bridge_mapping"])

    for key, expected in (
            ("expected_input_parent", "camera_init"),
            ("expected_input_child", "body"),
            ("output_parent_frame", "odom")):
        if config.get(key) != expected:
            missing.append("vision_bridge_%s" % key)
    if config.get("strict_input_frames") is not True:
        missing.append("vision_bridge_strict_input_frames")
    if config.get("frame_alignment_approved") is True:
        missing.append("real_config_frame_alignment_should_not_be_preapproved")

    launch = _read(launch_path)
    missing.extend("vision_launch_missing:%s" % token for token in _contains_all(launch, (
        '<arg name="input_topic" default="/Odometry" />',
        '<arg name="output_topic" default="/mavros/vision_pose/pose_cov" />',
        '<arg name="enable_mavros_output" default="false" />',
    )))
    source = _read(source_path)
    missing.extend("vision_source_missing:%s" % token for token in _contains_all(source, (
        "requested_output = _as_bool(rospy.get_param(\"~enable_mavros_output\", False))",
        "self.enable_mavros_output = requested_output and self.frame_alignment_approved",
        "self.pose_pub = rospy.Publisher(self.output_topic, PoseWithCovarianceStamped, queue_size=10)",
        "self.preview_pub = rospy.Publisher(self.preview_topic, PoseWithCovarianceStamped, queue_size=10)",
        "if self.enable_mavros_output and self.healthy:",
        "self.pose_pub.publish(output)",
        "self.status_pub.publish(String(data=reason))",
    )))
    if not missing:
        notes.append("FAST-LIO /Odometry camera_init->body is gated before /mavros/vision_pose/pose_cov")
    return _phase("fastlio_vision_pose_contract", not missing, missing, notes)


def _check_evidence_surface(root):
    missing = []
    notes = []
    required_files = (
        "src/robotac_flight/scripts/local_flight_preflight.py",
        "src/robotac_flight/scripts/ev_acceptance_observer.py",
        "src/robotac_flight/scripts/active_flight_observer.py",
        "src/robotac_flight/test/run_route_file_sim.sh",
        "scripts/collect_readonly_flight_evidence.sh",
        "scripts/analyze_readonly_flight_evidence.py",
        "scripts/analyze_active_flight_evidence.py",
        "scripts/flight_goal_audit.py",
        "scripts/flight_test_ladder.sh",
    )
    for relative in required_files:
        if not (root / relative).exists():
            missing.append("missing:%s" % relative)

    readonly = _read(root / "scripts" / "analyze_readonly_flight_evidence.py")
    active = _read(root / "scripts" / "analyze_active_flight_evidence.py")
    goal_audit = _read(root / "scripts" / "flight_goal_audit.py")
    ladder = _read(root / "scripts" / "flight_test_ladder.sh")
    route_file_test = _read(root / "src" / "robotac_flight" / "test" / "run_route_file_sim.sh")
    full_system = _read(root / "src" / "robotac_bringup" / "launch" / "full_system.launch")
    missing.extend("readonly_analyzer_missing:%s" % token for token in _contains_all(readonly, (
        "mavros_safe_state",
        "vision_to_mavros",
        "active_preflight_evidence",
        "ev_acceptance_observer.json",
        "local_flight_preflight",
        "px4_vision_params_not_checked",
        "check_px4_vision_params",
        "check_px4_offboard_failsafe_params",
        "px4_offboard_failsafe:COM_OF_LOSS_T",
        "read_only_no_setpoint_publishers",
    )))
    missing.extend("active_analyzer_missing:%s" % token for token in _contains_all(active, (
        "active_local_flight",
        "payload_local_flight",
        "waypoints_incomplete",
        "target_records_missing",
        "target_records_unreached",
        "takeoff_target_record_missing",
        "waypoint_target_records_missing",
        "waypoint_target_records",
        "raw_setpoint_count_below",
        "unique_raw_setpoints_below",
        "active_raw_setpoint_count_below",
        "active_unique_raw_setpoints_below",
        "raw_setpoint_frame_mismatch_count",
        "raw_setpoint_expected_publisher_missing",
        "--min-raw-setpoints",
        "--min-active-raw-setpoints",
        "--min-active-unique-raw-setpoints",
        "max_continuous_reach_s",
        "min_target_dwell_s",
        "expected_waypoints_mismatch",
        "takeoff_state_missing",
        "landing_state_missing",
        "active_vision_pose_count_below",
        "active_vision_local_delta_error",
        "min-active-vision-local-pairs",
        "max-active-vision-local-delta-m",
        "active_route_manifest",
        "route_manifest_missing",
        "route_manifest_target_tolerance",
        "route_manifest_observed_target_mismatch",
        "route_manifest_raw_setpoint_missing",
        "route_manifest_raw_setpoint_order_mismatch",
        "route_manifest_active_raw_setpoint_missing",
        "route_manifest_active_raw_setpoint_order_mismatch",
        "route_status_fingerprint_mismatch",
        "active_vision_output_enabled_seen",
        "active_fastlio_vision_status_ok_seen",
        "active_mavros_armed_seen",
        "active_mavros_offboard_seen",
        "final_disarmed",
        "final_on_ground",
    )))
    missing.extend("goal_audit_missing:%s" % token for token in _contains_all(goal_audit, (
        "active_route_matches_config",
        "%s_target_mismatch",
        "configured_route_manifest_missing",
        "route_manifest_source_not_configured",
        "prefix=\"route_manifest\"",
        "route_origin_mismatch",
        "initial_local_position",
        "%s_targets_match",
        "allow_dynamic_active_route",
        "dynamic_route_manifest_missing",
        "prefix=\"dynamic_route\"",
        "preflight_evidence_file",
        "route_file",
    )))
    missing.extend("ladder_missing:%s" % token for token in _contains_all(ladder, (
        "--route-file",
        "--deploy-workspace",
        "--deploy-route-file",
        "deploy_workspace",
        "deploy_route_file",
        "flight_route_file",
        "active flight commands hidden",
        "Read-only evidence did not pass active_preflight_evidence",
        "check_px4_offboard_failsafe_params:=true",
        "rosservice call /robotac/flight/start",
        "flight_auto_arm:=false",
        "flight_auto_mode:=false",
        "flight_auto_land:=true",
    )))
    missing.extend("full_system_missing:%s" % token for token in _contains_all(full_system, (
        '<arg name="flight_route_file" default="$(arg config_root)/flight/local_waypoints.yaml" />',
        '<arg name="route_file" value="$(arg flight_route_file)" />',
    )))
    missing.extend("route_file_test_missing:%s" % token for token in _contains_all(route_file_test, (
        "local_waypoints_simple_box.yaml",
        "route_file:=",
        "enable_payload:=false",
        "payload_open_at=none",
        "Route-file closed-loop flight simulation passed.",
    )))
    if not missing:
        notes.append("read-only and active evidence gates cover vision input, target reach, landing, and payload")
    return _phase("evidence_gate_contract", not missing, missing, notes)


def build_report(args):
    root = pathlib.Path(args.workspace).expanduser().resolve()
    phases = [
        _check_mavros_local_only(root),
        _check_local_route(root),
        _check_controller_source(root),
        _check_local_takeoff_landing(root),
        _check_route_generator(root),
        _check_fastlio_vision_bridge(root),
        _check_evidence_surface(root),
    ]
    return {
        "workspace": str(root),
        "overall_ready": all(phase["ready"] for phase in phases),
        "phases": phases,
    }


def _print_text(report):
    print("ROBOTAC_FLIGHT_CONTRACT_CHECK workspace=%s" % report["workspace"])
    for phase in report["phases"]:
        print("%s=%s" % (phase["name"], "READY" if phase["ready"] else "BLOCKED"))
        if phase["missing"]:
            print("  missing=%s" % ",".join(phase["missing"]))
        if phase["notes"]:
            print("  notes=%s" % "; ".join(phase["notes"]))
    print("overall_ready=%s" % report["overall_ready"])


def _build_parser():
    parser = argparse.ArgumentParser(
        description="Check the offline Robotac local MAVROS/FAST-LIO flight contract.")
    parser.add_argument("--workspace", default=str(WORKSPACE_DIR),
                        help="Robotac workspace/repository root, default: this script's parent")
    parser.add_argument("--json", action="store_true")
    return parser


def main():
    args = _build_parser().parse_args()
    report = build_report(args)
    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        _print_text(report)
    return 0 if report["overall_ready"] else 2


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        print("ERROR: %s" % exc, file=sys.stderr)
        sys.exit(1)

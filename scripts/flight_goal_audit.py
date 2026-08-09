#!/usr/bin/env python3
"""Aggregate Robotac local-flight goal readiness and evidence.

This is the top-level offline audit for the current goal: local relative
MAVROS waypoint flight, takeoff/landing, and FAST-LIO as MAVROS vision pose.
It reads configuration plus optional evidence directories/files. It never
starts ROS, opens serial devices, publishes topics, calls services, changes
modes, arms, or sends setpoints.
"""

import argparse
import json
import math
import pathlib
import sys
from types import SimpleNamespace


SCRIPT_DIR = pathlib.Path(__file__).resolve().parent
WORKSPACE_DIR = SCRIPT_DIR.parent
FLIGHT_SCRIPT_DIR = WORKSPACE_DIR / "src" / "robotac_flight" / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))
if str(FLIGHT_SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(FLIGHT_SCRIPT_DIR))

import analyze_active_flight_evidence  # noqa: E402
import analyze_frame_alignment_evidence  # noqa: E402
import analyze_readonly_flight_evidence  # noqa: E402
import local_flight_readiness  # noqa: E402
import preview_local_route  # noqa: E402


def _phase(name, ready, missing=None, notes=None):
    return {
        "name": name,
        "ready": bool(ready),
        "missing": list(missing or []),
        "notes": list(notes or []),
    }


def _find_phase(report, name):
    for phase in report.get("phases", []):
        if phase.get("name") == name:
            return phase
    return _phase(name, False, ["phase_missing:%s" % name])


def _readiness_report(args):
    readiness_args = SimpleNamespace(
        config_root=args.config_root,
        route_file=args.route_file,
        origin_x=args.origin_x,
        origin_y=args.origin_y,
        origin_z=args.origin_z,
        origin_yaw=None,
        origin_yaw_deg=args.origin_yaw_deg,
        require_phase="offline",
        json=False,
    )
    return local_flight_readiness.build_report(readiness_args)


def _readonly_report(args):
    if not args.readonly_evidence:
        return None
    readonly_args = SimpleNamespace(
        evidence_dir=args.readonly_evidence,
        mavros_node=args.mavros_node,
        min_local_hz=args.min_local_hz,
        min_lidar_hz=args.min_lidar_hz,
        min_lidar_imu_hz=args.min_lidar_imu_hz,
        min_fastlio_hz=args.min_fastlio_hz,
        min_preview_hz=args.min_preview_hz,
        min_vision_hz=args.min_vision_hz,
        min_timesync_hz=args.min_timesync_hz,
        preflight_evidence_file=args.preflight_evidence_file,
        ev_acceptance_file=args.ev_acceptance_file,
        require_phase="active_preflight_evidence",
        json=False,
    )
    return analyze_readonly_flight_evidence.build_report(readonly_args)


def _frame_alignment_report(args):
    if not args.frame_alignment_evidence:
        return None
    frame_args = SimpleNamespace(
        evidence=args.frame_alignment_evidence,
        required_axes=args.frame_alignment_required_axes,
        require_yaw=args.require_frame_alignment_yaw,
        pose_topic=args.frame_alignment_pose_topic,
        expected_parent=args.frame_alignment_expected_parent,
        min_pose_rate_hz=args.frame_alignment_min_pose_rate_hz,
        min_translation_m=args.frame_alignment_min_translation_m,
        max_direction_error_deg=args.frame_alignment_max_direction_error_deg,
        min_delta_scale=args.frame_alignment_min_delta_scale,
        max_delta_scale=args.frame_alignment_max_delta_scale,
        min_yaw_deg=args.frame_alignment_min_yaw_deg,
        max_yaw_error_deg=args.frame_alignment_max_yaw_error_deg,
        require_vision_status_ok=True,
        json=False,
    )
    return analyze_frame_alignment_evidence.build_report(frame_args)


def _active_report(args, configured_waypoints):
    if not args.active_evidence:
        return None
    expected_waypoints = args.expected_waypoints
    if expected_waypoints <= 0 and not args.allow_dynamic_active_route:
        expected_waypoints = configured_waypoints
    min_waypoints = args.min_waypoints if args.min_waypoints > 0 else max(1, expected_waypoints)
    active_args = SimpleNamespace(
        evidence=args.active_evidence,
        require_phase="active_local_flight",
        min_waypoints=min_waypoints,
        expected_waypoints=expected_waypoints,
        min_setpoints=args.min_setpoints,
        min_unique_setpoints=args.min_unique_setpoints,
        require_raw_setpoints=args.require_raw_setpoints,
        min_raw_setpoints=args.min_raw_setpoints,
        min_unique_raw_setpoints=args.min_unique_raw_setpoints,
        min_active_raw_setpoints=args.min_active_raw_setpoints,
        min_active_unique_raw_setpoints=args.min_active_unique_raw_setpoints,
        require_raw_setpoint_publisher=args.require_raw_setpoint_publisher,
        min_airborne_altitude=args.min_airborne_altitude,
        waypoint_reach_tolerance=args.waypoint_reach_tolerance,
        min_target_dwell_s=args.min_target_dwell_s,
        min_active_vision_pose_count=args.min_active_vision_pose_count,
        min_active_vision_local_pairs=args.min_active_vision_local_pairs,
        max_active_vision_local_delta_m=args.max_active_vision_local_delta_m,
        require_route_manifest=True,
        route_manifest_target_tolerance=args.route_target_tolerance,
        route_manifest_yaw_tolerance_deg=args.route_yaw_tolerance_deg,
        require_active_mavros_control=args.require_active_mavros_control,
        require_takeoff_landing_states=args.require_takeoff_landing_states,
        json=False,
    )
    return analyze_active_flight_evidence.build_report(active_args)


def _route_file(args):
    if args.route_file:
        return pathlib.Path(args.route_file).expanduser().resolve()
    return pathlib.Path(args.config_root).expanduser().resolve() / "flight" / "local_waypoints.yaml"


def _number(value):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _angle_error(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def _target_tuple(record):
    target = record.get("target") if isinstance(record, dict) else None
    if not isinstance(target, list) or len(target) < 4:
        return None
    values = tuple(_number(value) for value in target[:4])
    return values if all(value is not None for value in values) else None


def _position_tuple(values):
    if not isinstance(values, (list, tuple)) or len(values) < 3:
        return None
    result = tuple(_number(value) for value in values[:3])
    return result if all(value is not None for value in result) else None


def _active_targets_by_key(records):
    actual_by_key = {}
    for record in records:
        if not isinstance(record, dict):
            continue
        state = record.get("state")
        if state == "TAKEOFF":
            key = ("TAKEOFF", 0)
        elif state == "WAYPOINTS":
            try:
                key = ("WAYPOINTS", int(record.get("waypoint_index")))
            except (TypeError, ValueError):
                continue
        else:
            continue
        target = _target_tuple(record)
        if target is not None:
            actual_by_key[key] = target
    return actual_by_key


def _match_route_targets(records, expected_targets, args, prefix="route"):
    actual_by_key = _active_targets_by_key(records)
    missing = []
    notes = []
    max_position_delta = 0.0
    max_yaw_delta = 0.0
    position_tolerance = args.route_target_tolerance
    yaw_tolerance = math.radians(args.route_yaw_tolerance_deg)
    for index, expected in enumerate(expected_targets):
        if not isinstance(expected, dict):
            missing.append("%s_target_invalid:%d" % (prefix, index))
            continue
        if expected.get("state") == "TAKEOFF":
            key = ("TAKEOFF", 0)
        elif expected.get("state") == "WAYPOINTS":
            try:
                key = ("WAYPOINTS", int(expected.get("waypoint_index")))
            except (TypeError, ValueError):
                missing.append("%s_target_invalid:%d" % (prefix, index))
                continue
        else:
            missing.append("%s_target_invalid:%d" % (prefix, index))
            continue
        actual = actual_by_key.get(key)
        if actual is None:
            missing.append("%s_target_missing:%s%d" % (prefix, key[0].lower(), key[1]))
            continue
        expected_target = _target_tuple(expected)
        if expected_target is None:
            missing.append("%s_target_invalid:%s%d" % (prefix, key[0].lower(), key[1]))
            continue
        position_delta = math.sqrt(sum((actual[i] - expected_target[i]) ** 2 for i in range(3)))
        yaw_delta = abs(_angle_error(actual[3], expected_target[3]))
        max_position_delta = max(max_position_delta, position_delta)
        max_yaw_delta = max(max_yaw_delta, yaw_delta)
        if position_delta > position_tolerance or yaw_delta > yaw_tolerance:
            missing.append("%s_target_mismatch:%s%d:pos=%.3f:yaw_deg=%.2f" % (
                prefix, key[0].lower(), key[1], position_delta, math.degrees(yaw_delta)))
    if not missing:
        notes.append("%s_targets_match=%d max_pos_delta=%.3f max_yaw_delta_deg=%.2f" % (
            prefix, len(expected_targets), max_position_delta, math.degrees(max_yaw_delta)))
    return missing, notes


def _dynamic_route_match_report(args, summary, records):
    manifest = summary.get("route_manifest") if isinstance(summary.get("route_manifest"), dict) else None
    if manifest is None:
        return _phase("active_route_matches_config", False,
                      ["dynamic_route_manifest_missing"],
                      ["active_flight_observer must record /robotac/flight/route_manifest"])
    target_route = manifest.get("target_route")
    if not isinstance(target_route, list) or len(target_route) < 2:
        return _phase("active_route_matches_config", False,
                      ["dynamic_route_target_manifest_missing"])
    if manifest.get("event") != "mission_started":
        return _phase("active_route_matches_config", False,
                      ["dynamic_route_manifest_not_started"])
    if manifest.get("route_source") != "posearray":
        return _phase("active_route_matches_config", False,
                      ["dynamic_route_source_not_posearray"])
    waypoint_count = _number(manifest.get("waypoint_count"))
    waypoint_count_int = None if waypoint_count is None else int(waypoint_count)
    if (waypoint_count is None or abs(waypoint_count - waypoint_count_int) > 1.0e-9 or
            waypoint_count_int != len([item for item in target_route
                                       if isinstance(item, dict) and item.get("state") == "WAYPOINTS"])):
        return _phase("active_route_matches_config", False,
                      ["dynamic_route_waypoint_count_mismatch"])

    missing = []
    notes = []
    initial_position = _position_tuple(summary.get("initial_local_position"))
    initial_yaw = _number(summary.get("initial_local_yaw"))
    origin = _position_tuple(manifest.get("origin"))
    origin_yaw = _number(manifest.get("origin_yaw"))
    if origin is None:
        missing.append("dynamic_route_origin_missing")
    elif initial_position is None:
        missing.append("route_origin_position_missing")
    else:
        origin_delta = math.sqrt(sum((origin[i] - initial_position[i]) ** 2 for i in range(3)))
        if origin_delta > args.route_origin_tolerance:
            missing.append("route_origin_mismatch:pos=%.3f" % origin_delta)
    if origin_yaw is None:
        missing.append("dynamic_route_origin_yaw_missing")
    elif initial_yaw is None:
        missing.append("route_origin_yaw_missing")
    else:
        origin_yaw_delta = abs(_angle_error(origin_yaw, initial_yaw))
        if origin_yaw_delta > math.radians(args.route_yaw_tolerance_deg):
            missing.append("route_origin_yaw_mismatch:deg=%.2f" % math.degrees(origin_yaw_delta))

    target_missing, target_notes = _match_route_targets(records, target_route, args, prefix="dynamic_route")
    missing.extend(target_missing)
    notes.extend(target_notes)
    if not missing:
        notes.append("dynamic_route_source=%s revision=%s fingerprint=%s" % (
            manifest.get("route_source", "unknown"),
            manifest.get("route_revision", "unknown"),
            str(manifest.get("route_fingerprint", ""))[:12]))
    return _phase("active_route_matches_config", not missing, missing, notes)


def _active_route_match_report(args):
    if not args.active_evidence:
        return _phase("active_route_matches_config", False,
                      ["active_flight_evidence_missing"],
                      ["pass --active-evidence after active_flight_observer exits"])
    _path, data = analyze_active_flight_evidence._load(args.active_evidence)
    summary = data.get("summary") if isinstance(data.get("summary"), dict) else {}
    records = summary.get("target_records") if isinstance(summary.get("target_records"), list) else []
    if args.allow_dynamic_active_route:
        return _dynamic_route_match_report(args, summary, records)

    route_file = _route_file(args)
    section, frame, waypoints = preview_local_route._parse_route(str(route_file))
    preview_local_route._validate(section, waypoints)

    manifest = summary.get("route_manifest") if isinstance(summary.get("route_manifest"), dict) else None
    if manifest is None:
        return _phase("active_route_matches_config", False,
                      ["configured_route_manifest_missing"],
                      ["active_flight_observer must record /robotac/flight/route_manifest"])

    missing = []
    notes = []
    if manifest.get("event") != "mission_started":
        missing.append("route_manifest_not_started")
    if manifest.get("route_source") != "configured":
        missing.append("route_manifest_source_not_configured")
    manifest_count = _number(manifest.get("waypoint_count"))
    manifest_count_int = None if manifest_count is None else int(manifest_count)
    if (manifest_count is None or abs(manifest_count - manifest_count_int) > 1.0e-9 or
            manifest_count_int != len(waypoints)):
        missing.append("route_manifest_waypoint_count_mismatch")
    if manifest.get("waypoint_frame") != frame:
        missing.append("route_manifest_frame_mismatch")
    takeoff_height = preview_local_route._finite_float(
        section.get("takeoff_height", 1.0), "takeoff_height")
    manifest_takeoff = _number(manifest.get("takeoff_height"))
    if manifest_takeoff is None or abs(manifest_takeoff - takeoff_height) > args.route_target_tolerance:
        missing.append("route_manifest_takeoff_height_mismatch")
    if section.get("require_auto_land") is True and manifest.get("require_auto_land") is not True:
        missing.append("route_manifest_require_auto_land_missing")
    if section.get("require_auto_land") is True and manifest.get("auto_land") is not True:
        missing.append("route_manifest_auto_land_not_enabled")

    origin = _position_tuple(manifest.get("origin"))
    origin_yaw = _number(manifest.get("origin_yaw"))
    if origin is None:
        missing.append("route_manifest_origin_missing")
    if origin_yaw is None:
        missing.append("route_manifest_origin_yaw_missing")
    if missing:
        return _phase("active_route_matches_config", False, missing, notes)

    expected_targets = preview_local_route._build_targets(
        section, frame, waypoints, origin, origin_yaw, include_takeoff=True)

    initial_position = _position_tuple(summary.get("initial_local_position"))
    initial_yaw = _number(summary.get("initial_local_yaw"))
    max_origin_delta = 0.0
    max_origin_yaw_delta = 0.0
    if initial_position is None:
        missing.append("route_origin_position_missing")
    else:
        origin_delta = math.sqrt(sum((origin[i] - initial_position[i]) ** 2 for i in range(3)))
        max_origin_delta = origin_delta
        if origin_delta > args.route_origin_tolerance:
            missing.append("route_origin_mismatch:pos=%.3f" % origin_delta)
    if initial_yaw is None:
        missing.append("route_origin_yaw_missing")
    else:
        origin_yaw_delta = abs(_angle_error(origin_yaw, initial_yaw))
        max_origin_yaw_delta = origin_yaw_delta
        if origin_yaw_delta > math.radians(args.route_yaw_tolerance_deg):
            missing.append("route_origin_yaw_mismatch:deg=%.2f" % math.degrees(origin_yaw_delta))

    expected_records = []
    for index, expected in enumerate(expected_targets):
        expected_records.append({
            "state": "TAKEOFF" if index == 0 else "WAYPOINTS",
            "waypoint_index": None if index == 0 else index - 1,
            "target": list(expected["target"]),
        })
    manifest_route = manifest.get("target_route")
    if not isinstance(manifest_route, list):
        missing.append("route_manifest_target_route_missing")
    else:
        manifest_missing, manifest_notes = _match_route_targets(
            manifest_route, expected_records, args, prefix="route_manifest")
        missing.extend(manifest_missing)
        notes.extend(manifest_notes)
    target_missing, target_notes = _match_route_targets(records, expected_records, args, prefix="route")
    missing.extend(target_missing)
    notes.extend(target_notes)

    if not missing:
        notes.append("route_origin_delta=%.3f origin_yaw_delta_deg=%.2f" % (
            max_origin_delta, math.degrees(max_origin_yaw_delta)))
        notes.append("route_source=%s revision=%s fingerprint=%s" % (
            manifest.get("route_source", "unknown"),
            manifest.get("route_revision", "unknown"),
            str(manifest.get("route_fingerprint", ""))[:12]))
    return _phase("active_route_matches_config", not missing, missing, notes)


def _config_phase(readiness, name):
    phase = _find_phase(readiness, name)
    return _phase("config_%s" % name, phase["ready"], phase.get("missing"), phase.get("notes"))


def build_report(args):
    readiness = _readiness_report(args)
    readonly = _readonly_report(args)
    frame_alignment = _frame_alignment_report(args)
    configured_waypoints = int(readiness.get("mission", {}).get("waypoints") or 0)
    active = _active_report(args, configured_waypoints)
    active_route_match = _active_route_match_report(args)

    phases = [
        _config_phase(readiness, "vision_output"),
        _config_phase(readiness, "active_local_flight"),
        _config_phase(readiness, "payload_local_flight"),
    ]

    if frame_alignment is None:
        phases.append(_phase("fastlio_frame_alignment_preview_evidence", False,
                             ["frame_alignment_evidence_missing"],
                             ["pass --frame-alignment-evidence with preview +X/+Y/+Z JSON evidence"] ))
    else:
        missing = []
        notes = []
        for phase in frame_alignment.get("phases", []):
            if not phase.get("ready"):
                missing.extend(phase.get("missing") or [phase.get("name")])
        if frame_alignment.get("required_phase_ready"):
            notes.append("frame_alignment_preview_evidence ready")
        phases.append(_phase("fastlio_frame_alignment_preview_evidence",
                             frame_alignment.get("required_phase_ready") is True,
                             missing, notes))

    if readonly is None:
        phases.append(_phase("readonly_active_preflight_evidence", False,
                             ["readonly_evidence_dir_missing"],
                             ["pass --readonly-evidence after collect_readonly_flight_evidence.sh and EV acceptance"] ))
    else:
        missing = []
        notes = []
        for phase in readonly.get("phases", []):
            if not phase.get("ready"):
                missing.extend(phase.get("missing") or [phase.get("name")])
        if readonly.get("required_phase_ready"):
            notes.append("active_preflight_evidence ready")
        phases.append(_phase("readonly_active_preflight_evidence",
                             readonly.get("required_phase_ready") is True,
                             missing, notes))

    if active is None:
        phases.append(_phase("active_local_flight_evidence", False,
                             ["active_flight_evidence_missing"],
                             ["pass --active-evidence after active_flight_observer exits"] ))
        phases.append(_phase("payload_local_flight_evidence", False,
                             ["active_flight_evidence_missing"],
                             ["pass --active-evidence and require payload evidence"] ))
        phases.append(active_route_match)
    else:
        active_phase = _find_phase(active, "active_local_flight")
        payload_phase = _find_phase(active, "payload_local_flight")
        phases.append(_phase("active_local_flight_evidence", active_phase["ready"],
                             active_phase.get("missing"), active_phase.get("notes")))
        phases.append(_phase("payload_local_flight_evidence",
                             active_phase["ready"] and payload_phase["ready"],
                             (active_phase.get("missing") or []) + (payload_phase.get("missing") or []),
                             payload_phase.get("notes")))
        phases.append(active_route_match)

    lookup = {phase["name"]: phase for phase in phases}
    groups = {
        "configuration": (
            "config_vision_output",
            "config_active_local_flight",
        ),
        "active_preflight": (
            "config_vision_output",
            "config_active_local_flight",
            "fastlio_frame_alignment_preview_evidence",
            "readonly_active_preflight_evidence",
        ),
        "active_local_flight": (
            "config_vision_output",
            "config_active_local_flight",
            "fastlio_frame_alignment_preview_evidence",
            "readonly_active_preflight_evidence",
            "active_local_flight_evidence",
            "active_route_matches_config",
        ),
        "payload_local_flight": (
            "config_vision_output",
            "config_active_local_flight",
            "config_payload_local_flight",
            "fastlio_frame_alignment_preview_evidence",
            "readonly_active_preflight_evidence",
            "active_local_flight_evidence",
            "active_route_matches_config",
            "payload_local_flight_evidence",
        ),
    }
    phase_groups = {
        name: all(lookup[phase_name]["ready"] for phase_name in phase_names)
        for name, phase_names in groups.items()
    }
    return {
        "config_root": str(pathlib.Path(args.config_root).expanduser().resolve()),
        "route_file": str(_route_file(args)),
        "readonly_evidence": None if not args.readonly_evidence else str(pathlib.Path(args.readonly_evidence).expanduser().resolve()),
        "frame_alignment_evidence": None if not args.frame_alignment_evidence else [str(pathlib.Path(path).expanduser().resolve()) for path in args.frame_alignment_evidence],
        "active_evidence": None if not args.active_evidence else str(pathlib.Path(args.active_evidence).expanduser().resolve()),
        "required_phase": args.require_phase,
        "required_phase_ready": phase_groups[args.require_phase],
        "phase_groups": phase_groups,
        "phases": phases,
    }


def _print_text(report):
    print("ROBOTAC_FLIGHT_GOAL_AUDIT config_root=%s" % report["config_root"])
    print("route_file=%s" % report["route_file"])
    if report["readonly_evidence"]:
        print("readonly_evidence=%s" % report["readonly_evidence"])
    if report["frame_alignment_evidence"]:
        for path in report["frame_alignment_evidence"]:
            print("frame_alignment_evidence=%s" % path)
    if report["active_evidence"]:
        print("active_evidence=%s" % report["active_evidence"])
    for phase in report["phases"]:
        print("%s=%s" % (phase["name"], "READY" if phase["ready"] else "BLOCKED"))
        if phase["missing"]:
            print("  missing=%s" % ",".join(phase["missing"]))
        if phase["notes"]:
            print("  notes=%s" % "; ".join(phase["notes"]))
    for name, ready in sorted(report["phase_groups"].items()):
        print("%s=%s" % (name, "READY" if ready else "BLOCKED"))
    print("required_phase=%s" % report["required_phase"])
    print("required_phase_ready=%s" % report["required_phase_ready"])


def _build_parser():
    parser = argparse.ArgumentParser(
        description="Top-level offline audit for Robotac local MAVROS/FAST-LIO flight goal.")
    parser.add_argument("--config-root", default="config")
    parser.add_argument("--route-file", default="",
                        help="Local waypoint YAML; default: CONFIG_ROOT/flight/local_waypoints.yaml")
    parser.add_argument("--readonly-evidence", default="",
                        help="Directory containing read-only topic evidence and ev_acceptance_observer.json")
    parser.add_argument("--frame-alignment-evidence", nargs="*", default=[],
                        help="FAST-LIO preview frame-alignment JSON file(s) or directories")
    parser.add_argument("--active-evidence", default="",
                        help="active_flight_observer.json or directory containing it")
    parser.add_argument("--preflight-evidence-file", default="",
                        help="local_flight_preflight.json path; default: READONLY_EVIDENCE/local_flight_preflight.json")
    parser.add_argument("--ev-acceptance-file", default="")
    parser.add_argument("--mavros-node", default="/mavros")
    parser.add_argument("--origin-x", type=float, default=0.0)
    parser.add_argument("--origin-y", type=float, default=0.0)
    parser.add_argument("--origin-z", type=float, default=0.0)
    parser.add_argument("--origin-yaw-deg", type=float, default=0.0)
    parser.add_argument("--min-local-hz", type=float, default=5.0)
    parser.add_argument("--min-lidar-hz", type=float, default=5.0)
    parser.add_argument("--min-lidar-imu-hz", type=float, default=100.0)
    parser.add_argument("--min-fastlio-hz", type=float, default=5.0)
    parser.add_argument("--min-preview-hz", type=float, default=5.0)
    parser.add_argument("--min-vision-hz", type=float, default=5.0)
    parser.add_argument("--min-timesync-hz", type=float, default=2.0)
    parser.add_argument("--frame-alignment-required-axes", default="positive_x,positive_y,positive_z")
    parser.add_argument("--require-frame-alignment-yaw", action="store_true")
    parser.add_argument("--frame-alignment-pose-topic", default="/robotac/fastlio_vision/path_a_pose_preview")
    parser.add_argument("--frame-alignment-expected-parent", default="odom")
    parser.add_argument("--frame-alignment-min-pose-rate-hz", type=float, default=5.0)
    parser.add_argument("--frame-alignment-min-translation-m", type=float, default=0.30)
    parser.add_argument("--frame-alignment-max-direction-error-deg", type=float, default=25.0)
    parser.add_argument("--frame-alignment-min-delta-scale", type=float, default=0.50)
    parser.add_argument("--frame-alignment-max-delta-scale", type=float, default=2.00)
    parser.add_argument("--frame-alignment-min-yaw-deg", type=float, default=20.0)
    parser.add_argument("--frame-alignment-max-yaw-error-deg", type=float, default=25.0)
    parser.add_argument("--min-waypoints", type=int, default=0,
                        help="Minimum observed active-flight waypoints; 0 uses the configured route count")
    parser.add_argument("--expected-waypoints", type=int, default=0,
                        help="Exact observed active-flight waypoint count; 0 uses the configured route count")
    parser.add_argument("--allow-dynamic-active-route", action="store_true",
                        help="Do not require active-flight evidence to match the configured route waypoint count")
    parser.add_argument("--route-target-tolerance", type=float, default=0.05,
                        help="Position tolerance in metres for matching active evidence targets to the configured route")
    parser.add_argument("--route-origin-tolerance", type=float, default=0.35,
                        help="Position tolerance in metres for matching the inferred route origin to the observed initial local position")
    parser.add_argument("--route-yaw-tolerance-deg", type=float, default=1.0,
                        help="Yaw tolerance in degrees for matching active evidence targets to the configured route")
    parser.add_argument("--min-setpoints", type=int, default=20)
    parser.add_argument("--min-unique-setpoints", type=int, default=2)
    parser.add_argument("--no-require-raw-setpoints", dest="require_raw_setpoints",
                        action="store_false", default=True,
                        help="Do not require active evidence from /mavros/setpoint_raw/local")
    parser.add_argument("--min-raw-setpoints", type=int, default=20)
    parser.add_argument("--min-unique-raw-setpoints", type=int, default=2)
    parser.add_argument("--min-active-raw-setpoints", type=int, default=20)
    parser.add_argument("--min-active-unique-raw-setpoints", type=int, default=2)
    parser.add_argument("--no-require-raw-setpoint-publisher", dest="require_raw_setpoint_publisher",
                        action="store_false", default=True,
                        help="Do not require active evidence to show the expected raw setpoint publisher")
    parser.add_argument("--min-airborne-altitude", type=float, default=0.50)
    parser.add_argument("--waypoint-reach-tolerance", type=float, default=0.35)
    parser.add_argument("--min-target-dwell-s", type=float, default=0.25,
                        help="Require each TAKEOFF/WAYPOINTS target to remain within reach tolerance for this many continuous seconds")
    parser.add_argument("--min-active-vision-pose-count", type=int, default=5,
                        help="Require active-flight evidence to include this many MAVROS vision pose samples; 0 disables")
    parser.add_argument("--min-active-vision-local-pairs", type=int, default=5,
                        help="Require active-flight evidence to show local_position and vision_pose relative motion agree; 0 disables")
    parser.add_argument("--max-active-vision-local-delta-m", type=float, default=0.75,
                        help="Maximum active-flight local_position vs vision_pose relative-motion disagreement")
    parser.add_argument("--no-require-active-mavros-control", dest="require_active_mavros_control",
                        action="store_false", default=True,
                        help="Do not require active evidence to show MAVROS connected, armed, and OFFBOARD")
    parser.add_argument("--no-require-takeoff-landing-states", dest="require_takeoff_landing_states",
                        action="store_false", default=True,
                        help="Do not require active evidence to show TAKEOFF and LANDING controller states")
    parser.add_argument("--require-phase", default="active_local_flight",
                        choices=("configuration", "active_preflight",
                                 "active_local_flight", "payload_local_flight"))
    parser.add_argument("--json", action="store_true")
    return parser


def main():
    args = _build_parser().parse_args()
    if args.min_waypoints < 0 or args.expected_waypoints < 0:
        raise ValueError("min-waypoints and expected-waypoints must be non-negative")
    if (args.route_target_tolerance < 0.0 or args.route_origin_tolerance < 0.0 or
            args.route_yaw_tolerance_deg < 0.0):
        raise ValueError("route target/yaw tolerances must be non-negative")
    if args.min_target_dwell_s < 0.0:
        raise ValueError("min-target-dwell-s must be non-negative")
    if args.require_raw_setpoints and (args.min_raw_setpoints < 1 or args.min_unique_raw_setpoints < 1):
        raise ValueError("minimum raw setpoint counts must be positive when raw setpoints are required")
    if args.require_raw_setpoints and (args.min_active_raw_setpoints < 1 or
                                       args.min_active_unique_raw_setpoints < 1):
        raise ValueError("minimum active raw setpoint counts must be positive when raw setpoints are required")
    if args.min_active_vision_pose_count < 0:
        raise ValueError("min-active-vision-pose-count must be non-negative")
    if args.min_active_vision_local_pairs < 0:
        raise ValueError("min-active-vision-local-pairs must be non-negative")
    if not math.isfinite(args.max_active_vision_local_delta_m) or args.max_active_vision_local_delta_m < 0.0:
        raise ValueError("max-active-vision-local-delta-m must be finite and non-negative")
    report = build_report(args)
    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        _print_text(report)
    return 0 if report["required_phase_ready"] else 2


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        print("ERROR: %s" % exc, file=sys.stderr)
        sys.exit(1)

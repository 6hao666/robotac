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
        min_fastlio_hz=args.min_fastlio_hz,
        min_vision_hz=args.min_vision_hz,
        min_timesync_hz=args.min_timesync_hz,
        ev_acceptance_file=args.ev_acceptance_file,
        require_phase="active_preflight_evidence",
        json=False,
    )
    return analyze_readonly_flight_evidence.build_report(readonly_args)


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
        min_airborne_altitude=args.min_airborne_altitude,
        waypoint_reach_tolerance=args.waypoint_reach_tolerance,
        min_target_dwell_s=args.min_target_dwell_s,
        json=False,
    )
    return analyze_active_flight_evidence.build_report(active_args)


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


def _active_route_match_report(args):
    if not args.active_evidence:
        return _phase("active_route_matches_config", False,
                      ["active_flight_evidence_missing"],
                      ["pass --active-evidence after active_flight_observer exits"])
    if args.allow_dynamic_active_route:
        return _phase("active_route_matches_config", True, [],
                      ["dynamic active route explicitly allowed; config route target match skipped"])

    _path, data = analyze_active_flight_evidence._load(args.active_evidence)
    summary = data.get("summary") if isinstance(data.get("summary"), dict) else {}
    records = summary.get("target_records") if isinstance(summary.get("target_records"), list) else []
    takeoff_records = [record for record in records
                       if isinstance(record, dict) and record.get("state") == "TAKEOFF"]
    if not takeoff_records:
        return _phase("active_route_matches_config", False, ["route_takeoff_target_missing"])
    takeoff_target = _target_tuple(takeoff_records[0])
    if takeoff_target is None:
        return _phase("active_route_matches_config", False, ["route_takeoff_target_invalid"])

    route_file = pathlib.Path(args.config_root).expanduser().resolve() / "flight" / "local_waypoints.yaml"
    section, frame, waypoints = preview_local_route._parse_route(str(route_file))
    preview_local_route._validate(section, waypoints)
    takeoff_height = preview_local_route._finite_float(
        section.get("takeoff_height", 1.0), "takeoff_height")
    origin = (takeoff_target[0], takeoff_target[1], takeoff_target[2] - takeoff_height)
    origin_yaw = takeoff_target[3]
    expected_targets = preview_local_route._build_targets(
        section, frame, waypoints, origin, origin_yaw, include_takeoff=True)

    actual_by_key = {("TAKEOFF", 0): takeoff_target}
    for record in records:
        if not isinstance(record, dict) or record.get("state") != "WAYPOINTS":
            continue
        try:
            waypoint_index = int(record.get("waypoint_index"))
        except (TypeError, ValueError):
            continue
        target = _target_tuple(record)
        if target is not None:
            actual_by_key[("WAYPOINTS", waypoint_index)] = target

    missing = []
    max_position_delta = 0.0
    max_yaw_delta = 0.0
    position_tolerance = args.route_target_tolerance
    yaw_tolerance = math.radians(args.route_yaw_tolerance_deg)
    for index, expected in enumerate(expected_targets):
        key = ("TAKEOFF", 0) if index == 0 else ("WAYPOINTS", index - 1)
        actual = actual_by_key.get(key)
        if actual is None:
            missing.append("route_target_missing:%s%d" % (key[0].lower(), key[1]))
            continue
        expected_target = expected["target"]
        position_delta = math.sqrt(sum((actual[i] - expected_target[i]) ** 2 for i in range(3)))
        yaw_delta = abs(_angle_error(actual[3], expected_target[3]))
        max_position_delta = max(max_position_delta, position_delta)
        max_yaw_delta = max(max_yaw_delta, yaw_delta)
        if position_delta > position_tolerance or yaw_delta > yaw_tolerance:
            missing.append("route_target_mismatch:%s%d:pos=%.3f:yaw_deg=%.2f" % (
                key[0].lower(), key[1], position_delta, math.degrees(yaw_delta)))

    notes = []
    if not missing:
        notes.append("route_targets_match_config=%d max_pos_delta=%.3f max_yaw_delta_deg=%.2f" % (
            len(expected_targets), max_position_delta, math.degrees(max_yaw_delta)))
    return _phase("active_route_matches_config", not missing, missing, notes)


def _config_phase(readiness, name):
    phase = _find_phase(readiness, name)
    return _phase("config_%s" % name, phase["ready"], phase.get("missing"), phase.get("notes"))


def build_report(args):
    readiness = _readiness_report(args)
    readonly = _readonly_report(args)
    configured_waypoints = int(readiness.get("mission", {}).get("waypoints") or 0)
    active = _active_report(args, configured_waypoints)
    active_route_match = _active_route_match_report(args)

    phases = [
        _config_phase(readiness, "vision_output"),
        _config_phase(readiness, "active_local_flight"),
        _config_phase(readiness, "payload_local_flight"),
    ]

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
            "readonly_active_preflight_evidence",
        ),
        "active_local_flight": (
            "config_vision_output",
            "config_active_local_flight",
            "readonly_active_preflight_evidence",
            "active_local_flight_evidence",
            "active_route_matches_config",
        ),
        "payload_local_flight": (
            "config_vision_output",
            "config_active_local_flight",
            "config_payload_local_flight",
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
        "readonly_evidence": None if not args.readonly_evidence else str(pathlib.Path(args.readonly_evidence).expanduser().resolve()),
        "active_evidence": None if not args.active_evidence else str(pathlib.Path(args.active_evidence).expanduser().resolve()),
        "required_phase": args.require_phase,
        "required_phase_ready": phase_groups[args.require_phase],
        "phase_groups": phase_groups,
        "phases": phases,
    }


def _print_text(report):
    print("ROBOTAC_FLIGHT_GOAL_AUDIT config_root=%s" % report["config_root"])
    if report["readonly_evidence"]:
        print("readonly_evidence=%s" % report["readonly_evidence"])
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
    parser.add_argument("--readonly-evidence", default="",
                        help="Directory containing read-only topic evidence and ev_acceptance_observer.json")
    parser.add_argument("--active-evidence", default="",
                        help="active_flight_observer.json or directory containing it")
    parser.add_argument("--ev-acceptance-file", default="")
    parser.add_argument("--mavros-node", default="/mavros")
    parser.add_argument("--origin-x", type=float, default=0.0)
    parser.add_argument("--origin-y", type=float, default=0.0)
    parser.add_argument("--origin-z", type=float, default=0.0)
    parser.add_argument("--origin-yaw-deg", type=float, default=0.0)
    parser.add_argument("--min-local-hz", type=float, default=5.0)
    parser.add_argument("--min-fastlio-hz", type=float, default=5.0)
    parser.add_argument("--min-vision-hz", type=float, default=5.0)
    parser.add_argument("--min-timesync-hz", type=float, default=2.0)
    parser.add_argument("--min-waypoints", type=int, default=0,
                        help="Minimum observed active-flight waypoints; 0 uses the configured route count")
    parser.add_argument("--expected-waypoints", type=int, default=0,
                        help="Exact observed active-flight waypoint count; 0 uses the configured route count")
    parser.add_argument("--allow-dynamic-active-route", action="store_true",
                        help="Do not require active-flight evidence to match the configured route waypoint count")
    parser.add_argument("--route-target-tolerance", type=float, default=0.05,
                        help="Position tolerance in metres for matching active evidence targets to the configured route")
    parser.add_argument("--route-yaw-tolerance-deg", type=float, default=1.0,
                        help="Yaw tolerance in degrees for matching active evidence targets to the configured route")
    parser.add_argument("--min-setpoints", type=int, default=20)
    parser.add_argument("--min-unique-setpoints", type=int, default=2)
    parser.add_argument("--min-airborne-altitude", type=float, default=0.50)
    parser.add_argument("--waypoint-reach-tolerance", type=float, default=0.35)
    parser.add_argument("--min-target-dwell-s", type=float, default=0.25,
                        help="Require each TAKEOFF/WAYPOINTS target to remain within reach tolerance for this many continuous seconds")
    parser.add_argument("--require-phase", default="active_local_flight",
                        choices=("configuration", "active_preflight",
                                 "active_local_flight", "payload_local_flight"))
    parser.add_argument("--json", action="store_true")
    return parser


def main():
    args = _build_parser().parse_args()
    if args.min_waypoints < 0 or args.expected_waypoints < 0:
        raise ValueError("min-waypoints and expected-waypoints must be non-negative")
    if args.route_target_tolerance < 0.0 or args.route_yaw_tolerance_deg < 0.0:
        raise ValueError("route target/yaw tolerances must be non-negative")
    if args.min_target_dwell_s < 0.0:
        raise ValueError("min-target-dwell-s must be non-negative")
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

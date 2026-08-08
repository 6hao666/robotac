#!/usr/bin/env python3
"""Analyze Robotac active local-flight evidence.

The input is either ``active_flight_observer.json`` or a directory containing
that file. This analyzer only reads JSON; it never starts ROS, opens serial
devices, publishes topics, calls services, changes modes, arms, or sends
setpoints.
"""

import argparse
import json
import math
import pathlib
import sys


LANDED_STATE_ON_GROUND = 1


def _load(path):
    evidence_path = pathlib.Path(path).expanduser().resolve()
    if evidence_path.is_dir():
        evidence_path = evidence_path / "active_flight_observer.json"
    with evidence_path.open("r", encoding="utf-8") as stream:
        data = json.load(stream)
    if not isinstance(data, dict):
        raise ValueError("active-flight evidence must be a JSON object")
    return evidence_path, data


def _number(value):
    if value is None:
        return None
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _int(value):
    if value is None:
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _phase(name, ready, missing=None, notes=None):
    return {
        "name": name,
        "ready": bool(ready),
        "missing": list(missing or []),
        "notes": list(notes or []),
    }


def _target_reached(record, tolerance, min_dwell_s):
    distance = _number(record.get("min_distance_m"))
    dwell = _number(record.get("max_continuous_reach_s"))
    if min_dwell_s <= 0.0 and dwell is None:
        dwell = 0.0
    return (record.get("reached") is True and distance is not None and distance <= tolerance and
            dwell is not None and dwell >= min_dwell_s)


def _base_phase(data, args):
    summary = data.get("summary") if isinstance(data.get("summary"), dict) else {}
    last_status = summary.get("last_status") if isinstance(summary.get("last_status"), dict) else {}
    missing = []
    notes = []
    if data.get("observer") != "active_flight_observer":
        missing.append("observer_identity")
    if data.get("success") is not True:
        missing.append("observer_success")
    if data.get("reason") != "active_local_flight_passed":
        missing.append("observer_reason:%s" % data.get("reason"))
    if summary.get("abort_reason"):
        missing.append("flight_abort:%s" % summary.get("abort_reason"))
    if last_status.get("state") != "COMPLETE":
        missing.append("flight_state_complete")

    total_waypoints = _int(summary.get("total_waypoints"))
    max_waypoint_index = _int(summary.get("max_waypoint_index"))
    expected_waypoints = _int(getattr(args, "expected_waypoints", 0)) or 0
    if expected_waypoints > 0 and total_waypoints != expected_waypoints:
        missing.append("expected_waypoints_mismatch:%s!=%d" % (
            "unknown" if total_waypoints is None else total_waypoints, expected_waypoints))
    if total_waypoints is None or total_waypoints < args.min_waypoints:
        missing.append("total_waypoints_below_%d" % args.min_waypoints)
    if max_waypoint_index is None or total_waypoints is None or max_waypoint_index < total_waypoints:
        missing.append("waypoints_incomplete")
    else:
        notes.append("waypoints=%d/%d" % (max_waypoint_index, total_waypoints))

    setpoint_count = _int(summary.get("setpoint_count"))
    if setpoint_count is None or setpoint_count < args.min_setpoints:
        missing.append("setpoint_count_below_%d" % args.min_setpoints)
    else:
        notes.append("setpoints=%d" % setpoint_count)

    unique_setpoints = summary.get("unique_setpoints")
    unique_count = len(unique_setpoints) if isinstance(unique_setpoints, list) else 0
    if unique_count < args.min_unique_setpoints:
        missing.append("unique_setpoints_below_%d" % args.min_unique_setpoints)
    else:
        notes.append("unique_setpoints=%d" % unique_count)

    target_records = summary.get("target_records")
    flight_targets = []
    if isinstance(target_records, list):
        flight_targets = [record for record in target_records
                          if isinstance(record, dict) and
                          record.get("state") in ("TAKEOFF", "WAYPOINTS")]
    if not flight_targets:
        missing.append("target_records_missing")
    else:
        takeoff_targets = [record for record in flight_targets if record.get("state") == "TAKEOFF"]
        if not takeoff_targets:
            missing.append("takeoff_target_record_missing")
        waypoint_targets = [record for record in flight_targets if record.get("state") == "WAYPOINTS"]
        if total_waypoints is not None:
            reached_indices = set()
            for record in waypoint_targets:
                waypoint_index = _int(record.get("waypoint_index"))
                if waypoint_index is not None and _target_reached(
                        record, args.waypoint_reach_tolerance, args.min_target_dwell_s):
                    reached_indices.add(waypoint_index)
            missing_indices = [index for index in range(total_waypoints) if index not in reached_indices]
            if missing_indices:
                missing.append("waypoint_target_records_missing:%s" % ",".join(
                    str(index) for index in missing_indices))
            else:
                notes.append("waypoint_target_records=%d/%d" % (len(reached_indices), total_waypoints))
        unreached = []
        for index, record in enumerate(flight_targets):
            distance = _number(record.get("min_distance_m"))
            if not _target_reached(record, args.waypoint_reach_tolerance, args.min_target_dwell_s):
                label = "%s" % record.get("state")
                waypoint_index = _int(record.get("waypoint_index"))
                if waypoint_index is not None:
                    label = "%s[%d]" % (label, waypoint_index)
                dwell = _number(record.get("max_continuous_reach_s"))
                unreached.append("%d:%s:%s" % (
                    index, label,
                    "unknown" if distance is None else "%.3f/dwell=%.3f" % (
                        distance, -1.0 if dwell is None else dwell)))
        if unreached:
            missing.append("target_records_unreached:%s" % ";".join(unreached))
        else:
            notes.append("target_records_reached=%d tolerance=%.2f dwell=%.2f" % (
                len(flight_targets), args.waypoint_reach_tolerance, args.min_target_dwell_s))

    max_relative_z = _number(summary.get("max_relative_local_z"))
    if max_relative_z is None or max_relative_z < args.min_airborne_altitude:
        missing.append("relative_airborne_altitude_below_%.2f" % args.min_airborne_altitude)
    else:
        notes.append("max_relative_z=%.3f" % max_relative_z)

    if summary.get("final_armed") is not False:
        missing.append("final_disarmed")
    if summary.get("final_landed_state") != LANDED_STATE_ON_GROUND:
        missing.append("final_on_ground")
    if not missing:
        notes.append("complete/disarmed/on-ground")
    return _phase("active_local_flight", not missing, missing, notes)


def _payload_phase(data):
    summary = data.get("summary") if isinstance(data.get("summary"), dict) else {}
    missing = []
    notes = []
    if summary.get("payload_open_seen") is not True:
        missing.append("payload_open_seen")
    else:
        notes.append("payload_open_ack_seen")
    return _phase("payload_local_flight", not missing, missing, notes)


def build_report(args):
    path, data = _load(args.evidence)
    phases = [_base_phase(data, args), _payload_phase(data)]
    lookup = {phase["name"]: phase for phase in phases}
    if args.require_phase == "active_local_flight":
        required_ready = lookup["active_local_flight"]["ready"]
    else:
        required_ready = lookup["active_local_flight"]["ready"] and lookup["payload_local_flight"]["ready"]
    return {
        "evidence": str(path),
        "required_phase": args.require_phase,
        "required_phase_ready": required_ready,
        "phases": phases,
    }


def _print_text(report):
    print("ACTIVE_FLIGHT_EVIDENCE_ANALYSIS evidence=%s" % report["evidence"])
    for phase in report["phases"]:
        print("%s=%s" % (phase["name"], "READY" if phase["ready"] else "BLOCKED"))
        if phase["missing"]:
            print("  missing=%s" % ",".join(phase["missing"]))
        if phase["notes"]:
            print("  notes=%s" % "; ".join(phase["notes"]))
    print("required_phase=%s" % report["required_phase"])
    print("required_phase_ready=%s" % report["required_phase_ready"])


def _build_parser():
    parser = argparse.ArgumentParser(
        description="Analyze Robotac active local-flight observer evidence.")
    parser.add_argument("evidence", help="active_flight_observer.json or a directory containing it")
    parser.add_argument("--require-phase", default="active_local_flight",
                        choices=("active_local_flight", "payload_local_flight"))
    parser.add_argument("--min-waypoints", type=int, default=1)
    parser.add_argument("--expected-waypoints", type=int, default=0,
                        help="Require the observed mission waypoint count to exactly match this value; 0 disables exact matching")
    parser.add_argument("--min-setpoints", type=int, default=20)
    parser.add_argument("--min-unique-setpoints", type=int, default=2)
    parser.add_argument("--min-airborne-altitude", type=float, default=0.50)
    parser.add_argument("--waypoint-reach-tolerance", type=float, default=0.35)
    parser.add_argument("--min-target-dwell-s", type=float, default=0.25,
                        help="Require each TAKEOFF/WAYPOINTS target to remain within reach tolerance for this many continuous seconds")
    parser.add_argument("--json", action="store_true")
    return parser


def main():
    args = _build_parser().parse_args()
    if args.min_waypoints < 1 or args.min_setpoints < 1 or args.min_unique_setpoints < 1:
        raise ValueError("minimum waypoint/setpoint counts must be positive")
    if args.expected_waypoints < 0:
        raise ValueError("expected-waypoints must be non-negative")
    if not math.isfinite(args.min_airborne_altitude) or args.min_airborne_altitude < 0.0:
        raise ValueError("min-airborne-altitude must be finite and non-negative")
    if not math.isfinite(args.waypoint_reach_tolerance) or args.waypoint_reach_tolerance <= 0.0:
        raise ValueError("waypoint-reach-tolerance must be finite and positive")
    if not math.isfinite(args.min_target_dwell_s) or args.min_target_dwell_s < 0.0:
        raise ValueError("min-target-dwell-s must be finite and non-negative")
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

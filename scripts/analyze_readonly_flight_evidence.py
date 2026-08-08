#!/usr/bin/env python3
"""Analyze a Robotac read-only flight evidence bundle.

The input is a directory produced by ``collect_readonly_flight_evidence.sh``.
This analyzer only reads text files; it never starts ROS, opens serial devices,
publishes topics, calls services, changes modes, arms, or sends setpoints.
"""

import argparse
import json
import pathlib
import re
import sys


TOPIC_FILES = {
    "/mavros/state": "mavros_state",
    "/mavros/extended_state": "mavros_extended_state",
    "/mavros/local_position/odom": "mavros_local_position_odom",
    "/mavros/estimator_status": "mavros_estimator_status",
    "/mavros/timesync_status": "mavros_timesync_status",
    "/mavros/vision_pose/pose_cov": "mavros_vision_pose_pose_cov",
    "/mavros/setpoint_raw/local": "mavros_setpoint_raw_local",
    "/robotac/fastlio_vision/healthy": "robotac_fastlio_vision_healthy",
    "/robotac/fastlio_vision/status": "robotac_fastlio_vision_status",
    "/robotac/fastlio_vision/output_enabled": "robotac_fastlio_vision_output_enabled",
    "/robotac/fastlio_vision/pose_preview": "robotac_fastlio_vision_pose_preview",
    "/Odometry": "Odometry",
}

EXPECTED_TYPES = {
    "/mavros/state": "mavros_msgs/State",
    "/mavros/extended_state": "mavros_msgs/ExtendedState",
    "/mavros/local_position/odom": "nav_msgs/Odometry",
    "/mavros/estimator_status": "mavros_msgs/EstimatorStatus",
    "/mavros/timesync_status": "mavros_msgs/TimesyncStatus",
    "/mavros/vision_pose/pose_cov": "geometry_msgs/PoseWithCovarianceStamped",
    "/mavros/setpoint_raw/local": "mavros_msgs/PositionTarget",
    "/robotac/fastlio_vision/healthy": "std_msgs/Bool",
    "/robotac/fastlio_vision/status": "std_msgs/String",
    "/robotac/fastlio_vision/output_enabled": "std_msgs/Bool",
    "/robotac/fastlio_vision/pose_preview": "geometry_msgs/PoseWithCovarianceStamped",
    "/Odometry": "nav_msgs/Odometry",
}


def _read(directory, filename):
    path = pathlib.Path(directory) / filename
    if not path.exists():
        return ""
    return path.read_text(encoding="utf-8", errors="replace")


def _topic_file(kind, topic):
    return "topic_%s_%s.txt" % (kind, TOPIC_FILES[topic])


def _extract_average_hz(text):
    match = re.search(r"average rate:\s*([0-9]+(?:\.[0-9]+)?)", text)
    if not match:
        return None
    return float(match.group(1))


def _contains_bool(text, name, expected):
    wanted = "true" if expected else "false"
    pattern = r"(?:^|\n)\s*%s:\s*(?:['\"])?%s(?:['\"])?\s*(?:\n|$)" % (
        re.escape(name), wanted)
    return re.search(pattern, text, flags=re.IGNORECASE) is not None


def _contains_int(text, name, expected):
    pattern = r"(?:^|\n)\s*%s:\s*%d\s*(?:\n|$)" % (re.escape(name), expected)
    return re.search(pattern, text) is not None


def _string_data(text):
    match = re.search(r"(?:^|\n)\s*data:\s*(.*?)(?:\n|$)", text)
    if not match:
        return ""
    return match.group(1).strip().strip("'\"")


def _info_nodes(text, section):
    marker = section + ":"
    if marker not in text:
        return []
    tail = text.split(marker, 1)[1]
    next_sections = [index for index in (
        tail.find("Publishers:"), tail.find("Subscribers:")) if index > 0]
    if next_sections:
        tail = tail[:min(next_sections)]
    if re.search(r"^\s*None\s*$", tail, flags=re.MULTILINE):
        return []
    return re.findall(r"\*\s+(\S+)", tail)


def _info_has_node(text, section, node):
    return node in _info_nodes(text, section)


def _phase(name, ready, missing=None, notes=None):
    return {
        "name": name,
        "ready": bool(ready),
        "missing": list(missing or []),
        "notes": list(notes or []),
    }


def _type_phase(directory):
    missing = []
    notes = []
    for topic, expected_type in sorted(EXPECTED_TYPES.items()):
        text = _read(directory, _topic_file("type", topic))
        if expected_type not in text:
            missing.append("%s_type" % topic)
    if not missing:
        notes.append("all required topic types observed")
    return _phase("topic_types", not missing, missing, notes)


def _mavros_safe_state_phase(directory):
    state = _read(directory, _topic_file("echo", "/mavros/state"))
    extended = _read(directory, _topic_file("echo", "/mavros/extended_state"))
    missing = []
    notes = []
    if not _contains_bool(state, "connected", True):
        missing.append("mavros_connected")
    if not _contains_bool(state, "armed", False):
        missing.append("mavros_disarmed")
    if not _contains_int(extended, "landed_state", 1):
        missing.append("vehicle_on_ground")
    if not missing:
        mode_match = re.search(r"(?:^|\n)\s*mode:\s*(.*?)(?:\n|$)", state)
        if mode_match:
            notes.append("mode=%s" % mode_match.group(1).strip().strip("'\""))
        notes.append("connected/disarmed/on-ground")
    return _phase("mavros_safe_state", not missing, missing, notes)


def _stream_phase(directory, name, topic, min_hz):
    text = _read(directory, _topic_file("hz", topic))
    hz = _extract_average_hz(text)
    missing = []
    notes = []
    if hz is None:
        missing.append("%s_hz_missing" % topic)
    elif hz < min_hz:
        missing.append("%s_hz_below_%.2f" % (topic, min_hz))
        notes.append("hz=%.3f" % hz)
    else:
        notes.append("hz=%.3f" % hz)
    return _phase(name, not missing, missing, notes)


def _fastlio_health_phase(directory):
    health = _read(directory, _topic_file("echo", "/robotac/fastlio_vision/healthy"))
    status = _string_data(_read(directory, _topic_file("echo", "/robotac/fastlio_vision/status")))
    output = _read(directory, _topic_file("echo", "/robotac/fastlio_vision/output_enabled"))
    missing = []
    notes = []
    if not _contains_bool(health, "data", True):
        missing.append("fastlio_vision_healthy")
    if not (status == "ok" or status.startswith("ok ")):
        missing.append("fastlio_vision_status_ok")
    else:
        notes.append("status=%s" % status)
    if not _contains_bool(output, "data", True):
        missing.append("fastlio_vision_output_enabled")
    return _phase("fastlio_vision_health", not missing, missing, notes)


def _consumer_phase(directory, name, topic, consumer):
    text = _read(directory, _topic_file("info", topic))
    missing = []
    notes = []
    if not _info_has_node(text, "Subscribers", consumer):
        missing.append("%s_subscriber_%s" % (topic, consumer))
    else:
        notes.append("%s subscribes %s" % (consumer, topic))
    return _phase(name, not missing, missing, notes)


def _no_publishers_phase(directory, name, topic):
    text = _read(directory, _topic_file("info", topic))
    publishers = _info_nodes(text, "Publishers")
    missing = []
    notes = []
    if publishers:
        missing.append("%s_publishers_present:%s" % (topic, ",".join(publishers)))
    else:
        notes.append("no publishers on %s" % topic)
    return _phase(name, not missing, missing, notes)


def _ev_acceptance_phase(directory, path):
    evidence_path = pathlib.Path(path).expanduser() if path else pathlib.Path(directory) / "ev_acceptance_observer.json"
    missing = []
    notes = []
    if not evidence_path.exists():
        return _phase("ev_acceptance_observer", False,
                      ["ev_acceptance_observer_json"],
                      ["expected=%s" % evidence_path])
    try:
        data = json.loads(evidence_path.read_text(encoding="utf-8", errors="replace"))
    except Exception as exc:
        return _phase("ev_acceptance_observer", False,
                      ["ev_acceptance_observer_json_invalid:%s" % exc],
                      ["path=%s" % evidence_path])
    if not isinstance(data, dict):
        return _phase("ev_acceptance_observer", False,
                      ["ev_acceptance_observer_json_not_mapping"],
                      ["path=%s" % evidence_path])
    if data.get("observer") != "ev_acceptance_observer":
        missing.append("ev_acceptance_observer_identity")
    reason = str(data.get("reason", ""))
    if data.get("success") is not True:
        missing.append("ev_acceptance_failed:%s" % (reason or "unknown"))
    elif not reason.startswith("ev_acceptance_passed"):
        missing.append("ev_acceptance_reason:%s" % (reason or "empty"))
    parameters = data.get("parameters") if isinstance(data.get("parameters"), dict) else {}
    for key in (
            "require_connected",
            "require_disarmed",
            "require_on_ground",
            "require_vision_output_enabled",
            "require_vision_status_ok",
    ):
        if parameters.get(key) is not True:
            missing.append("ev_acceptance_%s" % key)
    if not missing:
        notes.append("path=%s" % evidence_path)
        notes.append(reason)
    return _phase("ev_acceptance_observer", not missing, missing, notes)


def build_report(args):
    directory = pathlib.Path(args.evidence_dir).expanduser().resolve()
    phases = [
        _type_phase(directory),
        _mavros_safe_state_phase(directory),
        _stream_phase(directory, "mavros_local_position_stream", "/mavros/local_position/odom",
                      args.min_local_hz),
        _stream_phase(directory, "fastlio_odometry_stream", "/Odometry", args.min_fastlio_hz),
        _stream_phase(directory, "mavros_vision_pose_stream", "/mavros/vision_pose/pose_cov",
                      args.min_vision_hz),
        _stream_phase(directory, "mavros_timesync_stream", "/mavros/timesync_status",
                      args.min_timesync_hz),
        _fastlio_health_phase(directory),
        _consumer_phase(directory, "mavros_vision_pose_consumer", "/mavros/vision_pose/pose_cov",
                        args.mavros_node),
        _consumer_phase(directory, "mavros_setpoint_raw_consumer", "/mavros/setpoint_raw/local",
                        args.mavros_node),
        _no_publishers_phase(directory, "read_only_no_setpoint_publishers", "/mavros/setpoint_raw/local"),
        _ev_acceptance_phase(directory, args.ev_acceptance_file),
    ]
    phase_lookup = {phase["name"]: phase for phase in phases}
    phase_groups = {
        "topic_types": ("topic_types",),
        "mavros_safe_state": ("topic_types", "mavros_safe_state"),
        "vision_to_mavros": (
            "topic_types",
            "mavros_safe_state",
            "fastlio_odometry_stream",
            "mavros_vision_pose_stream",
            "mavros_timesync_stream",
            "fastlio_vision_health",
            "mavros_vision_pose_consumer",
        ),
        "active_preflight_evidence": (
            "topic_types",
            "mavros_safe_state",
            "mavros_local_position_stream",
            "fastlio_odometry_stream",
            "mavros_vision_pose_stream",
            "mavros_timesync_stream",
            "fastlio_vision_health",
            "mavros_vision_pose_consumer",
            "mavros_setpoint_raw_consumer",
            "read_only_no_setpoint_publishers",
            "ev_acceptance_observer",
        ),
    }
    group_status = {}
    for group, names in phase_groups.items():
        group_status[group] = all(phase_lookup[name]["ready"] for name in names)
    return {
        "evidence_dir": str(directory),
        "required_phase": args.require_phase,
        "required_phase_ready": group_status[args.require_phase],
        "phase_groups": group_status,
        "phases": phases,
    }


def _print_text(report):
    print("READ_ONLY_EVIDENCE_ANALYSIS evidence_dir=%s" % report["evidence_dir"])
    for phase in report["phases"]:
        print("%s=%s" % (phase["name"], "READY" if phase["ready"] else "BLOCKED"))
        if phase["missing"]:
            print("  missing=%s" % ",".join(phase["missing"]))
        if phase["notes"]:
            print("  notes=%s" % "; ".join(phase["notes"]))
    for name, ready in sorted(report["phase_groups"].items()):
        print("%s=%s" % (name, "READY" if ready else "BLOCKED"))


def _build_parser():
    parser = argparse.ArgumentParser(
        description="Analyze a read-only Robotac flight evidence bundle.")
    parser.add_argument("evidence_dir", help="Directory produced by collect_readonly_flight_evidence.sh")
    parser.add_argument("--mavros-node", default="/mavros")
    parser.add_argument("--min-local-hz", type=float, default=5.0)
    parser.add_argument("--min-fastlio-hz", type=float, default=5.0)
    parser.add_argument("--min-vision-hz", type=float, default=5.0)
    parser.add_argument("--min-timesync-hz", type=float, default=2.0)
    parser.add_argument("--ev-acceptance-file", default="",
                        help="EV acceptance JSON; default: EVIDENCE_DIR/ev_acceptance_observer.json")
    parser.add_argument("--require-phase", default="active_preflight_evidence",
                        choices=("topic_types", "mavros_safe_state", "vision_to_mavros",
                                 "active_preflight_evidence"))
    parser.add_argument("--json", action="store_true")
    return parser


def main():
    args = _build_parser().parse_args()
    report = build_report(args)
    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        _print_text(report)
    if not report["required_phase_ready"]:
        return 2
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        print("ERROR: %s" % exc, file=sys.stderr)
        sys.exit(1)

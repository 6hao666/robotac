#!/usr/bin/env python3
"""Analyze FAST-LIO preview frame-alignment evidence.

Inputs are JSON files written by ``fastlio_frame_alignment_observer.py``. This
analyzer is offline/read-only: it only reads files and never starts ROS, opens
serial devices, publishes topics, calls services, changes modes, arms, or sends
setpoints.
"""

import argparse
import json
import math
import pathlib
import sys


AXIS_LABELS = {
    (1, 0, 0): "positive_x",
    (0, 1, 0): "positive_y",
    (0, 0, 1): "positive_z",
    (-1, 0, 0): "negative_x",
    (0, -1, 0): "negative_y",
    (0, 0, -1): "negative_z",
}


def _phase(name, ready, missing=None, notes=None):
    return {
        "name": name,
        "ready": bool(ready),
        "missing": list(missing or []),
        "notes": list(notes or []),
    }


def _number(value):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _read_json(path):
    try:
        data = json.loads(path.read_text(encoding="utf-8", errors="replace"))
    except Exception as exc:
        return None, "json_invalid:%s" % exc
    if not isinstance(data, dict):
        return None, "json_not_mapping"
    return data, None


def _iter_paths(inputs):
    paths = []
    for item in inputs:
        path = pathlib.Path(item).expanduser()
        if path.is_dir():
            paths.extend(sorted(child for child in path.glob("*.json") if child.is_file()))
        else:
            paths.append(path)
    return paths


def _axis_from_expected(parameters):
    values = (
        _number(parameters.get("expected_x")),
        _number(parameters.get("expected_y")),
        _number(parameters.get("expected_z")),
    )
    if any(value is None for value in values):
        return None
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1.0e-9:
        return None
    unit = tuple(value / norm for value in values)
    dominant = max(range(3), key=lambda index: abs(unit[index]))
    axis = [0, 0, 0]
    axis[dominant] = 1 if unit[dominant] >= 0.0 else -1
    # Require the expected vector itself to be close to a cardinal axis. This
    # keeps the deployment checklist auditable instead of accepting diagonal
    # motions as a substitute for per-axis checks.
    cardinal_cos = abs(unit[dominant])
    if cardinal_cos < math.cos(math.radians(10.0)):
        return "non_cardinal_translation"
    return AXIS_LABELS.get(tuple(axis))


def _validate_common(path, data, args):
    missing = []
    notes = []
    if data.get("observer") != "fastlio_frame_alignment_observer":
        missing.append("observer_identity")
    if data.get("success") is not True:
        missing.append("observer_failed:%s" % data.get("reason", "unknown"))
    reason = str(data.get("reason", ""))
    if data.get("success") is True and not reason.startswith("frame_alignment_preview_passed"):
        missing.append("observer_reason:%s" % (reason or "empty"))
    parameters = data.get("parameters") if isinstance(data.get("parameters"), dict) else {}
    metrics = data.get("metrics") if isinstance(data.get("metrics"), dict) else {}
    if parameters.get("pose_topic") != args.pose_topic:
        missing.append("pose_topic:%s" % parameters.get("pose_topic"))
    if parameters.get("expected_parent") != args.expected_parent:
        missing.append("expected_parent:%s" % parameters.get("expected_parent"))
    if parameters.get("require_mavros_output_disabled") is not True:
        missing.append("mavros_output_disabled_not_required")
    if args.require_vision_status_ok and parameters.get("require_vision_status_ok") is not True:
        missing.append("vision_status_ok_not_required")
    pose_rate = _number(metrics.get("pose_rate_hz"))
    if pose_rate is None:
        missing.append("pose_rate_missing")
    elif pose_rate < args.min_pose_rate_hz:
        missing.append("pose_rate:%.3f<%.3f" % (pose_rate, args.min_pose_rate_hz))
    else:
        notes.append("pose_rate=%.3f" % pose_rate)
    notes.append("file=%s" % path)
    return missing, notes, parameters, metrics


def _translation_phase(label, path, data, args):
    missing, notes, parameters, metrics = _validate_common(path, data, args)
    if parameters.get("motion_type") != "translation" or metrics.get("motion_type") != "translation":
        missing.append("motion_type_not_translation")
    distance = _number(metrics.get("translation_distance_m"))
    if distance is None:
        missing.append("translation_distance_missing")
    elif distance < args.min_translation_m:
        missing.append("translation_distance:%.3f<%.3f" % (distance, args.min_translation_m))
    direction_error = _number(metrics.get("direction_error_deg"))
    if direction_error is None:
        missing.append("direction_error_missing")
    elif direction_error > args.max_direction_error_deg:
        missing.append("direction_error_deg:%.2f>%.2f" % (
            direction_error, args.max_direction_error_deg))
    projection = _number(metrics.get("projection_m"))
    if projection is None:
        missing.append("projection_missing")
    elif projection <= 0.0:
        missing.append("projection_not_positive:%.3f" % projection)
    scale = _number(metrics.get("delta_scale"))
    expected_distance = _number(parameters.get("expected_distance_m"))
    if expected_distance is not None and expected_distance > 0.0:
        if scale is None:
            missing.append("delta_scale_missing")
        elif scale < args.min_delta_scale or scale > args.max_delta_scale:
            missing.append("delta_scale:%.3f_not_in_%.3f..%.3f" % (
                scale, args.min_delta_scale, args.max_delta_scale))
    if not missing:
        notes.append("distance=%.3f direction_error_deg=%.2f" % (distance, direction_error))
    return _phase("translation_%s" % label, not missing, missing, notes)


def _yaw_phase(path, data, args):
    missing, notes, parameters, metrics = _validate_common(path, data, args)
    if parameters.get("motion_type") != "yaw" or metrics.get("motion_type") != "yaw":
        missing.append("motion_type_not_yaw")
    yaw_delta = _number(metrics.get("observed_yaw_delta_deg"))
    if yaw_delta is None:
        missing.append("yaw_delta_missing")
    elif abs(yaw_delta) < args.min_yaw_deg:
        missing.append("yaw_delta:%.2f<%.2f" % (abs(yaw_delta), args.min_yaw_deg))
    expected_sign = int(parameters.get("expected_yaw_sign") or 0)
    if expected_sign not in (-1, 0, 1):
        missing.append("expected_yaw_sign_invalid:%s" % parameters.get("expected_yaw_sign"))
    elif expected_sign != 0 and yaw_delta is not None and yaw_delta * expected_sign <= 0.0:
        missing.append("yaw_wrong_direction:%.2f_sign_%d" % (yaw_delta, expected_sign))
    yaw_error = _number(metrics.get("yaw_error_deg"))
    expected_yaw = _number(parameters.get("expected_yaw_deg"))
    if expected_yaw is not None and abs(expected_yaw) > 1.0e-9:
        if yaw_error is None:
            missing.append("yaw_error_missing")
        elif yaw_error > args.max_yaw_error_deg:
            missing.append("yaw_error_deg:%.2f>%.2f" % (yaw_error, args.max_yaw_error_deg))
    if not missing:
        notes.append("yaw_delta_deg=%.2f" % yaw_delta)
    return _phase("yaw_alignment", not missing, missing, notes)


def build_report(args):
    paths = _iter_paths(args.evidence)
    file_phases = []
    translations = {}
    yaw_candidates = []
    missing = []
    if not paths:
        missing.append("frame_alignment_evidence_files")
    for path in paths:
        data, issue = _read_json(path)
        if issue:
            file_phases.append(_phase("file_%s" % path.name, False, [issue], ["file=%s" % path]))
            continue
        parameters = data.get("parameters") if isinstance(data.get("parameters"), dict) else {}
        motion_type = parameters.get("motion_type")
        if motion_type == "translation":
            label = _axis_from_expected(parameters)
            if label is None:
                phase = _phase("translation_unknown", False,
                               ["expected_axis_missing"], ["file=%s" % path])
            elif label == "non_cardinal_translation":
                phase = _phase("translation_non_cardinal", False,
                               ["expected_axis_non_cardinal"], ["file=%s" % path])
            else:
                phase = _translation_phase(label, path, data, args)
                if phase["ready"]:
                    translations[label] = phase
            file_phases.append(phase)
        elif motion_type == "yaw":
            phase = _yaw_phase(path, data, args)
            if phase["ready"]:
                yaw_candidates.append(phase)
            file_phases.append(phase)
        else:
            file_phases.append(_phase("file_%s" % path.name, False,
                                      ["motion_type_unknown:%s" % motion_type],
                                      ["file=%s" % path]))

    required_axes = [item.strip() for item in args.required_axes.split(",") if item.strip()]
    axis_phases = []
    for label in required_axes:
        phase = translations.get(label)
        if phase is None:
            axis_phases.append(_phase("translation_%s_required" % label, False,
                                      ["translation_%s_evidence" % label]))
        else:
            axis_phases.append(_phase("translation_%s_required" % label, True,
                                      notes=phase.get("notes")))
    yaw_phase = None
    if args.require_yaw:
        yaw_phase = yaw_candidates[0] if yaw_candidates else _phase(
            "yaw_alignment_required", False, ["yaw_alignment_evidence"])
    phases = []
    if missing:
        phases.append(_phase("evidence_files", False, missing))
    phases.extend(file_phases)
    phases.extend(axis_phases)
    if yaw_phase is not None:
        phases.append(yaw_phase)
    required_names = [phase["name"] for phase in axis_phases]
    if args.require_yaw and yaw_phase is not None:
        required_names.append(yaw_phase["name"])
    required_lookup = {phase["name"]: phase for phase in phases}
    ready = bool(required_names) and all(required_lookup[name]["ready"] for name in required_names)
    if missing:
        ready = False
    return {
        "evidence": [str(path.expanduser().resolve()) for path in paths],
        "required_axes": required_axes,
        "require_yaw": bool(args.require_yaw),
        "required_phase_ready": ready,
        "phases": phases,
    }


def _print_text(report):
    print("FRAME_ALIGNMENT_EVIDENCE_ANALYSIS")
    for path in report["evidence"]:
        print("evidence=%s" % path)
    print("required_axes=%s" % ",".join(report["required_axes"]))
    print("require_yaw=%s" % report["require_yaw"])
    for phase in report["phases"]:
        print("%s=%s" % (phase["name"], "READY" if phase["ready"] else "BLOCKED"))
        if phase["missing"]:
            print("  missing=%s" % ",".join(phase["missing"]))
        if phase["notes"]:
            print("  notes=%s" % "; ".join(phase["notes"]))
    print("required_phase_ready=%s" % report["required_phase_ready"])


def _build_parser():
    parser = argparse.ArgumentParser(
        description="Analyze FAST-LIO preview frame-alignment JSON evidence.")
    parser.add_argument("evidence", nargs="+",
                        help="Evidence JSON file(s) or directories containing *.json")
    parser.add_argument("--required-axes", default="positive_x,positive_y,positive_z",
                        help="Comma-separated cardinal translation evidence labels")
    parser.add_argument("--require-yaw", action="store_true")
    parser.add_argument("--pose-topic", default="/robotac/fastlio_vision/path_a_pose_preview")
    parser.add_argument("--expected-parent", default="odom")
    parser.add_argument("--min-pose-rate-hz", type=float, default=5.0)
    parser.add_argument("--min-translation-m", type=float, default=0.30)
    parser.add_argument("--max-direction-error-deg", type=float, default=25.0)
    parser.add_argument("--min-delta-scale", type=float, default=0.50)
    parser.add_argument("--max-delta-scale", type=float, default=2.00)
    parser.add_argument("--min-yaw-deg", type=float, default=20.0)
    parser.add_argument("--max-yaw-error-deg", type=float, default=25.0)
    parser.add_argument("--require-vision-status-ok", action="store_true", default=True)
    parser.add_argument("--json", action="store_true")
    return parser


def _validate_args(args):
    for name in (
            "min_pose_rate_hz", "min_translation_m", "max_direction_error_deg",
            "min_delta_scale", "max_delta_scale", "min_yaw_deg", "max_yaw_error_deg"):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError("%s must be finite and positive" % name)
    if args.max_direction_error_deg >= 90.0:
        raise ValueError("max_direction_error_deg must be below 90 degrees")
    if args.min_delta_scale > args.max_delta_scale:
        raise ValueError("min_delta_scale must be <= max_delta_scale")


def main():
    args = _build_parser().parse_args()
    _validate_args(args)
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

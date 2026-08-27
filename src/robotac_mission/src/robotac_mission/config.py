"""mission.yaml 加载与校验（纯 Python，不依赖 rospy）。

- load_config(path)：从磁盘读取并校验，成功返回 dict；失败抛 ConfigError。
- 骨架轮使用：BOOT 阶段调用；mission_reset（ERROR -> BOOT）时由 mission_node
  重新从磁盘读取，不复用内存缓存。
"""

import os

import yaml


class ConfigError(ValueError):
    pass


_REQUIRED_SECTIONS = (
    "frames", "limits", "obstacle", "tables", "timing",
    "landing", "tag", "waypoints", "payload", "control", "mission",
)

_STAGE_TIMEOUT_KEYS = (
    "takeoff", "transit", "search", "align", "release", "return", "land",
)

# 全部启动门话题的数据年龄阈值键（timing.topic_timeout）
_TOPIC_TIMEOUT_KEYS = (
    "fcu_state", "extended_state", "estimator_status",
    "timesync_status", "vision_healthy", "vision_state",
)

_ROUTING_KEYS = ("approach", "gap_enter", "gap_cross", "resume")


def load_config(path):
    """读取并校验 mission.yaml。失败抛 ConfigError。"""
    if not os.path.isfile(path):
        raise ConfigError("配置文件不存在: %s" % path)
    try:
        with open(path, "r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream)
    except yaml.YAMLError as exc:
        raise ConfigError("YAML 解析失败: %s" % exc)
    if not isinstance(data, dict):
        raise ConfigError("配置根节点必须为映射")
    validate(data)
    return data


def validate(data):
    """结构校验：缺少必填节/字段、数值非法、航点越界均抛 ConfigError。"""
    for section in _REQUIRED_SECTIONS:
        if section not in data:
            raise ConfigError("缺少配置节: %s" % section)
        if not isinstance(data[section], dict):
            raise ConfigError("配置节 %s 必须为映射" % section)

    frames = data["frames"]
    _require_str(frames, "mission_frame")
    _require_str(frames, "body_frame")

    limits = data["limits"]
    field_min = _require_vec(limits, "field_min", 3)
    field_max = _require_vec(limits, "field_max", 3)
    for index in range(3):
        if not field_min[index] < field_max[index]:
            raise ConfigError("limits.field_min[%d] 必须小于 field_max" % index)
    _require_positive(limits, "max_speed")

    obstacle = data["obstacle"]
    obstacle_center = _require_vec(obstacle, "center", 2)
    obstacle_size = _require_vec(obstacle, "size", 3)
    if any(value <= 0.0 for value in obstacle_size):
        raise ConfigError("obstacle.size 必须全部为正数")
    _check_in_field(obstacle_center + [0.0], field_min, field_max,
                    "obstacle.center")
    _require_positive(obstacle, "cross_gap")
    _require_positive(obstacle, "route_clearance")
    if not isinstance(obstacle.get("no_overfly"), bool):
        raise ConfigError("obstacle.no_overfly 必须为布尔")

    tables = data["tables"]
    _require_vec(tables, "takeoff_center", 2)
    _require_vec(tables, "delivery_center", 2)
    _require_positive(tables, "height")
    _require_positive(tables, "search_radius")

    timing = data["timing"]
    _require_positive(timing, "pose_timeout")
    _require_positive(timing, "tag_timeout")
    _require_positive(timing, "max_rtt_ms")
    _require_positive(timing, "total_window")
    _require_positive(timing, "flight_budget")
    _require_nonnegative(timing, "takeoff_hold")
    _require_nonnegative(timing, "align_hold")
    _require_nonnegative(timing, "waypoint_hold")
    _require_nonnegative(timing, "release_hold")
    _require_positive(timing, "land_confirm")
    stage = timing["stage_timeout"]
    if not isinstance(stage, dict):
        raise ConfigError("timing.stage_timeout 必须为映射")
    for key in _STAGE_TIMEOUT_KEYS:
        if key not in stage:
            raise ConfigError("timing.stage_timeout 缺少 %s" % key)
        _require_positive(stage, key)
    topic_timeout = timing.get("topic_timeout")
    if not isinstance(topic_timeout, dict):
        raise ConfigError("timing.topic_timeout 必须为映射")
    for key in _TOPIC_TIMEOUT_KEYS:
        if key not in topic_timeout:
            raise ConfigError("timing.topic_timeout 缺少 %s" % key)
        _require_positive(topic_timeout, key)

    landing = data["landing"]
    _require_positive(landing, "mode_retry_seconds")
    if (not isinstance(landing.get("mode_retry_count"), int) or
            landing["mode_retry_count"] < 1):
        raise ConfigError("landing.mode_retry_count 必须为正整数")
    if (not isinstance(landing.get("confirm_samples"), int) or
            landing["confirm_samples"] < 2):
        raise ConfigError("landing.confirm_samples 必须为至少 2 的整数")
    _require_positive(landing, "max_height")
    _require_positive(landing, "max_vertical_speed")
    _require_positive(landing, "velocity_timeout")

    tag = data["tag"]
    _require_str(tag, "family")
    if not isinstance(tag.get("id"), int) or tag["id"] < 0:
        raise ConfigError("tag.id 必须为非负整数")
    _require_positive(tag, "black_size_m")
    _require_nonnegative(tag, "stable_time")
    if not isinstance(tag.get("stable_samples"), int) or tag["stable_samples"] < 1:
        raise ConfigError("tag.stable_samples 必须为正整数（多样本均值窗口，§16.10）")

    waypoints = data["waypoints"]
    takeoff = _require_vec(waypoints, "takeoff", 3)
    _check_in_field(takeoff, field_min, field_max, "waypoints.takeoff")
    for name in ("obstacle_routing", "return_routing"):
        routing = waypoints.get(name)
        if not isinstance(routing, dict):
            raise ConfigError("waypoints.%s 必须为映射" % name)
        for key in _ROUTING_KEYS:
            if key not in routing:
                raise ConfigError("waypoints.%s 缺少 %s" % (name, key))
            point = _require_vec(routing, key, 3)
            _check_in_field(point, field_min, field_max,
                            "waypoints.%s.%s" % (name, key))
    for name in ("mission", "return"):
        points = waypoints.get(name)
        if not isinstance(points, list) or not points:
            raise ConfigError("waypoints.%s 必须为非空列表" % name)
        for index, point in enumerate(points):
            point = _require_vec_raw(point, "waypoints.%s[%d]" % (name, index), 3)
            _check_in_field(point, field_min, field_max,
                            "waypoints.%s[%d]" % (name, index))

    _check_route_clearance(
        [takeoff] + [waypoints["obstacle_routing"][key]
                     for key in _ROUTING_KEYS] + [waypoints["mission"][0]],
        obstacle_center, obstacle_size, obstacle["route_clearance"],
        "去程")
    _check_route_clearance(
        [waypoints["mission"][0]] + [waypoints["return_routing"][key]
                                       for key in _ROUTING_KEYS] +
        [waypoints["return"][0]],
        obstacle_center, obstacle_size, obstacle["route_clearance"],
        "返程")

    # 桌上方航点（takeoff / mission / return 首个）z 不得低于桌面高：起飞/对准投放/
    # 返航均在桌面上方稳定，z 低于桌面即撞桌（R5-2；占位值曾为 0.6 < 桌面 0.75）。
    # 不写死裕量（避免现场常量），填正式值须加裕量并在 README §8 注明。
    table_height = tables["height"]
    for label, point in (
            ("waypoints.takeoff", takeoff),
            ("waypoints.mission[0]", waypoints["mission"][0]),
            ("waypoints.return[0]", waypoints["return"][0])):
        if point[2] < table_height:
            raise ConfigError(
                "%s z=%g 低于桌面高 %g：桌上方航点须在桌面之上（C1 保持 1.0-2.0m）"
                % (label, point[2], table_height))

    payload = data["payload"]
    if not isinstance(payload.get("enable"), bool):
        raise ConfigError("payload.enable 必须为布尔")
    if not isinstance(payload.get("retry_count"), int) or payload["retry_count"] < 0:
        raise ConfigError("payload.retry_count 必须为非负整数")

    control = data["control"]
    for key in ("rate_hz", "position_tolerance", "max_step",
                "prestream_seconds", "health_debounce",
                "tag_jump_limit", "pose_jump_limit"):
        _require_positive(control, key)
    if not isinstance(control["health_debounce"], int):
        raise ConfigError("control.health_debounce 必须为整数")

    mission = data["mission"]
    if not isinstance(mission.get("dry_run"), bool):
        raise ConfigError("mission.dry_run 必须为布尔")
    if not isinstance(mission.get("flight_enabled"), bool):
        raise ConfigError("mission.flight_enabled 必须为布尔")
    if "route_only" in mission and not isinstance(mission["route_only"], bool):
        raise ConfigError("mission.route_only 必须为布尔")


def _is_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _require_str(section, key):
    value = section.get(key)
    if not isinstance(value, str) or not value:
        raise ConfigError("%s 必须为非空字符串" % key)


def _require_vec(section, key, length):
    return _require_vec_raw(section.get(key), key, length)


def _require_vec_raw(value, label, length):
    if (not isinstance(value, (list, tuple)) or len(value) != length or
            not all(_is_number(item) for item in value)):
        raise ConfigError("%s 必须为长度为 %d 的数值列表" % (label, length))
    return [float(item) for item in value]


def _require_positive(section, key):
    value = section.get(key)
    if not _is_number(value) or float(value) <= 0:
        raise ConfigError("%s 必须为正数" % key)


def _require_nonnegative(section, key):
    value = section.get(key)
    if not _is_number(value) or float(value) < 0:
        raise ConfigError("%s 必须为非负数" % key)


def _check_in_field(point, field_min, field_max, label):
    for index in range(3):
        if not (field_min[index] <= point[index] <= field_max[index]):
            raise ConfigError("%s 坐标 %g 超出场地边界" % (label, point[index]))


def _check_route_clearance(points, center, size, clearance, label):
    """拒绝水平航段穿过障碍物及其机体/定位裕量膨胀包围盒。"""
    half_x = size[0] * 0.5 + clearance
    half_y = size[1] * 0.5 + clearance
    minimum = (center[0] - half_x, center[1] - half_y)
    maximum = (center[0] + half_x, center[1] + half_y)
    for index, (start, end) in enumerate(zip(points, points[1:])):
        if _segment_intersects_box(start[:2], end[:2], minimum, maximum):
            raise ConfigError(
                "%s航段[%d] 穿过障碍物安全裕量包围盒" % (label, index))


def _segment_intersects_box(start, end, minimum, maximum):
    """二维线段与闭合 AABB 是否相交（Liang-Barsky 裁剪）。"""
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    lower, upper = 0.0, 1.0
    for p, q in ((-dx, start[0] - minimum[0]),
                 (dx, maximum[0] - start[0]),
                 (-dy, start[1] - minimum[1]),
                 (dy, maximum[1] - start[1])):
        if p == 0.0:
            if q < 0.0:
                return False
            continue
        value = q / p
        if p < 0.0:
            if value > upper:
                return False
            lower = max(lower, value)
        else:
            if value < lower:
                return False
            upper = min(upper, value)
    return True

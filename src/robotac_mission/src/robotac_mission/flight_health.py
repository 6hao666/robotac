"""飞行健康门：飞行态 20Hz 检查（FlightDriver 调用）。

与预启动门（guards.readiness）分离：飞行中飞机已解锁离地，判据不同——
mode_ok（OFFBOARD 保持）、位姿跳变、场地边界（map_to_field 后）、软项去抖
（瞬态类连续 health_debounce 次才中止，06 修复；硬项立即中止）。

纯逻辑不依赖 rospy：数据经 ctx（节点消息快照）传入，便于后续拆测。
state 为可变去抖状态（soft_bad / prev_pose），由调用方持有。
"""

from robotac_mission import guards


def check(ctx, coord, config, state):
    """返回 None（健康）或失败原因字符串。"""
    timing = config["timing"]
    control = config["control"]
    if ctx.fcu_state is None or not ctx.fcu_state.connected:
        return "飞控未连接"
    if ctx.topic_age("fcu_state") > timing["topic_timeout"]["fcu_state"]:
        return "飞控状态过期"
    if ctx.pose is None or ctx.topic_age("pose") > timing["pose_timeout"]:
        return "本地位置过期"
    estimator = ctx.estimator
    if estimator is None or ctx.topic_age("estimator") > (
            timing["topic_timeout"]["estimator_status"]):
        return "PX4 estimator 状态过期"
    if not guards.estimator_ok({
            "attitude": estimator.attitude_status_flag,
            "pos_horiz_rel": estimator.pos_horiz_rel_status_flag,
            "pos_vert_abs": estimator.pos_vert_abs_status_flag})[0]:
        return "PX4 estimator 异常"
    ok, reason = guards.mode_ok(ctx.fcu_state.mode, ctx.fcu_state.armed)
    if not ok:
        return reason
    # L3：视觉桥 state 持续非 OK 立即中止（硬门，同预启动语义）
    if (ctx.vision_state is None or ctx.topic_age("vision_state") >
            timing["topic_timeout"]["vision_state"]):
        return "外部视觉状态过期"
    if ctx.vision_state != "OK":
        return "外部视觉状态非 OK"
    issue = None
    timesync = ctx.timesync
    if (timesync is None or ctx.topic_age("timesync") >
            timing["topic_timeout"]["timesync_status"] or
            timesync.round_trip_time_ms > timing["max_rtt_ms"]):
        issue = "时间同步"
    elif (ctx.vision_healthy is None or not ctx.vision_healthy or
          ctx.topic_age("vision_healthy") >
          timing["topic_timeout"]["vision_healthy"]):
        issue = "外部视觉"
    if issue is not None:
        state["soft_bad"] += 1
        if state["soft_bad"] >= control["health_debounce"]:
            return issue
    else:
        state["soft_bad"] = 0
    if ctx.pose is not None:
        current = (ctx.pose.pose.position.x, ctx.pose.pose.position.y,
                   ctx.pose.pose.position.z)
        ok, reason = guards.pose_jump(state["prev_pose"], current,
                                      control["pose_jump_limit"])
        state["prev_pose"] = current
        if not ok:
            return reason
        limits = config["limits"]
        # 规则以边线内侧为界，不能为数值误差把允许区域扩大到场外；配置的是
        # 向内缩的安全裕量。标定误差必须通过 field_yaw/home 标定修正。
        margin = limits.get("boundary_margin", 0.0)
        pose_field = coord.map_to_field(current)
        for index in range(3):
            if not (limits["field_min"][index] + margin <= pose_field[index]
                    <= limits["field_max"][index] - margin):
                return "超出场地边界"
        obstacle = config["obstacle"]
        if obstacle["no_overfly"]:
            length, thickness, _height = obstacle["size"]
            center_x, center_y = obstacle["center"]
            if (center_x - length * 0.5 <= pose_field[0] <=
                    center_x + length * 0.5 and
                    center_y - thickness * 0.5 <= pose_field[1] <=
                    center_y + thickness * 0.5):
                # 规则按水平投影判定，飞得比障碍高同样属于顶部越障。
                return "进入固定障碍物水平投影范围（禁止顶部越障）"
    return None

"""安全门（guards）：数据年龄、边界、飞控连接、定位健康、Tag 超时、模式丢失等。

设计：纯函数，输入为从 ROS 消息提取的普通值（不依赖 rospy），便于单元测试。
每个守卫返回 (ok: bool, reason: str)；reason 为空串表示通过。

骨架轮启用的安全门：fcu_connected / not_armed / on_ground / pose_valid /
estimator_ok / timesync_ok / vision_healthy。mode_ok、window_ok、tf_chain_ok
为预留项，飞行轮启用。
"""

import math

# PX4 estimator 关键 flag 允许组合（mavros_msgs/EstimatorStatus）
REQUIRED_ESTIMATOR_FLAGS = ("attitude", "pos_horiz_rel", "pos_vert_abs")

# 外部视觉桥 state 取值（vision_pose_bridge）
VISION_STATE_OK = "OK"


def fcu_connected(connected):
    """/mavros/state.connected 必须为 True。"""
    if not bool(connected):
        return False, "飞控未连接"
    return True, ""


def not_armed(armed):
    """禁止在已解锁状态启动。"""
    if bool(armed):
        return False, "飞机已解锁"
    return True, ""


def on_ground(landed_state, on_ground_state=1):
    """仅 landed_state == LANDED_STATE_ON_GROUND(=1) 允许启动。"""
    if landed_state != on_ground_state:
        return False, "飞机不在地面"
    return True, ""


def pose_valid(position, age, field_min, field_max, max_age):
    """本地位姿：数据年龄、有限数、场地边界。

    position: (x, y, z)；age: 秒（0 <= age <= max_age）。"""
    if age < 0.0 or age > max_age:
        return False, "本地位置过期"
    try:
        values = [float(value) for value in position]
    except (TypeError, ValueError):
        return False, "本地位置类型错误"
    if not all(math.isfinite(value) for value in values):
        return False, "本地位置含非有限数"
    for index in range(3):
        if not (field_min[index] <= values[index] <= field_max[index]):
            return False, "本地位置超出场地边界"
    return True, ""


def estimator_ok(flags):
    """关键 estimator flag 必须全部为 True。flags: 键为 flag 名的 dict。"""
    missing = [name for name in REQUIRED_ESTIMATOR_FLAGS
               if not bool(flags.get(name, False))]
    if missing:
        return False, "PX4 estimator 异常：" + ", ".join(missing)
    return True, ""


def timesync_ok(rtt_ms, max_rtt_ms):
    """timesync 往返时延必须在 [0, max_rtt_ms] 内。"""
    if rtt_ms is None or rtt_ms < 0.0 or rtt_ms > max_rtt_ms:
        return False, "时间同步超限"
    return True, ""


def topic_fresh(age, max_age, label="话题"):
    """话题数据年龄：0 <= age <= max_age 为新鲜（age 为秒）。

    用于全部启动门话题的"静默死亡"检测：某话题停止发布后守卫必须变红。
    age 由节点按到达时刻计算（不依赖发送方 header.stamp——Bool/String 无
    header，且第三方 fake 仅 pose 打了时间戳）。"""
    if age is None or age < 0.0 or age > max_age:
        return False, "%s数据过期" % label
    return True, ""


def vision_healthy(healthy, state):
    """外部视觉：healthy 必须为布尔 True，且 state 必须为 OK。"""
    if not isinstance(healthy, bool) or not healthy:
        return False, "外部视觉不健康"
    if state != VISION_STATE_OK:
        return False, "外部视觉状态非 OK"
    return True, ""


def readiness(checks):
    """聚合一组 (ok, reason)。返回 (全部通过, 失败原因列表)。"""
    failed = [reason for ok, reason in checks if not ok]
    return (not failed), failed

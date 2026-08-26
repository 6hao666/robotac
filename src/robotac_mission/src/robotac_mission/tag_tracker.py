"""Tag 检测 → map 系位姿 + 多样本稳定过滤（C3 依赖 TF 链，硬依赖②）。

与示例 TagTracker/StableTag 同语义（§16.10）：/tag_detections → TF(map) →
连续 stable_samples 个样本取均值；样本跳变超限重开窗口。fresh 受数据年龄门控：
age（最近 tag 消息到达龄）超过 tag_timeout 判超龄——发布流冻结时不再推进（M3，
避免陈旧样本推进 SEARCH/ALIGN）。TF 链不可用时（示例 04 曾失败）pose_map 恒 None，
SEARCH 超时 → ABORT——结构在、真实行为依赖 C3 TF/外参解锁后验证。
"""

import math
import time

import rospy
import tf2_geometry_msgs
import tf2_ros
from geometry_msgs.msg import PoseStamped


class TagTracker(object):
    def __init__(self, config, tf_buffer):
        self.config = config
        self.tf_buffer = tf_buffer
        self.pose_map = None   # (x, y, z) map 系，最近稳定均值
        self.fresh = False     # 本次 update 是否产生稳定位姿
        self._samples = []     # 未满稳定窗口前的候选样本
        self._last_sample = None
        self._last_message_stamp = None
        self._stable_since = None

    def update(self, detections, age):
        """处理一次检测数组。age: 最近 tag 消息到达龄（s），防冻结流推进。

        返回本次是否产生稳定位姿（fresh）。"""
        self.fresh = False
        if age > self.config["timing"]["tag_timeout"]:
            self._samples = []   # 发布流冻结/超龄：清窗，不推进
            self._stable_since = None
            return False
        if detections is None or not getattr(detections, "detections", None):
            return False
        stamp = self._stamp_key(detections)
        if stamp is not None and stamp == self._last_message_stamp:
            # FlightDriver 比相机快；同一条 ROS 消息不能被重复计入稳定样本。
            self.fresh = self.pose_map is not None
            return self.fresh
        self._last_message_stamp = stamp
        candidate = self._to_map(detections)
        if candidate is None:
            return False
        if (self._last_sample is not None and
                self._dist3(candidate, self._last_sample) >
                self.config["control"]["tag_jump_limit"]):
            self._samples = []   # 场景切换/跳变：重开窗口
            self._stable_since = None
        if not self._samples:
            self._stable_since = time.monotonic()
        self._samples.append(candidate)
        self._last_sample = candidate
        if (len(self._samples) >= self.config["tag"]["stable_samples"] and
                time.monotonic() - self._stable_since >=
                self.config["tag"]["stable_time"]):
            count = len(self._samples)
            self.pose_map = tuple(
                sum(sample[i] for sample in self._samples) / count
                for i in range(3))
            self.fresh = True
        return self.fresh

    def _to_map(self, detections):
        for detection in detections.detections:
            ids = getattr(detection, "id", ())
            if not isinstance(ids, (list, tuple)):
                ids = (ids,)
            if not any(int(detected_id) == self.config["tag"]["id"]
                       for detected_id in ids):
                continue
            source = PoseStamped()
            source.header = detection.pose.header
            if not source.header.frame_id:
                source.header = detections.header
            source.pose = detection.pose.pose.pose
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.config["frames"]["mission_frame"],
                    source.header.frame_id, source.header.stamp,
                    rospy.Duration(0.15))
            except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                    tf2_ros.ConnectivityException):
                continue
            local = tf2_geometry_msgs.do_transform_pose(source, transform)
            return (local.pose.position.x, local.pose.position.y,
                    local.pose.position.z)
        return None

    @staticmethod
    def _stamp_key(detections):
        """返回消息时间戳键；没有有效时间戳时退化为 None。"""
        header = getattr(detections, "header", None)
        stamp = getattr(header, "stamp", None)
        if stamp is None:
            return None
        try:
            return (stamp.secs, stamp.nsecs)
        except AttributeError:
            return None

    @staticmethod
    def _dist3(first, second):
        return math.sqrt((first[0] - second[0]) ** 2 +
                         (first[1] - second[1]) ** 2 +
                         (first[2] - second[2]) ** 2)

"""AprilTag ID 筛选和位置稳定判断。"""

from robotac_examples.geometry import distance3


def find_detection(detections, tag_id):
    for detection in detections:
        for detected_id in detection.id:
            if int(detected_id) == int(tag_id):
                return detection
    return None


class StableTag(object):
    def __init__(self, sample_count=5, jump_limit=0.15):
        if sample_count < 1:
            raise ValueError("sample_count 必须大于 0")
        if jump_limit <= 0.0:
            raise ValueError("jump_limit 必须大于 0")
        self.sample_count = sample_count
        self.jump_limit = jump_limit
        self.samples = []

    def reset(self):
        self.samples = []

    def add(self, point):
        if self.samples and distance3(point, self.samples[-1]) > self.jump_limit:
            self.reset()
        self.samples.append(point)
        self.samples = self.samples[-self.sample_count:]
        if len(self.samples) < self.sample_count:
            return None
        total = [0.0, 0.0, 0.0]
        for sample in self.samples:
            total[0] += sample[0]
            total[1] += sample[1]
            total[2] += sample[2]
        count = float(len(self.samples))
        return [total[0] / count, total[1] / count, total[2] / count]

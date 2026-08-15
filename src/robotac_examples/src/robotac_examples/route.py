"""读取简短、明确的相对航点列表。"""

import math

import yaml


def load_route(path):
    with open(path, "r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    if set(data.keys()) != {"waypoints"}:
        raise ValueError("航点文件只能包含 waypoints")
    if not isinstance(data["waypoints"], list) or not data["waypoints"]:
        raise ValueError("waypoints 必须是非空列表")
    result = []
    for index, item in enumerate(data["waypoints"]):
        if not isinstance(item, dict):
            raise ValueError("第 %d 个航点不是字典" % index)
        if set(item.keys()) != {"x", "y", "z", "hold"}:
            raise ValueError("航点字段必须为 x、y、z、hold")
        point = [float(item["x"]), float(item["y"]), float(item["z"])]
        hold = float(item["hold"])
        values = point + [hold]
        if not all(math.isfinite(value) for value in values):
            raise ValueError("航点包含非有限数值")
        if hold < 0.0 or hold > 30.0:
            raise ValueError("航点停留时间必须在 0 至 30 秒之间")
        result.append((point, hold))
    return result

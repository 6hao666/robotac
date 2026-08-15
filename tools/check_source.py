#!/usr/bin/env python3
"""Robotac 项目自有源码和文档的静态检查。"""

import ast
import json
import pathlib
import re
import subprocess
import sys

import yaml


ROOT = pathlib.Path(__file__).resolve().parents[1]
OWN_PACKAGES = [
    ROOT / "src/robotac_bringup",
    ROOT / "src/robotac_examples",
    ROOT / "src/robotac_localization",
    ROOT / "src/robotac_servo",
]
EXAMPLE_NAMES = [
    "fcu_state", "local_pose", "apriltag_detection",
    "apriltag_local_pose", "setpoint_preview", "hover",
    "move_relative", "waypoints", "tag_centering_preview",
    "tag_centering_flight", "payload_release",
]


def fail(message):
    print("检查失败：%s" % message, file=sys.stderr)
    raise SystemExit(1)


def require_paths():
    paths = [
        "src/robotac_bringup/launch/sensors.launch",
        "src/robotac_bringup/launch/perception.launch",
        "src/robotac_bringup/launch/flight_base.launch",
        "src/robotac_bringup/tools/livox_lidar_discover.cpp",
        "src/robotac_localization/launch/vision_preview.launch",
        "src/robotac_localization/launch/vision_to_px4.launch",
        "tools/sensor_setup.py",
    ]
    expected_scripts = set()
    for number, name in enumerate(EXAMPLE_NAMES, 1):
        base = "%02d_%s" % (number, name)
        launch = ROOT / "src/robotac_examples/launch" / (base + ".launch")
        script = ROOT / "src/robotac_examples/scripts" / (base + ".py")
        tutorial = ROOT / "docs/tutorials" / (
            "%02d-%s.md" % (number, name.replace("_", "-")))
        paths.extend([launch.relative_to(ROOT), script.relative_to(ROOT),
                      tutorial.relative_to(ROOT)])
        expected_scripts.add(script.name)
    for path in paths:
        if not (ROOT / path).is_file():
            fail("缺少文件 %s" % path)

    actual_scripts = set(
        path.name for path in (ROOT / "src/robotac_examples/scripts").glob("*.py"))
    if actual_scripts != expected_scripts:
        fail("编号教学脚本集合不符合要求")

    for number, name in enumerate(EXAMPLE_NAMES, 1):
        base = "%02d_%s" % (number, name)
        launch = ROOT / "src/robotac_examples/launch" / (base + ".launch")
        result = subprocess.run(
            ["xmllint", "--xpath",
             "string((//node[@pkg='robotac_examples']/@type)[1])",
             str(launch)], stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            text=True, check=False)
        if result.returncode != 0 or result.stdout.strip() != base + ".py":
            fail("launch 未引用同名教学脚本：%s" % launch.relative_to(ROOT))


def check_removed_paths():
    forbidden = [
        "config", "scripts", "src/robotac_flight",
        "src/robotac_bringup/launch/full_system.launch",
        "src/robotac_bringup/launch/tag_payload_mission_full.launch",
        "src/robotac_bringup/launch/payload_drop_box_test.launch",
    ]
    for relative in forbidden:
        if (ROOT / relative).exists():
            fail("旧结构仍然存在：%s" % relative)


def check_python():
    files = []
    for package in OWN_PACKAGES:
        files.extend(package.rglob("*.py"))
    files.extend((ROOT / "tools").glob("*.py"))
    files.extend((ROOT / "tools/test").glob("*.py"))
    files = sorted(set(files))
    for path in files:
        source = path.read_text(encoding="utf-8")
        try:
            tree = ast.parse(source, filename=str(path))
        except SyntaxError as error:
            fail("Python 语法错误 %s:%s" % (path.relative_to(ROOT), error))
        lines = len(source.splitlines())
        relative = path.relative_to(ROOT)
        if "robotac_examples/scripts" in str(relative) and lines > 200:
            fail("教学脚本超过 200 行：%s" % relative)
        if "/src/" in str(relative):
            if lines > 250:
                fail("公共模块超过 250 行：%s" % relative)
        if path != ROOT / "tools/check_source.py":
            if "from __future__ import annotations" in source:
                fail("项目源码使用了 future annotations：%s" % relative)
            for node in ast.walk(tree):
                if isinstance(node, (ast.AsyncFunctionDef, ast.Await,
                                     ast.NamedExpr)):
                    fail("项目源码使用了不允许的复杂语法：%s" % relative)
                if node.__class__.__name__ == "Match":
                    fail("项目源码使用了模式匹配：%s" % relative)


def check_shell():
    for path in sorted((ROOT / "tools").glob("*.sh")):
        lines = len(path.read_text(encoding="utf-8").splitlines())
        if lines > 150:
            fail("Shell 脚本超过 150 行：%s" % path.relative_to(ROOT))


def check_structured_files():
    for package in OWN_PACKAGES:
        for path in sorted(package.rglob("*.launch")):
            check_xml(path)
        for path in sorted(package.rglob("package.xml")):
            check_xml(path)
        for path in sorted(package.rglob("*.yaml")):
            with path.open(encoding="utf-8") as stream:
                yaml.safe_load(stream)
        for path in sorted(package.rglob("*.json")):
            with path.open(encoding="utf-8") as stream:
                json.load(stream)


def check_xml(path):
    result = subprocess.run(["xmllint", "--noout", str(path)],
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                            text=True, check=False)
    if result.returncode != 0:
        fail("XML 语法错误 %s: %s" %
             (path.relative_to(ROOT), result.stderr.strip()))


def package_locations():
    result = {}
    for path in (ROOT / "src").rglob("package.xml"):
        source = path.read_text(encoding="utf-8", errors="replace")
        match = re.search(r"<name>\s*([^<]+?)\s*</name>", source)
        if match:
            result[match.group(1)] = path.parent
    return result


def check_launch_paths():
    packages = package_locations()
    pattern = re.compile(r"\$\(find ([^)]+)\)/([^\s\"'<>$]+)")
    for package in OWN_PACKAGES:
        for path in package.rglob("*.launch"):
            source = path.read_text(encoding="utf-8")
            for package_name, relative in pattern.findall(source):
                base = packages.get(package_name)
                if base is None or not (base / relative).exists():
                    fail("launch 路径无效：%s -> %s/%s" %
                         (path.relative_to(ROOT), package_name, relative))


def check_hardware_defaults():
    lidar_path = (ROOT / "src/robotac_bringup/config/lidar/mid360s.json")
    with lidar_path.open(encoding="utf-8") as stream:
        lidar = json.load(stream)
    host_ip = lidar["Mid360s"]["host_net_info"][0]["host_ip"]
    lidar_ip = lidar["lidar_configs"][0]["ip"]
    if host_ip != "<机载网卡地址>" or lidar_ip != "<雷达地址>":
        fail("MID360 配置必须保留现场填写的地址占位符")

    discover_path = (
        ROOT / "src/robotac_bringup/tools/livox_lidar_discover.cpp")
    discover_source = discover_path.read_text(encoding="utf-8")
    required_discovery = ["EnableLivoxLidarDiscoveryOnly",
                          "LivoxLidarSdkInit(nullptr", "LivoxLidarSdkUninit",
                          "SetLivoxLidarInfoChangeCallback"]
    for value in required_discovery:
        if value not in discover_source:
            fail("Livox 发现器缺少只读发现接口：%s" % value)
    forbidden_discovery = ["SetLivoxLidarWorkMode", "SetLivoxLidarIp",
                           "SetLivoxLidarPointCloudCallBack"]
    for value in forbidden_discovery:
        if value in discover_source:
            fail("Livox 发现器包含设备修改或数据流接口：%s" % value)

    mavros_launch = (ROOT / "src/robotac_bringup/launch/mavros_px4.launch")
    mavros_source = mavros_launch.read_text(encoding="utf-8")
    if "serial:///dev/robotac_px4:921600" not in mavros_source:
        fail("MAVROS 默认入口必须使用稳定 PX4 设备别名")

    servo_path = ROOT / "src/robotac_servo/config/servo.yaml"
    with servo_path.open(encoding="utf-8") as stream:
        servo = yaml.safe_load(stream)
    if servo.get("port") != "/dev/robotac_servo":
        fail("舵机默认入口必须使用稳定设备别名")
    if servo.get("port_candidates") != ["/dev/robotac_servo"]:
        fail("舵机自动候选列表不得包含未核验的原始串口")


def check_interfaces():
    roots = [ROOT / "README.md", ROOT / "docs", ROOT / "tools"] + OWN_PACKAGES
    texts = []
    for root in roots:
        paths = [root] if root.is_file() else root.rglob("*")
        for path in paths:
            if path.is_file() and path.suffix in {".py", ".launch", ".md", ".yaml", ".sh"}:
                texts.append(path.read_text(encoding="utf-8", errors="replace"))
    source = "\n".join(texts)
    required = ["/mavros/setpoint_position/local",
                "/robotac_examples/tag/pose",
                "/robotac_examples/tag/error",
                "/robotac_servo/set_released"]
    runtime_roots = [ROOT / "tools"] + OWN_PACKAGES
    runtime_texts = []
    for root in runtime_roots:
        for path in root.rglob("*"):
            if path.is_file() and path.suffix in {".py", ".launch", ".yaml", ".sh"}:
                runtime_texts.append(path.read_text(encoding="utf-8",
                                                    errors="replace"))
    runtime_source = "\n".join(runtime_texts)
    for value in required:
        if value not in runtime_source:
            fail("运行代码缺少接口 %s" % value)
    forbidden = ["/robotac/" + "flight/",
                 "/robotac/" + "tag_payload_mission/",
                 "Position" + "Target"]
    for value in forbidden:
        if value in runtime_source:
            fail("仍存在旧接口或旧消息：%s" % value)


def check_markdown_links():
    pattern = re.compile(r"\[[^]]+\]\(([^)]+)\)")
    for path in [ROOT / "README.md"] + sorted((ROOT / "docs").rglob("*.md")):
        source = path.read_text(encoding="utf-8")
        for target in pattern.findall(source):
            target = target.strip().strip("<>").split("#", 1)[0]
            target = re.sub(r'\s+(?:"[^"]*"|\'[^\']*\'|\([^)]*\))\s*$',
                            "", target)
            if not target or re.match(r"^[a-z]+://", target):
                continue
            if not (path.parent / target).resolve().exists():
                fail("文档链接无效：%s -> %s" %
                     (path.relative_to(ROOT), target))


def check_document_disclosure():
    paths = [ROOT / "README.md"] + sorted((ROOT / "docs").rglob("*.md"))
    source = "\n".join(path.read_text(encoding="utf-8") for path in paths)
    for pattern in [r"/Users/", r"\b(?:\d{1,3}\.){3}\d{1,3}\b", r"\bPID\s*[:=]\s*\d+"]:
        if re.search(pattern, source):
            fail("公开文档包含机器相关信息：%s" % pattern)
    if "学生" in source:
        fail("公开文档应统一使用“参赛选手”")


def main():
    require_paths()
    check_removed_paths()
    check_python()
    check_shell()
    check_structured_files()
    check_launch_paths()
    check_hardware_defaults()
    check_interfaces()
    check_markdown_links()
    check_document_disclosure()
    print("源码、结构、接口和文档链接检查通过。")


if __name__ == "__main__":
    main()

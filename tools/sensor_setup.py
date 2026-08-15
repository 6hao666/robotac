#!/usr/bin/env python3
"""Robotac 雷达地址发现与 RGB 相机现场检查工具。"""

import argparse
import atexit
import glob
import ipaddress
import json
import os
import pathlib
import re
import shlex
import shutil
import subprocess
import sys
import tempfile
import time


ROOT = pathlib.Path(__file__).resolve().parents[1]
DEFAULT_LIDAR_CONFIG = (
    ROOT / "src/robotac_bringup/config/lidar/mid360s.json")
VENDOR_LIDAR_CONFIG = ROOT / "src/livox_ros_driver2/config/MID360s_config.json"
CAMERA_RULE_TEMPLATE = (
    ROOT / "src/robotac_bringup/config/udev/99-robotac-rgb-camera.rules.template")
CAMERA_ALIAS = pathlib.Path("/dev/robotac_rgb_camera")

EXIT_USAGE = 2
EXIT_PREREQUISITE = 3
EXIT_NOT_FOUND = 4
EXIT_MISMATCH = 5
EXIT_AMBIGUOUS = 6
EXIT_VERIFY = 7
SUPPORTED_LIVOX_TYPES = {9, 35}


class SetupError(Exception):
    def __init__(self, message, exit_code=EXIT_VERIFY):
        super().__init__(message)
        self.exit_code = exit_code


def run_command(command, check=False, capture=True, timeout=None):
    result = subprocess.run(
        command,
        stdout=subprocess.PIPE if capture else None,
        stderr=subprocess.PIPE if capture else None,
        text=True,
        check=False,
        timeout=timeout,
    )
    if check and result.returncode != 0:
        detail = (result.stderr or result.stdout or "命令执行失败").strip()
        raise SetupError("%s：%s" % (shlex.join(command), detail))
    return result


def require_commands(names):
    missing = [name for name in names if shutil.which(name) is None]
    if missing:
        raise SetupError(
            "缺少命令：%s。请先执行 tools/install_ubuntu20.sh。" %
            ", ".join(missing), EXIT_PREREQUISITE)


def read_json(path):
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError, TypeError) as error:
        raise SetupError("无法读取 JSON %s：%s" % (path, error), EXIT_USAGE)


def configured_lidar_addresses(data):
    try:
        host = data["Mid360s"]["host_net_info"][0]["host_ip"]
        lidar = data["lidar_configs"][0]["ip"]
    except (KeyError, IndexError, TypeError):
        raise SetupError("MID360s 配置结构不完整。", EXIT_USAGE)
    return host, lidar


def vendor_host_address():
    data = read_json(VENDOR_LIDAR_CONFIG)
    net_info = data["Mid360s"]["host_net_info"]
    if isinstance(net_info, list):
        return net_info[0]["host_ip"]
    return net_info["host_ip"]


def discovery_candidate(configured_host, assigned_addresses):
    candidate = valid_ipv4(configured_host) or vendor_host_address()
    if candidate in assigned_addresses:
        return None
    return candidate


def valid_ipv4(value):
    try:
        return str(ipaddress.IPv4Address(value))
    except ipaddress.AddressValueError:
        return None


def atomic_write_lidar_config(path, host_ip, lidar_ip):
    data = read_json(path)
    configured_lidar_addresses(data)
    data["Mid360s"]["host_net_info"][0]["host_ip"] = host_ip
    data["lidar_configs"][0]["ip"] = lidar_ip
    mode = path.stat().st_mode
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(
                mode="w", encoding="utf-8", dir=str(path.parent),
                prefix=".%s." % path.name, suffix=".tmp", delete=False) as stream:
            temporary = pathlib.Path(stream.name)
            json.dump(data, stream, ensure_ascii=False, indent=2)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.chmod(str(temporary), mode)
        os.replace(str(temporary), str(path))
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()


def ask_yes_no(prompt, non_interactive=False):
    if non_interactive or not sys.stdin.isatty():
        return False
    answer = input("%s [y/N] " % prompt).strip().lower()
    return answer in {"y", "yes"}


def network_interfaces():
    require_commands(["ip"])
    links = json.loads(run_command(["ip", "-j", "link", "show"], check=True).stdout)
    addresses = json.loads(
        run_command(["ip", "-j", "-4", "address", "show"], check=True).stdout)
    address_map = {item["ifindex"]: item.get("addr_info", []) for item in addresses}
    result = []
    for link in links:
        name = link.get("ifname", "")
        flags = set(link.get("flags", []))
        wireless = pathlib.Path("/sys/class/net", name, "wireless").exists()
        virtual = (name.startswith(("br", "docker", "veth", "virbr", "l4tbr")) or
                   bool(link.get("master")))
        if link.get("link_type") != "ether" or wireless or virtual:
            continue
        ipv4 = []
        for address in address_map.get(link.get("ifindex"), []):
            if address.get("family") == "inet" and address.get("scope") == "global":
                ipv4.append({
                    "address": address["local"],
                    "prefix": int(address["prefixlen"]),
                })
        result.append({
            "name": name,
            "carrier": "LOWER_UP" in flags,
            "ipv4": ipv4,
        })
    return result


def select_interface(interfaces, requested, non_interactive=False):
    if requested:
        matches = [item for item in interfaces if item["name"] == requested]
        if not matches:
            raise SetupError("接口 %s 不是可用的物理以太网接口。" % requested,
                             EXIT_NOT_FOUND)
        selected = matches[0]
        if not selected["carrier"]:
            raise SetupError("接口 %s 没有物理链路。" % requested, EXIT_NOT_FOUND)
        return selected
    connected = [item for item in interfaces if item["carrier"]]
    if len(connected) == 1:
        return connected[0]
    if not connected:
        raise SetupError("没有检测到已连接的物理以太网接口。", EXIT_NOT_FOUND)
    names = ", ".join(item["name"] for item in connected)
    raise SetupError("存在多个已连接接口（%s），请使用 --interface 指定。" % names,
                     EXIT_AMBIGUOUS)


class TemporaryAddress:
    def __init__(self, interface, address, prefix=24, runner=run_command):
        self.interface = interface
        self.address = address
        self.prefix = prefix
        self.runner = runner
        self.active = False
        atexit.register(self.cleanup)

    def add(self):
        require_commands(["arping", "sudo"])
        conflict = self.runner([
            "arping", "-D", "-q", "-c", "2", "-w", "3",
            "-I", self.interface, self.address,
        ])
        if conflict.returncode != 0:
            raise SetupError("候选主机地址已被占用，未修改网卡。", EXIT_MISMATCH)
        self.runner([
            "sudo", "ip", "address", "add",
            "%s/%d" % (self.address, self.prefix),
            "dev", self.interface,
        ], check=True, capture=False)
        self.active = True

    def cleanup(self):
        if not self.active:
            return
        self.runner([
            "sudo", "ip", "address", "del",
            "%s/%d" % (self.address, self.prefix),
            "dev", self.interface,
        ], capture=False)
        self.active = False

    def __enter__(self):
        self.add()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.cleanup()
        return False


def helper_command(explicit=None):
    if explicit:
        return [explicit]
    override = os.environ.get("ROBOTAC_LIDAR_DISCOVER_BIN")
    if override:
        return [override]
    local = ROOT / "devel/lib/robotac_bringup/robotac_lidar_discover"
    if local.is_file() and os.access(str(local), os.X_OK):
        return [str(local)]
    executable = shutil.which("robotac_lidar_discover")
    if executable:
        return [executable]
    if shutil.which("rosrun"):
        return ["rosrun", "robotac_bringup", "robotac_lidar_discover"]
    raise SetupError(
        "未找到 robotac_lidar_discover，请先执行 tools/test_02_build.sh。",
        EXIT_PREREQUISITE)


def parse_helper_json(output):
    for line in reversed(output.splitlines()):
        line = line.strip()
        if line.startswith("["):
            data = json.loads(line)
            if not isinstance(data, list):
                break
            return data
    raise SetupError("Livox 发现器没有返回有效 JSON。", EXIT_VERIFY)


def discover_livox(command, host_ip, timeout_seconds):
    complete = command + [
        "--host-ip", host_ip, "--timeout", str(timeout_seconds), "--json"]
    result = run_command(complete, timeout=timeout_seconds + 8)
    if result.returncode == 4:
        return []
    if result.returncode != 0:
        detail = (result.stderr or result.stdout).strip()
        raise SetupError("Livox 发现失败：%s" % detail, EXIT_VERIFY)
    return parse_helper_json(result.stdout)


def persistence_command(interface, address, prefix):
    name = "robotac-lidar-%s" % interface
    command = [
        "sudo", "nmcli", "connection", "add", "type", "ethernet",
        "ifname", interface, "con-name", name, "ipv4.method", "manual",
        "ipv4.addresses", "%s/%d" % (address, prefix),
    ]
    return shlex.join(command)


def lidar_setup(args):
    config_path = pathlib.Path(args.config).expanduser().resolve()
    config = read_json(config_path)
    configured_host, configured_lidar = configured_lidar_addresses(config)
    interface = select_interface(
        network_interfaces(), args.interface, args.non_interactive or args.json)
    command = helper_command(args.helper)
    addresses = list(interface["ipv4"])
    temporary = None
    if not addresses:
        suggested = valid_ipv4(configured_host) or vendor_host_address()
        message = ("接口 %s 有链路但没有 IPv4。临时添加候选地址并在退出时删除？" %
                   interface["name"])
        if not ask_yes_no(message, args.non_interactive or args.json):
            raise SetupError(
                "接口没有 IPv4；未获授权，因此没有修改网卡。", EXIT_MISMATCH)
        temporary = TemporaryAddress(interface["name"], suggested, 24)
        temporary.add()
        addresses = [{"address": suggested, "prefix": 24}]

    discoveries = {}
    try:
        for address in addresses:
            found = discover_livox(command, address["address"], args.timeout)
            for device in found:
                device["host_ip"] = address["address"]
                device["interface"] = interface["name"]
                discoveries[device["lidar_ip"]] = device
    finally:
        if temporary is not None:
            temporary.cleanup()

    if not discoveries and temporary is None:
        configured_addresses = {item["address"] for item in addresses}
        suggested = discovery_candidate(configured_host, configured_addresses)
        message = ("现有网卡地址没有发现 Livox。临时追加配置候选地址重试，"
                   "并在退出时删除？")
        if (suggested is not None and
                ask_yes_no(message, args.non_interactive or args.json)):
            temporary = TemporaryAddress(interface["name"], suggested, 24)
            temporary.add()
            try:
                found = discover_livox(command, suggested, args.timeout)
                for device in found:
                    device["host_ip"] = suggested
                    device["interface"] = interface["name"]
                    discoveries[device["lidar_ip"]] = device
            finally:
                temporary.cleanup()

    devices = sorted(discoveries.values(), key=lambda item: item["lidar_ip"])
    supported = [item for item in devices
                 if int(item.get("device_type", -1)) in SUPPORTED_LIVOX_TYPES]
    payload = {
        "interface": interface["name"],
        "configured": {"host_ip": configured_host, "lidar_ip": configured_lidar},
        "devices": devices,
        "temporary_address_removed": temporary is not None,
    }
    if args.json:
        print(json.dumps(payload, ensure_ascii=False, indent=2))
    else:
        print("雷达接口：%s" % interface["name"])
        print("当前配置：主机 %s，雷达 %s" % (configured_host, configured_lidar))
        for device in devices:
            print("发现：%s  主机=%s  雷达=%s  SN=%s" % (
                device.get("device_name", "Livox"), device["host_ip"],
                device["lidar_ip"], device.get("serial", "")))

    if not devices:
        raise SetupError("未发现 Livox 设备；没有把空结果当作有效配置。",
                         EXIT_NOT_FOUND)
    if len(supported) != 1:
        raise SetupError("MID360/MID360s 发现结果不是唯一设备，未写配置。",
                         EXIT_AMBIGUOUS)

    selected = supported[0]
    matches = (configured_host == selected["host_ip"] and
               configured_lidar == selected["lidar_ip"])
    if not args.json and not args.non_interactive:
        if matches:
            print("配置与发现结果一致。")
        elif ask_yes_no("将发现的主机和雷达地址写入 %s？" % config_path):
            atomic_write_lidar_config(
                config_path, selected["host_ip"], selected["lidar_ip"])
            matches = True
            print("已写入配置。该文件含现场地址，不得提交到公开仓库。")
    if temporary is not None and not args.json:
        print("临时地址已经删除。需要持久化时人工确认后执行：")
        print(persistence_command(
            interface["name"], selected["host_ip"], 24))
    return 0 if matches else EXIT_MISMATCH


def parse_udev_properties(text):
    result = {}
    for line in text.splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            result[key] = value
    return result


def camera_mode_supported(text, width=1920, height=1080, fps=30):
    mjpeg = bool(re.search(r"\[(?:0|[0-9]+)\]:\s+'MJPG'", text))
    size = "Size: Discrete %dx%d" % (width, height)
    rate = "(%.3f fps)" % float(fps)
    return mjpeg and size in text and rate in text


def inspect_camera_device(device):
    properties = parse_udev_properties(run_command([
        "udevadm", "info", "--query=property", "--name", device], check=True).stdout)
    details = run_command(["v4l2-ctl", "-d", device, "--all"], check=True).stdout
    formats = run_command([
        "v4l2-ctl", "-d", device, "--list-formats-ext"], check=True).stdout
    return {
        "device": device,
        "capture": ":capture:" in properties.get("ID_V4L_CAPABILITIES", ""),
        "vendor_id": properties.get("ID_VENDOR_ID", ""),
        "model_id": properties.get("ID_MODEL_ID", ""),
        "serial": properties.get("ID_SERIAL_SHORT", properties.get("ID_SERIAL", "")),
        "card": next((line.split(":", 1)[1].strip() for line in details.splitlines()
                      if line.strip().startswith("Card type")), ""),
        "target_mode": camera_mode_supported(formats),
    }


def camera_rule_ids(path):
    source = path.read_text(encoding="utf-8")
    vendor = re.search(r'ATTRS\{idVendor\}=="([0-9a-fA-F]+)"', source)
    model = re.search(r'ATTRS\{idProduct\}=="([0-9a-fA-F]+)"', source)
    if vendor is None or model is None or "<" in source or ">" in source:
        raise SetupError("相机 udev 模板仍含占位符或缺少 USB ID。", EXIT_MISMATCH)
    return vendor.group(1).lower(), model.group(1).lower()


def install_camera_rule(selected, non_interactive=False):
    vendor, model = camera_rule_ids(CAMERA_RULE_TEMPLATE)
    if (selected["vendor_id"].lower(), selected["model_id"].lower()) != (vendor, model):
        raise SetupError("相机 USB ID 与 udev 模板不一致，拒绝安装。", EXIT_MISMATCH)
    if not ask_yes_no("安装并重载 robotac_rgb_camera udev 规则？", non_interactive):
        raise SetupError("未获授权，因此没有安装 udev 规则。", EXIT_MISMATCH)
    require_commands(["sudo"])
    destination = "/etc/udev/rules.d/99-robotac-rgb-camera.rules"
    run_command([
        "sudo", "install", "-m", "0644", str(CAMERA_RULE_TEMPLATE), destination,
    ], check=True, capture=False)
    run_command(["sudo", "udevadm", "control", "--reload-rules"],
                check=True, capture=False)
    run_command([
        "sudo", "udevadm", "trigger", "--subsystem-match=video4linux", "--action=add",
    ], check=True, capture=False)
    run_command(["udevadm", "settle"], check=True)
    time.sleep(1)


def camera_setup(args):
    require_commands(["udevadm", "v4l2-ctl"])
    devices = [args.device] if args.device else sorted(glob.glob("/dev/video[0-9]*"))
    inspected = [inspect_camera_device(device) for device in devices
                 if pathlib.Path(device).exists()]
    captures = [item for item in inspected if item["capture"]]
    if not captures:
        raise SetupError("没有发现 V4L2 视频采集节点。", EXIT_NOT_FOUND)
    if len(captures) > 1 and not args.device:
        names = ", ".join(item["device"] for item in captures)
        raise SetupError("存在多个采集节点（%s），请用 --device 指定。" % names,
                         EXIT_AMBIGUOUS)
    selected = captures[0]
    vendor, model = camera_rule_ids(CAMERA_RULE_TEMPLATE)
    selected["matches_rule"] = (
        selected["vendor_id"].lower(), selected["model_id"].lower()) == (vendor, model)
    selected["stable_alias"] = (
        CAMERA_ALIAS.exists() and CAMERA_ALIAS.resolve() == pathlib.Path(selected["device"]).resolve())

    if args.json:
        print(json.dumps({"devices": inspected, "selected": selected},
                         ensure_ascii=False, indent=2))
    else:
        for item in inspected:
            role = "视频采集" if item["capture"] else "元数据/其他"
            print("%s：%s，%s" % (item["device"], role, item["card"] or "未知设备"))
        print("目标模式 MJPEG 1920x1080@30：%s" %
              ("支持" if selected["target_mode"] else "不支持"))
        print("udev 模板匹配：%s" % ("是" if selected["matches_rule"] else "否"))
        print("稳定别名：%s" % ("正常" if selected["stable_alias"] else "缺失或指向错误"))

    if not selected["target_mode"] or not selected["matches_rule"]:
        raise SetupError("相机能力或 udev 身份与项目配置不一致。", EXIT_MISMATCH)
    if args.install_udev and not selected["stable_alias"]:
        install_camera_rule(selected, args.non_interactive or args.json)
        selected["stable_alias"] = (
            CAMERA_ALIAS.exists() and
            CAMERA_ALIAS.resolve() == pathlib.Path(selected["device"]).resolve())
        if not selected["stable_alias"]:
            raise SetupError("规则已安装，但稳定别名没有正确生成。", EXIT_VERIFY)
        print("已验证 %s -> %s" % (CAMERA_ALIAS, selected["device"]))
    return 0 if selected["stable_alias"] else EXIT_MISMATCH


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    lidar = subparsers.add_parser("lidar", help="发现并比较 Livox 雷达地址")
    lidar.add_argument("--interface", help="指定物理以太网接口")
    lidar.add_argument("--timeout", type=int, default=5, choices=range(1, 61),
                       metavar="SECONDS")
    lidar.add_argument("--config", default=str(DEFAULT_LIDAR_CONFIG))
    lidar.add_argument("--helper", help=argparse.SUPPRESS)
    lidar.add_argument("--json", action="store_true")
    lidar.add_argument("--non-interactive", action="store_true")

    camera = subparsers.add_parser("camera", help="识别并检查 RGB 相机")
    camera.add_argument("--device", help="指定 /dev/videoN 采集节点")
    camera.add_argument("--install-udev", action="store_true")
    camera.add_argument("--json", action="store_true")
    camera.add_argument("--non-interactive", action="store_true")
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    try:
        if args.command == "lidar":
            return lidar_setup(args)
        return camera_setup(args)
    except (SetupError, json.JSONDecodeError, subprocess.TimeoutExpired) as error:
        if isinstance(error, SetupError):
            code = error.exit_code
            message = str(error)
        elif isinstance(error, subprocess.TimeoutExpired):
            code = EXIT_VERIFY
            message = "命令超时：%s" % shlex.join(error.cmd)
        else:
            code = EXIT_VERIFY
            message = "数据解析失败：%s" % error
        print(message, file=sys.stderr)
        return code


if __name__ == "__main__":
    sys.exit(main())

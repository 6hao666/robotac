#!/usr/bin/env python3
"""Robotac USB PWM 舵机交互式角度标定工具。"""

import argparse
import curses
import fcntl
import glob
import locale
import os
import sys
import termios
import time

import rospy
from typing import List, Optional, Sequence, Tuple

from robotac_servo.protocol import duty_for_angle, make_frame, pulse_width_us

try:
    import serial
except ImportError:  # pragma: no cover - deployment fallback
    serial = None


AUTO_PORT_PATTERNS = ("/dev/robotac_servo", "/dev/ttyUSB0", "/dev/ttyUSB1")


class SerialTransport:
    def __init__(self, port: str, baudrate: int) -> None:
        self.port = port
        self._serial = None
        self._fd = None
        if serial is not None:
            connection = serial.Serial(
                port=None,
                baudrate=baudrate,
                timeout=0.0,
                write_timeout=1.0,
                rtscts=False,
                dsrdtr=False,
                xonxoff=False,
            )
            connection.dtr = False
            connection.rts = False
            connection.port = port
            connection.open()
            try:
                fcntl.flock(connection.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            except (OSError, IOError):
                connection.close()
                raise RuntimeError(f"串口正在被其他程序占用：{port}")
            self._serial = connection
            return

        flags = os.O_RDWR | os.O_NOCTTY
        if hasattr(os, "O_CLOEXEC"):
            flags |= os.O_CLOEXEC
        fd = os.open(port, flags)
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            attrs = termios.tcgetattr(fd)
            attrs[0] = 0
            attrs[1] = 0
            attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
            attrs[3] = 0
            speed = getattr(termios, f"B{baudrate}")
            attrs[4] = speed
            attrs[5] = speed
            attrs[6][termios.VMIN] = 0
            attrs[6][termios.VTIME] = 0
            termios.tcsetattr(fd, termios.TCSANOW, attrs)
        except Exception:
            os.close(fd)
            raise
        self._fd = fd

    def write(self, data: bytes) -> None:
        if self._serial is not None:
            count = self._serial.write(data)
            self._serial.flush()
            if count != len(data):
                raise OSError(f"串口数据未完整写入：{count}/{len(data)} 字节")
            return
        if self._fd is None:
            raise OSError("串口已关闭")
        written = 0
        while written < len(data):
            count = os.write(self._fd, data[written:])
            if count <= 0:
                raise OSError("串口设备未接收任何数据")
            written += count
        termios.tcdrain(self._fd)

    def probe(self) -> None:
        if self._serial is not None:
            if not self._serial.is_open:
                raise OSError("串口已关闭")
            _ = self._serial.in_waiting
            return
        if self._fd is None:
            raise OSError("串口已关闭")
        termios.tcgetattr(self._fd)

    def close(self) -> None:
        if self._serial is not None:
            if self._serial.is_open:
                self._serial.close()
            return
        if self._fd is not None:
            os.close(self._fd)
            self._fd = None


class AutoConnector:
    def __init__(self, ports: Sequence[str], baudrate: int) -> None:
        self.ports = list(ports)
        self.baudrate = baudrate
        self.transport: Optional[SerialTransport] = None
        self.last_errors: List[str] = []

    @property
    def port(self) -> Optional[str]:
        return self.transport.port if self.transport is not None else None

    def candidate_ports(self) -> List[str]:
        candidates = []
        for pattern in self.ports:
            for path in sorted(glob.glob(pattern)):
                if path not in candidates:
                    candidates.append(path)
        return candidates

    def connect(self) -> bool:
        if self.transport is not None:
            try:
                self.transport.probe()
                return True
            except OSError:
                self.disconnect()
        self.last_errors = []
        for path in self.candidate_ports():
            transport = None
            try:
                transport = SerialTransport(path, self.baudrate)
                transport.probe()
            except (OSError, RuntimeError, ValueError) as exc:
                if transport is not None:
                    transport.close()
                self.last_errors.append(f"{path}: {exc}")
                continue
            self.transport = transport
            return True
        return False

    def probe(self) -> bool:
        if self.transport is None:
            return False
        try:
            self.transport.probe()
        except OSError:
            self.disconnect()
            return False
        return True

    def write(self, data: bytes) -> None:
        if self.transport is None:
            raise OSError("尚未连接舵机控制器")
        try:
            self.transport.write(data)
        except OSError:
            self.disconnect()
            raise

    def disconnect(self) -> None:
        if self.transport is not None:
            try:
                self.transport.close()
            except OSError:
                pass
        self.transport = None


class TunerState:
    def __init__(self, angle: int, minimum_angle: int,
                 maximum_angle: int) -> None:
        self.angle = angle
        self.minimum_angle = minimum_angle
        self.maximum_angle = maximum_angle
        self.last_sent_duty: Optional[int] = None
        self.last_sent_angle: Optional[int] = None
        self.closed_angle: Optional[int] = None
        self.open_angle: Optional[int] = None
        self.output_active = False
        self.message = "正在搜索舵机控制器，尚未发送任何控制指令。"


class ServoAngleTuner:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.connector = AutoConnector(args.port, args.baudrate)
        self.state = TunerState(args.start_angle, args.min_angle, args.max_angle)
        self._last_connect_attempt = 0.0
        self._last_probe = 0.0
        self._release_deadline: Optional[float] = None
        self._release_message = "本次点动完成，PWM 已自动释放。"

    def frame_for_angle(self, angle: int) -> Tuple[bytes, int]:
        duty = duty_for_angle(angle, self.args.pwm_min_duty, self.args.pwm_max_duty)
        return make_frame(self.args.channel, self.args.frequency_hz, duty), duty

    def send_angle(self) -> None:
        if not self.connector.connect():
            self.state.message = "在稳定别名、ttyUSB0 和 ttyUSB1 上均未发现舵机控制器。"
            return
        frame, duty = self.frame_for_angle(self.state.angle)
        previous_duty = self.state.last_sent_duty
        try:
            self.connector.write(frame)
        except OSError as exc:
            self.state.output_active = False
            self._release_deadline = None
            self.state.message = f"写入失败：{exc}。正在等待重连，不会自动重发旧指令。"
            return
        self.state.last_sent_angle = self.state.angle
        self.state.last_sent_duty = duty
        self.state.output_active = True
        self._release_deadline = time.monotonic() + self.args.drive_seconds
        if previous_duty == duty:
            self.state.message = (
                f"正在点动 {self.args.drive_seconds:.2f} 秒；"
                "目标角度已更新，但整数 PWM 占空比没有变化。"
            )
            self._release_message = "点动完成，PWM 已释放；本次占空比没有变化。"
        else:
            self.state.message = f"正在点动 {self.args.drive_seconds:.2f} 秒，随后自动释放 PWM。"
            self._release_message = "本次点动完成，PWM 已自动释放。"

    def release_output(self, reason: str) -> None:
        if self.connector.transport is None:
            self.state.output_active = False
            self._release_deadline = None
            self.state.message = "PWM 已释放，舵机控制器当前未连接。"
            return
        frame = make_frame(self.args.channel, self.args.frequency_hz, 0)
        try:
            self.connector.write(frame)
        except OSError as exc:
            self.state.message = f"释放 PWM 失败：{exc}"
        else:
            self.state.message = reason
        self.state.output_active = False
        self._release_deadline = None

    @staticmethod
    def _put_line(screen, row: int, text: str, style: int = 0) -> None:
        height, width = screen.getmaxyx()
        if 0 <= row < height and width > 1:
            try:
                screen.addnstr(row, 0, text, width - 1, style)
            except curses.error:
                pass

    def draw(self, screen) -> None:
        screen.erase()
        port = self.connector.port or "正在搜索稳定别名 / ttyUSB0 / ttyUSB1"
        duty = duty_for_angle(
            self.state.angle, self.args.pwm_min_duty, self.args.pwm_max_duty)
        pulse = pulse_width_us(self.args.frequency_hz, duty)
        sent = "未发送" if self.state.last_sent_angle is None else str(self.state.last_sent_angle)
        output = "点动中" if self.state.output_active else "已释放"
        closed = "未设置" if self.state.closed_angle is None else str(self.state.closed_angle)
        opened = "未设置" if self.state.open_angle is None else str(self.state.open_angle)

        self._put_line(screen, 0, "Robotac 舵机角度标定工具", curses.A_BOLD)
        self._put_line(screen, 2, f"串口：{port}")
        self._put_line(screen, 3, f"目标角度：{self.state.angle:3d} 度", curses.A_BOLD)
        self._put_line(
            screen,
            4,
            f"有效输出：占空比={duty}%  脉宽={pulse} 微秒  频率={self.args.frequency_hz} 赫兹",
        )
        self._put_line(screen, 5, f"最后发送角度：{sent} 度    输出状态：{output}")
        self._put_line(screen, 6, f"单次点动时间：{self.args.drive_seconds:.2f} 秒")
        self._put_line(screen, 7, f"已标记闭合角度：{closed} 度")
        self._put_line(screen, 8, f"已标记打开角度：{opened} 度")
        self._put_line(screen, 10, "方向键上/下：+/- 1 度    翻页键上/下：+/- 10 度")
        self._put_line(screen, 11, "方向键左/右：点动时间 -/+ 0.01 秒")
        self._put_line(screen, 12, "c：标记闭合角度    o：标记打开角度    r：重新点动")
        self._put_line(screen, 13, "空格：立即释放 PWM    q：释放 PWM 并退出")
        self._put_line(screen, 15, "显示的是目标角度，不是传感器测得的实际角度。")
        self._put_line(screen, 17, self.state.message, curses.A_REVERSE)
        screen.refresh()

    def _adjust(self, delta: int) -> None:
        self.state.angle = max(
            self.state.minimum_angle,
            min(self.state.maximum_angle, self.state.angle + delta),
        )
        self.send_angle()

    def _adjust_drive_seconds(self, delta: float) -> None:
        self.args.drive_seconds = max(
            0.02,
            min(0.30, round(self.args.drive_seconds + delta, 2)),
        )
        self.state.message = f"单次点动时间已调整为 {self.args.drive_seconds:.2f} 秒。"

    def _release_if_due(self, now: float) -> None:
        if (
            self.state.output_active
            and self._release_deadline is not None
            and now >= self._release_deadline
        ):
            self.release_output(self._release_message)

    def run_ui(self, screen) -> TunerState:
        try:
            curses.curs_set(0)
        except curses.error:
            pass
        screen.keypad(True)
        screen.timeout(10)
        if self.args.send_on_start:
            self.send_angle()
        while True:
            now = time.monotonic()
            self._release_if_due(now)
            if (
                self.connector.transport is not None
                and now - self._last_probe >= 0.25
            ):
                self._last_probe = now
                if not self.connector.probe():
                    self.state.output_active = False
                    self._release_deadline = None
                    self.state.message = "USB 已断开，正在重新搜索；不会自动重发旧角度。"
            if self.connector.transport is None and now - self._last_connect_attempt >= 1.0:
                self._last_connect_attempt = now
                if self.connector.connect():
                    self.state.message = "舵机控制器已连接。按 r 或方向键后才会发送指令。"
                else:
                    self.state.message = "正在等待稳定别名、ttyUSB0 或 ttyUSB1 上的控制器。"
            self.draw(screen)
            key = screen.getch()
            if key == curses.KEY_UP:
                self._adjust(1)
            elif key == curses.KEY_DOWN:
                self._adjust(-1)
            elif key == curses.KEY_PPAGE:
                self._adjust(10)
            elif key == curses.KEY_NPAGE:
                self._adjust(-10)
            elif key == curses.KEY_LEFT:
                self._adjust_drive_seconds(-0.01)
            elif key == curses.KEY_RIGHT:
                self._adjust_drive_seconds(0.01)
            elif key in (ord("c"), ord("C")):
                if self.state.last_sent_angle == self.state.angle:
                    self.state.closed_angle = self.state.angle
                    self.state.message = "已将当前目标角度标记为闭合角度。"
                else:
                    self.state.message = "请先发送当前角度，再将其标记为闭合角度。"
            elif key in (ord("o"), ord("O")):
                if self.state.last_sent_angle == self.state.angle:
                    self.state.open_angle = self.state.angle
                    self.state.message = "已将当前目标角度标记为打开角度。"
                else:
                    self.state.message = "请先发送当前角度，再将其标记为打开角度。"
            elif key in (ord("r"), ord("R")):
                self.send_angle()
            elif key == ord(" "):
                self.release_output("已按用户指令释放 PWM。")
            elif key in (ord("q"), ord("Q")):
                return self.state

    def close(self) -> None:
        if self.args.release_on_exit and self.connector.transport is not None:
            self.release_output("程序退出时已释放 PWM。")
        self.connector.disconnect()


class ChineseArgumentParser(argparse.ArgumentParser):
    def format_usage(self) -> str:
        return super().format_usage().replace("usage:", "用法：", 1)

    def format_help(self) -> str:
        return (
            super()
            .format_help()
            .replace("usage:", "用法：", 1)
            .replace("options:", "选项：", 1)
            .replace("optional arguments:", "可选参数：", 1)
        )

    def error(self, message: str) -> None:
        translations = (
            ("unrecognized arguments:", "无法识别的参数："),
            ("the following arguments are required:", "缺少以下必填参数："),
            ("expected one argument", "需要提供一个参数值"),
            ("not allowed with argument", "不能与此参数同时使用"),
            ("argument ", "参数 "),
        )
        for source, target in translations:
            message = message.replace(source, target)
        self.print_usage(sys.stderr)
        self.exit(2, f"{self.prog}：参数错误：{message}\n")


def integer_argument(value: str) -> int:
    try:
        return int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"必须是整数：{value}") from exc


def decimal_argument(value: str) -> float:
    try:
        return float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"必须是数字：{value}") from exc


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    raw_argv = sys.argv if argv is None else [sys.argv[0]] + list(argv)
    clean_argv = rospy.myargv(argv=raw_argv)[1:]
    parser = ChineseArgumentParser(
        description="使用终端方向键交互式标定 Robotac 舵机的打开和闭合角度。",
        add_help=False,
    )
    parser.add_argument(
        "-h",
        "--help",
        action="help",
        help="显示本帮助信息并退出。",
    )
    parser.add_argument(
        "--port",
        action="append",
        default=None,
        metavar="串口",
        help="串口设备或通配符，可重复指定；默认尝试稳定别名、ttyUSB0 和 ttyUSB1。",
    )
    parser.add_argument(
        "--baudrate", type=integer_argument, default=115200, metavar="波特率",
        help="串口波特率，默认 115200。")
    parser.add_argument(
        "--channel", type=integer_argument, default=1, metavar="通道",
        help="控制板通道，默认 1。")
    parser.add_argument(
        "--frequency-hz", type=integer_argument, default=50, metavar="频率",
        help="PWM 频率，默认 50 赫兹。")
    parser.add_argument(
        "--pwm-min-duty", type=integer_argument, default=3, metavar="最小占空比",
        help="0 度对应的整数占空比，默认 3。")
    parser.add_argument(
        "--pwm-max-duty", type=integer_argument, default=12, metavar="最大占空比",
        help="180 度对应的整数占空比，默认 12。")
    parser.add_argument(
        "--min-angle", type=integer_argument, default=0, metavar="最小角度",
        help="允许测试的最小角度，默认 0 度。")
    parser.add_argument(
        "--max-angle", type=integer_argument, default=180, metavar="最大角度",
        help="允许测试的最大角度，默认 180 度。")
    parser.add_argument(
        "--start-angle", type=integer_argument, default=45, metavar="起始角度",
        help="界面初始目标角度，默认 45 度；默认不会自动发送。")
    parser.add_argument(
        "--drive-seconds", type=decimal_argument, default=0.03, metavar="点动秒数",
        help="每次按键输出 PWM 的时间，默认 0.03 秒，随后自动释放。")
    parser.add_argument(
        "--send-on-start", action="store_true",
        help="启动界面后立即发送起始角度，机械机构未标定时不要使用。")
    parser.add_argument(
        "--hold-on-exit", dest="release_on_exit", action="store_false",
        help="退出时继续保持 PWM；通常不要使用。")
    parser.add_argument("--yes", action="store_true", help="跳过交互式安全确认。")
    parser.set_defaults(release_on_exit=True)
    args = parser.parse_args(clean_argv)
    if args.port is None:
        args.port = list(AUTO_PORT_PATTERNS)
    if not 0 <= args.min_angle < args.max_angle <= 180:
        parser.error("角度范围必须按从小到大设置，并且位于 0 到 180 度之间")
    if not args.min_angle <= args.start_angle <= args.max_angle:
        parser.error("起始角度必须位于设置的角度范围内")
    if not 0.02 <= args.drive_seconds <= 0.30:
        parser.error("点动时间必须位于 0.02 到 0.30 秒之间")
    try:
        duty_for_angle(args.start_angle, args.pwm_min_duty, args.pwm_max_duty)
        make_frame(args.channel, args.frequency_hz, 0)
    except ValueError as exc:
        protocol_errors = {
            "channel must fit in one byte": "控制通道必须位于 0 到 255 之间",
            "frequency_hz must be between 1 and 65535": "PWM 频率必须位于 1 到 65535 赫兹之间",
            "duty must be between 0 and 100": "PWM 占空比必须位于 0 到 100 之间",
            "angle must be between 0 and 180 degrees": "角度必须位于 0 到 180 度之间",
            "PWM duty calibration must be ordered within 0..100":
                "PWM 最小占空比必须小于最大占空比，且两者均位于 0 到 100 之间",
        }
        parser.error(protocol_errors.get(str(exc), f"舵机参数无效：{exc}"))
    return args


def confirm_safety(args: argparse.Namespace) -> None:
    if args.yes:
        return
    print("警告：本工具会控制真实舵机运动，可能使抛投机构的铁丝顶死。")
    print("请先拆除螺旋桨、让手远离机构，并做好立即切断舵机电源的准备。")
    answer = input("请输入 YES 或 Y 后继续：").strip().upper()
    if answer not in {"YES", "Y"}:
        raise SystemExit("操作已取消，未发送任何舵机控制指令。")


def print_result(state: TunerState) -> None:
    print("\n标定结果（目标角度，并非传感器测得的实际角度）：")
    if state.closed_angle is not None:
        print(f"闭合角度 closed_angle: {state.closed_angle}")
    else:
        print("闭合角度 closed_angle：未标记")
    if state.open_angle is not None:
        print(f"打开角度 open_angle: {state.open_angle}")
    else:
        print("打开角度 open_angle：未标记")


def main(argv: Optional[Sequence[str]] = None) -> int:
    locale.setlocale(locale.LC_ALL, "")
    args = parse_args(argv)
    if not sys.stdin.isatty() or not sys.stdout.isatty():
        raise SystemExit("servo_angle_tuner.py 必须在交互式终端中运行。")
    confirm_safety(args)
    tuner = ServoAngleTuner(args)
    try:
        state = curses.wrapper(tuner.run_ui)
    except KeyboardInterrupt:
        state = tuner.state
    finally:
        tuner.close()
    print_result(state)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

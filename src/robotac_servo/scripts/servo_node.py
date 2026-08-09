#!/usr/bin/env python3
"""ROS Boolean switch for the channel-1 USB PWM servo controller.

The controller protocol is the one used by yundrone_blink:
5A <channel> <frequency high> <frequency low> <duty percent>.
Only the ``control`` Boolean topic is exposed. False maps to 0 degrees and True
maps to the configured opening angle (45 degrees by default).
"""

from __future__ import annotations

import threading
import os
import termios
import uuid

import rospy
from std_msgs.msg import Bool, String

try:
    import serial
except ImportError as exc:  # pragma: no cover - exercised on deployment host
    serial = None
    SERIAL_IMPORT_ERROR = exc
else:
    SERIAL_IMPORT_ERROR = None

SERIAL_ERRORS = (OSError,) if serial is None else (OSError, serial.SerialException)


class PosixSerial:
    """Small Linux serial writer used when pyserial is unavailable."""

    def __init__(self, port: str, baudrate: int) -> None:
        self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY)
        attrs = termios.tcgetattr(self.fd)
        attrs[0] = 0
        attrs[1] = 0
        attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
        attrs[3] = 0
        speed = getattr(termios, f"B{baudrate}")
        attrs[4] = speed
        attrs[5] = speed
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 5
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        self.is_open = True

    def write(self, data: bytes) -> int:
        written = 0
        while written < len(data):
            count = os.write(self.fd, data[written:])
            if count <= 0:
                raise OSError("serial device accepted no data")
            written += count
        return written

    def flush(self) -> None:
        termios.tcdrain(self.fd)

    def close(self) -> None:
        if self.is_open:
            os.close(self.fd)
            self.is_open = False


BAUDRATE = 115200
CHANNEL = 1
FREQUENCY_HZ = 50
CLOSED_ANGLE = 0
DEFAULT_OPEN_ANGLE = 45
MIN_DUTY = 3
MAX_DUTY = 12


def duty_for_angle(angle: int) -> int:
    if not 0 <= angle <= 180:
        raise ValueError(f"servo angle must be between 0 and 180, got {angle}")
    return round(MIN_DUTY + (MAX_DUTY - MIN_DUTY) * angle / 180.0)


def make_frame(duty: int) -> bytes:
    return bytes((0x5A, CHANNEL, FREQUENCY_HZ >> 8, FREQUENCY_HZ & 0xFF, duty))


def bool_param(name: str, default: bool) -> bool:
    """Read a ROS bool parameter without treating the string 'false' as true."""
    value = rospy.get_param(name, default)
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"true", "1", "yes", "on"}:
            return True
        if normalized in {"false", "0", "no", "off"}:
            return False
    raise RuntimeError(f"{name} must be a boolean")


class ServoSwitch:
    def __init__(self) -> None:
        self.port = rospy.get_param("~port", "/dev/robotac_servo")
        self.baudrate = int(rospy.get_param("~baudrate", BAUDRATE))
        self.open_angle = int(rospy.get_param("~open_angle", DEFAULT_OPEN_ANGLE))
        if not 0 <= self.open_angle <= 180:
            raise RuntimeError("~open_angle must be between 0 and 180 degrees")
        self.initial_open = bool_param("~initial_open", False)
        self.close_on_shutdown = bool_param("~close_on_shutdown", True)
        self._lock = threading.Lock()
        self._connection = None
        self._command_sequence = 0
        self._boot_id = uuid.uuid4().hex

        try:
            if serial is not None:
                self._connection = serial.Serial(
                    port=self.port,
                    baudrate=self.baudrate,
                    timeout=0.5,
                    write_timeout=1.0,
                    rtscts=False,
                    dsrdtr=False,
                    xonxoff=False,
                )
            else:
                self._connection = PosixSerial(self.port, self.baudrate)
        except SERIAL_ERRORS as exc:
            raise RuntimeError(f"cannot open servo port {self.port}: {exc}") from exc

        # This is the only control interface. The initial state is a parameter
        # so a bench launch can select the configured opening angle without a
        # second command.
        self._subscriber = rospy.Subscriber(
            "control", Bool, self._on_open, queue_size=1
        )
        # This is feedback only.  It confirms that the command frame was
        # accepted by the USB serial device; it is not a measured servo angle.
        self._status_pub = rospy.Publisher("status", String, queue_size=5, latch=True)
        self._send_open(self.initial_open)
        rospy.on_shutdown(self._shutdown)
        rospy.loginfo(
            "servo switch ready on %s: false=0 degrees, true=%d degrees",
            self.port, self.open_angle,
        )

    def _publish_status(self, is_open: bool, success: bool) -> None:
        state = "open" if is_open else "closed"
        self._status_pub.publish(String(data=(
            "state=%s success=%s seq=%d boot=%s" %
            (state, str(success).lower(), self._command_sequence, self._boot_id))))

    def _send_open(self, is_open: bool) -> bool:
        angle = self.open_angle if is_open else CLOSED_ANGLE
        frame = make_frame(duty_for_angle(angle))
        self._command_sequence += 1
        with self._lock:
            if self._connection is None or not self._connection.is_open:
                rospy.logerr("servo serial connection is not open")
                self._publish_status(is_open, False)
                return False
            try:
                self._connection.write(frame)
                self._connection.flush()
            except SERIAL_ERRORS as exc:
                rospy.logerr("failed to send servo command: %s", exc)
                self._publish_status(is_open, False)
                return False
        rospy.loginfo("servo %s: angle=%d duty=%d%% frame=%s",
                      "open" if is_open else "closed", angle,
                      duty_for_angle(angle), frame.hex(" ").upper())
        self._publish_status(is_open, True)
        return True

    def _on_open(self, message: Bool) -> None:
        self._send_open(bool(message.data))

    def _shutdown(self) -> None:
        if self.close_on_shutdown and self._connection is not None:
            self._send_open(False)
        if self._connection is not None:
            self._connection.close()


def main() -> int:
    rospy.init_node("servo_switch")
    try:
        ServoSwitch()
    except RuntimeError as exc:
        rospy.logfatal("%s", exc)
        return 2
    rospy.spin()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

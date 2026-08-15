"""Linux 串口传输，仅负责打开设备和完整写入控制帧。"""

import fcntl
import os
import termios

try:
    import serial
except ImportError:
    serial = None


class SerialPort(object):
    def __init__(self, path, baudrate):
        self.path = path
        self.serial_port = None
        self.fd = None
        if serial is not None:
            self._open_pyserial(baudrate)
        else:
            self._open_posix(baudrate)

    def _open_pyserial(self, baudrate):
        port = serial.Serial(port=None, baudrate=baudrate, timeout=0.0,
                             write_timeout=1.0, rtscts=False,
                             dsrdtr=False, xonxoff=False)
        port.dtr = False
        port.rts = False
        port.port = self.path
        port.open()
        try:
            fcntl.flock(port.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except (OSError, IOError):
            port.close()
            raise RuntimeError("串口已被其他进程占用: %s" % self.path)
        self.serial_port = port

    def _open_posix(self, baudrate):
        flags = os.O_RDWR | os.O_NOCTTY
        if hasattr(os, "O_CLOEXEC"):
            flags |= os.O_CLOEXEC
        fd = os.open(self.path, flags)
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            attrs = termios.tcgetattr(fd)
            attrs[0] = 0
            attrs[1] = 0
            attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
            attrs[3] = 0
            speed = getattr(termios, "B%d" % baudrate)
            attrs[4] = speed
            attrs[5] = speed
            attrs[6][termios.VMIN] = 0
            attrs[6][termios.VTIME] = 0
            termios.tcsetattr(fd, termios.TCSANOW, attrs)
        except Exception:
            os.close(fd)
            raise
        self.fd = fd

    def is_open(self):
        if self.serial_port is not None:
            return self.serial_port.is_open
        return self.fd is not None

    def write(self, data):
        if self.serial_port is not None:
            count = self.serial_port.write(data)
            self.serial_port.flush()
            if count != len(data):
                raise OSError("串口写入长度不完整")
            return
        if self.fd is None:
            raise OSError("串口未打开")
        offset = 0
        while offset < len(data):
            count = os.write(self.fd, data[offset:])
            if count <= 0:
                raise OSError("串口未接受数据")
            offset += count
        termios.tcdrain(self.fd)

    def probe(self):
        if not self.is_open():
            raise OSError("串口已关闭")
        if self.serial_port is not None:
            return self.serial_port.in_waiting
        return termios.tcgetattr(self.fd)

    def close(self):
        if self.serial_port is not None:
            if self.serial_port.is_open:
                self.serial_port.close()
            return
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None


def find_port(configured_path, candidates):
    if configured_path and configured_path != "auto":
        if os.path.exists(configured_path):
            return configured_path
        return None
    for path in candidates:
        if os.path.exists(path):
            return path
    return None

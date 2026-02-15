#!/usr/bin/env python3
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import serial


class Controller(Node):
    def __init__(self):
        super().__init__('controller')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('readback', True)
        self.declare_parameter('cmd_rate_hz', 50.0)   # resend last cmd at this rate
        self.declare_parameter('timeout_s', 0.5)      # if no cmd_vel received, send STOP once

        self.port = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baud').value)
        self.readback = bool(self.get_parameter('readback').value)
        self.cmd_rate_hz = float(self.get_parameter('cmd_rate_hz').value)
        self.timeout_s = float(self.get_parameter('timeout_s').value)

        self._lock = threading.Lock()
        self._stop_reader = threading.Event()

        self._last_rx_time = time.time()
        self._last_cmd = (0.0, 0.0)   # (v, w)
        self._sent_stop = False

        self.get_logger().info(f"Opening serial: {self.port} @ {self.baud}")
        self.ser = serial.Serial(self.port, self.baud, timeout=0.1, write_timeout=0.5)

        # Arduino often resets when serial opens; give it a moment
        time.sleep(2.0)
        self.get_logger().info("Serial open. Bridge ready.")

        # Subscribe to cmd_vel
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_cb, 10)

        # Timer to send commands at fixed rate
        period = 1.0 / self.cmd_rate_hz if self.cmd_rate_hz > 0 else 0.05
        self.timer = self.create_timer(period, self._tx_loop)

        if self.readback:
            self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
            self._reader_thread.start()

    def cmd_vel_cb(self, msg: Twist):
        v = float(msg.linear.x)
        w = float(msg.angular.z)
        self._last_cmd = (v, w)
        self._last_rx_time = time.time()
        self._sent_stop = False

    def _write_line(self, line: str):
        data = (line.strip() + "\n").encode("utf-8")
        with self._lock:
            self.ser.write(data)
            self.ser.flush()

    def _tx_loop(self):
        now = time.time()
        dt = now - self._last_rx_time

        if dt > self.timeout_s:
            # no recent cmd_vel: send STOP once
            if not self._sent_stop:
                self._write_line("STOP")
                self.get_logger().warn("cmd_vel timeout -> Sent STOP")
                self._sent_stop = True
            return

        v, w = self._last_cmd
        self._write_line(f"VEL {v:.3f} {w:.3f}")

    def _reader_loop(self):
        buf = b""
        while rclpy.ok() and not self._stop_reader.is_set():
            try:
                chunk = self.ser.read(256)
                if not chunk:
                    continue
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    line = line.replace(b"\r", b"").decode("utf-8", errors="replace").strip()
                    if line:
                        self.get_logger().info(f"Arduino: {line}")
            except Exception as e:
                self.get_logger().warn(f"Serial read error: {e}")
                time.sleep(0.2)

    def destroy_node(self):
        self._stop_reader.set()
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

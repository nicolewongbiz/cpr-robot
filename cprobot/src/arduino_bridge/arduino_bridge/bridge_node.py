#!/usr/bin/env python3
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty

import serial


class ArduinoBridge(Node):
    """
    ROS2 -> Arduino serial bridge

    Subscribes:
      - /turn_90 (std_msgs/Empty)  -> sends "TURN90\\n" to Arduino

    Arduino firmware expected:
      - Line-based commands, e.g. "TURN90"
      - Serial prints optional: READY / ACK TURN90 / DONE TURN90
    """

    def __init__(self):
        super().__init__('arduino_bridge')

        # Parameters (change via CLI or launch)
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('write_cooldown_s', 0.0)  # extra guard; Arduino already has 1s cooldown
        self.declare_parameter('readback', True)

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.write_cooldown_s = self.get_parameter('write_cooldown_s').get_parameter_value().double_value
        self.readback = self.get_parameter('readback').get_parameter_value().bool_value

        self._last_write_time = 0.0
        self._lock = threading.Lock()
        self._stop_reader = threading.Event()

        self.get_logger().info(f"Opening serial port: {self.port} @ {self.baud}")
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=int(self.baud),
                timeout=0.1,          # non-blocking-ish reads
                write_timeout=0.5
            )
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port {self.port}: {e}")
            raise

        # Give Arduino time to reset when serial opens (common on Uno/Nano)
        time.sleep(2.0)
        self.get_logger().info("Serial open. Bridge ready.")

        self.sub_turn = self.create_subscription(Empty, 'turn_90', self.turn90_cb, 10)

        if self.readback:
            self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
            self._reader_thread.start()

    def _write_line(self, line: str) -> bool:
        """
        Writes a newline-terminated command to Arduino with optional cooldown.
        """
        now = time.time()
        if self.write_cooldown_s > 0 and (now - self._last_write_time) < self.write_cooldown_s:
            self.get_logger().warn("Command ignored due to write cooldown.")
            return False

        data = (line.strip() + "\n").encode("utf-8")

        with self._lock:
            try:
                self.ser.write(data)
                self.ser.flush()
                self._last_write_time = now
                return True
            except Exception as e:
                self.get_logger().error(f"Serial write failed: {e}")
                return False

    def turn90_cb(self, _msg: Empty) -> None:
        ok = self._write_line("TURN90")
        if ok:
            self.get_logger().info("Sent: TURN90")

    def _reader_loop(self) -> None:
        """
        Reads lines from Arduino and logs them.
        """
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
            if hasattr(self, "ser") and self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
ROS2 node: listen for YOLO UDP from cam0 (cx, area, conf, t), publish /person_det and yolo/cmd_vel.
Host: scripts/yolo_udp_sender.py sends cam0 YOLO to port 5000. Container: --net=host.
"""
import socket

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist


class PersonFollowerUdp(Node):
    def __init__(self):
        super().__init__("person_follower_udp")

        self.declare_parameter("udp_port", 5000)
        self.declare_parameter("udp_bind", "0.0.0.0")
        self.declare_parameter("publish_hz", 20.0)
        self.declare_parameter("close_area", 0.5)
        self.declare_parameter("kp_turn", 0.01)
        self.declare_parameter("max_lin", 0.5)
        self.declare_parameter("max_ang", 0.5)

        port = int(self.get_parameter("udp_port").value)
        bind = self.get_parameter("udp_bind").value
        publish_hz = float(self.get_parameter("publish_hz").value)
        self.close_area = float(self.get_parameter("close_area").value)
        self.kp_turn = float(self.get_parameter("kp_turn").value)
        self.max_lin = float(self.get_parameter("max_lin").value)
        self.max_ang = float(self.get_parameter("max_ang").value)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.1)
        self.sock.bind((bind, port))

        self.pub_det = self.create_publisher(Float32MultiArray, "person_det", 10)
        self.pub_cmd = self.create_publisher(Twist, "yolo/cmd_vel", 10)
        self._last_cmd = Twist()
        self.timer = self.create_timer(0.02, self.tick)  # 50 Hz UDP poll
        self.pub_timer = self.create_timer(1.0 / publish_hz, self._publish_cmd)
        self._last_log_time = 0.0

        self.get_logger().info(f"Listening UDP {bind}:{port} → /person_det, yolo/cmd_vel @ {publish_hz} Hz")

    def clamp(self, x, lo, hi):
        return max(lo, min(hi, x))

    def _publish_cmd(self):
        """Publish at fixed rate for smooth Arduino commands."""
        self.pub_cmd.publish(self._last_cmd)

    def tick(self):
        try:
            data, _ = self.sock.recvfrom(256)
        except socket.timeout:
            return
        line = data.decode("utf-8", errors="ignore").strip()
        parts = line.split(",")
        if len(parts) != 4:
            return
        try:
            cx_norm = float(parts[0])
            area_norm = float(parts[1])
            conf = float(parts[2])
            t = float(parts[3])
        except ValueError:
            return

        # Publish detection: data = [cx_norm, area_norm, conf]
        self.pub_det.publish(Float32MultiArray(data=[cx_norm, area_norm, conf]))

        # Compute cmd; publish happens at fixed rate in _publish_cmd
        cmd = Twist()
        if not (cx_norm == cx_norm and area_norm == area_norm):
            self._last_cmd = cmd
            return

        ang = -self.kp_turn * cx_norm
        ang = self.clamp(ang, -self.max_ang, self.max_ang)
        if area_norm >= self.close_area:
            lin = 0.0
        else:
            lin = self.max_lin
            if abs(cx_norm) > 0.3:
                lin *= 0.5
        cmd.linear.x = float(lin)
        cmd.angular.z = float(ang)
        self._last_cmd = cmd

        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._last_log_time >= 0.5:
            self.get_logger().info(f"person cx={cx_norm:.2f} area={area_norm:.3f} conf={conf:.2f}")
            self._last_log_time = now

    def destroy_node(self):
        try:
            self.sock.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = PersonFollowerUdp()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()

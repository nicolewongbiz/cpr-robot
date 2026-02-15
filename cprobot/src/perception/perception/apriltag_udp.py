#!/usr/bin/env python3
"""
ROS2 node: listen for AprilTag UDP from cam1 (tag_id, margin, x, y, z, yaw, t), publish tag/seen and tag/cmd_vel.
Host: scripts/yolo_udp_sender.py sends cam1 AprilTag to port 5010. Container: --net=host.
"""
import socket

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist


class AprilTagUdp(Node):
    def __init__(self):
        super().__init__("apriltag_udp")

        self.declare_parameter("udp_port", 5010)
        self.declare_parameter("udp_bind", "0.0.0.0")
        self.declare_parameter("publish_hz", 20.0)
        self.declare_parameter("z_des", 0.8)
        self.declare_parameter("x_des", 0.35)
        self.declare_parameter("kp_z", 0.8)
        self.declare_parameter("kp_x", 1.2)
        self.declare_parameter("kp_yaw", 1.0)
        self.declare_parameter("max_lin", 0.35)
        self.declare_parameter("max_ang", 1.5)

        port = int(self.get_parameter("udp_port").value)
        bind = self.get_parameter("udp_bind").value
        publish_hz = float(self.get_parameter("publish_hz").value)
        self.z_des = float(self.get_parameter("z_des").value)
        self.x_des = float(self.get_parameter("x_des").value)
        self.kp_z = float(self.get_parameter("kp_z").value)
        self.kp_x = float(self.get_parameter("kp_x").value)
        self.kp_yaw = float(self.get_parameter("kp_yaw").value)
        self.max_lin = float(self.get_parameter("max_lin").value)
        self.max_ang = float(self.get_parameter("max_ang").value)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.1)
        self.sock.bind((bind, port))

        self.pub_seen = self.create_publisher(Bool, "tag/seen", 10)
        self.pub_cmd = self.create_publisher(Twist, "tag/cmd_vel", 10)
        self._last_cmd = Twist()
        self.timer = self.create_timer(0.02, self.tick)  # 50 Hz UDP poll
        self.pub_timer = self.create_timer(1.0 / publish_hz, self._publish_cmd)
        self._last_log_time = 0.0

        self.get_logger().info(f"AprilTag UDP {bind}:{port} → tag/seen, tag/cmd_vel @ {publish_hz} Hz")

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

        parts = data.decode("utf-8", errors="ignore").strip().split(",")
        if len(parts) != 7:
            return
        try:
            tag_id = float(parts[0])
            margin = float(parts[1])
            x = float(parts[2])
            y = float(parts[3])
            z = float(parts[4])
            yaw = float(parts[5])
        except ValueError:
            return

        if tag_id != tag_id:  # nan
            self.pub_seen.publish(Bool(data=False))
            self._last_cmd = Twist()
            return

        self.pub_seen.publish(Bool(data=True))

        ez = z - self.z_des
        ex = x - self.x_des
        lin = self.clamp(self.kp_z * ez, -self.max_lin, self.max_lin)
        ang = self.kp_x * ex + self.kp_yaw * yaw
        ang = self.clamp(ang, -self.max_ang, self.max_ang)
        if abs(ex) > 0.25:
            lin *= 0.3

        cmd = Twist()
        cmd.linear.x = float(lin)
        cmd.angular.z = float(ang)
        self._last_cmd = cmd

        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._last_log_time >= 0.5:
            self.get_logger().info(f"tag_id={int(tag_id)} z={z:.3f}m x={x:.3f}m yaw={yaw:.2f}")
            self._last_log_time = now

    def destroy_node(self):
        try:
            self.sock.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = AprilTagUdp()
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

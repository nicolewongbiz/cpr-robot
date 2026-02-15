#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist


class CmdVelArbiter(Node):
    def __init__(self):
        super().__init__("cmd_vel_arbiter")

        self.declare_parameter("tag_hold_s", 0.5)    # keep tag mode for this long after last seen
        self.declare_parameter("stale_stop_s", 0.5)  # stop if chosen cmd is stale
        self.declare_parameter("publish_hz", 20.0)

        self.tag_hold_s = float(self.get_parameter("tag_hold_s").value)
        self.stale_stop_s = float(self.get_parameter("stale_stop_s").value)

        self.last_tag_seen_time = 0.0
        self.tag_seen = False

        self.last_yolo_cmd = Twist()
        self.last_yolo_time = 0.0

        self.last_tag_cmd = Twist()
        self.last_tag_time = 0.0

        self.sub_seen = self.create_subscription(Bool, "tag/seen", self.seen_cb, 10)
        self.sub_yolo = self.create_subscription(Twist, "yolo/cmd_vel", self.yolo_cb, 10)
        self.sub_tag = self.create_subscription(Twist, "tag/cmd_vel", self.tag_cb, 10)

        self.pub = self.create_publisher(Twist, "cmd_vel", 10)

        period = 1.0 / float(self.get_parameter("publish_hz").value)
        self.timer = self.create_timer(period, self.tick)

        self.get_logger().info("CmdVel arbiter started.")

    def seen_cb(self, msg: Bool):
        self.tag_seen = bool(msg.data)
        if self.tag_seen:
            self.last_tag_seen_time = time.time()

    def yolo_cb(self, msg: Twist):
        self.last_yolo_cmd = msg
        self.last_yolo_time = time.time()

    def tag_cb(self, msg: Twist):
        self.last_tag_cmd = msg
        self.last_tag_time = time.time()

    def tick(self):
        now = time.time()
        out = Twist()

        in_tag_mode = (now - self.last_tag_seen_time) <= self.tag_hold_s

        if in_tag_mode:
            # use tag cmd if fresh
            if (now - self.last_tag_time) <= self.stale_stop_s:
                out = self.last_tag_cmd
        else:
            # use yolo cmd if fresh
            if (now - self.last_yolo_time) <= self.stale_stop_s:
                out = self.last_yolo_cmd

        self.pub.publish(out)


def main():
    rclpy.init()
    node = CmdVelArbiter()
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

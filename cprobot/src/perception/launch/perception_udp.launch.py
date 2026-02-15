"""
Launch UDP perception: person follower (YOLO) + AprilTag follower. No camera in container.
Host: python3 scripts/yolo_udp_sender.py  (cam0=YOLO→5000, cam1=AprilTag→5010)
Container: ros2 launch perception perception_udp.launch.py
Docker: --net=host so the container receives UDP.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    yolo_port = LaunchConfiguration("yolo_udp_port", default_value="5000")
    tag_port = LaunchConfiguration("tag_udp_port", default_value="5010")
    publish_hz = LaunchConfiguration("publish_hz", default_value="20.0")

    return LaunchDescription([
        DeclareLaunchArgument(
            "yolo_udp_port",
            default_value="5000",
            description="UDP port for YOLO (sender cam0 → 5000)",
        ),
        DeclareLaunchArgument(
            "tag_udp_port",
            default_value="5010",
            description="UDP port for AprilTag (sender cam1 → 5010)",
        ),
        DeclareLaunchArgument(
            "publish_hz",
            default_value="20.0",
            description="Fixed cmd_vel publish rate (Hz) for smooth Arduino commands",
        ),
        Node(
            package="perception",
            executable="person_follower_udp",
            name="person_follower_udp",
            output="screen",
            parameters=[{"udp_port": yolo_port, "publish_hz": publish_hz}],
        ),
        Node(
            package="perception",
            executable="apriltag_udp",
            name="apriltag_udp",
            output="screen",
            parameters=[{"udp_port": tag_port, "publish_hz": publish_hz}],
        ),
        Node(
            package="perception",
            executable="cmd_vel_arbiter",
            name="cmd_vel_arbiter",
            output="screen",
            parameters=[{"publish_hz": publish_hz}],
        ),
    ])

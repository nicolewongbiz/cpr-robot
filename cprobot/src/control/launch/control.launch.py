from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port = LaunchConfiguration("port")
    baud = LaunchConfiguration("baud")

    return LaunchDescription([
        DeclareLaunchArgument(
            "port",
            default_value="",
            description="Serial port for Arduino (empty = disabled)"
        ),
        DeclareLaunchArgument(
            "baud",
            default_value="115200",
            description="Serial baud rate"
        ),

        Node(
            package="control",
            executable="controller",
            name="controller",
            output="screen",
            parameters=[{
                "port": port,
                "baud": baud,
            }],
        ),
    ])

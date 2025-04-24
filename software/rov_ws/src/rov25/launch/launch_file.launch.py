from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_dir = get_package_share_directory("rov25")
    config = os.path.join(package_dir, "config", "pid.yaml")
    gui_script = "/home/abdelrhman/MATE-ROV-2025/software/gui/main.py"

    return LaunchDescription([
        Node(
            package='rov25',
            executable='movement_feedback',
            name='kinematic_model',
            output='screen',
            parameters=[config]
        ),
        Node(
            package='rov25',
            executable='motion_control',
        ),
        Node(
            package='rov25',
            executable='joystick',
        ),
        ExecuteProcess(
            cmd=['python3', gui_script],
        )
    ])

from launch import LaunchDescription
from launch_ros.actions.node import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    moveit_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('arm6dof_moveit_config'), 'launch', 'demo.launch.py')
        )
    )

    return LaunchDescription([
        ExecuteProcess(cmd=['stty', '-F', '/dev/ttyUSB0', '2000000'], output='screen'),
        Node(
            namespace="arm6dof",
            executable="armnode",
            package="arm6dof"
        ),
        moveit_demo,
    ])

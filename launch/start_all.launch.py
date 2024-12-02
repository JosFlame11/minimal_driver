import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
def generate_launch_description():
    # Get the package share directory
    package_name = "minimal_driver"

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name),'launch','camera_init.launch.py'
        )])
    )
    micro_ros = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name),'launch','micro_ros_init.launch.py'
        )])
    )
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name),'launch','robot_init.launch.py'
        )])
    )
    robot_behavior = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name),'launch','robot_behavior_init.launch.py'
        )])
    )

    nodes = [
        camera,
        micro_ros,
        robot,
        robot_behavior
    ]
    return LaunchDescription(nodes)
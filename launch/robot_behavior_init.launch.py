from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = "robot_behavior"
        # Sensors Node
    sensors = Node(
        package=package_name,  # Replace with your actual package name
        executable='sensors_node',  # Replace with your executable name
        name='sensors',
        output='screen'
    )

    # Artificial Vision Node
    av = Node(
        package=package_name,  # Replace with your actual package name
        executable='av_node',  # Replace with your executable name
        name='artificial_vision'
    )

    # Vision Command Node
    av_cmd = Node(
        package=package_name,  # Replace with your actual package name
        executable='vision_cmd',  # Replace with your executable name
        name='vision_cmd',
        output='screen'
    )

    # Path Tracking Node
    tracking = Node(
        package=package_name,  # Replace with your actual package name
        executable='path_tracking_node',  # Replace with your executable name
        name='path_tracking'
    )
    nodes = [
        sensors,
        av,
        av_cmd,
        tracking
    ]

    return LaunchDescription(nodes)

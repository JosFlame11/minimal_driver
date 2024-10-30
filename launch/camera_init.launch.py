from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        # Declare arguments for camera settings
        DeclareLaunchArgument(
            'video_device', default_value='/dev/video0',
            description='Video device to use for the camera'
        ),
        DeclareLaunchArgument(
            'image_width', default_value='320',
            description='Width of the camera image'
        ),
        DeclareLaunchArgument(
            'image_height', default_value='240',
            description='Height of the camera image'
        ),
        DeclareLaunchArgument(
            'frame_rate', default_value='30.0',
            description='Frame rate for the camera'
        ),
        
        # Launch the camera driver (v4l2_camera)
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='usb_camera',
            parameters=[
                {
                    'video_device': LaunchConfiguration('video_device'),
                    'image_width': LaunchConfiguration('image_width'),
                    'image_height': LaunchConfiguration('image_height'),
                    'frame_rate': LaunchConfiguration('frame_rate'),
                    'pixel_format': 'YUYV'  # Modify as per your camera's format
                }
            ],
            output='screen'
        )
    ])
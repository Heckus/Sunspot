from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='camera_node',  # You can keep the same node name
            namespace='camera',
            parameters=[{
                'video_device': '/dev/video0',
                'image_size': [640, 480],
                # This will automatically load your saved calibration file
                'camera_info_url': 'file:///root/.ros/camera_info/default_camera.yaml'
            }]
        )
    ])
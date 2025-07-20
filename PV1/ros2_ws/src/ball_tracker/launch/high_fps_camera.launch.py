from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # --- QoS Profile for High-FPS Video ---
    # 'Best Effort' tells ROS it's OK to drop frames if the network is busy.
    # 'Depth: 1' means we only care about the absolute latest frame.
    video_qos_profile = {
        'reliability': 'best_effort',
        'durability': 'volatile',
        'history': 'keep_last',
        'depth': 1,
    }

    # --- Camera Node Configuration ---
    camera_node = Node(
        package='camera_ros',
        executable='camera_node',
        name='camera',
        parameters=[
            # Match the resolution and FPS that works on your host
            {'width': 1280},
            {'height': 720},
            # For 90 FPS, the frame duration is 1,000,000 / 90 = 11111 microseconds
            {'FrameDurationLimits': '[11111, 11111]'},
            # YUYV is often more efficient than RGB/BGR formats
            {'format': 'YUYV'},
        ],
        output='screen'
    )

    # --- Image Viewer Node with Correct QoS ---
    image_viewer_node = Node(
        package='image_view',
        executable='image_view',
        name='image_viewer',
        remappings=[('image', '/image_raw')],
        # Apply the high-FPS QoS profile to the subscriber
        ros_arguments=[
            '--qos-profile-reliability', video_qos_profile['reliability'],
            '--qos-profile-durability', video_qos_profile['durability'],
            '--qos-profile-history', video_qos_profile['history'],
            '--qos-profile-depth', str(video_qos_profile['depth']),
        ],
        output='screen'
    )

    return LaunchDescription([
        camera_node,
        image_viewer_node
    ])
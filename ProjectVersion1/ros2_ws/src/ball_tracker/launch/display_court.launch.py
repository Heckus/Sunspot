from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    package_share_dir = get_package_share_directory('ball_tracker')
    urdf_file = os.path.join(package_share_dir, 'urdf', 'court.urdf')

    # 1. Robot State Publisher
    # Publishes the transform from 'world' to 'court_link' based on the URDF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        arguments=[urdf_file]
    )
    
    # 2. Static Transform Publisher (The New Addition!)
    # Publishes a static transform from 'map' to 'world'. This creates the 'map' frame.
    static_tf_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=[
            '0', '0', '0',  # x, y, z offset
            '0', '0', '0',  # roll, pitch, yaw rotation
            'map',         # parent frame
            'world'        # child frame
        ]
    )

    # 3. RViz2
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        static_tf_publisher_node, # Add the new node to the launch description
        rviz2_node
    ])
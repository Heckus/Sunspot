from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Get the path to your package
    package_share_dir = get_package_share_directory('ball_tracker')

    # Get the path to the URDF file
    urdf_file = os.path.join(package_share_dir, 'urdf', 'court.urdf')

    # --- Nodes to Launch ---

    # 1. Robot State Publisher
    # Reads the URDF file and publishes the /robot_description topic and tf transforms
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        arguments=[urdf_file]
    )

    # 2. RViz2
    # Visualizer tool
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2'
        # You can add a custom RViz config file here if you have one
        # arguments=['-d', os.path.join(package_share_dir, 'rviz', 'court.rviz')]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        rviz2_node
    ])
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directory
    package_share_dir = get_package_share_directory('ball_tracker')

    # Define the default path to the YOLO model and config files
    default_yolo_model_path = os.path.join(package_share_dir, '..', '..', '..', 'src', 'ball_tracker', 'config', 'models_ball4.pt')
    default_intrinsics_path = os.path.join(package_share_dir, 'config', 'picam_calibration.yaml')
    default_extrinsics_path = os.path.join(package_share_dir, 'config', 'extrinsics.yaml')

    return LaunchDescription([
        # --- Declare Launch Arguments ---
        DeclareLaunchArgument('cam_width', default_value='1280', description='Camera frame width'),
        DeclareLaunchArgument('cam_height', default_value='720', description='Camera frame height'),
        DeclareLaunchArgument('cam_framerate', default_value='30.0', description='Camera framerate'),
        DeclareLaunchArgument('yolo_model', default_value=default_yolo_model_path, description='Path to the YOLOv8 model file'),
        DeclareLaunchArgument('confidence_threshold', default_value='0.65', description='Confidence threshold for YOLO detection'),
        DeclareLaunchArgument('ball_radius', default_value='0.105', description='Physical radius of the ball in meters'),
        DeclareLaunchArgument('save_frames', default_value='False', description='Enable/disable saving raw frames'),
        DeclareLaunchArgument('save_path', default_value='/media/pi/USB_DRIVE/ros_images', description='Path to save raw frames'),

        # --- Node Definitions ---

        # 1. Camera Node
        Node(
            package='ball_tracker',
            executable='camera_node',
            name='camera_node',
            parameters=[
                {'camera.width': LaunchConfiguration('cam_width')},
                {'camera.height': LaunchConfiguration('cam_height')},
                {'camera.framerate': LaunchConfiguration('cam_framerate')},
                {'camera_info_url': default_intrinsics_path}
            ],
            output='screen'
        ),

        # 2. Detection Node (with dynamic parameter setting)
        Node(
            package='ball_tracker',
            executable='detection_node',
            name='detection_node',
            parameters=[
                {'yolo_model_path': LaunchConfiguration('yolo_model')},
                {'confidence_threshold': LaunchConfiguration('confidence_threshold')}
            ],
            output='screen'
        ),

        # 3. Calculation Node
        Node(
            package='ball_tracker',
            executable='calculation_node',
            name='calculation_node',
            parameters=[
                {'ball_radius_m': LaunchConfiguration('ball_radius')}
            ],
            output='screen'
        ),

        # 4. Battery Monitor Node
        Node(
            package='ball_tracker',
            executable='battery_monitor',
            name='battery_monitor_node',
            output='screen'
        ),
        
        # 5. Frame Saver Node
        Node(
            package='ball_tracker',
            executable='frame_saver',
            name='frame_saver_node',
            parameters=[
                {'saving_enabled': LaunchConfiguration('save_frames')},
                {'save_path': LaunchConfiguration('save_path')}
            ],
            output='screen'
        ),
        
        # 6. RViz2 Node for Visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            # arguments=['-d', os.path.join(package_share_dir, 'rviz', 'ball_tracker.rviz')] # Optional: load a saved RViz config
            output='log'
        ),
    ])
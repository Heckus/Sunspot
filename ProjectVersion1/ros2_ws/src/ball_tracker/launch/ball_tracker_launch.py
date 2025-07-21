"""
ros2 launch ball_tracker ball_tracker_launch.py

ros2 launch ball_tracker ball_tracker_launch.py save_frames:=True battery_monitor:=True run_intrinsic_cal:=True

ros2 launch ball_tracker ball_tracker_launch.py run_intrinsic_cal:=True checkerboard_size:=9x7 square_size:=0.020
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    # --- Launch Arguments ---
    declare_run_cal_arg = DeclareLaunchArgument(
        'run_intrinsic_cal', default_value='False',
        description='Run the intrinsic camera calibration tool'
    )
    declare_cal_checkerboard_size_arg = DeclareLaunchArgument(
        'checkerboard_size', default_value='8x6',
        description='Checkerboard internal corners'
    )
    declare_cal_square_size_arg = DeclareLaunchArgument(
        'square_size', default_value='0.025',
        description='Size of a checkerboard square in meters'
    )

    # --- Camera Node (using the stable C++ node) ---
    camera_node = Node(
        package='camera_ros',
        executable='camera_node',
        name='camera_node',
        namespace='camera',
        parameters=[{
            'camera_id': 0,
            'image_width': 640,
            'image_height': 480,
            'camera_info_url': 'file:///root/.ros/camera_info/default_camera.yaml'
        }]
    )

    # --- Calibration Process (runs conditionally) ---
    calibrator = ExecuteProcess(
        cmd=[
            FindExecutable(name='ros2'), ' run camera_calibration cameracalibrator',
            ' --size ', LaunchConfiguration('checkerboard_size'),
            ' --square ', LaunchConfiguration('square_size'),
            ' image:=/camera/image_raw',
            ' camera:=/camera'
        ],
        shell=True,
        condition=IfCondition(LaunchConfiguration('run_intrinsic_cal'))
    )

    # --- Main Application Nodes (run unless calibrating) ---
    detection_node = Node(
        package='ball_tracker',
        executable='detection_node',
        name='detection_node',
        remappings=[('/image_raw', '/camera/image_raw')],
        condition=UnlessCondition(LaunchConfiguration('run_intrinsic_cal'))
    )
    calculation_node = Node(
        package='ball_tracker',
        executable='calculation_node',
        name='calculation_node',
        remappings=[('/camera_info', '/camera/camera_info')],
        condition=UnlessCondition(LaunchConfiguration('run_intrinsic_cal'))
    )

    return LaunchDescription([
        declare_run_cal_arg,
        declare_cal_checkerboard_size_arg,
        declare_cal_square_size_arg,
        camera_node,
        calibrator,
        detection_node,
        calculation_node
    ])
"""
ros2 launch ball_tracker ball_tracker_launch.py

ros2 launch ball_tracker ball_tracker_launch.py save_frames:=True battery_monitor:=True run_intrinsic_cal:=True

ros2 launch ball_tracker ball_tracker_launch.py run_intrinsic_cal:=True checkerboard_size:=9x7 square_size:=0.020
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    OpaqueFunction,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    FindExecutable,
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_share_dir = get_package_share_directory("ball_tracker")
    default_rviz_config_path = os.path.join(
        package_share_dir, "rviz", "ball_tracker.rviz"
    )

    # --- Declare Launch Arguments ---
    declare_save_frames_arg = DeclareLaunchArgument(
        "save_frames", default_value="False", description="Enable saving raw frames"
    )
    declare_battery_monitor_arg = DeclareLaunchArgument(
        "battery_monitor",
        default_value="False",
        description="Enable the battery monitor",
    )
    declare_run_intrinsic_cal_arg = DeclareLaunchArgument(
        "run_intrinsic_cal",
        default_value="False",
        description="Run the intrinsic camera calibration tool",
    )
    declare_checkerboard_size_arg = DeclareLaunchArgument(
        "checkerboard_size",
        default_value="8x6",
        description="Checkerboard internal corners (e.g., 8x6)",
    )
    declare_checkerboard_square_size_arg = DeclareLaunchArgument(
        "square_size",
        default_value="0.025",
        description="Size of a checkerboard square in meters",
    )

    # --- Get Launch Configurations ---
    run_intrinsic_cal = LaunchConfiguration("run_intrinsic_cal")
    save_frames = LaunchConfiguration("save_frames")
    battery_monitor = LaunchConfiguration("battery_monitor")

    # --- Define Nodes ---
    camera_node = Node(
        package="ball_tracker",
        executable="camera_node",
        name="camera_node",
        output="screen",
        # Assuming camera_info_manager will handle loading the calibration file
    )

    # Intrinsic calibration node (runs conditionally)
    intrinsic_calibrator = ExecuteProcess(
        cmd=[
            FindExecutable(name="ros2"),
            " run camera_calibration cameracalibrator",
            " --size ",
            LaunchConfiguration("checkerboard_size"),
            " --square ",
            LaunchConfiguration("square_size"),
            " image:=/image_raw",
        ],
        shell=True,
        condition=IfCondition(run_intrinsic_cal),
    )

    # Extrinsic calibration node
    extrinsic_calibrator = ExecuteProcess(
        cmd=[
            FindExecutable(name="ros2"),
            " run ball_tracker extrinsic_calibrator",
        ],
        shell=True,
        # Will be launched by an event handler
    )

    # Core application nodes
    detection_node = Node(
        package="ball_tracker",
        executable="detection_node",
        name="detection_node",
        output="screen",
    )
    calculation_node = Node(
        package="ball_tracker",
        executable="calculation_node",
        name="calculation_node",
        output="screen",
    )
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", default_rviz_config_path],
    )

    # Optional nodes
    frame_saver_node = Node(
        package="ball_tracker",
        executable="frame_saver",
        name="frame_saver_node",
        output="screen",
        condition=IfCondition(save_frames),
    )
    battery_monitor_node = Node(
        package="ball_tracker",
        executable="battery_monitor",
        name="battery_monitor_node",
        output="screen",
        condition=IfCondition(battery_monitor),
    )

    # --- Define Event Handlers for Sequencing ---

    # This handler launches the main application after extrinsic calibration finishes
    main_app_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=extrinsic_calibrator,
            on_exit=[detection_node, calculation_node, rviz2_node],
        )
    )

    # This function decides what to do based on the 'run_intrinsic_cal' argument
    def launch_logic(context):
        if context.launch_configurations["run_intrinsic_cal"] == "True":
            # If running intrinsic calibration, launch extrinsic AFTER intrinsic is done
            handler = RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=intrinsic_calibrator, on_exit=[extrinsic_calibrator]
                )
            )
            return [intrinsic_calibrator, handler]
        else:
            # Otherwise, launch extrinsic calibration right away
            return [extrinsic_calibrator]

    return LaunchDescription(
        [
            declare_run_intrinsic_cal_arg,
            declare_checkerboard_size_arg,
            declare_checkerboard_square_size_arg,
            declare_save_frames_arg,
            declare_battery_monitor_arg,
            camera_node,
            frame_saver_node,
            battery_monitor_node,
            main_app_handler,
            # The OpaqueFunction allows for conditional logic at launch time
            OpaqueFunction(function=launch_logic),
        ]
    )
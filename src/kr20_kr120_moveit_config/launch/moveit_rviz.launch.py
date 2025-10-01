# file: kr20_kr120_moveit_config/launch/viz.launch.py
from launch import LaunchDescription
from moveit_configs_utils import MoveItConfigsBuilder      
from moveit_configs_utils.launches.py import (
    generate_static_virtual_joint_tfs_launch,
    generate_robot_state_publisher_launch,
    generate_move_group_launch,
    generate_moveit_rviz_launch,
)

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("kr20_kr120", package_name="kr20_kr120_moveit_config")
        .to_moveit_configs()
    )

    return LaunchDescription([
        # TFs for virtual joints (if any)
        generate_static_virtual_joint_tfs_launch(moveit_config),

        # Publishes robot_description and /tf from URDF/SRDF
        generate_robot_state_publisher_launch(moveit_config),

        # MoveIt move_group node (planning)
        generate_move_group_launch(moveit_config),

        # Single RViz with the MoveIt MotionPlanning panel
        generate_moveit_rviz_launch(moveit_config),
    ])

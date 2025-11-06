# src/kr20_kr120_gz/launch/cell_gz.launch.py
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_gz        = get_package_share_directory("kr20_kr120_gz")
    pkg_moveit    = get_package_share_directory("kr20_kr120_moveit_config")
    controllers   = os.path.join(pkg_moveit, "config", "ros2_controllers.yaml")

    world   = os.path.join(pkg_gz, "worlds", "cell.world")
    cell_sdf = os.path.join(pkg_gz, "models", "kr20", "kr20.sdf")

    env_ign = SetEnvironmentVariable('IGN_IP', '127.0.0.1')
    env_gz = SetEnvironmentVariable('GZ_IP', '127.0.0.1')

    # 1) Start Gazebo (gz sim)
    gz = ExecuteProcess(
        cmd=['gz', 'sim', world],
        output='screen'
    )

    # 2) Spawn the full cell model
    spawn_cell = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2", "run", "ros_gz_sim", "create",
                    "-file", cell_sdf,
                    "-name", "cell"
                ],
                output="screen"
            )
        ]
    )

    # 3) Bring up controller manager (this matches what Mujoco uses)
    ros2_control = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controllers],
        output="screen"
    )

    # 4) Spawn controllers after controller_manager is up
    spawners = [
        TimerAction(
            period=3.5,
            actions=[Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
                output="screen"
            )]
        ),
        TimerAction(
            period=4.0,
            actions=[Node(
                package="controller_manager",
                executable="spawner",
                arguments=["kr120_arm_controller", "--controller-manager", "/controller_manager"],
                output="screen"
            )]
        ),
        TimerAction(
            period=4.0,
            actions=[Node(
                package="controller_manager",
                executable="spawner",
                arguments=["kr20_arm_controller", "--controller-manager", "/controller_manager"],
                output="screen"
            )]
        ),
        TimerAction(
            period=4.0,
            actions=[Node(
                package="controller_manager",
                executable="spawner",
                arguments=["ee_kr120_controller", "--controller-manager", "/controller_manager"],
                output="screen"
            )]
        ),
        TimerAction(
            period=4.0,
            actions=[Node(
                package="controller_manager",
                executable="spawner",
                arguments=["ee_kr20_controller", "--controller-manager", "/controller_manager"],
                output="screen"
            )]
        ),
    ]

    return LaunchDescription([env_ign, env_gz, gz, spawn_cell, ros2_control, *spawners])

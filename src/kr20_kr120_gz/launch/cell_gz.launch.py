# src/kr20_kr120_gz/launch/cell_gz.launch.py
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable, OpaqueFunction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import shutil


def generate_launch_description():
    pkg_gz        = get_package_share_directory("kr20_kr120_gz")
    pkg_moveit    = get_package_share_directory("kr20_kr120_moveit_config")
    pkg_description = get_package_share_directory("kr20_kr120_description")
    controllers   = os.path.join(pkg_moveit, "config", "ros2_controllers.yaml")

    world   = os.path.join(pkg_gz, "worlds", "cell.world")
    cell_sdf = os.path.join(pkg_gz, "models", "kr20", "kr20.sdf")

    def _extend_env(var_name, extra_paths):
        current = os.environ.get(var_name, "")
        parts = [current] if current else []
        parts.extend(extra_paths)
        # Remove empties and duplicates while preserving order
        seen, cleaned = set(), []
        for p in parts:
            if not p:
                continue
            if p in seen:
                continue
            seen.add(p)
            cleaned.append(p)
        return ":".join(cleaned)

    env_ign = SetEnvironmentVariable('IGN_IP', '127.0.0.1')
    env_gz = SetEnvironmentVariable('GZ_IP', '127.0.0.1')

    resource_paths = [
        os.path.join(pkg_gz, "models"),
        pkg_description,
    ]
    env_resource = SetEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH',
        _extend_env('IGN_GAZEBO_RESOURCE_PATH', resource_paths)
    )
    env_gz_resource = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        _extend_env('GZ_SIM_RESOURCE_PATH', resource_paths)
    )

    workspace_lib = os.path.join(os.path.dirname(os.path.dirname(pkg_gz)), 'lib')
    system_lib = '/opt/ros/humble/lib'
    env_plugins = SetEnvironmentVariable(
        'IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
        _extend_env('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', [workspace_lib, system_lib])
    )

    def _sim_launcher(_context, *args, **kwargs):
        if shutil.which("ign"):
            cmd = ["ign", "gazebo", world]
        elif shutil.which("gz"):
            cmd = ["gz", "sim", world]
        else:
            raise RuntimeError("Neither 'ign' nor 'gz' command found in PATH.")
        return [ExecuteProcess(cmd=cmd, output='screen')]

    gz = OpaqueFunction(function=_sim_launcher)

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

    return LaunchDescription([
        env_ign,
        env_gz,
        env_resource,
        env_gz_resource,
        env_plugins,
        gz,
        spawn_cell,
        ros2_control,
        *spawners,
    ])

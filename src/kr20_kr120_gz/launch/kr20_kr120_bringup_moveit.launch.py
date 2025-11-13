from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
    ExecuteProcess,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, TextSubstitution
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_description = get_package_share_directory("kr20_kr120_description")
    pkg_moveit = get_package_share_directory("kr20_kr120_moveit_config")
    pkg_gz = get_package_share_directory("kr20_kr120_gz")

    world_path = os.path.join(pkg_gz, "worlds", "cell.world")
    xacro_file = os.path.join(pkg_description, "urdf", "kr20_kr120_gazebo.urdf.xacro")
    controllers_file = os.path.join(pkg_moveit, "config", "ros2_controllers.yaml")

    resource_path = os.pathsep.join(
        filter(
            None,
            [
                os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
                pkg_description,
                os.path.join(pkg_gz, "models"),
            ],
        )
    )

    env_vars = [
        SetEnvironmentVariable("IGN_IP", "127.0.0.1"),
        SetEnvironmentVariable("GZ_IP", "127.0.0.1"),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
        SetEnvironmentVariable("IGN_FILE_PATH", resource_path),
        SetEnvironmentVariable("IGN_RESOURCE_PATH", resource_path),
    ]

    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={"gz_args": world_path}.items(),
    )

    robot_description = ParameterValue(
        Command(
            [
                TextSubstitution(
                    text=f'ros2 run xacro xacro "{xacro_file}"'
                )
            ]
        ),
        value_type=str,
    )

    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-entity",
            "kr20_kr120",
            "-topic",
            "robot_description",
        ],
        output="screen",
    )

    controller_names = [
        "joint_state_broadcaster",
        "kr20_arm_controller",
        "kr120_arm_controller",
        "ee_kr20_controller",
        "ee_kr120_controller",
    ]

    controller_spawners = []
    start_time = 6.0
    for name in controller_names:
        controller_spawners.append(
            TimerAction(
                period=start_time,
                actions=[
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        arguments=[name, "--controller-manager", "/controller_manager"],
                        output="screen",
                    )
                ],
            )
        )
        start_time += 0.5

    coordinator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("kr20_kr120_moveit_demos"),
                "launch",
                "coordinator.launch.py",
            )
        )
    )

    return LaunchDescription(
        env_vars
        + [
            gz_launch,
            state_publisher,
            spawn_robot,
            *controller_spawners,
            coordinator_launch,
        ]
    )

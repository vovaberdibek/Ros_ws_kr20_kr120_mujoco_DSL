from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, PathJoinSubstitution


def generate_launch_description():
    desc_share = FindPackageShare('kr20_kr120_description')

    xacro_file     = PathJoinSubstitution([desc_share, 'urdf',   'kr20_kr120.urdf.xacro'])
    scene_file     = PathJoinSubstitution([desc_share, 'mujoco', 'scenes', 'kuka_cell_scene.xml'])
    controllers_y  = PathJoinSubstitution([desc_share, 'config', 'controllers.yaml'])

    # Make robot_description a *string* param
    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', xacro_file]),
            value_type=str
        )
    }

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    mujoco = Node(
        package='mujoco_ros2_control',
        executable='mujoco_ros2_control',   # use this name; only switch to *_node if needed
        output='screen',
        parameters=[
            controllers_y,                # YAML file with your controllers
            robot_description,            # same XML string passed to mujoco node
            {
                'mujoco_model_path': scene_file,
                'use_mujoco_viewer': True,
                'use_sim_time': True
            }
        ]
    )

    # (Optional) start controllers a bit later
    spawn_js = TimerAction(
        period=2.0,
        actions=[Node(package='controller_manager', executable='spawner',
                      arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
                      output='screen')]
    )
    spawn_kr20 = TimerAction(
        period=3.0,
        actions=[Node(package='controller_manager', executable='spawner',
                      arguments=['kr20_arm_controller', '--controller-manager', '/controller_manager'],
                      output='screen')]
    )
    spawn_kr120 = TimerAction(
        period=3.5,
        actions=[Node(package='controller_manager', executable='spawner',
                      arguments=['kr120_arm_controller', '--controller-manager', '/controller_manager'],
                      output='screen')]
    )
    spawn_kr120_gripper = TimerAction(
    period=4.2,
    actions=[Node(
        package='controller_manager', executable='spawner',
        arguments=['kr120_gripper_controller', '--controller-manager', '/controller_manager'],
        output='screen')]
    )

    return LaunchDescription([rsp, mujoco, spawn_js, spawn_kr20, spawn_kr120, spawn_kr120_gripper])

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    pkg_desc = FindPackageShare('kr20_kr120_description')
    xacro_path = PathJoinSubstitution([pkg_desc, 'urdf', 'kr20_kr120_gazebo.urdf.xacro'])
    robot_description = Command([FindExecutable(name='xacro'), ' ', xacro_path])

    # Gazebo (classic) empty world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare('gazebo_ros'), '/launch', '/gazebo.launch.py']
        ),
        launch_arguments={'world': PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'worlds', 'empty.world'])}.items()
    )

    # Publish TF and robot_description param (spawn will read from this topic)
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # Spawn into Gazebo from the topic
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'kr20_kr120', '-topic', 'robot_description'],
        output='screen'
    )

    # Controller spawners (they will wait until /controller_manager appears)
    jsb = Node(package='controller_manager', executable='spawner',
               arguments=['joint_state_broadcaster'], output='screen')
    kr20 = Node(package='controller_manager', executable='spawner',
                arguments=['kr20_arm_controller'], output='screen')
    kr120 = Node(package='controller_manager', executable='spawner',
                 arguments=['kr120_arm_controller'], output='screen')

    # Add more spawners if you want the extra axes/gripper now:
    # slide = Node(package='controller_manager', executable='spawner',
    #             arguments=['kr20_slide_controller'], output='screen')
    # rl_slider = Node(package='controller_manager', executable='spawner',
    #                  arguments=['rl_slider_controller'], output='screen')
    # rl_lifter = Node(package='controller_manager', executable='spawner',
    #                  arguments=['rl_lifter_controller'], output='screen')
    # rl_plate = Node(package='controller_manager', executable='spawner',
    #                 arguments=['rl_plate_controller'], output='screen')
    # grip = Node(package='controller_manager', executable='spawner',
    #             arguments=['kr120_gripper_controller'], output='screen')

    return LaunchDescription([gazebo, rsp, spawn, jsb, kr20, kr120])

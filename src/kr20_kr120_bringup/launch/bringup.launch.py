from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    world = LaunchConfiguration('world')
    robot_sdf = LaunchConfiguration('robot_sdf')
    desc_share = LaunchConfiguration('desc_share')
    render_engine = LaunchConfiguration('render_engine')

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='/tmp/min.world'),
        DeclareLaunchArgument(
            'robot_sdf',
            default_value='/home/user/Desktop/ros2_ws/install/share/kr20_kr120_gz/models/kr20_kr120.sdf'
        ),
        DeclareLaunchArgument(
            'desc_share',
            default_value='/home/user/Desktop/ros2_ws/install/share/kr20_kr120_description'
        ),
        DeclareLaunchArgument('render_engine', default_value='ogre'),  # ogre1 is more forgiving on many GPUs

        # Make sure Gazebo can find your meshes/materials
        SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=desc_share),

        # Start Gazebo GUI with your world
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', '-v', '2', '--render-engine', render_engine, world],
            output='screen'
        ),

        # After a short delay, spawn the robot into the running world
        TimerAction(period=3.0, actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'ros_gz_sim', 'create',
                    '-world', 'default',
                    '-file', robot_sdf,
                    '-name', 'kr_cell_ok',
                    '-z', '0.40'
                ],
                output='screen'
            )
        ]),
    ])

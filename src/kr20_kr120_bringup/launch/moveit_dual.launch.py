from launch import LaunchDescription
from launch_ros.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    kr20_pkg = 'kr20_kr120_moveit_config'  # your existing pkg with move_group/RViz
    moveit_launch = os.path.join(get_package_share_directory(kr20_pkg),
                                 'launch', 'move_group.launch.py')

    arm1 = GroupAction([
        PushRosNamespace('arm1'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(moveit_launch),
            launch_arguments={'use_sim_time': 'true'}.items()
        )
    ])

    arm2 = GroupAction([
        PushRosNamespace('arm2'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(moveit_launch),
            launch_arguments={'use_sim_time': 'true'}.items()
        )
    ])

    return LaunchDescription([arm1, arm2])

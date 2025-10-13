# ~/Desktop/ros2_ws/src/kr20_kr120_gz/launch/cell_gz_headless.launch.py

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    world = os.path.join(
        get_package_share_directory('kr20_kr120_gz'),
        'worlds', 'cell_bullet.world'
    )
    gz_server = ExecuteProcess(
        cmd=['gz', 'sim', '-s', '-r', '-v', '4',
             '--physics-engine', 'bullet-featherstone',
             world],
        output='screen'
    )
    return LaunchDescription([gz_server])

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_gz = get_package_share_directory('kr20_kr120_gz')

    world    = os.path.join(pkg_gz, 'worlds', 'cell.world')
    kr20_sdf = os.path.join(pkg_gz, 'models', 'kr20', 'kr20.sdf')  # this SDF is the WHOLE cell

    gz = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world],
        output='screen'
    )

    # Spawn the single big model (the whole cell)
    spawn_cell = TimerAction(
        period=2.0,  # give GUI a second to start
        actions=[ExecuteProcess(
            cmd=['gz', 'service', '-s', '/world/default/create',
                 '--reqtype', 'gz.msgs.EntityFactory', '--reptype', 'gz.msgs.Boolean',
                 '--timeout', '5000',
                 '--req', f'sdf_filename: "{kr20_sdf}", name: "cell"'],
            output='screen'
        )]
    )

    return LaunchDescription([gz, spawn_cell])

from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os, yaml

def _yaml(pkg, rel):
    with open(os.path.join(get_package_share_directory(pkg), rel), "r") as f:
        return yaml.safe_load(f)

def _text(pkg, rel):
    with open(os.path.join(get_package_share_directory(pkg), rel), "r") as f:
        return f.read()

def generate_launch_description():
    desc_pkg = "kr20_kr120_description"
    cfg_pkg  = "kr20_kr120_moveit_config"

    # ------------------------------
    # Common robot_description (URDF)
    # IMPORTANT: no trailing space after 'xacro'
    # ------------------------------
    xacro_file = PathJoinSubstitution([FindPackageShare(desc_pkg), "urdf", "kr20_kr120.urdf.xacro"])
    robot_description = {
        "robot_description": ParameterValue(Command(["xacro ", xacro_file]), value_type=str)
    }

    # ------------------------------
    # MuJoCo sim + controllers
    # ------------------------------
    scene_file    = PathJoinSubstitution([FindPackageShare(desc_pkg), "mujoco", "scenes", "kuka_cell_scene.xml"])
    controllers_y = PathJoinSubstitution([FindPackageShare(desc_pkg), "config", "controllers.yaml"])

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    mujoco = Node(
        package="mujoco_ros2_control",
        executable="mujoco_ros2_control",
        output="screen",
        parameters=[
            controllers_y,
            robot_description,
            {
                "mujoco_model_path": scene_file,
                "use_mujoco_viewer": True,
                "use_sim_time": True,
            },
        ],
    )

    # spawn ros2_control controllers with small delays
    spawn_js = TimerAction(
        period=2.0,
        actions=[Node(
            package="controller_manager", executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            output="screen")]
    )
    spawn_kr20 = TimerAction(
        period=3.0,
        actions=[Node(
            package="controller_manager", executable="spawner",
            arguments=["kr20_arm_controller", "--controller-manager", "/controller_manager"],
            output="screen")]
    )
    spawn_kr120 = TimerAction(
        period=3.5,
        actions=[Node(
            package="controller_manager", executable="spawner",
            arguments=["kr120_arm_controller", "--controller-manager", "/controller_manager"],
            output="screen")]
    )
    spawn_kr120_gripper = TimerAction(
        period=4.2,
        actions=[Node(
            package="controller_manager", executable="spawner",
            arguments=["kr120_gripper_controller", "--controller-manager", "/controller_manager"],
            output="screen")]
    )
    spawn_gyro = TimerAction(
        period=4.5,
        actions=[Node(
            package="controller_manager", executable="spawner",
            arguments=["kr120_gyro_velocity_controller", "--controller-manager", "/controller_manager"],
            output="screen")]
    )

    # ------------------------------
    # MoveIt (SRDF, kinematics, limits, OMPL, controllers, exec tuning)
    # ------------------------------
    robot_description_semantic   = {"robot_description_semantic": _text(cfg_pkg, "config/kr20_kr120.srdf")}
    robot_description_kinematics = {"robot_description_kinematics": _yaml(cfg_pkg, "config/kinematics.yaml")}
    robot_description_planning   = {"robot_description_planning":   _yaml(cfg_pkg, "config/joint_limits.yaml")}

    ompl_raw  = _yaml(cfg_pkg, "config/ompl_planning.yaml")
    ompl_param = {"ompl": ompl_raw} if ("planner_configs" in ompl_raw and "ompl" not in ompl_raw) else ompl_raw

    moveit_controllers = _yaml(cfg_pkg, "config/moveit_controllers.yaml")
    traj_exec          = _yaml(cfg_pkg, "config/trajectory_execution.yaml")

    move_group = TimerAction(
        period=5.0,  # start after sim/controllers are up
        actions=[Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                robot_description,
                robot_description_semantic,
                robot_description_kinematics,
                robot_description_planning,
                {"planning_pipelines": ["ompl"]},
                ompl_param,
                moveit_controllers,
                {"use_sim_time": True},
                traj_exec,
            ],
        )]
    )

    # RViz with config, after move_group
    rviz_cfg = os.path.join(get_package_share_directory(cfg_pkg), "config", "moveit.rviz")
    rviz = TimerAction(
        period=5.5,
        actions=[Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_cfg],
            output="screen",
            parameters=[
                robot_description,
                robot_description_semantic,
                robot_description_kinematics,
                {"use_sim_time": True},
            ],
        )]
    )

    # (Optional) start your demo node too (uncomment if you want it auto-run)
    # go_to_pose = TimerAction(
    #     period=6.0,
    #     actions=[Node(
    #         package="kr20_kr120_moveit_demos",
    #         executable="go_to_pose",
    #         output="screen",
    #         parameters=[  # pass the same params to the node if you want local model too
    #             robot_description,
    #             robot_description_semantic,
    #             robot_description_kinematics,
    #             robot_description_planning,
    #             {"planning_pipelines": ["ompl"]},
    #             ompl_param,
    #             {"use_sim_time": True},
    #         ],
    #     )]
    # )

    return LaunchDescription([
        rsp,
        mujoco,
        spawn_js,
        spawn_kr20,
        spawn_kr120,
        spawn_kr120_gripper,
        spawn_gyro,
        move_group,
        rviz,
        # go_to_pose,  # optional
    ])


# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, TimerAction
# from launch.substitutions import LaunchConfiguration

# def generate_launch_description():
#     world = LaunchConfiguration('world')
#     robot_sdf = LaunchConfiguration('robot_sdf')
#     desc_share = LaunchConfiguration('desc_share')
#     render_engine = LaunchConfiguration('render_engine')

#     return LaunchDescription([
#         DeclareLaunchArgument('world', default_value='/tmp/min.world'),
#         DeclareLaunchArgument(
#             'robot_sdf',
#             default_value='/home/user/Desktop/ros2_ws/install/share/kr20_kr120_gz/models/kr20_kr120.sdf'
#         ),
#         DeclareLaunchArgument(
#             'desc_share',
#             default_value='/home/user/Desktop/ros2_ws/install/share/kr20_kr120_description'
#         ),
#         DeclareLaunchArgument('render_engine', default_value='ogre'),  # ogre1 is more forgiving on many GPUs

#         # Make sure Gazebo can find your meshes/materials
#         SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=desc_share),

#         # Start Gazebo GUI with your world
#         ExecuteProcess(
#             cmd=['gz', 'sim', '-r', '-v', '2', '--render-engine', render_engine, world],
#             output='screen'
#         ),

#         # After a short delay, spawn the robot into the running world
#         TimerAction(period=3.0, actions=[
#             ExecuteProcess(
#                 cmd=[
#                     'ros2', 'run', 'ros_gz_sim', 'create',
#                     '-world', 'default',
#                     '-file', robot_sdf,
#                     '-name', 'kr_cell_ok',
#                     '-z', '0.40'
#                 ],
#                 output='screen'
#             )
#         ]),
#     ])

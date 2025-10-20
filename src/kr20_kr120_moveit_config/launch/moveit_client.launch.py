from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
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

    # URDF from xacro  (IMPORTANT: no trailing space after 'xacro')
    xacro_file = PathJoinSubstitution([FindPackageShare(desc_pkg), "urdf", "kr20_kr120.urdf.xacro"])
    robot_description = {
        "robot_description": ParameterValue(Command(["xacro ", xacro_file]), value_type=str)
    }

    # SRDF & YAMLs
    robot_description_semantic   = {"robot_description_semantic": _text(cfg_pkg, "config/kr20_kr120.srdf")}
    robot_description_kinematics = {"robot_description_kinematics": _yaml(cfg_pkg, "config/kinematics.yaml")}
    robot_description_planning   = {"robot_description_planning":   _yaml(cfg_pkg, "config/joint_limits.yaml")}

    # OMPL (wrap ROS1-style if needed)
    ompl_raw  = _yaml(cfg_pkg, "config/ompl_planning.yaml")
    ompl_param = {"ompl": ompl_raw} if ("planner_configs" in ompl_raw and "ompl" not in ompl_raw) else ompl_raw

    # ✅ Controllers for execution
    moveit_controllers = _yaml(cfg_pkg, "config/moveit_controllers.yaml")

    rviz_cfg = os.path.join(get_package_share_directory(cfg_pkg), "config", "moveit.rviz")

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # Optional GUI joint publisher
    jsp = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
    )

    traj_exec = _yaml(cfg_pkg, "config/trajectory_execution.yaml")

    move_group = Node(
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
            {"use_sim_time": True},      # <- good practice in sim
            traj_exec,                   # <- add this line
        ],
    )


    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_cfg],
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
        ],
    )

    return LaunchDescription([rsp, jsp, move_group, rviz])

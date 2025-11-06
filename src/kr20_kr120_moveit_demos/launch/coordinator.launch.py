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

    xacro_file = PathJoinSubstitution([FindPackageShare(desc_pkg), "urdf", "kr20_kr120.urdf.xacro"])
    robot_description = {
        "robot_description": ParameterValue(Command(["xacro", " ", xacro_file]), value_type=str)
    }

    robot_description_semantic   = {"robot_description_semantic": _text(cfg_pkg, "config/kr20_kr120.srdf")}
    robot_description_kinematics = {"robot_description_kinematics": _yaml(cfg_pkg, "config/kinematics.yaml")}
    robot_description_planning   = {"robot_description_planning":   _yaml(cfg_pkg, "config/joint_limits.yaml")}
    ompl_raw  = _yaml(cfg_pkg, "config/ompl_planning.yaml")
    ompl      = {"ompl": ompl_raw} if ("planner_configs" in ompl_raw and "ompl" not in ompl_raw) else ompl_raw

    return LaunchDescription([
        Node(
            package="kr20_kr120_moveit_demos",
            executable="init_parameters_server",
            output="screen",
            parameters=[
                robot_description,
                robot_description_semantic,
                robot_description_kinematics,
                robot_description_planning,
                ompl,
                {"planning_pipelines": ["ompl"]},
                {"use_sim_time": True},  # optional
            ],
        )
    ])

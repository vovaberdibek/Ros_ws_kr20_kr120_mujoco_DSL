# ~/Desktop/ros2_ws/src/kr20_kr120_moveit_demos/launch/smoke_plan.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
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

    xacro_path = os.path.join(get_package_share_directory(desc_pkg), "urdf", "kr20_kr120.urdf.xacro")
    srdf_text  = _text(cfg_pkg, "config/kr20_kr120.srdf")
    kin_yaml   = _yaml(cfg_pkg, "config/kinematics.yaml")
    ompl_raw   = _yaml(cfg_pkg, "config/ompl_planning.yaml")
    joint_lim  = _yaml(cfg_pkg, "config/joint_limits.yaml")
    controllers_yaml = _yaml(cfg_pkg, "config/moveit_controllers.yaml")

    # Wrap ROS1-style OMPL files if needed
    ompl = {"ompl": ompl_raw} if ("planner_configs" in ompl_raw and "ompl" not in ompl_raw) else ompl_raw

    robot_description = {
        "robot_description": ParameterValue(Command(["xacro ", xacro_path]), value_type=str)
    }
    robot_description_semantic   = {"robot_description_semantic": srdf_text}
    robot_description_kinematics = {"robot_description_kinematics": kin_yaml}
    robot_description_planning   = {"robot_description_planning": joint_lim}

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # Use JSP ONLY when ros2_control isn’t running (MuJoCo/Gazebo off)


    robot_description = {"robot_description": ParameterValue(Command(["xacro ", xacro_path]), value_type=str)}

    jsp = Node(
        package="joint_state_publisher",           # or "joint_state_publisher_gui"
        executable="joint_state_publisher",
        output="screen",
        parameters=[
            {"rate": 30.0},
            robot_description,                     # <-- REQUIRED so JSP publishes
            {"use_sim_time": False},               # see #2
        ],
    )



    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl,
            controllers_yaml,
            {"moveit_manage_controllers": False},
            {"planning_pipelines": ["ompl"]},
        ],
    )

    demo = Node(
        package="kr20_kr120_moveit_demos",
        executable="smoke_plan",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
        ],
    )

    return LaunchDescription([rsp, jsp, move_group, demo])

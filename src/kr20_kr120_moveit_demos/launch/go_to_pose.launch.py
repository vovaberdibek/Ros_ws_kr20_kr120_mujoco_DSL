# go_to_pose.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os, yaml, xacro

def load_yaml(path):
    with open(path, "r") as f:
        return yaml.safe_load(f)

def read_text(path):
    with open(path, "r") as f:
        return f.read()

def generate_launch_description():
    desc_share   = get_package_share_directory("kr20_kr120_description")
    moveit_share = get_package_share_directory("kr20_kr120_moveit_config")

    urdf_xacro  = os.path.join(desc_share,   "urdf",  "kr20_kr120.urdf.xacro")
    srdf_plain  = os.path.join(moveit_share, "config","kr20_kr120.srdf")
    srdf_xacro  = os.path.join(moveit_share, "config","kr20_kr120.srdf.xacro")
    kin_yaml    = os.path.join(moveit_share, "config","kinematics.yaml")
    limits_yaml = os.path.join(moveit_share, "config","joint_limits.yaml")
    ompl_yaml   = os.path.join(moveit_share, "config","ompl_planning.yaml")

    # Render URDF/SRDF
    urdf_xml = xacro.process_file(urdf_xacro).toxml()
    srdf_xml = xacro.process_file(srdf_xacro).toxml() if os.path.exists(srdf_xacro) else read_text(srdf_plain)

    # NOTE: keep numeric types from YAML (don’t stringify), to avoid the
    # “kinematics_solver_timeout has invalid type” warning.
    params = {
        "robot_description": urdf_xml,
        "robot_description_semantic": srdf_xml,
        "robot_description_kinematics": load_yaml(kin_yaml) if os.path.exists(kin_yaml) else {},
        "robot_description_planning":   load_yaml(limits_yaml) if os.path.exists(limits_yaml) else {},
        # OMPL may need to be nested under 'ompl'
        "ompl": load_yaml(ompl_yaml) if os.path.exists(ompl_yaml) else {},
        "planning_pipelines": ["ompl"],
        "use_sim_time": True,  # match MuJoCo/bringup
    }

    return LaunchDescription([
        Node(
            package="kr20_kr120_moveit_demos",
            executable="go_to_pose",
            name="go_to_pose_node",
            output="screen",
            parameters=[params],  # <-- pass them!
        )
    ])
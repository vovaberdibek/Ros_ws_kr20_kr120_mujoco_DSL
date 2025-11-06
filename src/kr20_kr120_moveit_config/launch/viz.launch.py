from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os, yaml

def _load_yaml(pkg: str, relpath: str):
    path = os.path.join(get_package_share_directory(pkg), relpath)
    with open(path, "r") as f:
        return yaml.safe_load(f)

def _load_text(pkg: str, relpath: str):
    path = os.path.join(get_package_share_directory(pkg), relpath)
    with open(path, "r") as f:
        return f.read()

def generate_launch_description():
    # --- Robot description (xacro -> URDF) ---
    robot_xacro = PathJoinSubstitution([
        FindPackageShare("kr20_kr120_description"),
        "urdf",
        "kr20_kr120.urdf.xacro",   # << correct installed filename
    ])

    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro", " ", robot_xacro]),
            value_type=str,
        )
    }

    # --- SRDF + MoveIt configs ---
    robot_description_semantic = {
        "robot_description_semantic": _load_text(
            "kr20_kr120_moveit_config", "config/kr20_kr120.srdf"
        )
    }
    kinematics_yaml = {
        "robot_description_kinematics": _load_yaml(
            "kr20_kr120_moveit_config", "config/kinematics.yaml"
        )
    }

    # Wrap ROS1-style OMPL file if needed
    ompl_yaml_raw = _load_yaml(
        "kr20_kr120_moveit_config", "config/ompl_planning.yaml"
    )
    ompl_yaml = (
        {"ompl": ompl_yaml_raw}
        if ("planner_configs" in ompl_yaml_raw and "ompl" not in ompl_yaml_raw)
        else ompl_yaml_raw
    )

    joint_limits_yaml = {
        "robot_description_planning": _load_yaml(
            "kr20_kr120_moveit_config", "config/joint_limits.yaml"
        )
    }
    moveit_controllers_yaml = _load_yaml(
        "kr20_kr120_moveit_config", "config/moveit_controllers.yaml"
    )

    ros2_controllers_yaml = PathJoinSubstitution([
        FindPackageShare("kr20_kr120_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    ])

    # --- Nodes ---
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="both",
        parameters=[{"rate": 30.0}],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[robot_description, ros2_controllers_yaml],
    )

    spawner_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    spawner_kr20 = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["kr20_arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    spawner_kr120 = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["kr120_arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    jsp = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
        parameters=[{"rate": 30.0}],
    )


    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_yaml,
            joint_limits_yaml,
            moveit_controllers_yaml,
            {"moveit_manage_controllers": False}, 
            {"planning_pipelines": ["ompl"]},
        ],
    )

    rviz_cfg = PathJoinSubstitution([
        FindPackageShare("kr20_kr120_moveit_config"),
        "config",
        "moveit.rviz",
    ])
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_cfg],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,     
        ],
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        ros2_control_node,
        # small delays so controller_manager is up before spawning controllers
        TimerAction(period=1.5, actions=[spawner_jsb]),
        TimerAction(period=2.0, actions=[spawner_kr20]),
        TimerAction(period=2.5, actions=[spawner_kr120]),
        move_group,
        rviz,
        jsp
    ])


# from launch import LaunchDescription
# from launch_ros.actions import Node
# from moveit_configs_utils import MoveItConfigsBuilder

# def generate_launch_description():
#     # Use the robot description already referenced by your MoveIt config package
#     moveit_config = (
#         MoveItConfigsBuilder("kr20_kr120", package_name="kr20_kr120_moveit_config")
#         .to_moveit_configs()
#     )

#     robot_state_publisher = Node(
#         package="robot_state_publisher",
#         executable="robot_state_publisher",
#         name="robot_state_publisher",
#         output="both",
#         parameters=[moveit_config.robot_description],  # provides /robot_description + TF
#     )

#     joint_state_publisher = Node(
#         package="joint_state_publisher",
#         executable="joint_state_publisher",
#         name="joint_state_publisher",
#         output="both",
#         parameters=[{"rate": 30.0}],  # publishes /joint_states so RViz shows the model immediately
#     )

#     move_group = Node(
#         package="moveit_ros_move_group",
#         executable="move_group",
#         name="move_group",
#         output="screen",
#         parameters=[moveit_config.to_dict()],  # all kinematics/OMPL/semantic params
#     )

#     rviz = Node(
#         package="rviz2",
#         executable="rviz2",
#         name="rviz2",
#         output="screen",
#         arguments=["-d", str(moveit_config.package_path / "config" / "moveit.rviz")],
#         parameters=[
#             moveit_config.robot_description,
#             moveit_config.robot_description_semantic,
#         ],
#     )

#     return LaunchDescription([
#         robot_state_publisher,
#         joint_state_publisher,
#         move_group,
#         rviz,
#     ])

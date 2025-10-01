from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("kr20_kr120", package_name="kr20_kr120_moveit_config")
        .robot_description(
            file_path="/home/user/Desktop/ros2_ws/src/kr20_kr120_description/urdf/kr20_kr120.urdf.xacro",
            mappings={
                # e.g. "use_pinza": "true", "use_avvitatore": "true",
            },
        )
        .to_moveit_configs()
    )

    # Base MoveIt demo: move_group, robot_state_publisher, etc.
    base = generate_demo_launch(moveit_config)

    controllers_yaml = str(moveit_config.package_path / "config" / "ros2_controllers.yaml")
    rviz_cfg = str(moveit_config.package_path / "config" / "moveit.rviz")

    extra = [
        # Joint State Publisher (so /joint_states exists immediately)
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            parameters=[moveit_config.robot_description],
            output="screen",
        ),

        # ros2_control controller manager with your controllers yaml
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            name="controller_manager",
            parameters=[moveit_config.robot_description, controllers_yaml],
            output="screen",
        ),

        # Spawn controllers (names must match config/ros2_controllers.yaml)
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["kr20_arm_controller", "--controller-manager", "/controller_manager"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["kr120_arm_controller", "--controller-manager", "/controller_manager"],
            output="screen",
        ),

        # Explicit RViz with your MoveIt layout (so you don't have to add RobotModel manually)
        # Node(
        #     package="rviz2",
        #     executable="rviz2",
        #     name="rviz2",
        #     arguments=["-d", rviz_cfg],
        #     output="screen",
        # ),
    ]

    return LaunchDescription(base.entities + extra)

# from moveit_configs_utils import MoveItConfigsBuilder
# from moveit_configs_utils.launches import generate_demo_launch


# def generate_launch_description():
#     moveit_config = MoveItConfigsBuilder("kr20_kr120", package_name="kr20_kr120_moveit_config").to_moveit_configs()
#     return generate_demo_launch(moveit_config)

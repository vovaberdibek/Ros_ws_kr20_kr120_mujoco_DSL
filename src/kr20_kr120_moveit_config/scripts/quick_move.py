#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from moveit_commander import MoveGroupCommander, RobotCommander

def main():
    rclpy.init()
    node = Node("quick_move")
    robot = RobotCommander()

    # Choose group: param "group" (default tries kr120_arm, falls back to the first group)
    wanted = node.declare_parameter("group", "kr120_arm").get_parameter_value().string_value
    groups = robot.get_group_names()
    group_name = wanted if wanted in groups else (groups[0] if groups else None)
    if group_name is None:
        node.get_logger().error("No planning groups found in SRDF.")
        return
    if wanted != group_name:
        node.get_logger().warn(f"Group '{wanted}' not found; using '{group_name}'")

    group = MoveGroupCommander(group_name)

    # Small motion on the first few joints
    current = group.get_current_joint_values()
    if not current:
        node.get_logger().error("Couldn't read current joints.")
        return

    # Nudge within limits (small +0.1 rad on up to 6 joints)
    target = current[:]
    for i in range(min(6, len(target))):
        target[i] = target[i] + 0.1

    group.set_max_velocity_scaling_factor(0.2)
    group.set_max_acceleration_scaling_factor(0.2)
    group.set_joint_value_target(target)

    node.get_logger().info(f"Planning & executing on group '{group_name}'...")
    ok = group.go(wait=True)
    group.stop()
    time.sleep(0.2)
    node.get_logger().info(f"Success: {ok}")
    rclpy.shutdown()

if __name__ == "__main__":
    main()

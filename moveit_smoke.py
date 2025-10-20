#!/usr/bin/env python3
import sys, math, time
import rclpy
from rclpy.duration import Duration
from moveit_commander import MoveGroupCommander, RobotCommander

def plan_and_optionally_execute(group: MoveGroupCommander, execute: bool):
    # Make planning deterministic-ish for a smoke test
    group.set_planning_time(5.0)
    group.set_num_planning_attempts(1)
    group.set_goal_joint_tolerance(1e-3)

    # Start from current state
    current = group.get_current_joint_values()

    # Nudge a couple joints by a small delta (stay within limits)
    delta = 0.1  # rad
    for i in range(min(2, len(current))):
        current[i] = current[i] + (delta if i % 2 == 0 else -delta)

    group.set_joint_value_target(current)

    print(f"[INFO] Planning for group '{group.get_name()}' ...")
    result = group.plan()

    # MoveIt2 on Humble may return either (success, plan, time, error_code) or just a RobotTrajectory
    success = False
    trajectory = None
    if isinstance(result, tuple):
        success, trajectory = result[0], result[1]
    else:
        # If it’s just a trajectory, trust it's valid
        trajectory = result
        success = trajectory is not None

    if not success or trajectory is None:
        print("[ERROR] Planning failed.")
        return False

    print("[OK] Planning succeeded.")
    # Try to execute (will only work if controllers are up)
    if execute:
        print("[INFO] Executing trajectory...")
        ok = group.execute(trajectory, wait=True)
        print("[OK] Execute result:", ok)
    return True

def main():
    rclpy.init(args=sys.argv)
    robot = RobotCommander()
    # Try kr20 first; switch this string to "kr120_arm" to test the other robot
    group = MoveGroupCommander("kr20_arm")

    # If you only want to plan (no controllers), set execute=False
    planned = plan_and_optionally_execute(group, execute=True)

    # Also try the other arm in the same run if you want:
    try:
        group2 = MoveGroupCommander("kr120_arm")
        plan_and_optionally_execute(group2, execute=True)
    except Exception as e:
        print("[WARN] Could not create group 'kr120_arm':", e)

    rclpy.shutdown()

if __name__ == "__main__":
    main()

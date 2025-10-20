// src/go_to_pose.cpp
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include <thread>
#include <vector>
#include <chrono>

namespace {
void waitForState(moveit::planning_interface::MoveGroupInterface& mgi, int ms=300) {
  rclcpp::sleep_for(std::chrono::milliseconds(ms));
  mgi.setStartStateToCurrentState();
}

bool execPlan(moveit::planning_interface::MoveGroupInterface& mgi,
              moveit::planning_interface::MoveGroupInterface::Plan& plan,
              const rclcpp::Logger& log, const char* tag) {
  auto ok_plan = (mgi.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (!ok_plan) { RCLCPP_ERROR(log, "[%s] Planning failed.", tag); return false; }
  auto ok_exec = (mgi.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (!ok_exec) { RCLCPP_ERROR(log, "[%s] Execution failed.", tag); return false; }
  RCLCPP_INFO(log, "[%s] Done.", tag);
  return true;
}
} // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("go_to_pose_node");

  // Spin so MoveIt can get TF/params/clock
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  std::thread spinner([&exec]() { exec.spin(); });

  // ---------------- KR120 ----------------
  static const std::string KR120_GROUP = "kr120_arm";
  static const std::string KR120_BASE  = "kr120_base_link";
  static const std::string KR120_EEF   = "kr120_flange"; // <-- important

  moveit::planning_interface::MoveGroupInterface kr120(node, KR120_GROUP);
  kr120.setPoseReferenceFrame(KR120_BASE);
  kr120.setEndEffectorLink(KR120_EEF);
  kr120.setPlanningPipelineId("ompl");
  kr120.setPlannerId("RRTConnectkConfigDefault");
  kr120.setPlanningTime(15.0);
  kr120.setMaxVelocityScalingFactor(0.25);
  kr120.setMaxAccelerationScalingFactor(0.25);
  kr120.setGoalPositionTolerance(0.01);
  kr120.setGoalOrientationTolerance(0.10);
  waitForState(kr120);

  // --- Move 1: small Cartesian nudge (+0.20m in X) ---
  geometry_msgs::msg::Pose start_pose = kr120.getCurrentPose(KR120_EEF).pose;
  geometry_msgs::msg::Pose cart_target = start_pose;
  cart_target.position.x += 0.20;

  {
    std::vector<geometry_msgs::msg::Pose> waypoints{cart_target};
    moveit_msgs::msg::RobotTrajectory traj;
    double fraction = kr120.computeCartesianPath(waypoints, /*eef_step=*/0.01, /*jump_threshold=*/0.0, traj);
    if (fraction >= 0.99) {
      moveit::planning_interface::MoveGroupInterface::Plan plan_cart;
      plan_cart.trajectory_ = traj;
      if (kr120.execute(plan_cart) != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "[KR120 Cartesian #1] Execution failed.");
        rclcpp::shutdown(); spinner.join(); return 1;
      }
      RCLCPP_INFO(node->get_logger(), "[KR120 Cartesian #1] Done.");
    } else {
      RCLCPP_WARN(node->get_logger(), "Cartesian fraction=%.2f; falling back to plan/execute.", fraction);
      kr120.setPoseTarget(cart_target, KR120_EEF);
      moveit::planning_interface::MoveGroupInterface::Plan plan_fallback;
      if (!execPlan(kr120, plan_fallback, node->get_logger(), "KR120 Fallback #1")) {
        rclcpp::shutdown(); spinner.join(); return 1;
      }
    }
  }
  kr120.stop();
  kr120.clearPoseTargets();
  waitForState(kr120, 400);

  // --- Move 2: robust absolute step (3-tier fallback) ---
  // Goal: a *small* move from the new current pose. Keep orientation to avoid singular IK.
  geometry_msgs::msg::PoseStamped goal_stamped;
  goal_stamped.header.frame_id = KR120_BASE;
  goal_stamped.pose = kr120.getCurrentPose(KR120_EEF).pose;
  goal_stamped.pose.position.x += 0.15;   // modest step
  goal_stamped.pose.position.y += 0.05;

  // (A) Try Cartesian with 2 waypoints (smoother & more likely to be valid)
  {
    std::vector<geometry_msgs::msg::Pose> wps;
    auto mid = goal_stamped.pose;
    mid.position.y = (goal_stamped.pose.position.y + start_pose.position.y) * 0.5; // tiny arc in Y
    wps.push_back(mid);
    wps.push_back(goal_stamped.pose);

    moveit_msgs::msg::RobotTrajectory traj2;
    double fraction2 = kr120.computeCartesianPath(wps, 0.01, 0.0, traj2);
    if (fraction2 >= 0.99) {
      moveit::planning_interface::MoveGroupInterface::Plan plan_cart2;
      plan_cart2.trajectory_ = traj2;
      if (kr120.execute(plan_cart2) != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "[KR120 Cartesian #2] Execution failed.");
        rclcpp::shutdown(); spinner.join(); return 1;
      }
      RCLCPP_INFO(node->get_logger(), "[KR120 Cartesian #2] Done.");
    } else {
      // (B) setPoseTarget (exact pose)
      RCLCPP_WARN(node->get_logger(), "Cartesian #2 fraction=%.2f; trying setPoseTarget.", fraction2);
      kr120.setPoseTarget(goal_stamped, KR120_EEF);
      moveit::planning_interface::MoveGroupInterface::Plan plan_abs;
      if (!execPlan(kr120, plan_abs, node->get_logger(), "KR120 Absolute")) {
        // (C) approximate IK seed (more forgiving)
        RCLCPP_WARN(node->get_logger(), "Absolute failed; trying setApproximateJointValueTarget.");
        kr120.clearPoseTargets();
        waitForState(kr120, 200);
        if (!kr120.setApproximateJointValueTarget(goal_stamped.pose, KR120_EEF)) {
          RCLCPP_ERROR(node->get_logger(), "setApproximateJointValueTarget() failed to find seed.");
          rclcpp::shutdown(); spinner.join(); return 1;
        }
        moveit::planning_interface::MoveGroupInterface::Plan plan_approx;
        if (!execPlan(kr120, plan_approx, node->get_logger(), "KR120 ApproxIK")) {
          rclcpp::shutdown(); spinner.join(); return 1;
        }
      }
    }
  }
  kr120.stop();
  kr120.clearPoseTargets();
  waitForState(kr120, 400);

  // ---------------- KR20 (extra command) ----------------
  static const std::string KR20_GROUP = "kr20_arm";
  static const std::string KR20_BASE  = "kr20_base_link";

  moveit::planning_interface::MoveGroupInterface kr20(node, KR20_GROUP);
  kr20.setPoseReferenceFrame(KR20_BASE);
  kr20.setPlanningPipelineId("ompl");
  kr20.setPlannerId("RRTConnectkConfigDefault");
  kr20.setPlanningTime(10.0);
  kr20.setMaxVelocityScalingFactor(0.25);
  kr20.setMaxAccelerationScalingFactor(0.25);
  kr20.setGoalPositionTolerance(0.01);
  kr20.setGoalOrientationTolerance(0.05);
  waitForState(kr20);

  // Joint-space nudge on joint 1 (safe & reliable)
  {
    std::vector<double> jvals = kr20.getCurrentJointValues();
    if (!jvals.empty()) {
      jvals[0] += 0.3;  // ~17 deg, keep within limits
      kr20.setJointValueTarget(jvals);
      moveit::planning_interface::MoveGroupInterface::Plan plan_j;
      if (!execPlan(kr20, plan_j, node->get_logger(), "KR20 Joint Nudge")) {
        // fallback small Cartesian
        RCLCPP_WARN(node->get_logger(), "KR20 joint nudge failed; trying Cartesian.");
        geometry_msgs::msg::Pose p0 = kr20.getCurrentPose().pose;
        geometry_msgs::msg::Pose p1 = p0; p1.position.x += 0.10;
        std::vector<geometry_msgs::msg::Pose> wps{p1};
        moveit_msgs::msg::RobotTrajectory traj20;
        double frac20 = kr20.computeCartesianPath(wps, 0.01, 0.0, traj20);
        if (frac20 >= 0.99) {
          moveit::planning_interface::MoveGroupInterface::Plan p20; p20.trajectory_ = traj20;
          if (kr20.execute(p20) != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(node->get_logger(), "KR20 Cartesian execution failed.");
            rclcpp::shutdown(); spinner.join(); return 1;
          }
        } else {
          RCLCPP_ERROR(node->get_logger(), "KR20 Cartesian fraction=%.2f; giving up.", frac20);
          rclcpp::shutdown(); spinner.join(); return 1;
        }
      }
    } else {
      RCLCPP_WARN(node->get_logger(), "KR20 joint values empty?");
    }
  }

  RCLCPP_INFO(node->get_logger(), "All motions complete.");
  rclcpp::shutdown();
  spinner.join();
  return 0;
}

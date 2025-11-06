#include "kr120_coordinator/motion_primitives.hpp"

#include <algorithm>
#include <stdexcept>

#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace kr120::motion {

geometry_msgs::msg::Pose poseFromArray(const std::array<double, 6> &xyz_rpy) {
  tf2::Quaternion q;
  q.setRPY(xyz_rpy[3], xyz_rpy[4], xyz_rpy[5]);

  geometry_msgs::msg::Pose pose;
  pose.position.x = xyz_rpy[0];
  pose.position.y = xyz_rpy[1];
  pose.position.z = xyz_rpy[2];
  pose.orientation = tf2::toMsg(q);
  return pose;
}

geometry_msgs::msg::Pose computeScrewPose(const geometry_msgs::msg::Pose &flange_pose,
                                          double approach_offset) {
  geometry_msgs::msg::Pose screw_pose = flange_pose;

  tf2::Quaternion q;
  tf2::fromMsg(flange_pose.orientation, q);
  tf2::Matrix3x3 R(q);

  screw_pose.position.x += R[0][2] * approach_offset;
  screw_pose.position.y += R[1][2] * approach_offset;
  screw_pose.position.z += R[2][2] * approach_offset;
  return screw_pose;
}

bool executeRRT(moveit::planning_interface::MoveGroupInterface &group,
                const geometry_msgs::msg::Pose &target_pose,
                int max_attempts) {
  moveit::planning_interface::MoveGroupInterface::Plan plan;

  for (int attempt = 0; attempt < max_attempts; ++attempt) {
    group.setStartStateToCurrentState();
    group.setPoseTarget(target_pose);

    auto plan_result = group.plan(plan);
    if (plan_result == moveit::core::MoveItErrorCode::SUCCESS) {
      auto exec_result = group.execute(plan);
      if (exec_result == moveit::core::MoveItErrorCode::SUCCESS) {
        return true;
      }
    }
  }

  return false;
}

bool executeLinearPath(moveit::planning_interface::MoveGroupInterface &group,
                       const std::vector<geometry_msgs::msg::Pose> &waypoints,
                       double eef_step,
                       double jump_threshold) {
  if (waypoints.empty()) {
    return false;
  }

  moveit_msgs::msg::RobotTrajectory trajectory;
  group.setStartStateToCurrentState();

  double fraction = group.computeCartesianPath(
      waypoints, eef_step, jump_threshold, trajectory, true);

  if (fraction <= 0.0) {
    return false;
  }

  auto exec_result = group.execute(trajectory);
  return exec_result == moveit::core::MoveItErrorCode::SUCCESS;
}

bool setJointPositions(moveit::planning_interface::MoveGroupInterface &group,
                       const std::vector<std::string> &joint_names,
                       const std::vector<double> &values) {
  if (joint_names.size() != values.size() || joint_names.empty()) {
    throw std::invalid_argument("Joint names and values must have the same non-zero length.");
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  group.setStartStateToCurrentState();

  for (std::size_t i = 0; i < joint_names.size(); ++i) {
    group.setJointValueTarget(joint_names[i], values[i]);
  }

  auto plan_result = group.plan(plan);
  if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
    return false;
  }

  auto exec_result = group.execute(plan);
  return exec_result == moveit::core::MoveItErrorCode::SUCCESS;
}

bool executeSlideStroke(moveit::planning_interface::MoveGroupInterface &group,
                        double insertion_target,
                        double retract_target) {
  const auto joint_names = group.getJointNames();
  if (joint_names.size() != 1) {
    throw std::runtime_error("Slide stroke expects a single-joint MoveGroupInterface.");
  }

  std::vector<double> target{insertion_target};

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  group.setStartStateToCurrentState();
  group.setJointValueTarget(target);
  auto plan_result = group.plan(plan);
  if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
    return false;
  }

  if (group.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    return false;
  }

  group.setStartStateToCurrentState();
  target[0] = retract_target;
  group.setJointValueTarget(target);
  plan_result = group.plan(plan);
  if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
    return false;
  }

  auto exec_result = group.execute(plan);
  return exec_result == moveit::core::MoveItErrorCode::SUCCESS;
}

}  // namespace kr120::motion

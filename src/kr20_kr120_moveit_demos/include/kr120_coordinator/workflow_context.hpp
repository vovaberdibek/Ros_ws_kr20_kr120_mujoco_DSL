#pragma once

#include <array>
#include <vector>
#include <string>

#include <geometry_msgs/msg/pose.hpp>

#include <custom_interfaces/srv/init_parameters.hpp>

#include "kr120_coordinator/motion_primitives.hpp"

namespace kr120::coord {

struct WorkflowContext {
  geometry_msgs::msg::Pose table_pose;
  geometry_msgs::msg::Pose tray_init_pose;
  geometry_msgs::msg::Pose recharge_pose;
  geometry_msgs::msg::Pose rest_pose;
  geometry_msgs::msg::Pose pick_pose;
  geometry_msgs::msg::Pose pre_pick_pose;
  geometry_msgs::msg::Pose post_pick_pose;
  geometry_msgs::msg::Pose bottom_pick_pose;
  geometry_msgs::msg::Pose end_pose;
  geometry_msgs::msg::Pose pre_end_pose;
  geometry_msgs::msg::Pose tray_final_pose;
  geometry_msgs::msg::Pose tray_operator_pose;

  std::vector<std::array<double, 6>> tray_step_poses;
  std::array<double, 3> tip_offset{};
  double tray_angle1{0.0};
  double tray_angle2{0.0};
  std::vector<int> tray_down_steps;
  std::vector<int> operator_steps;
  std::vector<int> rotation_steps;
  std::vector<int> new_tray_steps;
  std::vector<std::string> tray_unit_names;
  std::vector<int> tray_unit_pose_indices;
};

WorkflowContext makeWorkflowContext(
    const custom_interfaces::srv::InitParameters::Request &req);

}  // namespace kr120::coord

#pragma once

#include <array>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

namespace kr120::motion {

/// Convert an array laid out as [x, y, z, roll, pitch, yaw] to a Pose message.
geometry_msgs::msg::Pose poseFromArray(const std::array<double, 6> &xyz_rpy);

geometry_msgs::msg::Pose computeScrewPose(const geometry_msgs::msg::Pose &flange_pose,
                                          double approach_offset);

bool executeRRT(moveit::planning_interface::MoveGroupInterface &group,
                const geometry_msgs::msg::Pose &target_pose,
                int max_attempts = 10);

bool executeLinearPath(moveit::planning_interface::MoveGroupInterface &group,
                       const std::vector<geometry_msgs::msg::Pose> &waypoints,
                       double eef_step = 0.01,
                       double jump_threshold = 0.0);

bool setJointPositions(moveit::planning_interface::MoveGroupInterface &group,
                       const std::vector<std::string> &joint_names,
                       const std::vector<double> &values);

bool executeSlideStroke(moveit::planning_interface::MoveGroupInterface &group,
                        double insertion_target,
                        double retract_target = 0.0);

}  // namespace kr120::motion

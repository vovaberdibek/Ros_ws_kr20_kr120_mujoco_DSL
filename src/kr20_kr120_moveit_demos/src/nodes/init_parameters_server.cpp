#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <custom_interfaces/srv/init_parameters.hpp>
#include <custom_interfaces/srv/pick_tray.hpp>
#include <custom_interfaces/srv/place_tray.hpp>
#include <custom_interfaces/srv/internal_screwing_sequence.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "kr120_coordinator/motion_primitives.hpp"
#include "kr120_coordinator/workflow_context.hpp"

namespace {
constexpr std::array<double, 2> kGripperOpen{-0.035, 0.035};
constexpr std::array<double, 2> kGripperClosed{0.0, 0.0};
constexpr double kSlideRetract = 0.0;
constexpr double kApproachDistance = 0.35;
constexpr double kPi = 3.14159265358979323846;
constexpr double kMillimeterToMeter = 1e-3;

bool contains(const std::vector<int> &vec, std::size_t value) {
  return std::find(vec.begin(), vec.end(), static_cast<int>(value)) != vec.end();
}

geometry_msgs::msg::Quaternion rpyToQuaternion(double roll,
                                               double pitch,
                                               double yaw) {
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  return tf2::toMsg(q);
}

std::array<std::array<double, 3>, 3> quaternionToMatrix(
    const geometry_msgs::msg::Quaternion &q_msg) {
  tf2::Quaternion q;
  tf2::fromMsg(q_msg, q);
  tf2::Matrix3x3 tf2_matrix(q);
  std::array<std::array<double, 3>, 3> matrix{};
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      matrix[r][c] = tf2_matrix[r][c];
    }
  }
  return matrix;
}

geometry_msgs::msg::Quaternion matrixToQuaternion(
    const std::array<std::array<double, 3>, 3> &matrix) {
  tf2::Matrix3x3 tf2_matrix(
      matrix[0][0], matrix[0][1], matrix[0][2],
      matrix[1][0], matrix[1][1], matrix[1][2],
      matrix[2][0], matrix[2][1], matrix[2][2]);
  tf2::Quaternion q;
  tf2_matrix.getRotation(q);
  return tf2::toMsg(q);
}

geometry_msgs::msg::Pose computeFlangePose(
    const geometry_msgs::msg::Pose &hole_in_tray,
    const std::array<double, 6> &tray_pose,
    const std::array<double, 3> &tip_offset) {
  const geometry_msgs::msg::Quaternion tray_quat =
      rpyToQuaternion(tray_pose[0], tray_pose[1], tray_pose[2]);
  const auto tray_matrix = quaternionToMatrix(tray_quat);
  const auto hole_matrix = quaternionToMatrix(hole_in_tray.orientation);
  const auto tool_matrix =
      quaternionToMatrix(rpyToQuaternion(kPi, kPi / 2.0, 0.0));

  std::array<std::array<double, 3>, 3> target_world{};
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      for (int k = 0; k < 3; ++k) {
        target_world[r][c] += tray_matrix[r][k] * hole_matrix[k][c];
      }
    }
  }

  std::array<std::array<double, 3>, 3> flange_matrix{};
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      for (int k = 0; k < 3; ++k) {
        flange_matrix[r][c] += target_world[r][k] * tool_matrix[k][c];
      }
    }
  }

  std::array<double, 3> tray_translation{tray_pose[3], tray_pose[4],
                                         tray_pose[5]};
  std::array<double, 3> hole_translation{
      hole_in_tray.position.x,
      hole_in_tray.position.y,
      hole_in_tray.position.z};

  std::array<double, 3> flange_translation = tray_translation;
  for (int r = 0; r < 3; ++r) {
    for (int k = 0; k < 3; ++k) {
      flange_translation[r] += tray_matrix[r][k] * hole_translation[k];
      flange_translation[r] += flange_matrix[r][k] * tip_offset[k];
    }
  }

  geometry_msgs::msg::Pose flange_pose;
  flange_pose.position.x = flange_translation[0];
  flange_pose.position.y = flange_translation[1];
  flange_pose.position.z = flange_translation[2];
  flange_pose.orientation = matrixToQuaternion(flange_matrix);
  return flange_pose;
}

double screwApproachOffset(int screw_index) {
  if (screw_index == -1) {
    return 0.018;
  }
  if ((screw_index >= 0 && screw_index <= 87) || screw_index >= 96) {
    return 0.018;
  }
  return 0.042;
}

double slideInsertionTarget(int screw_index) {
  return -screwApproachOffset(screw_index);
}

geometry_msgs::msg::Pose makeApproachPose(const geometry_msgs::msg::Pose &screw_pose,
                                          double tray_angle) {
  geometry_msgs::msg::Pose approach = screw_pose;
  approach.position.x -= kApproachDistance * std::cos(tray_angle);
  approach.position.y += kApproachDistance * std::sin(tray_angle);
  return approach;
}

std::vector<geometry_msgs::msg::Pose> loadHolePoses(const std::string &path) {
  std::ifstream file(path);
  if (!file.is_open()) {
    throw std::runtime_error("Cannot open screw pose file: " + path);
  }

  std::vector<geometry_msgs::msg::Pose> poses;
  std::string line;
  while (std::getline(file, line)) {
    if (line.empty() || line.find(',') == std::string::npos) {
      continue;
    }
    std::stringstream ss(line);
    std::string token;
    std::vector<double> values;
    while (std::getline(ss, token, ',')) {
      try {
        values.push_back(std::stod(token));
      } catch (const std::exception &) {
        values.clear();
        break;
      }
    }
    if (values.size() != 6) {
      continue;
    }

    geometry_msgs::msg::Pose pose;
    pose.orientation = rpyToQuaternion(values[0], values[1], values[2]);
    pose.position.x = values[3] * kMillimeterToMeter;
    pose.position.y = values[4] * kMillimeterToMeter;
    pose.position.z = values[5] * kMillimeterToMeter;
    poses.push_back(pose);
  }

  return poses;
}

std::vector<std::vector<int>> loadScrewSequences(const std::string &path) {
  std::ifstream file(path);
  if (!file.is_open()) {
    throw std::runtime_error("Cannot open screw order file: " + path);
  }

  std::vector<std::vector<int>> sequences;
  std::string line;
  while (std::getline(file, line)) {
    if (line.empty() || line == "MAN" || line == "AUTO") {
      continue;
    }
    std::stringstream ss(line);
    std::string token;
    std::vector<int> values;
    while (std::getline(ss, token, ',')) {
      try {
        values.push_back(std::stoi(token));
      } catch (const std::exception &) {
        values.clear();
        break;
      }
    }
    if (!values.empty()) {
      sequences.push_back(values);
    }
  }

  return sequences;
}

}  // namespace

class CoordinatorNode : public rclcpp::Node {
  using InitParameters = custom_interfaces::srv::InitParameters;
  using PickTray = custom_interfaces::srv::PickTray;
  using PlaceTray = custom_interfaces::srv::PlaceTray;
  using InternalScrewingSequence = custom_interfaces::srv::InternalScrewingSequence;

public:
  CoordinatorNode()
      : rclcpp::Node("kr120_coordinator",
                     rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) {
    using std::placeholders::_1;
    using std::placeholders::_2;

    try {
      const auto share =
          ament_index_cpp::get_package_share_directory("kr20_kr120_moveit_demos");
      const auto pose_path = share + "/data/pose_target.txt";
      const auto order_path = share + "/data/ordine_viti.txt";
      screw_targets_ = loadHolePoses(pose_path);
      screw_sequences_ = loadScrewSequences(order_path);
      RCLCPP_INFO(
          get_logger(),
          "Loaded screw resources: %zu poses, %zu sequences.",
          screw_targets_.size(),
          screw_sequences_.size());
    } catch (const std::exception &ex) {
      RCLCPP_FATAL(get_logger(), "Failed to initialise screw data: %s", ex.what());
      throw;
    }

    init_srv_ = create_service<InitParameters>(
        "init_parameters",
        std::bind(&CoordinatorNode::handleInitParameters, this, _1, _2));
    pick_srv_ = create_service<PickTray>(
        "pick_tray",
        std::bind(&CoordinatorNode::handlePickTray, this, _1, _2));
    place_srv_ = create_service<PlaceTray>(
        "place_tray",
        std::bind(&CoordinatorNode::handlePlaceTray, this, _1, _2));
    screw_srv_ = create_service<InternalScrewingSequence>(
        "internal_screwing_sequence",
        std::bind(&CoordinatorNode::handleInternalScrewing, this, _1, _2));

    gripper_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/kr120_gripper_controller/joint_trajectory", rclcpp::QoS(10));

    init_timer_ = create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&CoordinatorNode::initializeMoveGroups, this));
  }

private:
  void initializeMoveGroups() {
    if (move_groups_ready_) {
      return;
    }

    try {
      auto self = shared_from_this();

      kr120_arm_group_ =
          std::make_shared<moveit::planning_interface::MoveGroupInterface>(
              shared_from_this(), "kr120_arm");
      kr20_arm_group_ =
          std::make_shared<moveit::planning_interface::MoveGroupInterface>(
              shared_from_this(), "kr20_arm");
      kr20_slide_group_ =
          std::make_shared<moveit::planning_interface::MoveGroupInterface>(
              shared_from_this(), "kr20_slide");

      move_groups_ready_ = true;
      init_timer_->cancel();
      RCLCPP_INFO(get_logger(), "MoveGroup interfaces ready.");
    } catch (const std::exception &ex) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Waiting for MoveIt interfaces: %s", ex.what());
    }
  }

  template <typename ResponseT>
  bool ensureReady(const std::string &service_name,
                   const std::shared_ptr<ResponseT> &response) {
    if (!move_groups_ready_) {
      RCLCPP_WARN(get_logger(),
                  "[%s] Move groups not ready yet.", service_name.c_str());
      response->success = false;
      return false;
    }
    if (!context_) {
      RCLCPP_WARN(get_logger(),
                  "[%s] InitParameters not received yet.", service_name.c_str());
      response->success = false;
      return false;
    }
    return true;
  }

  void handleInitParameters(
      const std::shared_ptr<InitParameters::Request> request,
      std::shared_ptr<InitParameters::Response> response) {
    try {
      context_ = kr120::coord::makeWorkflowContext(*request);
      if (context_->tray_step_poses.size() > screw_sequences_.size()) {
        RCLCPP_WARN(
            get_logger(),
            "Received %zu tray step poses but only %zu screw sequences are available.",
            context_->tray_step_poses.size(),
            screw_sequences_.size());
      }
      RCLCPP_INFO(
          get_logger(),
          "InitParameters stored. Tray poses: %zu entries, tip offset [%.3f, %.3f, %.3f]",
          context_->tray_step_poses.size(),
          context_->tip_offset[0],
          context_->tip_offset[1],
          context_->tip_offset[2]);
      response->success = true;
    } catch (const std::exception &ex) {
      RCLCPP_ERROR(get_logger(), "Failed to process InitParameters: %s", ex.what());
      response->success = false;
    }
  }

  void handlePickTray(
      const std::shared_ptr<PickTray::Request> request,
      std::shared_ptr<PickTray::Response> response) {
    (void)request;
    if (!ensureReady("pick_tray", response)) {
      return;
    }

    bool ok = performPickSequence();
    response->success = ok;
    if (!ok) {
      RCLCPP_WARN(get_logger(), "Pick sequence failed.");
    }
  }

  void handlePlaceTray(
      const std::shared_ptr<PlaceTray::Request> request,
      std::shared_ptr<PlaceTray::Response> response) {
    (void)request;
    if (!ensureReady("place_tray", response)) {
      return;
    }

    bool ok = performPlaceSequence();
    response->success = ok;
    if (!ok) {
      RCLCPP_WARN(get_logger(), "Place sequence failed.");
    }
  }

  void handleInternalScrewing(
      const std::shared_ptr<InternalScrewingSequence::Request> request,
      std::shared_ptr<InternalScrewingSequence::Response> response) {
    if (!ensureReady("internal_screwing_sequence", response)) {
      return;
    }

    bool ok = performScrewSequence(static_cast<std::size_t>(request->index));
    response->success = ok;
    if (!ok) {
      RCLCPP_WARN(get_logger(), "Internal screwing sequence failed for index %d.",
                  request->index);
    }
  }

  bool performPickSequence() {
    const auto &ctx = *context_;

    bool ok = true;

    // 1. Move to pre-pick via free-space plan.
    ok &= kr120::motion::executeRRT(*kr120_arm_group_, ctx.pre_pick_pose);
    waitForStateUpdate();

    // 2. Ensure the gripper is open before approaching.
    commandGripper(kGripperOpen, 0.25);
    rclcpp::sleep_for(std::chrono::milliseconds(250));

    // 3. Approach the tray with a straight cartesian move.
    ok &= kr120::motion::executeLinearPath(
        *kr120_arm_group_, {ctx.pick_pose}, 0.01, 0.0);
    waitForStateUpdate();

    // 4. Close the gripper and wait briefly for the controller.
    commandGripper(kGripperClosed, 0.25);
    rclcpp::sleep_for(std::chrono::milliseconds(250));

    // 5. Retreat back to the same pre-pick pose using a free-space plan to avoid collisions.
    ok &= kr120::motion::executeRRT(*kr120_arm_group_, ctx.pre_pick_pose);
    waitForStateUpdate();

    return ok;
  }

  bool performPlaceSequence() {
    const auto &ctx = *context_;

    bool ok = true;
    ok &= kr120::motion::executeRRT(*kr120_arm_group_, ctx.pre_end_pose);
    waitForStateUpdate();
    ok &= kr120::motion::executeLinearPath(*kr120_arm_group_,
                                           {ctx.end_pose}, 0.01, 0.0);
    commandGripper(kGripperOpen, 0.35);
    waitForStateUpdate();
    ok &= kr120::motion::executeLinearPath(*kr120_arm_group_,
                                           {ctx.pre_end_pose}, 0.01, 0.0);
    waitForStateUpdate();
    return ok;
  }

  bool performScrewSequence(std::size_t index) {
    if (index >= context_->tray_step_poses.size()) {
      RCLCPP_ERROR(get_logger(), "Requested tray step index %zu out of range.",
                   index);
      return false;
    }

    if (index >= screw_sequences_.size()) {
      RCLCPP_ERROR(
          get_logger(),
          "No screw sequence defined for step %zu (only %zu sequences loaded).",
          index,
          screw_sequences_.size());
      return false;
    }

    const auto &tray_pose = context_->tray_step_poses.at(index);
    const auto &sequence = screw_sequences_.at(index);

    if (sequence.empty()) {
      RCLCPP_WARN(get_logger(), "Screw sequence for step %zu is empty.", index);
      return true;
    }

    const double tray_angle =
        contains(context_->rotation_steps, index) ? context_->tray_angle2
                                                  : context_->tray_angle1;

    for (int screw_index : sequence) {
      if (screw_index < 0 ||
          static_cast<std::size_t>(screw_index) >= screw_targets_.size()) {
        RCLCPP_WARN(get_logger(),
                    "Skipping screw index %d (outside loaded pose range %zu).",
                    screw_index,
                    screw_targets_.size());
        continue;
      }

      const geometry_msgs::msg::Pose flange_pose = computeFlangePose(
          screw_targets_.at(screw_index), tray_pose, context_->tip_offset);

      const double approach_offset = screwApproachOffset(screw_index);
      const geometry_msgs::msg::Pose screw_pose =
          kr120::motion::computeScrewPose(flange_pose, approach_offset);
      const geometry_msgs::msg::Pose approach_pose =
          makeApproachPose(screw_pose, tray_angle);

      if (!kr120::motion::executeLinearPath(
              *kr20_arm_group_, {approach_pose}, 0.01, 0.0)) {
        return false;
      }
      waitForStateUpdate();
      if (!kr120::motion::executeRRT(*kr20_arm_group_, screw_pose)) {
        return false;
      }
      waitForStateUpdate();
      if (!kr120::motion::executeSlideStroke(*kr20_slide_group_,
                                             slideInsertionTarget(screw_index),
                                             kSlideRetract)) {
        return false;
      }
      waitForStateUpdate();
      if (!kr120::motion::executeRRT(*kr20_arm_group_, approach_pose)) {
        return false;
      }
      waitForStateUpdate();
    }

    return true;
  }

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> kr120_arm_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> kr20_arm_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> kr20_slide_group_;

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr gripper_pub_;

  rclcpp::Service<InitParameters>::SharedPtr init_srv_;
  rclcpp::Service<PickTray>::SharedPtr pick_srv_;
  rclcpp::Service<PlaceTray>::SharedPtr place_srv_;
  rclcpp::Service<InternalScrewingSequence>::SharedPtr screw_srv_;

  rclcpp::TimerBase::SharedPtr init_timer_;
  bool move_groups_ready_{false};
  std::optional<kr120::coord::WorkflowContext> context_;

  std::vector<geometry_msgs::msg::Pose> screw_targets_;
  std::vector<std::vector<int>> screw_sequences_;

  void commandGripper(const std::array<double, 2> &target_positions,
                      double duration_seconds = 0.5) {
    if (!gripper_pub_) {
      RCLCPP_WARN(get_logger(), "Gripper command ignored: publisher not ready.");
      return;
    }

    trajectory_msgs::msg::JointTrajectory trajectory;
    trajectory.header.stamp = now();
    trajectory.joint_names = {"kr120_pinza_left_joint", "kr120_pinza_right_joint"};

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.assign(target_positions.begin(), target_positions.end());
    point.velocities.assign(2, 0.0);
    point.accelerations.assign(2, 0.0);
    point.time_from_start.sec = static_cast<int32_t>(duration_seconds);
    point.time_from_start.nanosec =
        static_cast<uint32_t>((duration_seconds - point.time_from_start.sec) * 1e9);

    trajectory.points.push_back(point);
    gripper_pub_->publish(trajectory);

    // Allow the controller a short window to execute before planning the next motion.
    const auto wait_ms = static_cast<int>(std::max(duration_seconds, 0.1) * 1000.0);
    rclcpp::sleep_for(std::chrono::milliseconds(wait_ms));
  }

  void waitForStateUpdate(double timeout_seconds = 1.0) {
    auto state = kr120_arm_group_->getCurrentState(timeout_seconds);
    if (!state) {
      RCLCPP_WARN(
          get_logger(),
          "Timed out waiting for /joint_states to update (%.2fs). Planning may start from stale state.",
          timeout_seconds);
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CoordinatorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

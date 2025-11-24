#include "kr120_coordinator/workflow_context.hpp"

#include <algorithm>

#include <custom_interfaces/srv/init_parameters.hpp>

namespace kr120::coord {

using custom_interfaces::srv::InitParameters;
using kr120::motion::poseFromArray;

WorkflowContext makeWorkflowContext(
    const InitParameters::Request &req) {
  WorkflowContext ctx;

  ctx.tray_step_poses.reserve(req.tray_step_poses.size() / 6);
  for (std::size_t i = 0; i + 5 < req.tray_step_poses.size(); i += 6) {
    std::array<double, 6> pose_arr{};
    for (std::size_t j = 0; j < 6; ++j) {
      pose_arr[j] = req.tray_step_poses[i + j];
    }
    ctx.tray_step_poses.push_back(pose_arr);
  }

  ctx.tray_final_pose = poseFromArray(req.tray_final_pose);
  ctx.tray_operator_pose = poseFromArray(req.tray_pose_operator);
  ctx.table_pose = poseFromArray(req.table_pose);
  ctx.tray_init_pose = poseFromArray(req.tray_init_pose);
  ctx.recharge_pose = poseFromArray(req.recharge_pose);
  ctx.rest_pose = poseFromArray(req.rest_pose);
  ctx.pick_pose = poseFromArray(req.pick_pose);
  ctx.pre_pick_pose = poseFromArray(req.pre_pick_pose);
  ctx.post_pick_pose = poseFromArray(req.post_pick_pose);
  ctx.bottom_pick_pose = poseFromArray(req.bottom_pick_pose);
  ctx.end_pose = poseFromArray(req.end_pose);
  ctx.pre_end_pose = poseFromArray(req.pre_end_pose);

  ctx.tip_offset = req.tip_offset;
  ctx.tray_angle1 = req.tray_angle1;
  ctx.tray_angle2 = req.tray_angle2;
  ctx.tray_down_steps.assign(req.tray_down_steps.begin(), req.tray_down_steps.end());
  ctx.operator_steps.assign(req.operator_steps.begin(), req.operator_steps.end());
  ctx.rotation_steps.assign(req.rotation_steps.begin(), req.rotation_steps.end());
  ctx.new_tray_steps.assign(req.new_tray_steps.begin(), req.new_tray_steps.end());
  ctx.tray_unit_names.assign(req.tray_unit_names.begin(), req.tray_unit_names.end());
  ctx.tray_unit_pose_indices.assign(req.tray_unit_pose_indices.begin(),
                                    req.tray_unit_pose_indices.end());

  return ctx;
}

}  // namespace kr120::coord

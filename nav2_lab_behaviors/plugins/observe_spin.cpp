// Copyright (c) 2026 nav2_lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include <cmath>
#include <algorithm>
#include <memory>
#include <utility>

#include "nav2_lab_behaviors/plugins/observe_spin.hpp"

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"

namespace nav2_lab_behaviors
{

ObserveSpin::ObserveSpin()
: nav2_behaviors::TimedBehavior<SpinAction>(),
  feedback_(std::make_shared<SpinAction::Feedback>()),
  candidate_angle_(1.57),
  probe_angle_(0.785),
  max_rotational_vel_(1.0),
  min_rotational_vel_(0.4),
  rotational_acc_lim_(3.2),
  simulate_ahead_time_(2.0),
  cmd_yaw_(0.0),
  prev_yaw_(0.0),
  relative_yaw_(0.0)
{
}

ObserveSpin::~ObserveSpin() = default;

void ObserveSpin::onConfigure()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // NOTE: parameters are intentionally prefixed with behavior_name_ + ".".
  // The stock nav2_behaviors::Spin reads BARE names (max_rotational_vel, ...)
  // on the SAME behavior_server node, so a bare name here would collide with
  // Spin and silently reuse its value via declare_parameter_if_not_declared.
  nav2_util::declare_parameter_if_not_declared(
    node, behavior_name_ + ".candidate_angle", rclcpp::ParameterValue(1.57));
  node->get_parameter(behavior_name_ + ".candidate_angle", candidate_angle_);

  nav2_util::declare_parameter_if_not_declared(
    node, behavior_name_ + ".probe_angle", rclcpp::ParameterValue(0.785));
  node->get_parameter(behavior_name_ + ".probe_angle", probe_angle_);

  nav2_util::declare_parameter_if_not_declared(
    node, behavior_name_ + ".max_rotational_vel", rclcpp::ParameterValue(1.0));
  node->get_parameter(behavior_name_ + ".max_rotational_vel", max_rotational_vel_);

  nav2_util::declare_parameter_if_not_declared(
    node, behavior_name_ + ".min_rotational_vel", rclcpp::ParameterValue(0.4));
  node->get_parameter(behavior_name_ + ".min_rotational_vel", min_rotational_vel_);

  nav2_util::declare_parameter_if_not_declared(
    node, behavior_name_ + ".rotational_acc_lim", rclcpp::ParameterValue(3.2));
  node->get_parameter(behavior_name_ + ".rotational_acc_lim", rotational_acc_lim_);

  nav2_util::declare_parameter_if_not_declared(
    node, behavior_name_ + ".simulate_ahead_time", rclcpp::ParameterValue(2.0));
  node->get_parameter(behavior_name_ + ".simulate_ahead_time", simulate_ahead_time_);

  RCLCPP_INFO(
    logger_,
    "ObserveSpin configured: candidate_angle=%.3f probe_angle=%.3f "
    "max_rot=%.3f min_rot=%.3f acc=%.3f sim_ahead=%.3f",
    candidate_angle_, probe_angle_, max_rotational_vel_, min_rotational_vel_,
    rotational_acc_lim_, simulate_ahead_time_);
}

Status ObserveSpin::onRun(const std::shared_ptr<const SpinAction::Goal> command)
{
  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_))
  {
    RCLCPP_ERROR(logger_, "ObserveSpin: current robot pose is not available.");
    return Status::FAILED;
  }

  prev_yaw_ = tf2::getYaw(current_pose.pose.orientation);
  relative_yaw_ = 0.0;

  // ObserveSpin owns the rotation magnitude via its own parameter; the BT's
  // spin_dist (goal target_yaw) is only a fallback when candidate_angle <= 0.
  double magnitude = candidate_angle_;
  if (magnitude <= 0.0) {
    magnitude = std::abs(command->target_yaw);
  }
  if (magnitude < 1e-6) {
    RCLCPP_WARN(logger_, "ObserveSpin: zero rotation requested, nothing to do.");
    return Status::SUCCEEDED;
  }

  geometry_msgs::msg::Pose2D current2d;
  current2d.x = current_pose.pose.position.x;
  current2d.y = current_pose.pose.position.y;
  current2d.theta = prev_yaw_;

  bool left_free = false;
  bool right_free = false;
  const double direction = probeDirection(current2d, left_free, right_free);

  cmd_yaw_ = direction * magnitude;

  RCLCPP_INFO(
    logger_,
    "ObserveSpin: probe left_free=%d right_free=%d -> turning %.3f rad (%s).",
    left_free, right_free, cmd_yaw_, direction >= 0.0 ? "left" : "right");

  command_time_allowance_ = command->time_allowance;
  end_time_ = this->clock_->now() + command_time_allowance_;

  return Status::SUCCEEDED;
}

double ObserveSpin::probeDirection(
  const geometry_msgs::msg::Pose2D & current,
  bool & left_free,
  bool & right_free)
{
  // In-place rotation only: x/y unchanged, vary theta by +/- probe_angle_.
  geometry_msgs::msg::Pose2D left = current;
  geometry_msgs::msg::Pose2D right = current;
  left.theta = current.theta + probe_angle_;
  right.theta = current.theta - probe_angle_;

  // Fetch the latest costmap + footprint on the first probe, reuse on the second
  // (matches the CostmapTopicCollisionChecker's intended optimization).
  left_free = collision_checker_->isCollisionFree(left, true);
  right_free = collision_checker_->isCollisionFree(right, false);

  if (left_free && !right_free) {
    return 1.0;  // left is open, right is blocked
  }
  if (right_free && !left_free) {
    return -1.0;  // right is open, left is blocked
  }

  if (left_free && right_free) {
    // Both free: tie-break by clearance. scorePose returns the footprint cost
    // for genuinely-free poses; lower cost == more open side.
    const double left_score = collision_checker_->scorePose(left, false);
    const double right_score = collision_checker_->scorePose(right, false);
    if (right_score < left_score - 1e-6) {
      return -1.0;
    }
    RCLCPP_INFO(
      logger_,
      "ObserveSpin: both sides free (left_score=%.1f right_score=%.1f); picking left.",
      left_score, right_score);
    return 1.0;
  }

  // Neither free (or data unavailable): the obstacle is likely not yet in the
  // local costmap. An in-place spin still has value -- it changes the laser
  // viewpoint and sweeps the obstacle into the local costmap so the NEXT replan
  // sees it. Default to left.
  RCLCPP_WARN(
    logger_,
    "ObserveSpin: probe inconclusive (left_free=%d right_free=%d); "
    "obstacle likely not yet in local costmap. Defaulting to left.",
    left_free, right_free);
  return 1.0;
}

// --- control loop below mirrors nav2_behaviors::Spin ---

Status ObserveSpin::onCycleUpdate()
{
  rclcpp::Duration time_remaining = end_time_ - this->clock_->now();
  if (time_remaining.seconds() < 0.0 && command_time_allowance_.seconds() > 0.0) {
    stopRobot();
    RCLCPP_WARN(logger_, "ObserveSpin: exceeded time allowance - exiting.");
    return Status::FAILED;
  }

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_))
  {
    RCLCPP_ERROR(logger_, "ObserveSpin: current robot pose is not available.");
    return Status::FAILED;
  }

  const double current_yaw = tf2::getYaw(current_pose.pose.orientation);

  double delta_yaw = current_yaw - prev_yaw_;
  if (abs(delta_yaw) > M_PI) {
    delta_yaw = copysign(2 * M_PI - abs(delta_yaw), prev_yaw_);
  }

  relative_yaw_ += delta_yaw;
  prev_yaw_ = current_yaw;

  feedback_->angular_distance_traveled = static_cast<float>(relative_yaw_);
  action_server_->publish_feedback(feedback_);

  double remaining_yaw = abs(cmd_yaw_) - abs(relative_yaw_);
  if (remaining_yaw < 1e-6) {
    stopRobot();
    return Status::SUCCEEDED;
  }

  // Trapezoidal deceleration profile (matches Spin).
  double vel = sqrt(2 * rotational_acc_lim_ * remaining_yaw);
  vel = std::min(std::max(vel, min_rotational_vel_), max_rotational_vel_);

  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
  cmd_vel->angular.z = copysign(vel, cmd_yaw_);

  geometry_msgs::msg::Pose2D pose2d;
  pose2d.x = current_pose.pose.position.x;
  pose2d.y = current_pose.pose.position.y;
  pose2d.theta = tf2::getYaw(current_pose.pose.orientation);

  if (!isCollisionFree(relative_yaw_, cmd_vel.get(), pose2d)) {
    stopRobot();
    RCLCPP_WARN(logger_, "ObserveSpin: collision ahead - exiting.");
    return Status::FAILED;
  }

  vel_pub_->publish(std::move(cmd_vel));

  return Status::RUNNING;
}

bool ObserveSpin::isCollisionFree(
  const double & relative_yaw,
  geometry_msgs::msg::Twist * cmd_vel,
  geometry_msgs::msg::Pose2D & pose2d)
{
  // Simulate ahead by simulate_ahead_time_ in cycle_frequency_ increments.
  int cycle_count = 0;
  double sim_position_change;
  const int max_cycle_count = static_cast<int>(cycle_frequency_ * simulate_ahead_time_);
  geometry_msgs::msg::Pose2D init_pose = pose2d;
  bool fetch_data = true;

  while (cycle_count < max_cycle_count) {
    sim_position_change = cmd_vel->angular.z * (cycle_count / cycle_frequency_);
    pose2d.theta = init_pose.theta + sim_position_change;
    cycle_count++;

    if (abs(relative_yaw) - abs(sim_position_change) <= 0.) {
      break;
    }

    if (!collision_checker_->isCollisionFree(pose2d, fetch_data)) {
      return false;
    }
    fetch_data = false;
  }
  return true;
}

}  // namespace nav2_lab_behaviors

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_lab_behaviors::ObserveSpin, nav2_core::Behavior)

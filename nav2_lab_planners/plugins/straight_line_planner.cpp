// Copyright (c) 2026 nav2_lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include <cmath>
#include <memory>
#include <string>
#include <utility>

#include "nav2_lab_planners/straight_line_planner.hpp"

#include "nav2_util/line_iterator.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

namespace nav2_lab_planners
{

void StraightLinePlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> /*tf*/,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  global_frame_ = costmap_ros_->getGlobalFrameID();
  name_ = std::move(name);

  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock lifecycle node in StraightLinePlanner."};
  }
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  // Plugin params live under the planner id (name_), e.g.
  // planner_server.ros__parameters.LabStraightLine.<param>, matching how
  // planner_server scopes each plugin's parameters.
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".obstacle_cost_threshold", rclcpp::ParameterValue(253.0));
  obstacle_cost_threshold_ =
    node->get_parameter(name_ + ".obstacle_cost_threshold").as_double();

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".allow_unknown", rclcpp::ParameterValue(false));
  allow_unknown_ = node->get_parameter(name_ + ".allow_unknown").as_bool();

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".sample_step", rclcpp::ParameterValue(1));
  sample_step_ = static_cast<int>(node->get_parameter(name_ + ".sample_step").as_int());
  if (sample_step_ < 1) {
    sample_step_ = 1;  // guard against modulo-by-zero
  }

  RCLCPP_INFO(
    logger_, "StraightLinePlanner configured: threshold=%.0f allow_unknown=%d sample_step=%d",
    obstacle_cost_threshold_, allow_unknown_, sample_step_);
}

void StraightLinePlanner::activate()
{
  RCLCPP_INFO(logger_, "Activating StraightLinePlanner");
}

void StraightLinePlanner::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating StraightLinePlanner");
}

void StraightLinePlanner::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up StraightLinePlanner");
  costmap_ = nullptr;
  costmap_ros_.reset();
}

nav_msgs::msg::Path StraightLinePlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = global_frame_;
  path.header.stamp = clock_->now();

  if (!costmap_) {
    RCLCPP_ERROR(logger_, "Costmap is null, cannot create plan");
    return path;
  }

  unsigned int mx0, my0, mx1, my1;
  if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, mx0, my0) ||
    !costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, mx1, my1))
  {
    RCLCPP_WARN(
      logger_,
      "Start (%.2f,%.2f) or goal (%.2f,%.2f) outside costmap bounds; no plan",
      start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);
    return path;
  }

  // Heading along the straight line; applied to every sampled pose.
  const double yaw = std::atan2(
    goal.pose.position.y - start.pose.position.y,
    goal.pose.position.x - start.pose.position.x);
  const double half_yaw = yaw / 2.0;

  nav2_util::LineIterator it(
    static_cast<int>(mx0), static_cast<int>(my0),
    static_cast<int>(mx1), static_cast<int>(my1));

  int step = 0;
  for (; it.isValid(); it.advance()) {
    const unsigned int mx = static_cast<unsigned int>(it.getX());
    const unsigned int my = static_cast<unsigned int>(it.getY());
    const unsigned char cost = costmap_->getCost(mx, my);

    const bool blocked = static_cast<double>(cost) >= obstacle_cost_threshold_ ||
      (!allow_unknown_ && cost == nav2_costmap_2d::NO_INFORMATION);
    if (blocked) {
      RCLCPP_WARN(
        logger_,
        "StraightLinePlanner: blocked at cell (%d,%d) cost=%d; returning empty path",
        static_cast<int>(mx), static_cast<int>(my), static_cast<int>(cost));
      path.poses.clear();  // planning failure: empty path so the BT RecoveryNode reacts
      return path;
    }

    if (step % sample_step_ == 0) {
      double wx = 0.0;
      double wy = 0.0;
      costmap_->mapToWorld(mx, my, wx, wy);
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = wx;
      pose.pose.position.y = wy;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.z = std::sin(half_yaw);
      pose.pose.orientation.w = std::cos(half_yaw);
      path.poses.push_back(pose);
    }
    step++;
  }

  // Append the exact goal so the path is not truncated at grid resolution.
  geometry_msgs::msg::PoseStamped goal_pose = goal;
  goal_pose.header = path.header;
  path.poses.push_back(goal_pose);

  RCLCPP_INFO(
    logger_,
    "StraightLinePlanner: built plan with %zu poses from (%.2f,%.2f) to (%.2f,%.2f)",
    path.poses.size(), start.pose.position.x, start.pose.position.y,
    goal.pose.position.x, goal.pose.position.y);

  return path;
}

}  // namespace nav2_lab_planners

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_lab_planners::StraightLinePlanner, nav2_core::GlobalPlanner)

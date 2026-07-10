// Copyright (c) 2026 nav2_lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef NAV2_LAB_PLANNERS__STRAIGHT_LINE_PLANNER_HPP_
#define NAV2_LAB_PLANNERS__STRAIGHT_LINE_PLANNER_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav2_lab_planners
{

/**
 * @class nav2_lab_planners::StraightLinePlanner
 * @brief Minimal global planner: traces a Bresenham line from start to goal on
 *        the costmap, collision-checks every cell, and returns the line as a
 *        nav_msgs/Path. Returns an empty path (planning failure) if any cell on
 *        the line is at/above the obstacle threshold. Pedagogical planner to
 *        exercise the nav2_core::GlobalPlanner plugin plumbing.
 */
class StraightLinePlanner : public nav2_core::GlobalPlanner
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  std::string global_frame_;
  std::string name_;

  // Tunables, namespaced under the planner id (e.g. LabStraightLine.*).
  double obstacle_cost_threshold_{253.0};  // cost >= this is treated as blocked
  bool allow_unknown_{false};              // if false, NO_INFORMATION (255) blocks
  int sample_step_{1};                     // emit one path pose every N cells

  rclcpp::Logger logger_{rclcpp::get_logger("StraightLinePlanner")};
  rclcpp::Clock::SharedPtr clock_;
};

}  // namespace nav2_lab_planners

#endif  // NAV2_LAB_PLANNERS__STRAIGHT_LINE_PLANNER_HPP_

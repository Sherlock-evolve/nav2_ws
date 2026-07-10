// Copyright (c) 2026 nav2_lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include "nav2_lab_costmaps/marker_layer.hpp"

#include "nav2_util/node_utils.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_lab_costmaps
{

void MarkerLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock lifecycle node in MarkerLayer."};
  }

  // Plugin params live under the layer id (name_), e.g.
  // global_costmap.lab_marker_layer.<param>, matching how LayeredCostmap scopes
  // each plugin's parameters.
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".enabled", rclcpp::ParameterValue(true));
  enabled_ = node->get_parameter(name_ + ".enabled").as_bool();

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".point_x", rclcpp::ParameterValue(0.0));
  point_x_ = node->get_parameter(name_ + ".point_x").as_double();

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".point_y", rclcpp::ParameterValue(0.0));
  point_y_ = node->get_parameter(name_ + ".point_y").as_double();

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".radius", rclcpp::ParameterValue(0.5));
  radius_ = node->get_parameter(name_ + ".radius").as_double();
  if (radius_ <= 0.0) {
    radius_ = 0.0;  // a zero radius makes the layer a no-op
    enabled_ = false;
  }

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".peak_cost", rclcpp::ParameterValue(254));
  peak_cost_ = static_cast<int>(node->get_parameter(name_ + ".peak_cost").as_int());
  if (peak_cost_ < 0) {
    peak_cost_ = 0;
  } else if (peak_cost_ > 255) {
    peak_cost_ = 255;
  }

  current_ = true;

  RCLCPP_INFO(
    logger_,
    "MarkerLayer configured: enabled=%d point=(%.2f,%.2f) radius=%.2f peak_cost=%d",
    enabled_, point_x_, point_y_, radius_, peak_cost_);
}

void MarkerLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  (void)robot_x;
  (void)robot_y;
  (void)robot_yaw;
  if (!enabled_) {
    return;
  }
  // Expand the shared bounding box to include the marker blob. Must use
  // min/max (not overwrite) — bounds are shared across all layers.
  *min_x = std::min(*min_x, point_x_ - radius_);
  *min_y = std::min(*min_y, point_y_ - radius_);
  *max_x = std::max(*max_x, point_x_ + radius_);
  *max_y = std::max(*max_y, point_y_ + radius_);
}

void MarkerLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }

  unsigned char * master_array = master_grid.getCharMap();

  for (int j = min_j; j < max_j; j++) {
    for (int i = min_i; i < max_i; i++) {
      double wx = 0.0;
      double wy = 0.0;
      master_grid.mapToWorld(
        static_cast<unsigned int>(i), static_cast<unsigned int>(j), wx, wy);
      const double dist = std::hypot(wx - point_x_, wy - point_y_);
      if (dist > radius_) {
        continue;
      }
      // Linear decay: peak_cost at center, 0 at radius.
      const double factor = 1.0 - (dist / radius_);
      const unsigned char cost = static_cast<unsigned char>(peak_cost_ * factor);
      const unsigned int index = master_grid.getIndex(
        static_cast<unsigned int>(i), static_cast<unsigned int>(j));
      // Merge with max so we never lower existing cost (walls, inflation).
      master_array[index] = std::max(master_array[index], cost);
    }
  }
  current_ = true;
}

}  // namespace nav2_lab_costmaps

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_lab_costmaps::MarkerLayer, nav2_costmap_2d::Layer)

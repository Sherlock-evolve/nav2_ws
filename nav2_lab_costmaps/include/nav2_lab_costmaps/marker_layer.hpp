// Copyright (c) 2026 nav2_lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef NAV2_LAB_COSTMAPS__MARKER_LAYER_HPP_
#define NAV2_LAB_COSTMAPS__MARKER_LAYER_HPP_

#include <string>

#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace nav2_lab_costmaps
{

/**
 * @class nav2_lab_costmaps::MarkerLayer
 * @brief Minimal costmap layer that stamps a linearly decaying cost blob around
 *        a parameterized fixed point (point_x, point_y): cost = peak_cost at the
 *        center, linearly decaying to 0 at `radius`. Merged into the master grid
 *        with std::max so it never lowers existing cost (walls, inflation).
 *        Parameter-only (no subscriptions, no TF). Pedagogical layer to exercise
 *        the nav2_costmap_2d::Layer plugin plumbing.
 */
class MarkerLayer : public nav2_costmap_2d::Layer
{
public:
  MarkerLayer() = default;
  ~MarkerLayer() override = default;

  void onInitialize() override;

  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;

  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

  void reset() override { current_ = false; }
  bool isClearable() override { return false; }

private:
  // Tunables, namespaced under the layer id (e.g. lab_marker_layer.*).
  double point_x_{0.0};
  double point_y_{0.0};
  double radius_{0.5};
  int peak_cost_{254};
};

}  // namespace nav2_lab_costmaps

#endif  // NAV2_LAB_COSTMAPS__MARKER_LAYER_HPP_

// Copyright (c) 2026 nav2_lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef NAV2_LAB_BEHAVIORS__PLUGINS__OBSERVE_SPIN_HPP_
#define NAV2_LAB_BEHAVIORS__PLUGINS__OBSERVE_SPIN_HPP_

#include <memory>
#include <string>

#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_msgs/action/spin.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_lab_behaviors
{

using SpinAction = nav2_msgs::action::Spin;

// Status is declared at namespace scope inside nav2_behaviors (see
// timed_behavior.hpp), NOT as a nested type of TimedBehavior. The stock
// nav2_behaviors::Spin lives in that namespace so it can use the bare name;
// since this plugin is in a different namespace we must import it so the
// override signatures match the base class.
using nav2_behaviors::Status;

/**
 * @class nav2_lab_behaviors::ObserveSpin
 * @brief Recovery behavior that probes left/right candidate poses via the local
 *        costmap collision checker, then spins in place toward the more-open
 *        side so the laser sweeps unmapped obstacles into the local costmap,
 *        letting Navfn's next replan route around them.
 *
 * Reuses nav2_msgs::action::Spin (no custom .action). It is exposed under the
 * action-server name it is registered with (the yaml key, e.g. "observe_spin")
 * and is reached from a BT XML via `<Spin server_name="observe_spin" .../>`.
 */
class ObserveSpin : public nav2_behaviors::TimedBehavior<SpinAction>
{
public:
  ObserveSpin();
  ~ObserveSpin();

  /**
   * @brief Reads the current pose, probes both candidate end-poses, fixes the
   *        spin direction (sign) and magnitude, and seeds control-loop state.
   * @return SUCCEEDED once targets are set; FAILED if the pose/TF is unavailable.
   */
  Status onRun(const std::shared_ptr<const SpinAction::Goal> command) override;

  /** @brief Declares/reads the observe_spin.* parameters. */
  void onConfigure() override;

  /** @brief Per-cycle trapezoidal in-place rotation with collision look-ahead. */
  Status onCycleUpdate() override;

protected:
  /**
   * @brief Forward-simulates the commanded angular velocity for
   *        simulate_ahead_time_ in cycle_frequency_ increments and rejects the
   *        command if any projected pose is in collision. Mirrors Spin.
   */
  bool isCollisionFree(
    const double & relative_yaw,
    geometry_msgs::msg::Twist * cmd_vel,
    geometry_msgs::msg::Pose2D & pose2d);

  /**
   * @brief Probes the left/right end-poses (rotated in place by probe_angle_)
   *        and returns the recommended direction sign (+1 left / -1 right).
   *        Sets left_free / right_free out-params for logging.
   */
  double probeDirection(
    const geometry_msgs::msg::Pose2D & current,
    bool & left_free,
    bool & right_free);

  SpinAction::Feedback::SharedPtr feedback_;

  // Tunables. Prefixed (observe_spin.*) to avoid colliding with the stock
  // Spin, which reads BARE names on the same behavior_server node.
  double candidate_angle_;       // magnitude of the in-place spin [rad]
  double probe_angle_;           // look-ahead angle used to sample left/right [rad]
  double max_rotational_vel_;
  double min_rotational_vel_;
  double rotational_acc_lim_;
  double simulate_ahead_time_;

  // Control-loop state (mirrors Spin).
  double cmd_yaw_;
  double prev_yaw_;
  double relative_yaw_;
  rclcpp::Duration command_time_allowance_{0, 0};
  rclcpp::Time end_time_;
};

}  // namespace nav2_lab_behaviors

#endif  // NAV2_LAB_BEHAVIORS__PLUGINS__OBSERVE_SPIN_HPP_

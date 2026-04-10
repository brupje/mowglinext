// Copyright 2026 Mowgli Project
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
/**
 * @file gps_pose_converter_node.hpp
 * @brief GPS pose converter — publishes position + heading for EKF fusion.
 *
 * Converts mowgli_interfaces/msg/AbsolutePose into
 * geometry_msgs/msg/PoseWithCovarianceStamped for robot_localization's EKF.
 *
 * Position: every fix is published with covariance scaled by fix quality.
 * Heading:  derived from consecutive GPS positions (velocity vector).
 *           Only published with tight covariance when speed exceeds a
 *           threshold; when stationary, yaw covariance is very large so
 *           the IMU dominates heading.
 */

#pragma once

#include <memory>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "mowgli_interfaces/msg/absolute_pose.hpp"
#include "mowgli_interfaces/msg/status.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mowgli_localization
{

class GpsPoseConverterNode : public rclcpp::Node
{
public:
  explicit GpsPoseConverterNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~GpsPoseConverterNode() override = default;

private:
  void declare_parameters();
  void create_publishers();
  void create_subscribers();

  void on_absolute_pose(mowgli_interfaces::msg::AbsolutePose::ConstSharedPtr msg);
  void on_status(mowgli_interfaces::msg::Status::ConstSharedPtr msg);

  double compute_xy_variance(const mowgli_interfaces::msg::AbsolutePose& msg) const;

  /// Returns the covariance multiplier for the dock/undock transition ramp.
  /// 1.0 = normal, >1.0 = inflated during ramp-down after undock.
  double dock_covariance_multiplier() const;

  // ---------------------------------------------------------------------------
  // Parameters
  // ---------------------------------------------------------------------------
  double min_accuracy_threshold_{0.5};

  /// Minimum speed (m/s) for GPS heading to be considered valid.
  double heading_min_speed_{0.15};

  /// Yaw variance (rad²) when speed is well above the threshold.
  double heading_good_variance_{0.1};

  /// Duration (seconds) to ramp GPS covariance from inflated back to normal
  /// after undocking. During this ramp, position covariance is multiplied by
  /// a factor that linearly decreases from gps_ramp_start_multiplier_ to 1.0.
  double gps_ramp_duration_{8.0};

  /// Starting covariance multiplier at the moment of undock.
  double gps_ramp_start_multiplier_{100.0};

  // ---------------------------------------------------------------------------
  // State for heading computation
  // ---------------------------------------------------------------------------
  bool has_prev_pose_{false};
  double prev_x_{0.0};
  double prev_y_{0.0};
  rclcpp::Time prev_stamp_;

  // ---------------------------------------------------------------------------
  // Dock/undock GPS gating state
  // ---------------------------------------------------------------------------

  /// True when the robot is charging (docked). GPS is suppressed.
  bool is_docked_{false};

  /// Timestamp when undock was detected (is_charging went false).
  /// Used to compute the covariance ramp-down.
  rclcpp::Time undock_time_;

  /// True once the ramp-down period has completed after undock.
  bool ramp_complete_{true};

  // ---------------------------------------------------------------------------
  // ROS handles
  // ---------------------------------------------------------------------------
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Subscription<mowgli_interfaces::msg::AbsolutePose>::SharedPtr abs_pose_sub_;
  rclcpp::Subscription<mowgli_interfaces::msg::Status>::SharedPtr status_sub_;
};

}  // namespace mowgli_localization

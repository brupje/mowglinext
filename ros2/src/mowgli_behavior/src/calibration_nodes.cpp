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

#include "mowgli_behavior/calibration_nodes.hpp"

#include <cmath>

#include "rcl_interfaces/msg/parameter.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_localization/srv/set_pose.hpp"
#include "slam_toolbox/srv/reset.hpp"

namespace mowgli_behavior
{

// ---------------------------------------------------------------------------
// RecordUndockStart
// ---------------------------------------------------------------------------

BT::NodeStatus RecordUndockStart::tick()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  ctx->undock_start_x = ctx->gps_x;
  ctx->undock_start_y = ctx->gps_y;
  ctx->undock_start_recorded = true;
  RCLCPP_INFO(ctx->node->get_logger(),
              "RecordUndockStart: pos=(%.3f, %.3f)",
              ctx->undock_start_x,
              ctx->undock_start_y);
  return BT::NodeStatus::SUCCESS;
}

// ---------------------------------------------------------------------------
// CalibrateHeadingFromUndock
// ---------------------------------------------------------------------------

BT::NodeStatus CalibrateHeadingFromUndock::tick()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (!ctx->undock_start_recorded)
  {
    RCLCPP_WARN(ctx->node->get_logger(), "CalibrateHeadingFromUndock: no start position recorded");
    return BT::NodeStatus::SUCCESS;  // non-fatal
  }

  // Only calibrate with RTK fix — without it, GPS position is too noisy
  // and the computed heading would be wrong.
  if (!ctx->gps_is_fixed)
  {
    RCLCPP_WARN(ctx->node->get_logger(),
                "CalibrateHeadingFromUndock: no RTK fix, skipping (would corrupt heading)");
    ctx->undock_start_recorded = false;
    return BT::NodeStatus::SUCCESS;  // non-fatal
  }

  const double dx = ctx->gps_x - ctx->undock_start_x;
  const double dy = ctx->gps_y - ctx->undock_start_y;
  const double dist = std::sqrt(dx * dx + dy * dy);

  if (dist < 0.3)
  {
    RCLCPP_WARN(ctx->node->get_logger(),
                "CalibrateHeadingFromUndock: displacement too small (%.2f m), skipping",
                dist);
    return BT::NodeStatus::SUCCESS;  // non-fatal
  }

  // The robot moved BACKWARD, so the heading is OPPOSITE to the displacement vector
  const double heading = std::atan2(-dy, -dx);

  RCLCPP_INFO(ctx->node->get_logger(),
              "CalibrateHeadingFromUndock: displacement=(%.3f, %.3f) dist=%.2f heading=%.1f deg",
              dx,
              dy,
              dist,
              heading * 180.0 / M_PI);

  // Set EKF pose via /set_pose service
  auto client = ctx->node->create_client<robot_localization::srv::SetPose>("/set_pose");
  if (!client->wait_for_service(std::chrono::seconds(2)))
  {
    RCLCPP_WARN(ctx->node->get_logger(),
                "CalibrateHeadingFromUndock: /set_pose service not available");
    return BT::NodeStatus::SUCCESS;  // non-fatal
  }

  auto request = std::make_shared<robot_localization::srv::SetPose::Request>();
  request->pose.header.stamp = ctx->node->now();
  request->pose.header.frame_id = "map";
  request->pose.pose.pose.position.x = ctx->gps_x;
  request->pose.pose.pose.position.y = ctx->gps_y;
  request->pose.pose.pose.orientation.z = std::sin(heading / 2.0);
  request->pose.pose.pose.orientation.w = std::cos(heading / 2.0);
  // Tight covariance for position and yaw
  request->pose.pose.covariance[0] = 0.01;  // x
  request->pose.pose.covariance[7] = 0.01;  // y
  request->pose.pose.covariance[35] = 0.05;  // yaw

  auto future = client->async_send_request(request);
  // Don't wait for result — fire and forget
  (void)future;

  ctx->undock_start_recorded = false;

  // Also reset SLAM so it restarts with correct heading.
  // Set map_start_pose parameter to GPS position + heading, then reset.
  auto param_client =
      ctx->node->create_client<rcl_interfaces::srv::SetParameters>("/slam_toolbox/set_parameters");
  if (param_client->wait_for_service(std::chrono::seconds(2)))
  {
    auto param_req = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    rcl_interfaces::msg::Parameter p;
    p.name = "map_start_pose";
    p.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY;
    p.value.double_array_value = {ctx->gps_x, ctx->gps_y, heading};
    param_req->parameters.push_back(p);
    auto param_future = param_client->async_send_request(param_req);
    // Wait briefly for parameter to be set before resetting
    param_future.wait_for(std::chrono::seconds(1));

    RCLCPP_INFO(ctx->node->get_logger(),
                "CalibrateHeadingFromUndock: set SLAM map_start_pose to [%.3f, %.3f, %.3f]",
                ctx->gps_x,
                ctx->gps_y,
                heading);

    // Now reset SLAM — it will restart from the new map_start_pose
    auto reset_client = ctx->node->create_client<slam_toolbox::srv::Reset>("/slam_toolbox/reset");
    if (reset_client->wait_for_service(std::chrono::seconds(2)))
    {
      auto reset_req = std::make_shared<slam_toolbox::srv::Reset::Request>();
      reset_req->pause_new_measurements = false;
      auto reset_future = reset_client->async_send_request(reset_req);
      (void)reset_future;
      RCLCPP_INFO(ctx->node->get_logger(),
                  "CalibrateHeadingFromUndock: SLAM reset with correct heading");
    }
    else
    {
      RCLCPP_WARN(ctx->node->get_logger(),
                  "CalibrateHeadingFromUndock: /slam_toolbox/reset not available");
    }
  }
  else
  {
    RCLCPP_WARN(ctx->node->get_logger(),
                "CalibrateHeadingFromUndock: /slam_toolbox/set_parameters not available");
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace mowgli_behavior

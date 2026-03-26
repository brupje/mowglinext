// Copyright (C) 2024 Cedric
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * @file test_coverage_planner.cpp
 * @brief Integration tests for CoveragePlannerNode service logic.
 *
 * Tests spin the CoveragePlannerNode together with a lightweight client node
 * inside a MultiThreadedExecutor running on a background thread.  Each test
 * sends a synchronous service request and verifies the geometric properties
 * of the returned paths.
 *
 * All tests use a simple 10×10 m² square boundary unless stated otherwise.
 */

#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "mowgli_interfaces/srv/plan_coverage_path.hpp"
#include "nav_msgs/msg/path.hpp"

#include "mowgli_coverage_planner/coverage_planner_node.hpp"
#include "mowgli_coverage_planner/polygon_utils.hpp"

using mowgli_coverage_planner::CoveragePlannerNode;
using PlanCoveragePath = mowgli_interfaces::srv::PlanCoveragePath;
using namespace std::chrono_literals;

// ---------------------------------------------------------------------------
// Test fixture
// ---------------------------------------------------------------------------

class CoveragePlannerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Build the planner node with test-friendly parameters.
    rclcpp::NodeOptions planner_opts;
    planner_opts.append_parameter_override("tool_width", 0.5);
    planner_opts.append_parameter_override("headland_passes", 1);
    planner_opts.append_parameter_override("headland_width", 0.5);
    planner_opts.append_parameter_override("path_spacing", 0.5);
    planner_opts.append_parameter_override("map_frame", "map");
    planner_node_ = std::make_shared<CoveragePlannerNode>(planner_opts);

    // Build a minimal client node.
    client_node_ = rclcpp::Node::make_shared("test_client_node");
    client_ = client_node_->create_client<PlanCoveragePath>(
      "/coverage_planner_node/plan_coverage");

    // Spin both nodes on a background thread.
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(planner_node_);
    executor_->add_node(client_node_);

    spin_thread_ = std::thread([this]() {executor_->spin();});

    // Wait for service to be ready.
    ASSERT_TRUE(client_->wait_for_service(5s))
      << "PlanCoveragePath service did not become available within 5 s";
  }

  void TearDown() override
  {
    executor_->cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
  }

  // ---- Helpers ------------------------------------------------------------

  /// Build a 10×10 m² square polygon (CCW).
  static geometry_msgs::msg::Polygon make_square_10x10()
  {
    return make_rectangle(0.0f, 0.0f, 10.0f, 10.0f);
  }

  /// Build an axis-aligned rectangular polygon (CCW).
  static geometry_msgs::msg::Polygon make_rectangle(
    float x0, float y0, float x1, float y1)
  {
    geometry_msgs::msg::Polygon poly;
    for (auto [x, y] : std::vector<std::pair<float, float>>{
        {x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}})
    {
      geometry_msgs::msg::Point32 pt;
      pt.x = x;
      pt.y = y;
      pt.z = 0.0f;
      poly.points.push_back(pt);
    }
    return poly;
  }

  /// Send a synchronous service request and return the response.
  PlanCoveragePath::Response::SharedPtr call_service(
    const geometry_msgs::msg::Polygon & boundary,
    const std::vector<geometry_msgs::msg::Polygon> & obstacles = {},
    double mow_angle_deg = -1.0,
    bool skip_outline = false)
  {
    auto req = std::make_shared<PlanCoveragePath::Request>();
    req->outer_boundary = boundary;
    req->obstacles = obstacles;
    req->mow_angle_deg = mow_angle_deg;
    req->skip_outline = skip_outline;

    auto future = client_->async_send_request(req);
    if (future.wait_for(10s) != std::future_status::ready) {
      auto error_res = std::make_shared<PlanCoveragePath::Response>();
      error_res->success = false;
      error_res->message = "timeout waiting for service response";
      return error_res;
    }
    return future.get();
  }

  // ---- Members ------------------------------------------------------------

  std::shared_ptr<CoveragePlannerNode> planner_node_;
  rclcpp::Node::SharedPtr client_node_;
  rclcpp::Client<PlanCoveragePath>::SharedPtr client_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
};

// ---------------------------------------------------------------------------
// Test: valid path on a simple 10×10 square
// ---------------------------------------------------------------------------

TEST_F(CoveragePlannerTest, SimpleSqaureProducesValidPath)
{
  const auto res = call_service(make_square_10x10());

  ASSERT_TRUE(res->success) << "Service failed: " << res->message;
  EXPECT_GT(res->path.poses.size(), 0u) << "Expected non-empty swath path";
}

// ---------------------------------------------------------------------------
// Test: total distance exceeds a reasonable lower bound
// ---------------------------------------------------------------------------

TEST_F(CoveragePlannerTest, TotalDistanceReasonable)
{
  const auto res = call_service(make_square_10x10());

  ASSERT_TRUE(res->success) << res->message;

  // Inner polygon after 0.5 m headland inset: ~9×9 = 81 m².
  // Lower bound: inner_area / path_spacing * 0.5 (50 % efficiency).
  const double inner_area = res->coverage_area;
  const double path_spacing = 0.5;
  const double min_expected = inner_area / path_spacing * 0.5;

  EXPECT_GT(res->total_distance, min_expected)
    << "total_distance=" << res->total_distance
    << " expected > " << min_expected;
}

// ---------------------------------------------------------------------------
// Test: outline path generated when skip_outline=false
// ---------------------------------------------------------------------------

TEST_F(CoveragePlannerTest, OutlinePathGeneratedWhenNotSkipped)
{
  const auto res = call_service(make_square_10x10(), {}, -1.0, false);

  ASSERT_TRUE(res->success) << res->message;
  EXPECT_GT(res->outline_path.poses.size(), 0u)
    << "Expected non-empty outline path when skip_outline=false";
}

// ---------------------------------------------------------------------------
// Test: no outline when skip_outline=true
// ---------------------------------------------------------------------------

TEST_F(CoveragePlannerTest, OutlinePathEmptyWhenSkipped)
{
  const auto res = call_service(make_square_10x10(), {}, -1.0, true);

  ASSERT_TRUE(res->success) << res->message;
  EXPECT_EQ(res->outline_path.poses.size(), 0u)
    << "Expected empty outline path when skip_outline=true";
}

// ---------------------------------------------------------------------------
// Test: custom mow angle is respected
// ---------------------------------------------------------------------------

TEST_F(CoveragePlannerTest, CustomMowAngle45Degrees)
{
  const auto res = call_service(make_square_10x10(), {}, 45.0, true);

  ASSERT_TRUE(res->success) << res->message;
  ASSERT_GT(res->path.poses.size(), 0u);

  // Extract yaw from the first pose quaternion.
  const auto & q = res->path.poses.front().pose.orientation;
  const double yaw = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));

  // The yaw should be close to ±45° (forward) or ±135° (return pass).
  const double abs_yaw = std::abs(yaw);
  const double pi_4 = M_PI / 4.0;

  const bool near_45_or_135 =
    (std::abs(abs_yaw - pi_4) < 15.0 * M_PI / 180.0) ||
    (std::abs(abs_yaw - 3.0 * pi_4) < 15.0 * M_PI / 180.0);

  EXPECT_TRUE(near_45_or_135)
    << "Expected yaw near ±45° or ±135°, got " << yaw * 180.0 / M_PI << "°";
}

// ---------------------------------------------------------------------------
// Test: coverage_area matches expected inner area
// ---------------------------------------------------------------------------

TEST_F(CoveragePlannerTest, CoverageAreaMatchesInnerPolygon)
{
  const auto res = call_service(make_square_10x10());

  ASSERT_TRUE(res->success) << res->message;

  // After 0.5 m inset on all sides: (10 - 2*0.5) × (10 - 2*0.5) = 9×9 = 81 m².
  EXPECT_NEAR(res->coverage_area, 81.0, 1.5);  // ±1.5 m² tolerance
}

// ---------------------------------------------------------------------------
// Test: degenerate polygon (< 3 vertices) is rejected gracefully
// ---------------------------------------------------------------------------

TEST_F(CoveragePlannerTest, DegeneratePolygonRejected)
{
  geometry_msgs::msg::Polygon bad_poly;
  geometry_msgs::msg::Point32 pt;
  pt.x = 0.0f; pt.y = 0.0f; pt.z = 0.0f;
  bad_poly.points.push_back(pt);
  pt.x = 1.0f;
  bad_poly.points.push_back(pt);
  // Only 2 vertices — must be rejected.

  const auto res = call_service(bad_poly);
  EXPECT_FALSE(res->success);
  EXPECT_FALSE(res->message.empty());
}

// ---------------------------------------------------------------------------
// Test: auto-angle on a wide rectangle produces a valid path
// ---------------------------------------------------------------------------

TEST_F(CoveragePlannerTest, AutoAngleOnWideRectangle)
{
  const auto res = call_service(make_rectangle(0.0f, 0.0f, 20.0f, 4.0f));

  ASSERT_TRUE(res->success) << res->message;
  EXPECT_GT(res->path.poses.size(), 0u);
}

// ---------------------------------------------------------------------------
// GTest main
// ---------------------------------------------------------------------------

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}

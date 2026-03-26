// Copyright (C) 2024 Cedric
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * @file coverage_planner_node.hpp
 * @brief ROS2 node that provides coverage path planning via a service interface.
 *
 * The node accepts an area boundary and a list of obstacle polygons, then
 * generates an efficient boustrophedon (zigzag) mowing path using:
 *   1. Headland pass generation (polygon contraction by headland_width).
 *   2. Optimal or user-specified swath angle selection.
 *   3. Parallel swath generation clipped to the inner area.
 *   4. Route ordering to minimise travel distance.
 *
 * The implementation is self-contained and relies only on the polygon_utils
 * utilities — no external geometry library (CGAL, Clipper, Boost.Geometry)
 * is required.
 */

#ifndef MOWGLI_COVERAGE_PLANNER__COVERAGE_PLANNER_NODE_HPP_
#define MOWGLI_COVERAGE_PLANNER__COVERAGE_PLANNER_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mowgli_interfaces/srv/plan_coverage_path.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

#include "mowgli_coverage_planner/polygon_utils.hpp"

namespace mowgli_coverage_planner
{

/**
 * @brief Coverage path planner node.
 *
 * Advertises the `~/plan_coverage` service and publishes the computed paths
 * on `~/coverage_path` and `~/coverage_outline` for RViz visualisation.
 */
class CoveragePlannerNode : public rclcpp::Node
{
public:
  /**
   * @brief Construct the node, declare parameters, create publishers and service.
   *
   * @param options Node options forwarded to rclcpp::Node.
   */
  explicit CoveragePlannerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // -------------------------------------------------------------------------
  // Service callback
  // -------------------------------------------------------------------------

  /**
   * @brief Handle a PlanCoveragePath service request.
   *
   * Validates the input polygon, runs the coverage algorithm, fills the
   * response, and publishes the resulting paths for visualisation.
   *
   * @param request  Incoming service request.
   * @param response Outgoing service response (mutated in place).
   */
  void handle_plan_coverage(
    const std::shared_ptr<mowgli_interfaces::srv::PlanCoveragePath::Request> request,
    std::shared_ptr<mowgli_interfaces::srv::PlanCoveragePath::Response> response);

  // -------------------------------------------------------------------------
  // Internal planning helpers
  // -------------------------------------------------------------------------

  /**
   * @brief Generate the headland outline path.
   *
   * Contracts the outer boundary by (headland_passes_ * headland_width_) and
   * returns a nav_msgs::Path that traces all headland offset rings.
   *
   * @param outer   Outer boundary polygon.
   * @param frame   Coordinate frame for the header stamp.
   * @return Path tracing the headland contour(s).
   */
  nav_msgs::msg::Path generate_outline_path(
    const Polygon2D & outer,
    const std::string & frame) const;

  /**
   * @brief Generate the boustrophedon swath path for the inner area.
   *
   * @param inner      Inner polygon (already contracted by headland passes).
   * @param angle_rad  Mowing angle in radians.
   * @param frame      Coordinate frame for the header stamp.
   * @return Path containing all swath traversals.
   */
  nav_msgs::msg::Path generate_swath_path(
    const Polygon2D & inner,
    double angle_rad,
    const std::string & frame) const;

  /**
   * @brief Compute the total Euclidean path length.
   *
   * @param path Input path (pose sequence).
   * @return Sum of distances between consecutive poses in metres.
   */
  static double compute_path_length(const nav_msgs::msg::Path & path);

  /**
   * @brief Build a PoseStamped with the given position and heading.
   *
   * @param x     X position.
   * @param y     Y position.
   * @param yaw   Heading angle in radians.
   * @param frame Coordinate frame id.
   * @return Fully populated PoseStamped.
   */
  static geometry_msgs::msg::PoseStamped make_pose(
    double x, double y, double yaw, const std::string & frame);

  // -------------------------------------------------------------------------
  // ROS interfaces
  // -------------------------------------------------------------------------

  rclcpp::Service<mowgli_interfaces::srv::PlanCoveragePath>::SharedPtr service_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr outline_pub_;

  // -------------------------------------------------------------------------
  // Parameters
  // -------------------------------------------------------------------------

  double tool_width_;        ///< Mowing blade / disc width [m].
  int headland_passes_;      ///< Number of headland perimeter passes.
  double headland_width_;    ///< Width of one headland pass [m].
  double default_mow_angle_; ///< Default mowing angle [deg]; -1 = auto.
  double path_spacing_;      ///< Distance between parallel swath centrelines [m].
  std::string map_frame_;    ///< TF frame for output paths.
};

}  // namespace mowgli_coverage_planner

#endif  // MOWGLI_COVERAGE_PLANNER__COVERAGE_PLANNER_NODE_HPP_

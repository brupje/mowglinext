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
// ---------------------------------------------------------------------------
// GPS-SLAM Corrector Node
//
// Publishes the map→slam_map TF that bridges GPS (real-world ENU) with
// SLAM Toolbox's internal coordinate frame.
//
// Architecture:
//   map  →  slam_map  →  odom  →  base_footprint
//   (GPS      (SLAM)      (FusionCore: IMU+wheels)
//    correction)
//
// SLAM Toolbox publishes slam_map→odom from lidar scan matching.
// FusionCore publishes odom→base_footprint from IMU+wheels (no GPS).
// This node computes map→slam_map so that the final map→base_footprint
// is anchored to GPS coordinates while using SLAM for local precision.
//
// The correction is low-pass filtered to avoid GPS noise causing jumps.
// In feature-rich areas, SLAM is precise and the correction stays small.
// In open fields with RTK, GPS gently pulls the map toward reality.
// ---------------------------------------------------------------------------

#include <cmath>
#include <memory>
#include <mutex>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace mowgli_localization
{

class GpsSlamCorrectorNode : public rclcpp::Node
{
public:
  GpsSlamCorrectorNode() : Node("gps_slam_corrector")
  {
    declare_parameter<double>("alpha", 0.02);
    declare_parameter<double>("max_correction_rate", 0.05);
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<std::string>("slam_frame", "slam_map");
    declare_parameter<std::string>("base_frame", "base_footprint");
    declare_parameter<double>("publish_rate", 20.0);
    declare_parameter<double>("gps_noise_threshold", 2.0);
    // Dock position as initial GPS datum (from mowgli_robot.yaml).
    // If both are 0.0, falls back to first GPS fix as datum.
    declare_parameter<double>("datum_lat", 0.0);
    declare_parameter<double>("datum_lon", 0.0);
    // GPS antenna offset from base_footprint (from mowgli_robot.yaml gps_x/y).
    // Used to compensate the heading-dependent error when comparing
    // antenna position (GPS) with base_footprint position (SLAM).
    declare_parameter<double>("gps_x", 0.0);
    declare_parameter<double>("gps_y", 0.0);

    alpha_ = get_parameter("alpha").as_double();
    max_correction_rate_ = get_parameter("max_correction_rate").as_double();
    map_frame_ = get_parameter("map_frame").as_string();
    slam_frame_ = get_parameter("slam_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    publish_rate_ = get_parameter("publish_rate").as_double();
    gps_noise_threshold_ = get_parameter("gps_noise_threshold").as_double();
    gps_lever_x_ = get_parameter("gps_x").as_double();
    gps_lever_y_ = get_parameter("gps_y").as_double();

    // Use dock position as datum if configured
    double cfg_lat = get_parameter("datum_lat").as_double();
    double cfg_lon = get_parameter("datum_lon").as_double();
    if (cfg_lat != 0.0 && cfg_lon != 0.0)
    {
      datum_lat_ = cfg_lat;
      datum_lon_ = cfg_lon;
      cos_datum_lat_ = std::cos(datum_lat_ * M_PI / 180.0);
      datum_set_ = true;
      RCLCPP_INFO(get_logger(),
                  "GPS datum from config (dock): lat=%.7f lon=%.7f",
                  datum_lat_,
                  datum_lon_);
    }

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        "/gps/fix",
        rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
        {
          on_gps_fix(msg);
        });

    publish_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / publish_rate_),
                                       [this]()
                                       {
                                         publish_correction();
                                         // SLAM heading injection disabled: creates feedback loop.
                                         // SLAM already corrects heading via slam_map→odom TF.
                                         // Nav2 uses map→base_footprint which includes SLAM
                                         // correction. publish_slam_heading();
                                       });

    // SLAM heading publisher (currently disabled — see publish_slam_heading()).
    // hardware_bridge publishes dock heading on ~/dock_heading → /gnss/heading
    // while charging. This publisher is reserved for future SLAM heading
    // injection if needed — uses a distinct topic to avoid dual-publisher race.
    heading_pub_ = create_publisher<sensor_msgs::msg::Imu>("~/slam_heading", rclcpp::QoS(10));

    RCLCPP_INFO(get_logger(),
                "GPS-SLAM corrector: %s→%s, alpha=%.3f, max_rate=%.3f m/s, "
                "SLAM heading → ~/slam_heading (disabled)",
                map_frame_.c_str(),
                slam_frame_.c_str(),
                alpha_,
                max_correction_rate_);
  }

private:
  static constexpr double METERS_PER_DEG = 111320.0;

  // ─── GPS callback ─────────────────────────────────────────────────────────

  void on_gps_fix(sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
  {
    if (msg->status.status < 0)
      return;

    // Set datum on first valid fix
    if (!datum_set_)
    {
      datum_lat_ = msg->latitude;
      datum_lon_ = msg->longitude;
      cos_datum_lat_ = std::cos(datum_lat_ * M_PI / 180.0);
      datum_set_ = true;
      RCLCPP_INFO(get_logger(), "GPS datum set: lat=%.7f lon=%.7f", datum_lat_, datum_lon_);
      return;
    }

    // Convert to ENU (antenna position)
    double gps_east = (msg->longitude - datum_lon_) * cos_datum_lat_ * METERS_PER_DEG;
    double gps_north = (msg->latitude - datum_lat_) * METERS_PER_DEG;

    // Compensate for antenna lever arm: GPS reports antenna position, but
    // we need base_footprint position. Rotate the lever arm by the robot's
    // current heading and subtract from the GPS ENU position.
    if (gps_lever_x_ != 0.0 || gps_lever_y_ != 0.0)
    {
      geometry_msgs::msg::TransformStamped odom_to_base;
      try
      {
        odom_to_base =
            tf_buffer_->lookupTransform("odom", base_frame_, tf2::TimePointZero);
        double yaw = tf2::getYaw(odom_to_base.transform.rotation);
        // Also need slam_map→odom to get heading in map frame
        geometry_msgs::msg::TransformStamped slam_to_odom;
        slam_to_odom =
            tf_buffer_->lookupTransform(slam_frame_, "odom", tf2::TimePointZero);
        double slam_yaw = tf2::getYaw(slam_to_odom.transform.rotation);
        double total_yaw = yaw + slam_yaw;
        // Add current correction to get approximate heading in map frame
        double cos_h = std::cos(total_yaw);
        double sin_h = std::sin(total_yaw);
        gps_east -= gps_lever_x_ * cos_h - gps_lever_y_ * sin_h;
        gps_north -= gps_lever_x_ * sin_h + gps_lever_y_ * cos_h;
      }
      catch (const tf2::TransformException&)
      {
        // TF not yet available — skip lever arm compensation this cycle
      }
    }

    // Weight by NavSatFix status:
    //   -1 = no fix       → 0 (already rejected above)
    //    0 = basic GPS     → 0.01 (barely nudge — covariance lies)
    //    1 = SBAS/DGPS     → 0.1
    //    2 = GBAS/RTK      → 1.0 (trust fully)
    double weight;
    switch (msg->status.status)
    {
      case 2:
        weight = 1.0;
        break;  // RTK
      case 1:
        weight = 0.1;
        break;  // DGPS
      default:
        weight = 0.01;
        break;  // Basic GPS
    }

    // Look up where SLAM thinks the robot is in slam_map frame
    geometry_msgs::msg::TransformStamped slam_to_base;
    try
    {
      slam_to_base = tf_buffer_->lookupTransform(slam_frame_, base_frame_, tf2::TimePointZero);
    }
    catch (const tf2::TransformException& ex)
    {
      return;  // TF not yet available
    }

    double slam_x = slam_to_base.transform.translation.x;
    double slam_y = slam_to_base.transform.translation.y;

    // The correction: where GPS says the robot is minus where SLAM says
    // map→slam_map = gps_position - slam_position (offset)
    // But we need to account for the existing correction:
    //   robot_in_map = correction + robot_in_slam
    //   gps = correction + slam → correction = gps - slam
    double target_x = gps_east - slam_x;
    double target_y = gps_north - slam_y;

    // Re-read tuning params (allows runtime adjustment via ros2 param set)
    {
      std::lock_guard<std::mutex> lock(params_mutex_);
      alpha_ = get_parameter("alpha").as_double();
      max_correction_rate_ = get_parameter("max_correction_rate").as_double();
    }

    // Low-pass filter with GPS quality weighting
    double effective_alpha = alpha_ * weight;
    correction_x_ += effective_alpha * (target_x - correction_x_);
    correction_y_ += effective_alpha * (target_y - correction_y_);

    has_correction_ = true;
    last_gps_time_ = now();

    RCLCPP_DEBUG(get_logger(),
                 "GPS=(%.2f,%.2f) SLAM=(%.2f,%.2f) corr=(%.2f,%.2f) status=%d w=%.3f",
                 gps_east,
                 gps_north,
                 slam_x,
                 slam_y,
                 correction_x_,
                 correction_y_,
                 msg->status.status,
                 weight);
  }

  // ─── Publish map→slam_map TF ──────────────────────────────────────────────

  void publish_correction()
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now();
    t.header.frame_id = map_frame_;
    t.child_frame_id = slam_frame_;

    if (has_correction_)
    {
      // Rate-limit the correction change to avoid jumps
      double dt = 1.0 / publish_rate_;
      double dx = correction_x_ - published_x_;
      double dy = correction_y_ - published_y_;
      double dist = std::sqrt(dx * dx + dy * dy);
      double max_step;
      {
        std::lock_guard<std::mutex> lock(params_mutex_);
        max_step = max_correction_rate_ * dt;
      }

      if (dist > max_step && dist > 1e-6)
      {
        double scale = max_step / dist;
        published_x_ += dx * scale;
        published_y_ += dy * scale;
      }
      else
      {
        published_x_ = correction_x_;
        published_y_ = correction_y_;
      }
    }

    t.transform.translation.x = published_x_;
    t.transform.translation.y = published_y_;
    t.transform.translation.z = 0.0;
    // No rotation correction — SLAM heading from scan matching is trusted.
    // GPS doesn't provide heading; dock_pose_yaw aligns SLAM at startup.
    t.transform.rotation.w = 1.0;
    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = 0.0;

    tf_broadcaster_->sendTransform(t);
  }

  // ─── Publish SLAM heading to FusionCore ────────────────────────────────────
  // DISABLED — creates feedback loop. Kept for reference.
  // Reads map→base_footprint TF (SLAM + FusionCore combined), extracts yaw,
  // publishes on ~/slam_heading. If re-enabled, would need remapping to
  // /gnss/heading in the launch file (currently used by hardware_bridge
  // dock heading only).

  void publish_slam_heading()
  {
    geometry_msgs::msg::TransformStamped tf;
    try
    {
      tf = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
    }
    catch (const tf2::TransformException&)
    {
      return;
    }

    double yaw = tf2::getYaw(tf.transform.rotation);

    auto msg = sensor_msgs::msg::Imu{};
    msg.header.stamp = now();
    msg.header.frame_id = base_frame_;
    msg.orientation.z = std::sin(yaw / 2.0);
    msg.orientation.w = std::cos(yaw / 2.0);
    // Loose yaw covariance so FusionCore's Mahalanobis gate accepts large
    // corrections. With 1.0 rad² and threshold 100, accepts up to ~10 rad
    // (any angle). The heading value is correct — we just need the gate open.
    msg.orientation_covariance[0] = 0.01;  // roll
    msg.orientation_covariance[4] = 0.01;  // pitch
    msg.orientation_covariance[8] = 1.0;  // yaw — loose to pass outlier gate

    heading_pub_->publish(msg);
  }

  // ─── Parameters ────────────────────────────────────────────────────────────

  std::mutex params_mutex_;  // Guards alpha_ and max_correction_rate_
  double alpha_;  // Low-pass filter coefficient (0-1)
  double max_correction_rate_;  // Max m/s the correction can move
  std::string map_frame_;
  std::string slam_frame_;
  std::string base_frame_;
  double publish_rate_;
  double gps_noise_threshold_;  // Sigma below which GPS gets full weight
  double gps_lever_x_ = 0.0;  // Antenna offset from base_footprint (m)
  double gps_lever_y_ = 0.0;

  // ─── Datum ─────────────────────────────────────────────────────────────────

  bool datum_set_ = false;
  double datum_lat_ = 0.0;
  double datum_lon_ = 0.0;
  double cos_datum_lat_ = 1.0;

  // ─── State ─────────────────────────────────────────────────────────────────

  bool has_correction_ = false;
  double correction_x_ = 0.0;  // Filtered correction
  double correction_y_ = 0.0;
  double published_x_ = 0.0;  // Rate-limited published value
  double published_y_ = 0.0;
  rclcpp::Time last_gps_time_{0, 0, RCL_ROS_TIME};

  // ─── ROS ───────────────────────────────────────────────────────────────────

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr heading_pub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace mowgli_localization

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mowgli_localization::GpsSlamCorrectorNode>());
  rclcpp::shutdown();
  return 0;
}

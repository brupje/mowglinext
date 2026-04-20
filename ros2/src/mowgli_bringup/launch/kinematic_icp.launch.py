# Copyright 2026 Mowgli Project
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.


"""
kinematic_icp.launch.py

Starts the Kinematic-ICP LiDAR odometry pipeline for a 2D LD19 LiDAR:

  1. kinematic_icp_online_node (kinematic_icp pkg, upstream)
     - /scan (sensor_msgs/LaserScan) — native 2D input via use_2d_lidar=true
     - Motion prior: TF delta of base_footprint across the scan window in
       the odom frame. In Mowgli, FusionCore publishes odom -> base_footprint,
       so that IS the motion prior source.
     - Sensor extrinsic: static base_footprint -> lidar_link from URDF.
     -> /kinematic_icp/lidar_odometry (nav_msgs/Odometry)
     - publish_odom_tf: False (FusionCore owns odom -> base_footprint;
       Kinematic-ICP MUST NOT publish a competing TF)

     NOTE ON FEEDBACK: Upstream PRBonn assumes the wheel-odom TF publisher
     is a separate node from the downstream fusion filter. In Mowgli,
     FusionCore IS the fusion filter AND the TF publisher, so using its
     TF as Kinematic-ICP's motion prior creates a mild feedback loop
     (K-ICP output -> encoder2 -> FusionCore -> odom TF -> K-ICP prior).
     This is bounded by:
       * FusionCore's strong anchors (GPS, wheels, IMU) dominating over
         encoder2 (encoder2 covariance is intentionally loose)
       * FusionCore's Mahalanobis gate on encoder2
       * Kinematic-ICP's own kinematic regularization rejecting non-
         physical corrections
     If it becomes measurable in tests we will add a wheel-only TF
     publisher + scan-frame republish to fully decouple the sub-graphs.

  2. kinematic_icp_encoder_adapter (mowgli_localization)
     - /kinematic_icp/lidar_odometry  ->  finite-difference body-frame twist
     -> /encoder2/odom                     (FusionCore's encoder2 slot)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bringup_dir = get_package_share_directory("mowgli_bringup")
    kicp_config = os.path.join(bringup_dir, "config", "kinematic_icp.yaml")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock when true.",
    )
    use_sim_time = LaunchConfiguration("use_sim_time")

    # ------------------------------------------------------------------
    # 1. Kinematic-ICP online node.
    #    - input 'lidar_topic' = /scan (LaserScan, use_2d_lidar=true)
    #    - output remapped to /kinematic_icp/lidar_odometry
    #    - wheel_odom_frame = odom (FusionCore-owned), base_frame = base_footprint
    #    - publish_odom_tf: False (FusionCore owns odom -> base_footprint)
    # ------------------------------------------------------------------
    kinematic_icp_node = Node(
        package="kinematic_icp",
        executable="kinematic_icp_online_node",
        name="kinematic_icp_online_node",
        namespace="kinematic_icp",
        output="screen",
        parameters=[
            kicp_config,
            {
                "use_sim_time": use_sim_time,
                "lidar_topic": "/scan",
                "use_2d_lidar": True,
                "wheel_odom_frame": "odom",
                "base_frame": "base_footprint",
                # Kinematic-ICP publishes lidar_odom_kicp -> base_footprint
                # internally only; this frame is not consumed by anything else.
                "lidar_odom_frame": "lidar_odom_kicp",
                "publish_odom_tf": False,
                "invert_odom_tf": False,
                "tf_timeout": 0.0,
            },
        ],
        remappings=[
            ("lidar_odometry", "/kinematic_icp/lidar_odometry"),
        ],
    )

    # ------------------------------------------------------------------
    # 2. Encoder-twist adapter: pose-only KICP odometry -> FusionCore
    #    encoder2 twist.
    # ------------------------------------------------------------------
    kicp_encoder_adapter = Node(
        package="mowgli_localization",
        executable="kinematic_icp_encoder_adapter.py",
        name="kinematic_icp_encoder_adapter",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "input_topic": "/kinematic_icp/lidar_odometry",
                "output_topic": "/encoder2/odom",
            }
        ],
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            kinematic_icp_node,
            kicp_encoder_adapter,
        ]
    )

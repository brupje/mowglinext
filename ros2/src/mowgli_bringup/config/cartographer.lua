-- Cartographer configuration for MowgliNext robot mower.
--
-- Stance: DRIFT OVERLAY ON FUSIONCORE ODOM (not primary SLAM).
--   FusionCore is GPS-RTK-anchored (σ ≈ 5 mm) at 100 Hz — it's the
--   primary localizer. Cartographer only scan-matches the LiDAR to
--   correct short-term drift in the map→odom TF. Global pose graph
--   optimization is DISABLED; loop closure is OFF. This prevents
--   the symmetric-submap 180° flip seen with sparse outdoor features
--   (hedges, fences), where the pose graph's global optimizer accepts
--   near-mirror constraints and rotates the frame.
--
-- Architecture:
--   map → odom → base_footprint
--   Cartographer publishes map→odom.
--   FusionCore publishes odom→base_footprint (GPS+IMU+wheels fused).
--   GPS-SLAM corrector is not used.
--
-- Tuned for outdoor garden with:
--   - LD19 LiDAR (10 Hz, 455 points per sweep, 6-8m effective outdoor)
--   - FusionCore /fusion/odom @ 100 Hz (GPS RTK Fixed)
--   - Low feature density (hedges, fences, trees, grass)
--
-- Sources:
--   - https://google-cartographer-ros.readthedocs.io/en/latest/tuning.html
--   - https://google-cartographer-ros.readthedocs.io/en/latest/algo_walkthrough.html
--   - https://github.com/cartographer-project/cartographer_ros/issues/770

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_footprint",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,        -- FusionCore owns odom→base_footprint
  publish_frame_projected_to_2d = true,
  use_odometry = true,               -- FusionCore's /fusion/odom (GPS+IMU+wheels)
  use_nav_sat = false,               -- GPS already in FusionCore odometry
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.5,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 0.05,    -- 20 Hz TF publish
  trajectory_publish_period_sec = 0.1,
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
}

-- 2D trajectory builder for outdoor mowing
MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 2  -- ARM has limited cores

-- Disable IMU input to Cartographer: it asserts the IMU frame must be
-- colocated with tracking_frame (base_footprint), which our imu_link
-- is not. FusionCore's /fusion/odom already has IMU fused in — Cartographer
-- only needs LiDAR + odometry for 2D scan matching on flat ground.
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.min_range = 0.20            -- filter grass near robot
TRAJECTORY_BUILDER_2D.max_range = 8.0             -- LD19 effective outdoor range
-- Match max_range: non-returns in grass/sky still clear free space.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 6.0
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

-- Larger submaps (60 → 160) accumulate enough features to disambiguate
-- orientation — small submaps in sparse hedgerows look identical flipped.
-- At 10 Hz LiDAR, 160 scans = 16s of data per submap.
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 160
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05

-- Disable real-time correlative scan matcher: redundant when odometry is
-- GPS-RTK grade (FusionCore σ ≈ 5 mm). The coarse search introduces yaw
-- jitter on sparse outdoor scans without improving accuracy.
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false

-- Ceres scan matcher (fine tuning). Weights bumped 10x (10→100, 40→400)
-- so the Ceres solver trusts the FusionCore odom prior strongly and only
-- nudges the pose to match the LiDAR scan — rather than fighting the prior
-- when scans are symmetric.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 100.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 400.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1.0

-- Motion filter — insert a new node whenever any threshold is crossed.
-- max_angle_radians dropped 15° → 0.5° to keep the local graph fed with
-- new nodes even during slow turns. Upstream default is 1°; 15° was an
-- outlier that starved the graph during stationary / slow-rotation phases.
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.5
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.2
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.5)

-- Global pose graph optimization DISABLED.
-- optimize_every_n_nodes = 0 turns off global SLAM entirely. We don't need
-- loop closure because FusionCore's GPS-RTK odometry is already globally
-- anchored (~5 mm σ). Cartographer acts as a drift overlay on odom.
POSE_GRAPH.optimize_every_n_nodes = 0

-- Lower huber_scale: downweights outlier constraints (spurious matches from
-- symmetric hedges). Upstream default is 10; 5 is more robust for sparse
-- outdoor environments. Still applies to any local constraints if
-- optimize_every_n_nodes is re-enabled in the future.
POSE_GRAPH.optimization_problem.huber_scale = 5e0

-- Constraint builder (loop closure). With optimize_every_n_nodes=0 these
-- don't fire, but we tighten them defensively for the case we ever re-enable:
--   - min_score 0.55 → 0.65: stricter gate, rejects near-mirror matches.
--   - max_constraint_distance 15 → 6: matches LD19 effective range; beyond
--     this constraints are dominated by noise.
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.constraint_builder.max_constraint_distance = 6.0
POSE_GRAPH.constraint_builder.log_matches = true

-- Fast correlative scan matcher for loop closure (dormant while disabled)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 3.0
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(30.0)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 7

-- Ceres scan matcher for loop closure refinement (dormant while disabled)
POSE_GRAPH.constraint_builder.ceres_scan_matcher.translation_weight = 10.0
POSE_GRAPH.constraint_builder.ceres_scan_matcher.rotation_weight = 1.0

-- Odometry / local-SLAM trust weights (1e5 each) — keep Cartographer
-- faithful to FusionCore's pose. These dominate over the Ceres matcher
-- weights above, so the final trajectory is dictated by odom with minor
-- scan-matching corrections.
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e5
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e5
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e5

return options

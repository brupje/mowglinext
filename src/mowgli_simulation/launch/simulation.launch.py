"""
simulation.launch.py

Full simulation launch file for the Mowgli robot mower using Gazebo Ignition Fortress.

Launch sequence:
  1. Declare arguments (world, use_rviz, headless, spawn_x/y/z/yaw).
  2. Launch Gazebo Ignition with the selected world SDF.
  3. Publish /robot_description via robot_state_publisher (from mowgli_bringup URDF).
  4. Spawn the mowgli_mower Gazebo model at the docking station position.
  5. Start ros_gz_bridge with the YAML bridge configuration.
  6. Optionally start RViz2 with the simulation config.

The mower spawns from the standalone Gazebo model (models/mowgli_mower/model.sdf)
so Gazebo handles its own physics/sensors, while robot_state_publisher provides
/robot_description for RViz visualisation and TF from the URDF.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    # ------------------------------------------------------------------
    # Package share directories
    # ------------------------------------------------------------------
    sim_share = get_package_share_directory("mowgli_simulation")
    bringup_share = get_package_share_directory("mowgli_bringup")
    gz_sim_share = get_package_share_directory("ros_gz_sim")

    # ------------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------------
    world_arg = DeclareLaunchArgument(
        "world",
        default_value="garden",
        description=(
            "Gazebo world to load. Either 'garden' or 'empty_garden' (selects the "
            "matching SDF from the mowgli_simulation worlds/ directory), or an "
            "absolute path to a custom SDF file."
        ),
    )

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Launch RViz2 with the mowgli_sim.rviz configuration.",
    )

    headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="Run Gazebo in headless mode (no GUI). Useful for CI/testing.",
    )

    spawn_x_arg = DeclareLaunchArgument(
        "spawn_x",
        default_value="0.0",
        description="Robot spawn X position (metres).",
    )

    spawn_y_arg = DeclareLaunchArgument(
        "spawn_y",
        default_value="0.0",
        description="Robot spawn Y position (metres).",
    )

    spawn_z_arg = DeclareLaunchArgument(
        "spawn_z",
        default_value="0.05",
        description="Robot spawn Z position (metres). Slightly above ground.",
    )

    spawn_yaw_arg = DeclareLaunchArgument(
        "spawn_yaw",
        default_value="0.0",
        description="Robot spawn yaw angle (radians).",
    )

    # ------------------------------------------------------------------
    # LaunchConfiguration handles
    # ------------------------------------------------------------------
    world = LaunchConfiguration("world")
    use_rviz = LaunchConfiguration("use_rviz")
    headless = LaunchConfiguration("headless")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_yaw = LaunchConfiguration("spawn_yaw")

    # ------------------------------------------------------------------
    # World SDF path — resolve name → absolute path
    # PathJoinSubstitution handles both a bare name and an absolute path
    # gracefully when the caller passes a full path.
    # ------------------------------------------------------------------
    world_sdf = PathJoinSubstitution(
        [FindPackageShare("mowgli_simulation"), "worlds", [world, ".sdf"]]
    )

    # ------------------------------------------------------------------
    # 1. Gazebo Ignition
    #    -r  : run simulation on start
    #    -v4 : verbose level 4 (info)
    #    The conditional flag --headless-rendering is appended for CI.
    # ------------------------------------------------------------------
    gz_args_with_gui = ["-r -v3 ", world_sdf]
    gz_args_headless = ["-r -v3 -s ", world_sdf]

    gazebo_with_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": gz_args_with_gui}.items(),
        condition=UnlessCondition(headless),
    )

    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": gz_args_headless}.items(),
        condition=IfCondition(headless),
    )

    # ------------------------------------------------------------------
    # 2. robot_state_publisher — publishes /robot_description and static TF
    #    Uses the mowgli URDF from mowgli_bringup for RViz display.
    # ------------------------------------------------------------------
    urdf_xacro = os.path.join(bringup_share, "urdf", "mowgli.urdf.xacro")

    robot_description_content = Command(
        [FindExecutable(name="xacro"), " ", urdf_xacro]
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description_content},
            {"use_sim_time": True},
        ],
    )

    # ------------------------------------------------------------------
    # 3. Spawn the mowgli_mower Gazebo model
    #    Uses the standalone SDF model (not the URDF) so Gazebo plugins
    #    (diff-drive, LiDAR, IMU) are fully functional.
    # ------------------------------------------------------------------
    mower_sdf = os.path.join(
        sim_share, "models", "mowgli_mower", "model.sdf"
    )

    spawn_mower_node = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_mowgli",
        output="screen",
        arguments=[
            "-name", "mowgli_mower",
            "-file", mower_sdf,
            "-x", spawn_x,
            "-y", spawn_y,
            "-z", spawn_z,
            "-Y", spawn_yaw,
        ],
        parameters=[{"use_sim_time": True}],
    )

    # ------------------------------------------------------------------
    # 4. ros_gz_bridge — topic bridging (YAML parameter file)
    # ------------------------------------------------------------------
    bridge_config = os.path.join(sim_share, "config", "gazebo_bridge.yaml")

    bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_ros2_bridge",
        output="screen",
        parameters=[
            {"config_file": bridge_config},
            {"use_sim_time": True},
        ],
    )

    # ------------------------------------------------------------------
    # 5. RViz2 — optional, started after spawner exits
    # ------------------------------------------------------------------
    rviz_config = os.path.join(sim_share, "rviz", "mowgli_sim.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_rviz),
    )

    # Start RViz only after the mower has been successfully spawned
    rviz_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_mower_node,
            on_exit=[rviz_node],
        )
    )

    # ------------------------------------------------------------------
    # LaunchDescription
    # ------------------------------------------------------------------
    return LaunchDescription(
        [
            # Arguments
            world_arg,
            use_rviz_arg,
            headless_arg,
            spawn_x_arg,
            spawn_y_arg,
            spawn_z_arg,
            spawn_yaw_arg,
            # Nodes / includes
            gazebo_with_gui,
            gazebo_headless,
            robot_state_publisher_node,
            spawn_mower_node,
            bridge_node,
            rviz_after_spawn,
        ]
    )

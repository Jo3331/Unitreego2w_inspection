from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    go2_driver_pkg = get_package_share_directory("go2_driver")
    go2_core_pkg = get_package_share_directory("go2_core")
    go2_slam_pkg = get_package_share_directory("go2_slam")
    go2_perception_pkg = get_package_share_directory("go2_perception")

    use_rviz = DeclareLaunchArgument(
        name="main_use_rviz",
        default_value="true"
    )

    use_description = DeclareLaunchArgument(
        name="use_description",
        default_value="true"
    )

    external_odom_topic = DeclareLaunchArgument(
        name="external_odom_topic",
        default_value="/SMX/Odom_2D"
    )

    pointcloud_topic = DeclareLaunchArgument(
        name="pointcloud_topic",
        default_value="/utlidar/cloud_synced"
    )

    scan_yaw_offset_rad = DeclareLaunchArgument(
        name="scan_yaw_offset_rad",
        default_value="3.1415926"
    )

    go2_robot_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(go2_core_pkg, "launch", "go2_robot_localization.launch.py")
        )
    )

    go2_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(go2_driver_pkg, "launch", "driver.launch.py")
        ),
        launch_arguments=[
            ("use_rviz", "false"),
            ("use_description", LaunchConfiguration("use_description")),
            ("external_odom_topic", LaunchConfiguration("external_odom_topic")),
        ]
    )

    go2_pointcloud_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(go2_perception_pkg, "launch", "go2_pointcloud.launch.py")
        ),
        launch_arguments=[
            ("pointcloud_topic", LaunchConfiguration("pointcloud_topic")),
            ("scan_target_frame", "base_footprint"),
            ("use_cloud_accumulation", "true"),
            ("accumulation_max_clouds", "3"),
            ("accumulation_publish_period_ms", "100"),
            ("accumulation_min_height", "-10.0"),
            ("accumulation_max_height", "10.0"),
            ("scan_transform_tolerance", "0.01"),
            ("use_latest_transform", "true"),
            ("output_now_stamp", "true"),
            ("publish_debug_cloud", "false"),
            ("publish_projected_debug_cloud", "false"),
            ("remove_tilt", "true"),
            ("height_relative_to_sensor", "true"),
            ("near_sensor_exclusion_radius", "0.45"),
            ("near_sensor_exclusion_min_z", "0.12"),
            ("effective_range_max", "10.0"),
            ("keep_farthest_per_angle", "true"),
            ("scan_yaw_offset_rad", LaunchConfiguration("scan_yaw_offset_rad")),
            ("scan_min_height", "0.02"),
            ("scan_max_height", "0.60"),
            ("scan_time", "0.25"),
            ("scan_range_min", "0.3"),
            ("scan_range_max", "10.0"),
        ]
    )

    go2_slamtoolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(go2_slam_pkg, "launch", "go2_slamtoolbox.launch.py")
        ),
        launch_arguments=[
            (
                "slam_params_file",
                os.path.join(go2_slam_pkg, "config", "mapper_params_toolbox_compat.yaml")
            )
        ]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d", os.path.join(go2_core_pkg, "rviz2", "display.rviz"),
            "--ros-args", "--log-level", "warn"
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("main_use_rviz"))
    )

    return LaunchDescription([
        use_rviz,
        use_description,
        external_odom_topic,
        pointcloud_topic,
        scan_yaw_offset_rad,
        go2_driver_launch,
        go2_robot_localization,
        go2_pointcloud_launch,
        go2_slamtoolbox_launch,
        rviz_node,
    ])
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    go2_core_pkg = get_package_share_directory("go2_core")
    go2_description_pkg = get_package_share_directory("go2_description")
    go2_perception_pkg = get_package_share_directory("go2_perception")
    go2_slam_pkg = get_package_share_directory("go2_slam")
    slam_toolbox_pkg = get_package_share_directory("slam_toolbox")

    pointcloud_topic = DeclareLaunchArgument(
        name="pointcloud_topic",
        default_value="/utlidar/cloud_synced"
    )

    use_rviz = DeclareLaunchArgument(
        name="use_rviz",
        default_value="true"
    )

    go2_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(go2_description_pkg, "launch", "display.launch.py")
        ),
        launch_arguments=[("use_joint_state_publisher", "false")]
    )

    twist_bridge = Node(
        package="go2_twist_bridge",
        executable="twist_bridge"
    )

    static_odom_to_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"]
    )

    go2_pointcloud_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(go2_perception_pkg, "launch", "go2_pointcloud.launch.py")
        ),
        launch_arguments=[
            ("pointcloud_topic", LaunchConfiguration("pointcloud_topic")),
            ("scan_target_frame", "base_link"),
        ]
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_pkg, "launch", "online_async_launch.py")
        ),
        launch_arguments=[
            (
                "slam_params_file",
                os.path.join(go2_slam_pkg, "config", "mapper_params_online_async_pure_scan.yaml"),
            ),
            ("use_sim_time", "false"),
        ]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(go2_core_pkg, "rviz2", "display.rviz")],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_rviz"))
    )

    return LaunchDescription([
        pointcloud_topic,
        use_rviz,
        go2_description_launch,
        twist_bridge,
        static_odom_to_base,
        go2_pointcloud_launch,
        slam_toolbox_launch,
        rviz_node,
    ])
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
import os

def generate_launch_description():

    #获取各功能包
    go2_driver_pkg = get_package_share_directory("go2_driver")
    go2_core_pkg = get_package_share_directory("go2_core")
    go2_slam_pkg = get_package_share_directory("go2_slam")
    go2_perception_pkg = get_package_share_directory("go2_perception")
    
    # 添加启动开关
    use_slamtoolbox = DeclareLaunchArgument(
        name="use_slamtoolbox",
        default_value="true"
    )

    pointcloud_topic = DeclareLaunchArgument(
        name="pointcloud_topic",
        default_value="/utlidar/cloud_synced"
    )

    use_cloud_accumulation = DeclareLaunchArgument(
        name="use_cloud_accumulation",
        default_value="true"
    )

    accumulation_max_clouds = DeclareLaunchArgument(
        name="accumulation_max_clouds",
        default_value="10"  # 原 "3": 累积3帧导致scan时戳滞后~300ms漂移, 改1帧消除
    )

    accumulation_publish_period_ms = DeclareLaunchArgument(
        name="accumulation_publish_period_ms",
        default_value="100"  # 原 100ms，降低发布频率减轻下游压力
    )

    accumulation_max_cloud_age_sec = DeclareLaunchArgument(
        name="accumulation_max_cloud_age_sec",
        default_value="2.0"
    )

    remove_tilt = DeclareLaunchArgument(
        name="remove_tilt",
        default_value="false"
    )

    use_description = DeclareLaunchArgument(
        name="use_description",
        default_value="true"
    )

    use_main_rviz = DeclareLaunchArgument(
        name="use_main_rviz",
        default_value="true"
    )

    external_odom_topic = DeclareLaunchArgument(
        name="external_odom_topic",
        default_value="/SMX/Odom_2D"
    )

    # 里程计融合imu
    go2_robot_localization = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(go2_core_pkg, "launch", "go2_robot_localization.launch.py")
            )
        )

    # 启动驱动包
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

    # 点云处理
    go2_pointcloud_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(go2_perception_pkg, "launch", "go2_pointcloud.launch.py")
            ),
            launch_arguments=[
                ("pointcloud_topic", LaunchConfiguration("pointcloud_topic")),
                ("scan_target_frame", "base_footprint"),
                ("use_cloud_accumulation", LaunchConfiguration("use_cloud_accumulation")),
                ("accumulation_max_clouds", LaunchConfiguration("accumulation_max_clouds")),
                ("accumulation_publish_period_ms", LaunchConfiguration("accumulation_publish_period_ms")),
                ("accumulation_max_cloud_age_sec", LaunchConfiguration("accumulation_max_cloud_age_sec")),
                ("remove_tilt", LaunchConfiguration("remove_tilt")),
            ]
        )

    # slam-toolbox 配置
    go2_slamtoolbox_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(go2_slam_pkg, "launch", "go2_slamtoolbox.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration('use_slamtoolbox'))
        )

    # 包含rviz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(go2_core_pkg, "rviz2", "display.rviz")],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_main_rviz'))
    )

    return LaunchDescription([
        use_slamtoolbox,
        pointcloud_topic,
        use_cloud_accumulation,
        accumulation_max_clouds,
        accumulation_publish_period_ms,
        accumulation_max_cloud_age_sec,
        remove_tilt,
        use_description,
        external_odom_topic,
        use_main_rviz,
        go2_driver_launch,
        go2_robot_localization,
        go2_pointcloud_launch,
        go2_slamtoolbox_launch,
        rviz_node
    ])
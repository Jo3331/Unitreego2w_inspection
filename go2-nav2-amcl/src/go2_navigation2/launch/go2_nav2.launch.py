from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
import os
import launch

def generate_launch_description():
    get_nav2_pkg = get_package_share_directory("go2_navigation2")
    go2_description_pkg = get_package_share_directory("go2_description")
    go2_core_pkg = get_package_share_directory("go2_core")
    go2_driver_pkg = get_package_share_directory("go2_driver")

    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='false')
    default_map = os.path.join(get_nav2_pkg, 'maps', 'my_room.yaml')
    external_odom_topic = DeclareLaunchArgument(
        'external_odom_topic',
        default_value='/SMX/Odom_2D',
        description='外部 2D 里程计话题，用于驱动 /odom 与 odom->base_footprint TF',
    )
    declare_map = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='地图 yaml 路径（与 .pgm 同目录）；可改为 ~/go2_maps/xxx.yaml',
    )
    declare_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='是否启动 RViz2',
    )
    map_yaml_path = launch.substitutions.LaunchConfiguration('map')
    nav2_param_path = launch.substitutions.LaunchConfiguration(
        'params_file', default=os.path.join(get_nav2_pkg, 'config', 'nav2_params.yaml'))

    # --- map_server ---
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_yaml_path,
            'use_sim_time': use_sim_time,
            'topic_name': 'map'
        }]
    )

    # --- AMCL ---
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_param_path, {'use_sim_time': use_sim_time}]
    )

    # --- lifecycle_manager for localization ---
    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'bond_timeout': 10.0,
            'attempt_respawn_reconnection': True,
            'node_names': ['map_server', 'amcl']
        }]
    )

    # --- Nav2 navigation nodes (replaces nav2_bringup/navigation_launch.py) ---
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_param_path, {'use_sim_time': use_sim_time}]
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_param_path, {'use_sim_time': use_sim_time}]
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_param_path, {
            'use_sim_time': use_sim_time,
            'default_bt_xml_filename': os.path.join(
                get_nav2_pkg, 'config', 'nav2_bt_nav.xml'),
        }]
    )

    recoveries_server = Node(
        package='nav2_recoveries',
        executable='recoveries_server',
        name='recoveries_server',
        output='screen',
        parameters=[nav2_param_path, {'use_sim_time': use_sim_time}]
    )

    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'bond_timeout': 10.0,
            'attempt_respawn_reconnection': True,
            'node_names': ['controller_server', 'planner_server',
                           'recoveries_server', 'bt_navigator']
        }]
    )

    # 里程计融合imu
    go2_robot_localization = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(go2_core_pkg, "launch", "go2_robot_localization.launch.py")
            )
        )

    # 与 driver.launch 一致：cmd_vel -> Unitree API，以及 TF/关节/IMU
    twist_bridge = Node(
        package='go2_twist_bridge',
        executable='twist_bridge',
    )
    footprint_to_link = Node(
        package='go2_driver',
        executable='footprint_to_link',
    )
    go2_driver = Node(
        package='go2_driver',
        executable='driver',
        parameters=[
            os.path.join(go2_driver_pkg, 'params', 'driver.yaml'),
            {
                'external_odom_topic': LaunchConfiguration('external_odom_topic'),
                'odom_child_frame_id': 'base_footprint',
            },
        ],
    )
    lowstate_to_imu = Node(
        package='go2_driver',
        executable='lowstate_to_imu',
    )

    # 包含scan话题
    cloud_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            launch_file_path=os.path.join(
                get_package_share_directory("go2_perception"),
                "launch",
                "go2_pointcloud.launch.py",
            )
        ),
        launch_arguments=[
            ("pointcloud_topic", "/utlidar/cloud_synced"),
            ("scan_target_frame", "base_footprint"),
            ("use_cloud_accumulation", "true"),
            ("accumulation_max_clouds", "3"),
            ("accumulation_publish_period_ms", "100"),
            ("accumulation_max_cloud_age_sec", "2.0"),
            ("remove_tilt", "false"),
        ]
    )

    # 包含模型可视化（发布 /robot_description，供 PC 端 RViz2 使用）
    go2_display_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(go2_description_pkg, "launch", "display.launch.py")
            ),
            launch_arguments=[("use_joint_state_publisher", "false")]
        )

    # RViz2 可视化
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(get_nav2_pkg, 'rviz', 'nav2_foxy_default.rviz')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    return LaunchDescription([
        external_odom_topic,
        declare_map,
        declare_rviz,
        twist_bridge,
        footprint_to_link,
        go2_driver,
        lowstate_to_imu,
        map_server,
        amcl,
        lifecycle_manager_localization,
        controller_server,
        planner_server,
        bt_navigator,
        recoveries_server,
        lifecycle_manager_navigation,
        go2_robot_localization,
        cloud_launch,
        go2_display_launch,
        rviz_node
    ])

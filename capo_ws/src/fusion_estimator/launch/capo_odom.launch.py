from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('fusion_estimator')
    cfg = os.path.join(pkg, 'config.yaml')

    adapter = Node(
        package='fusion_estimator',
        executable='unitree_lowstate_adapter_node.py',
        name='unitree_lowstate_adapter_node',
        output='screen',
        parameters=[cfg],
    )

    estimator = Node(
        package='fusion_estimator',
        executable='fusion_estimator_node',
        name='fusion_estimator_node',
        output='screen',
        parameters=[cfg],
    )

    delayed_estimator = RegisterEventHandler(
        OnProcessStart(
            target_action=adapter,
            on_start=[estimator],
        )
    )

    return LaunchDescription([adapter, delayed_estimator])

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare('stm32_base_bridge'), 'config', 'bridge.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/stm32_motor'),
        DeclareLaunchArgument('config', default_value=config),
        Node(package='stm32_base_bridge', executable='bridge_node', name='stm32_base_bridge',
             output='screen', parameters=[LaunchConfiguration('config'),
                                         {'port': LaunchConfiguration('port')}]),
    ])

"""Offline serial simulator plus the real bridge/EKF/model. No real serial access."""
import tempfile
from pathlib import Path
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Unique path prevents separate demos from sharing a serial endpoint.
    port = str(Path(tempfile.mkdtemp(prefix='curiousrobot-demo-')) / 'mcu')
    executable = str(Path(get_package_prefix('stm32_base_bridge')) / 'lib' /
                     'stm32_base_bridge' / 'mock_mcu')
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false'),
        ExecuteProcess(cmd=[executable, '--link', port],
                       output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('stm32_robot_bringup'), 'launch', 'bringup.launch.py'])),
            launch_arguments={'port': port, 'lidar': 'false',
                              'rviz': LaunchConfiguration('rviz')}.items()),
    ])

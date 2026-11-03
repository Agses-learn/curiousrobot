"""Preview the model with placeholder wheel states and RViz."""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('curiousrobot_description'),
                'launch', 'description.launch.py'])),
            launch_arguments={'publish_joints': 'true', 'rviz': 'true'}.items()),
    ])

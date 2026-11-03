"""STL-06N via vendor ldlidar; mounting values are provisional."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare('stm32_robot_bringup'), 'config', 'lidar.yaml'])
    args = [DeclareLaunchArgument('port', default_value='/dev/stl06n'),
            DeclareLaunchArgument('config', default_value=config)]
    for key, value in [('x','0.0'), ('y','0.0'), ('z','0.10'),
                       ('roll','0.0'), ('pitch','0.0'), ('yaw','0.0')]:
        args.append(DeclareLaunchArgument(key, default_value=value,
                                         description='Measured pose relative to base_link'))
    tf_args = ['--frame-id', 'base_link', '--child-frame-id', 'laser']
    for key in ['x', 'y', 'z', 'roll', 'pitch', 'yaw']:
        tf_args.extend(['--' + key, LaunchConfiguration(key)])
    return LaunchDescription(args + [
        Node(package='ldlidar', executable='ldlidar', name='stl06n', output='screen',
             parameters=[LaunchConfiguration('config'), {'port_name': LaunchConfiguration('port')}]),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='lidar_mount', arguments=tf_args),
    ])

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    own = FindPackageShare('stm32_robot_bringup')
    bridge_config = PathJoinSubstitution([FindPackageShare('stm32_base_bridge'), 'config', 'bridge.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/stm32_motor'),
        DeclareLaunchArgument('bridge_config', default_value=bridge_config),
        DeclareLaunchArgument('ekf_config', default_value=PathJoinSubstitution([own, 'config', 'ekf.yaml'])),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('preview_joints', default_value='true',
                              description='Zero wheel positions for display; not encoder feedback'),
        DeclareLaunchArgument('lidar', default_value='false'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/stl06n'),
        DeclareLaunchArgument('lidar_x', default_value='0.0'),
        DeclareLaunchArgument('lidar_y', default_value='0.0'),
        DeclareLaunchArgument('lidar_z', default_value='0.10'),
        DeclareLaunchArgument('lidar_roll', default_value='0.0'),
        DeclareLaunchArgument('lidar_pitch', default_value='0.0'),
        DeclareLaunchArgument('lidar_yaw', default_value='0.0'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('curiousrobot_description'), 'launch', 'description.launch.py'])),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'publish_joints': LaunchConfiguration('preview_joints'),
                'rviz': LaunchConfiguration('rviz'),
            }.items()),
        Node(package='stm32_base_bridge', executable='bridge_node', name='stm32_base_bridge',
             output='screen', parameters=[
                 LaunchConfiguration('bridge_config'), {'port': LaunchConfiguration('port'),
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}]),
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
             output='screen', parameters=[LaunchConfiguration('ekf_config'),
                 {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}],
             remappings=[('odometry/filtered', '/odom')]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            own, 'launch', 'lidar.launch.py'])),
            condition=IfCondition(LaunchConfiguration('lidar')),
            launch_arguments={'port': LaunchConfiguration('lidar_port'),
                **{k: LaunchConfiguration('lidar_' + k) for k in ['x', 'y', 'z', 'roll', 'pitch', 'yaw']}}.items()),
    ])

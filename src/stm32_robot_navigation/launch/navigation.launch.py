"""AMCL and the required Nav2 servers for CuriousRobot, without docking hardware."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('stm32_robot_navigation')
    params = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    # Keep lifecycle membership explicit: generic Nav2 bringup also includes docking.
    servers = [
        ('nav2_controller', 'controller_server'),
        ('nav2_smoother', 'smoother_server'),
        ('nav2_planner', 'planner_server'),
        ('nav2_behaviors', 'behavior_server'),
        ('nav2_velocity_smoother', 'velocity_smoother'),
        ('nav2_collision_monitor', 'collision_monitor'),
        ('nav2_bt_navigator', 'bt_navigator'),
        ('nav2_waypoint_follower', 'waypoint_follower'),
    ]
    nodes = []
    for package, name in servers:
        remaps = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
        if name in ('controller_server', 'behavior_server', 'velocity_smoother'):
            remaps.append(('cmd_vel', 'cmd_vel_nav'))
        nodes.append(Node(package=package, executable=name, name=name,
                          output='screen', parameters=[params], remappings=remaps))
    nodes.append(Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{'autostart': ParameterValue(autostart, value_type=bool),
                     'node_names': [name for _, name in servers]}]))
    return LaunchDescription([
        DeclareLaunchArgument('map', description='Absolute path to the saved map YAML'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution(
            [share, 'config', 'navigation.yaml'])),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('nav2_bringup'), 'launch', 'localization_launch.py'])),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'use_sim_time': use_sim_time, 'params_file': params,
                'autostart': autostart, 'use_composition': 'False',
            }.items()),
        GroupAction([SetParameter('use_sim_time', ParameterValue(use_sim_time, value_type=bool)),
                     *nodes]),
        Node(package='rviz2', executable='rviz2', condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', PathJoinSubstitution([share, 'rviz', 'navigation.rviz'])],
             parameters=[{'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}]),
    ])


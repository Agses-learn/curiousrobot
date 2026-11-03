"""Publish CuriousRobot geometry; optional joint states are for preview only."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('curiousrobot_description')
    urdf = PathJoinSubstitution([share, 'urdf', 'robots', 'curiousrobot_4wd.urdf.xacro'])
    sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument('urdf', default_value=urdf),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('publish_joints', default_value='false',
                              description='Publish placeholder wheel states for preview only'),
        DeclareLaunchArgument('rviz', default_value='false'),
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name='curiousrobot_robot_state_publisher', output='screen',
            parameters=[{
                'use_sim_time': sim_time,
                'robot_description': ParameterValue(
                    Command([FindExecutable(name='xacro'), ' ', LaunchConfiguration('urdf')]),
                    value_type=str),
            }]),
        Node(
            package='joint_state_publisher', executable='joint_state_publisher',
            condition=IfCondition(LaunchConfiguration('publish_joints')),
            parameters=[{'use_sim_time': sim_time}]),
        Node(
            package='rviz2', executable='rviz2',
            condition=IfCondition(LaunchConfiguration('rviz')),
            arguments=['-d', PathJoinSubstitution([share, 'rviz', 'description.rviz'])],
            parameters=[{'use_sim_time': sim_time}]),
    ])

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('d1_planner_bridge')
    default_config = os.path.join(pkg_share, 'config', 'd1_bridge.yaml')

    config_file = LaunchConfiguration('config_file', default=default_config)
    pos_cmd_topic = LaunchConfiguration('pos_cmd_topic', default='/drone_0_planning/pos_cmd')
    odom_topic = LaunchConfiguration('odom_topic', default='/odom')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic', default='/command/cmd_twist')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file', default_value=default_config,
            description='Path to d1_bridge.yaml'),
        DeclareLaunchArgument(
            'pos_cmd_topic', default_value='/drone_0_planning/pos_cmd',
            description='EGO traj_server PositionCommand topic'),
        DeclareLaunchArgument(
            'odom_topic', default_value='/odom',
            description='Robot odometry in world frame'),
        DeclareLaunchArgument(
            'cmd_vel_topic', default_value='/command/cmd_twist',
            description='D1 cmd_twist topic (geometry_msgs/Twist; uses linear.x and angular.z only)'),

        Node(
            package='d1_planner_bridge',
            executable='d1_planner_bridge_node',
            name='d1_planner_bridge_node',
            output='screen',
            parameters=[
                config_file,
                {
                    'pos_cmd_topic': pos_cmd_topic,
                    'odom_topic': odom_topic,
                    'cmd_vel_topic': cmd_vel_topic,
                },
            ],
        ),
    ])

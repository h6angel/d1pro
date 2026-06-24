import os
import sys

_launch_dir = os.path.dirname(os.path.abspath(__file__))
if _launch_dir not in sys.path:
    sys.path.insert(0, _launch_dir)
from launch_log_utils import default_ego_log_dir, maybe_reexec_with_log

_DEFAULT_LOG_DIR = default_ego_log_dir(__file__)
maybe_reexec_with_log(
    'd1_planner_bridge', 'd1_planner_bridge', _DEFAULT_LOG_DIR, log_stem='d1_bridge')

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
    odom_topic = LaunchConfiguration('odom_topic', default='/ov_msckf/odomimu')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic', default='/command/cmd_twist')
    max_vx = LaunchConfiguration('max_vx', default='0.6')
    max_wz = LaunchConfiguration('max_wz', default='0.5')
    save_log = LaunchConfiguration('save_log', default='true')
    log_dir = LaunchConfiguration('log_dir', default=_DEFAULT_LOG_DIR)

    return LaunchDescription([
        DeclareLaunchArgument(
            'save_log', default_value=save_log,
            description='Default true: tee launch output to <workspace>/ego_log/ (save_log:=false to disable)'),
        DeclareLaunchArgument(
            'log_dir', default_value=log_dir,
            description='Log directory (default: <workspace>/ego_log, not colcon log/ or ~/.ros/log)'),
        DeclareLaunchArgument(
            'config_file', default_value=default_config,
            description='Path to d1_bridge.yaml'),
        DeclareLaunchArgument(
            'pos_cmd_topic', default_value='/drone_0_planning/pos_cmd',
            description='EGO traj_server PositionCommand topic'),
        DeclareLaunchArgument(
            'odom_topic', default_value='/ov_msckf/odomimu',
            description='Robot odometry in global frame'),
        DeclareLaunchArgument(
            'cmd_vel_topic', default_value='/command/cmd_twist',
            description='D1 cmd_twist topic (geometry_msgs/Twist; uses linear.x and angular.z only)'),
        DeclareLaunchArgument(
            'max_vx', default_value=max_vx,
            description='Max forward speed (m/s); match planner max_vel'),
        DeclareLaunchArgument(
            'max_wz', default_value=max_wz,
            description='Max angular speed (rad/s); match planner max_wz'),

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
                    'max_vx': max_vx,
                    'max_wz': max_wz,
                    'max_yaw_dot_ff': max_wz,
                    'max_wz_yaw_p': max_wz,
                    'min_turn_wz': max_wz,
                },
            ],
        ),
    ])

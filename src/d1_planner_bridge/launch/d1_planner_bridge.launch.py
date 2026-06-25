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

_ego_launch = os.path.join(get_package_share_directory('ego_planner'), 'launch')
if _ego_launch not in sys.path:
    sys.path.insert(0, _ego_launch)
from d1_robot_config import load_d1_robot_config

_d1 = load_d1_robot_config()
_topics = _d1['topics']
_limits = _d1['limits']


def generate_launch_description():
    pkg_share = get_package_share_directory('d1_planner_bridge')
    default_config = os.path.join(pkg_share, 'config', 'd1_bridge.yaml')

    config_file = LaunchConfiguration('config_file', default=default_config)
    pos_cmd_topic = LaunchConfiguration('pos_cmd_topic', default=_topics['pos_cmd'])
    odom_topic = LaunchConfiguration('odom_topic', default=_topics['odom'])
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic', default=_topics['cmd_vel'])
    max_vx = LaunchConfiguration('max_vx', default=str(_limits['max_vel']))
    max_wz = LaunchConfiguration('max_wz', default=str(_limits['max_wz']))
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
            description='Bridge tuning params (topics/limits default from ego_planner d1_robot.yaml)'),
        DeclareLaunchArgument(
            'pos_cmd_topic', default_value=pos_cmd_topic,
            description='EGO traj_server PositionCommand topic (default from d1_robot.yaml)'),
        DeclareLaunchArgument(
            'odom_topic', default_value=odom_topic,
            description='Robot odometry in global frame (default from d1_robot.yaml)'),
        DeclareLaunchArgument(
            'cmd_vel_topic', default_value=cmd_vel_topic,
            description='D1 cmd_twist topic (default from d1_robot.yaml)'),
        DeclareLaunchArgument(
            'max_vx', default_value=max_vx,
            description='Max forward speed (m/s); default from d1_robot.yaml'),
        DeclareLaunchArgument(
            'max_wz', default_value=max_wz,
            description='Max angular speed (rad/s); default from d1_robot.yaml'),

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

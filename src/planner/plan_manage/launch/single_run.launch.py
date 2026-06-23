"""
EGO planner + traj_server for D1 real robot (OpenVINS + RealSense depth mapping).

Prerequisites (separate terminals):
  ros2 launch ov_msckf d435i_openvins.launch.py
  ros2 launch d1_planner_bridge d1_planner_bridge.launch.py

Depth intrinsics: override cx/cy/fx/fy from
  ros2 topic echo /camera/camera/depth/camera_info --once
"""

import os
import sys

_launch_dir = os.path.dirname(os.path.abspath(__file__))
if _launch_dir not in sys.path:
    sys.path.insert(0, _launch_dir)
from launch_log_utils import default_ego_log_dir, maybe_reexec_with_log

_DEFAULT_LOG_DIR = default_ego_log_dir(__file__)
maybe_reexec_with_log('ego_planner', 'single_run', _DEFAULT_LOG_DIR)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    map_size_x = LaunchConfiguration('map_size_x', default=40.0)
    map_size_y = LaunchConfiguration('map_size_y', default=40.0)
    map_size_z = LaunchConfiguration('map_size_z', default=3.0)
    flight_type = LaunchConfiguration('flight_type', default='1')
    odom_topic = LaunchConfiguration('odom_topic', default='/ov_msckf/odomimu')
    pose_topic = LaunchConfiguration('pose_topic', default='/ov_msckf/pose_stamped')
    depth_topic = LaunchConfiguration(
        'depth_topic', default='/camera/camera/depth/image_rect_raw')
    pos_cmd_topic = LaunchConfiguration(
        'pos_cmd_topic', default='/drone_0_planning/pos_cmd')
    max_vel = LaunchConfiguration('max_vel', default='1.6')
    goal_reach_thresh = LaunchConfiguration('goal_reach_thresh', default='0.3')
    enable_tag_tracking = LaunchConfiguration('enable_tag_tracking', default='false')
    save_log = LaunchConfiguration('save_log', default='true')
    log_dir = LaunchConfiguration('log_dir', default=_DEFAULT_LOG_DIR)

    # D435i depth 640x360 (must match infra); from depth/camera_info on device
    cx = LaunchConfiguration('cx', default='323.168')
    cy = LaunchConfiguration('cy', default='180.403')
    fx = LaunchConfiguration('fx', default='320.244')
    fy = LaunchConfiguration('fy', default='320.244')

    advanced_param_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ego_planner'), 'launch', 'advanced_param.launch.py')),
        launch_arguments={
            'map_size_x_': map_size_x,
            'map_size_y_': map_size_y,
            'map_size_z_': map_size_z,
            'odometry_topic': odom_topic,
            'camera_pose_topic': pose_topic,
            'depth_topic': depth_topic,
            'map_frame_id': 'global',
            'cx': cx,
            'cy': cy,
            'fx': fx,
            'fy': fy,
            'max_vel': max_vel,
            'max_acc': str(2.0),
            'planning_horizon': str(7.5),
            'use_distinctive_trajs': 'False',
            'flight_type': flight_type,
            'goal_reach_thresh': goal_reach_thresh,
            'enable_tag_tracking': enable_tag_tracking,
            'thresh_replan_time': '2.5',
            'obstacles_inflation': '0.09',
            'optimization_dist0': '0.55',
            'lambda_fitness': '1.5',
            'point_num': str(1),
            'point0_x': str(5.0),
            'point0_y': str(0.0),
            'point0_z': str(0.0),
            'point1_x': str(-5.0),
            'point1_y': str(0.0),
            'point1_z': str(0.0),
            'point2_x': str(5.0),
            'point2_y': str(0.0),
            'point2_z': str(0.0),
            'point3_x': str(-5.0),
            'point3_y': str(0.0),
            'point3_z': str(0.0),
            'point4_x': str(5.0),
            'point4_y': str(0.0),
            'point4_z': str(0.0),
        }.items(),
    )

    traj_server_node = Node(
        package='ego_planner',
        executable='traj_server',
        name='drone_0_traj_server',
        output='screen',
        remappings=[
            ('position_cmd', pos_cmd_topic),
            ('planning/bspline', 'drone_0_planning/bspline'),
            ('planning/exec_bspline_path', 'drone_0_planning/exec_bspline_path'),
            ('odom', odom_topic),
        ],
        parameters=[
            {'traj_server/time_forward': 0.7},
            {'traj_server/use_odom_progress': True},
            {'traj_server/odom_lookahead_time': 0.5},
            {'traj_server/log_trace_period_ms': 500},
            {'traj_server/endpoint_approach_dist': 0.35},
            {'traj_server/endpoint_stop_dist': 0.08},
            {'traj_server/endpoint_vel_gain': 0.8},
            {'traj_server/endpoint_max_vel': 1.6},
        ],
    )

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('map_size_x', default_value=map_size_x))
    ld.add_action(DeclareLaunchArgument('map_size_y', default_value=map_size_y))
    ld.add_action(DeclareLaunchArgument('map_size_z', default_value=map_size_z))
    ld.add_action(DeclareLaunchArgument('odom_topic', default_value=odom_topic,
                                        description='VIO odom in global frame'))
    ld.add_action(DeclareLaunchArgument('pose_topic', default_value=pose_topic,
                                        description='Camera pose for depth sync'))
    ld.add_action(DeclareLaunchArgument('depth_topic', default_value=depth_topic))
    ld.add_action(DeclareLaunchArgument('pos_cmd_topic', default_value=pos_cmd_topic))
    ld.add_action(DeclareLaunchArgument('flight_type', default_value=flight_type))
    ld.add_action(DeclareLaunchArgument('max_vel', default_value=max_vel))
    ld.add_action(DeclareLaunchArgument('goal_reach_thresh', default_value=goal_reach_thresh))
    ld.add_action(DeclareLaunchArgument(
        'enable_tag_tracking', default_value=enable_tag_tracking,
        description='true: AprilTag tracking; false: RViz 2D Goal'))
    ld.add_action(DeclareLaunchArgument('save_log', default_value=save_log))
    ld.add_action(DeclareLaunchArgument('log_dir', default_value=log_dir))
    ld.add_action(DeclareLaunchArgument('cx', default_value=cx))
    ld.add_action(DeclareLaunchArgument('cy', default_value=cy))
    ld.add_action(DeclareLaunchArgument('fx', default_value=fx))
    ld.add_action(DeclareLaunchArgument('fy', default_value=fy))
    ld.add_action(advanced_param_include)
    ld.add_action(traj_server_node)

    return ld

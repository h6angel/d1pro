"""
EGO planner + traj_server for D1 real robot (OpenVINS + RealSense depth mapping).

Prerequisites (separate terminals):
  ros2 launch ov_msckf d435i_openvins.launch.py
  ros2 launch d1_planner_bridge d1_planner_bridge.launch.py

Topics / limits / planner tuning: config/d1_robot.yaml
Depth intrinsics override: ros2 topic echo /camera/camera/depth/camera_info --once

Logs: ./start_ego_stack.sh (ego_log/stack_*/*.log)
"""

import os
import sys

_launch_dir = os.path.dirname(os.path.abspath(__file__))
if _launch_dir not in sys.path:
    sys.path.insert(0, _launch_dir)
from d1_robot_config import load_d1_robot_config

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_d1 = load_d1_robot_config()
_topics = _d1['topics']
_limits = _d1['limits']
_camera = _d1['camera']
_planner = _d1['planner']


def generate_launch_description():
    map_size_x = LaunchConfiguration('map_size_x', default=str(_planner['map_size_x']))
    map_size_y = LaunchConfiguration('map_size_y', default=str(_planner['map_size_y']))
    map_size_z = LaunchConfiguration('map_size_z', default=str(_planner['map_size_z']))
    odom_topic = LaunchConfiguration('odom_topic', default=_topics['odom'])
    pose_topic = LaunchConfiguration('pose_topic', default=_topics['pose'])
    depth_topic = LaunchConfiguration('depth_topic', default=_topics['depth'])
    pos_cmd_topic = LaunchConfiguration('pos_cmd_topic', default=_topics['pos_cmd'])
    max_vel = LaunchConfiguration('max_vel', default=str(_limits['max_vel']))
    max_wz = LaunchConfiguration('max_wz', default=str(_limits['max_wz']))
    max_acc = LaunchConfiguration('max_acc', default=str(_limits['max_acc']))
    planning_horizon = LaunchConfiguration(
        'planning_horizon', default=str(_planner['planning_horizon']))
    goal_reach_thresh = LaunchConfiguration(
        'goal_reach_thresh', default=str(_planner['goal_reach_thresh']))
    thresh_replan_time = LaunchConfiguration(
        'thresh_replan_time', default=str(_planner['thresh_replan_time']))
    obstacles_inflation = LaunchConfiguration(
        'obstacles_inflation', default=str(_planner['obstacles_inflation']))
    optimization_dist0 = LaunchConfiguration(
        'optimization_dist0', default=str(_planner['optimization_dist0']))
    lambda_fitness = LaunchConfiguration(
        'lambda_fitness', default=str(_planner['lambda_fitness']))
    enable_tag_tracking = LaunchConfiguration('enable_tag_tracking', default='false')

    cx = LaunchConfiguration('cx', default=str(_camera['cx']))
    cy = LaunchConfiguration('cy', default=str(_camera['cy']))
    fx = LaunchConfiguration('fx', default=str(_camera['fx']))
    fy = LaunchConfiguration('fy', default=str(_camera['fy']))

    tag_pose_topic = LaunchConfiguration(
        'tag_pose_topic', default='/apriltag/target_pose_global')
    tag_detected_topic = LaunchConfiguration(
        'tag_detected_topic', default='/apriltag/target_detected')
    tag_stop_dist = LaunchConfiguration(
        'tag_stop_dist', default=str(_planner['tag_stop_dist']))
    tag_update_min_dist = LaunchConfiguration('tag_update_min_dist', default='0.08')
    tag_replan_min_period = LaunchConfiguration('tag_replan_min_period', default='0.5')

    ego_planner_node = Node(
        package='ego_planner',
        executable='ego_planner_node',
        name='drone_0_ego_planner_node',
        output='screen',
        remappings=[
            ('odom_world', odom_topic),
            ('planning/bspline', 'drone_0_planning/bspline'),
            ('goal_point', 'drone_0_plan_vis/goal_point'),
            ('global_list', 'drone_0_plan_vis/global_list'),
            ('init_list', 'drone_0_plan_vis/init_list'),
            ('optimal_list', 'drone_0_plan_vis/optimal_list'),
            ('a_star_list', 'drone_0_plan_vis/a_star_list'),
            # grid_map/odom: only used when grid_map/pose_type=2 (ODOMETRY depth sync)
            ('grid_map/odom', odom_topic),
            ('grid_map/pose', pose_topic),
            ('grid_map/depth', depth_topic),
            ('grid_map/occupancy_inflate', 'drone_0_grid/grid_map/occupancy_inflate'),
        ],
        parameters=[
            {'fsm/thresh_replan_time': thresh_replan_time},
            {'fsm/thresh_no_replan_meter': 1.0},
            {'fsm/odom_traj_mismatch_thresh': 0.22},
            {'fsm/thresh_goal_reach_meter': goal_reach_thresh},
            {'fsm/planning_horizon': planning_horizon},
            {'fsm/planning_horizen_time': 3.0},
            {'fsm/emergency_time': 1.0},
            {'fsm/global_replan_drift_thresh': 0.25},
            {'fsm/fail_safe': True},
            {'fsm/log_trace_period_ms': 500},
            {'fsm/gen_new_traj_max_failures': 8},
            {'fsm/gen_new_traj_backoff_base_sec': 0.25},
            {'fsm/gen_new_traj_backoff_max_sec': 2.0},
            {'fsm/enable_tag_tracking': enable_tag_tracking},
            {'fsm/tag_pose_topic': tag_pose_topic},
            {'fsm/tag_detected_topic': tag_detected_topic},
            {'fsm/tag_stop_dist': tag_stop_dist},
            {'fsm/tag_update_min_dist': tag_update_min_dist},
            {'fsm/tag_replan_min_period': tag_replan_min_period},
            {'grid_map/resolution': 0.1},
            {'grid_map/map_size_x': map_size_x},
            {'grid_map/map_size_y': map_size_y},
            {'grid_map/map_size_z': map_size_z},
            {'grid_map/local_update_range_x': 12.0},
            {'grid_map/local_update_range_y': 8.0},
            {'grid_map/local_update_range_z': 4.5},
            {'grid_map/obstacles_inflation': obstacles_inflation},
            {'grid_map/local_map_margin': 10},
            {'grid_map/ground_height': -0.01},
            {'grid_map/cx': cx},
            {'grid_map/cy': cy},
            {'grid_map/fx': fx},
            {'grid_map/fy': fy},
            {'grid_map/use_depth_filter': True},
            {'grid_map/depth_filter_tolerance': 0.15},
            {'grid_map/depth_filter_maxdist': 5.0},
            {'grid_map/depth_filter_mindist': 0.2},
            {'grid_map/depth_filter_margin': 2},
            {'grid_map/k_depth_scaling_factor': 1000.0},
            {'grid_map/skip_pixel': 2},
            {'grid_map/p_hit': 0.65},
            {'grid_map/p_miss': 0.35},
            {'grid_map/p_min': 0.12},
            {'grid_map/p_max': 0.90},
            {'grid_map/p_occ': 0.80},
            {'grid_map/min_ray_length': 0.1},
            {'grid_map/max_ray_length': 4.5},
            {'grid_map/virtual_ceil_height': 2.9},
            {'grid_map/visualization_truncate_height': 1.8},
            {'grid_map/show_occ_time': False},
            {'grid_map/pose_type': 1},
            {'grid_map/frame_id': _topics['map_frame_id']},
            {'grid_map/occ_confirm_frames': 3},
            {'grid_map/occ_clear_frames': 5},
            {'grid_map/use_fixed_publish_window': True},
            {'grid_map/map_vis_rate': 10.0},
            {'grid_map/odom_depth_timeout': 2.5},
            {'grid_map/ground_filter_enable': True},
            {'grid_map/ground_filter_margin': 0.12},
            {'grid_map/inflate_xy_only': True},
            {'grid_map/robot_footprint_radius': 0.35},
            {'manager/max_vel': max_vel},
            {'manager/max_acc': max_acc},
            {'manager/max_jerk': 4.0},
            {'manager/control_points_distance': 0.4},
            {'manager/feasibility_tolerance': 0.05},
            {'manager/planning_horizon': planning_horizon},
            {'manager/use_robot_z_planning': True},
            {'optimization/lambda_smooth': 1.0},
            {'optimization/lambda_collision': 0.5},
            {'optimization/lambda_feasibility': 0.1},
            {'optimization/lambda_fitness': lambda_fitness},
            {'optimization/dist0': optimization_dist0},
            {'optimization/max_vel': max_vel},
            {'optimization/max_acc': max_acc},
            {'bspline/limit_vel': max_vel},
            {'bspline/limit_acc': max_acc},
            {'bspline/limit_ratio': 1.1},
        ],
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
            {'traj_server/endpoint_max_vel': max_vel},
            {'traj_server/max_yaw_dot': max_wz},
        ],
    )

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('map_size_x', default_value=map_size_x))
    ld.add_action(DeclareLaunchArgument('map_size_y', default_value=map_size_y))
    ld.add_action(DeclareLaunchArgument('map_size_z', default_value=map_size_z))
    ld.add_action(DeclareLaunchArgument(
        'odom_topic', default_value=odom_topic,
        description='VIO odom in global frame (default from d1_robot.yaml)'))
    ld.add_action(DeclareLaunchArgument(
        'pose_topic', default_value=pose_topic,
        description='Camera pose for depth sync (default from d1_robot.yaml)'))
    ld.add_action(DeclareLaunchArgument(
        'depth_topic', default_value=depth_topic,
        description='Depth image topic (default from d1_robot.yaml)'))
    ld.add_action(DeclareLaunchArgument(
        'pos_cmd_topic', default_value=pos_cmd_topic,
        description='PositionCommand output (default from d1_robot.yaml)'))
    ld.add_action(DeclareLaunchArgument(
        'max_vel', default_value=max_vel,
        description='Planner max speed (m/s); default from d1_robot.yaml'))
    ld.add_action(DeclareLaunchArgument(
        'max_wz', default_value=max_wz,
        description='traj_server yaw_dot cap (rad/s); default from d1_robot.yaml'))
    ld.add_action(DeclareLaunchArgument(
        'max_acc', default_value=max_acc,
        description='Planner max acceleration (m/s^2); default from d1_robot.yaml'))
    ld.add_action(DeclareLaunchArgument(
        'planning_horizon', default_value=planning_horizon,
        description='Local planning horizon (m); default from d1_robot.yaml'))
    ld.add_action(DeclareLaunchArgument(
        'goal_reach_thresh', default_value=goal_reach_thresh,
        description='Goal reach distance (m); default from d1_robot.yaml'))
    ld.add_action(DeclareLaunchArgument(
        'thresh_replan_time', default_value=thresh_replan_time,
        description='Min EXEC time before replan (s); default from d1_robot.yaml'))
    ld.add_action(DeclareLaunchArgument(
        'obstacles_inflation', default_value=obstacles_inflation,
        description='Grid obstacle inflation (m); default from d1_robot.yaml'))
    ld.add_action(DeclareLaunchArgument(
        'optimization_dist0', default_value=optimization_dist0,
        description='B-spline clearance (m); default from d1_robot.yaml'))
    ld.add_action(DeclareLaunchArgument(
        'lambda_fitness', default_value=lambda_fitness,
        description='B-spline refine fitness weight; default from d1_robot.yaml'))
    ld.add_action(DeclareLaunchArgument(
        'enable_tag_tracking', default_value=enable_tag_tracking,
        description='true: AprilTag tracking; false: RViz 2D Goal'))
    ld.add_action(DeclareLaunchArgument('cx', default_value=cx))
    ld.add_action(DeclareLaunchArgument('cy', default_value=cy))
    ld.add_action(DeclareLaunchArgument('fx', default_value=fx))
    ld.add_action(DeclareLaunchArgument('fy', default_value=fy))
    ld.add_action(DeclareLaunchArgument('tag_pose_topic', default_value=tag_pose_topic))
    ld.add_action(DeclareLaunchArgument('tag_detected_topic', default_value=tag_detected_topic))
    ld.add_action(DeclareLaunchArgument(
        'tag_stop_dist', default_value=tag_stop_dist,
        description='AprilTag tracking stop distance to tag center (m, XY)'))
    ld.add_action(DeclareLaunchArgument('tag_update_min_dist', default_value=tag_update_min_dist))
    ld.add_action(DeclareLaunchArgument('tag_replan_min_period', default_value=tag_replan_min_period))
    ld.add_action(ego_planner_node)
    ld.add_action(traj_server_node)

    return ld

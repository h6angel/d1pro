"""
EGO planner + traj_server for D1 real robot (OpenVINS + RealSense depth mapping).

Prerequisites: run ../start_ego_stack.sh (recommended), or launch RealSense + OpenVINS
  manually before this node — see Readme.md.

All tuning defaults: config/d1_robot.yaml
Launch args override a subset at runtime (topics, limits, tag mode, camera intrinsics).

Logs: ./start_ego_stack.sh -> ego_log/stack_*/*.log
"""

import os
import sys

_launch_dir = os.path.dirname(os.path.abspath(__file__))
if _launch_dir not in sys.path:
    sys.path.insert(0, _launch_dir)

from d1_robot_config import (
    build_ego_planner_params,
    build_traj_server_params,
    load_d1_robot_config,
)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_d1 = load_d1_robot_config()
_topics = _d1['topics']
_limits = _d1['limits']
_camera = _d1['camera']
_planner = _d1['planner']
_fsm = _d1.get('fsm', {})


def _launch_setup(context, *args, **kwargs):
    cfg = load_d1_robot_config()

    def _float(lc):
        return float(lc.perform(context))

    def _str(lc):
        return lc.perform(context)

    def _bool(lc):
        return lc.perform(context).lower() in ('true', '1', 'yes', 'on')

    max_vel = _float(LaunchConfiguration('max_vel'))
    max_wz = _float(LaunchConfiguration('max_wz'))
    max_acc = _float(LaunchConfiguration('max_acc'))
    planning_horizon = _float(LaunchConfiguration('planning_horizon'))
    goal_reach_thresh = _float(LaunchConfiguration('goal_reach_thresh'))
    thresh_replan_time = _float(LaunchConfiguration('thresh_replan_time'))
    collision_check_step = _float(LaunchConfiguration('collision_check_step'))
    obstacles_inflation = _float(LaunchConfiguration('obstacles_inflation'))
    optimization_dist0 = _float(LaunchConfiguration('optimization_dist0'))
    lambda_fitness = _float(LaunchConfiguration('lambda_fitness'))
    map_size_x = _float(LaunchConfiguration('map_size_x'))
    map_size_y = _float(LaunchConfiguration('map_size_y'))
    map_size_z = _float(LaunchConfiguration('map_size_z'))
    cx = _float(LaunchConfiguration('cx'))
    cy = _float(LaunchConfiguration('cy'))
    fx = _float(LaunchConfiguration('fx'))
    fy = _float(LaunchConfiguration('fy'))
    tag_stop_dist = _float(LaunchConfiguration('tag_stop_dist'))

    odom_topic = _str(LaunchConfiguration('odom_topic'))
    pose_topic = _str(LaunchConfiguration('pose_topic'))
    depth_topic = _str(LaunchConfiguration('depth_topic'))
    pos_cmd_topic = _str(LaunchConfiguration('pos_cmd_topic'))
    tag_pose_topic = _str(LaunchConfiguration('tag_pose_topic'))
    tag_detected_topic = _str(LaunchConfiguration('tag_detected_topic'))
    tag_update_min_dist = _float(LaunchConfiguration('tag_update_min_dist'))
    tag_replan_min_period = _float(LaunchConfiguration('tag_replan_min_period'))
    enable_tag_tracking = _bool(LaunchConfiguration('enable_tag_tracking'))

    ego_overrides = {
        'fsm/thresh_replan_time': thresh_replan_time,
        'fsm/collision_check_step': collision_check_step,
        'fsm/thresh_goal_reach_meter': goal_reach_thresh,
        'fsm/planning_horizon': planning_horizon,
        'fsm/enable_tag_tracking': enable_tag_tracking,
        'fsm/tag_pose_topic': tag_pose_topic,
        'fsm/tag_detected_topic': tag_detected_topic,
        'fsm/tag_stop_dist': tag_stop_dist,
        'fsm/tag_update_min_dist': tag_update_min_dist,
        'fsm/tag_replan_min_period': tag_replan_min_period,
        'grid_map/map_size_x': map_size_x,
        'grid_map/map_size_y': map_size_y,
        'grid_map/map_size_z': map_size_z,
        'grid_map/obstacles_inflation': obstacles_inflation,
        'grid_map/cx': cx,
        'grid_map/cy': cy,
        'grid_map/fx': fx,
        'grid_map/fy': fy,
        'manager/max_vel': max_vel,
        'manager/max_acc': max_acc,
        'manager/planning_horizon': planning_horizon,
        'optimization/lambda_fitness': lambda_fitness,
        'optimization/dist0': optimization_dist0,
        'optimization/max_vel': max_vel,
        'optimization/max_acc': max_acc,
        'bspline/limit_vel': max_vel,
        'bspline/limit_acc': max_acc,
    }

    traj_overrides = {
        'traj_server/endpoint_max_vel': max_vel,
        'traj_server/max_yaw_dot': max_wz,
    }

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
            ('grid_map/odom', odom_topic),
            ('grid_map/pose', pose_topic),
            ('grid_map/depth', depth_topic),
            ('grid_map/occupancy_inflate', 'drone_0_grid/grid_map/occupancy_inflate'),
        ],
        parameters=[build_ego_planner_params(cfg, ego_overrides)],
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
        parameters=[build_traj_server_params(cfg, traj_overrides)],
    )

    return [ego_planner_node, traj_server_node]


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
    collision_check_step = LaunchConfiguration(
        'collision_check_step', default=str(_planner['collision_check_step']))
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
        'tag_pose_topic', default=_fsm.get('tag_pose_topic', '/apriltag/target_pose_global'))
    tag_detected_topic = LaunchConfiguration(
        'tag_detected_topic',
        default=_fsm.get('tag_detected_topic', '/apriltag/target_detected'))
    tag_stop_dist = LaunchConfiguration(
        'tag_stop_dist', default=str(_planner['tag_stop_dist']))
    tag_update_min_dist = LaunchConfiguration(
        'tag_update_min_dist', default=str(_fsm.get('tag_update_min_dist', 0.08)))
    tag_replan_min_period = LaunchConfiguration(
        'tag_replan_min_period', default=str(_fsm.get('tag_replan_min_period', 0.5)))

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
        'collision_check_step', default_value=collision_check_step,
        description='FSM traj safety arc step (m); default from d1_robot.yaml'))
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
    ld.add_action(OpaqueFunction(function=_launch_setup))

    return ld

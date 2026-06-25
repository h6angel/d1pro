from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# D1 实机话题/速度默认值见 config/d1_robot.yaml（由 single_run.launch.py 传入）。
# 单独 launch 本文件时仍使用下方 fallback 默认。

def generate_launch_description():
    map_size_x = LaunchConfiguration('map_size_x_', default=40.0)
    map_size_y = LaunchConfiguration('map_size_y_', default=40.0)
    map_size_z = LaunchConfiguration('map_size_z_', default=3.0)

    odometry_topic = LaunchConfiguration('odometry_topic', default='/ov_msckf/odomimu')
    camera_pose_topic = LaunchConfiguration('camera_pose_topic', default='/ov_msckf/pose_stamped')
    depth_topic = LaunchConfiguration('depth_topic', default='/camera/camera/depth/image_rect_raw')
    map_frame_id = LaunchConfiguration('map_frame_id', default='global')

    cx = LaunchConfiguration('cx', default=321.04638671875)
    cy = LaunchConfiguration('cy', default=243.44969177246094)
    fx = LaunchConfiguration('fx', default=387.229248046875)
    fy = LaunchConfiguration('fy', default=387.229248046875)

    max_vel = LaunchConfiguration('max_vel', default=0.6)
    max_acc = LaunchConfiguration('max_acc', default=1.0)
    planning_horizon = LaunchConfiguration('planning_horizon', default=7.5)

    goal_reach_thresh = LaunchConfiguration('goal_reach_thresh', default='0.3')
    thresh_replan_time = LaunchConfiguration('thresh_replan_time', default='1.0')
    obstacles_inflation = LaunchConfiguration('obstacles_inflation', default='0.099')
    optimization_dist0 = LaunchConfiguration('optimization_dist0', default='0.5')
    lambda_fitness = LaunchConfiguration('lambda_fitness', default='1.0')

    enable_tag_tracking = LaunchConfiguration('enable_tag_tracking', default='false')
    tag_pose_topic = LaunchConfiguration('tag_pose_topic', default='/apriltag/target_pose_global')
    tag_detected_topic = LaunchConfiguration('tag_detected_topic', default='/apriltag/target_detected')
    tag_follow_offset_x = LaunchConfiguration('tag_follow_offset_x', default='0.0')
    tag_follow_offset_y = LaunchConfiguration('tag_follow_offset_y', default='-0.3')
    tag_follow_offset_z = LaunchConfiguration('tag_follow_offset_z', default='0.0')
    tag_update_min_dist = LaunchConfiguration('tag_update_min_dist', default='0.08')
    tag_replan_min_period = LaunchConfiguration('tag_replan_min_period', default='0.5')
    tag_lost_timeout_sec = LaunchConfiguration('tag_lost_timeout_sec', default='30.0')

    map_size_x_arg = DeclareLaunchArgument('map_size_x_', default_value=map_size_x, description='Map size along X')
    map_size_y_arg = DeclareLaunchArgument('map_size_y_', default_value=map_size_y, description='Map size along Y')
    map_size_z_arg = DeclareLaunchArgument('map_size_z_', default_value=map_size_z, description='Map size along Z')
    odometry_topic_arg = DeclareLaunchArgument(
        'odometry_topic', default_value=odometry_topic,
        description='Robot odometry in map frame (nav_msgs/Odometry, e.g. /ov_msckf/odomimu)')
    camera_pose_topic_arg = DeclareLaunchArgument(
        'camera_pose_topic', default_value=camera_pose_topic,
        description='Camera pose for depth mapping (geometry_msgs/PoseStamped)')
    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic', default_value=depth_topic,
        description='Depth image (sensor_msgs/Image, 16UC1 mm typical)')
    map_frame_id_arg = DeclareLaunchArgument(
        'map_frame_id', default_value=map_frame_id,
        description='Grid map / visualization frame (OpenVINS: global)')
    cx_arg = DeclareLaunchArgument('cx', default_value=cx, description='Camera intrinsic cx')
    cy_arg = DeclareLaunchArgument('cy', default_value=cy, description='Camera intrinsic cy')
    fx_arg = DeclareLaunchArgument('fx', default_value=fx, description='Camera intrinsic fx')
    fy_arg = DeclareLaunchArgument('fy', default_value=fy, description='Camera intrinsic fy')
    max_vel_arg = DeclareLaunchArgument('max_vel', default_value=max_vel, description='Maximum velocity')
    max_acc_arg = DeclareLaunchArgument('max_acc', default_value=max_acc, description='Maximum acceleration')
    planning_horizon_arg = DeclareLaunchArgument('planning_horizon', default_value=planning_horizon, description='Planning horizon')

    goal_reach_thresh_arg = DeclareLaunchArgument(
        'goal_reach_thresh', default_value=goal_reach_thresh,
        description='FSM: odom within this distance of goal before finishing (m)')
    thresh_replan_time_arg = DeclareLaunchArgument(
        'thresh_replan_time', default_value=thresh_replan_time,
        description='FSM: minimum EXEC time before replan (s); use 2.0 for slow D1')
    obstacles_inflation_arg = DeclareLaunchArgument(
        'obstacles_inflation', default_value=obstacles_inflation,
        description='Grid map obstacle inflation radius (m); use 0.35+ for D1 body')
    optimization_dist0_arg = DeclareLaunchArgument(
        'optimization_dist0', default_value=optimization_dist0,
        description='B-spline optimizer clearance from obstacles (m)')
    lambda_fitness_arg = DeclareLaunchArgument(
        'lambda_fitness', default_value=lambda_fitness,
        description='B-spline refine fitness weight; raise if refine hits obstacles often')

    enable_tag_tracking_arg = DeclareLaunchArgument(
        'enable_tag_tracking', default_value=enable_tag_tracking,
        description='true: follow /apriltag targets; false: RViz 2D Goal')
    tag_pose_topic_arg = DeclareLaunchArgument('tag_pose_topic', default_value=tag_pose_topic)
    tag_detected_topic_arg = DeclareLaunchArgument('tag_detected_topic', default_value=tag_detected_topic)
    tag_follow_offset_x_arg = DeclareLaunchArgument('tag_follow_offset_x', default_value=tag_follow_offset_x)
    tag_follow_offset_y_arg = DeclareLaunchArgument('tag_follow_offset_y', default_value=tag_follow_offset_y)
    tag_follow_offset_z_arg = DeclareLaunchArgument('tag_follow_offset_z', default_value=tag_follow_offset_z)
    tag_update_min_dist_arg = DeclareLaunchArgument('tag_update_min_dist', default_value=tag_update_min_dist)
    tag_replan_min_period_arg = DeclareLaunchArgument('tag_replan_min_period', default_value=tag_replan_min_period)
    tag_lost_timeout_sec_arg = DeclareLaunchArgument('tag_lost_timeout_sec', default_value=tag_lost_timeout_sec)

    ego_planner_node = Node(
        package='ego_planner',
        executable='ego_planner_node',
        name='drone_0_ego_planner_node',
        output='screen',
        remappings=[
            ('odom_world', odometry_topic),
            ('planning/bspline', 'drone_0_planning/bspline'),

            ('goal_point', 'drone_0_plan_vis/goal_point'),
            ('global_list', 'drone_0_plan_vis/global_list'),
            ('init_list', 'drone_0_plan_vis/init_list'),
            ('optimal_list', 'drone_0_plan_vis/optimal_list'),
            ('a_star_list', 'drone_0_plan_vis/a_star_list'),

            ('grid_map/odom', odometry_topic),
            ('grid_map/pose', camera_pose_topic),
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
            {'fsm/tag_follow_offset_x': tag_follow_offset_x},
            {'fsm/tag_follow_offset_y': tag_follow_offset_y},
            {'fsm/tag_follow_offset_z': tag_follow_offset_z},
            {'fsm/tag_update_min_dist': tag_update_min_dist},
            {'fsm/tag_replan_min_period': tag_replan_min_period},
            {'fsm/tag_lost_timeout_sec': tag_lost_timeout_sec},

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
            {'grid_map/frame_id': map_frame_id},
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

            {'prediction/obj_num': 0},
            {'prediction/lambda': 1.0},
            {'prediction/predict_rate': 1.0},
        ],
    )

    ld = LaunchDescription()
    ld.add_action(map_size_x_arg)
    ld.add_action(map_size_y_arg)
    ld.add_action(map_size_z_arg)
    ld.add_action(odometry_topic_arg)
    ld.add_action(camera_pose_topic_arg)
    ld.add_action(depth_topic_arg)
    ld.add_action(map_frame_id_arg)
    ld.add_action(cx_arg)
    ld.add_action(cy_arg)
    ld.add_action(fx_arg)
    ld.add_action(fy_arg)
    ld.add_action(max_vel_arg)
    ld.add_action(max_acc_arg)
    ld.add_action(planning_horizon_arg)
    ld.add_action(goal_reach_thresh_arg)
    ld.add_action(thresh_replan_time_arg)
    ld.add_action(obstacles_inflation_arg)
    ld.add_action(optimization_dist0_arg)
    ld.add_action(lambda_fitness_arg)
    ld.add_action(enable_tag_tracking_arg)
    ld.add_action(tag_pose_topic_arg)
    ld.add_action(tag_detected_topic_arg)
    ld.add_action(tag_follow_offset_x_arg)
    ld.add_action(tag_follow_offset_y_arg)
    ld.add_action(tag_follow_offset_z_arg)
    ld.add_action(tag_update_min_dist_arg)
    ld.add_action(tag_replan_min_period_arg)
    ld.add_action(tag_lost_timeout_sec_arg)
    ld.add_action(ego_planner_node)

    return ld

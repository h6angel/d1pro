from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Single-robot D1 real deployment: fixed internal namespace drone_0, external topics via remaps.

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

    max_vel = LaunchConfiguration('max_vel', default=2.0)
    max_acc = LaunchConfiguration('max_acc', default=3.0)
    planning_horizon = LaunchConfiguration('planning_horizon', default=7.5)

    point_num = LaunchConfiguration('point_num', default=1)
    point0_x = LaunchConfiguration('point0_x', default=0.0)
    point0_y = LaunchConfiguration('point0_y', default=0.0)
    point0_z = LaunchConfiguration('point0_z', default=0.0)
    point1_x = LaunchConfiguration('point1_x', default=10.0)
    point1_y = LaunchConfiguration('point1_y', default=10.0)
    point1_z = LaunchConfiguration('point1_z', default=0.0)
    point2_x = LaunchConfiguration('point2_x', default=20.0)
    point2_y = LaunchConfiguration('point2_y', default=20.0)
    point2_z = LaunchConfiguration('point2_z', default=1.0)
    point3_x = LaunchConfiguration('point3_x', default=-10.0)
    point3_y = LaunchConfiguration('point3_y', default=-10.0)
    point3_z = LaunchConfiguration('point3_z', default=1.0)
    point4_x = LaunchConfiguration('point4_x', default=30.0)
    point4_y = LaunchConfiguration('point4_y', default=30.0)
    point4_z = LaunchConfiguration('point4_z', default=1.0)

    flight_type = LaunchConfiguration('flight_type', default='1')
    goal_reach_thresh = LaunchConfiguration('goal_reach_thresh', default='0.3')
    thresh_replan_time = LaunchConfiguration('thresh_replan_time', default='1.0')
    obstacles_inflation = LaunchConfiguration('obstacles_inflation', default='0.099')
    optimization_dist0 = LaunchConfiguration('optimization_dist0', default='0.5')
    lambda_fitness = LaunchConfiguration('lambda_fitness', default='1.0')
    use_distinctive_trajs = LaunchConfiguration('use_distinctive_trajs', default=False)

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

    point_num_arg = DeclareLaunchArgument('point_num', default_value=point_num, description='Number of waypoints')
    point0_x_arg = DeclareLaunchArgument('point0_x', default_value=point0_x, description='Waypoint 0 X coordinate')
    point0_y_arg = DeclareLaunchArgument('point0_y', default_value=point0_y, description='Waypoint 0 Y coordinate')
    point0_z_arg = DeclareLaunchArgument('point0_z', default_value=point0_z, description='Waypoint 0 Z coordinate')
    point1_x_arg = DeclareLaunchArgument('point1_x', default_value=point1_x, description='Waypoint 1 X coordinate')
    point1_y_arg = DeclareLaunchArgument('point1_y', default_value=point1_y, description='Waypoint 1 Y coordinate')
    point1_z_arg = DeclareLaunchArgument('point1_z', default_value=point1_z, description='Waypoint 1 Z coordinate')
    point2_x_arg = DeclareLaunchArgument('point2_x', default_value=point2_x, description='Waypoint 2 X coordinate')
    point2_y_arg = DeclareLaunchArgument('point2_y', default_value=point2_y, description='Waypoint 2 Y coordinate')
    point2_z_arg = DeclareLaunchArgument('point2_z', default_value=point2_z, description='Waypoint 2 Z coordinate')
    point3_x_arg = DeclareLaunchArgument('point3_x', default_value=point3_x, description='Waypoint 3 X coordinate')
    point3_y_arg = DeclareLaunchArgument('point3_y', default_value=point3_y, description='Waypoint 3 Y coordinate')
    point3_z_arg = DeclareLaunchArgument('point3_z', default_value=point3_z, description='Waypoint 3 Z coordinate')
    point4_x_arg = DeclareLaunchArgument('point4_x', default_value=point4_x, description='Waypoint 4 X coordinate')
    point4_y_arg = DeclareLaunchArgument('point4_y', default_value=point4_y, description='Waypoint 4 Y coordinate')
    point4_z_arg = DeclareLaunchArgument('point4_z', default_value=point4_z, description='Waypoint 4 Z coordinate')

    flight_type_arg = DeclareLaunchArgument('flight_type', default_value=flight_type, description='1=RViz 2D Goal, 2=preset waypoints')
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
    use_distinctive_trajs_arg = DeclareLaunchArgument('use_distinctive_trajs', default_value=use_distinctive_trajs, description='Distinctive trajectories (multi-agent only)')

    ego_planner_node = Node(
        package='ego_planner',
        executable='ego_planner_node',
        name='drone_0_ego_planner_node',
        output='screen',
        remappings=[
            ('odom_world', odometry_topic),
            ('planning/bspline', 'drone_0_planning/bspline'),
            ('planning/data_display', 'drone_0_planning/data_display'),

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
            {'fsm/flight_type': flight_type},
            {'fsm/thresh_replan_time': thresh_replan_time},
            {'fsm/thresh_no_replan_meter': 1.0},
            {'fsm/thresh_goal_reach_meter': goal_reach_thresh},
            {'fsm/planning_horizon': planning_horizon},
            {'fsm/planning_horizen_time': 3.0},
            {'fsm/emergency_time': 1.0},
            {'fsm/realworld_experiment': True},
            {'fsm/fail_safe': True},
            {'fsm/log_trace_period_ms': 500},

            {'fsm/waypoint_num': point_num},
            {'fsm/waypoint0_x': point0_x},
            {'fsm/waypoint0_y': point0_y},
            {'fsm/waypoint0_z': point0_z},
            {'fsm/waypoint1_x': point1_x},
            {'fsm/waypoint1_y': point1_y},
            {'fsm/waypoint1_z': point1_z},
            {'fsm/waypoint2_x': point2_x},
            {'fsm/waypoint2_y': point2_y},
            {'fsm/waypoint2_z': point2_z},
            {'fsm/waypoint3_x': point3_x},
            {'fsm/waypoint3_y': point3_y},
            {'fsm/waypoint3_z': point3_z},
            {'fsm/waypoint4_x': point4_x},
            {'fsm/waypoint4_y': point4_y},
            {'fsm/waypoint4_z': point4_z},

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
            {'grid_map/map_vis_rate': 25.0},
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
            {'manager/use_distinctive_trajs': use_distinctive_trajs},
            {'manager/drone_id': -1},
            {'manager/use_robot_z_planning': True},

            {'optimization/lambda_smooth': 1.0},
            {'optimization/lambda_collision': 0.5},
            {'optimization/lambda_feasibility': 0.1},
            {'optimization/lambda_fitness': lambda_fitness},
            {'optimization/dist0': optimization_dist0},
            {'optimization/swarm_clearance': 0.5},
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
    ld.add_action(point_num_arg)
    ld.add_action(point0_x_arg)
    ld.add_action(point0_y_arg)
    ld.add_action(point0_z_arg)
    ld.add_action(point1_x_arg)
    ld.add_action(point1_y_arg)
    ld.add_action(point1_z_arg)
    ld.add_action(point2_x_arg)
    ld.add_action(point2_y_arg)
    ld.add_action(point2_z_arg)
    ld.add_action(point3_x_arg)
    ld.add_action(point3_y_arg)
    ld.add_action(point3_z_arg)
    ld.add_action(point4_x_arg)
    ld.add_action(point4_y_arg)
    ld.add_action(point4_z_arg)
    ld.add_action(flight_type_arg)
    ld.add_action(goal_reach_thresh_arg)
    ld.add_action(thresh_replan_time_arg)
    ld.add_action(obstacles_inflation_arg)
    ld.add_action(optimization_dist0_arg)
    ld.add_action(lambda_fitness_arg)
    ld.add_action(use_distinctive_trajs_arg)
    ld.add_action(ego_planner_node)

    return ld

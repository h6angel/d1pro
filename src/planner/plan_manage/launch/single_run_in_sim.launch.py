import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    obj_num = LaunchConfiguration('obj_num', default=10)
    drone_id = LaunchConfiguration('drone_id', default=0)

    map_size_x = LaunchConfiguration('map_size_x', default=40.0)
    map_size_y = LaunchConfiguration('map_size_y', default=40.0)
    map_size_z = LaunchConfiguration('map_size_z', default=3.0)
    flight_type = LaunchConfiguration('flight_type', default='1')
    odom_topic = LaunchConfiguration('odom_topic', default='/odom')
    cloud_topic = LaunchConfiguration('cloud_topic', default='/gazebo_obstacles')
    depth_topic = LaunchConfiguration('depth_topic', default='depth')
    camera_pose_topic = LaunchConfiguration('camera_pose_topic', default='camera_pose')

    obj_num_cmd = DeclareLaunchArgument('obj_num', default_value=obj_num, description='Number of objects')
    drone_id_cmd = DeclareLaunchArgument('drone_id', default_value=drone_id, description='Drone ID')
    map_size_x_cmd = DeclareLaunchArgument('map_size_x', default_value=map_size_x, description='Map size along x')
    map_size_y_cmd = DeclareLaunchArgument('map_size_y', default_value=map_size_y, description='Map size along y')
    map_size_z_cmd = DeclareLaunchArgument('map_size_z', default_value=map_size_z, description='Map size along z')
    odom_topic_cmd = DeclareLaunchArgument('odom_topic', default_value=odom_topic, description='Odometry topic (absolute, e.g. /odom)')
    cloud_topic_cmd = DeclareLaunchArgument('cloud_topic', default_value=cloud_topic, description='Obstacle point cloud topic (absolute, e.g. /gazebo_obstacles)')
    depth_topic_cmd = DeclareLaunchArgument('depth_topic', default_value=depth_topic, description='Depth image topic suffix')
    camera_pose_topic_cmd = DeclareLaunchArgument('camera_pose_topic', default_value=camera_pose_topic, description='Camera pose topic suffix')

    use_dynamic = LaunchConfiguration('use_dynamic', default=False)
    use_dynamic_cmd = DeclareLaunchArgument('use_dynamic', default_value=use_dynamic, description='Use Drone Simulation Considering Dynamics or Not')

    use_builtin_simulator = LaunchConfiguration('use_builtin_simulator', default=False)
    use_builtin_simulator_cmd = DeclareLaunchArgument(
        'use_builtin_simulator',
        default_value=use_builtin_simulator,
        description='Launch built-in quadrotor simulator (disable when using Gazebo /odom)')
    flight_type_cmd = DeclareLaunchArgument(
        'flight_type',
        default_value=flight_type,
        description='1=RViz 2D Goal, 2=preset waypoints')

    advanced_param_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ego_planner'), 'launch', 'advanced_param.launch.py')),
        launch_arguments={
            'drone_id': drone_id,
            'map_size_x_': map_size_x,
            'map_size_y_': map_size_y,
            'map_size_z_': map_size_z,
            'odometry_topic': odom_topic,
            'obj_num_set': obj_num,
            'camera_pose_topic': camera_pose_topic,
            'depth_topic': depth_topic,
            'cloud_topic': cloud_topic,
            'cx': str(321.04638671875),
            'cy': str(243.44969177246094),
            'fx': str(387.229248046875),
            'fy': str(387.229248046875),
            'max_vel': str(2.0),
            'max_acc': str(6.0),
            'planning_horizon': str(7.5),
            'use_distinctive_trajs': 'True',
            'flight_type': flight_type,
            'point_num': str(1),
            'point0_x': str(15.0),
            'point0_y': str(0.0),
            'point0_z': str(1.0),
            'point1_x': str(-15.0),
            'point1_y': str(0.0),
            'point1_z': str(1.0),
            'point2_x': str(15.0),
            'point2_y': str(0.0),
            'point2_z': str(1.0),
            'point3_x': str(-15.0),
            'point3_y': str(0.0),
            'point3_z': str(1.0),
            'point4_x': str(15.0),
            'point4_y': str(0.0),
            'point4_z': str(1.0),
        }.items()
    )

    traj_server_node = Node(
        package='ego_planner',
        executable='traj_server',
        name=['drone_', drone_id, '_traj_server'],
        output='screen',
        remappings=[
            ('position_cmd', ['drone_', drone_id, '_planning/pos_cmd']),
            ('planning/bspline', ['drone_', drone_id, '_planning/bspline'])
        ],
        parameters=[
            {'traj_server/time_forward': 1.0}
        ]
    )

    simulator_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ego_planner'), 'launch', 'simulator.launch.py')),
        launch_arguments={
            'use_dynamic': use_dynamic,
            'drone_id': drone_id,
            'init_x_': str(-15.0),
            'init_y_': str(0.0),
            'init_z_': str(0.1),
            'odometry_topic': 'visual_slam/odom',
        }.items(),
        condition=IfCondition(use_builtin_simulator),
    )

    ld = LaunchDescription()
    ld.add_action(map_size_x_cmd)
    ld.add_action(map_size_y_cmd)
    ld.add_action(map_size_z_cmd)
    ld.add_action(odom_topic_cmd)
    ld.add_action(cloud_topic_cmd)
    ld.add_action(depth_topic_cmd)
    ld.add_action(camera_pose_topic_cmd)
    ld.add_action(obj_num_cmd)
    ld.add_action(drone_id_cmd)
    ld.add_action(use_dynamic_cmd)
    ld.add_action(use_builtin_simulator_cmd)
    ld.add_action(flight_type_cmd)
    ld.add_action(advanced_param_include)
    ld.add_action(traj_server_node)
    ld.add_action(simulator_include)

    return ld

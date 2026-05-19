import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

# Single D1 / Gazebo: subscribe /odom + /gazebo_obstacles, publish trajectory on /drone_0_planning/pos_cmd
def generate_launch_description():
    map_size_x = LaunchConfiguration('map_size_x', default=40.0)
    map_size_y = LaunchConfiguration('map_size_y', default=40.0)
    map_size_z = LaunchConfiguration('map_size_z', default=3.0)
    flight_type = LaunchConfiguration('flight_type', default='1')
    odom_topic = LaunchConfiguration('odom_topic', default='/odom')
    cloud_topic = LaunchConfiguration('cloud_topic', default='/gazebo_obstacles')
    pos_cmd_topic = LaunchConfiguration('pos_cmd_topic', default='/drone_0_planning/pos_cmd')

    map_size_x_cmd = DeclareLaunchArgument('map_size_x', default_value=map_size_x, description='Map size along x')
    map_size_y_cmd = DeclareLaunchArgument('map_size_y', default_value=map_size_y, description='Map size along y')
    map_size_z_cmd = DeclareLaunchArgument('map_size_z', default_value=map_size_z, description='Map size along z')
    odom_topic_cmd = DeclareLaunchArgument(
        'odom_topic', default_value=odom_topic,
        description='D1 odometry in world frame (pose + twist)')
    cloud_topic_cmd = DeclareLaunchArgument(
        'cloud_topic', default_value=cloud_topic,
        description='Obstacle point cloud in world frame')
    pos_cmd_topic_cmd = DeclareLaunchArgument(
        'pos_cmd_topic', default_value=pos_cmd_topic,
        description='Output position command topic for the robot controller')
    flight_type_cmd = DeclareLaunchArgument(
        'flight_type', default_value=flight_type,
        description='1=RViz 2D Goal, 2=preset waypoints')

    advanced_param_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ego_planner'), 'launch', 'advanced_param.launch.py')),
        launch_arguments={
            'map_size_x_': map_size_x,
            'map_size_y_': map_size_y,
            'map_size_z_': map_size_z,
            'odometry_topic': odom_topic,
            'cloud_topic': cloud_topic,
            'cx': str(321.04638671875),
            'cy': str(243.44969177246094),
            'fx': str(387.229248046875),
            'fy': str(387.229248046875),
            'max_vel': str(2.0),
            'max_acc': str(6.0),
            'planning_horizon': str(7.5),
            'use_distinctive_trajs': 'False',
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
        ],
        parameters=[
            {'traj_server/time_forward': 1.0},
        ],
    )

    ld = LaunchDescription()
    ld.add_action(map_size_x_cmd)
    ld.add_action(map_size_y_cmd)
    ld.add_action(map_size_z_cmd)
    ld.add_action(odom_topic_cmd)
    ld.add_action(cloud_topic_cmd)
    ld.add_action(pos_cmd_topic_cmd)
    ld.add_action(flight_type_cmd)
    ld.add_action(advanced_param_include)
    ld.add_action(traj_server_node)

    return ld

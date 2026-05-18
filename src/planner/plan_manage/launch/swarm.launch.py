import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    map_size_x = LaunchConfiguration('map_size_x', default=42.0)
    map_size_y = LaunchConfiguration('map_size_y', default=30.0)
    map_size_z = LaunchConfiguration('map_size_z', default=5.0)
    odom_topic = LaunchConfiguration('odom_topic', default='visual_slam/odom')

    map_size_x_cmd = DeclareLaunchArgument('map_size_x', default_value=map_size_x, description='Map size along x')
    map_size_y_cmd = DeclareLaunchArgument('map_size_y', default_value=map_size_y, description='Map size along y')
    map_size_z_cmd = DeclareLaunchArgument('map_size_z', default_value=map_size_z, description='Map size along z')
    odom_topic_cmd = DeclareLaunchArgument('odom_topic', default_value=odom_topic, description='Odometry topic')

    use_dynamic = LaunchConfiguration('use_dynamic', default=False)
    use_dynamic_cmd = DeclareLaunchArgument('use_dynamic', default_value=use_dynamic, description='Use Drone Simulation Considering Dynamics or Not')

    drone_configs = [
        {'drone_id': 0, 'init_x': -20.0, 'init_y': -9.0, 'init_z': 0.1, 'target_x': 20.0, 'target_y': 9.0, 'target_z': 1.0},
        {'drone_id': 1, 'init_x': -20.0, 'init_y': -7.0, 'init_z': 0.1, 'target_x': 20.0, 'target_y': 7.0, 'target_z': 1.0},
        {'drone_id': 2, 'init_x': -20.0, 'init_y': -5.0, 'init_z': 0.1, 'target_x': 20.0, 'target_y': 5.0, 'target_z': 1.0},
        {'drone_id': 3, 'init_x': -20.0, 'init_y': -3.0, 'init_z': 0.1, 'target_x': 20.0, 'target_y': 3.0, 'target_z': 1.0},
        {'drone_id': 4, 'init_x': -20.0, 'init_y': -1.0, 'init_z': 0.1, 'target_x': 20.0, 'target_y': 1.0, 'target_z': 1.0},
        {'drone_id': 5, 'init_x': -20.0, 'init_y': 1.0,  'init_z': 0.1, 'target_x': 20.0, 'target_y': -1.0, 'target_z': 1.0},
        {'drone_id': 6, 'init_x': -20.0, 'init_y': 3.0,  'init_z': 0.1, 'target_x': 20.0, 'target_y': -3.0, 'target_z': 1.0},
        {'drone_id': 7, 'init_x': -20.0, 'init_y': 5.0,  'init_z': 0.1, 'target_x': 20.0, 'target_y': -5.0, 'target_z': 1.0},
        {'drone_id': 8, 'init_x': -20.0, 'init_y': 7.0,  'init_z': 0.1, 'target_x': 20.0, 'target_y': -7.0, 'target_z': 1.0},
        {'drone_id': 9, 'init_x': -20.0, 'init_y': 9.0,  'init_z': 0.1, 'target_x': 20.0, 'target_y': -9.0, 'target_z': 1.0}
    ]

    drone_nodes = []
    for config in drone_configs:
        drone_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ego_planner'), 'launch', 'run_in_sim.launch.py')),
            launch_arguments={
                'use_dynamic': use_dynamic,
                'drone_id': str(config['drone_id']),
                'init_x': str(config['init_x']),
                'init_y': str(config['init_y']),
                'init_z': str(config['init_z']),
                'target_x': str(config['target_x']),
                'target_y': str(config['target_y']),
                'target_z': str(config['target_z']),
                'map_size_x': map_size_x,
                'map_size_y': map_size_y,
                'map_size_z': map_size_z,
                'odom_topic': odom_topic
            }.items()
        )
        drone_nodes.append(drone_launch)

    ld = LaunchDescription()
    ld.add_action(map_size_x_cmd)
    ld.add_action(map_size_y_cmd)
    ld.add_action(map_size_z_cmd)
    ld.add_action(odom_topic_cmd)
    ld.add_action(use_dynamic_cmd)
    for drone in drone_nodes:
        ld.add_action(drone)

    return ld

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
import launch_ros.actions
import launch_ros.descriptions
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    init_x = LaunchConfiguration('init_x_', default=0.0)
    init_y = LaunchConfiguration('init_y_', default=0.0)
    init_z = LaunchConfiguration('init_z_', default=0.0)
    odometry_topic = LaunchConfiguration('odometry_topic', default='visual_slam/odom')
    drone_id = LaunchConfiguration('drone_id', default=0)

    init_x_arg = DeclareLaunchArgument('init_x_', default_value=init_x, description='Initial X position')
    init_y_arg = DeclareLaunchArgument('init_y_', default_value=init_y, description='Initial Y position')
    init_z_arg = DeclareLaunchArgument('init_z_', default_value=init_z, description='Initial Z position')
    odometry_topic_arg = DeclareLaunchArgument('odometry_topic', default_value=odometry_topic, description='Odometry topic')
    drone_id_arg = DeclareLaunchArgument('drone_id', default_value=drone_id, description='Drone ID')

    use_dynamic = LaunchConfiguration('use_dynamic', default=True)
    use_dynamic_arg = DeclareLaunchArgument('use_dynamic', default_value=use_dynamic, description='Use Drone Simulation Considering Dynamics or Not')

    so3_quadrotor_simulator = launch_ros.actions.Node(
        package='so3_quadrotor_simulator', executable='so3_quadrotor_simulator',
        output='screen', name=['drone_', drone_id, '_quadrotor_simulator_so3'],
        parameters=[{'rate/odom': 100.0},
                    {'simulator/init_state_x': init_x},
                    {'simulator/init_state_y': init_y},
                    {'simulator/init_state_z': init_z}],
        remappings=[('odom', ['drone_', drone_id, '_visual_slam/odom']),
                    ('cmd', ['drone_', drone_id, '_so3_cmd']),
                    ('force_disturbance', ['drone_', drone_id, '_force_disturbance']),
                    ('moment_disturbance', ['drone_', drone_id, '_moment_disturbance'])],
        condition=IfCondition(use_dynamic)
    )

    gains_file = os.path.join(
        get_package_share_directory('so3_control'),
        'config',
        'gains_hummingbird.yaml'
    )
    corrections_file = os.path.join(
        get_package_share_directory('so3_control'),
        'config',
        'corrections_hummingbird.yaml'
    )

    so3_control_component = launch_ros.descriptions.ComposableNode(
        package='so3_control',
        plugin='SO3ControlComponent',
        name=['drone_', drone_id, '_so3_control_component'],
        parameters=[
            {'so3_control/init_state_x': init_x},
            {'so3_control/init_state_y': init_y},
            {'so3_control/init_state_z': init_z},
            {'mass': 0.98},
            {'use_angle_corrections': False},
            {'use_external_yaw': False},
            {'gains/rot/z': 1.0},
            {'gains/ang/z': 0.1},
            gains_file,
            corrections_file
        ],
        remappings=[('odom', ['drone_', drone_id, '_visual_slam/odom']),
                    ('position_cmd', ['drone_', drone_id, '_planning/pos_cmd']),
                    ('motors', ['drone_', drone_id, '_motors']),
                    ('corrections', ['drone_', drone_id, '_corrections']),
                    ('so3_cmd', ['drone_', drone_id, '_so3_cmd'])],
        condition=IfCondition(use_dynamic)
    )

    so3_control_container = ComposableNodeContainer(
        name=['drone_', drone_id, '_so3_control_container'],
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[so3_control_component],
        output='screen',
        condition=IfCondition(use_dynamic)
    )

    poscmd_2_odom_node = Node(
        package='poscmd_2_odom',
        executable='poscmd_2_odom',
        name=['drone_', drone_id, '_poscmd_2_odom'],
        output='screen',
        parameters=[
            {'init_x': init_x},
            {'init_y': init_y},
            {'init_z': init_z}
        ],
        remappings=[
            ('command', ['drone_', drone_id, '_planning/pos_cmd']),
            ('odometry', ['drone_', drone_id, '_', odometry_topic])
        ],
        condition=UnlessCondition(use_dynamic)
    )

    odom_visualization_node = Node(
        package='odom_visualization',
        executable='odom_visualization',
        name=['drone_', drone_id, '_odom_visualization'],
        output='screen',
        remappings=[
            ('odom', ['drone_', drone_id, '_visual_slam/odom']),
            ('robot', ['drone_', drone_id, '_vis/robot']),
            ('path', ['drone_', drone_id, '_vis/path']),
            ('time_gap', ['drone_', drone_id, '_vis/time_gap']),
        ],
        parameters=[
            {'color/a': 1.0},
            {'color/r': 0.0},
            {'color/g': 0.0},
            {'color/b': 0.0},
            {'covariance_scale': 100.0},
            {'robot_scale': 1.0},
            {'tf45': False},
            {'drone_id': drone_id}
        ]
    )

    ld = LaunchDescription()
    ld.add_action(init_x_arg)
    ld.add_action(init_y_arg)
    ld.add_action(init_z_arg)
    ld.add_action(odometry_topic_arg)
    ld.add_action(drone_id_arg)
    ld.add_action(use_dynamic_arg)
    ld.add_action(so3_quadrotor_simulator)
    ld.add_action(so3_control_container)
    ld.add_action(poscmd_2_odom_node)
    ld.add_action(odom_visualization_node)

    return ld

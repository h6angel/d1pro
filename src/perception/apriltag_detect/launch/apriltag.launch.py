"""Launch AprilTag detection + target pose node (camera and OpenVINS started separately)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('apriltag_detect')
    default_tags = os.path.join(pkg_share, 'config', 'tags.yaml')
    default_target_pose = os.path.join(pkg_share, 'config', 'target_pose.yaml')

    tags_config = LaunchConfiguration('tags_config', default=default_tags)
    target_pose_config = LaunchConfiguration('target_pose_config', default=default_target_pose)
    image_topic = LaunchConfiguration(
        'image_topic', default='/camera/camera/infra1/image_rect_raw')
    camera_info_topic = LaunchConfiguration(
        'camera_info_topic', default='/camera/camera/infra1/camera_info')

    return LaunchDescription([
        DeclareLaunchArgument(
            'tags_config', default_value=default_tags,
            description='apriltag_ros tag parameters (id, size, family)'),
        DeclareLaunchArgument(
            'target_pose_config', default_value=default_target_pose,
            description='target_pose_node parameters'),
        DeclareLaunchArgument(
            'image_topic', default_value='/camera/camera/infra1/image_rect_raw',
            description='rectified camera image for AprilTag'),
        DeclareLaunchArgument(
            'camera_info_topic', default_value='/camera/camera/infra1/camera_info',
            description='camera_info matching image_topic'),

        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag',
            namespace='apriltag',
            output='screen',
            parameters=[tags_config],
            remappings=[
                ('image_rect', image_topic),
                ('camera_info', camera_info_topic),
            ],
        ),
        Node(
            package='apriltag_detect',
            executable='target_pose_node',
            name='target_pose_node',
            output='screen',
            parameters=[target_pose_config],
        ),
    ])

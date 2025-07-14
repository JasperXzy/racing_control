import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'avoid_angular_ratio',
            default_value='0.2',
            description='angular speed ratio for obstacle avoidance'),
        DeclareLaunchArgument(
            'avoid_linear_speed',
            default_value='0.1',
            description='linear speed for obstacle avoidance'),
        DeclareLaunchArgument(
            'recovering_linear_speed',
            default_value='0.7',
            description='recovering_speed for obstacle avoidance'),
        DeclareLaunchArgument(
            'recovering_angular_ratio',
            default_value='0.8',
            description='recovering_ratio for obstacle avoidance'),
        DeclareLaunchArgument(
            'follow_angular_ratio',
            default_value='-1.0',
            description='angular speed ratio for line following'),
        DeclareLaunchArgument(
            'follow_linear_speed',
            default_value='0.1',
            description='linear speed for line following'),
        DeclareLaunchArgument(
            'bottom_threshold',
            default_value='200',
            description='bottom threshold for line following'),
        DeclareLaunchArgument(
            'parking_y_threshold',
            description='Y-pixel threshold for parking sign bottom edge to trigger stop'),
        DeclareLaunchArgument(
            'line_confidence_threshold',
            default_value='0.5',
            description='confidence threshold for line following'),
        DeclareLaunchArgument(
            'obstacle_confidence_threshold',
            default_value='0.5',
            description='confidence threshold for obstacle avoidance'),
        DeclareLaunchArgument(
            'parking_sign_confidence_threshold',
            default_value='0.6',
            description='confidence threshold for parking sign targeting'),
        Node(
            package='racing_control',
            executable='racing_control',
            output='screen',
            parameters=[
                {"pub_control_topic": "/cmd_vel"},
                {"avoid_angular_ratio": LaunchConfiguration('avoid_angular_ratio')},
                {"avoid_linear_speed": LaunchConfiguration('avoid_linear_speed')},
                {"recovering_linear_speed": LaunchConfiguration('recovering_linear_speed')},
                {"recovering_angular_ratio": LaunchConfiguration('recovering_angular_ratio')},
                {"follow_angular_ratio": LaunchConfiguration('follow_angular_ratio')},
                {"follow_linear_speed": LaunchConfiguration('follow_linear_speed')},
                {"bottom_threshold": LaunchConfiguration('bottom_threshold')},
                {"parking_y_threshold": LaunchConfiguration('parking_y_threshold')},
                {"line_confidence_threshold": LaunchConfiguration('line_confidence_threshold')},
                {"obstacle_confidence_threshold": LaunchConfiguration('obstacle_confidence_threshold')},
                {"parking_sign_confidence_threshold": LaunchConfiguration('parking_sign_confidence_threshold')},
            ],
            arguments=['--ros-args', '--log-level', 'warn']
        )
    ])

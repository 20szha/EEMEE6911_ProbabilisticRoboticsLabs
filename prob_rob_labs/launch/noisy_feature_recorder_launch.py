import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    duration_sec = LaunchConfiguration('duration_sec', default='40.0')
    threshold = LaunchConfiguration('threshold', default='321.55')
    output_file = LaunchConfiguration('output_file', default='feature_mean_data.txt')

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Set to true for simulation',
    )

    declare_duration_sec_arg = DeclareLaunchArgument(
        'duration_sec',
        default_value='40.0',
        description='Time duration to take measurements for',
    )

    declare_threshold_arg = DeclareLaunchArgument(
        'threshold',
        default_value='321.55',
        description='Forward velocity of the robot'
    )

    declare_output_file_arg = DeclareLaunchArgument(
        'output_file',
        default_value='feature_mean_data.txt',
        description='Output file to hold the datapoints'
    )

    noisy_feature_recorder_cmd = Node(
        package='prob_rob_labs',
        executable='noisy_feature_recorder',
        name='noisy_feature_recorder',
        parameters=[{
            'use_sim_time': use_sim_time,
            'duration_sec': duration_sec,
            'threshold': threshold,
            'output_file': output_file
        }]
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(declare_duration_sec_arg)
    ld.add_action(declare_threshold_arg)
    ld.add_action(declare_output_file_arg)
    ld.add_action(noisy_feature_recorder_cmd)

    return ld
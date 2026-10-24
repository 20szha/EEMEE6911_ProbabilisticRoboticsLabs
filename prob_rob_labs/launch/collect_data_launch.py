import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    duration_sec = LaunchConfiguration('duration_sec', default='40.0')
    output_file_linear = LaunchConfiguration('output_file_linear', default='/home/szhang2002/ros2_ws/src/prob_rob_labs_ros_2/prob_rob_labs/output/x_data.txt')
    output_file_angular = LaunchConfiguration('output_file_angular', default='/home/szhang2002/ros2_ws/src/prob_rob_labs_ros_2/prob_rob_labs/output/z_data.txt')

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

    declare_output_file_angular_arg = DeclareLaunchArgument(
        'output_file_angular',
        default_value='z_data.txt',
        description='Output file to hold the angular z datapoints'
    )

    declare_output_file_linear_arg = DeclareLaunchArgument(
        'output_file_linear',
        default_value='x_data.txt',
        description='Output file to hold the linear x datapoints'
    )

    collect_data_cmd = Node(
        package='prob_rob_labs',
        executable='collect_data',
        name='collect_data',
        parameters=[{
            'use_sim_time': use_sim_time,
            'duration_sec': duration_sec,
            'output_file_linear': output_file_linear,
            'output_file_angular': output_file_angular,
        }]
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(declare_duration_sec_arg)
    ld.add_action(declare_output_file_angular_arg)
    ld.add_action(declare_output_file_linear_arg)
    ld.add_action(collect_data_cmd)

    return ld

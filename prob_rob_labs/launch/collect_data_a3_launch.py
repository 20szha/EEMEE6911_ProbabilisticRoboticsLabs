import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    duration_sec = LaunchConfiguration('duration_sec', default='40.0')
    output_file = LaunchConfiguration('output_file', default='/home/szhang2002/ros2_ws/src/prob_rob_labs_ros_2/prob_rob_labs/output/x_linear_data.txt')
    experiment = LaunchConfiguration('experiment', default='linear')
    target_linear = LaunchConfiguration('target_linear', default='0.5')
    target_angular = LaunchConfiguration('target_angular', default='0.5')
    command_delay = LaunchConfiguration('command_delay', default='1.0')

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Set to true for simulation',
    )

    declare_experiment_arg = DeclareLaunchArgument(
        'experiment',
        default_value='linear',
        description='Which velocity type we are focusing on',
    )

    declare_target_linear_arg = DeclareLaunchArgument(
        'target_linear',
        default_value='0.5',
        description='Linear velocity to set',
    )

    declare_target_angular_arg = DeclareLaunchArgument(
        'target_angular',
        default_value='0.5',
        description='Angular velocity to set',
    )

    declare_command_delay_arg = DeclareLaunchArgument(
        'command_delay',
        default_value='1.0',
        description='Delay between commands',
    )

    declare_duration_sec_arg = DeclareLaunchArgument(
        'duration_sec',
        default_value='40.0',
        description='Time duration to take measurements for',
    )

    declare_output_file_arg = DeclareLaunchArgument(
        'output_file',
        default_value='/home/szhang2002/ros2_ws/src/prob_rob_labs_ros_2/prob_rob_labs/output/x_linear_data.txt',
        description='Output file to hold the linear x datapoints'
    )

    collect_data_a3_cmd = Node(
        package='prob_rob_labs',
        executable='collect_data_a3',
        name='collect_data_a3',
        parameters=[{
            'use_sim_time': use_sim_time,
            'duration_sec': duration_sec,
            'output_file': output_file
        }]
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(declare_duration_sec_arg)
    ld.add_action(declare_output_file_arg)
    ld.add_action(declare_experiment_arg)
    ld.add_action(declare_target_linear_arg)
    ld.add_action(declare_target_angular_arg)
    ld.add_action(declare_command_delay_arg)
    ld.add_action(collect_data_a3_cmd)

    return ld

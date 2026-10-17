import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    p_success = LaunchConfiguration('p_success', default='0.15714285714')
    p_z_open_given_open = LaunchConfiguration('p_z_open_given_open', default='0.9536')
    p_z_open_given_closed = LaunchConfiguration('p_z_open_given_closed', default='0.0910')
    p_z_closed_given_open = LaunchConfiguration('p_z_closed_given_open', default='0.0464')
    p_z_closed_given_closed =  LaunchConfiguration('p_z_closed_given_closed', default='0.9090')

    initial_belief_open = LaunchConfiguration('initial_belief_open', default='0.50')
    initial_belief_closed = LaunchConfiguration('initial_belief_closed', default='0.50')
    belief_threshold = LaunchConfiguration('belief_threshold', default='0.99')
    command_wait_sec = LaunchConfiguration('command_wait_sec', default='4.0')
    forward_velocity = LaunchConfiguration('forward_velocity', default='1.0')

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Set to true for simulation',
    )

    full_bayes_filter_cmd = Node(
        package='prob_rob_labs',
        executable='full_bayes_filter',
        name='full_bayes_filter',
        parameters=[{
            'use_sim_time': use_sim_time,
            'p_success': p_success,
            'p_z_open_given_open': p_z_open_given_open,
            'p_z_open_given_closed': p_z_open_given_closed,
            'p_z_closed_given_open': p_z_closed_given_open,
            'p_z_closed_given_closed': p_z_closed_given_closed,
            'initial_belief_open': initial_belief_open,
            'initial_belief_closed': initial_belief_closed,
            'belief_threshold': belief_threshold,
            'command_wait_sec': command_wait_sec,
            'forward_velocity': forward_velocity
        }]
    )

    declare_p_success_arg = DeclareLaunchArgument(
        'p_success',
        default_value='0.15714285714',
        description='Probability of successfully opening the door'
    )

    declare_p_z_open_given_open_arg = DeclareLaunchArgument(
        'p_z_open_given_open',
        default_value='0.9363',
        description='Probability of measuring open when door is open'
    )

    declare_p_z_open_given_closed_arg = DeclareLaunchArgument(
        'p_z_open_given_closed',
        default_value='0.0750',
        description='Probability of measuring open when door is closed'
    )

    declare_p_z_closed_given_open_arg = DeclareLaunchArgument(
        'p_z_closed_given_open',
        default_value='0.0637',
        description='Probability of measuring closed when door is open'
    )

    declare_p_z_closed_given_closed_arg = DeclareLaunchArgument(
        'p_z_closed_given_closed',
        default_value='0.9250',
        description='Probability of measuring closed when door is closed'
    )



    declare_initial_belief_open_arg = DeclareLaunchArgument(
        'initial_belief_open',
        default_value='0.5',
        description='Initial belief of open door'
    )

    declare_initial_belief_closed_arg = DeclareLaunchArgument(
        'initial_belief_closed',
        default_value='0.5',
        description='Initial belief of closed door'
    )

    declare_belief_threshold_arg = DeclareLaunchArgument(
        'belief_threshold',
        default_value='0.99',
        description='Belief threshold'
    )

    declare_command_wait_sec_arg = DeclareLaunchArgument(
        'command_wait_sec',
        default_value='4.0',
        description='How many seconds to wait between door open command and measurement'
    )

    declare_forward_velocity_arg = DeclareLaunchArgument(
        'forward_velocity',
        default_value='1.0',
        description='Forward velocity of the robot'
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(declare_p_success_arg)
    ld.add_action(declare_p_z_open_given_open_arg)
    ld.add_action(declare_p_z_open_given_closed_arg)
    ld.add_action(declare_p_z_closed_given_open_arg)
    ld.add_action(declare_p_z_closed_given_closed_arg)
    ld.add_action(declare_initial_belief_open_arg)
    ld.add_action(declare_initial_belief_closed_arg)
    ld.add_action(declare_belief_threshold_arg)
    ld.add_action(declare_command_wait_sec_arg)
    ld.add_action(declare_forward_velocity_arg)
    ld.add_action(full_bayes_filter_cmd)


    return ld

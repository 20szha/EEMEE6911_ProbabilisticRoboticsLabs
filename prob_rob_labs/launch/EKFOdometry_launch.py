import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    tau_v = LaunchConfiguration('tau_v', default='0.621855')
    tau_w = LaunchConfiguration('tau_w', default='0.669411')
    gain_v = LaunchConfiguration('gain_v', default='0.999628')
    gain_w = LaunchConfiguration('gain_w', default='0.995552')
    encoder_variance = LaunchConfiguration('encoder_variance', default='0.04')
    gyro_variance = LaunchConfiguration('gyro_variance', default='0.01')
    sync_slop = LaunchConfiguration('sync_slop', default='0.05')
    command_timeout = LaunchConfiguration('command_timeout', default='1.0')
    odom_frame = LaunchConfiguration('odom_frame', default='odom')
    base_frame = LaunchConfiguration('base_frame', default='base_footprint')

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Set to true for simulation',
    )

    declare_tau_v_arg = DeclareLaunchArgument(
        'tau_v',
        default_value='0.621855',
        description='Tau value for linear velocity'
    )

    declare_tau_w_arg = DeclareLaunchArgument(
        'tau_w',
        default_value='0.669411',
        description='Tau value for angular velocity'
    )

    declare_gain_v_arg = DeclareLaunchArgument(
        'gain_v',
        default_value='0.999628',
        description='Gain value for linear velocity'
    )

    declare_gain_w_arg = DeclareLaunchArgument(
        'gain_w',
        default_value='0.995552',
        description='Gain value for angular velocity'
    )

    declare_encoder_variance_arg = DeclareLaunchArgument(
        'encoder_variance',
        default_value='0.04',
        description='Variance of the encoder'
    )

    declare_gyro_variance_arg = DeclareLaunchArgument(
        'gyro_variance',
        default_value='0.01',
        description='Variance of the gyro'
    )

    declare_sync_slop_arg = DeclareLaunchArgument(
        'sync_slop',
        default_value='0.05',
        description='Sync slop'
    )

    declare_command_timeout_arg = DeclareLaunchArgument(
        'command_timeout',
        default_value='1.0',
        description='Command timeout'
    )

    declare_odom_frame_arg = DeclareLaunchArgument(
        'odom_frame',
        default_value='odom',
        description='Odometry frame'
    )

    declare_base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='base_footprint',
        description='Base frame'
    )

    EKFOdometry_cmd = Node(
        package='prob_rob_labs',
        executable='EKFOdometry',
        name='EKFOdometry',
        parameters=[{
            'use_sim_time': use_sim_time,
            'tau_v': tau_v,
            'tau_w': tau_w,
            'gain_v': gain_v,
            'gain_w': gain_w,
            'encoder_variance': encoder_variance,
            'gyro_variance': gyro_variance,
            'sync_slop': sync_slop,
            'command_timeout': command_timeout,
            'odom_frame': odom_frame,
            'base_frame': base_frame
        }]
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(declare_tau_v_arg)
    ld.add_action(declare_tau_w_arg)
    ld.add_action(declare_gain_v_arg)
    ld.add_action(declare_gain_w_arg)
    ld.add_action(declare_encoder_variance_arg)
    ld.add_action(declare_gyro_variance_arg)
    ld.add_action(declare_sync_slop_arg)
    ld.add_action(declare_command_timeout_arg)
    ld.add_action(declare_odom_frame_arg)
    ld.add_action(declare_base_frame_arg)
    ld.add_action(EKFOdometry_cmd)

    return ld





    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='set to true for simulation'),
        Node(
            package='prob_rob_labs',
            executable='EKFOdometry',
            name='EKFOdometry',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        )
    ])




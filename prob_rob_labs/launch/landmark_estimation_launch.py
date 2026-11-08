import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from std_msgs.msg import Float64


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    landmark_color = LaunchConfiguration('landmark_color', default='cyan')
    landmark_height = LaunchConfiguration('landmark_height', default='0.5')

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Set to true for simulation',
    )

    declare_landmark_color_arg = DeclareLaunchArgument(
        'landmark_color',
        default_value='cyan',
        description='Color of the landmark of interest'
    )

    declare_landmark_height_arg = DeclareLaunchArgument(
        'landmark_height',
        default_value='0.5',
        description='Height of the landmark',
    )

    landmark_estimation_cmd = Node(
        package='prob_rob_labs',
        executable='landmark_estimation',
        name='landmark_estimation',
        parameters=[{
            'use_sim_time': use_sim_time,
            'landmark_color': landmark_color,
            'landmark_height': landmark_height
        }]
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(declare_landmark_color_arg)
    ld.add_action(declare_landmark_height_arg)
    ld.add_action(landmark_estimation_cmd)


    return ld

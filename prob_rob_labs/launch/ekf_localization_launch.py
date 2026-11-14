import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    default_map_file = os.path.join(get_package_share_directory('prob_rob_labs'), 'config', 'landmark_map.json')
    map_file = LaunchConfiguration('map_file', default=default_map_file)

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Set to true for simulation',
    )

    declare_map_file = DeclareLaunchArgument(
        'map_file',
        default_value=default_map_file,
        description='Path to the JSON landmark map'
    )
    declare_margin = DeclareLaunchArgument(
        'margin',
        default_value='10',
        description='Ignored image boundary margin in pixels',
    )
    declare_pixel_stddev = DeclareLaunchArgument(
        'pixel_stddev',
        default_value='1.0',
        description='Assumed standard deviation of each detected box edge in pixels',
    )

    ekf_localization_cmd = Node(
        package='prob_rob_labs',
        executable='ekf_localization',
        name='ekf_localization',
        parameters=[{
            'map_file': map_file,
            'use_sim_time': use_sim_time,
            'margin': LaunchConfiguration('margin'),
            'pixel_stddev': LaunchConfiguration('pixel_stddev'),
        }]
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(declare_map_file)
    ld.add_action(declare_margin)
    ld.add_action(declare_pixel_stddev)
    ld.add_action(ekf_localization_cmd)

    return ld

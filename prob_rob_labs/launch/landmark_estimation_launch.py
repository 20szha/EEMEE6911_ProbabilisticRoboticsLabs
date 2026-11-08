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
    image_width = LaunchConfiguration('image_width', default='640')
    image_height = LaunchConfiguration('image_height', default='480')
    margin = LaunchConfiguration('margin', default='10')

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

    declare_image_width_arg = DeclareLaunchArgument(
        'image_width',
        default_value='640',
        description='Width of the camera image',
    )

    declare_image_height_arg = DeclareLaunchArgument(
        'image_height',
        default_value='480',
        description='Height of the camera image',
    )

    declare_margin_arg = DeclareLaunchArgument(
        'margin',
        default_value='5',
        description='Margin from the image boundary',
    )

    landmark_estimation_cmd = Node(
        package='prob_rob_labs',
        executable='landmark_estimation',
        name='landmark_estimation',
        parameters=[{
            'use_sim_time': use_sim_time,
            'landmark_color': landmark_color,
            'landmark_height': landmark_height,
            'image_width': image_width,
            'image_height': image_height,
            'margin': margin
        }]
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(declare_landmark_color_arg)
    ld.add_action(declare_landmark_height_arg)
    ld.add_action(declare_image_width_arg)
    ld.add_action(declare_image_height_arg)
    ld.add_action(declare_margin_arg)
    ld.add_action(landmark_estimation_cmd)

    return ld

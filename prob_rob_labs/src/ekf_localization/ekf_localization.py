import json
import math
import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node


class EKFLocalization(Node):
    def __init__(self):
        super().__init__('ekf_localization')

        default_map_file = os.path.join(get_package_share_directory('prob_rob_labs'), 'config', 'landmark_map.json')
        self.declare_parameter('map_file', default_map_file)
        map_file = self.get_parameter('map_file').get_parameter_value().string_value
        self.landmarks = self.load_map(map_file)

        self.get_logger().info(
            f'Loaded {len(self.landmarks)} landmarks from {map_file} '
        )

    @staticmethod
    def load_map(map_file):
        if not map_file:
            raise ValueError('The map_file parameter must not be empty')

        try:
            with open(map_file, 'r', encoding='utf-8') as file:
                map_data = json.load(file)
        except (OSError, json.JSONDecodeError) as error:
            raise ValueError(f'Could not load landmark map "{map_file}": {error}') from error

        if not isinstance(map_data, dict):
            raise ValueError('The landmark map must contain a JSON object')

        landmarks = map_data.get('landmarks')
        if not isinstance(landmarks, dict) or not landmarks:
            raise ValueError('The landmark map must define a non-empty landmarks object')

        for color, landmark in landmarks.items():
            if not isinstance(color, str) or not isinstance(landmark, dict):
                raise ValueError('Each landmark must map a color string to an object')

            position = landmark.get('position')
            dimensions = landmark.get('dimensions')
            if not isinstance(position, dict) or not isinstance(dimensions, dict):
                raise ValueError(
                    f'Landmark "{color}" must define position and dimensions objects'
                )

            for axis in ('x', 'y', 'z'):
                value = position.get(axis)
                if not isinstance(value, (int, float)) or not math.isfinite(value):
                    raise ValueError(
                        f'Landmark "{color}" has invalid position.{axis}'
                    )

            for dimension in ('radius', 'height'):
                value = dimensions.get(dimension)
                if (
                    not isinstance(value, (int, float))
                    or not math.isfinite(value)
                    or value <= 0
                ):
                    raise ValueError(
                        f'Landmark "{color}" has invalid dimensions.{dimension}'
                    )

        return landmarks

    def get_landmark(self, color):
        try:
            return self.landmarks[color]
        except KeyError as error:
            raise KeyError(f'No landmark of color "{color}" is in the map') from error


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = EKFLocalization()
        rclpy.spin(node)
    except (ValueError, KeyError) as error:
        if node is not None:
            node.get_logger().fatal(str(error))
        else:
            print(f'Failed to initialize EKF localization: {error}')
        raise
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

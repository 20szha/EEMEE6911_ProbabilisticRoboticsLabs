import json
import math
import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from prob_rob_msgs.msg import Point2DArrayStamped
from sensor_msgs.msg import CameraInfo

from rclpy.time import Time

from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray


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

        self.camera_info = None
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10,
        )

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.vision_subscriptions = {}

        for color in self.landmarks:
            topic = f'/vision_{color}/corners'

            self.vision_subscriptions[color] = self.create_subscription(
                Point2DArrayStamped,
                topic,
                lambda msg, landmark_color=color:                                                       # effectively defining the callback as    ----      def callback(msg, landmark_color=color):
                    self.vision_corners_callback(msg, landmark_color),                                  #                                                       self.vision_corners_callback(msg, landmark_color)
                10,
            )

        self.measured_distance = {}
        self.measured_bearing = {}
        self.robot_pose = {}
        self.landmark_poses = {}

        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('margin', 10)

        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        self.margin = self.get_parameter('margin').value

        # topic = f'/vision_{self.landmark_color}/corners'                                                # topic of the corners of the landmark to listen to

        # self.red_vision_corners = self.create_subscription(
        #     Point2DArrayStamped,
        #     '/vision_red/corners',                                                                      # topic to listen to
        #     self.vision_corners_callback,
        #     10
        # )

        # self.cyan_vision_corners = self.create_subscription(
        #     Point2DArrayStamped,
        #     '/vision_cyan/corners',                                                                     # topic to listen to
        #     self.vision_corners_callback,
        #     10
        # )

        # self.magenta_vision_corners = self.create_subscription(
        #     Point2DArrayStamped,
        #     '/vision_magenta/corners',                                                                  # topic to listen to
        #     self.vision_corners_callback,
        #     10
        # )

        # self.green_vision_corners = self.create_subscription(
        #     Point2DArrayStamped,
        #     '/vision_green/corners',                                                                    # topic to listen to
        #     self.vision_corners_callback,
        #     10
        # )

        # self.yellow_vision_corners = self.create_subscription(
        #     Point2DArrayStamped,
        #     '/vision_yellow/corners',                                                                   # topic to listen to
        #     self.vision_corners_callback,
        #     10
        # )


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

    def vision_corners_callback(self, msg, landmark_color):
        num_points = len(msg.points)

        self.get_logger().info(
            f'Received cornerpoints for {landmark_color}: {num_points} points'
        )

        if num_points == 0:
            self.get_logger().debug(
                f'No cornerpoints detected for {landmark_color}'
            )
            return
        
        self.get_logger().info(
            f'Received cornerpoints for {landmark_color}: '
            f'{len(msg.points)} points'
        )
        
        if not msg.points:
            return

        if self.fx is None or self.fy is None:
            self.get_logger().error('Camera intrinsic parameters not received')
            return

        min_x = min(point.x for point in msg.points)
        max_x = max(point.x for point in msg.points)
        min_y = min(point.y for point in msg.points)
        max_y = max(point.y for point in msg.points)

        height_pixels = max_y - min_y

        if height_pixels <= 0:
            self.get_logger().warning(
                f'Invalid bounding box for {landmark_color}'
            )
            return

        if (min_x <= self.margin or max_x >= self.image_width - self.margin or min_y <= self.margin or max_y >= self.image_height - self.margin):
            self.get_logger().warning(f'{landmark_color} landmark too close to image boundary')
            return

        center_x = (min_x + max_x) / 2.0

        landmark = self.get_landmark(landmark_color)

        landmark_x = landmark['position']['x']
        landmark_y = landmark['position']['y']
        landmark_height = landmark['dimensions']['height']

        if self.fx <= 0 or self.fy <= 0:
            self.get_logger().warning('Invalid camera calibration')
            return

        measurement = self.estimate_distance_bearing(height_pixels, center_x, self.fx, self.fy, self.cx, landmark_height)

        if measurement is None:
            return

        distance, bearing = measurement

        self.get_logger().info(
            f'Observed {landmark_color} landmark: '
            f'x={landmark_x:.2f}, y={landmark_y:.2f}, '
            f'distance={distance:.2f} m, '
            f'bearing={bearing:.3f} rad'
        )

    @staticmethod
    def estimate_distance_bearing(height_pixels, center_x, fx, fy, cx, landmark_height):
        if height_pixels <= 0:
            return None

        theta = math.atan((cx - center_x) / fx)

        distance = ((fy * landmark_height) / (height_pixels * math.cos(theta)))

        return distance, theta

    def camera_info_callback(self, msg):
        self.fx = msg.p[0]
        self.cx = msg.p[2]
        self.fy = msg.p[5]
        self.cy = msg.p[6]

        if self.fx <= 0 or self.fy <= 0:
            self.get_logger().warning('Invalid camera intrinsic parameters')
            self.fx = None
            self.fy = None
            return

        self.get_logger().info(
            f'Camera calibration received: '
            f'fx={self.fx:.2f}, fy={self.fy:.2f}, '
            f'cx={self.cx:.2f}, cy={self.cy:.2f}'
        )

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

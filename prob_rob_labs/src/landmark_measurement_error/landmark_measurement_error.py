import rclpy
from rclpy.node import Node
from rclpy.time import Time
import math
from prob_rob_msgs.msg import Point2DArrayStamped

from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray

heartbeat_period = 0.1

def estimate_distance_bearing(height_pixels, center_x, fx, fy, cx, landmark_height):

        if height_pixels <= 0:
            return None

        theta = math.atan((cx - center_x) / fx)

        distance = (fy * landmark_height / (height_pixels * math.cos(theta)))

        return distance, theta

class LandmarkMeasurementError(Node):

    def __init__(self):
        super().__init__('landmark_measurement_error')
        self.log = self.get_logger()
        self.timer = self.create_timer(heartbeat_period, self.heartbeat)

        self.declare_parameter('landmark_color', 'cyan')   
        self.declare_parameter('landmark_height', 0.5)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('margin', 10)

        self.landmark_color = self.get_parameter('landmark_color').value
        self.landmark_height = self.get_parameter('landmark_height').value
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        self.margin = self.get_parameter('margin').value

        self.last_measurement_time = None
        self.measurement_timeout = 0.5

        topic = f'/vision_{self.landmark_color}/corners'                                                # topic of the corners of the landmark to listen to

        self.vision_corners = self.create_subscription(
            Point2DArrayStamped,
            topic,                                                                                      # topic to listen to
            self.vision_corners_callback,
            10
        )

        self.camera_info = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10
        )

        self.distance_pub = self.create_publisher(
            Float64,
            '/landmark/distance',
            10
        )

        self.bearing_pub = self.create_publisher(
            Float64,
            '/landmark/bearing',
            10
        )

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.camera_info_received = False

        self.measured_distance = None
        self.measured_bearing = None
        self.robot_pose = None
        self.landmark_poses = None

        self.landmark_indices = {
            'red': 1,
            'green': 2,
            'yellow': 3,
            'magenta': 4,
            'cyan': 5
        }

        if self.landmark_color not in self.landmark_indices:
            raise ValueError(
                f'Invalid landmark_color: {self.landmark_color}'
            )

        self.landmark_index = self.landmark_indices[self.landmark_color]

        self.camera_offset_x = 0.076
        self.camera_offset_y = 0.0

        self.distance_sub = self.create_subscription(
            Float64,
            '/landmark/distance',
            self.distance_callback,
            10
        )

        self.bearing_sub = self.create_subscription(
            Float64,
            '/landmark/bearing',
            self.bearing_callback,
            10
        )

        self.ground_truth_sub = self.create_subscription(
            PoseStamped,
            '/tb3/ground_truth/pose',
            self.ground_truth_callback,
            10
        )

        self.landmark_pose_sub = self.create_subscription(
            PoseArray,
            '/world/default/pose/info',
            self.landmark_pose_callback,
            10
        )

        self.distance_error_pub = self.create_publisher(
            Float64,
            '/landmark/distance_error',
            10
        )

        self.bearing_error_pub = self.create_publisher(
            Float64,
            '/landmark/bearing_error',
            10
        )

    def camera_info_callback(self, msg):
        self.fx = msg.p[0]
        self.cx = msg.p[2]
        self.fy = msg.p[5]
        self.cy = msg.p[6]

        self.camera_info_received = True

    def vision_corners_callback(self, msg):
        if not msg.points:
            self.get_logger().debug(
                f'No corners detected for {self.landmark_color}'
            )
            return

        if not self.camera_info_received:
            self.get_logger().debug('Waiting for camera calibration...')
            return

        min_x = min(point.x for point in msg.points)
        max_x = max(point.x for point in msg.points)
        min_y = min(point.y for point in msg.points)
        max_y = max(point.y for point in msg.points)

        height_pixels = max_y - min_y
        center_x = (min_x + max_x) / 2.0

        if (min_x <= self.margin or max_x >= self.image_width - self.margin):
            self.get_logger().warning(f'Landmark is too close to the image boundary for {self.landmark_color}')
            return
        
        measurement = estimate_distance_bearing(height_pixels, center_x, self.fx, self.fy, self.cx, self.landmark_height)

        if measurement is None:
            self.get_logger().warning(f'Failed to estimate distance and bearing for {self.landmark_color}')
            return

        self.get_logger().info(
            f'Number of detected corners: {len(msg.points)}'
        )

        distance, theta = measurement

        distance_msg = Float64()
        distance_msg.data = float(distance)
        self.distance_pub.publish(distance_msg)

        bearing_msg = Float64()
        bearing_msg.data = float(theta)
        self.bearing_pub.publish(bearing_msg)

        self.get_logger().info(
            f'{self.landmark_color}: min_x={min_x:.2f}, max_x={max_x:.2f}, '
            f'min_y={min_y:.2f}, max_y={max_y:.2f}, '
            f'height={height_pixels:.2f}, '
            f'vertical symmetry axis={center_x:.2f}, '
            f'distance={distance:.2f} m, ' 
            f'bearing={theta:.2f} radians or {math.degrees(theta):.2f} degrees'
        )

    def heartbeat(self):
        self.get_logger().info('heartbeat')
        self.calculate_errors()

    def distance_callback(self, msg):
        self.measured_distance = msg.data
        self.get_logger().info(
            f'Measured distance: {self.measured_distance:.3f} m'
        )

    def bearing_callback(self, msg):
        self.measured_bearing = msg.data
        self.get_logger().info(
            f'Measured bearing: {self.measured_bearing:.3f} rad'
        )
        self.last_measurement_time = self.get_clock().now()

    def landmark_pose_callback(self, msg):
        self.landmark_poses = msg
        self.get_logger().info(
            f'Received landmark poses: {len(self.landmark_poses.poses)}'
        )
        # self.calculate_errors()

    def ground_truth_callback(self, msg):
        self.robot_pose = msg
        self.get_logger().info(
            f'Received ground truth pose: ({self.robot_pose.pose.position.x:.3f}, {self.robot_pose.pose.position.y:.3f})'
        )

    def calculate_errors(self):
        if (self.measured_distance is None or self.measured_bearing is None or self.robot_pose is None or self.landmark_poses is None): 
            self.get_logger().warn(
                    'Waiting for measurements, robot pose, or landmark poses'
            )
            return

        if len(self.landmark_poses.poses) <= 5:
            return

        if self.measured_distance is None:
            self.get_logger().warn('Missing measured distance')
            return

        if self.measured_bearing is None:
            self.get_logger().warn('Missing measured bearing')
            return

        if self.robot_pose is None:
            self.get_logger().warn('Missing robot pose')
            return

        if self.landmark_poses is None:
            self.get_logger().warn('Missing landmark poses')
            return

        if self.last_measurement_time is None:
            return

        elapsed = (
            self.get_clock().now() - self.last_measurement_time
        ).nanoseconds / 1e9

        if elapsed > self.measurement_timeout:
            return

        landmark = self.landmark_poses.poses[self.landmark_index]

        landmark_x = landmark.position.x
        landmark_y = landmark.position.y

        robot_x = self.robot_pose.pose.position.x
        robot_y = self.robot_pose.pose.position.y

        q = self.robot_pose.pose.orientation

        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y**2 + q.z**2))

        dx = landmark_x - robot_x
        dy = landmark_y - robot_y

        # Transform landmark position into robot/camera frame
        landmark_robot_x = (
            math.cos(yaw) * dx + math.sin(yaw) * dy
        )
        
        landmark_robot_y = (
            -math.sin(yaw) * dx + math.cos(yaw) * dy
        )

        true_distance = math.sqrt(
            landmark_robot_x**2 +
            landmark_robot_y**2
        )

        distance_error = (self.measured_distance - true_distance)

        true_bearing = math.atan2(landmark_robot_y, landmark_robot_x)

        bearing_error = math.atan2(
            math.sin(self.measured_bearing - true_bearing),
            math.cos(self.measured_bearing - true_bearing)
        )

        distance_error_msg = Float64()
        distance_error_msg.data = distance_error
        self.distance_error_pub.publish(distance_error_msg)

        bearing_error_msg = Float64()
        bearing_error_msg.data = bearing_error
        self.bearing_error_pub.publish(bearing_error_msg)

        self.get_logger().info(
            f'For landmark={self.landmark_color}: '
            f'measured_distance={self.measured_distance:.6f},'
            f'measured_bearing={self.measured_bearing:.6f},'
            f'true_distance={true_distance:.6f},'
            f'true_bearing={true_bearing:.6f},'
            f'distance error = {distance_error:.3f} m, '
            f'bearing error = {bearing_error:.3f} rad '
            f'({math.degrees(bearing_error):.2f} deg)'
        )

    def spin(self):
        rclpy.spin(self)


def main():
    rclpy.init()
    landmark_measurement_error = LandmarkMeasurementError()
    landmark_measurement_error.spin()
    landmark_measurement_error.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

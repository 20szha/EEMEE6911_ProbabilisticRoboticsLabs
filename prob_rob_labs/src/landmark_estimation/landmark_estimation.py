import rclpy
from rclpy.node import Node
from prob_rob_msgs.msg import Point2DArrayStamped
import math
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float64

# heartbeat_period = 0.1

def estimate_distance_bearing(height_pixels, center_x, fx, fy, cx, landmark_height):

        if height_pixels <= 0:
            return None

        theta = math.atan((cx - center_x) / fx)

        distance = (fy * landmark_height / (height_pixels * math.cos(theta)))

        return distance, theta

class LandmarkEstimation(Node):

    def __init__(self):
        super().__init__('landmark_estimation')
        self.log = self.get_logger()
        # self.timer = self.create_timer(heartbeat_period, self.heartbeat)

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

    def camera_info_callback(self, msg):
        self.fx = msg.p[0]
        self.cx = msg.p[2]
        self.fy = msg.p[5]
        self.cy = msg.p[6]

        self.camera_info_received = True


    # def heartbeat(self):
    #     self.log.info('heartbeat')

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

    def spin(self):
        rclpy.spin(self)


def main():
    rclpy.init()
    landmark_estimation = LandmarkEstimation()
    landmark_estimation.spin()
    landmark_estimation.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

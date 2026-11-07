import rclpy
from rclpy.node import Node
from prob_rob_msgs.msg import Point2DArrayStamped

heartbeat_period = 0.1

class LandmarkEstimation(Node):

    def __init__(self):
        super().__init__('landmark_estimation')
        self.log = self.get_logger()
        self.timer = self.create_timer(heartbeat_period, self.heartbeat)

        self.declare_parameter('landmark_color', 'cyan')   

        self.landmark_color = self.get_parameter('landmark_color').value

        topic = f'/vision_{self.get_parameter("landmark_color").value}/corners'                         # topic of the corners of the landmark to listen to

        self.vision_corners = self.create_subscription(
            Point2DArrayStamped,
            topic,                                                                                      # topic to listen to
            self.vision_corners_callback,
            10
        )


    def heartbeat(self):
        self.log.info('heartbeat')

    def vision_corners_callback(self, msg):
        if not msg.points:
            self.get_logger().debug(
                f'No corners detected for {self.landmark_color}'
            )
            return

        min_x = min(point.x for point in msg.points)
        max_x = max(point.x for point in msg.points)
        min_y = min(point.y for point in msg.points)
        max_y = max(point.y for point in msg.points)

        self.get_logger().info(
            f'{self.landmark_color}: min_x={min_x:.2f}, max_x={max_x:.2f}, '
            f'min_y={min_y:.2f}, max_y={max_y:.2f}, '
            f'height={max_y - min_y:.2f}, '
            f'vertical symmetry axis={(min_x + max_x) / 2.0:.2f} '
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

import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import TwistStamped

from pathlib import Path

heartbeat_period = 0.1

class CollectData(Node):

    def __init__(self):
        super().__init__('collect_data')
        self.log = self.get_logger()

        self.timer = self.create_timer(heartbeat_period, self.heartbeat)

        self.declare_parameter('duration_sec', 40.0)
        self.declare_parameter('output_file_linear', '/home/szhang2002/ros2_ws/src/prob_rob_labs_ros_2/prob_rob_labs/output/x_data.txt')
        self.declare_parameter('output_file_angular', '/home/szhang2002/ros2_ws/src/prob_rob_labs_ros_2/prob_rob_labs/output/z_data.txt')

        self.duration_sec = self.get_parameter('duration_sec').value
        self.output_file_linear = self.get_parameter('output_file_linear').value
        self.output_file_angular = self.get_parameter('output_file_angular').value

        self.start_time = time.monotonic()
        self.count = 0
        self.finished = False

        Path(self.output_file_linear).parent.mkdir(parents=True, exist_ok=True)
        Path(self.output_file_angular).parent.mkdir(parents=True, exist_ok=True)

        self.output_file_linear = open(self.output_file_linear, 'w')    
        self.output_file_angular = open(self.output_file_angular, 'w')                                     

        self.create_subscription(                                                                                        # subscription to the linear and angular velocity of the robot's twist
            TwistStamped, '/tb3/ground_truth/twist', self.callback, 10
        )

        self.log.info(
            f'Recording /tb3/ground_truth/twist for {self.duration_sec:.1f}s '
            f'to {self.output_file_linear} and {self.output_file_angular}'
        )

    def callback(self, msg):
        if time.monotonic() - self.start_time >= self.duration_sec:
            return

        self.output_file_linear.write(f'{msg.twist.linear.x:.6f}\n')
        self.output_file_angular.write(f'{msg.twist.angular.z:.6f}\n')

        self.count += 1

        self.output_file_linear.flush()
        self.output_file_angular.flush()

    def heartbeat(self):
        self.log.info('heartbeat')

        if time.monotonic() - self.start_time < self.duration_sec:
            return

        self.timer.cancel()
        self.output_file_linear.close()
        self.output_file_angular.close()

        self.get_logger().info(
            f'Recording complete. Data saved to {self.output_file_linear} and {self.output_file_angular}'
        )

    def spin(self):
        rclpy.spin(self)


def main():
    rclpy.init()
    collect_data = CollectData()
    collect_data.spin()
    collect_data.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import TwistStamped

from pathlib import Path

heartbeat_period = 0.01

class CollectDataA3(Node):

    def __init__(self):
        super().__init__('collect_data_a3')
        self.log = self.get_logger()

        self.timer = self.create_timer(heartbeat_period, self.heartbeat)

        self.declare_parameter('duration_sec', 40.0)
        self.declare_parameter('experiment', 'linear')
        self.declare_parameter('target_linear', 3)
        self.declare_parameter('target_angular', 3)
        self.declare_parameter('command_delay', 2.0)
        self.declare_parameter('output_file', '/home/szhang2002/ros2_ws/src/prob_rob_labs_ros_2/prob_rob_labs/output/x_linear_data.txt')
        
        self.experiment = self.get_parameter('experiment').value
        self.target_linear = self.get_parameter('target_linear').value
        self.target_angular = self.get_parameter('target_angular').value
        self.command_delay = self.get_parameter('command_delay').value
        self.duration_sec = self.get_parameter('duration_sec').value
        self.output_file = self.get_parameter('output_file').value

        self.start_time = time.monotonic()
        self.count = 0
        self.finished = False

        Path(self.output_file).parent.mkdir(parents=True, exist_ok=True)

        self.output_file = open(self.output_file, 'w')            

        self.output_file.write('time_sec,velocity\n')                           

        self.pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)

        self.sub = self.create_subscription(                                                                                        # subscription to the linear and angular velocity of the robot's twist
            TwistStamped, '/tb3/ground_truth/twist', self.callback, 10
        )

        self.log.info(
            f'Recording /tb3/ground_truth/twist for {self.duration_sec:.1f}s '
            f'to {self.output_file} and {self.output_file}'
        )

    def publish_command(self, linear, angular):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = float(linear)
        msg.twist.angular.z = float(angular)
        self.pub.publish(msg)

    def callback(self, msg):
        if self.finished:
            return

        elapsed = time.monotonic() - self.start_time

        if time.monotonic() - self.start_time >= self.duration_sec:
            return

        self.output_file.write(f'{elapsed:.6f}, {msg.twist.linear.x:.6f}\n')

        self.count += 1

        self.output_file.flush()

    def heartbeat(self):
        elapsed = time.monotonic() - self.start_time

        if elapsed >= self.duration_sec:
            self.finish()
            return

        # Wait before applying the velocity command.
        if elapsed < self.command_delay:
            self.publish_command(0.0, 0.0)
            return

        # Run only one experiment at a time.
        if self.experiment == 'linear':
            self.publish_command(self.target_linear, 0.0)

        elif self.experiment == 'angular':
            self.publish_command(0.0, self.target_angular)

        else:
            self.get_logger().error(
                f'Unknown experiment: {self.experiment}'
            )
            self.finish()


    def finish(self):
        if self.finished:
            return

        self.finished = True

        self.publish_command(0.0, 0.0)

        self.timer.cancel()

        self.output_file.close()

        self.get_logger().info(
            f'Recording complete: {self.count} samples. '
            f'Data saved to: {self.output_file}'
        )

    def spin(self):
        rclpy.spin(self)

def main():
    rclpy.init()
    collect_data_a3 = CollectDataA3()
    collect_data_a3.spin()
    collect_data_a3.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

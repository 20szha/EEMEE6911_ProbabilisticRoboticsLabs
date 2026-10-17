import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

heartbeat_period = 0.1

class NoisyFeatureRecorder(Node):

    def __init__(self):
        super().__init__('noisy_feature_recorder')
        self.log = self.get_logger()

        self.declare_parameter('duration_sec', 40.0)
        self.declare_parameter('threshold', 321.55)
        self.declare_parameter('output_file', 'feature_mean_data.txt')

        self.duration_sec = self.get_parameter('duration_sec').value
        self.threshold = self.get_parameter('threshold').value
        self.output_file = self.get_parameter('output_file').value

        self.count = 0
        self.open_count = 0
        self.closed_count = 0

        self.start_time = time.monotonic()
        self.file = open(self.output_file, 'w')

        self.create_subscription(
            Float64, '/feature_mean', self.callback, 10
        )

        self.timer = self.create_timer(heartbeat_period, self.heartbeat)

        self.log.info(
            f'Recording /feature_mean for {self.duration_sec:.1f}s '
            f'to {self.output_file}'
        )

    def callback(self, msg):
        if time.monotonic() - self.start_time >= self.duration_sec:
            return

        self.file.write(f'{msg.data:.4f}\n')
        if msg.data <= self.threshold:
            self.open_count += 1
        else:
            self.closed_count += 1

        self.count += 1

    def heartbeat(self):
        if time.monotonic() - self.start_time < self.duration_sec:
            return

        self.timer.cancel()
        self.file.flush()
        self.file.close()

        total_samples = (
            self.open_count + self.closed_count
        )

        self.get_logger().info(
            f'Recording complete. Data saved to {self.output_file}'
        )

        self.get_logger().info(
            f'threshold={self.threshold:.4f}; '
            f'z=open (value <= threshold): '
            f'{self.open_count}; '
            f'z=closed (value > threshold): '
            f'{self.closed_count}; total={total_samples}'
        )

        # rclpy.shutdown()

    def spin(self):
        rclpy.spin(self)


def main():
    rclpy.init()
    noisy_feature_recorder = NoisyFeatureRecorder()
    noisy_feature_recorder.spin()
    noisy_feature_recorder.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

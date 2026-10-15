import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class FeatureRecorder(Node):
    def __init__(self):
        super().__init__('feature_recorder')

        self.start_time = time.monotonic()
        self.file = open('feature_mean_data.txt', 'w')

        self.create_subscription(
            Float64, '/feature_mean', self.callback, 10
        )

        self.create_timer(0.1, self.check_time)

    def callback(self, msg):
        elapsed = time.monotonic() - self.start_time
        if elapsed < 40:
            self.file.write(f'{msg.data:.4f}\n')

    def check_time(self):
        if time.monotonic() - self.start_time >= 40:
            self.file.close()
            print('Recording complete. Data saved to feature_mean_data.txt')
            raise SystemExit

def main():
    rclpy.init()
    node = FeatureRecorder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
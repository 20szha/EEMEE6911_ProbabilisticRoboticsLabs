import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from message_filters import Subscriber, ApproximateTimeSynchronizer


heartbeat_period = 0.1

class EKFOdometry(Node):

    def __init__(self):
        super().__init__('EKFOdometry')
        self.log = self.get_logger()

        self.joint_sub = Subscriber(
            self,
            JointState,
            '/joint_states'
        )

        self.imu_sub = Subscriber(
            self,
            Imu,
            '/imu'
        )

        self.sync = ApproximateTimeSynchronizer(
            [self.joint_sub, self.imu_sub],
            queue_size=20,
            slop=0.05
        )

        self.sync.registerCallback(self.synced_callback)
        self.get_logger().info('Waiting for synchronized sensor messages...')

        # self.timer = self.create_timer(heartbeat_period, self.heartbeat)

    def synced_callback(self, joint_msg, imu_msg):

        joint_time = (
            joint_msg.header.stamp.sec +
            joint_msg.header.stamp.nanosec * 1e-9
        )

        imu_time = (
            imu_msg.header.stamp.sec +
            imu_msg.header.stamp.nanosec * 1e-9
        )

        difference = abs(joint_time - imu_time)

        self.get_logger().info(
            f'Synchronized! Timestamp difference: {difference:.6f} s'
        )

    def heartbeat(self):
        self.log.info('heartbeat')

    def spin(self):
        rclpy.spin(self)


def main():
    rclpy.init()
    ekfodometry = EKFOdometry()
    ekfodometry.spin()
    ekfodometry.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

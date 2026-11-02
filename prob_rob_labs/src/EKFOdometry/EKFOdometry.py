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

        self.rw = 0.033
        self.R = 0.1435 / 2.0
        self.C = np.array([
            [0, 0, 0, 1.0/self.rw,  self.R/self.rw],
            [0, 0, 0, 1.0/self.rw, -self.R/self.rw],
            [0, 0, 0, 0, 1],
        ], dtype=float)

        self.x = np.zeros(5)
        self.P = np.diag([0.01, 0.01, 0.01, 0.10, 0.10])
        # Example process-noise strengths per second; tune experimentally.
        self.q_diag = np.array([0.001, 0.001, 0.001, 0.05, 0.05])
        self.last_sensor_time = None
        self.u = np.zeros(2)
        self.last_cmd_received = None

        self.cmd_sub = self.create_subscription(
            TwistStamped, '/cmd_vel', self.cmd_callback, 10)

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

        # difference = abs(joint_time - imu_time)

        sensor_time = joint_time

        if self.last_sensor_time is None:                                                               # Calculate time elapsed since previous sensor update
            self.last_sensor_time = sensor_time
            return

        dt = sensor_time - self.last_sensor_time

        self.last_sensor_time = sensor_time                                                             # Update timestamp for next iteration

        if dt <= 0:
            return

        # self.get_logger().info(
        #     f'Synchronized! Timestamp difference: {difference:.6f} s'
        # )

        self.u = self.get_control_input()                                                               # Get current command

        z = np.array([
            joint_msg.velocity[1],                                                                      # Right wheel (rad/s)
            joint_msg.velocity[0],                                                                      # Left wheel (rad/s)
            imu_msg.angular_velocity.z                                                                  # Gyroscope (rad/s)
        ])

        self.get_logger().info(f'dt: {dt:.4f}, Command vector u: {self.u},Measurement vector z: {z}')

    def cmd_callback(self, msg):
        self.u[:] = (msg.twist.linear.x, msg.twist.angular.z)                                           # Store the command velocities
        
        self.last_cmd_received = self.get_clock().now().nanoseconds * 1e-9                              # Record when the callback receives the command, receipt time uses the simulation time to count how long ago the robot received the command

    def get_control_input(self):                                                                        # Make sure EKF uses most recent command until expiration, and switch to zero velocity if expired
        if self.last_cmd_received is None:                                                              # No command has arrived yet
            return np.zeros(2)

        current_time = self.get_clock().now().nanoseconds * 1e-9                                        # Current simulation time

        if current_time - self.last_cmd_received > 1.0:                                                 # Check the 1-second safety timeout
            return np.zeros(2)

        return self.u.copy()                                                                            # Command is still valid

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

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

        self.declare_parameter('tau_v', 0.621855)
        self.declare_parameter('tau_w', 0.669411)
        self.declare_parameter('gain_v', 0.999628)
        self.declare_parameter('gain_w', 0.995552)
        self.declare_parameter('encoder_variance', 0.04)
        self.declare_parameter('gyro_variance', 0.01)
        self.declare_parameter('sync_slop', 0.05)
        self.declare_parameter('command_timeout', 1.0)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')

        self.tau_v = float(self.get_parameter('tau_v').value)
        self.tau_w = float(self.get_parameter('tau_w').value)
        self.gain_v = float(self.get_parameter('gain_v').value)
        self.gain_w = float(self.get_parameter('gain_w').value)
        self.encoder_var = float(self.get_parameter('encoder_variance').value)
        self.gyro_var = float(self.get_parameter('gyro_variance').value)
        self.cmd_timeout = float(self.get_parameter('command_timeout').value)
        self.sync_slop = float(self.get_parameter('sync_slop').value)
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        if min(self.tau_v, self.tau_w, self.encoder_var, self.gyro_var) <= 0:
            raise ValueError('tau_v, tau_w, encoder_variance and gyro_variance must be > 0')

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
            slop=self.sync_slop
        )

        self.sync.registerCallback(self.synced_callback)
        self.get_logger().info('Waiting for synchronized sensor messages...')

        self.odom_pub = self.create_publisher(
            Odometry,
            '/ekf_odom',
            10
        )
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

        self.get_logger().info(
            f'Joint time: {joint_time:.6f}, '
            f'IMU time: {imu_time:.6f}, '
            f'Difference: {abs(joint_time - imu_time):.6f}'
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

        self.predict(dt, self.u)                                                                        # EKF prediction step

        z = np.array([                                                                                  # Measurement vector
            joint_msg.velocity[1],                                                                      # Right wheel (rad/s)
            joint_msg.velocity[0],                                                                      # Left wheel (rad/s)
            imu_msg.angular_velocity.z                                                                  # Gyroscope (rad/s)
        ])

        imu_var = imu_msg.angular_velocity_covariance[8]
        if not math.isfinite(imu_var) or imu_var <= 0:
            imu_var = self.gyro_var
        R = np.diag([self.encoder_var, self.encoder_var, imu_var])

        self.update(z, R)

        self.publish_odometry(joint_msg.header.stamp)

        self.get_logger().info(f'dt: {dt:.4f}, Command vector u: {self.u}, Measurement vector z: {z}')
        self.get_logger().info(f'Updated state: {self.x}')

        omega_enc = (
            self.rw * (z[0] - z[1])
            / (2.0 * self.R)
        )

        self.get_logger().info(
            f'Wheel omega: {omega_enc:.5f}, '
            f'Gyro omega: {z[2]:.5f}, '
            f'Ratio: {omega_enc / z[2]:.3f}'
        )

    def cmd_callback(self, msg):
        self.u[:] = (msg.twist.linear.x, msg.twist.angular.z)                                           # Store the command velocities
        
        self.last_cmd_received = self.get_clock().now().nanoseconds * 1e-9                              # Record when the callback receives the command, receipt time uses the simulation time to count how long ago the robot received the command

    def get_control_input(self):                                                                        # Make sure EKF uses most recent command until expiration, and switch to zero velocity if expired
        if self.last_cmd_received is None:                                                              # No command has arrived yet
            return np.zeros(2)

        current_time = self.get_clock().now().nanoseconds * 1e-9                                        # Current simulation time

        if current_time - self.last_cmd_received > self.cmd_timeout:                                    # Check the 1-second safety timeout
            return np.zeros(2)

        return self.u.copy()                                                                            # Command is still valid

    def predict(self, dt, u):
        theta, x, y, v, omega = self.x
        uv, uw = u

        av = 0.1 ** (dt / self.tau_v)
        aw = 0.1 ** (dt / self.tau_w)


        x_pred = np.array([                                                                             # State transition
            theta + omega * dt,
            x + v * math.cos(theta) * dt,
            y + v * math.sin(theta) * dt,
            av * v + self.gain_v * (1.0 - av) * uv,
            aw * omega + self.gain_w * (1.0 - aw) * uw
        ])
        
        F = np.array([                                                                                  # State-transition Jacobian
            [1, 0, 0, 0, dt],
            [-v * math.sin(theta) * dt, 1, 0, math.cos(theta) * dt, 0],
            [v * math.cos(theta) * dt, 0, 1, math.sin(theta) * dt, 0],
            [0, 0, 0, av, 0],
            [0, 0, 0, 0, aw]
        ])

        Q = np.diag(self.q_diag * dt)                                                                   # Process covariance - need to scale by dt

        P_pred = F @ self.P @ F.T + Q                                                                   # Covariance prediction

        self.x = x_pred
        self.P = P_pred

        self.log.info(f'Predicted state: {self.x}')

    def update(self, z, R):
        # Expected measurement from the predicted state
        z_pred = self.C @ self.x

        # Measurement residual
        y = z - z_pred

        # Innovation covariance
        S = self.C @ self.P @ self.C.T + R

        # Kalman gain
        K = self.P @ self.C.T @ np.linalg.inv(S)

        # Correct the state estimate
        self.x = self.x + K @ y

        # Correct the covariance (Joseph form)
        I = np.eye(5)
        IKC = I - K @ self.C
        self.P = IKC @ self.P @ IKC.T + K @ R @ K.T

    def publish_odometry(self, stamp):
        theta, x, y, v, omega = self.x

        msg = Odometry()

        msg.header.stamp = stamp                                                                        # Timestamp from the synchronized sensor message   

        msg.header.frame_id = self.odom_frame                                                           # Coordinate frames (we used ros2 topic echo /odom and saw frame_id: odom and child_frame_id: base_footprint so we are following it)
        msg.child_frame_id = self.base_frame

        # Position
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0

        # Orientation: planar yaw converted to quaternion
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        msg.pose.pose.orientation.w = math.cos(theta / 2.0)

        # Linear and angular velocity
        msg.twist.twist.linear.x = float(v)
        msg.twist.twist.angular.z = float(omega)

        # Pose covariance: EKF state order is [theta, x, y, v, omega]
        pose_indices = [5, 0, 1]
        for i in range(3):
            for j in range(3):
                msg.pose.covariance[pose_indices[i] * 6 + pose_indices[j]] = float(
                    self.P[[0, 1, 2][i], [0, 1, 2][j]]
                )

        # Twist covariance: EKF velocity states are [v, omega]
        msg.twist.covariance[0] = float(self.P[3, 3])
        msg.twist.covariance[5] = float(self.P[3, 4])
        msg.twist.covariance[30] = float(self.P[4, 3])
        msg.twist.covariance[35] = float(self.P[4, 4])

        self.odom_pub.publish(msg)

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

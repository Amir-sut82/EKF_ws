import math
import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped, Quaternion  # *** اضافه شد


from tf2_ros import TransformBroadcaster
from tf_transformations import euler_from_quaternion, quaternion_from_euler


class EkfNode(Node):
    def __init__(self):
        super().__init__('ekf_node')

        # parameters
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('time_step', 0.1)

        # read parameters
        self.odom_frame = self.get_parameter('odom_frame_id').value
        self.base_frame = self.get_parameter('base_frame_id').value
        self.dt = float(self.get_parameter('time_step').value)

        # subscribers
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Odometry, '/measurement/odom', self.measurement_callback, 10)

        # EKF parameters
        self.R = np.diag([2.0e-3, 2.0e-3, 1.2e-2])  # process (yaw بزرگ‌تر)
        self.Q = np.diag([4.0e-4, 4.0e-4, 7.0e-4])  # measurement

        self.mu = np.zeros((3, 1))
        self.Sigma = np.eye(3) * 1.0
        self.vx = 0.0
        self.wz = 0.0
        self.est_x = 0.0
        self.est_y = 0.0

        
        self.est = Quaternion()
        self.est.x = 0.0
        self.est.y = 0.0
        self.est.z = 0.0
        self.est.w = 1.0   

        # publisher
        self.ekf_pub = self.create_publisher(Odometry, 'ekf/odom', 10)

        # tf broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # timer
        self.create_timer(self.dt, self.step)

        # log info
        self.get_logger().info('ekf started')
        

    def cmd_vel_callback(self, msg):
        self.vx = msg.linear.x
        self.wz = msg.angular.z

    def measurement_callback(self, msg):
        self.est_x = msg.pose.pose.position.x
        self.est_y = msg.pose.pose.position.y
        self.est = msg.pose.pose.orientation  

    def step(self):

        # Prediction step
        yaw = self.mu[2, 0]

        # state prediction using simple unicycle model
        self.mu[0, 0] += self.vx * math.cos(yaw) * self.dt
        self.mu[1, 0] += self.vx * math.sin(yaw) * self.dt
        self.mu[2, 0] += self.wz * self.dt

        # normalize yaw
        self.mu[2, 0] = math.atan2(math.sin(self.mu[2, 0]), math.cos(self.mu[2, 0]))

        # Jacobian of motion model
        G = np.array([
            [1.0, 0.0, -self.vx * math.sin(yaw) * self.dt],
            [0.0, 1.0,  self.vx * math.cos(yaw) * self.dt],
            [0.0, 0.0,  1.0]
        ])

        # covariance prediction
        self.Sigma = G @ self.Sigma @ G.T + self.R

        # Measurement step
        z = np.zeros((3, 1))

     
        z[0, 0] = self.est_x
        z[1, 0] = self.est_y

        q = self.est
        _, _, z[2, 0] = euler_from_quaternion([q.x, q.y, q.z, q.w])

        # measurement matrix 
        H = np.eye(3)
        K = self.Sigma @ H.T @ np.linalg.inv(H @ self.Sigma @ H.T + self.Q)

        innovation = z - self.mu

        # normalize yaw innovation
        innovation[2, 0] = math.atan2(
            math.sin(innovation[2, 0]),
            math.cos(innovation[2, 0])
        )

        # state update
        self.mu = self.mu + K @ innovation
        self.mu[2, 0] = math.atan2(math.sin(self.mu[2, 0]), math.cos(self.mu[2, 0]))

        # covariance update
        self.Sigma = (np.eye(3) - K @ H) @ self.Sigma



        # publish fused state
        self.publish()

     # Publish fused odom + TF
    def publish(self):
        now = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = float(self.mu[0, 0])
        odom.pose.pose.position.y = float(self.mu[1, 0])

        q = quaternion_from_euler(0.0, 0.0, float(self.mu[2, 0]))
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        odom.pose.covariance[0] = float(self.Sigma[0, 0])
        odom.pose.covariance[7] = float(self.Sigma[1, 1])
        odom.pose.covariance[35] = float(self.Sigma[2, 2])

        self.ekf_pub.publish(odom)

        # TF
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = float(self.mu[0, 0])
        t.transform.translation.y = float(self.mu[1, 0])
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = EkfNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

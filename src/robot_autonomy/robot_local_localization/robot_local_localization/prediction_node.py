import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler
from std_msgs.msg import Float64MultiArray  


class PredictionNode(Node):
    def __init__(self):
        super().__init__('prediction_node')

        # parameters
        self.declare_parameter('wheel_separation', 0.46)
        self.declare_parameter('wheel_radius', 0.10)
        self.declare_parameter('time_step', 0.1)
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw', 0.0)
        self.declare_parameter('initial_linear_vel', 0.0)
        self.declare_parameter('initial_angular_vel', 0.0)
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')


        # read parameters
        self.wheel_separation = float(self.get_parameter('wheel_separation').value)
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.dt = float(self.get_parameter('time_step').value)
        self.x = float(self.get_parameter('initial_x').value)
        self.y = float(self.get_parameter('initial_y').value)
        self.yaw = float(self.get_parameter('initial_yaw').value)
        self.vx = float(self.get_parameter('initial_linear_vel').value)
        self.wz = float(self.get_parameter('initial_angular_vel').value)
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value

        # subscriber
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback,10)

        # publisher
        self.odom_pub = self.create_publisher(Odometry, '/prediction/odom', 10)

        # timer
        self.create_timer(self.dt, self.update_prediction)
        
        # log info
        self.get_logger().info('prediction started')


    def cmd_vel_callback(self, msg):
        self.vx = msg.linear.x
        self.wz = msg.angular.z

    def update_prediction(self):
        # integrate motion model
        self.x += self.vx * math.cos(self.yaw) * self.dt
        self.y += self.vx * math.sin(self.yaw) * self.dt
        self.yaw += self.wz * self.dt

        # normalize yaw
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        # Odometry
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y

        q = quaternion_from_euler(0.0, 0.0, self.yaw)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.wz

        self.odom_pub.publish(odom)




def main(args=None):
    rclpy.init(args=args)
    node = PredictionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

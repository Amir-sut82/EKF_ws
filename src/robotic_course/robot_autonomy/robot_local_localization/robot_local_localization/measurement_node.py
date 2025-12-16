import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry


class MeasurementNode(Node):
    def __init__(self):
        super().__init__('measurement_node')

        # parameters
        self.declare_parameter('vo_odom_topic', '/vo/odom')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('time_step', 0.1)

        # read parameters
        self.vo_odom_topic = self.get_parameter('vo_odom_topic').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.dt = float(self.get_parameter('time_step').value)
        self.last_imu = None
        self.last_vo = None

        # subscribers
        self.create_subscription(Imu, '/zed/zed_node/imu/data_raw', self.imu_callback, 10)
        self.create_subscription(Odometry, self.vo_odom_topic, self.vo_callback, 10)

        # publisher
        self.meas_pub = self.create_publisher(Odometry, 'measurement/odom', 10)

        # timer
        self.create_timer(self.dt, self.publish_measurement)

        self.get_logger().info('measurement_node started')

    def imu_callback(self, msg):
        self.last_imu = msg

    def vo_callback(self, msg):
        self.last_vo = msg

    def publish_measurement(self):
        # wait until both sensors have data
        if self.last_imu is None or self.last_vo is None:
            return

        meas = Odometry()
        meas.header.stamp = self.last_vo.header.stamp
        meas.header.frame_id = self.odom_frame_id
        meas.child_frame_id = self.base_frame_id

        # position and linear velocity from visual odometry
        meas.pose.pose.position = self.last_vo.pose.pose.position
        meas.twist.twist.linear = self.last_vo.twist.twist.linear

        # orientation and angular velocity from imu
        meas.pose.pose.orientation = self.last_imu.orientation
        meas.twist.twist.angular = self.last_imu.angular_velocity

        self.meas_pub.publish(meas)

def main(args=None):
    rclpy.init(args=args)
    node = MeasurementNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

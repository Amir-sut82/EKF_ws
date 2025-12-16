import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from rclpy.qos import QoSPresetProfiles
import copy


class PointCloudFrameIdConverter(Node):
    def __init__(self):
        super().__init__('frame_id_converter_node')

        # Publisher
        self.pub_ = self.create_publisher(
            PointCloud2,
            '/points',
            QoSPresetProfiles.SENSOR_DATA.value   
        )

        # Subscriber
        self.sub_ = self.create_subscription(
            PointCloud2,
            '/gz_lidar/points',
            self.pointCloudCallback,
            QoSPresetProfiles.SENSOR_DATA.value
        )

        self.get_logger().info(
            'PointCloud FrameIdConverter started. Listening to /gz_lidar/points...'
        )

    def pointCloudCallback(self, msg: PointCloud2):
        new_msg = copy.deepcopy(msg)           
        new_msg.header.frame_id = 'rplidar_c1' 
        self.pub_.publish(new_msg)             


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudFrameIdConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

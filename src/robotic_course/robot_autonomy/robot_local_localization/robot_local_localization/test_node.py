import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path

from tf2_msgs.msg import TFMessage
from tf_transformations import quaternion_from_euler  

import matplotlib.pyplot as plt


class RectTestNode(Node):

    def __init__(self):
        super().__init__('rect_test_node')

        # PARAMETERS
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('dt', 3)
        self.declare_parameter('linear_speed', 1)   # m/s
        self.declare_parameter('angular_speed', 0.705)  # rad/s
        self.declare_parameter('distance', 0.5)        # m
        self.declare_parameter('angle_deg', 90.0)      # degree
        self.declare_parameter('enable_rect_cmd', True)

        self.declare_parameter('time_scale', 2.0)
        self.time_scale = float(self.get_parameter('time_scale').value)
        self.declare_parameter('gt_child_frame', 'robot')
        self.gt_child_frame = self.get_parameter('gt_child_frame').value

        self.odom_frame = self.get_parameter('odom_frame_id').value
        self.dt = float(self.get_parameter('dt').value)
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.distance = float(self.get_parameter('distance').value)

        self.declare_parameter('side3_time_scale', 1.5)
        self.side3_time_scale = float(self.get_parameter('side3_time_scale').value)

        self.declare_parameter('side4_time_scale', 3)
        self.side4_time_scale = float(self.get_parameter('side4_time_scale').value)

        self.declare_parameter('side5_time_scale', 1.2)
        self.side5_time_scale = float(self.get_parameter('side5_time_scale').value)

        self.angle_deg = float(self.get_parameter('angle_deg').value)
        self.angle = math.radians(self.angle_deg)

        self.enable_rect_cmd = bool(self.get_parameter('enable_rect_cmd').value)

        # Paths
        self.ekf_path = Path()
        self.ekf_path.header.frame_id = self.odom_frame

        self.meas_path = Path()  
        self.meas_path.header.frame_id = self.odom_frame

        self.pred_path = Path()  
        self.pred_path.header.frame_id = self.odom_frame

        self.gt_path = Path()    
        self.gt_path.header.frame_id = self.odom_frame

        
        self.ekf_xy = []
        self.gt_xy = []
        self.meas_xy = []  
        self.pred_xy = []  

        # Subscriptions
        self.create_subscription(Odometry, '/ekf/odom', self.ekf_callback, 10)
        self.create_subscription(Odometry, '/measurement/odom', self.meas_callback, 10)  
        self.create_subscription(Odometry, '/prediction/odom', self.pred_callback, 10)   
        self.create_subscription(TFMessage, '/world/depot/dynamic_pose/info', self.ground_callback, 10)

        # Publishers 
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.ekf_path_pub = self.create_publisher(Path, '/ekf_path', 10)
        self.meas_path_pub = self.create_publisher(Path, '/measurement_path', 10)  
        self.pred_path_pub = self.create_publisher(Path, '/prediction_path', 10)   
        self.gt_path_pub = self.create_publisher(Path, '/gt_path', 10)             

        self.rect_state = 0
        self.state_elapsed = 0.0

        # Timer
        self.timer = self.create_timer(self.dt, self.timer_cb)

        self.get_logger().info(
            f'rect_test_node started | dt={self.dt} | v={self.linear_speed} | w={self.angular_speed} | '
            f'distance={self.distance} | angle_deg={self.angle_deg} | time_scale={self.time_scale} | '
            f'gt_child_frame={self.gt_child_frame}'
        )

    def append_to_path(self, path_msg: Path, odom_msg: Odometry):
        pose = PoseStamped()
        pose.header = odom_msg.header
        pose.pose = odom_msg.pose.pose
        path_msg.header.stamp = odom_msg.header.stamp
        path_msg.poses.append(pose)

    def append_pose_to_path(self, path_msg: Path, stamp, x: float, y: float, yaw: float = 0.0):
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.odom_frame
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        q = quaternion_from_euler(0.0, 0.0, float(yaw))
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        path_msg.header.stamp = stamp
        path_msg.poses.append(pose)

    # Odom callbacks
    def ekf_callback(self, msg: Odometry):
        self.append_to_path(self.ekf_path, msg)
        self.ekf_path_pub.publish(self.ekf_path)
        self.ekf_xy.append((msg.pose.pose.position.x, msg.pose.pose.position.y))

    def meas_callback(self, msg: Odometry):  
        self.append_to_path(self.meas_path, msg)
        self.meas_path_pub.publish(self.meas_path)
        self.meas_xy.append((msg.pose.pose.position.x, msg.pose.pose.position.y))

    def pred_callback(self, msg: Odometry):  
        self.append_to_path(self.pred_path, msg)
        self.pred_path_pub.publish(self.pred_path)
        self.pred_xy.append((msg.pose.pose.position.x, msg.pose.pose.position.y))

    # Ground truth callback
    def ground_callback(self, msg: TFMessage):
        for t in msg.transforms:
            if t.child_frame_id == self.gt_child_frame:
                x = t.transform.translation.x
                y = t.transform.translation.y
                stamp = t.header.stamp
                self.gt_xy.append((x, y))
                self.append_pose_to_path(self.gt_path, stamp, x, y, yaw=0.0) 
                self.gt_path_pub.publish(self.gt_path)
                return

    # State durations
    def straight_duration(self):
        if self.linear_speed <= 0.0:
            return 0.0

        base = (self.distance / self.linear_speed) * self.time_scale

        if self.rect_state == 4:
            return base * self.side3_time_scale
        if self.rect_state == 6:
            return base * self.side4_time_scale
        if self.rect_state == 7:
            return base * self.side5_time_scale
        return base

    def turn_duration(self):
        if self.angular_speed == 0.0:
            return 0.0
        return abs(self.angle / self.angular_speed) * self.time_scale

    def is_straight_state(self, s: int) -> bool:
        return s in (0, 2, 4, 6)

    # Timer
    def timer_cb(self):
        if not self.enable_rect_cmd:
            return

        if self.rect_state >= 8:
            cmd = Twist()
            self.cmd_pub.publish(cmd)
            self.enable_rect_cmd = False
            self.get_logger().info('Rectangle path finished. Stopping commands.')
            return

        cmd = Twist()

        if self.is_straight_state(self.rect_state):
            cmd.linear.x = self.linear_speed
            cmd.angular.z = 0.0
            state_time_limit = self.straight_duration()
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = self.angular_speed
            state_time_limit = self.turn_duration()

        self.cmd_pub.publish(cmd)

        self.state_elapsed += self.dt

        if state_time_limit > 0.0 and self.state_elapsed >= state_time_limit:
            self.rect_state += 1
            self.state_elapsed = 0.0
            self.get_logger().info(f'Switching to rect_state={self.rect_state}')

def plot_paths(node: RectTestNode):
    plt.figure(figsize=(8, 8))

    def draw_arrows(x, y, every=30, scale=1.0, alpha=0.7):
        if len(x) < every + 1:
            return
        for i in range(0, len(x) - every, every):
            dx = (x[i + every] - x[i]) * scale
            dy = (y[i + every] - y[i]) * scale
            plt.arrow(
                x[i], y[i], dx, dy,
                width=0.004,
                head_width=0.05,
                length_includes_head=True,
                alpha=alpha,
                zorder=5
            )

    # EKF
    if node.ekf_xy:
        xe, ye = zip(*node.ekf_xy)
        plt.plot(
            xe, ye,
            linestyle='-',
            linewidth=2.8,
            marker='o',
            markersize=2.5,
            markevery=max(1, len(xe)//60),
            label='EKF',
            zorder=30
        )
        plt.scatter(xe[0], ye[0], s=120, zorder=60, label='Start (EKF)')
        plt.scatter(xe[-1], ye[-1], s=120, zorder=60, label='End (EKF)')
        draw_arrows(xe, ye, every=max(10, len(xe)//25), alpha=0.5)

    # Measurement
    if node.meas_xy:
        xm, ym = zip(*node.meas_xy)
        plt.plot(
            xm, ym,
            linestyle='--',
            linewidth=2.0,
            marker='x',
            markersize=3,
            markevery=max(1, len(xm)//70),
            label='Measurement',
            alpha=0.9,
            zorder=20
        )
        draw_arrows(xm, ym, every=max(10, len(xm)//30), alpha=0.35)

    # Prediction
    if node.pred_xy:
        xp, yp = zip(*node.pred_xy)
        plt.plot(
            xp, yp,
            linestyle=':',
            linewidth=2.2,
            marker='s',
            markersize=2.5,
            markevery=max(1, len(xp)//70),
            label='Prediction',
            alpha=0.95,
            zorder=15
        )
        draw_arrows(xp, yp, every=max(10, len(xp)//30), alpha=0.35)

    
    if node.gt_xy:
        xg, yg = zip(*node.gt_xy)
        plt.plot(
            xg, yg,
            linestyle='-',
            linewidth=4.5,
            alpha=0.9,
            label='Ground Truth',
            zorder=10
        )
        
        plt.scatter(xg[0], yg[0], s=90, zorder=40, label='Start (GT)')
        plt.scatter(xg[-1], yg[-1], s=90, zorder=40, label='End (GT)')

  
    plt.xlabel('x [m]', fontsize=12)
    plt.ylabel('y [m]', fontsize=12)
    plt.title('Paths – Rectangle Test', fontsize=14, fontweight='bold')
    plt.axis('equal')
    plt.grid(True, linestyle='--', alpha=0.5)

   
    plt.legend(fontsize=10, loc='best', framealpha=0.9)
    plt.tight_layout()
    plt.show()



def main(args=None):
    rclpy.init(args=args)
    node = RectTestNode()
    try:
        rclpy.spin(node)
    finally:
        node.get_logger().info('shutting down, plotting paths...')
        try:
            plot_paths(node)
        except Exception as e:
            node.get_logger().error(f'Error while plotting paths: {e}')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
import tf_transformations


class RobotMarker(Node):
    def __init__(self):
        super().__init__('robot_marker_node')

        self.position_x = 0.0
        self.timer_period = 0.1  # ثانیه

        self.vel_sub = self.create_subscription(Twist, '/cmd_vel_unstamped', self.cmd_callback, 10)
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)

        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.last_velocity = 0.0

        self.get_logger().info("📍 Robot Marker Node started.")

    def cmd_callback(self, msg):
        self.last_velocity = msg.linear.x

    def timer_callback(self):
        self.position_x += self.last_velocity * self.timer_period

        marker = Marker()
        marker.header.frame_id = "base_scan"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "robot"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = self.position_x
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.1
        marker.pose.orientation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
        marker.scale = Vector3(x=0.3, y=0.3, z=0.2)
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime.sec = 0  # دائمی

        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = RobotMarker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


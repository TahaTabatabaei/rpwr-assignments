import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class KnockNode(Node):
    def __init__(self):
        super().__init__('knock_node')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_unstamped', 10) 
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        self.door_open = False
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.get_logger().info("✅ Knock Node with LiDAR started. Waiting for door to open...")

    def lidar_callback(self, msg):
        # Focus on front-facing LiDAR angles (approx. -15° to +15°)
        ranges = msg.ranges
        total = len(ranges)
        front_ranges = ranges[total // 2 - 15: total // 2 + 15]  # 30-degree frontal arc

        # Filter valid distances (exclude inf or 0)
        valid = [r for r in front_ranges if 0.05 < r < msg.range_max]
        if not valid:
            self.door_open = False
            return

        min_range = min(valid)
        if min_range > 0.8:  # No obstacle ahead
            self.door_open = True
            self.get_logger().info(f'🚪 Door OPEN (min range: {min_range:.2f} m)')
        else:
            self.door_open = False
            self.get_logger().info(f'🚧 Door CLOSED (min range: {min_range:.2f} m)')

    def timer_callback(self):
        twist = Twist()
        if self.door_open:
            twist.linear.x = 0.2  # Move forward
            self.get_logger().info("🔼 Moving forward")
        else:
            twist.linear.x = 0.0  # Stay still
            self.get_logger().info("⏹️ Stopped")
        self.cmd_pub.publish(twist)


def main(args=None):

    try:
        rclpy.init(args=args)
        node = KnockNode()

        rclpy.spin(node)


    except (KeyboardInterrupt, Exception) as e:
        node.get_logger().info("Wall follower node stopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        # log_filepath.close()

if __name__ == '__main__':
    main()


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class AdvancedKnockNode(Node):
    def __init__(self):
        super().__init__('advanced_knock_node')

        # بررسی تاپیک‌ها برای شناسایی ربات واقعی
        topic_list = self.get_topic_names_and_types()
        self.robot_real = any('/base/cmd_vel' in topic for topic, _ in topic_list)

        if self.robot_real:
            self.cmd_topic = '/base/cmd_vel'
            self.scan_topic = '/scan_throttle'
            self.get_logger().info("✅ Real robot detected. Using real robot topics.")
        else:
            self.cmd_topic = '/cmd_vel_unstamped'
            self.scan_topic = '/scan'
            self.get_logger().info("⚙️ Simulated/Fake setup. Using default topics.")

        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.lidar_sub = self.create_subscription(LaserScan, self.scan_topic, self.lidar_callback, 10)

        self.door_open = False
        self.open_counter = 0
        self.required_stable_count = 4
        self.timer = self.create_timer(0.5, self.timer_callback)

        self.get_logger().info("🚀 Advanced Knock Node started. Waiting for door to open...")

    def lidar_callback(self, msg):
        ranges = msg.ranges
        total = len(ranges)

        half_window = 20
        center = total // 2
        front_ranges = ranges[center - half_window: center + half_window]

        valid = []
        for r in front_ranges:
            if r < 0.15:
                valid.append(10.0)
            elif r > 5.0:
                valid.append(5.0)
            else:
                valid.append(r)

        if not valid:
            self.open_counter = 0
            self.door_open = False
            self.get_logger().info("❓ No valid LiDAR data in front.")
            return

        free_threshold = 1.0
        required_ratio = 0.7
        min_obstacle_count = 5

        free_count = sum(1 for r in valid if r > free_threshold)
        obstacle_count = sum(1 for r in valid if r < 0.5)
        ratio = free_count / len(valid)

        if obstacle_count < min_obstacle_count and ratio >= required_ratio:
            self.open_counter += 1
            self.get_logger().info(f"✅ DOOR OPEN — {ratio:.0%} free, only {obstacle_count} obstacles")
        elif obstacle_count < min_obstacle_count:
            self.get_logger().info(f"🟡 Small obstacle ignored ({obstacle_count})")
            self.open_counter += 1
        elif ratio >= required_ratio:
            self.open_counter += 1
            self.get_logger().info(f"🚪 Door MAY BE OPEN ({ratio:.0%} free) — counter: {self.open_counter}")
        else:
            self.open_counter = 0
            self.get_logger().info("🚧 Door CLOSED")

        self.door_open = self.open_counter >= self.required_stable_count

    def timer_callback(self):
        twist = Twist()
        if self.door_open:
            twist.linear.x = 0.2
            self.get_logger().info("🔼 Moving forward")
        else:
            twist.linear.x = 0.0
            self.get_logger().info("⏹️ Stopped")
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = AdvancedKnockNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("❌ Node interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


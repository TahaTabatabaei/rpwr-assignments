import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import euler_from_quaternion

import math

class AdvancedKnockNode(Node):
    def __init__(self):
        super().__init__('advanced_knock_node')

    # to handle transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, qos=10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_unstamped', 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        self.door_open = False
        self.open_counter = 0
        self.required_stable_count = 4  
        self.timer = self.create_timer(0.5, self.timer_callback)

        self.get_logger().info("✅ Advanced Knock Node started. Waiting for stable door opening...")

    def lidar_callback(self, msg):
        source_frame = msg.header.frame_id
        target_frame = 'base_link'

        if self.tf_buffer.can_transform(target_frame, source_frame, rclpy.time.Time()):

            t = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )

            roll, pitch, yaw = euler_from_quaternion(
                [t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w]
            )
            angle_between_frames = math.degrees(yaw)

            step = math.degrees(msg.angle_increment)

            # Get the index of the front direction in the scan data
            front_index = int(int((0.0 - msg.angle_min) / msg.angle_increment) - int(angle_between_frames / step))
            # print(f"Front index: {front_index}")

            min_front_index = front_index - 64
            max_front_index = front_index + 64

            front_ranges = msg.ranges[min_front_index:max_front_index]
        #ranges = msg.ranges
        #total = len(ranges)


        #half_window = 20
        #center = total // 2
        #front_ranges = ranges[center - half_window: center + half_window]


        valid = [r for r in front_ranges if 0.15 < r < msg.range_max]
        if not valid:
            self.open_counter = 0
            self.door_open = False
            self.get_logger().info("❓ No valid LiDAR data in front.")
            return

        #cleaned_front = []
        #for r in front_ranges:
            #if r < 0.17:
                #cleaned_front.append(10.0)
            #elif r > 5.0:
                #cleaned_front.append(5.0)   
            #else:
                #cleaned_front.append(r)

        #valid = [r for r in cleaned_front if 0.15 < r < msg.range_max]
       
        free_threshold = 0.8       
        required_ratio = 0.6       
        min_obstacle_count = 5     


        free_count = sum(1 for r in valid if r > free_threshold)
        ratio = free_count / len(valid)

        #obstacle_count = sum(1 for r in valid if r < 0.5)

        #if obstacle_count < min_obstacle_count:
            #self.get_logger().info(f"🟡 Small obstacle ignored ({obstacle_count} points < 0.5m)")
            #self.open_counter += 1
        if ratio >= required_ratio:
            self.open_counter += 1
            self.get_logger().info(f"🚪 Door MAY BE OPEN ({ratio:.0%} free) — counter: {self.open_counter}")
        else:
            self.open_counter = 0
            self.get_logger().info(f"🚧 Door CLOSED ({ratio:.0%} free)")

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


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
import math


class FakeLaserPublisher(Node):
    def __init__(self):
        super().__init__('fake_laser_publisher')
        self.publisher_ = self.create_publisher(LaserScan, '/scan', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.start_time = self.get_clock().now().nanoseconds
        self.get_logger().info("Fake LiDAR Publisher started")

    def timer_callback(self):
        now = self.get_clock().now().nanoseconds
        elapsed_sec = (now - self.start_time) / 1e9

        scan = LaserScan()
        scan.header = Header()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'base_scan'

        scan.angle_min = -1.57
        scan.angle_max = 1.57
        scan.angle_increment = 0.01
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.1
        scan.range_max = 3.0

        num_readings = int((scan.angle_max - scan.angle_min) / scan.angle_increment) + 1
        scan.ranges = [2.5] * num_readings

        desired_angle = 0.0
        index = int((desired_angle - scan.angle_min) / scan.angle_increment)

        if 0 <= index < num_readings:
            if elapsed_sec < 10.0:
            
                for offset in range(-5, 6):
                    i = index + offset
                    if 0 <= i < num_readings:
                        scan.ranges[i] = 0.4
                self.get_logger().info("Door CLOSED (Obstacle at 0.4m)")
            else:
                self.get_logger().info("Door OPEN (No obstacle)")
        else:
            self.get_logger().warn(f"Index {index} out of range for {num_readings} samples!")

        self.publisher_.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = FakeLaserPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


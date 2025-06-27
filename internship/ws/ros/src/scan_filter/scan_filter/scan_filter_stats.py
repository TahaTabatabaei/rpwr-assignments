import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
import numpy as np

class ScanFilterStatsVisualizer(Node):
    def __init__(self):
        super().__init__('scan_filter_stats_visualizer')
        # Parameters for topics and display
        self.declare_parameter('original_topic', '/scan')
        self.declare_parameter('filtered_topic', '/scan_filtered')
        self.declare_parameter('outlier_topic', '/scan_filtered_outliers')
        self.declare_parameter('marker_topic', '/scan_filter_stats_marker')
        self.declare_parameter('marker_frame', 'laser')  # Use 'laser' as default for your setup
        self.declare_parameter('marker_height', 1.0)  # Height at which to display text in RViz

        self.original_topic = self.get_parameter('original_topic').value
        self.filtered_topic = self.get_parameter('filtered_topic').value
        self.outlier_topic = self.get_parameter('outlier_topic').value
        self.marker_topic = self.get_parameter('marker_topic').value
        self.marker_frame = self.get_parameter('marker_frame').value
        self.marker_height = self.get_parameter('marker_height').value

        self.orig_ranges = None
        self.filt_ranges = None
        self.outl_ranges = None

        self.percent_filtered_window = []
        self.window_size = 30

        self.create_subscription(LaserScan, self.original_topic, self.orig_cb, 10)
        self.create_subscription(LaserScan, self.filtered_topic, self.filt_cb, 10)
        self.create_subscription(LaserScan, self.outlier_topic, self.outl_cb, 10)
        self.marker_pub = self.create_publisher(Marker, self.marker_topic, 1)

        self.timer = self.create_timer(0.2, self.publish_marker)  # 5Hz updates

        self.get_logger().info(f"Scan Filter Stats Visualizer running. Using marker frame: {self.marker_frame}")

    def orig_cb(self, msg):
        self.orig_ranges = np.array(msg.ranges)

    def filt_cb(self, msg):
        self.filt_ranges = np.array(msg.ranges)

    def outl_cb(self, msg):
        self.outl_ranges = np.array(msg.ranges)

    def publish_marker(self):
        if self.orig_ranges is None or self.filt_ranges is None:
            return

        orig_finite = self.orig_ranges[np.isfinite(self.orig_ranges)]
        filt_finite = self.filt_ranges[np.isfinite(self.filt_ranges)]

        orig_count = len(orig_finite)
        filt_count = len(filt_finite)
        removed_count = orig_count - filt_count if orig_count >= filt_count else 0
        percent_filtered = 100.0 * removed_count / orig_count if orig_count > 0 else 0.0

        # Running average of percent filtered
        self.percent_filtered_window.append(percent_filtered)
        if len(self.percent_filtered_window) > self.window_size:
            self.percent_filtered_window = self.percent_filtered_window[-self.window_size:]
        running_avg = float(np.mean(self.percent_filtered_window)) if self.percent_filtered_window else 0.0

        # Outlier count (optional)
        outlier_count = 0
        if self.outl_ranges is not None:
            outlier_count = np.sum(np.isfinite(self.outl_ranges))

        # Range stats
        mean_range = np.mean(filt_finite) if filt_count > 0 else 0.0
        std_range = np.std(filt_finite) if filt_count > 0 else 0.0
        min_range = np.min(filt_finite) if filt_count > 0 else 0.0
        max_range = np.max(filt_finite) if filt_count > 0 else 0.0

        marker = Marker()
        marker.header.frame_id = self.marker_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "scan_filter_stats"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.scale.z = 0.25  # Text height
        marker.color.a = 1.0
        # Color: green if <30% filtered, yellow if <60%, red otherwise
        if percent_filtered < 30:
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        elif percent_filtered < 60:
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        else:
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = self.marker_height
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.text = (
            f"Scan Filter Stats\n"
            f"Original: {orig_count}\n"
            f"Filtered: {filt_count}\n"
            f"Removed: {removed_count} ({percent_filtered:.1f}%)\n"
            f"Running Avg: {running_avg:.1f}%\n"
            f"Mean: {mean_range:.2f}m  Std: {std_range:.2f}m\n"
            f"Min: {min_range:.2f}m  Max: {max_range:.2f}m\n"
            + (f"Outliers: {outlier_count}\n" if self.outl_ranges is not None else "")
        )

        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = ScanFilterStatsVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import LaserScan
import numpy as np
import math

class ScanFilterNode(Node):
    def __init__(self):
        super().__init__('scan_filter_node')

        # --- Declare Parameters ---
        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/scan_filtered')
        self.declare_parameter('group_size', 3)
        self.declare_parameter('min_range', 0.15)
        self.declare_parameter('max_range', 3.5)
        self.declare_parameter('filter_mode', 'mean')  # mean, median, min, max
        self.declare_parameter('filter_intensity', False)
        self.declare_parameter('outlier_threshold', 0.3)
        self.declare_parameter('outlier_window_size', 5)
        self.declare_parameter('blind_spot_angles', [-15.0, -5.0, 5.0, 15.0])  # flat list
        self.declare_parameter('output_rate_limit', 0.0)  # Hz, 0.0 means unlimited
        self.declare_parameter('intensity_out_of_range_value', float('nan'))

        # --- Get Parameters ---
        self.group_size = self.get_parameter('group_size').value
        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value
        self.filter_mode = self.get_parameter('filter_mode').value.lower()
        self.filter_intensity = self.get_parameter('filter_intensity').value
        self.outlier_threshold = self.get_parameter('outlier_threshold').value
        self.outlier_window_size = self.get_parameter('outlier_window_size').value
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.output_rate_limit = self.get_parameter('output_rate_limit').value
        self.intensity_out_val = self.get_parameter('intensity_out_of_range_value').value

        # --- Parse blind spot angles ---
        blind_spot_flat = self.get_parameter('blind_spot_angles').value
        self.blind_spot_rad = []
        if len(blind_spot_flat) % 2 != 0:
            self.get_logger().warn("blind_spot_angles should contain pairs. Ignoring last value.")
            blind_spot_flat = blind_spot_flat[:-1]
        for i in range(0, len(blind_spot_flat), 2):
            self.blind_spot_rad.append([
                math.radians(blind_spot_flat[i]),
                math.radians(blind_spot_flat[i+1])
            ])

        # --- Validate Parameters ---
        if self.group_size <= 0:
            self.get_logger().warn("group_size <= 0. Setting to 1.")
            self.group_size = 1
        if self.min_range >= self.max_range:
            self.get_logger().warn("min_range >= max_range. Adjusting.")
            self.max_range = self.min_range + 0.01
        if self.filter_mode not in ['mean', 'median', 'min', 'max']:
            self.get_logger().warn(f"Invalid filter_mode. Using 'mean'.")
            self.filter_mode = 'mean'
        if self.outlier_window_size < 3 or self.outlier_window_size % 2 == 0:
            self.get_logger().warn("outlier_window_size must be odd and >= 3. Setting to 5.")
            self.outlier_window_size = 5

        # --- Subscribers and Publishers ---
        self.subscription = self.create_subscription(
            LaserScan, self.input_topic, self.scan_callback, 10)

        self.publisher = self.create_publisher(LaserScan, self.output_topic, 10)
        self.debug_publisher = self.create_publisher(LaserScan, self.output_topic + '_debug', 10)
        self.diagnostics_publisher = self.create_publisher(LaserScan, self.output_topic + '_info', 10)

        self.min_output_period = 1.0 / max(self.output_rate_limit, 1e-8) if self.output_rate_limit > 0.0 else 0.0
        self.last_output_time = self.get_clock().now()
        self.last_msg_time = self.get_clock().now()
        self.watchdog_timer = self.create_timer(2.0, self.input_topic_check)

        self.get_logger().info("Scan Filter Node started.")
        self.log_all_parameters()
        if self.blind_spot_rad:
            self.get_logger().info(f"Blind spot filtering enabled: {blind_spot_flat}")

    def log_all_parameters(self):
        param_names = [
            'input_topic', 'output_topic', 'group_size', 'min_range', 'max_range',
            'filter_mode', 'filter_intensity', 'outlier_threshold', 'outlier_window_size',
            'blind_spot_angles', 'output_rate_limit', 'intensity_out_of_range_value']
        self.get_logger().info("==== Filter Node Parameters ====")
        for pn in param_names:
            self.get_logger().info(f"{pn}: {self.get_parameter(pn).value}")
        self.get_logger().info("==============================")

    def input_topic_check(self):
        time_since_msg = (self.get_clock().now() - self.last_msg_time).nanoseconds / 1e9
        if time_since_msg > 2.0:
            self.get_logger().warn(f"No LaserScan received on {self.input_topic} in {time_since_msg:.1f}s")

    def apply_filter(self, group):
        finite = group[np.isfinite(group)]
        if finite.size == 0:
            return float('inf')
        if self.filter_mode == 'mean':
            return float(np.mean(finite))
        elif self.filter_mode == 'median':
            return float(np.median(finite))
        elif self.filter_mode == 'min':
            return float(np.min(finite))
        elif self.filter_mode == 'max':
            return float(np.max(finite))
        return float('inf')

    def remove_spatial_outliers(self, ranges):
        filtered = ranges.copy()
        half_window = self.outlier_window_size // 2
        for i in range(len(ranges)):
            if not np.isfinite(ranges[i]):
                continue
            start = max(0, i - half_window)
            end = min(len(ranges), i + half_window + 1)
            window = ranges[start:end]
            finite_window = window[np.isfinite(window)]
            if len(finite_window) < 3:
                continue
            median = np.median(finite_window)
            if abs(ranges[i] - median) > self.outlier_threshold:
                filtered[i] = float('inf')
        return filtered

    def scan_callback(self, msg):
        now = self.get_clock().now()
        self.last_msg_time = now

        if self.min_output_period > 0.0:
            time_since_last = (now - self.last_output_time).nanoseconds / 1e9
            if time_since_last < self.min_output_period:
                return
        self.last_output_time = now

        ranges = np.array(msg.ranges, dtype=np.float32)
        intensities = np.array(msg.intensities, dtype=np.float32) if self.filter_intensity and len(msg.intensities) == len(ranges) else None

        if self.blind_spot_rad:
            angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
            for r_rad in self.blind_spot_rad:
                in_spot = (angles >= r_rad[0]) & (angles <= r_rad[1]) if r_rad[0] <= r_rad[1] else (angles >= r_rad[0]) | (angles <= r_rad[1])
                ranges[in_spot] = float('inf')

        out_of_range = (ranges < self.min_range) | (ranges > self.max_range)
        ranges[out_of_range] = float('inf')
        if intensities is not None:
            intensities[out_of_range] = float(self.intensity_out_val) if not math.isnan(self.intensity_out_val) else float('nan')

        ranges = self.remove_spatial_outliers(ranges)

        group_size = self.group_size
        num_points = len(ranges)
        num_groups = num_points // group_size

        filtered_ranges = []
        filtered_intensities = []

        for i in range(num_groups):
            r_group = ranges[i * group_size:(i + 1) * group_size]
            filtered_ranges.append(self.apply_filter(r_group))
            if intensities is not None:
                i_group = intensities[i * group_size:(i + 1) * group_size]
                filtered_intensities.append(np.nanmean(i_group) if not np.all(np.isnan(i_group)) else float('nan'))

        if num_points % group_size != 0:
            r_group = ranges[num_groups * group_size:]
            filtered_ranges.append(self.apply_filter(r_group))
            if intensities is not None:
                i_group = intensities[num_groups * group_size:]
                filtered_intensities.append(np.nanmean(i_group) if not np.all(np.isnan(i_group)) else float('nan'))

        new_scan = LaserScan()
        new_scan.header = msg.header
        new_scan.angle_min = msg.angle_min
        new_scan.angle_increment = msg.angle_increment * group_size
        new_scan.angle_max = new_scan.angle_min + (len(filtered_ranges) - 1) * new_scan.angle_increment
        new_scan.time_increment = msg.time_increment * group_size
        new_scan.scan_time = msg.scan_time
        new_scan.range_min = self.min_range
        new_scan.range_max = self.max_range
        new_scan.ranges = filtered_ranges
        new_scan.intensities = filtered_intensities if intensities is not None else []

        self.publisher.publish(new_scan)

        # --- Debug scan: removed points ---
        debug_scan = LaserScan()
        debug_scan.header = msg.header
        debug_scan.angle_min = msg.angle_min
        debug_scan.angle_max = msg.angle_max
        debug_scan.angle_increment = msg.angle_increment
        debug_scan.time_increment = msg.time_increment
        debug_scan.scan_time = msg.scan_time
        debug_scan.range_min = msg.range_min
        debug_scan.range_max = msg.range_max
        debug_scan.ranges = [msg.range_max if not np.isfinite(r) else 0.0 for r in ranges]
        debug_scan.intensities = []
        self.debug_publisher.publish(debug_scan)

        # --- Diagnostics ---
        finite_ranges = np.array(filtered_ranges)[np.isfinite(filtered_ranges)]
        if finite_ranges.size > 0:
            diag = LaserScan()
            diag.header = msg.header
            diag.angle_min = 0.0
            diag.angle_max = 0.0
            diag.angle_increment = 0.0
            diag.time_increment = 0.0
            diag.scan_time = 0.0
            diag.range_min = np.min(finite_ranges)
            diag.range_max = np.max(finite_ranges)
            diag.ranges = [float(np.mean(finite_ranges))]
            diag.intensities = [float(len(finite_ranges))]
            self.diagnostics_publisher.publish(diag)


def main(args=None):
    rclpy.init(args=args)
    node = ScanFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


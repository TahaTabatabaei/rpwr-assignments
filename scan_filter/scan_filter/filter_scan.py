#!/usr/bin/env python

import rclpy
from rclpy.node import Node
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
        # NEW: Blind spot parameter
        # Expects a list of lists, e.g., [[-15.0, -5.0], [5.0, 15.0]] in degrees
        self.declare_parameter('blind_spot_angles', [])#-26 26

        # --- Get Parameters ---
        self.group_size = self.get_parameter('group_size').value
        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value
        self.filter_mode = self.get_parameter('filter_mode').value.lower()
        self.filter_intensity = self.get_parameter('filter_intensity').value
        self.outlier_threshold = self.get_parameter('outlier_threshold').value
        self.outlier_window_size = self.get_parameter('outlier_window_size').value
        
        # NEW: Get and convert blind spot angles to radians
        blind_spot_deg = self.get_parameter('blind_spot_angles').value
        self.blind_spot_rad = []
        for r in blind_spot_deg:
            if isinstance(r, list) and len(r) == 2:
                self.blind_spot_rad.append([math.radians(r[0]), math.radians(r[1])])

        # --- Validate Parameters ---
        if self.group_size <= 0:
            self.get_logger().warn("Invalid group_size. Must be > 0. Setting to 1.")
            self.group_size = 1
        if self.min_range >= self.max_range:
            self.get_logger().warn("min_range >= max_range. Adjusting max_range.")
            self.max_range = self.min_range + 0.01
        if self.filter_mode not in ['mean', 'median', 'min', 'max']:
            self.get_logger().warn(f"Invalid filter_mode '{self.filter_mode}'. Using 'mean'.")
            self.filter_mode = 'mean'
        if self.outlier_window_size < 3 or self.outlier_window_size % 2 == 0:
            self.get_logger().warn("outlier_window_size must be odd and >= 3. Setting to 5.")
            self.outlier_window_size = 5

        # --- Subscribers and Publishers ---
        self.subscription = self.create_subscription(
            LaserScan,
            self.get_parameter('input_topic').value,
            self.scan_callback,
            10)

        self.publisher = self.create_publisher(
            LaserScan,
            self.get_parameter('output_topic').value,
            10)

        self.get_logger().info("Scan Filter Node started.")
        if self.blind_spot_rad:
            self.get_logger().info(f"Applying blind spot filters (degrees): {blind_spot_deg}")

    def apply_filter(self, group: np.ndarray):
        """Applies the selected filter (mean, median, etc.) to a group of points."""
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
        return float('inf') # Fallback

    def remove_spatial_outliers(self, ranges: np.ndarray) -> np.ndarray:
        """Removes points that are spatial outliers compared to their neighbors."""
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

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=np.float32)
        intensities = np.array(msg.intensities, dtype=np.float32) if self.filter_intensity and len(msg.intensities) == len(ranges) else None

        # --- NEW: Blind Spot Filtering ---
        if self.blind_spot_rad:
            # Create an array of angles corresponding to each range measurement
            angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
            for r_rad in self.blind_spot_rad:
                # Create a boolean mask for points within the current blind spot
                in_blind_spot = (angles >= r_rad[0]) & (angles <= r_rad[1])
                # Set ranges in the blind spot to infinity
                ranges[in_blind_spot] = float('inf')

        # --- Range Filtering ---
        # Remove out-of-range values by setting them to infinity
        out_of_range = (ranges < self.min_range) | (ranges > self.max_range)
        ranges[out_of_range] = float('inf')
        if intensities is not None:
            # Use nan for intensities so they don't affect mean calculations
            intensities[out_of_range] = float('nan')

        # --- Spatial Outlier Removal ---
        ranges = self.remove_spatial_outliers(ranges)

        # --- Grouping for Resolution Reduction ---
        group_size = self.group_size
        num_points = len(ranges)
        num_groups = num_points // group_size

        filtered_ranges = []
        filtered_intensities = []

        for i in range(num_groups):
            start_idx = i * group_size
            end_idx = (i + 1) * group_size
            
            r_group = ranges[start_idx:end_idx]
            filtered_ranges.append(self.apply_filter(r_group))

            if intensities is not None:
                i_group = intensities[start_idx:end_idx]
                # nanmean safely computes the mean ignoring nans
                filtered_intensities.append(
                    float(np.nanmean(i_group)) if not np.all(np.isnan(i_group)) else float('nan')
                )

        # Handle leftover points if the total number isn't divisible by group_size
        if num_points % group_size != 0:
            start_idx = num_groups * group_size
            r_group = ranges[start_idx:]
            filtered_ranges.append(self.apply_filter(r_group))
            
            if intensities is not None:
                i_group = intensities[start_idx:]
                filtered_intensities.append(
                    float(np.nanmean(i_group)) if not np.all(np.isnan(i_group)) else float('nan')
                )

        # --- Create New Filtered Scan Message ---
        new_scan = LaserScan()
        new_scan.header = msg.header
        new_scan.angle_min = msg.angle_min
        new_scan.angle_increment = msg.angle_increment * group_size
        # Correctly calculate the angle of the last measurement in the new scan
        new_scan.angle_max = new_scan.angle_min + (len(filtered_ranges) - 1) * new_scan.angle_increment
        new_scan.time_increment = msg.time_increment * group_size
        new_scan.scan_time = msg.scan_time
        new_scan.range_min = self.min_range
        new_scan.range_max = self.max_range
        new_scan.ranges = filtered_ranges
        new_scan.intensities = filtered_intensities if intensities is not None else []

        self.publisher.publish(new_scan)
        self.get_logger().debug(f"Published filtered scan with {len(filtered_ranges)} points.")

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

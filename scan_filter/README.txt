1-build:
colcon build --packages-select scan_filter

2-source you terminal

3-run rviz:
rviz2

4-In one terminal, play your rosbag:
ros2 bag play rosbag2_2025_06_04-13_12_30_0.mcap

5-In another terminal, run your scan filter node:
ros2 run scan_filter filter_scan

6-if you want to see a visualized filtering:
	- install matplotlib
	-then run this node: ros2 run scan_filter filter_plot


# Example ROS 2 Command:

ros2 run scan_filter filter_scan \
  --ros-args \
  -p input_topic:=/scan \
  -p output_topic:=/scan_filtered \
  -p group_size:=3 \
  -p min_range:=0.15 \
  -p max_range:=3.5 \
  -p filter_mode:=mean \
  -p filter_intensity:=False \
  -p outlier_threshold:=0.3 \
  -p outlier_window_size:=5 \
  -p blind_spot_angles:="[88.0,92.0,268.0,272.0]" \
  -p output_rate_limit:=10.0




Parameter		Example Value		Description / How to Use
input_topic		/scan			The topic your LiDAR publishes raw data to. Usually /scan for Hokuyo and similar sensors.
output_topic		/scan_filtered		The topic for the filtered output. Use this in your navigation stack.
group_size		3			Downsamples by grouping 3 beams together (balances detail and noise reduction).
min_range		0.15			Ignore anything closer than 15cm (below reliable measurement range for most LiDARs).
max_range		3.5			Ignore anything farther than 3.5m (indoors, reliable readings and less mapping noise).
filter_mode		mean			Use the mean for each group. For more robustness to outliers, you can try median.
filter_intensity	False			Unless you specifically use intensity data, keep this False.
outlier_threshold	0.3			Remove points that differ by more than 0.3m from local median (removes spikes/reflections).
outlier_window_size	5			Use 5-neighbor window for outlier detection (must be odd, 5–11 typical).
blind_spot_angles    [88.0,92.0,268.0,272.0]	Blocks out the two support poles (assuming they are at ±90° from LiDAR forward).
output_rate_limit	10.0			Limits output to 10 scans/sec (Hz). Set to 0.0 for no limit; match your robot's navigation rate.


## Quick Explanations for Each Value
	group_size:=3: Smooths data and reduces processing for navigation.
	min_range:=0.15: Discards unreliable close points (robot body/ground).
	max_range:=3.5: Most indoor environments don't need data beyond this; reduces false positives from distant glass/objects.
	filter_mode:=mean: Good balance for most environments; median if you experience spikes.
	outlier_threshold:=0.3 and outlier_window_size:=5: Removes sudden spikes in data.
	blind_spot_angles:="[88.0,92.0,268.0,272.0]": Masks out your support poles.
	output_rate_limit:=10.0: Most navigation stacks (like nav2) run at ≤10Hz, so this is efficient.

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


-(Optional) Tune Parameters
If you want to adjust the filter:

ros2 run scan_filter filter_scan   --ros-args   -p input_topic:=/scan   -p output_topic:=/scan_filtered   -p group_size:=1   -p min_range:=0.35  -p max_range:=7.0   -p filter_mode:=median   -p filter_intensity:=False   -p outlier_threshold:=0.6   -p outlier_window_size:=5   -p blind_spot_angles:="[-135.0, -125.0,-80.0, -70.0,55.0, 80.0,125.0, 135.0"]   -p output_rate_limit:=0.0


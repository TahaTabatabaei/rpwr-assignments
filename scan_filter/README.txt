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
ros2 run scan_filter filter_scan --ros-args -p group_size:=4 -p min_range:=0.2 -p max_range:=3.0

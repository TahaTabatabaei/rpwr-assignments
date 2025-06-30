- The script `keep_lane_with_teleop.py` works fine with the simulation environment from the 4th assignment.

- `keep_lane_real_robot.py` is for testing on the real robot.

- `keep_lane_with_teleop_filteredScan.py` listen to topic `/scan_filtered` published by the node from the scan_filter package.

You can try each of them with a command like this:

`ros2 run group6_package keep_lane_with_teleop` 

and just replace the name of the python executive for different scripts.

example command:
```
ros2 run group6_package keep_lane_real_robot \
  --ros-args \
  -p publish_topic:=/base/cmd_vel \
  -p scan_topic:=/scan_throttle \
  -p cmd_topic:=/ps3/cmd_vel_throttled \
  -p remap_topic:=/ps3/cmd_vel \
  -p turn_speed:=0.2 \
  -p min_distance_front:=0.8 \
  -p min_distance_front_side:=0.9 \
  -p min_distance_side:=1.0 \
  -p field_of_view:=64 \
  -p min_valid:=0.15 \
  -p max_valid:=5.0 \
  -p checkValid:=true
```
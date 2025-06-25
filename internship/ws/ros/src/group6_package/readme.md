- The script `keep_lane_with_teleop.py` works fine with the simulation environment from the 4th assignment.

- `keep_lane_real_robot.py` is for testing on the real robot.

- `keep_lane_with_teleop_filteredScan.py` listen to topic `/scan_filtered` published by the node from the scan_filter package.

You can try each of them with a command like this:

`ros2 run group6_package keep_lane_with_teleop` 

and just replace the name of the python executive for different scripts.
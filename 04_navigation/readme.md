**Group number 6**\
Seyed mohammad taha Tabatabaei - @TahaTabatabaei \
Zakaria Ouaddi - @zakaria-ouaddi \
Helia Moradi - @hmoradi79 \
Mohamed amine Kina - @Amineki6 

## notes for tutor

- To check the solutiones for task 1 & 2, after running the simulation command `ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py world:=office_area`,
in a new terminal, run each of them like this:
    ```
    python task1.py
    ```
    and 
    ```
    python task2.py
    ```

- Files `task2_bang.py` and `task2_pid.py` are related to two different algorithms. I tried to solve task 2 with those algorithms as well, but results are not good. You may ignore them.

- There are 2 demo videos. You may watch them to see our results.


## notes for teammates
- To install all the dependencies, it is advised to run this command in your workspace root:
    ```
    rosdep update --rosdistro=$ROS_DISTRO

    sudo apt-get update

    rosdep install --from-paths ./ -i -y --rosdistro ${ROS_DISTRO}
    ```
    Then it is ok to build the workspace.

- Steps to run task 3:

    basic gz simulation:

    ```
    ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py world:=office_area
    ```

    testing the laser scan topic
    in a new terminal:

    ```
    ros2 topic echo /scan --once
    ```

    mapping
    in a new terminal:

    ```
    ros2 launch turtlebot4_navigation slam.launch.py use_sim_time:=true
    ```
    if you are trying to do the localization, run this command instead (take care of the map's path based on your environment directories):
    ```
    ros2 launch turtlebot4_navigation localization.launch.py map:=src/turtlebot4_simulator/turtlebot4_gz_bringup/maps/map_num1.yaml 
    ```

    visualization in rviz
    in a new terminal:

    ```
    ros2 launch turtlebot4_viz view_navigation.launch.py use_sim_time:=true
    ```

    to control the robot
    in a new terminal:

    ```
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
    ```

    Once the mapping is completed, from the path `ws/ros/turtlebot4_simulator/turtlebot4_gz_bringup/maps` execute the following command to save the map. Change <map_name> to desired name of the map.:

    ```
    ros2 run nav2_map_server map_saver_cli -f <map_name> --occ 0.65 --free 0.15 --ros-args -p save_map_timeout:=20.0
    ```

    if maps successfuly saved, you should be able to see them inside the `/masps` folder with the name you specified.

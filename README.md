# rpwr-assignments

This repository contains my work for the Robot Programming with ROS course during the Summer semester 2025. It includes four structured assignments and a final project focused on robot control and navigation using ROS 2 and Gazebo.

## Assignments Overview

1. Git, Linux & Python Basics (01_git-linux-python)
    Introduces fundamental tools for robotics development:

    - Basic Git usage and version control workflows
    - Linux shell scripting and package management

2. Coordinate Frames & TF (02_coordinates-tf)
Covers:

    - Homogeneous transforms and coordinate math

    - Introduction to the ROS tf2 library

    - Static and dynamic transforms for simulated robots

3. ROS Communication (03_ros)
Focuses on:

    - Creating ROS 2 nodes in Python

    - Publishing/subscribing to topics

    - Services, parameters, and launch files

4. Navigation & SLAM (04_navigation)
Involves:

    - Running SLAM and path planning on Turtlebot

    - Obstacle detection and local/global planning

    - Testing in simulation using nav2 stack

    - Sloved A wall Follower chalenge in the simulation

## Final Project — Autonomous Turtlebot Behaviors (internship/)

For the final project, our group implemented three distinct behaviors in ROS 2 that combine perception, control, and real-time decision making in a simulated Turtlebot3:

### Lane Keeping Assist

**Goal:** Allow user control via PS3 controller, while enabling the robot to autonomously avoid collisions with hallway walls or obstacles.

**Approach:** 

*LIDAR-based region mapping:* The robot divides its surroundings into 5 angular zones: left, fleft, front, fright, and right. It computes the mean distance within each zone to assess proximity to walls or obstacles.

*Reactive FSM (Finite State Machine):* Based on proximity analysis, the robot switches between:

- Find the wall

- Turn left

- Turn right

- Follow the wall

*Smooth navigation:* It avoids abrupt stops or oscillations by applying state-driven angular corrections and reducing linear speed only when necessary.

*Throttle PS3 commands:* Velocity commands from the PS3 controller are throttled and blended with the robot’s autonomous corrections to allow safe control.


### Knock-Knock (Door Detection & Response)
    
**Goal:** Allow the robot to detect whether a door is open or closed, and move forward only when the path is clear.

**Approach:**

- *Robust door detection* using LIDAR scan analysis across a window of ~40 degrees in front of the robot.

- *Noise handling and normalization:*

    - Invalid points (e.g. too close or too far) are capped or discarded.

    - Applied safety rules: ignore spurious small obstacles, smooth decision over time.

- *Stable open-check mechanism:* The robot only moves forward after the opening is consistently clear for several consecutive scans (required_stable_count = 4).

- *Automatic environment detection:* Supports both simulated and real robot setups by detecting topic names and switching accordingly.

### Laser Scan Filtering

**Goal:** Filter noisy or irrelevant points from the /scan topic in real time to produce cleaner, more consistent LIDAR data for use by downstream behaviors such as navigation, obstacle detection, and wall following.

**Approach:** 

*Grouped filtering:*
LIDAR ranges are grouped in sliding windows (group_size = 3), and each group is aggregated using a configurable statistic: mean, median, min, or max. This reduces angular resolution and smooths scan data.

*Blind spot masking:*
Angular sectors known to be unreliable (e.g., −15° to −5°, +5° to +15°) are force-cleared by setting their ranges to infinity (inf), effectively ignoring them during processing.

*Outlier removal:*
Local spatial outliers are filtered using a sliding median window. Any point that differs from the local median by more than outlier_threshold = 0.3 is discarded as noise.

*Range capping:*
Points below min_range = 0.15m or above max_range = 3.5m are treated as invalid and ignored, helping suppress reflective artifacts and distant clutter.


## Demo & Presentation

📄 [Final Report (PDF)](https://github.com/TahaTabatabaei/rpwr-assignments/blob/main/images/Robot%20Programming%20with%20ROS.pdf)  
▶️ [![Watch the demo](https://github.com/TahaTabatabaei/rpwr-assignments/blob/main/images/wallp.png)](https://drive.google.com/file/d/1_moAkDP2B7ynNaq6fkaidSIL31K33pZ3/view?usp=sharing)

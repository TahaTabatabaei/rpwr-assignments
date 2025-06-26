# solution from this link https://www.theconstruct.ai/wall-follower-algorithm/

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, TwistStamped

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import euler_from_quaternion

import math
import logging

import os
from datetime import datetime

# Create logs directory if it doesn't exist
log_dir = "internship/scripts/logs_group6"
os.makedirs(log_dir, exist_ok=True)

# Generate filename with current date and time
now = datetime.now()
timestamp = now.strftime("%Y-%m-%d_%H-%M-%S")
log_filename = f"keep_lane_{timestamp}.txt  "
log_filepath = os.path.join(log_dir, log_filename)
log_file = open(log_filepath, 'w')

# Custom print function that duplicates to log file
def print_and_log(*args, **kwargs):
    print(*args, **kwargs)
    print(*args, **kwargs, file=log_file)
    log_file.flush()

class WallFollower(Node):

    def __init__(self):
        super().__init__('wall_follower')

        # --- Declare Parameters ---
        self.declare_parameter('publish_topic', '/base/cmd_vel')
        self.declare_parameter('scan_topic', '/scan_throttle')
        self.declare_parameter('cmd_topic', '/ps3/cmd_vel_throttled')
        self.declare_parameter('remap_topic', '/ps3/cmd_vel')
        self.declare_parameter('turn_speed', 0.2)
        self.declare_parameter('min_distance_front', 0.8)
        self.declare_parameter('min_distance_front_side', 0.9)
        self.declare_parameter('min_distance_side', 1.0)
        self.declare_parameter('field_of_view', 64)
        self.declare_parameter('min_valid', 0.15)
        self.declare_parameter('max_valid', 5.0)
        self.declare_parameter('checkValid', True)

        # --- Get Parameters ---
        self.publish_topic = self.get_parameter('publish_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.cmd_topic = self.get_parameter('cmd_topic').value
        self.remap_topic = self.get_parameter('remap_topic').value

        self.turn_speed = self.get_parameter('turn_speed').value
        self.min_distance_front = self.get_parameter('min_distance_front').value
        self.min_distance_front_side = self.get_parameter('min_distance_front_side').value
        self.min_distance_side = self.get_parameter('min_distance_side').value

        self.field_of_view = self.get_parameter('field_of_view').value
        self.min_valid = self.get_parameter('min_valid').value
        self.max_valid = self.get_parameter('max_valid').value
        self.checkValid = self.get_parameter('checkValid').value

        # --- Print Parameters ---
        print_and_log(20*"*")
        print_and_log(f'publish_topic = {self.publish_topic}')
        print_and_log(f'scan_topic = {self.scan_topic}')
        print_and_log(f'cmd_topic = {self.cmd_topic}')
        print_and_log(f'remap_topic = {self.remap_topic}')
        print_and_log(f'turn_speed = {self.turn_speed}')
        print_and_log(f'min_distance_front = {self.min_distance_front}')
        print_and_log(f'min_distance_front_side = {self.min_distance_front_side}')
        print_and_log(f'min_distance_side = {self.min_distance_side}')
        print_and_log(f'field_of_view = {self.field_of_view}')
        print_and_log(f'min_valid = {self.min_valid}')
        print_and_log(f'max_valid = {self.max_valid}')
        print_and_log(f'checkValid = {self.checkValid}')
        print_and_log(20*"*")
        
        topic_list = self.get_topic_names_and_types()
        self.robot_is_ok = False

        for i in range(len(topic_list)):

            if  self.remap_topic in topic_list[i]: # publish to /base/cmd_vel for real robot and subscribe to throttled topics
                # Publisher to /base/cmd_vel
                self.cmd_pub = self.create_publisher(Twist, self.publish_topic, 10)

                # Subscriber to scan_throttle data
                self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)

                # Subscriber to throttled controller commands
                self.ps3_sub = self.create_subscription(Twist, self.cmd_topic, self.ps3_callback, qos_profile=10)
                
                print_and_log("Found real robot...")
                self.robot_is_ok = True
                break
        
        self.last_msg = Twist()

        
        regions_ = {
            'right': 0,
            'fright': 0,
            'front': 0,
            'fleft': 0,
            'left': 0,
        }
        self.state_ = 0
        self.state_dict_ = {
            0: 'find the wall',
            1: 'turn left',
            2: 'follow the wall',
            3: 'turn right',
        }
        
        print_and_log("Wall follower node started")

    def take_action(self):
        global regions_
        regions = regions_

        msg = Twist()
        linear_x = 0
        angular_z = 0

        global state_description

        d_f = self.min_distance_front
        d = self.min_distance_front_side
        d_side = self.min_distance_side

        if regions['right'] < d_side and regions['front'] > d and regions['fright'] < d:
            # Wall on the right side and in front, but nothing on the left side, so forllow the wall on the right side
            state_description = 'case 0 - right and fright'
            self.change_state(2)
        elif regions['left'] < d_side and regions['front'] > d and regions['fleft'] < d:
            # Wall on the left side and in front, but nothing on the right side, so follow the wall on the left side
            state_description = 'case 1 - left and fleft'
            self.change_state(2)
        elif regions['right'] < d_side and regions['front'] > d and regions['fright'] > d:
            # Wall on the right side, but nothing on the front and fright, so turn right to find the wall again
            state_description = 'case 2 - right'
            # self.change_state(3)
            self.change_state(2)
        elif regions['left'] < d_side and regions['front'] > d and regions['fleft'] > d:
            # Wall on the left side, but nothing on the front and fleft, so turn left to find the wall again
            state_description = 'case 3 - left'
            # self.change_state(1)
            self.change_state(2)
        elif regions['front'] > d and regions['fleft'] > d and regions['fright'] > d and regions['right'] > d_side and regions['left'] > d_side:
            # No wall in front, left, right, fleft, fright, so nothing to do. just go forward 
            state_description = 'case 4 - nothing'
            self.change_state(0)
        elif regions['front'] < d_f and regions['fleft'] > d and regions['fright'] > d:
            # Wall in front, but nothing on the fleft and fright, so it means there is a wall directly in front of the robot, so turn left to find the wall
            state_description = 'case 5 - front'
            self.change_state(1)
        elif regions['front'] > d and regions['fleft'] > d and regions['fright'] < d:
            # Wall in fright, but nothing on the front and fleft, so it means there is a wall directly in fright of the robot, so turn right to find the wall
            state_description = 'case 6 - fright'
            self.change_state(2)
        elif regions['front'] > d and regions['fleft'] < d and regions['fright'] > d:
            # Wall in fleft, but nothing on the front and fright, so it means there is a wall directly in fleft of the robot, so turn left to find the wall
            state_description = 'case 7 - fleft'
            self.change_state(0)
        elif regions['front'] < d and regions['fleft'] > d and regions['fright'] < d:
            # Wall in front and fright, but nothing on the fleft, it means there is a wall directly in front and fright of the robot, turn left to avoid hitting the wall
            state_description = 'case 8 - front and fright'
            self.change_state(1)
        elif regions['front'] < d and regions['fleft'] < d and regions['fright'] > d:
            # Wall in front and fleft, but nothing on the fright, it means there is a wall directly in front and fleft of the robot, turn right to avoid hitting the wall
            state_description = 'case 9 - front and fleft'
            self.change_state(3)
        elif regions['front'] < d_f and regions['fleft'] < d and regions['fright'] < d: 
            # locked in a corner. turn left by default to find a wall
            state_description = 'case 10 - front and fleft and fright'
            self.change_state(1)
        elif regions['front'] > d_f and regions['fleft'] < d and regions['fright'] < d:
            # between two objects or wall, but front is open (e.g. corridor)
            state_description = 'case 11 - fright and fleft - front open'
            self.change_state(2)
        else:
            state_description = 'unknown case'
            print_and_log('Unknown case, no action taken')

        return

    def change_state(self, state):
        # self.get_logger().info(f"{state_description}")
        print_and_log(f"{state_description}")
        if state is not self.state_:
            print_and_log('Wall follower - [%s] - %s' % (state, self.state_dict_[state]))
            print_and_log(f"front:{regions_['front']}, fright:{regions_['fright']}, right:{regions_['right']}, fleft:{regions_['fleft']}, left:{regions_['left']}")
            print_and_log(30* '@')

            self.state_ = state

        return

    def find_wall(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        return msg

    def turn_left(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = self.turn_speed
        return msg

    def follow_the_wall(self):
        global regions_

        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        return msg

    def turn_right(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = -self.turn_speed
        return msg

    def scan_callback(self, msg):
        valid_ranges = []
        if self.checkValid:
            for r in msg.ranges:
                if (r < self.min_valid):
                    valid_ranges.append(self.max_valid+10.0)
                elif (r > self.max_valid):
                    valid_ranges.append(self.max_valid)
                else:
                    valid_ranges.append(r)
        else:
            valid_ranges = msg.ranges
        # print(len(msg.ranges))


        step = math.degrees(msg.angle_increment)
        # print(f'degree step = {step}')
        stepx = (1/step)
        print(f'stepx = {stepx}')
        # Get the index of the front direction in the scan data

        front_index = int((0.0 - msg.angle_min) / msg.angle_increment)
        print(f"Front index: {front_index}")

        # print(f'len ranges = {len(msg.ranges)}')

        min_front_index = front_index - int((self.field_of_view/2)*stepx)
        # print(min_front_index)
        max_front_index = front_index + int((self.field_of_view/2)*stepx)
        # print(max_front_index)

        front_ranges = valid_ranges[min_front_index:max_front_index]

        right_front_index = front_index - int(self.field_of_view*stepx)
        # print(right_front_index)
        min_right_front_index = right_front_index - int((self.field_of_view/2)*stepx)
        # print(min_right_front_index)
        max_right_front_index = right_front_index + int((self.field_of_view/2)*stepx)
        # print(max_right_front_index)

        f_right_ranges = valid_ranges[min_right_front_index:max_right_front_index]


        max_right_index = min_right_front_index-1
        # print(max_right_index)
        min_right_index = 0
        # print(min_right_index)

        right_ranges = valid_ranges[min_right_index:max_right_index]


        left_front_index = front_index + int(self.field_of_view*stepx)
        # print(left_front_index)
        min_left_front_index = left_front_index - int((self.field_of_view/2)*stepx)
        # print(min_left_front_index)
        max_left_front_index = left_front_index + int((self.field_of_view/2)*stepx)
        # print(max_left_front_index)

        left_front_ranges = valid_ranges[min_left_front_index:max_left_front_index]


        min_left_index = max_left_front_index+1
        # print(min_left_index)
        max_left_index = len(valid_ranges)-1
        # print(max_left_index)

        left_ranges = valid_ranges[min_left_index:max_left_index]
        
        # print(f"fright: {right_front_index}, front: {front_index}, fleft: {left_front_index}")

        global regions_

        regions_ = {
        'right':  min(right_ranges),
        'fright': min(f_right_ranges),
        'front':  min(front_ranges),
        'fleft':  min(left_front_ranges),
        'left':   min(left_ranges),
        }


        self.take_action()

        twist = Twist()


        if self.state_ == 0:
            return
            twist = self.find_wall()
        elif self.state_ == 1:
            twist = self.turn_left()
        elif self.state_ == 2:
            return
            twist = self.follow_the_wall()  
        elif self.state_ == 3:
            twist = self.turn_right()
        else:
            print_and_log(f"Unknown state: {self.state_}. No action taken.")

        self.cmd_pub.publish(twist)

    def ps3_callback(self, msg: TwistStamped):
        self.last_msg = msg
        print_and_log(f"Received PS3 command: linear.x={self.last_msg.twist.linear.x}, angular.z={self.last_msg.twist.angular.z}")

        msg = Twist()
        msg.linear.x = self.last_msg.twist.linear.x
        msg.angular.z = self.last_msg.twist.angular.z
        
        self.cmd_pub.publish(msg)



def main():

    try:
        rclpy.init()
        node = WallFollower()

        if node.robot_is_ok:
            logger = logging.getLogger("rclpy")
            handler = logging.StreamHandler(log_filepath)
            logger.addHandler(handler)

            rclpy.spin(node)
        else:
            print_and_log("Could not find the real robot...")


    except (KeyboardInterrupt, Exception) as e:
        node.get_logger().info("Wall follower node stopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        # log_filepath.close()

if __name__ == '__main__':
    main()
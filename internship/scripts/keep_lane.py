# solution from this link https://www.theconstruct.ai/wall-follower-algorithm/

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import euler_from_quaternion

import math
import time

class WallFollower(Node):

    def __init__(self):
        super().__init__('wall_follower')

        # to handle transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, qos=10)

        # Publisher to cmd_vel_unstamped
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_unstamped', 10)

        # Subscriber to scan data
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Movement parameters
        # self.forward_speed = 0.1
        # self.turn_speed = 0.4
        self.min_distance_front = 0.45  # meters
        self.min_distance_front_side = 0.5  # meters
        self.min_distance_side = 0.6  # meters

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

        self.get_logger().info("Wall follower node started")

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
            self.change_state(3)
        elif regions['left'] < d_side and regions['front'] > d and regions['fleft'] > d:
            # Wall on the left side, but nothing on the front and fleft, so turn left to find the wall again
            state_description = 'case 3 - left'
            self.change_state(1)
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
            self.get_logger().info("unkown case")

        return

    def change_state(self, state):
        
        self.get_logger().info(f"{state_description}")
        if state is not self.state_:
            print('Wall follower - [%s] - %s' % (state, self.state_dict_[state]))
            self.get_logger().info(f"front:{regions_['front']}, fright:{regions_['fright']}, right:{regions_['right']}, fleft:{regions_['fleft']}, left:{regions_['left']}")
            print(30* '@')


            self.state_ = state
        
        return
    
    def find_wall(self):
        msg = Twist()
        msg.linear.x = 0.10
        msg.angular.z = 0.0
        return msg

    def turn_left(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.2
        return msg

    def follow_the_wall(self):
        global regions_
        
        msg = Twist()
        msg.linear.x = 0.2
        msg.angular.z = 0.0
        return msg
    
    def turn_right(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = -0.2
        return msg


    def scan_callback(self, msg):

        # it seems the lidar front is not aligned with the robot base link, so we need to adjust the angle
        source_frame = msg.header.frame_id
        target_frame = 'base_link'

        if self.tf_buffer.can_transform(target_frame, source_frame, rclpy.time.Time()):

            t = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )

            roll, pitch, yaw = euler_from_quaternion(
                [t.transform.rotation.x,
                 t.transform.rotation.y,
                 t.transform.rotation.z,
                 t.transform.rotation.w]
            )
            angle_between_frames = math.degrees(yaw)


            # print(f"Angle difference between lidar and base_link (in degree): {angle_between_frames}")

            step = math.degrees(msg.angle_increment)

            # Get the index of the front direction in the scan data
            front_index = int(int((0.0 - msg.angle_min) / msg.angle_increment) - int(angle_between_frames / step))
            # print(f"Front index: {front_index}")

            min_front_index = front_index - 64
            max_front_index = front_index + 64

            front_ranges = msg.ranges[min_front_index:max_front_index]

            right_front_index = front_index - 128
            min_right_front_index = right_front_index - 64
            max_right_front_index = right_front_index + 64

            if min_right_front_index < 0:
                min_front_index += len(msg.ranges)

                f_right_ranges = msg.ranges[min_right_front_index:]
                f_right_ranges.extend(msg.ranges[:max_right_front_index])

            right_index = right_front_index - 128

            if right_index < 0:
                right_index += len(msg.ranges)
            
            min_right_index = right_index - 64
            max_right_index = right_index + 64

            right_ranges = msg.ranges[min_right_index:max_right_index]


            left_front_index = front_index + 128

            min_left_front_index = left_front_index - 64
            max_left_front_index = left_front_index + 64

            left_front_ranges = msg.ranges[min_left_front_index:max_left_front_index]


            left_index = left_front_index + 128

            min_left_index = left_index - 64
            max_left_index = left_index + 64

            left_ranges = msg.ranges[min_left_index:max_left_index]
            
            # print(f"right: {right_index}, fright: {right_front_index}, front: {front_index}, fleft: {left_front_index}, left: {left_index}")

            global regions_
            regions_ = {
            'right':  min(min(right_ranges), 10),
            'fright': min(min(f_right_ranges), 10),
            'front':  min(min(front_ranges), 10),
            'fleft':  min(min(left_front_ranges), 10),
            'left':   min(min(left_ranges), 10),
            }


            self.take_action()

            twist = Twist()


            if self.state_ == 0:
                twist = self.find_wall()
            elif self.state_ == 1:
                twist = self.turn_left()
            elif self.state_ == 2:
                twist = self.follow_the_wall()  
            elif self.state_ == 3:
                twist = self.turn_right()
            else:
                self.get_logger().warn(f"Unknown state: {self.state_}. No action taken.")
            
            
            self.cmd_pub.publish(twist)


            
            
            
        else:
            self.get_logger().warn(f"Cannot transform from {source_frame} to {target_frame}. Waiting for transform...")
def main(args=None):
    try:
        rclpy.init(args=args)
        node = WallFollower()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Wall follower node stopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

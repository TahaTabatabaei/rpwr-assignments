#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import euler_from_quaternion

import math

class ObstacleAvoider(Node):

    def __init__(self):
        super().__init__('obstacle_avoider')

        # to handle transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, qos=10)

        # Publisher to cmd_vel_unstamped
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_unstamped', 10)

        # Subscriber to scan data
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Movement parameters
        self.forward_speed = 0.15
        self.turn_speed = 0.2
        self.min_distance = 1  # meters

        self.get_logger().info("Obstacle Avoider node started")

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


            print(f"Angle difference between lidar and base_link (in degree): {angle_between_frames}")

            step = math.degrees(msg.angle_increment)

            # Get the index of the front direction in the scan data
            front_index = int(int((0.0 - msg.angle_min) / msg.angle_increment) - int(angle_between_frames / step))
            print(f"Front index: {front_index}")

            # set about 60 degrees field of view
            min_front_index = front_index - 30
            max_front_index = front_index + 30
            
            # get the minimum distance in the front direction
            front_distance = msg.ranges[min_front_index:max_front_index]
            front_distance = min(front_distance)
            print(f"Front distance: {front_distance}")

            twist = Twist()

            if front_distance > self.min_distance:
                # Move forward
                twist.linear.x = self.forward_speed
                twist.angular.z = 0.0
                self.get_logger().info("Moving forward")
            else:
                # Turn in place
                twist.linear.x = 0.0
                twist.angular.z = self.turn_speed
                self.get_logger().info("Obstacle detected! Turning")

            self.cmd_pub.publish(twist)

        else:
            self.get_logger().warn(f"Cannot transform from {source_frame} to {target_frame}. Waiting for transform...")
def main(args=None):
    try:
        rclpy.init(args=args)
        node = ObstacleAvoider()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Obstacle Avoider node stopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

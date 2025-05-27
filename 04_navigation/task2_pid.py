

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import euler_from_quaternion

import math
import time

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
        # self.forward_speed = 0.1
        # self.turn_speed = 0.4
        self.min_distance = 1  # meters
        self.telorance = 0.05  # meters
        self.Kp = 0.25
        self.Ki = 0.1
        self.Kd = 0.05
        self.integral = 0.0
        self.last_error = 0
        self.last_time = self.get_clock().now()
        self.state = 1

        self.get_logger().info("Obstacle Avoider node started")


    def scan_callback(self, msg):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9  # seconds as float
        self.last_time = current_time


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

            # set about 45 degrees field of view
            min_front_index = front_index - 45
            max_front_index = front_index + 45
            
            # get the minimum distance in the front direction
            front_distance = msg.ranges[min_front_index:max_front_index]
            front_distance = min(front_distance)
            print(f"Front distance: {front_distance}")

            right_index = int(front_index - int(90 / step)) 
            print(f"Right index: {right_index}")

            # set about 60 degrees field of view
            min_right_index = right_index - 45
            max_right_index = right_index + 45

            # get the minimum distance in the right direction
            if min_right_index < 0:
                min_right_index += len(msg.ranges)

                right_distance = msg.ranges[min_right_index:]
                right_distance.extend(msg.ranges[:max_right_index])
            
            else:
                right_distance = msg.ranges[min_right_index:max_right_index]

            right_distance = min(right_distance)
            print(f"Right distance: {right_distance}")

            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0

            # print(f"Current state: {self.state}, Previous move: {self.previous_move}")

            # PID control logic for wall following
            if self.state == 1:
                if front_distance < self.min_distance:
                    # turn left
                    twist.linear.x = 0.0
                    twist.angular.z = 0.4
                    self.get_logger().info("Obstacle detected in front, turning left")
                    self.cmd_pub.publish(twist)
                    self.state = 2
                else:
                    twist.linear.x = 0.1
                    twist.angular.z = 0.0
                    self.cmd_pub.publish(twist)
                    self.get_logger().info("Moving forward")

            if self.state == 2:
                error = self.min_distance - right_distance

                self.integral +=  error * dt
                derivative = (error - self.last_error) / dt
                angular_velocity = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

                max_ang_vel = 0.3  # rad/s (or whatever your robot supports)
                angular_velocity = max(min(angular_velocity, max_ang_vel), -max_ang_vel)

                twist.linear.x = 0.1
                twist.angular.z = -angular_velocity
                self.cmd_pub.publish(twist)
                self.last_error = error

                self.get_logger().info(f"Adjusting angle: {angular_velocity:.2f} rad/s")
            

                

            
            
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

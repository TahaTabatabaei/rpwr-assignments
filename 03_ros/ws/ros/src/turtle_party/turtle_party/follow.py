import math

from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from turtlesim.srv import Spawn


class FollowerNode(Node):
    def __init__(self):
        super().__init__('followerNode')

        # Declare and read parameters
        self.declare_parameter('turtle_name', 'turtle2')
        self.declare_parameter('target_frame', 'turtle1')

        self.turtle_name = self.get_parameter('turtle_name').get_parameter_value().string_value
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value


        self.get_logger().info(f"{self.turtle_name} will follow {self.target_frame}")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.spawner = self.create_client(Spawn, 'spawn')
        self.turtle_spawning_service_ready = False
        self.turtle_spawned = False
        self.publisher = self.create_publisher(Twist, f'{self.turtle_name}/cmd_vel', 1)

        self.timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        if self.turtle_spawning_service_ready:
            if self.turtle_spawned:
                try:
                    t = self.tf_buffer.lookup_transform(
                        self.turtle_name,
                        self.target_frame,
                        rclpy.time.Time())
                except TransformException as ex:
                    self.get_logger().info(
                        f'Could not transform {self.turtle_name} to {self.target_frame}: {ex}')
                    return

                msg = Twist()
                msg.angular.z = 1.0 * math.atan2(
                    t.transform.translation.y,
                    t.transform.translation.x)
                msg.linear.x = 0.5 * math.sqrt(
                    t.transform.translation.x ** 2 +
                    t.transform.translation.y ** 2)

                self.publisher.publish(msg)
            else:
                if self.result.done():
                    self.get_logger().info(
                        f'Successfully spawned {self.result.result().name}')
                    self.turtle_spawned = True
                else:
                    self.get_logger().info('Spawn is not finished')
        else:
            if self.spawner.service_is_ready():
                request = Spawn.Request()
                request.name = self.turtle_name
                request.x = 2.0 + hash(self.turtle_name) % 5
                request.y = 2.0
                request.theta = 0.0
                self.result = self.spawner.call_async(request)
                self.turtle_spawning_service_ready = True
            else:
                self.get_logger().info('Spawn service not ready yet')


def main():
    rclpy.init()
    node = FollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

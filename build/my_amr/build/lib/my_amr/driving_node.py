#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


class AmrDrivingNode(Node):
    def __init__(self):
        super().__init__("velocity_publisher")
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.5

# seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0


    
    def timer_callback(self):
        left_vel = 0.5  # Set desired left wheel velocity
        right_vel = 0.5  # Set desired right wheel velocity
        msg = Twist()
        msg.linear.x = left_vel
        msg.linear.y = right_vel
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: left_vel: {left_vel}, right_vel: {right_vel}")
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    velocity_publisher = AmrDrivingNode()
    rclpy.spin(velocity_publisher)
    velocity_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
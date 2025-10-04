import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class MovingNode(Node):
    def __init__(self):
        super().__init__('go_to')
        self.publisher_ = self.create_publisher(
            Twist,
            'turtle1/cmd_vel',
            10
        )
        self.subscription = self.create_subscription(
            Pose,
            'turtle1/pose',
            self.listener_callback,
            10
        )
        self.declare_parameter('coords', [0.0, 0.0, 0.0])
        coords = self.get_parameter('coords').get_parameter_value().double_array_value
        self.goal_x = coords[0]
        self.goal_y = coords[1]
        self.goal_theta = coords[2]

        self.epsilon = 0.1

        self.get_logger().info(f'Go to x: {self.goal_x}, y: {self.goal_y}, theta: {self.goal_theta:.2f}')

    def listener_callback(self, msg: Pose):
        x = msg.x
        y = msg.y
        theta = msg.theta

        dx = self.goal_x - x
        dy = self.goal_y - y
        distance = math.sqrt(dx**2 + dy**2)
        
        twist = Twist()

        if distance < self.epsilon:
            angle_error = self.goal_theta - theta
            angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

            if abs(angle_error) > self.epsilon:
                twist.angular.z = 1.5 * angle_error
            else:
                self.publisher_.publish(twist)
                self.get_logger().info("Goal reached!")
                self.destroy_node()
                rclpy.shutdown()
                return

        else:
            goal_theta = math.atan2(dy, dx)
            angle_error = goal_theta - theta
            angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

            if abs(angle_error) > self.epsilon:
                twist.angular.z = 1.5 * angle_error
            else:
                twist.linear.x = 1.5 * distance

        self.publisher_.publish(twist)
        
        
def main():
    rclpy.init()

    moving_node = MovingNode()

    rclpy.spin(moving_node)

if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Vector3


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.msg = {
            "turn_right": Twist(angular=Vector3(x=0.0, y=0.0, z=-1.6)),
            "turn_left": Twist(angular=Vector3(x=0.0, y=0.0, z=1.6)),
            "move_forward": Twist(linear=Vector3(x=1.0, y=0.0, z=0.0)),
            "move_backward": Twist(linear=Vector3(x=-1.0, y=0.0, z=0.0)),
        }
        print(f'Use the commands below to move the turtle.\nturn_right | turn_left | move_forward | move_backward\n')
        while True:
            self.publish()

    def publish(self):
        command = input()
        if self.msg.get(command, None):
            self.publisher_.publish(self.msg[command])
            self.get_logger().info('Publishing: "%s"' % self.msg[command])


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

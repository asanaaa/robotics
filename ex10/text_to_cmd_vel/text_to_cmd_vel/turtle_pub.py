import rclpy
from rclpy.node import Node

from custom_msg.msg import Command
from geometry_msgs.msg import Twist, Vector3


class ControlNode(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        super().__init__('minimal_publisher')
        # Subscriber
        self.subscription = self.create_subscription(
            Command,
            'cmd_text',
            self.listener_callback,
            10)
        self.subscription

        # Publisher
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.msg = {
            "turn_right": Twist(angular=Vector3(x=0.0, y=0.0, z=-1.6)),
            "turn_left": Twist(angular=Vector3(x=0.0, y=0.0, z=1.6)),
            "move_forward": Twist(linear=Vector3(x=1.0, y=0.0, z=0.0)),
            "move_backward": Twist(linear=Vector3(x=-1.0, y=0.0, z=0.0)),
        }

    def listener_callback(self, msg):
        self.publisher_.publish(self.msg[msg.command])
        self.get_logger().info('Publishing: "%s"' % self.msg[msg.command])

def main(args=None):
    rclpy.init(args=args)

    control_node = ControlNode()

    rclpy.spin(control_node)

    control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

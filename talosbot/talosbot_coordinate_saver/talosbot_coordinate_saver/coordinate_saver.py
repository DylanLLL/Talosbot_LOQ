import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        odom = msg.pose.pose
        x = odom.position.x
        y = odom.position.y
        z = odom.orientation.z
        w = odom.orientation.w
        self.get_logger().info(f'\n\n<TalosbotNavigateToPose action_name="/navigate_to_pose" goal_x="{x:.3f}" goal_y="{y:.3f}" quat_w="{w:.3f}" quat_z="{z:.3f}"/>')
        exit()


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from nav_msgs.msg import Odometry


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.target_frame = "base_footprint"
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.1, self.on_timer)


    def on_timer(self):
        from_frame_rel = self.target_frame
        to_frame_rel = 'map'

        try:
            now =rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                now
            )
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            z = trans.transform.rotation.z
            w = trans.transform.rotation.w
            self.get_logger().info(f'\n\n<TalosbotNavigateToPose action_name="/navigate_to_pose" goal_x="{x:.3f}" goal_y="{y:.3f}" quat_w="{w:.3f}" quat_z="{z:.3f}"/>')
        except TransformException as ex:
            self.get_logger().info(f'\n\nInvalid')

       
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
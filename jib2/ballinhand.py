""""
This is a demonstration of Atlas with a ball in hand. The ball is attached to the left hand and follows the hand's position.
The ball is visualized as a red sphere in RViz. The ball's position is updated at a fixed rate. The ball can be detached
from the hand by setting the 'attached' flag to False. The ball will then remain at its last position.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Point, Vector3, Quaternion
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener, LookupException
from hw5code.TransformHelpers import Point_from_p
import numpy as np


class BallInHandNode(Node):
    def __init__(self, name, rate):
        super().__init__(name)

        # Set up the publisher for the ball visualization marker
        quality = QoSProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)
        self.pub = self.create_publisher(MarkerArray, '/visualization_marker_array', quality)

        # Set up TF2 transform listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Initialize the ball's marker properties
        self.radius = 0.1
        diam = 2 * self.radius
        self.ball_marker = Marker()
        self.ball_marker.header.frame_id = "world"
        self.ball_marker.action = Marker.ADD
        self.ball_marker.ns = "ball"
        self.ball_marker.id = 1
        self.ball_marker.type = Marker.SPHERE
        self.ball_marker.scale = Vector3(x=diam, y=diam, z=diam)
        self.ball_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)

        # Initialize ball and hand position
        self.ball_position = np.array([0.0, 0.0, 1.0])  # Initial guess
        self.attached = True  # Ball starts attached to the hand

        # Timer for updating the ball's position
        self.dt = 1.0 / float(rate)
        self.create_timer(self.dt, self.update)
        self.get_logger().info("BallInHandNode running at {} Hz".format(rate))

    def get_hand_position(self):
        """Get the position of the hand relative to the world frame."""
        try:
            transform = self.tf_buffer.lookup_transform("world", "l_hand", rclpy.time.Time())
            hand_position = np.array([
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            ])
            return hand_position
        except LookupException:
            self.get_logger().warn("Transform from 'world' to 'l_hand' not found.")
            return None

    def update(self):
        """Update the ball's position to follow the hand."""
        if self.attached:
            hand_position = self.get_hand_position()
            if hand_position is not None:
                # Ball follows the hand's position
                self.ball_position = hand_position

        # Update the marker message
        self.ball_marker.header.stamp = self.get_clock().now().to_msg()
        self.ball_marker.pose.position = Point_from_p(self.ball_position)

        # Publish the updated marker
        marker_array = MarkerArray(markers=[self.ball_marker])
        self.pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = BallInHandNode('ballinhand', 100)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
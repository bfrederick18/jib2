""""
This is a demonstration of Atlas with a ball in hand. The ball is attached to the left hand and follows the hand's position.
The ball is visualized as a red sphere in RViz. The ball's position is updated at a fixed rate. The ball can be detached
from the hand by setting the 'attached' flag to False. The ball will then remain at its last position.
"""

import rclpy
from rclpy.node                 import Node
from rclpy.qos                  import QoSProfile, DurabilityPolicy
from rclpy.time                 import Duration
from geometry_msgs.msg          import Point, Vector3, Quaternion
from std_msgs.msg               import ColorRGBA
from visualization_msgs.msg     import Marker, MarkerArray
from tf2_ros                    import Buffer, TransformListener, LookupException
from hw5code.TransformHelpers   import Point_from_p
import numpy as np

from hw5code.TransformHelpers   import *
from hw6code.KinematicChain     import *


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

        self.p = np.array([0.0, 0.0, self.radius])
        self.v = np.array([0.0, 0.0,  0.0       ])
        self.a = np.array([0.0, 0.0, -9.81      ])

        diam = 2 * self.radius
        self.ball_marker = Marker()
        self.ball_marker.header.frame_id = "world"
        self.ball_marker.header.stamp     = self.get_clock().now().to_msg()
        self.ball_marker.action = Marker.ADD
        self.ball_marker.ns = "ball"
        self.ball_marker.id = 1
        self.ball_marker.type = Marker.SPHERE
        self.ball_marker.pose.orientation = Quaternion()
        self.ball_marker.pose.position    = Point_from_p(self.p)
        self.ball_marker.scale = Vector3(x=diam, y=diam, z=diam)
        self.ball_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)

        self.marker_array = MarkerArray(markers=[self.ball_marker])

        # Initialize ball and hand position
        self.inhand = True  # Ball starts attached to the hand

        # Timer for updating the ball's position
        self.dt = 1.0 / float(rate)
        self.t  = -self.dt
        self.start = self.get_clock().now() + Duration(seconds=self.dt)
        self.create_timer(self.dt, self.update)
        self.get_logger().info("BallInHandNode running at {} Hz".format(rate))

    # Return the current time (in ROS format).
    def now(self):
        return self.start + Duration(seconds=self.t)

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
        # To avoid any time jitter enforce a constant time step and
        # integrate to get the current time.
        self.t += self.dt

        print(self.t)

        if self.t > 2.0:
            self.inhand = False

        if self.inhand:
            hand_position = self.get_hand_position()
            if hand_position is not None:
                # Ball follows the hand's position
                self.p = hand_position
            else:
                print("Hand position not found")

            
        else:

            k = 0.05
            v_mag = np.linalg.norm(self.v)
            drag = -k * v_mag * self.v

            self.a = np.array([0.0, 0.0, -9.81]) + drag
            
            # Integrate the velocity, then the position.
            self.v += self.dt * self.a
            self.p += self.dt * self.v

            # Check for a bounce - not the change in x velocity is non-physical.
            if self.p[2] < self.radius:
                self.p[2] = self.radius + (self.radius - self.p[2])
                self.v[2] *= -1.0
                self.v[0] *= 0.0

        # Update the message and publish.
        self.ball_marker.header.stamp  = self.now().to_msg()
        self.ball_marker.pose.position = Point_from_p(self.p)
        self.pub.publish(self.marker_array)


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
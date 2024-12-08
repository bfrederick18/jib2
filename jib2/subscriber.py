"""subsciber.py

   Subscribe to all GUI topics.  You probably only want a subset.

   Node:      /subscriber
   Subscribe: /boolean          std_msgs.msg.Bool
   Subscribe: /float            std_msgs.msg.Float64
   Subscribe: /point            geometry_msgs.msg.PointStamped
   Subscribe: /pose             geometry_msgs.msg.PoseStamped
"""

import rclpy

from rclpy.node                 import Node
from std_msgs.msg               import Bool, Float64

from hw5code.TransformHelpers   import *


#
#   Demo Node Class
#
class GUINode(Node):
    # Initialization.
    def __init__(self, name):
        # Initialize the node, naming it as specified.
        super().__init__(name)

        # Create the subscribers.
        self.create_subscription(Float64,      '/speed',   self.CB_speed, 10)
        self.create_subscription(Bool,         '/fired', self.CB_fired,  10)

        # Initialize the local values.
        self.speed  = 0.0
        self.fired = False

    # Callback functions.
    def CB_speed(self, msg):
        self.speed = msg.data
        self.get_logger().info(f"Received speed: {self.speed}")

    def CB_fired(self, msg):
        self.fired = msg.data
        self.get_logger().info(f"Received fired status: {self.fired}")


#
#  Main Code
#
def main(args=None):
    # Initialize ROS and the demo node.
    rclpy.init(args=args)
    node = GUINode('subscriber')

    # Spin, until interrupted.
    rclpy.spin(node)

    # Shutdown the node and ROS.
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
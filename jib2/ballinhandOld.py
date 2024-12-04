"""subsciber.py

   Node:      /subscriber
   Subscribe: /boolean          std_msgs.msg.Bool
   Subscribe: /float            std_msgs.msg.Float64
   Subscribe: /point            geometry_msgs.msg.PointStamped
   Subscribe: /pose             geometry_msgs.msg.PoseStamped


   Subscribe: /joint_states     sensor_msgs/msg/JointState
"""

import rclpy

from rclpy.node                 import Node
from std_msgs.msg               import Bool, Float64
from geometry_msgs.msg          import PointStamped, PoseStamped
from sensor_msgs                import JointState

from hw5code.TransformHelpers   import *


#
#   Demo Node Class
#
class DemoNode(Node):
    # Initialization.
    def __init__(self, name):
        # Initialize the node, naming it as specified.
        super().__init__(name)

        # Create the subscribers.
        self.create_subscription(Bool,         '/boolean', self.CB_bool,  10)
        self.create_subscription(Float64,      '/float',   self.CB_float, 10)
        self.create_subscription(PointStamped, '/point',   self.CB_point, 10)
        self.create_subscription(PoseStamped,  '/pose',    self.CB_pose,  10)

        # Initialize the local values.
        self.bool  = False
        self.float = 0.0
        self.p     = pzero()
        self.R     = Reye()

    # Callback functions.
    def CB_bool(self, msg):
        self.bool = msg.data
        self.get_logger().info(f"Received boolean {self.bool}")

    def CB_float(self, msg):
        self.float = msg.data
        self.get_logger().info(f"Received float {self.float}")

    def CB_point(self, msg):
        self.p = p_from_Point(msg.point)
        self.get_logger().info(f"Received point: \np:\n{self.p}")

    def CB_pose(self, msg):
        T = T_from_Pose(msg.pose)
        self.p = p_from_T(T)
        self.R = R_from_T(T)
        self.get_logger().info(f"Received pose: \np:\n{self.p}\nR:\n{self.R}")


#
#  Main Code
#
def main(args=None):
    # Initialize ROS and the demo node.
    rclpy.init(args=args)
    node = DemoNode('subscriber')

    # Spin, until interrupted.
    rclpy.spin(node)

    # Shutdown the node and ROS.
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
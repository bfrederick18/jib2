'''squat.py

   This is a demo for moving/placing an ungrounded robot and moving joints.

   In particular, imagine a humanoid robot.  This moves/rotates the
   pelvis frame relative to the world.  And waves an arm.

   Node:      /squat
   Publish:   /joint_states             sensor_msgs.msg.JointState
   Broadcast: 'pelvis' w.r.t. 'world'   geometry_msgs.msg.TransformStamped

'''

import rclpy
import numpy as np

from math import pi, sin, cos, acos, atan2, sqrt, fmod, exp

from rclpy.node                 import Node
from rclpy.time                 import Duration
from tf2_ros                    import TransformBroadcaster
from geometry_msgs.msg          import TransformStamped
from sensor_msgs.msg            import JointState

from hw5code.TransformHelpers     import *


#
#   Atlas Joint Names
#
jointnames = ['l_leg_hpx', 'l_leg_hpy', 'l_leg_hpz',
              'l_leg_kny',
              'l_leg_akx', 'l_leg_aky',

              'r_leg_hpx', 'r_leg_hpy', 'r_leg_hpz',
              'r_leg_kny',
              'r_leg_akx', 'r_leg_aky',

              'back_bkx', 'back_bky', 'back_bkz',
              'neck_ry',

              'l_arm_elx', 'l_arm_ely',
              'l_arm_shx', 'l_arm_shz',
              'l_arm_wrx', 'l_arm_wry', 'l_arm_wry2',

              'r_arm_elx', 'r_arm_ely',
              'r_arm_shx', 'r_arm_shz',
              'r_arm_wrx', 'r_arm_wry', 'r_arm_wry2']


#
#   Demo Node Class
#
class DemoNode(Node):
    def __init__(self, name, rate):
        super().__init__(name)

        self.broadcaster = TransformBroadcaster(self)
        self.pub = self.create_publisher(JointState, '/joint_states', 10)

        self.get_logger().info("Waiting for a /joint_states subscriber...")
        while(not self.count_subscribers('/joint_states')):
            pass

        self.dt    = 1.0 / float(rate)
        self.t     = -self.dt
        self.start = self.get_clock().now() + Duration(seconds=self.dt)

        self.create_timer(self.dt, self.update)
        self.get_logger().info("Running with dt of %f seconds (%fHz)" %
                               (self.dt, rate))
        
        self.X_PELVIS = 0.0
        self.Y_PELVIS = 0.5
        self.Z_PELVIS_TOP = 0.85
        self.Z_PELVIS_LOW = 0.55

        self.z_mid = (self.Z_PELVIS_TOP + self.Z_PELVIS_LOW) / 2
        self.z_A = (self.Z_PELVIS_TOP - self.Z_PELVIS_LOW) / 2  # amplitude

        # Inverse kinamatic variables


    def shutdown(self):
        self.destroy_node()


    def now(self):
        return self.start + Duration(seconds=self.t)


    def update(self):
        self.t += self.dt

        z_pelvis = self.z_mid + self.z_A * cos(self.t)
        p_pelvis = pxyz(self.X_PELVIS, self.Y_PELVIS, z_pelvis)
        R_pelvis = Reye()
        T_pelvis = T_from_Rp(R_pelvis, p_pelvis)
        
        trans = TransformStamped()
        trans.header.stamp    = self.now().to_msg()
        trans.header.frame_id = 'world'
        trans.child_frame_id  = 'pelvis'
        trans.transform       = Transform_from_T(T_pelvis)
        self.broadcaster.sendTransform(trans)

        q    = np.zeros(len(jointnames))
        qdot = np.zeros(len(jointnames))
        
        """ OLD SHIT WE DUNNO MAYBE THE WAVE
        i_rknee_y = jointnames.index('r_leg_kny')
        i_lknee_y = jointnames.index('l_leg_kny')

        i_relbow = jointnames.index('r_arm_elx')

        q[i_relbow]    = - pi/2 + pi/8 * sin(2*self.t)
        qdot[i_relbow] =          pi/4 * cos(2*self.t)
        """

        cmdmsg = JointState()
        cmdmsg.header.stamp = self.now().to_msg()       # Current time for ROS
        cmdmsg.name         = jointnames                # List of names
        cmdmsg.position     = q.flatten().tolist()      # List of positions
        cmdmsg.velocity     = qdot.flatten().tolist()   # List of velocities
        self.pub.publish(cmdmsg)


#
#  Main Code
#
def main(args=None):
    rclpy.init(args=args)
    node = DemoNode('squat', 100)

    rclpy.spin(node)

    node.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
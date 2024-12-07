import rclpy
import numpy as np
from math import pi, sin, cos
from rclpy.node import Node
from rclpy.time import Duration
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from hw5code.TransformHelpers import *
from hw6code.KinematicChain import KinematicChain

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

        # ROS2 setup
        self.broadcaster = TransformBroadcaster(self)
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.get_logger().info("Waiting for a /joint_states subscriber...")
        while not self.count_subscribers('/joint_states'):
            pass

        # Timing setup
        self.dt = 1.0 / float(rate)
        self.t = 0.0
        self.start = self.get_clock().now() + Duration(seconds=self.dt)

        # Initialize pelvis motion parameters
        self.X_PELVIS = 0.0
        self.Y_PELVIS = 0.5
        self.Z_PELVIS_TOP = 0.85
        self.Z_PELVIS_LOW = 0.55
        self.z_pelvis_mid = (self.Z_PELVIS_TOP + self.Z_PELVIS_LOW) / 2
        self.z_pelvis_A = (self.Z_PELVIS_TOP - self.Z_PELVIS_LOW) / 2

        # Initialize kinematic chain for the left leg
        self.chain_left_leg = KinematicChain(self, 'pelvis', 'l_foot', [jointnames[i] for i in [2, 0, 1, 3, 5, 4]])

        # Joint positions and velocities
        self.q = np.zeros(len(jointnames))
        self.qdot = np.zeros(len(jointnames))

        # Initial foot position
        (ptip, Rtip, _, _) = self.chain_left_leg.fkin(self.q[0:6]) 
        self.pd = ptip + pxyz(self.X_PELVIS, self.Y_PELVIS, self.Z_PELVIS_TOP)

        # Foot transform
        self.Td = T_from_Rp(Reye(), self.pd)

        # Controller gains
        self.K_p = 10.0
        self.gamma = 0.1

        # Timer for periodic updates
        self.create_timer(self.dt, self.update)
        self.get_logger().info(f"Running with dt of {self.dt} seconds ({rate}Hz)")

    def now(self):
        """Returns the current ROS2 time adjusted for simulation start."""
        return self.start + Duration(seconds=self.t)

    def update(self):
        """Control loop for squatting motion."""
        self.t += self.dt

        # Target pelvis position
        z_pelvis = self.z_pelvis_mid + self.z_pelvis_A * cos(self.t)
        p_pelvis = pxyz(self.X_PELVIS, self.Y_PELVIS, z_pelvis)
        R_pelvis = Reye()
        T_pelvis = T_from_Rp(R_pelvis, p_pelvis)

        # Broadcast pelvis transform
        trans = TransformStamped()
        trans.header.stamp = self.now().to_msg()
        trans.header.frame_id = 'world'
        trans.child_frame_id = 'pelvis'
        trans.transform = Transform_from_T(T_pelvis)
        self.broadcaster.sendTransform(trans)

        # Target foot position and velocity
        z_foot = self.z_pelvis_mid - self.z_pelvis_A * cos(self.t)
        self.pd = pxyz(self.X_PELVIS, self.Y_PELVIS, z_foot)
        vd = np.array([0.0, 0.0, 0.0])

        Td = np.linalg.inv(T_pelvis) @ self.Td
        self.pd = [i[3] for i in Td[0:3]]

        # Perform forward kinematics
        (ptip, Rtip, Jv, Jw) = self.chain_left_leg.fkin(self.q[0:6])

        # Compute position error
        err_pos = self.pd - ptip

        # Form full error vector (position only, no orientation control)
        err = np.concatenate((err_pos, np.zeros(3)))

        # Combine Jacobians
        J = np.vstack((Jv, Jw))

        # Weighted pseudoinverse of the Jacobian
        weight_mat = self.gamma**2 * np.eye(6)
        Jwinv = np.linalg.pinv(J.T @ J + weight_mat) @ J.T

        # Compute nominal secondary task velocities
        q0 = np.zeros(6)  # Nominal joint positions
        qsdot = -10 * (self.q[0:6] - q0)

        # Compute joint velocities
        self.qdot[0:6] = Jwinv @ (np.concatenate((vd, np.zeros(3))) + self.K_p * err) #+ np.eye(6) - Jwinv @ J) @ qsdot

        # Update joint positions
        self.q[0:6] += self.qdot[0:6] * self.dt

        # Publish joint states
        cmdmsg = JointState()
        cmdmsg.header.stamp = self.now().to_msg()
        cmdmsg.name = jointnames
        cmdmsg.position = self.q.tolist()
        cmdmsg.velocity = self.qdot.tolist()
        self.pub.publish(cmdmsg)


#
#  Main Code
#
def main(args=None):
    rclpy.init(args=args)
    node = DemoNode('squat', 100)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
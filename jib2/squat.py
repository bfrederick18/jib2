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


JOINT_NAMES = ['l_leg_hpx', 'l_leg_hpy', 'l_leg_hpz',
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


JOINT_ORDERS = {
    'l_leg': [
        JOINT_NAMES.index('l_leg_hpz'), 
        JOINT_NAMES.index('l_leg_hpx'), 
        JOINT_NAMES.index('l_leg_hpy'), 
        JOINT_NAMES.index('l_leg_kny'), 
        JOINT_NAMES.index('l_leg_aky'), 
        JOINT_NAMES.index('l_leg_akx')
    ], 

    'r_leg': [
        JOINT_NAMES.index('r_leg_hpz'), 
        JOINT_NAMES.index('r_leg_hpx'), 
        JOINT_NAMES.index('r_leg_hpy'), 
        JOINT_NAMES.index('r_leg_kny'), 
        JOINT_NAMES.index('r_leg_aky'), 
        JOINT_NAMES.index('r_leg_akx')
    ], 

    'head': [
        JOINT_NAMES.index('back_bkz'), 
        JOINT_NAMES.index('back_bky'), 
        JOINT_NAMES.index('back_bkx'), 
        JOINT_NAMES.index('neck_ry')
    ], 

    'l_hand': [
        JOINT_NAMES.index('back_bkz'), 
        JOINT_NAMES.index('back_bky'), 
        JOINT_NAMES.index('back_bkx'), 
        JOINT_NAMES.index('l_arm_shz'), 
        JOINT_NAMES.index('l_arm_shx'), 
        JOINT_NAMES.index('l_arm_ely'), 
        JOINT_NAMES.index('l_arm_elx'), 
        JOINT_NAMES.index('l_arm_wry'), 
        JOINT_NAMES.index('l_arm_wrx'), 
        JOINT_NAMES.index('l_arm_wry2')],

    'r_hand': [
        JOINT_NAMES.index('back_bkz'), 
        JOINT_NAMES.index('back_bky'), 
        JOINT_NAMES.index('back_bkx'), 
        JOINT_NAMES.index('r_arm_shz'), 
        JOINT_NAMES.index('r_arm_shx'), 
        JOINT_NAMES.index('r_arm_ely'), 
        JOINT_NAMES.index('r_arm_elx'), 
        JOINT_NAMES.index('r_arm_wry'), 
        JOINT_NAMES.index('r_arm_wrx'), 
        JOINT_NAMES.index('r_arm_wry2')]
}


class DemoNode(Node):
    def __init__(self, name, rate):
        super().__init__(name)

        self.broadcaster = TransformBroadcaster(self)
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.get_logger().info("Waiting for a /joint_states subscriber...")
        while not self.count_subscribers('/joint_states'):
            pass

        self.dt = 1.0 / float(rate)
        self.t = 0.0
        self.start = self.get_clock().now() + Duration(seconds=self.dt)

        self.X_PELVIS = 0.0
        self.Y_PELVIS = 0.5
        self.Z_PELVIS_TOP = 0.85
        self.Z_PELVIS_LOW = 0.55
        self.z_pelvis_mid = (self.Z_PELVIS_TOP + self.Z_PELVIS_LOW) / 2
        self.z_pelvis_A = (self.Z_PELVIS_TOP - self.Z_PELVIS_LOW) / 2

        # Correct joint order for KinematicChain
        self.chain_left_leg = KinematicChain(self, 'pelvis', 'l_foot', [JOINT_NAMES[i] for i in JOINT_ORDERS['l_leg']])
        self.chain_right_leg = KinematicChain(self, 'pelvis', 'r_foot', [JOINT_NAMES[i] for i in JOINT_ORDERS['r_leg']])
        self.chain_head = KinematicChain(self, 'pelvis', 'head', [JOINT_NAMES[i] for i in JOINT_ORDERS['head']])
        self.chain_left_hand = KinematicChain(self, 'pelvis', 'l_hand', [JOINT_NAMES[i] for i in JOINT_ORDERS['l_hand']])
        self.chain_right_hand = KinematicChain(self, 'pelvis', 'r_hand', [JOINT_NAMES[i] for i in JOINT_ORDERS['r_hand']])

        self.q = np.zeros(len(JOINT_NAMES))
        self.qdot = np.zeros(len(JOINT_NAMES))

        # Forward kinematics with correct joint order
        (ptip, Rtip, _, _) = self.chain_left_leg.fkin([self.q[i] for i in [2, 0, 1, 3, 5, 4]])
        self.p_foot_l_0 = ptip
        self.pd = self.p_foot_l_0 + pxyz(self.X_PELVIS, self.Y_PELVIS, self.Z_PELVIS_TOP)

        self.Td = T_from_Rp(Reye(), self.pd)

        self.K_p = 10.0
        self.K_s = 10.0
        self.gamma = 0.1

        self.create_timer(self.dt, self.update)
        self.get_logger().info(f"Running with dt of {self.dt} seconds ({rate}Hz)")

    def now(self):
        """Returns the current ROS2 time adjusted for simulation start."""
        return self.start + Duration(seconds=self.t)

    def update(self):
        """Control loop for squatting motion."""
        self.t += self.dt

        z_pelvis = self.z_pelvis_mid + self.z_pelvis_A * cos(self.t)
        p_pelvis = pxyz(self.X_PELVIS, self.Y_PELVIS, z_pelvis)
        R_pelvis = Reye()
        T_pelvis = T_from_Rp(R_pelvis, p_pelvis)
        v_z_pelvis = -self.z_pelvis_A * sin(self.t)
        v_pelvis = np.array([0.0, 0.0, v_z_pelvis])

        trans = TransformStamped()
        trans.header.stamp = self.now().to_msg()
        trans.header.frame_id = 'world'
        trans.child_frame_id = 'pelvis'
        trans.transform = Transform_from_T(T_pelvis)
        self.broadcaster.sendTransform(trans)

        Td = np.linalg.inv(T_pelvis) @ self.Td
        self.pd = [i[3] for i in Td[0:3]]
        vd_foot = -v_pelvis
        wd_foot = np.zeros(3)

        # Forward kinematics with correct joint order
        (ptip, Rtip, Jv, Jw) = self.chain_left_leg.fkin([self.q[i] for i in [2, 0, 1, 3, 5, 4]])

        err_pos = self.pd - ptip
        err_rot = eR(Reye(), Rtip)

        err = np.concatenate((err_pos, err_rot))
        J = np.vstack((Jv, Jw))

        weight_mat = self.gamma**2 * np.eye(6)
        Jwinv = np.linalg.pinv(J.T @ J + weight_mat) @ J.T

        q_nominal = np.array([0.0, 0.0, 0.0, (pi / 2), 0.0, 0.0])
        qsdot = -self.K_s * (self.q[0:6] - q_nominal)

        # Update joint velocities using correct joint order
        self.qdot[0:6] = (
            Jwinv @ (np.concatenate((vd_foot, wd_foot)) + self.K_p * err)
            + (np.eye(6) - Jwinv @ J) @ qsdot
        )
        # Update joint positions using correct joint order
        for idx, mapped_idx in enumerate([2, 0, 1, 3, 5, 4]):
            self.q[mapped_idx] += self.qdot[idx] * self.dt

        cmdmsg = JointState()
        cmdmsg.header.stamp = self.now().to_msg()
        cmdmsg.name = JOINT_NAMES
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
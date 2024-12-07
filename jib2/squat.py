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


class SquatNode(Node):
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

        # Kinematic Chains
        self.chain_lfoot = KinematicChain(self, 'pelvis', 'l_foot', [JOINT_NAMES[i] for i in JOINT_ORDERS['l_leg']])

        self.q = np.zeros(len(JOINT_NAMES))
        self.qdot = np.zeros(len(JOINT_NAMES))

        # Initial Left Foot Position
        ptip0_lfoot, _, _, _ = self.chain_lfoot.fkin([self.q[i] for i in JOINT_ORDERS['l_leg']])
        self.pd_lfoot = ptip0_lfoot + pxyz(self.X_PELVIS, self.Y_PELVIS, self.Z_PELVIS_TOP)

        self.Td_lfoot = T_from_Rp(Reye(), self.pd_lfoot)

        # Control Gains
        self.K_p = 10.0
        self.K_s = 10.0
        self.gamma = 0.1

        self.create_timer(self.dt, self.update)
        self.get_logger().info(f"Running with dt of {self.dt} seconds ({rate}Hz)")

    def now(self):
        return self.start + Duration(seconds=self.t)

    def update(self):
        self.t += self.dt

        z_pelvis = self.z_pelvis_mid + self.z_pelvis_A * cos(self.t)
        p_pelvis = pxyz(self.X_PELVIS, self.Y_PELVIS, z_pelvis)
        R_pelvis = Reye()
        T_pelvis = T_from_Rp(R_pelvis, p_pelvis)
        v_z_pelvis = -self.z_pelvis_A * sin(self.t)
        v_pelvis = np.array([0.0, 0.0, v_z_pelvis])

        # Publish Pelvis Transform
        trans = TransformStamped()
        trans.header.stamp = self.now().to_msg()
        trans.header.frame_id = 'world'
        trans.child_frame_id = 'pelvis'
        trans.transform = Transform_from_T(T_pelvis)
        self.broadcaster.sendTransform(trans)

        # Desired Foot Transform
        Td_lfoot = np.linalg.inv(T_pelvis) @ self.Td_lfoot
        self.pd_lfoot = [i[3] for i in Td_lfoot[0:3]]
        vd_lfoot = -v_pelvis
        wd_lfoot = np.zeros(3)

        # Forward Kinematics for Left Foot
        ptip_lfoot, Rtip_lfoot, Jv_lfoot, Jw_lfoot = self.chain_lfoot.fkin(
            [self.q[i] for i in JOINT_ORDERS['l_leg']]
        )

        # Errors
        err_pos_lfoot = self.pd_lfoot - ptip_lfoot
        err_rot_lfoot = eR(Reye(), Rtip_lfoot)
        err_lfoot = np.concatenate((err_pos_lfoot, err_rot_lfoot))
        J_lfoot = np.vstack((Jv_lfoot, Jw_lfoot))

        # Weighted Pseudoinverse
        weight_mat = self.gamma**2 * np.eye(6)
        Jwinv_lfoot = np.linalg.pinv(J_lfoot.T @ J_lfoot + weight_mat) @ J_lfoot.T

        # Nominal Joint Positions
        q_nominal_lfoot = np.array([0.0, 0.0, 0.0, (pi / 2), 0.0, 0.0])
        qsdot_lfoot = -self.K_s * (self.q[0:6] - q_nominal_lfoot)

        # Update Joint Velocities and Positions
        self.qdot[0:6] = (
            Jwinv_lfoot @ (np.concatenate((vd_lfoot, wd_lfoot)) + self.K_p * err_lfoot)
            + (np.eye(6) - Jwinv_lfoot @ J_lfoot) @ qsdot_lfoot
        )
        for idx, mapped_idx in enumerate(JOINT_ORDERS['l_leg']):
            self.q[mapped_idx] += self.qdot[idx] * self.dt

        # Publish Joint States
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
    node = SquatNode('squat', 100)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

'''throw.py

This is a simple demo node that will make the Atlas robot throw up and down.

   Node:      /throw
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
from hw6code.KinematicChain       import KinematicChain


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


class ThrowNode(Node):
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

        self.chain_lfoot = KinematicChain(self, 'pelvis', 'l_foot', [JOINT_NAMES[i] for i in JOINT_ORDERS['l_leg']])
        self.chain_rfoot = KinematicChain(self, 'pelvis', 'r_foot', [JOINT_NAMES[i] for i in JOINT_ORDERS['r_leg']])
        self.chain_head = KinematicChain(self, 'pelvis', 'head', [JOINT_NAMES[i] for i in JOINT_ORDERS['head']])
        self.chain_lhand = KinematicChain(self, 'pelvis', 'l_hand', [JOINT_NAMES[i] for i in JOINT_ORDERS['l_hand']])
        self.chain_rhand = KinematicChain(self, 'pelvis', 'r_hand', [JOINT_NAMES[i] for i in JOINT_ORDERS['r_hand']])

        self.q = np.zeros(len(JOINT_NAMES))
        self.qdot = np.zeros(len(JOINT_NAMES))

        ptip0_lfoot, _, _, _ = self.chain_lfoot.fkin([self.q[i] for i in JOINT_ORDERS['l_leg']])
        ptip0_rfoot, _, _, _ = self.chain_rfoot.fkin([self.q[i] for i in JOINT_ORDERS['r_leg']])

        self.pd_lfoot = ptip0_lfoot + pxyz(self.X_PELVIS, self.Y_PELVIS, self.Z_PELVIS_TOP)
        self.pd_rfoot = ptip0_rfoot + pxyz(self.X_PELVIS, self.Y_PELVIS, self.Z_PELVIS_TOP)

        self.Td_lfoot = T_from_Rp(Reye(), self.pd_lfoot)
        self.Td_rfoot = T_from_Rp(Reye(), self.pd_rfoot)


        ptip0_rhand, _, _, _ = self.chain_rhand.fkin([self.q[i] for i in JOINT_ORDERS['r_hand']])
        self.pd_rhand = ptip0_rhand + pxyz(self.X_PELVIS, -self.Y_PELVIS * 4, self.Z_PELVIS_TOP)

        self.Td_rhand = T_from_Rp(Reye(), self.pd_rhand)


        self.K_p = 50.0
        self.K_s = 10.0
        self.gamma = 0.1

        self.create_timer(self.dt, self.update)
        self.get_logger().info(f"Running with dt of {self.dt} seconds ({rate}Hz)")


    def shutdown(self):
        self.destroy_node()


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

        trans = TransformStamped()
        trans.header.stamp = self.now().to_msg()
        trans.header.frame_id = 'world'
        trans.child_frame_id = 'pelvis'
        trans.transform = Transform_from_T(T_pelvis)
        self.broadcaster.sendTransform(trans)

        Td_lfoot = np.linalg.inv(T_pelvis) @ self.Td_lfoot
        Td_rfoot = np.linalg.inv(T_pelvis) @ self.Td_rfoot

        self.pd_lfoot = [i[3] for i in Td_lfoot[0:3]]
        self.pd_rfoot = [i[3] for i in Td_rfoot[0:3]]

        vd_lfoot = vd_rfoot = -v_pelvis
        wd_lfoot = wd_rfoot = np.zeros(3)

        ptip_lfoot, Rtip_lfoot, Jv_lfoot, Jw_lfoot = self.chain_lfoot.fkin(
            [self.q[i] for i in JOINT_ORDERS['l_leg']]
        )
        ptip_rfoot, Rtip_rfoot, Jv_rfoot, Jw_rfoot = self.chain_rfoot.fkin(
            [self.q[i] for i in JOINT_ORDERS['r_leg']]
        )

        err_pos_lfoot = self.pd_lfoot - ptip_lfoot
        err_rot_lfoot = eR(Reye(), Rtip_lfoot)
        err_pos_rfoot = self.pd_rfoot - ptip_rfoot
        err_rot_rfoot = eR(Reye(), Rtip_rfoot)


        # Right hand movement (outward in x direction)
        x_rhand = self.X_PELVIS + 0.2 * sin(self.t)  # Outward motion in x
        v_x_rhand = 0.2 * cos(self.t)  # Velocity of right hand in x

        p_rhand = pxyz(x_rhand, 0.0, self.Z_PELVIS_TOP)
        vd_rhand = np.array([v_x_rhand, 0.0, 0.0])
        wd_rhand = np.zeros(3)

        Td_rhand = T_from_Rp(Reye(), p_rhand)

        # Update the desired position
        self.pd_rhand = [i[3] for i in Td_rhand[0:3]]

        # Forward kinematics for right hand
        ptip_rhand, Rtip_rhand, Jv_rhand, Jw_rhand = self.chain_rhand.fkin(
            [self.q[i] for i in JOINT_ORDERS['r_hand']]
        )

        # Right hand errors
        err_pos_rhand = self.pd_rhand - ptip_rhand
        err_rot_rhand = eR(Reye(), Rtip_rhand)


        # Combined error for legs and right hand
        err = np.concatenate((err_pos_lfoot, err_rot_lfoot, err_pos_rfoot, err_rot_rfoot, err_pos_rhand, err_rot_rhand))

        # Combined Jacobian
        J = np.vstack([
            np.hstack([Jv_lfoot, np.zeros_like(Jv_rfoot), np.zeros_like(Jv_rhand)]),
            np.hstack([Jw_lfoot, np.zeros_like(Jw_rfoot), np.zeros_like(Jw_rhand)]),
            np.hstack([np.zeros_like(Jv_lfoot), Jv_rfoot, np.zeros_like(Jv_rhand)]),
            np.hstack([np.zeros_like(Jw_lfoot), Jw_rfoot, np.zeros_like(Jw_rhand)]),
            np.hstack([np.zeros_like(Jv_lfoot), np.zeros_like(Jv_rfoot), Jv_rhand]),
            np.hstack([np.zeros_like(Jw_lfoot), np.zeros_like(Jw_rfoot), Jw_rhand])
        ])

        weight_mat = self.gamma**2 * np.eye(J.shape[1])
        Jwinv = np.linalg.pinv(J.T @ J + weight_mat) @ J.T

        q_nominal_lfoot = np.array([0.0, 0.0, 0.0, (pi / 2), 0.0, 0.0])
        q_nominal_rfoot = np.array([0.0, 0.0, 0.0, (pi / 2), 0.0, 0.0])
        q_nominal_rhand = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        q_nominal = np.concatenate((q_nominal_lfoot, q_nominal_rfoot, q_nominal_rhand))
        qsdot = -self.K_s * (self.q[np.r_[JOINT_ORDERS['l_leg'], JOINT_ORDERS['r_leg'], JOINT_ORDERS['r_hand']]] - q_nominal)

        # Update joint velocities and positions
        qdot = Jwinv @ (np.concatenate((vd_lfoot, wd_lfoot, vd_rfoot, wd_rfoot, vd_rhand, wd_rhand)) + self.K_p * err) + \
                                (np.eye(J.shape[1]) - Jwinv @ J) @ qsdot

        for i, mapped_i in enumerate(np.r_[JOINT_ORDERS['l_leg'], JOINT_ORDERS['r_leg'], JOINT_ORDERS['r_hand']]):
            self.q[mapped_i] += qdot[i] * self.dt

        cmdmsg = JointState()
        cmdmsg.header.stamp = self.now().to_msg()
        cmdmsg.name = JOINT_NAMES
        cmdmsg.position = self.q.tolist()
        cmdmsg.velocity = self.qdot.tolist()
        self.pub.publish(cmdmsg)


def main(args=None):
    rclpy.init(args=args)
    node = ThrowNode('throw', 100)
    rclpy.spin(node)
    node.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
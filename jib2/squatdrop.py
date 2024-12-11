'''squatdrop.py

   Node:      /squatdrop
   Publish:   /joint_states             sensor_msgs.msg.JointState
   Broadcast: 'pelvis' w.r.t. 'world'   geometry_msgs.msg.TransformStamped

'''

import rclpy
import numpy as np

from math import pi, sin, cos, acos, atan2, sqrt, fmod, exp

from rclpy.node                 import Node
from rclpy.qos                  import QoSProfile, DurabilityPolicy
from rclpy.time                 import Duration
from tf2_ros                    import TransformBroadcaster, Buffer, TransformListener, LookupException
from geometry_msgs.msg          import TransformStamped
from sensor_msgs.msg            import JointState
from std_msgs.msg               import ColorRGBA
from visualization_msgs.msg     import Marker, MarkerArray

from hw5code.TransformHelpers     import *
from hw6code.KinematicChain       import KinematicChain

import jib2.constants as c


class DemoNode(Node):
    def __init__(self, name, rate):
        super().__init__(name)

        self.broadcaster = TransformBroadcaster(self)
        self.pub_atlas = self.create_publisher(JointState, '/joint_states', 10)
        self.get_logger().info('Waiting for a /joint_states subscriber...')
        while not self.count_subscribers('/joint_states'):
            pass

        quality = QoSProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)
        self.pub_ball = self.create_publisher(MarkerArray, '/visualization_marker_array', quality)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.dt = 1.0 / float(rate)
        self.t = -self.dt
        self.start = self.get_clock().now() + Duration(seconds=self.dt)

        self.chain_lfoot = KinematicChain(self, 'pelvis', 'l_foot', c.joint_builder(c.JOINT_NAMES, 'l_leg'))
        self.chain_rfoot = KinematicChain(self, 'pelvis', 'r_foot', c.joint_builder(c.JOINT_NAMES, 'r_leg'))
        self.chain_head = KinematicChain(self, 'pelvis', 'head', c.joint_builder(c.JOINT_NAMES, 'head'))
        self.chain_lhand = KinematicChain(self, 'pelvis', 'l_hand', c.joint_builder(c.JOINT_NAMES, 'l_arm'))
        self.chain_rhand = KinematicChain(self, 'pelvis', 'r_hand', c.joint_builder(c.JOINT_NAMES, 'r_arm'))

        self.q = np.zeros(len(c.JOINT_NAMES))
        self.qdot = np.zeros(len(c.JOINT_NAMES))

        p_lfoot_pelvis_pelvis, _, _, _ = self.chain_lfoot.fkin(c.joint_builder(self.q, 'l_leg'))
        p_rfoot_pelvis_pelvis, _, _, _ = self.chain_rfoot.fkin(c.joint_builder(self.q, 'r_leg'))
        p_rhand_pelvis_pelvis, _, _, _ = self.chain_rhand.fkin(c.joint_builder(self.q, 'r_arm'))

        self.p_lfoot_pelvis_world = p_lfoot_pelvis_pelvis + pxyz(c.X_PELVIS, c.Y_PELVIS, c.Z_PELVIS_TOP)
        self.p_rfoot_pelvis_world = p_rfoot_pelvis_pelvis + pxyz(c.X_PELVIS, c.Y_PELVIS, c.Z_PELVIS_TOP)
        self.p_rhand_pelvis_world = p_rhand_pelvis_pelvis + pxyz(c.X_PELVIS, c.Y_PELVIS, c.Z_PELVIS_TOP)

        self.Td_lfoot_pelvis_world = T_from_Rp(Reye(), self.p_lfoot_pelvis_world)
        self.Td_rfoot_pelvis_world = T_from_Rp(Reye(), self.p_rfoot_pelvis_world)
        self.Td_rhand_pelvis_world = T_from_Rp(Reye(), self.p_rhand_pelvis_world)

        self.K_p = 50.0
        self.K_s = 10.0
        self.gamma = 0.1

        self.radius_ball = 0.1
        self.p_ball_world_world = np.array([0.0, 0.0, self.radius_ball])
        self.v_ball_world_world = np.array([0.0, 0.0, 0.0])
        self.a_ball_world_world = np.array([0.0, 0.0, -c.GRAVITY])

        ball_diameter = 2 * self.radius_ball
        self.ball_marker = Marker()
        self.ball_marker.header.frame_id = 'world'
        self.ball_marker.header.stamp = self.get_clock().now().to_msg()
        self.ball_marker.action = Marker.ADD
        self.ball_marker.ns = 'ball'
        self.ball_marker.id = 1
        self.ball_marker.type = Marker.SPHERE
        self.ball_marker.pose.orientation = Quaternion()
        self.ball_marker.pose.position = Point_from_p(self.p_ball_world_world)
        self.ball_marker.scale = Vector3(x=ball_diameter, y=ball_diameter, z=ball_diameter)
        self.ball_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.8)

        self.marker_array = MarkerArray(markers=[self.ball_marker])
        self.ball_in_hand = True

        self.create_timer(self.dt, self.update)
        self.get_logger().info(f'Running with dt of {self.dt} seconds ({rate}Hz)')


    def shutdown(self):
        self.destroy_node()


    def now(self):
        return self.start + Duration(seconds=self.t)
    

    def update_ball(self):
        if self.t > 4.0 and self.ball_in_hand:
            self.v_ball_world_world = self.v_rhand_world_world
            self.ball_in_hand = False

        p_rhand_world_world = self.p_rhand_pelvis_pelvis + self.p_pelvis_world_world

        if self.ball_in_hand:
            if p_rhand_world_world is not None:
                self.p_ball_world_world = p_rhand_world_world 
            else:
                print('Hand position not found')
        else:
            a_drag_ball = 0.05 * np.linalg.norm(self.v_ball_world_world) * self.v_ball_world_world
            self.a_ball_world_world = np.array([0.0, 0.0, -c.GRAVITY]) - a_drag_ball
            
            self.v_ball_world_world += self.dt * self.a_ball_world_world
            self.p_ball_world_world += self.dt * self.v_ball_world_world

            if self.p_ball_world_world[2] < self.radius_ball:
                self.p_ball_world_world[2] = self.radius_ball + (self.radius_ball - self.p_ball_world_world[2])
                self.v_ball_world_world[2] *= -1.0
                self.v_ball_world_world[0] *= 0.0


    def update(self):
        self.t += self.dt

        z_pelvis_world = c.Z_PELVIS_MID + c.Z_PELVIS_AMP * cos(c.Z_PELVIS_PER_PI * pi * self.t)
        self.p_pelvis_world_world = pxyz(c.X_PELVIS, c.Y_PELVIS, z_pelvis_world)
        R_pelvis_world_world = Reye()
        T_pelvis_world_world = T_from_Rp(R_pelvis_world_world, self.p_pelvis_world_world)
        v_pelvis_world_world = np.array([0.0, 0.0, -c.Z_PELVIS_AMP * c.Z_PELVIS_PER_PI * pi * sin(c.Z_PELVIS_PER_PI * pi * self.t)])

        y_rhand_world = c.THROW_Y_RHAND_MID + c.THROW_Y_RHAND_AMP * sin(c.THROW_Y_RHAND_PER_PI * pi * (self.t + c.THROW_Y_RHAND_SHIFT_PI * pi))
        v_y_rhand_world = c.THROW_Y_RHAND_AMP * c.THROW_Y_RHAND_PER_PI * pi * cos(c.THROW_Y_RHAND_PER_PI * pi * (self.t + c.THROW_Y_RHAND_SHIFT_PI * pi))
        p_rhand_world_world = pxyz(c.THROW_X_RHAND, y_rhand_world, c.THROW_Z_RHAND)
        R_rhand_world_world = Reye()
        Td_rhand_world_world = T_from_Rp(R_rhand_world_world, p_rhand_world_world)

        self.pd_rhand_world_world = [i[3] for i in Td_rhand_world_world[0:3]]


        trans = TransformStamped()
        trans.header.stamp = self.now().to_msg()
        trans.header.frame_id = 'world'
        trans.child_frame_id = 'pelvis'
        trans.transform = Transform_from_T(T_pelvis_world_world)
        self.broadcaster.sendTransform(trans)

        Td_lfoot_pelvis_pelvis = np.linalg.inv(T_pelvis_world_world) @ self.Td_lfoot_pelvis_world
        Td_rfoot_pelvis_pelvis = np.linalg.inv(T_pelvis_world_world) @ self.Td_rfoot_pelvis_world

        self.pd_lfoot_pelvis_world = [i[3] for i in Td_lfoot_pelvis_pelvis[0:3]]
        self.pd_rfoot_pelvis_world = [i[3] for i in Td_rfoot_pelvis_pelvis[0:3]]

        vd_lfoot = vd_rfoot = -v_pelvis_world_world
        wd_lfoot = wd_rfoot = np.zeros(3)

        vd_rhand_world_world = np.array([0.0, v_y_rhand_world, 0.0])
        self.v_rhand_world_world = vd_rhand_world_world
        wd_rhand_world_world = np.zeros(3)

        (p_lfoot_pelvis_pelvis, Rtip_lfoot_pelvis_pelvis, 
         Jv_lfoot_pelvis_pelvis, Jw_lfoot_pelvis_pelvis) = self.chain_lfoot.fkin(
             c.joint_builder(self.q, 'l_leg'))
        (p_rfoot_pelvis_pelvis, Rtip_rfoot_pelvis_pelvis, 
         Jv_rfoot_pelvis_pelvis, Jw_rfoot_pelvis_pelvis) = self.chain_rfoot.fkin(
             c.joint_builder(self.q, 'r_leg'))
        (p_rhand_pelvis_pelvis, R_rhand_pelvis_pelvis, 
         Jv_rhand_pelvis_pelvis, Jw_rhand_pelvis_pelvis) = self.chain_rhand.fkin(
             c.joint_builder(self.q, 'r_arm'))
        
        self.p_rhand_pelvis_pelvis = p_rhand_pelvis_pelvis

        err_pos_lfoot = self.pd_lfoot_pelvis_world - p_lfoot_pelvis_pelvis
        err_rot_lfoot = eR(Reye(), Rtip_lfoot_pelvis_pelvis)
        err_pos_rfoot = self.pd_rfoot_pelvis_world - p_rfoot_pelvis_pelvis
        err_rot_rfoot = eR(Reye(), Rtip_rfoot_pelvis_pelvis)
        err_pos_rhand = self.pd_rhand_world_world - p_rhand_pelvis_pelvis
        err_rot_rhand = eR(Reye(), R_rhand_pelvis_pelvis)

        err = np.concatenate((
            err_pos_lfoot, err_rot_lfoot, 
            err_pos_rfoot, err_rot_rfoot, 
            err_pos_rhand, err_rot_rhand
        ))

        J = np.vstack([
            np.hstack([Jv_lfoot_pelvis_pelvis, np.zeros_like(Jv_rfoot_pelvis_pelvis), np.zeros_like(Jv_rhand_pelvis_pelvis)]),
            np.hstack([Jw_lfoot_pelvis_pelvis, np.zeros_like(Jw_rfoot_pelvis_pelvis), np.zeros_like(Jw_rhand_pelvis_pelvis)]),
            np.hstack([np.zeros_like(Jv_lfoot_pelvis_pelvis), Jv_rfoot_pelvis_pelvis, np.zeros_like(Jv_rhand_pelvis_pelvis)]),
            np.hstack([np.zeros_like(Jw_lfoot_pelvis_pelvis), Jw_rfoot_pelvis_pelvis, np.zeros_like(Jw_rhand_pelvis_pelvis)]),
            np.hstack([np.zeros_like(Jv_lfoot_pelvis_pelvis), np.zeros_like(Jv_rfoot_pelvis_pelvis), Jv_rhand_pelvis_pelvis]),
            np.hstack([np.zeros_like(Jw_lfoot_pelvis_pelvis), np.zeros_like(Jw_rfoot_pelvis_pelvis), Jw_rhand_pelvis_pelvis])
        ])

        weight_mat = self.gamma**2 * np.eye(J.shape[1])
        Jwinv = np.linalg.pinv(J.T @ J + weight_mat) @ J.T

        q_nominal_lfoot = np.array([0.0, 0.0, 0.0, (3 * pi / 4), 0.0, 0.0])
        q_nominal_rfoot = np.array([0.0, 0.0, 0.0, (3 * pi / 4), 0.0, 0.0])
        q_nominal_rhand = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, (pi / 2), 0.0, 0.0, 0.0])
        q_nominal = np.concatenate((q_nominal_lfoot, q_nominal_rfoot, q_nominal_rhand))
        qsdot = -self.K_s * (self.q[np.r_[c.JOINT_ORDERS['l_leg'], c.JOINT_ORDERS['r_leg'], c.JOINT_ORDERS['r_arm']]] - q_nominal)

        qdot = Jwinv @ (np.concatenate((vd_lfoot, wd_lfoot, vd_rfoot, wd_rfoot, vd_rhand_world_world, wd_rhand_world_world)) + self.K_p * err) + \
            (np.eye(J.shape[1]) - Jwinv @ J) @ qsdot

        for i, mapped_i in enumerate(np.r_[c.JOINT_ORDERS['l_leg'], c.JOINT_ORDERS['r_leg'], c.JOINT_ORDERS['r_arm']]):
            self.q[mapped_i] += qdot[i] * self.dt


        self.update_ball()
        

        cmdmsg = JointState()
        cmdmsg.header.stamp = self.now().to_msg()
        cmdmsg.name = c.JOINT_NAMES
        cmdmsg.position = self.q.tolist()
        cmdmsg.velocity = self.qdot.tolist()
        self.pub_atlas.publish(cmdmsg)

        self.ball_marker.header.stamp  = self.now().to_msg()
        self.ball_marker.pose.position = Point_from_p(self.p_ball_world_world)
        self.pub_ball.publish(self.marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = DemoNode('squatdrop', 100)
    rclpy.spin(node)
    node.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
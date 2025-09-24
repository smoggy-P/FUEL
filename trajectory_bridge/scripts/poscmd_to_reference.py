#!/usr/bin/env python3
# coding: utf-8
"""
poscmd_to_reference.py
订阅:  /planning/pos_cmd (quadrotor_msgs/PositionCommand)
发布:  /kingsfisher/agiros_pilot/trajectory (agiros_msgs/Reference)

将 PositionCommand 打包为单点 Reference(Setpoint)。
可调参数:
  ~mass                [kg]     default: 1.5
  ~gravity             [m/s^2]  default: 9.81
  ~thrust_scale                 default: 1.0   # 乘在合力模长上；若飞控用归一化推力可设为 1/f_max
  ~thrust_min                   default: 0.0
  ~thrust_max                   default: 100.0
  ~is_single_rotor_thrust       default: False
  ~output_frame                 default: auto  # Reference/Setpoint/Command 的 header.frame_id
  ~queue_size                   default: 50
"""

import math
import rospy
from std_msgs.msg import Header, Empty
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3
from quadrotor_msgs.msg import PositionCommand
from agiros_msgs.msg import Reference, Setpoint, QuadState, Command as AgiCommand

def yaw_to_quat(yaw):
    """绕 Z 轴的四元数"""
    half = 0.5 * yaw
    return Quaternion(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))

class PosCmdToReference:
    def __init__(self):
        # params
        self.mass  = float(rospy.get_param("~mass", 1.5))
        self.g     = float(rospy.get_param("~gravity", 9.81))
        self.k     = float(rospy.get_param("~thrust_scale", 1.0))
        self.u_min = float(rospy.get_param("~thrust_min", 0.0))
        self.u_max = float(rospy.get_param("~thrust_max", 100.0))
        self.single_rotor = bool(rospy.get_param("~is_single_rotor_thrust", False))
        self.frame = rospy.get_param("~output_frame", "")
        qsize = int(rospy.get_param("~queue_size", 50))

        self.pub = rospy.Publisher("/kingfisher/agiros_pilot/reference", QuadState, queue_size=qsize)
        self.sub = rospy.Subscriber("/planning/pos_cmd", PositionCommand, self.cb, queue_size=qsize)

        rospy.loginfo("[poscmd_to_reference] ready. m=%.3f, k=%.3f, range=[%.2f, %.2f], single_rotor=%s",
                      self.mass, self.k, self.u_min, self.u_max, str(self.single_rotor))

    def cb(self, msg: PositionCommand):
        px, py, pz = float(msg.position.x), float(msg.position.y), float(msg.position.z)
        vx, vy, vz = float(msg.velocity.x), float(msg.velocity.y), float(msg.velocity.z)
        ax, ay, az = float(msg.acceleration.x), float(msg.acceleration.y), float(msg.acceleration.z)
        yaw        = float(msg.yaw)
        yaw_dot    = float(msg.yaw_dot)

        import numpy as np
        import tf.transformations as tf_trans
        from geometry_msgs.msg import Quaternion

        g = 9.81
        acc = np.array([ax, ay, az]) + np.array([0, 0, g])
        z_b = acc / np.linalg.norm(acc)

        x_c = np.array([np.cos(yaw), np.sin(yaw), 0.0])
        y_b = np.cross(z_b, x_c)
        y_b /= np.linalg.norm(y_b)
        x_b = np.cross(y_b, z_b)

        R = np.vstack([x_b, y_b, z_b]).T
        q_arr = tf_trans.quaternion_from_matrix(np.vstack([
            np.hstack([R, np.array([[0],[0],[0]])]),
            np.array([0,0,0,1])
        ]))
        q = Quaternion(x=q_arr[0], y=q_arr[1], z=q_arr[2], w=q_arr[3])

        state = QuadState()
        state.header = Header(stamp=msg.header.stamp)
        state.header.frame_id = self.frame if self.frame else (msg.header.frame_id or "world")
        state.t = 0.0

        state.pose = Pose(
            position=Point(x=px, y=py, z=pz),
            orientation=q
        )
        state.velocity = Twist(
            linear=Vector3(x=vx, y=vy, z=vz),
            angular=Vector3(x=0.0, y=0.0, z=yaw_dot)
        )
        state.acceleration = Twist(
            linear=Vector3(x=ax, y=ay, z=az),
            angular=Vector3(x=0.0, y=0.0, z=0.0)
        )

        self.pub.publish(state)


def main():
    rospy.init_node("poscmd_to_reference", anonymous=False)
    PosCmdToReference()
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

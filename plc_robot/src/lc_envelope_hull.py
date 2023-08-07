#!/usr/bin/python3

import sys
import rospy
import utils
import numpy as np
import lc_wrapper_py
import tf
import quaternion


class PlacementStrategies:
    def __init__(self):
      listener = tf.TransformListener()
      lc_id = rospy.get_namespace().replace("/", "")
      self.device_name = lc_id
      listener.waitForTransform("architect/base_link", lc_id + "_ir_optical", rospy.Time(), rospy.Duration(4))
      (lc_trans, lc_orient) = listener.lookupTransform("architect/base_link", lc_id + "_ir_optical", rospy.Time(0))
      self.lc_orient = quaternion.as_rotation_matrix(
        np.quaternion(lc_orient[3], lc_orient[0], lc_orient[1], lc_orient[2]))
      self.lc_trans = np.array([lc_trans[0], lc_trans[1], lc_trans[2]])
      self.rate = None
      self.random = rospy.get_param(rospy.get_namespace() + 'lc_envelope/randCurtains')

    def run_system(self):
      lc_wrapper_py.ros_init("lc_wrapper_instance")
      lc = lc_wrapper_py.LCWrapper(sim=False,
                                   rand_curtains=self.random,
                                   device_name=self.device_name,
                                   cam_id=3,
                                   laser_power=40)
      rate = lc_wrapper_py.Rate(30)
      thetas = lc.getThetas()

      unit_vec = np.array([0, 1])
      thetas = thetas[::-1]
      rays = []
      for theta in thetas:
        ray = np.array([[np.cos(np.pi * theta / 180), -np.sin(np.pi * theta / 180)],
                        [np.sin(np.pi * theta / 180), np.cos(np.pi * theta / 180)]]) @ unit_vec
        rays.append(ray/np.linalg.norm(ray))
      rays = np.array(rays)

      while lc.ok():
        panda1_joint_positions = rospy.get_param('/robot/joint_positions')
        joint_positions = {
          "p1": panda1_joint_positions
        }
        design_pts = utils.get_closest_pts(joint_positions, self.lc_orient, self.lc_trans, rays)
        lc.sendCurtain(pts=design_pts)
        rate.sleep()


def main(args):
  rospy.init_node("lc_envelope_joints")
  placement_strategies = PlacementStrategies()
  placement_strategies.run_system()


if __name__ == '__main__':
    main(sys.argv)

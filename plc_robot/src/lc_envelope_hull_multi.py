#!/usr/bin/python3

import sys
import rospy
import utils
import numpy as np
import lc_wrapper_py
import tf
import quaternion
import time


class PlacementStrategies:
    def __init__(self,
                 use_sim,
                 min_range,
                 max_range):
      self.use_sim = use_sim
      self.min_range = min_range
      self.max_range = max_range
      self.lc_id = rospy.get_namespace().split('/')[1]

      listener = tf.TransformListener()
      listener.waitForTransform("architect/base_link", f"{self.lc_id}_ir_optical", rospy.Time(), rospy.Duration(15))
      (lc_trans, lc_orient) = listener.lookupTransform("architect/base_link", f"{self.lc_id}_ir_optical", rospy.Time(0))
      self.lc_archi_orient = quaternion.as_rotation_matrix(
        np.quaternion(lc_orient[3], lc_orient[0], lc_orient[1], lc_orient[2]))
      self.lc_archi_trans = np.array([lc_trans[0], lc_trans[1], lc_trans[2]])
      
      listener.waitForTransform("developer/base_link", f"{self.lc_id}_ir_optical", rospy.Time(), rospy.Duration(15))
      (lc_trans, lc_orient) = listener.lookupTransform("developer/base_link", f"{self.lc_id}_ir_optical", rospy.Time(0))
      self.lc_devel_orient = quaternion.as_rotation_matrix(
        np.quaternion(lc_orient[3], lc_orient[0], lc_orient[1], lc_orient[2]))
      self.lc_devel_trans = np.array([lc_trans[0], lc_trans[1], lc_trans[2]])
      self.rate = None

    def run_system(self):
      lc_wrapper_py.ros_init("lc_wrapper_instance")
      lc = lc_wrapper_py.LCWrapper(sim=False,
                                   rand_curtains=False,
                                   device_name=self.lc_id,
                                   cam_id=3,
                                   laser_power=40)
      rate = lc_wrapper_py.Rate(60)
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
        robot1_joint_positions = rospy.get_param('/architect/joint_positions')
        robot2_joint_positions = rospy.get_param('/developer/joint_positions')
        joint_positions = {
          "p1": robot1_joint_positions,
          "p2": robot2_joint_positions}
        #start = time.time()
        design_pts = utils.get_closest_pts(joint_positions, self.lc_archi_orient, self.lc_archi_trans,
          self.lc_devel_orient, self.lc_devel_trans, rays)
        #elapsed = time.time() - start
        #print(f"design takes: {elapsed:.6f}")
        #start = time.time()
        lc.sendCurtain(pts=design_pts)
        #curtains = lc.getCurtains()[0]
        #elapsed = time.time() - start
        #print(f"imaging takes: {elapsed:.6f}")
        rate.sleep()


def main(args):
  rospy.init_node("lc_envelope_joints")
  placement_strategies = PlacementStrategies(use_sim=True,
                                             min_range=1,
                                             max_range=7)
  placement_strategies.run_system()


if __name__ == '__main__':
    main(sys.argv)

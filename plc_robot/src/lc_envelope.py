#!/usr/bin/python3

import rospy
import lc_wrapper_py
import lc_planner_py
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from plc_panda_control.msg import StampedArray
from cv_bridge import CvBridge
import numpy as np
import utils
import time

class Envelope:
  def __init__(self):
    lc_wrapper_py.ros_init("lc_wrapper_instance")
    self.lc = lc_wrapper_py.LCWrapper(sim=True, rand_curtains=False, laser_power=50)
    self.thetas = self.lc.getThetas()
    self.vertical_range = [0.35, 2]
    self.max_range = 10
    fx = 423.4358
    fy = 423.4358
    cx = 251.5836
    cy = 325.38754
    self.K = np.array([[fx, 0, cx],
                      [0, fy, cy],
                      [0, 0, 1]])

    self.heuristic_planner = lc_planner_py.PlannerHeuristicGreedy(1.5)
    rospy.Subscriber("rgbd_camera/depth/image_slow", Image, self.depth_callback)
    self.pub = rospy.Publisher('design_pts', StampedArray, queue_size=10)

  def back_project(self, depth_img):
    depth_img = np.nan_to_num(depth_img, nan=self.max_range)
    H, W = depth_img.shape[:2]

    y, x = np.mgrid[0:H, 0:W] # x and y are both (H, W)
    z = depth_img[:, :] # (H, W)
    x, y = x * z, y * z  # (H, W)
    xyz = np.stack([x, y, z], axis=-1)  # (H, W, 3)
    xyz = xyz @ np.linalg.inv(self.K).T  # (H, W, 3)
    return xyz
  
  def depth_callback(self, message):
    bridge = CvBridge()
    depth_img = bridge.imgmsg_to_cv2(message, desired_encoding='passthrough')
    
    cam_xyz = self.back_project(depth_img)
    cam_x, cam_y, cam_z = cam_xyz[:, :, 0], cam_xyz[:, :, 1], cam_xyz[:, :, 2]  # all are (H, C)
    bev_range = np.sqrt(np.square(cam_x) + np.square(cam_z))  # (H, C)  sqrt(x**2 + z**2)
    bev_range -= 0.4

    bev_range = bev_range.clip(max=self.max_range)  # (H, C)

    vrange_min, vrange_max = self.vertical_range
    outside_vrange_mask = (cam_y > vrange_min) | (-cam_y > vrange_max)  # (H, C)
    bev_range[outside_vrange_mask] = self.max_range

    se_ranges = bev_range.min(axis=0).astype(np.float32)  # (C,)
    design_pts = utils.ranges_to_design_filter(se_ranges, self.thetas)

    self.lc.sendCurtain(pts=design_pts)


if __name__ == '__main__':
  rospy.init_node("lc_envelope")
  envelope = Envelope()
  rospy.spin()
   

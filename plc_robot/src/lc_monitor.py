#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

import cv2
from cv_bridge import CvBridge
import numpy as np
import time

bridge = CvBridge()
disabled = False
startup = True
ref_img = None
mask = None

roi = np.array([[220, 290], [350, 230], [430, 330], [290, 390]])
roi_pub = rospy.Publisher("/redversion/roi_detection", Image, queue_size=1)

def enable_robot_service(ns):
    try:
        enable_robot = rospy.ServiceProxy(f'/{ns}/robot_enable', Trigger)
        resp = enable_robot()
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

def disable_robot_service(ns):
    try:
        disable_robot = rospy.ServiceProxy(f'/{ns}/robot_disable', Trigger)
        resp = disable_robot()
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

def roi_img_publish(roi):
    global roi_pub
    roi_pub.publish(bridge.cv2_to_imgmsg(roi))

def ir_img_callback(msg):
    #start = time.time()
    global disabled, startup, ref_img, mask, roi
    ir_img = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    if startup:
        ref_img = ir_img
        startup = False
        mask = np.zeros_like(ir_img)
        cv2.fillPoly(mask, [roi], 255)
        return

    diff_img = cv2.absdiff(ref_img, ir_img)
    roi_diff = cv2.bitwise_and(diff_img, mask)
    roi_img_publish(roi_diff)
    if(np.max(roi_diff) > 120):
        rospy.logwarn('CURTAIN INTRUSION!')
        if not disabled:
            #rstart = time.time()
            disable_robot_service("architect")
            #relapsed = time.time() - rstart
            #print(f"Robot stop takes: {relapsed:.6f}")
            disable_robot_service("developer")
            disabled = True
    else:
        if disabled:
            enable_robot_service("architect")
            enable_robot_service("developer")
            disabled = False

    #elapsed = time.time() - start
    #print(f"Detection takes: {elapsed:.6f}")

if __name__ == '__main__':
    rospy.init_node('lc_detector')
    rospy.Subscriber('/redversion/ir_image/all', Image, ir_img_callback)
    time.sleep(3)
    rospy.spin()

#!/usr/bin/env python3

import math

import rospy
import tf
from sensor_msgs.msg import JointState
import numpy as np
import quaternion


class Direct_Kinematics():

    def __init__(self):
        rospy.init_node("arm_positions")
        rospy.Subscriber("franka_state_controller/joint_states_slow", JointState, self.jointStateCallback)
        arm_id = rospy.get_param("arm_positions/arm_id")
        
        listener = tf.TransformListener()
        listener.waitForTransform("world", f"{arm_id}_link0", rospy.Time(), rospy.Duration(4))
        (arm_trans, arm_orient) = listener.lookupTransform("world", f"{arm_id}_link0", rospy.Time(0))
        self.arm_orient = quaternion.as_rotation_matrix(
            np.quaternion(arm_orient[3], arm_orient[0], arm_orient[1], arm_orient[2]))
        self.arm_trans = np.array([arm_trans[0], arm_trans[1], arm_trans[2]])
     
    def dh_params(self,joint_variable):
        joint_var = joint_variable
        M_PI = math.pi

        # Create DH parameters (data given by maker franka-emika) (alpha, a, d, theta)
        self.dh = np.array([[ 0,        0,        0.333,   joint_var[0]],
                            [-M_PI/2,   0,        0,       joint_var[1]],
                            [ M_PI/2,   0,        0.316,   joint_var[2]],
                            [ M_PI/2,   0.0825,   0,       joint_var[3]],
                            [-M_PI/2,  -0.0825,   0.384,   joint_var[4]],
                            [ M_PI/2,   0,        0,       joint_var[5]],
                            [ M_PI/2,   0.088,    0,       joint_var[6]],
                            [ 0,        0,        0.107,   0]])
        
        return self.dh
      
    def TF_matrix(self,i,dh):
        # Define Transformation matrix based on DH params
        alpha = dh[i, 0]
        a = dh[i, 1]
        d = dh[i, 2]
        q = dh[i, 3]
        
        TF = np.array([[np.cos(q),-np.sin(q), 0, a],
                       [np.sin(q)*np.cos(alpha), np.cos(q)*np.cos(alpha), -np.sin(alpha), -np.sin(alpha)*d],
                       [np.sin(q)*np.sin(alpha), np.cos(q)*np.sin(alpha),  np.cos(alpha),  np.cos(alpha)*d],
                       [0,  0,  0,  1]])
        return TF

    def jointStateCallback(self,message):
        rate = rospy.Rate(10)
        self.joint_var = []
        for i in range(0,7):
            self.joint_var.append((message.position[i]))

        dh_parameters = self.dh_params(self.joint_var)

        joint_positions = []
        T = np.eye(4)
        for i in range(8):
            T = T @ self.TF_matrix(i, dh_parameters)
            position = tf.transformations.translation_from_matrix(T)
            position = self.arm_orient @ position + self.arm_trans
            if i > 2:
                joint_positions.append([float(position[0]), float(position[1]), float(position[2])])
                joint_positions.append([float(position[0]) + 0.2, float(position[1]), float(position[2])])
                joint_positions.append([float(position[0]) - 0.2, float(position[1]), float(position[2])])
                joint_positions.append([float(position[0]), float(position[1]), float(position[2]) + 0.2])
                joint_positions.append([float(position[0]), float(position[1]), float(position[2]) - 0.2])
                joint_positions.append([float(position[0]), float(position[1]) + 0.2, float(position[2])])
                joint_positions.append([float(position[0]), float(position[1]) - 0.2, float(position[2])])
                joint_positions.append([float(position[0]) + 0.2, float(position[1]), float(position[2]) + 0.2])
                joint_positions.append([float(position[0]) - 0.2, float(position[1]), float(position[2]) + 0.2])
                joint_positions.append([float(position[0]) + 0.2, float(position[1]), float(position[2]) - 0.2])
                joint_positions.append([float(position[0]) - 0.2, float(position[1]), float(position[2]) - 0.2])
            else:
                joint_positions.append([float(position[0]), float(position[1]), float(position[2])])
                joint_positions.append([float(position[0]) + 0.4, float(position[1]), float(position[2])])
                joint_positions.append([float(position[0]) - 0.4, float(position[1]), float(position[2])])
                joint_positions.append([float(position[0]), float(position[1]), float(position[2]) + 0.4])
                joint_positions.append([float(position[0]), float(position[1]), float(position[2]) - 0.4])
                joint_positions.append([float(position[0]) + 0.4, float(position[1]), float(position[2]) + 0.4])
                joint_positions.append([float(position[0]) - 0.4, float(position[1]), float(position[2]) + 0.4])
                joint_positions.append([float(position[0]) + 0.4, float(position[1]), float(position[2]) - 0.4])
                joint_positions.append([float(position[0]) - 0.4, float(position[1]), float(position[2]) - 0.4])
        
        rospy.set_param('end_effect_position', [float(position[0]),
            float(position[1]), float(position[2])])
        rospy.set_param('joint_positions', joint_positions)


if __name__ == '__main__':
    Direct_Kinematics()
    rospy.spin()

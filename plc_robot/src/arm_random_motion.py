#! /usr/bin/env python3

import rospy
import moveit_commander
import geometry_msgs.msg

import sys

def send_command(move_group, goal):
    move_group.set_pose_target(goal)
    move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()


def main():
    rospy.init_node("arm_random_motion", anonymous=True)
    arm_id = rospy.get_param("arm_random_motion/arm_id")

    moveit_commander.roscpp_initialize(sys.argv)
    arm_group = moveit_commander.MoveGroupCommander(name=f"{arm_id}_arm",
        robot_description=f"{arm_id}/robot_description")

    reset_goal = geometry_msgs.msg.Pose()
    reset_goal.orientation.w = 1.0
    reset_goal.position.x = 0.0
    reset_goal.position.y = 0.0
    reset_goal.position.z = 0.0

    while not rospy.is_shutdown():
        pose_goal = arm_group.get_random_pose()
        pose_goal.pose.orientation.w = 1.0
        pose_goal.pose.orientation.x = 0.4
        pose_goal.pose.orientation.y = 0.1
        pose_goal.pose.orientation.z = 0.4
        send_command(arm_group, pose_goal)
    


if __name__ == '__main__':
    main()

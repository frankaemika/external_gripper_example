#!/usr/bin/env python

import actionlib_msgs.msg
import sensor_msgs.msg
import rospy
import moveit_commander

if __name__ == '__main__':
    rospy.init_node('demo')

    rospy.loginfo('Waiting for move_group/status')
    rospy.wait_for_message('move_group/status',
                           actionlib_msgs.msg.GoalStatusArray)

    commander = moveit_commander.MoveGroupCommander('panda_arm')

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        pose = commander.get_current_pose('panda_link8').pose
        print('pos: [{}, {}, {}]'.format(pose.position.x, pose.position.y, pose.position.z))
        print('rot: [{}, {}, {}, {}]'.format(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z))
        msg = rospy.wait_for_message('franka_gripper/joint_states',
                                     sensor_msgs.msg.JointState)
        print('width: {}'.format(sum(msg.position)))
        print('--')
        rate.sleep()

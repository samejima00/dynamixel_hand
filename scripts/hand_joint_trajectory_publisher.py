#!/usr/bin/env python

import sys
import rospy

from control_msgs.msg import FollowJointTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectoryPoint

class DynamixelHandJointTrajectoryPublisher():
    def __init__(self, handname):
        rospy.init_node(handname + '_hand_joint_trajectory_publisher', anonymous=True)
        rospy.Rate(100)
        self.subscriber = rospy.Subscriber('/dynamixel_' + handname + '_hand_controller/follow_joint_trajectory_action/goal', FollowJointTrajectoryActionGoal, self.callback, queue_size=2)
        self.joint_command_pub = rospy.Publisher('/dynamixel_' + handname + '_hand_controller/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, queue_size=2)

    def callback(self, msg):
        pub_msg = FollowJointTrajectoryActionGoal()
        pub_msg.header = msg.header
        pub_msg.goal_id = msg.goal_id
        pub_msg.goal.trajectory.header = msg.goal.trajectory.header
        pub_msg.goal.trajectory.joint_names = ['r_thumb_roll', 'r_thumb_pitch', 'r_middle_pitch']
        thumb_r_id = msg.goal.trajectory.joint_names.index('THUMB_R')
        thumb_p_id = msg.goal.trajectory.joint_names.index('THUMB_P')
        middle_p_id = msg.goal.trajectory.joint_names.index('MIDDLE_P')

        pub_msg.goal.trajectory.points = []
        for p in msg.goal.trajectory.points:
            point = JointTrajectoryPoint()
            point.positions = [p.positions[thumb_r_id], p.positions[thumb_p_id], p.positions[middle_p_id]]
            point.velocities = [p.velocities[thumb_r_id], p.velocities[thumb_p_id], p.velocities[middle_p_id]]
            point.time_from_start = p.time_from_start
            pub_msg.goal.trajectory.points.append(point)

        self.joint_command_pub.publish(pub_msg)

if __name__ == '__main__':
    args = sys.argv
    try:
        handname = 'rarm'
        if (len(args) > 1):
            if args[1] == 'rarm' or args[1] == 'larm':
                handname = args[1]
            else:
                print("DynamixelHandJointTrajectoryPublisher: Invalid hand name")
        s = DynamixelHandJointTrajectoryPublisher(handname)
        rospy.spin()
    except rospy.ROSInterruptException: pass


#!/usr/bin/env python

import rospy
from control_msgs.msg import FollowJointTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class DynamixelJointTrajectoryPublisher():
    def __init__(self):
        rospy.init_node('hand_joint_trajectory_subscriber', anonymous=True)
        rospy.Rate(100)
        self.subscriber = rospy.Subscriber('/fullbody_controller/follow_joint_trajectory_action/goal', FollowJointTrajectoryActionGoal, self.callback, queue_size=2)
        self.joint_command_pub = rospy.Publisher('/dynamixel_hand_controller/joint_trajectory', JointTrajectory, queue_size=2)

    def callback(self, msg):
        pub_msg = JointTrajectory()
        pub_msg.header = msg.header
        pub_msg.joint_names = ['rhand_thumb_roll', 'rhand_thumb_pitch', 'rhand_middle_pitch']
        rhand_thumb_roll_id = msg.goal.trajectory.joint_names.index('R_THUMB_JOINT0')
        rhand_thumb_pitch_id = msg.goal.trajectory.joint_names.index('R_THUMB_JOINT1')
        rhand_middle_pitch_id = msg.goal.trajectory.joint_names.index('R_MIDDLE_JOINT0')
        pub_msg.points = []
        for p in msg.goal.trajectory.points:
            point = JointTrajectoryPoint()
            point.positions = [p.positions[head_pitch_id], p.positions[head_yaw_id]]#TOFIX
            point.velocities = [p.velocities[head_pitch_id], p.velocities[head_yaw_id]]
            point.time_from_start = p.time_from_start
            pub_msg.points.append(point)
        self.joint_command_pub.publish(pub_msg)

if __name__ == '__main__':
    try:
        s = DynamixelJointTrajectoryPublisher()
        rospy.spin()
    except rospy.ROSInterruptException: pass


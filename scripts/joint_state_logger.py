#!/usr/bin/env python

import sys
import rospy
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState

class JointStateLogger():
    def __init__(self):
        rospy.init_node('joint_state_logger', anonymous=True)
        rospy.Rate(100)
        self.subscriber = rospy.Subscriber('/dynamixel_workbench/joint_states', JointState, self.callback, queue_size=2)
        self.datafile = open('log.txt', mode='w')
        self.start_time = rospy.get_rostime()

    def callback(self, msg):
        d = msg.header.stamp - self.start_time
        data = str(d.to_sec()) + " "
        for p in msg.position:
            data = data + str(p) + " "
        data = data + '\n'
        self.datafile.write(data)

if __name__ == '__main__':
    try:
        logger = JointStateLogger()
        rospy.spin()
    except rospy.ROSInterruptException:
        logger.datafile.close()
        


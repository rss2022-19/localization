#!/usr/bin/env python2

from time import time
import numpy as np

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from math import pi
# from visualization_tools import *

class SimpleDrive:
    DRIVE_TOPIC = "/drive"
    
    def __init__(self):

        # initialize publisher
        self.pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)
        rate = rospy.Rate(10)

        ack_msg = AckermannDriveStamped()
        ack_msg.drive.speed = 1
        # ack_msg.drive.steering_angle_velocity = 100
        ack_msg.drive.steering_angle = pi/40

        # self.pub.publish(ack_msg)

        while not rospy.is_shutdown():
            self.pub.publish(ack_msg)
            rate.sleep()

        # # publish msg to drive topic
        # ack_msg = AckermannDriveStamped()
        # ack_msg.drive.speed = 1
        # self.pub.publish(ack_msg)

if __name__ == "__main__":
    try:
        rospy.init_node('simple_drive')
        simple_drive = SimpleDrive()
    except rospy.ROSInterruptException:
        pass

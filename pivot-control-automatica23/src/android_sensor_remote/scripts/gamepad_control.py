#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

import numpy as np


class RemoteControl:
    def __init__(self):
        self.max_vel = 0.4 # 0.6 default, at 0.4 there are no oscialations anymore
        self.pub = rospy.Publisher("/pre_teleop_vel", Twist, queue_size=1)
        self.sensor_subscriber = rospy.Subscriber("/joy", Joy, self.cb)
    
    def cb(self, msg):
        twist_msg = Twist()
        twist_msg.linear.x = np.clip(-msg.axes[1], -self.max_vel, self.max_vel)
        twist_msg.linear.y = np.clip(-msg.axes[0], -self.max_vel, self.max_vel)
        twist_msg.linear.z = np.clip(msg.axes[4], -self.max_vel, self.max_vel)
        self.pub.publish(twist_msg)

if __name__ == '__main__':
    rospy.init_node('remote_control')
    RemoteControl()
    rospy.spin()
    

#!/usr/bin/env python
# Author: Rafael Cabral
# Date: 23 Oct. 2022

import rospy
from sensor_msgs.msg import Imu, Illuminance
from geometry_msgs.msg import Twist

import numpy as np
from dqrobotics import DQ
from dqrobotics import *

class RemoteControl:
    def __init__(self):

        self.max_angle = 60*np.pi/180
        self.max_vel = 0.6

        self.init_orientation = None
        self.orientation = None

        self.pub = rospy.Publisher("/pre_teleop_vel", Twist, queue_size=1)
        self.sensor_subscriber = rospy.Subscriber("/android/imu", Imu, self.callback_sensor)
    
    def callback_sensor(self, msg):
        if self.init_orientation is None:
            self.init_orientation = DQ([msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z])
        self.orientation = DQ([msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z])

        # rotation unit quaternion
        r = self.init_orientation.inv()*self.orientation

        # normalize
        r = r.normalize()
        
        # axis angle representation
        axis = r.rotation_axis()
        angle = r.rotation_angle()
        axis_angle = axis*angle
        axis_angle = np.array(axis_angle.vec4()[1:])

        # clip to max_angle
        axis_angle = np.clip(axis_angle, -self.max_angle, self.max_angle)

        # map to max_vel
        vel = axis_angle/self.max_angle * self.max_vel

        twist_msg = Twist()
        twist_msg.linear.x = vel[0]
        twist_msg.linear.y = vel[1]
        twist_msg.linear.z = vel[2]
        
        self.pub.publish(twist_msg)

if __name__ == '__main__':
    rospy.init_node('remote_control')
    RemoteControl()
    rospy.spin()
    

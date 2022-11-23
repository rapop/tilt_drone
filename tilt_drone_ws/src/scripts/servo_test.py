#!/usr/bin/env python

import rospy
import tf
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from math import pi, atan
import numpy as np

pub_front_axle = rospy.Publisher('/hummingbird/front_axle_joint_position_controller/command', Float64, queue_size=1)

rate_hz = 15

def generate():
   while True:
     yield -0.2618
     yield 0.7854

def rosnode():
    rospy.init_node('servo_test', anonymous=True)
    rate = rospy.Rate(rate_hz)
    generator = generate()

    while not rospy.is_shutdown():
        #angle = np.random.uniform(-0.7854,0.7854)
        angle = generator.next()
        set_angle(angle)
        rate.sleep()

def set_angle(angle):
    pub_front_axle.publish(Float64(angle))

if __name__ == '__main__':
    try:
        rosnode()
    except rospy.ROSInterruptException:
        pass

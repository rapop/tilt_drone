#!/usr/bin/env python

import rospy
import tf
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from math import pi, atan
import numpy as np

pub_r = rospy.Publisher('/hummingbird/gimbal_r_joint_position_controller/command', Float64, queue_size=1)
pub_p = rospy.Publisher('/hummingbird/gimbal_p_joint_position_controller/command', Float64, queue_size=1)
pub_y = rospy.Publisher('/hummingbird/gimbal_y_joint_position_controller/command', Float64, queue_size=1)
pub_front_axle = rospy.Publisher('/hummingbird/front_axle_joint_position_controller/command', Float64, queue_size=1)
pub_back_axle = rospy.Publisher('/hummingbird/back_axle_joint_position_controller/command', Float64, queue_size=1)

def set_angle(roll, pitch, yaw):
    pub_r.publish(Float64(roll))
    pub_p.publish(Float64(pitch))
    pub_y.publish(Float64(yaw))
    # pub_front_axle.publish(Float64(0))
    # pub_back_axle.publish(Float64(0))

def odo_cb(msg):
    odo = msg
    x = odo.pose.pose.position.x
    y = odo.pose.pose.position.y
    z = odo.pose.pose.position.z
    qx = odo.pose.pose.orientation.x
    qy = odo.pose.pose.orientation.y
    qz = odo.pose.pose.orientation.z
    qw = odo.pose.pose.orientation.w
    roll, pitch, yaw = tf.transformations.euler_from_quaternion((qx, qy, qz, qw))

    # x_target = 4
    # x_change = 2.5
    # y_target = 0.8
    # z_target = 1.2
    # alpha = 0.5
    #
    # yaw_ref = atan((y-y_target)/(x-x_target))
    # roll_ref = 0
    # pitch_ref = - atan((z-z_target)/(x-x_target)) + alpha* (x-x_change)

    x_target = 3.95
    x_change = 2.5#1.7
    y_target = 0.65
    z_target = 1.2
    alpha = 0.2

    yaw_ref = atan((y-y_target)/(x-x_target))
    roll_ref = 0
    pitch_ref = - atan((z-z_target)/(x-x_target)) + alpha* (x-x_change)
    gz = yaw_ref - yaw
    gz = np.clip(gz,-pi/10,pi/10)
    gy = pitch_ref - pitch
    gy = np.clip(gy,-pi/4,pi/4)

        # x_target = 0.75
        # x_change = -1.5
        # y_target = 0
        # z_target = 1.2
        # alpha = 0.2
        # yaw_ref = atan((y-y_target)/(xf - x_target))
        # pitch_ref = -atan((z-z_target)/(xf - x_target)) + alpha* (xf-x_change)
        # gz = yaw_ref - psi
        # gz = np.clip(gz,-pi/10,pi/10)
        # gy = theta - pitch_ref
        # # print(pitch_ref, theta)
        # gy = np.clip(gy,-pi/4,pi/4)

    set_angle(roll_ref - roll, gy, gz)


def gimbal_command():

    rospy.init_node('gimbal_command', anonymous=True)
    # odometry subscribe
    rospy.Subscriber("/hummingbird/ground_truth/odometry", Odometry, odo_cb)
    rospy.spin()

if __name__ == "__main__":
    try:
        gimbal_command()
    except rospy.ROSInterruptException:
        pass

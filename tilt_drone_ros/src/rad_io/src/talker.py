#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from mavros_msgs.msg import ActuatorControl

def talker():
    pub = rospy.Publisher("/mavros/actuator_control", ActuatorControl, queue_size=1)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        actuator_control_message = ActuatorControl()
        actuator_control_message.header.stamp = rospy.Time.now()
        actuator_control_message.group_mix = 2 # ActuatorControl.PX4_MIX_PAYLOAD
        actuator_control_message.controls[6] = 1
        pub.publish(actuator_control_message)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

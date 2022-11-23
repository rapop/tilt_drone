import rospy
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Twist, PoseStamped, Transform, Quaternion, Point, Pose
import std_msgs.msg
from nav_msgs.msg import Odometry
import tf
import math
import time
import numpy as np
import matplotlib.pyplot as plt
from math import cos, sin, pi
from gazebo_msgs.msg import ModelStates

rate_hz = 100

def rosnode():
    rospy.init_node('tiltquad_traj2', anonymous=True)
    rate = rospy.Rate(rate_hz)

    trajObj = TrajectoryHelper()
    time.sleep(1)
    trajObj.send_reference()
    print('Trajectory sent')

    # Node starting
    start = time.time()
    while not rospy.is_shutdown():
        elapsed = time.time() - start
        if (elapsed > 2):
            rospy.signal_shutdown("Simulation End")
        rate.sleep()

class TrajectoryHelper(object):
    def __init__(self):


        ''' Defining trajectory point '''
        traj_time=0.8
        phi_des=0
        theta_des=0
        psi_des=0
        x_des=1.5
        y_des=0.65
        z_des=1.2
        dx_des=2
        dy_des=0
        dz_des=0
        ddx_des=0
        ddy_des=0
        ddz_des=0
        ''' ------------------------- '''

        # create msg object
        quaternion = tf.transformations.quaternion_from_euler(phi_des, theta_des, psi_des)
        transforms = Transform(translation=Point(x_des, y_des, z_des), rotation=Quaternion(quaternion[0],quaternion[1],quaternion[2],quaternion[3]))
        velocities = Twist()
        velocities.linear.x = dx_des
        velocities.linear.y = dy_des
        velocities.linear.z = dz_des
        accelerations = Twist()
        accelerations.linear.x = ddx_des
        accelerations.linear.y = ddy_des
        accelerations.linear.z = ddz_des
        self.traj_point = MultiDOFJointTrajectoryPoint([transforms],[velocities],[accelerations], rospy.Time(traj_time))

        self.pub_traj_point = rospy.Publisher('/hummingbird/mav_ilqr_control/trajectory', MultiDOFJointTrajectoryPoint, queue_size=1)

    def send_reference(self):

        self.pub_traj_point.publish(self.traj_point)


if __name__ == '__main__':
    try:
        rosnode()
    except rospy.ROSInterruptException:
        pass

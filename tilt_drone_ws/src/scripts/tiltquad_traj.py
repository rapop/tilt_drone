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
from traj_gen import TrajectoryGenerator

rate_hz = 100

def rosnode():
    rospy.init_node('tiltquad_traj', anonymous=True)
    rate = rospy.Rate(rate_hz)

    trajObj = TrajectoryHelper()
    time.sleep(1)
    trajObj.send_reference()
    print('Trajectory sent')

    # Node starting
    start = time.time()
    while not rospy.is_shutdown():
        elapsed = time.time() - start
        trajObj.get_realpos(elapsed)
        if (elapsed > trajObj.traj_time):
            trajObj.plotting()
            rospy.signal_shutdown("Simulation End")
        rate.sleep()

class TrajectoryHelper(object):
    def __init__(self):

        # append for graph
        self.t_list = []
        self.xdes_list = []
        self.ydes_list = []
        self.zdes_list = []
        self.dxdes_list = []
        self.dydes_list = []
        self.dzdes_list = []

        self.traj_time=2
        self.x_init=0.5
        self.y_init=0.65
        self.z_init=1

        ''' Defining trajectory point '''
        self.impact_time = 0.83
        self.phi_des = 0
        self.theta_des = 0
        self.psi_des = 0
        self.x_des = 1.5 + 0.2
        self.y_des = 0.4
        self.z_des = 1
        self.dx_des = 2
        self.dy_des = 0
        self.dz_des = 0
        self.ddx_des = 0
        self.ddy_des = 0
        self.ddz_des = 0
        ''' ------------------------- '''

        # create msg object
        quaternion = tf.transformations.quaternion_from_euler(self.phi_des, self.theta_des, self.psi_des)
        transforms = Transform(translation=Point(self.x_des, self.y_des, self.z_des), rotation=Quaternion(quaternion[0],quaternion[1],quaternion[2],quaternion[3]))
        velocities = Twist()
        velocities.linear.x = self.dx_des
        velocities.linear.y = self.dy_des
        velocities.linear.z = self.dz_des
        accelerations = Twist()
        accelerations.linear.x = self.ddx_des
        accelerations.linear.y = self.ddy_des
        accelerations.linear.z = self.ddz_des
        self.traj_point = MultiDOFJointTrajectoryPoint([transforms],[velocities],[accelerations],rospy.Time(self.impact_time))

        self.pub_traj_point = rospy.Publisher('/hummingbird/mav_ilqr_control/trajectory', MultiDOFJointTrajectoryPoint, queue_size=1)

        # odometry subscribe
        self.odo = Odometry()
        self.odo_sub = rospy.Subscriber("/hummingbird/ground_truth/odometry", Odometry, self.odo_cb, tcp_nodelay=True)

        # init for plotting
        self.time_nav=np.zeros((1,1))
        self.nav_pos = np.zeros((1,3))
        self.nav_vel = np.zeros((1,3))
        self.nav_ang = np.zeros((1,3))
        self.nav_pos_plot = np.zeros((1,3))
        self.nav_vel_plot = np.zeros((1,3))
        self.nav_ang_plot = np.zeros((1,3))

        self.create_traj_full_class()

    def send_reference(self):
        self.pub_traj_point.publish(self.traj_point)

    def odo_cb(self,msg):
        self.odo = msg
        #self.odo_stamp = rospy.Time.now()
        x = self.odo.pose.pose.position.x
        y = self.odo.pose.pose.position.y
        z = self.odo.pose.pose.position.z
        qx = self.odo.pose.pose.orientation.x
        qy = self.odo.pose.pose.orientation.y
        qz = self.odo.pose.pose.orientation.z
        qw = self.odo.pose.pose.orientation.w
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((qx, qy, qz, qw))
        vx = self.odo.twist.twist.linear.x
        vy = self.odo.twist.twist.linear.y
        vz = self.odo.twist.twist.linear.z
        self.nav_pos=np.array([[x, y, z]])
        self.nav_vel=np.array([[vx, vy, vz]])
        self.nav_ang=np.array([[roll, pitch, yaw]])

    def get_realpos(self,t):
        self.time_nav=np.append(self.time_nav,t) #Construction du vecteur de temps
        self.nav_pos_plot=np.append(self.nav_pos_plot,self.nav_pos, axis=0)
        self.nav_vel_plot=np.append(self.nav_vel_plot,self.nav_vel, axis=0)
        self.nav_ang_plot=np.append(self.nav_ang_plot,self.nav_ang, axis=0)

    def create_traj_full_class(self):

        t = [0,self.impact_time,self.traj_time]
        wps = 2
        dt = 0.01
        constraints = np.empty([5,wps+1,4])
        # x  constraints
        constraints[:,:,0] = np.array([[self.x_init, self.x_des, self.x_init],
                                [0, self.dx_des, 0], # velocity constraints
                                [0, self.ddx_des, 0], # acceleration constraints
                                [0, None, 0], # jerk constraints
                                [0, None, 0]]) # snap constraints
        # y constraints
        constraints[:,:,1] = np.array([[self.y_init, self.y_des, self.y_init],
                                [0, self.dy_des, 0],
                                [0, self.ddy_des, 0],
                                [0, None, 0],
                                [0, None, 0]])
        # z constraints
        constraints[:,:,2] = np.array([[self.z_init, self.z_des, self.z_init],
                                [0, self.dz_des, 0],
                                [0, self.ddz_des, 0],
                                [0, None, 0],
                                [0, None, 0]])
        # yaw constraints
        constraints[:,:,3] = np.array([[0, self.psi_des, 0],
                                [0, 0, 0],
                                [0, 0, 0],
                                [0, None, 0],
                                [0, None, 0]])

        Traj = TrajectoryGenerator(wps, t, dt, constraints)
        traj_np_array = Traj.return_array()

        for data in traj_np_array:
            t, x_des, dx_des, ddx_des, dddx_des, ddddx_des, y_des, dy_des, ddy_des, dddy_des, ddddy_des, z_des, dz_des, ddz_des, dddz_des, ddddz_des, psi_des, dpsi_des,ddpsi_des, dddpsi_des, ddddpsi_des = data

            # append for graph
            self.t_list.append(t)
            self.xdes_list.append(x_des)
            self.ydes_list.append(y_des)
            self.zdes_list.append(z_des)
            self.dxdes_list.append(dx_des)
            self.dydes_list.append(dy_des)
            self.dzdes_list.append(dz_des)

    def plotting(self):

        # position graph
        plt.figure()
        plt.subplot(3, 1, 1)
        #print(np.shape(self.time_nav),np.shape(self.nav_pos_plot[:,0]),np.shape(self.xdes_list))
        plt.plot(self.time_nav,self.nav_pos_plot[:,0],'-',self.t_list,self.xdes_list,'--')
        plt.title('Position')
        plt.ylabel('x (m)')
        plt.xlabel('t (s)')
        plt.gca().legend(('Actual','Desired'))
        plt.subplot(3, 1, 2)
        plt.plot(self.time_nav,self.nav_pos_plot[:,1],'-',self.t_list,self.ydes_list,'--')
        plt.ylabel('y (m)')
        plt.xlabel('t (s)')
        plt.subplot(3, 1, 3)
        plt.plot(self.time_nav,self.nav_pos_plot[:,2],'-',self.t_list,self.zdes_list,'--')
        plt.ylabel('z (m)')
        plt.xlabel('t (s)')
        #plt.show()

        # velocity graph
        plt.figure()
        plt.subplot(3, 1, 1)
        plt.plot(self.time_nav,self.nav_vel_plot[:,0],'-',self.t_list,self.dxdes_list,'--')
        plt.title('Velocity')
        plt.ylabel('dx (m/s)')
        plt.xlabel('t (s)')
        plt.gca().legend(('Actual','Desired'))
        plt.subplot(3, 1, 2)
        plt.plot(self.time_nav,self.nav_vel_plot[:,1],'-',self.t_list,self.dydes_list,'--')
        plt.ylabel('dy (m/s)')
        plt.xlabel('t (s)')
        plt.subplot(3, 1, 3)
        plt.plot(self.time_nav,self.nav_vel_plot[:,2],'-',self.t_list,self.dzdes_list,'--')
        plt.ylabel('dz (m/s)')
        plt.xlabel('t (s)')

        # angles graph
        plt.figure()
        plt.subplot(3, 1, 1)
        plt.plot(self.time_nav,self.nav_ang_plot[:,0])
        plt.title('Angles')
        plt.ylabel('roll (rad)')
        plt.xlabel('t (s)')
        plt.gca().legend('Actual')
        plt.subplot(3, 1, 2)
        plt.plot(self.time_nav,self.nav_ang_plot[:,1])
        plt.ylabel('pitch (rad)')
        plt.xlabel('t (s)')
        plt.subplot(3, 1, 3)
        plt.plot(self.time_nav,self.nav_ang_plot[:,2])
        plt.ylabel('yaw (rad)')
        plt.xlabel('t (s)')
        plt.show()


if __name__ == '__main__':
    try:
        rosnode()
    except rospy.ROSInterruptException:
        pass

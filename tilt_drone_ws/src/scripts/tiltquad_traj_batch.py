import rospy
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Twist, PoseStamped, Transform, Quaternion, Point, Pose
import std_msgs.msg
from nav_msgs.msg import Odometry
import tf
import math
import csv
import time
import numpy as np
import matplotlib.pyplot as plt
from math import cos, sin, pi
from gazebo_msgs.msg import ModelStates
from traj_gen import TrajectoryGenerator
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import ball_helper

rate_hz = 100
np.set_printoptions(formatter={'float': '{: 0.3f}'.format})

def rosnode():
    rospy.init_node('tiltquad_traj', anonymous=True)
    rate = rospy.Rate(rate_hz)

    traj_time=2 #time for the full traj
    batch_size=10
    nb_good_strikes=0
    nb_good_services=0
    data_array = np.empty((1,13))

    while not rospy.is_shutdown():
        time.sleep(1)
        for i in xrange(batch_size):
            #Ball spawn conditions
            # x, y, z = [np.random.uniform(4.2,4.8),np.random.uniform(0.3,1.1),np.random.uniform(0.8,1.4)]
            x = 4.2
            y = np.random.uniform(-0.5,0.5) + 0.65
            z = 1.2
            vx, vy, vz = [np.random.uniform(-2.2,-3.2),0,np.random.uniform(2.2,2.8)]
            print('----------------------------------------------------------------------')
            print('Hit :', i+1,' Initial ball position : ',round(x,2),round(y,2),round(z,2),' Initial ball speed : ',round(vx,2),round(vy,2),round(vz,2))
            trajObj = TrajectoryHelper(x,y,z,vx,vy,vz,traj_time,rate_hz)
            time.sleep(5)
            trajObj.send_reference()
            print('Trajectory sent')

            # spawn_ball
            ball_helper.spawn_ball(x,y,z,vx,vy,vz)

            # Node starting
            start = time.time()
            while True:
                elapsed = time.time() - start
                trajObj.get_realpos(elapsed)
                if (elapsed > traj_time):
                    # ball_helper.delete_ball()
                    #trajObj.plotting()
                    break;
                        #rospy.signal_shutdown("Simulation End")
                rate.sleep()

            #Get information about the quality of the try
            contacts, impact_pt_info, outcome = trajObj.send_contact_info()
            print('*Ball contact points*')
            print(contacts)
            print('*Impact waypoint*')
            print(impact_pt_info)
            print('*Outcome*')
            print(outcome)
            if outcome[0]=='Ball correctly received.': #to dont penalize if the return was impossible
                nb_good_services+=1
                if outcome[1]=='Ball correctly striked.':
                    nb_good_strikes+=1
                    data_array=np.around(np.append(data_array,np.append(np.array([[x,y,z,vx,vy,vz]]),impact_pt_info,axis=1),axis=0),2)
            print('Currently: ',nb_good_strikes,'/',nb_good_services,' % of balls returned. ')
            print('----------------------------------------------------------------------')
            if i==batch_size-1:
                print('****** Results: ',nb_good_strikes,'/',nb_good_services,' % of balls returned. ******')
                # PATH = "dataset_17jun.csv"
                # with open(PATH, 'a') as writeFile:
                #     writer = csv.writer(writeFile)
                #     #header = ['x', 'y', 'z', 'vx', 'vy', 'vz', 't', 'xf', 'zf', 'vxf','vzf','ax','az']
                #     #writer.writerow(header)
                #     for i in xrange(np.size(data_array,0)-1):
                #         writer.writerow(data_array[i+1,:].tolist()) #+1 to skeep first line with zeros
                #
                # writeFile.close()
                rospy.signal_shutdown("Simulation End") #shut down node at the end of the batch

class TrajectoryHelper(object):
    def __init__(self,x,y,z,vx,vy,vz,traj_time,rate_hz):

        # append for graph
        self.t_list = []
        self.xdes_list = []
        self.ydes_list = []
        self.zdes_list = []
        self.dxdes_list = []
        self.dydes_list = []
        self.dzdes_list = []

        self.traj_time=traj_time
        self.x_init=0.5
        self.y_init=0.65
        self.z_init=1
        self.dt=1.0/rate_hz

        timef, xf, yf, zf, _, _, _  = ball_helper.get_impact_info(x,y,z,vx,vy,vz)

        ''' Defining trajectory point '''
        self.impact_time = timef
        self.phi_des = 0
        self.theta_des = 0
        self.psi_des = 0
        self.x_des = xf + 0.2
        self.y_des = yf
        self.z_des = zf
        self.dx_des = 2
        self.dy_des = 0
        self.dz_des = 0
        self.ddx_des = 0
        self.ddy_des = 0
        self.ddz_des = 0
        ''' ------------------------- '''
        self.impact_pt_info=np.array([[timef,xf,zf,self.dx_des,self.dz_des,self.ddx_des,self.ddz_des]])

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

        # gazebo tilt joint states subscribe
        self.tiltStates = JointState()
        self.tiltStates_cb = rospy.Subscriber("/hummingbird/joint_states", JointState, self.tiltStates_cb, tcp_nodelay=True)

        # commanded tilt back joint subscribe
        self.tiltCmdBack = Float64()
        self.tiltCmdBack_cb = rospy.Subscriber("/hummingbird/back_axle_joint_position_controller/command", Float64, self.tiltCmdBack_cb, tcp_nodelay=True)
        # commanded tilt front joint subscribe
        self.tiltCmdFront = Float64()
        self.tiltCmdFront_cb = rospy.Subscriber("/hummingbird/front_axle_joint_position_controller/command", Float64, self.tiltCmdFront_cb, tcp_nodelay=True)

        # init for plotting
        self.time_nav=np.zeros((1,1))
        self.nav_pos = np.zeros((1,3))
        self.nav_vel = np.zeros((1,3))
        self.nav_ang = np.zeros((1,3))
        self.nav_pos_plot = np.zeros((1,3))
        self.nav_vel_plot = np.zeros((1,3))
        self.nav_ang_plot = np.zeros((1,3))

        self.tilt_back_angle=np.zeros((1,1))
        self.tilt_back_angle_plot=np.zeros((1,1))
        self.tilt_front_angle=np.zeros((1,1))
        self.tilt_front_angle_plot=np.zeros((1,1))

        self.tilt_back_cmd=np.zeros((1,1))
        self.tilt_back_cmd_plot=np.zeros((1,1))
        self.tilt_front_cmd=np.zeros((1,1))
        self.tilt_front_cmd_plot=np.zeros((1,1))

        self.create_traj_full_class() #only needed to be able to plot the ref

        #Table information
        self.table_height=0.755-0.01 #soustract min_depth of the ball (the ball can sink in the table)

        self.xtableminopp=1.78
        self.xtablemaxopp=2.88
        self.ytableminopp=-0.03
        self.ytablemaxopp=1.38

        self.xtableminours=3.15
        self.xtablemaxours=4.45
        self.ytableminours=-0.06
        self.ytablemaxours=1.43

        #Ball
        self.nav_ball = np.zeros((1,3))
        self.contacts_balltable = np.zeros((1,4)) #store contacts with the table
        self.lastcontacttime=0
        # ball subscribe
        self.ball_pos = ModelStates()
        self.ball_pos_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.ball_pos_cb, tcp_nodelay=True)

    def send_reference(self):
        self.pub_traj_point.publish(self.traj_point)

    def odo_cb(self,msg):
        self.odo = msg

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

    def tiltStates_cb(self,msg):
        self.tiltStates = msg

        try:
            back_tilt_angle = self.tiltStates.position[0]
            front_tilt_angle = self.tiltStates.position[0]
            self.tilt_back_angle=back_tilt_angle
            self.tilt_front_angle=front_tilt_angle
        except:
            void=1

    def tiltCmdBack_cb(self,msg):
        self.tiltCmdBack = msg

        back_tilt_cmd = self.tiltCmdBack.data
        self.tilt_back_cmd=back_tilt_cmd

    def tiltCmdFront_cb(self,msg):
        self.tiltCmdFront = msg

        front_tilt_cmd = self.tiltCmdFront.data
        self.tilt_front_cmd=front_tilt_cmd

    def ball_pos_cb(self,msg):
        self.ball_pos = msg
        try:
            x = self.ball_pos.pose[4].position.x #change index number if models are added to the world
            y = self.ball_pos.pose[4].position.y
            z = self.ball_pos.pose[4].position.z
            self.nav_ball=np.array([[x, y, z]])
        except:
            pass

    def get_realpos(self,t):
        self.time_nav=np.append(self.time_nav,t) #Construction du vecteur de temps
        self.nav_pos_plot=np.append(self.nav_pos_plot,self.nav_pos, axis=0)
        self.nav_vel_plot=np.append(self.nav_vel_plot,self.nav_vel, axis=0)
        self.nav_ang_plot=np.append(self.nav_ang_plot,self.nav_ang, axis=0)
        self.tilt_back_angle_plot=np.append(self.tilt_back_angle_plot,self.tilt_back_angle)
        self.tilt_front_angle_plot=np.append(self.tilt_front_angle_plot,self.tilt_front_angle)
        self.tilt_back_cmd_plot=np.append(self.tilt_back_cmd_plot,self.tilt_back_cmd)
        self.tilt_front_cmd_plot=np.append(self.tilt_front_cmd_plot,self.tilt_front_cmd)

        #Check ball impacts pos
        if abs(round(self.nav_ball[0,2],2)-round(self.table_height,2))<0.15: #adjust tolerance in function of ball speed
            if (t-self.lastcontacttime)>0.2: #verify if two contacts are not too close to each other
                self.contacts_balltable=np.append(self.contacts_balltable,np.append(np.array([[t]]),self.nav_ball,axis=1), axis=0)
                self.lastcontacttime=t

    def create_traj_full_class(self):

        t = [0,self.impact_time,self.traj_time]
        wps = 2
        dt = self.dt
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

    def send_contact_info(self):
        try:
            if self.xtableminopp<self.contacts_balltable[1,1]<self.xtablemaxopp and self.ytableminopp<self.contacts_balltable[1,2]<self.ytablemaxopp: #verify if ball first felt in the opposite field
                outcome=['Ball correctly received.']
            else:
                outcome=['Ball incorrectly received.']

            if np.size(self.contacts_balltable,0)>2:
                if self.xtableminours<self.contacts_balltable[2,1]<self.xtablemaxours and self.ytableminours<self.contacts_balltable[2,2]<self.ytablemaxours: #verify ball lends on the field
                    outcome.append('Ball correctly striked.')
                else:
                    outcome.append('Ball incorrectly striked.')
            else:
                outcome.append('Ball was not hit!')

            return self.contacts_balltable[1:,:], self.impact_pt_info, outcome
        except:
            return self.contacts_balltable, self.impact_pt_info, ['Error','Ball too fast to be detected.']

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

        # tilt angle graph
        plt.figure()
        plt.subplot(2, 1, 1)
        plt.plot(self.time_nav,self.tilt_back_angle_plot,'-',self.time_nav,self.tilt_back_cmd_plot,'--')
        plt.title('Back Tilt Angle')
        plt.ylabel('angle (rad)')
        plt.xlabel('t (s)')
        plt.gca().legend(('Actual','Commanded'))
        plt.subplot(2, 1, 2)
        plt.plot(self.time_nav,self.tilt_front_angle_plot,'-',self.time_nav,self.tilt_front_cmd_plot,'--')
        plt.title('Front Tilt Angle')
        plt.ylabel('angle (rad)')
        plt.xlabel('t (s)')
        plt.gca().legend(('Actual','Commanded'))
        plt.show()

if __name__ == '__main__':
    try:
        rosnode()
    except rospy.ROSInterruptException:
        pass

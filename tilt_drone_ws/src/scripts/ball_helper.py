#!/usr/bin/env python

#Added random hit point before of after maximum bounce

import rospy
from gazebo_msgs.srv import DeleteModel, SpawnModel, SpawnModelRequest, SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import *
import numpy as np

def spawn_ball(x,y,z,vx,vy,vz):
    #rospy.init_node("spawn_ball")
    rospy.wait_for_service("gazebo/spawn_sdf_model")
    rospy.wait_for_service("gazebo/set_model_state")
    try:
        spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
        set_state = rospy.ServiceProxy("gazebo/set_model_state", SetModelState)

        # spawn and init position
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        with open('/home/radu/tilt2_ws/src/rotors_simulator/rotors_gazebo/models/ping_pong_ball/model.sdf', "r") as f:
            model_xml = f.read()
        model_name = 'ping_pong_ball'
        req = SpawnModelRequest()
        req.model_name = model_name
        req.model_xml = model_xml
        # should this be unique so ros_control can use each individually?
        req.robot_namespace = "/foo/pball"
        req.initial_pose = pose
        # resp = spawn_model(req)

        # init velocity
        state_msg = ModelState()
        state_msg.model_name = 'ping_pong_ball'
        state_msg.pose.position.x = x
        state_msg.pose.position.y = y
        state_msg.pose.position.z = z
        state_msg.twist.linear.x = vx
        state_msg.twist.linear.y = vy
        state_msg.twist.linear.z = vz
        resp = set_state(state_msg)

    except rospy.ServiceException, e:
        print("Service call failed: %s",e)

def get_impact_info(x,y,z,vx,vy,vz):

    g=-9.81
    h_collision=0.755
    min_depth=0.01 #la balle rentre de cette dimension dans la table
    z_table=h_collision-min_depth #hauteur de la table pour point dimpact
    K=0.8 #coefficient restitution balle
    pourcentage_from_max_hit=0.8 #on prend 20% de la hauteur qui peut etre differente pour frapper la balle ce qui fait que un temps de frappe different est aussi calculer

    t1=max((-vz+(vz**2-4*(g/2*(z-z_table)))**(0.5))/g,(-vz-(vz**2-4*(g/2*(z-z_table)))**(0.5))/g)
    vz_plus1=-K*(vz+g*t1)
    t2=-vz_plus1/g
    tf=t1+t2
    xf=vx*tf+x
    yf=vy*tf+y
    zf=z_table+vz_plus1*t2+0.5*g*t2**2
    zf_new=np.random.uniform(pourcentage_from_max_hit,1)*(zf-z_table)+z_table
    before_or_after_max=np.random.randint(2, size=1) #either 0 or 1
    t2_vec=[(-vz_plus1+(vz_plus1**2-4*(g/2*(z_table-zf_new)))**(0.5))/g,(-vz_plus1-(vz_plus1**2-4*(g/2*(z_table-zf_new)))**(0.5))/g]
    t2_new=t2_vec[before_or_after_max[0]]
    tf_new=t1+t2_new
    xf_new=vx*tf_new+x
    yf_new=vy*tf_new+y
    vxf=vx
    vyf=vy
    vzf=0
    vzf_new=vz_plus1+g*t2_new

    return [tf_new, xf_new, yf_new, zf_new, vxf, vyf, vzf_new]

def get_ball_pos(x,y,z,vx,vy,vz,t):

    g=-9.81
    h_collision=0.755 #hauteur du modele de collision de la table
    min_depth=0.001 #la balle rentre de cette dimension dans la table
    z_table=h_collision-min_depth #hauteur de la table pour point dimpact
    K=0.8 #coefficient restitution balle

    #si on utilise pas un class pour stocker les valeurs on doit utiliser un vecteur t et faire un boucle for sur la longueur de t
    x_vec=np.empty(len(t))
    y_vec=np.empty(len(t))
    z_vec=np.empty(len(t))
    timpact=0
    for i in xrange(len(t)):
        x_vec[i]=vx*t[i]+x
        y_vec[i]=vy*t[i]+y
        z_vec[i]=z+vz*(t[i]-timpact)+0.5*g*(t[i]-timpact)**2
        if abs(z_vec[i]-z_table)<0.05 and t[i]-timpact>0.3: #changer erreur z si on veut plus precis, erreur sur temps permet de ne pas rentrer 2 fois de suite dans la boucle
            vz=-K*(vz+g*(t[i]-timpact))
            z=z_table
            timpact=t[i]


    return np.transpose(np.vstack((x_vec,y_vec,z_vec)))

def delete_ball():

    rospy.wait_for_service("gazebo/delete_model")

    try:
        delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)

        model_name = 'ping_pong_ball'
        delete_model(model_name)
    except rospy.ServiceException, e:
        print("Service call failed: %s",e)

if __name__ == "__main__":
    x, y, z = [4.2,0.65,1.2]
    vx, vy, vz = [-2.5,0,3]
    spawn_ball(x,y,z,vx,vy,vz)
    # delete_ball()

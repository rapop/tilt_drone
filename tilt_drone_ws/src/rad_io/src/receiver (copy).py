#!/usr/bin/env python

import sys
import rospy
import func
from geometry_msgs.msg import PoseStamped

seqNum = 0
seqNumPrev = -1
iter = 0 #iterations since last sequence number reset
vpXoff = 0
vpYoff = 0
vpZoff = 0
vpXcorr = 0
vpYcorr = 0
vpZcorr = 0
vpOffThresh = 0.5 #threshold (meters) for offsetting vision measurement

def TopicListener():
    
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, UpdatePosition)
    rospy.spin()

def UpdatePosition(lpdata):
    vpX = 0
    vpY = 0
    vpZ = 0
      
    global seqNumPrev,iter,seqNum,vpXoff,vpYoff,vpZoff,vpOffThresh,vpXcorr,vpYcorr,vpZcorr
    
    #read vision pose data from radio module
    receivedData = func.COM_RecvData()
    elem = receivedData.split()
    try:
        seqNum = int(elem[0])
        vpX = float(elem[1])
        vpY = float(elem[2])
        vpZ = float(elem[3])
        #print str(seqNum)+' '+str(vpX)+' '+str(vpY)+' '+str(vpZ)
        #rospy.loginfo( "Received data: "+str(seqNum)+' '+str(vpX)+' '+str(vpY)+' '+str(vpZ) )
    except:
        print str(seqNum)+' '+str(vpX)+' '+str(vpY)+' '+str(vpZ)
        rospy.loginfo("Invalid data received")
        return
    else:
        if (iter>1 and seqNum!=seqNumPrev+1):
            rospy.loginfo("Invalid sequence number")
            iter=0
            return
        seqNumPrev=seqNum
        
    #local pose data from callback function   
    lpX = lpdata.pose.position.x
    lpY = lpdata.pose.position.y
    lpZ = lpdata.pose.position.z
    '''
    #calculate offset of vision pose to local pose and adjust offsets

    #print "local X: ",lpX," vision X: ",vpX
    #print "local Y: ",lpY," vision Y: ",vpY
    #print "local Z: ",lpZ," vision Z: ",vpZ
    vpXcorr = vpX + vpXoff
    vpYcorr = vpY + vpYoff
    vpZcorr = vpZ + vpZoff  
    
    offset=lpX-vpXcorr
    if abs(offset)>vpOffThresh:
        vpXoff = vpXoff + offset
        rospy.loginfo("DEVIATION!!! X is offset by "+str(offset)+" Total X offset is "+str(vpXoff))
    offset=lpY-vpYcorr 
    if abs(offset)>vpOffThresh:
        vpYoff = vpYoff + offset
        rospy.loginfo("DEVIATION!!! Y is offset by "+str(offset)+" Total Y offset is "+str(vpYoff))
    offset=lpZ-vpZcorr
    if abs(offset)>vpOffThresh:
        vpZoff = vpZoff + offset
        rospy.loginfo("DEVIATION!!! Z is offset by "+str(offset)+" Total Z offset is "+str(vpZoff))
    '''
    

    
    #publish 
    poseMsg.header.stamp = rospy.Time.now()
    poseMsg.header.seq = seqNum
    poseMsg.pose.position.x = vpX#corr
    poseMsg.pose.position.y = vpY#corr
    poseMsg.pose.position.z = vpZ#corr
    poseMsg.header.frame_id = 'fcu'
    pub.publish(poseMsg)
    iter+=1
    
#connect to radio module
if func.COM_OpenConnection('/dev/ttyUSB0', 115200)[0]:
    sys.exit("Can not open Port... exiting")

# Set up ROS:
rospy.init_node('receiver_node')
pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=1)
poseMsg = PoseStamped()
rate = rospy.Rate(20)
print "ROS-node is set up"

while not rospy.is_shutdown():  
    TopicListener()
    rate.sleep()

rospy.loginfo("Closing serial connection")
func.COM_CloseConnection()


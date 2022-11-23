#!/usr/bin/env python

import sys
import rospy
import func
from geometry_msgs.msg import PoseStamped



seqNum = 0
seqNumPrev = -1
iter = 0 #iterations since last sequence number reset
poseXprev = 0
poseYprev = 0
poseZprev = 0

def Update():
    poseX = 0
    poseY = 0
    poseZ = 0
    global seqNumPrev,iter,poseXprev,poseYprev,poseZprev
 
    receivedData = func.COM_RecvData()
    elem = receivedData.split()
    try:
        seqNum = int(elem[0])
        poseX = float(elem[1])
        poseY = float(elem[2])
        poseZ = float(elem[3])
        print str(seqNum)+' '+str(poseX)+' '+str(poseY)+' '+str(poseZ)
        #rospy.loginfo( "Received data: "+str(seqNum)+' '+str(poseX)+' '+str(poseY)+' '+str(poseZ) )

    except:
        print str(seqNum)+' '+str(poseX)+' '+str(poseY)+' '+str(poseZ)
        rospy.loginfo("Invalid data received, using previous data")
        return
        poseX,poseY,poseZ,seqNum = poseXprev,poseYprev,poseZprev,seqNumPrev+1
    
    else:
        if (iter>1 and seqNum!=seqNumPrev+1):
            rospy.loginfo("Invalid sequence number, using previous +1")
            iter=0
            #return
            poseX,poseY,poseZ,seqNum = poseXprev,poseYprev,poseZprev,seqNumPrev+1
        seqNumPrev=seqNum
        poseXprev,poseYprev,poseZprev = poseX,poseY,poseZ
     
    poseMsg.header.stamp = rospy.Time.now()
    poseMsg.header.seq = seqNum
    poseMsg.pose.position.x = poseX
    poseMsg.pose.position.y = poseY
    poseMsg.pose.position.z = poseZ
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
print "ROS-node is set up"

while not rospy.is_shutdown():  
    Update()


rospy.loginfo("Closing serial connection")
func.COM_CloseConnection()

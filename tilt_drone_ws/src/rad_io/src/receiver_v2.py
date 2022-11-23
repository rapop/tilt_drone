#!/usr/bin/env python

import sys
import rospy
import func
from geometry_msgs.msg import PointStamped

# Handling options
usage = "usage: rosrun leica_interface %prog [options]"
parser = OptionParser(usage=usage)
parser.set_defaults(port="/dev/ttyUSB1",baudrate=115200, debug=False)
parser.add_option("-p", "--port", action="store", type="string", dest="port", help="specify used port [default: %default]")
parser.add_option("-b", "--baudrate", action="store", type="int", dest="baudrate", help="specify used baudrate [default: %default]")
parser.add_option("-d", "--debug", action="store_true", dest="debug", help="print debug information")
(options, args) = parser.parse_args()

#connect to radio module
if func.COM_OpenConnection(options.port, options.baudrate)[0]:
    sys.exit("Can not open Port... exiting")

# Set up ROS:
rospy.init_node('receiver_node')
#pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=1)
point_pub = rospy.Publisher('/leica/radio/worldposition', PoseStamped, queue_size=1)
point_msg = PoseStamped()
print "ROS-node is set up"

seqNum = 0
seqNumPrev = -1
iter = 0 #iterations since last sequence number reset
x,y,z = 0,0,0


def Update():

    global seqNum,seqNumPrev,iter,poseXprev,poseYprev,poseZprev

    receivedData = func.COM_RecvData()

    elem = receivedData.split()
    try:
        seqNum = int(elem[0])
        poseX = float(elem[1])
        poseY = float(elem[2])
        poseZ = float(elem[3])
        #rospy.loginfo( 'Got data ' + str(seqNum)+' '+str(poseX)+' '+str(poseY)+' '+str(poseZ) )
    except:
        #print "ERROR, skipping"
        return

    else:
        if (iter>0): #we received at least one data package
            if (seqNum==seqNumPrev):
                #rospy.loginfo( ' Already received: ' +str(seqNum)+ ' skipping ' )
                return
            if (seqNum!=seqNumPrev+1):
                #rospy.loginfo( ' Invalid sequence: ' +str(seqNum)+ ' skipping ' )
                iter=0
                return
    seqNumPrev=seqNum
    iter+=1

    #rospy.loginfo( 'Valid data received: ')
    rospy.loginfo( str(seqNum)+' '+str(poseX)+' '+str(poseY)+' '+str(poseZ)+ '')
    return poseX,poseY,poseZ

def Publish(x,y,z):
    poseMsg.header.stamp = rospy.Time.now()
    poseMsg.header.seq = seqNum
    poseMsg.pose.position.x = x
    poseMsg.pose.position.y = y
    poseMsg.pose.position.z = z
    poseMsg.header.frame_id = 'fcu'
    pub.publish(poseMsg)



while not rospy.is_shutdown():
    try:
        x,y,z = Update()
        print 'update xyz=',x,y,z
    except:
        print 'WARNING! Failed to update position!'
    Publish(x,y,z)


rospy.loginfo("Closing serial connection")
func.COM_CloseConnection()

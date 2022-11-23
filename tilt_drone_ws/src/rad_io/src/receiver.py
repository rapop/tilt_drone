#!/usr/bin/env python

import sys
import rospy
import func
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from optparse import OptionParser
from operator import neg

# Handling options
usage = "usage: rosrun leica_interface %prog [options]"
parser = OptionParser(usage=usage)
parser.set_defaults(port="/dev/ttyUSB0",baudrate=115200, topic=None,offset=False,debug=False)
parser.add_option("-p", "--port", action="store", type="string", dest="port", help="specify used port [default: %default]")
parser.add_option("-b", "--baudrate", action="store", type="int", dest="baudrate", help="specify used baudrate [default: %default]")
parser.add_option("-d", "--debug", action="store_true", dest="debug", help="print debug information")
parser.add_option("-t", "--topic", action="store", type="string",dest="topic", help="publish to specified PoseStamped topic")
parser.add_option("-o", "--offset", action="store_true",dest="offset", help="offset received coordinates to zero at start")
(options, args) = parser.parse_args()

#connect to radio module
if func.COM_OpenConnection(options.port, options.baudrate)[0]:
    sys.exit("Can not open Port... exiting")



# Set up ROS:
rospy.init_node('receiver_node')
#pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=1)
point_pub = rospy.Publisher('/leica/radio/worldposition', PointStamped, queue_size=2)
point_msg = PointStamped()
print options.topic
if options.topic != None:
    print 'creating publisher for  ' ,options.topic
    pose_pub = rospy.Publisher(options.topic, PoseStamped, queue_size=2)
    pose_msg = PoseStamped()

print "ROS-node is set up"


seqNum = 0
seqNumPrev = -1
iter = 0 #iterations since last sequence number reset
x,y,z = 0,0,0
rate = rospy.Rate(20) #20hz
pub_rate = rospy.Duration(0.05) #20hz
# rate = rospy.Rate(100) #20hz
# pub_rate = rospy.Duration(0.01) #20hz
loop_count = 1
xoff = 0
yoff = 0
zoff = 0
xprev = 0
yprev = 0
zprev = 0


def Update():
    global seqNum,seqNumPrev,iter
    global loop_count,xoff,yoff,zoff,xprev,yprev,zprev

    receivedData = func.COM_RecvData()

    if options.debug: rospy.loginfo( 'Received data: '+ str(receivedData))

    if receivedData == None:
        rospy.logfatal( 'NO DATA!!!' )
        return

    try:
        elem = receivedData.split()
        # print(elem)
        if len(elem) != 4:
            rospy.logwarn( 'Wrong number of received elements! ' +str(seqNum)+ ' skipping... ' )
            return
        else:
            seqNum = int(elem[0])
            x = float(elem[1])
            y = float(elem[2])
            z = float(elem[3])
    except:
        rospy.logfatal( "ERROR trying to parse data!!!" )
        return

    if (iter>0): #received at least one data package
        if (seqNum==seqNumPrev):
            if options.debug:
                rospy.logwarn( 'Already received: ' +str(seqNum)+ ' skipping... ' )
            return
        if (seqNum!=seqNumPrev+1):
            if options.debug:
                rospy.logwarn( ' Invalid sequence: ' +str(seqNum)+ ' skipping... ' )
            iter=0
            return
        if abs(x-xprev)>0.3 or abs(x-xprev)>0.3 or abs(x-xprev)>0.3:
            if options.debug: rospy.logwarn( 'x: '+str(x)+'xprev: '+str(xprev)+'y: '+str(y)+'yprev: '+str(yprev)+'z: '+str(z)+'zprev: '+str(zprev))
            rospy.logwarn( ' Value too different: '+str(seqNum)+' '+str(x)+' '+str(y)+' '+str(z)+ ' skipping...')
            return

    seqNumPrev=seqNum
    iter+=1
    xprev,yprev,zprev = x,y,z

    if options.offset:
        if loop_count < 10:
            xoff =  x #calculate average value
            yoff =  y
            zoff =  z
            loop_count +=1
            if options.debug: rospy.loginfo( 'Offset coords:'+str(seqNum)+' '+str(y + (yoff*-1))+' '+str(x + (xoff*-1))+' '+str(-z - (zoff*-1))+ '')
    #
    PublishPoint(x, y, z, seqNum)
    rospy.loginfo('published xyz = '+str(seqNum)+' '+str(x)+' '+str(y)+' '+str(z))

#

def PublishPoint(x,y,z,seqNum):

    point_msg.header.seq = seqNum
    point_msg.header.stamp = rospy.Time.now()
    point_msg.header.frame_id = 'fcu'
    point_msg.point.x = x
    point_msg.point.y = y
    point_msg.point.z = z
    point_pub.publish(point_msg)
#

def PublishPose():
    global xoff,yoff,zoff
    global x,y,z,seqNum
    pose_msg.header.seq = seqNum
    pose_msg.header.stamp = rospy.Time.now()
    pose_msg.header.frame_id = 'fcu'
    pose_msg.pose.position.x = y + (yoff*-1)
    pose_msg.pose.position.y = x + (xoff*-1)
    pose_msg.pose.position.z = -z - (zoff*-1)
    pose_pub.publish(pose_msg)

#


while not rospy.is_shutdown():

    #try:
    Update()

    #except:
        #if options.debug:  rospy.logwarn ('EXCEPTION!!!' )
    if options.topic !=None:
        PublishPose(x,y,z)

    # rate.sleep()

rospy.loginfo("Closing serial connection")
func.COM_CloseConnection()

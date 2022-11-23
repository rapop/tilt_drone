#!/usr/bin/env python

import sys
import rospy
import time
import func
from geometry_msgs.msg import PointStamped
from optparse import OptionParser
from operator import neg
# Handling options
usage = "usage: rosrun leica_interface %prog [options]"
parser = OptionParser(usage=usage)
parser.set_defaults(port="/dev/ttyUSB0",baudrate=230400, debug=False)
parser.add_option("-p", "--port", action="store", type="string", dest="port", help="specify used port [default: %default]")
parser.add_option("-b", "--baudrate", action="store", type="int", dest="baudrate", help="specify used baudrate [default: %default]")
parser.add_option("-d", "--debug", action="store_true", dest="debug", help="print debug information")
(options, args) = parser.parse_args()


dataToSend = ''

def ReadData(data):
    global dataToSend
    dataToSend = str(data.header.seq) +" "+ str(data.point.x) +" "+ str(data.point.y) +" "+ str(data.point.z)

#connect to radio module
if func.COM_OpenConnection(options.port, options.baudrate)[0]:
    sys.exit("Can not open Port... exiting")

# Set up ROS:
rospy.init_node('sender_node')
# rate = rospy.Rate(30)
rate = rospy.Rate(100)
#rospy.Timer(rospy.Duration(0.03), SendData)
rospy.Subscriber("/leica/worldposition", PointStamped, ReadData)
print "ROS-node is set up"

while not rospy.is_shutdown():
    print ('Sending data: '+ dataToSend)
    func.COM_SendData(dataToSend)
    rate.sleep()
    # TODO: Implement check if the data is the same

rospy.loginfo("Closing serial connection")
func.COM_CloseConnection()

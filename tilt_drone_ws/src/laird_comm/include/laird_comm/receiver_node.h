#ifndef INCLUDE_LAIRD_COMMUNICATION_RECEIVER_NODE_H_
#define INCLUDE_LAIRD_COMMUNICATION_RECEIVER_NODE_H_

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <SerialStream.h>
#include <iostream>
#include <string>
#include <sstream>
#include <boost/algorithm/string.hpp>
#include <stdlib.h>
#include <std_msgs/String.h>
// #include <termios.h>
// #include <SerialPort.h>
#include <termios.h>
#include <stdio.h>   /* Standard input/output definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */

namespace patch
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << n;
        return stm.str() ;
    }
}

class LairdCommunicationReceiverNode
{
public:
  LairdCommunicationReceiverNode(const ros::NodeHandle& nh);
  ~LairdCommunicationReceiverNode();

  void readData();

private:
  // void dataCallback(const geometry_msgs::PointStamped::ConstPtr& msg);

  ros::NodeHandle nh_;
  ros::Publisher lairdReceiver_pub_;
  ros::Publisher lairdReceiver_odo_pub_;
  ros::Publisher lairdReceiver_odo_filtered_pub_;
  ros::Publisher trajectory_pub_;
  // ros::Subscriber lairdReceiver_sub_;

  LibSerial::SerialStream serial_port_;
  // LibSerial::SerialPort serial_port_;

  //params
  double xf_impact_offset_;
  double vx_impact_;
  double xf_max_;
  double xf_min_;
  double yf_max_;
  double yf_min_;
  double zf_max_;
  double zf_min_;
  double tf_max_;
  double tf_min_;

  // bool key_;
  // int fd_;
};

#endif

#ifndef INCLUDE_LAIRD_COMMUNICATION_SENDER_NODE_H_
#define INCLUDE_LAIRD_COMMUNICATION_SENDER_NODE_H_

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <SerialStream.h>
#include <iostream>
#include <string>
#include <sstream>
#include <chrono>
#include <ctime>
#include <boost/algorithm/string.hpp>
#include <boost/chrono/chrono.hpp>
#include <stdlib.h>
#include <std_msgs/String.h>
// #include <SerialPort.h>
// #include <serial.h>
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
        stm << n ;
        return stm.str() ;
    }
}

class LairdCommunicationSenderNode
{
public:
  LairdCommunicationSenderNode(const ros::NodeHandle& nh);
  ~LairdCommunicationSenderNode();

  // void readData();

private:
  void dataCallback(const geometry_msgs::TransformStamped::ConstPtr& msg);

  ros::NodeHandle nh_;
  ros::Subscriber lairdSender_sub_;
  // ros::Publisher lairdSender_pub_;

  LibSerial::SerialStream serial_port_;
  // serial::Serial serial_port_;
  // LibSerial::SerialPort serial_port_;

  // bool key_;
  int fd_;

};

#endif

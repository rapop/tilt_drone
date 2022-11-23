#ifndef INCLUDE_SERVO_CONTROL_NODE_H_
#define INCLUDE_SERVO_CONTROL_NODE_H_

#include <ros/ros.h>
#include <mavros_msgs/CommandInt.h>
#include <std_msgs/Float64.h>

class ServoControlNode
{
public:
  ServoControlNode(const ros::NodeHandle& nh);
  ~ServoControlNode();

private:
  void dataCallback(const std_msgs::Float64::ConstPtr& msg);
  bool setServo(int index, float value);

  ros::NodeHandle nh_;
  ros::Subscriber angle_sub_;
  ros::ServiceClient cmd_client_;
};

#endif

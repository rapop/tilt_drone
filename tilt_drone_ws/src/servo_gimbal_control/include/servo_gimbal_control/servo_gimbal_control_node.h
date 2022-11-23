#ifndef INCLUDE_SERVO_GIMBAL_CONTROL_NODE_H_
#define INCLUDE_SERVO_GIMBAL_CONTROL_NODE_H_

#include <ros/ros.h>
#include <mavros_msgs/CommandInt.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <math.h>

class ServoGimbalControlNode
{
public:
  ServoGimbalControlNode(const ros::NodeHandle& nh);
  ~ServoGimbalControlNode();

private:
  void odoCallback(const nav_msgs::Odometry::ConstPtr& msg);
  bool setServoYaw(int index, float value);
  bool setServoPitch(int index, float value);
  double clip(double n, double lower, double upper); 

  ros::NodeHandle nh_;
  ros::Subscriber angle_sub_;
  ros::ServiceClient cmd_client_;
};

#endif

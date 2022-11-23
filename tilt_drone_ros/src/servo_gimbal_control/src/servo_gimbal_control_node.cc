/**
 * @file servo_gimbal_control_node.cc
 */

#include <servo_gimbal_control/servo_gimbal_control_node.h>

ServoGimbalControlNode::ServoGimbalControlNode(const ros::NodeHandle& nh) : nh_(nh)
{
  angle_sub_ = nh_.subscribe<nav_msgs::Odometry>("ground_truth/odometry", 1,
                              &ServoGimbalControlNode::odoCallback, this, ros::TransportHints().tcpNoDelay());
  cmd_client_ = nh_.serviceClient<mavros_msgs::CommandInt>("mavros/cmd/command_int");
}

ServoGimbalControlNode::~ServoGimbalControlNode()
{
}

double ServoGimbalControlNode::clip(double n, double lower, double upper)
{
    return std::max(lower, std::min(n, upper));
}

bool ServoGimbalControlNode::setServoYaw(int index, float value) {
    int pulse_width = -500/0.7854*value + 1500; //mapping 0.7854rad to 1000ms and -0.7854rad to 2000ms
    // send mavros command message
    // http://docs.ros.org/api/mavros_msgs/html/srv/CommandLong.html
    // MAV_CMD_DO_SET_SERVO: https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_SERVO
    //ROS_INFO("setServo(%d,%f->%d)", index, value, pulse_width);
    mavros_msgs::CommandInt srv;
    srv.request.command = 183;
    srv.request.param1 = index;
    srv.request.param2 = pulse_width;
    cmd_client_.call(srv);
    //ROS_INFO("Success:%d", (bool)srv.response.success);
    return true;
}

bool ServoGimbalControlNode::setServoPitch(int index, float value) {
    int pulse_width = 500/0.7854*value + 1500;
    // send mavros command message
    // http://docs.ros.org/api/mavros_msgs/html/srv/CommandLong.html
    // MAV_CMD_DO_SET_SERVO: https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_SERVO
    //ROS_INFO("setServo(%d,%f->%d)", index, value, pulse_width);
    mavros_msgs::CommandInt srv;
    srv.request.command = 183;
    srv.request.param1 = index;
    srv.request.param2 = pulse_width;
    cmd_client_.call(srv);
    //ROS_INFO("Success:%d", (bool)srv.response.success);
    return true;
}

void ServoGimbalControlNode::odoCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  //get position
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;
  double z = msg->pose.pose.position.z;
  //get angles
  geometry_msgs::Quaternion quat = msg->pose.pose.orientation;
  tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
  tf::Matrix3x3 m(q);
  double phi, theta, psi;
  m.getRPY(phi, theta, psi);
  //ROS_INFO("angles(%f,%f,%f)", phi, theta, psi);

  double angle_pitch, angle_yaw;
  //double pitch_ref = 0;
  //double yaw_ref = 0;
  double x_target = 1.2;
  double x_change = 1; 
  double y_target = 0.52;
  double z_target = 1.0;
  double alpha = 0.2;
  
  double yaw_ref = atan((y-y_target)/(x-x_target));
  double pitch_ref = - atan((z-z_target)/(x-x_target)) + alpha*(x-x_change);

  double offset_pitch = -0.15708;
  double offset_yaw = 0.15708;

  angle_pitch = pitch_ref - theta + offset_pitch; //pitch_ref - theta;
  angle_yaw = yaw_ref - psi + offset_yaw; //yaw_ref - (psi+1.9);

  angle_pitch = ServoGimbalControlNode::clip(angle_pitch, -45*3.1416/180, 20*3.1416/180);
  angle_yaw = ServoGimbalControlNode::clip(angle_yaw, -20*3.1416/180, 20*3.1416/180);

  ServoGimbalControlNode::setServoPitch(4,angle_pitch);
  ServoGimbalControlNode::setServoYaw(5,angle_yaw);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "servo_gimbal_control_node");

  ros::NodeHandle nh;
  ServoGimbalControlNode node(nh);
  ros::spin();

  return 0;
}

/**
 * @file servo_control_node.cc
 */

#include <servo_control/servo_control_node.h>

ServoControlNode::ServoControlNode(const ros::NodeHandle& nh) : nh_(nh)
{
  angle_sub_ = nh_.subscribe<std_msgs::Float64>("hummingbird/front_axle_joint_position_controller/command", 1,
                              &ServoControlNode::dataCallback, this, ros::TransportHints().tcpNoDelay());
  cmd_client_ = nh_.serviceClient<mavros_msgs::CommandInt>("mavros/cmd/command_int");
}

ServoControlNode::~ServoControlNode()
{
}

bool ServoControlNode::setServo(int index, float value) {
    int pulse_width = 500/0.7854*value + 1500; //mapping 0.7854rad to 2000ms and -0.7854rad to 1000ms
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

void ServoControlNode::dataCallback(const std_msgs::Float64::ConstPtr& msg)
{
  float angle = msg->data;
  ServoControlNode::setServo(6,angle);
  //ServoControlNode::setServo(5,-0.15708); //yaw
  //ServoControlNode::setServo(4,-0.15708); //pitch
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "servo_control_node");

  ros::NodeHandle nh;
  ServoControlNode node(nh);
  ros::spin();

  return 0;
}

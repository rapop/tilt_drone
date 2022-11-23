#include <laird_comm/receiver_node.h>

LairdCommunicationReceiverNode::LairdCommunicationReceiverNode(const ros::NodeHandle& nh) : nh_(nh)
{
  lairdReceiver_pub_ = nh_.advertise<geometry_msgs::TransformStamped>("laird/dataReceived", 1);
  lairdReceiver_odo_pub_ = nh_.advertise<nav_msgs::Odometry>("laird/dataReceived_odo", 1);
  lairdReceiver_odo_filtered_pub_ = nh_.advertise<nav_msgs::Odometry>("laird/dataReceived_odo_filtered", 1);
  // lairdReceiver_sub_ = nh_.subscribe<geometry_msgs::PointStamped>("laird/dataToSend", 1,
  //                             &LairdCommunicationReceiverNode::dataCallback, this, ros::TransportHints().tcpNoDelay());
  trajectory_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("/hummingbird/mav_ilqr_control/trajectory", 1);

  //retrieve params
  nh_.getParam("/laird/xf_impact_offset", xf_impact_offset_);
  nh_.getParam("/laird/vx_impact", vx_impact_);
  nh_.getParam("/laird/xf_max", xf_max_);
  nh_.getParam("/laird/xf_min", xf_min_);
  nh_.getParam("/laird/yf_max", yf_max_);
  nh_.getParam("/laird/yf_min", yf_min_);
  nh_.getParam("/laird/zf_max", zf_max_);
  nh_.getParam("/laird/zf_min", zf_min_);
  nh_.getParam("/laird/tf_max", tf_max_);
  nh_.getParam("/laird/tf_min", tf_min_);


  serial_port_.Open("/dev/ttyS4");
  //serial_port_.SetBaudRate(LibSerial::SerialStreamBuf::BAUD_115200);
  serial_port_.SetCharSize(LibSerial::SerialStreamBuf::CHAR_SIZE_8);
  serial_port_.SetParity(LibSerial::SerialStreamBuf::PARITY_NONE);
  serial_port_.SetNumOfStopBits(1);
  serial_port_.SetFlowControl(LibSerial::SerialStreamBuf::FLOW_CONTROL_NONE);
  serial_port_.SetVTime(1);
  serial_port_.SetVMin(100);
  // fd_ = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
  system("echo 'pop123123' | sudo -S stty -F /dev/ttyS4 115200");
  //system("setserial /dev/ttyS4 low_latency");
  ROS_INFO_STREAM("Receiver Connected to Port.");
  // key_ = false;
}

LairdCommunicationReceiverNode::~LairdCommunicationReceiverNode()
{
}

void LairdCommunicationReceiverNode::readData()
{
    try
    {
      // tcflush(serial_port_,TCIFLUSH);
      geometry_msgs::TransformStamped data;
      nav_msgs::Odometry data_odo;
      nav_msgs::Odometry data_odo_filtered;
      std::string read_string = "" ;

      // if(key_==true)
      // {
      //   serial_port_.Open("/dev/ttyUSB0");
      //   serial_port_.SetBaudRate(LibSerial::SerialStreamBuf::BAUD_115200);
      //   serial_port_.SetCharSize(LibSerial::SerialStreamBuf::CHAR_SIZE_8);
      //   serial_port_.SetParity(LibSerial::SerialStreamBuf::PARITY_NONE);
      //   serial_port_.SetNumOfStopBits(1);
      //   serial_port_.SetFlowControl( LibSerial::SerialStreamBuf::FLOW_CONTROL_NONE ) ;
      //   serial_port_.SetVTime(1);
      //   serial_port_.SetVMin(100);
      // }

      std::getline(serial_port_, read_string) ;

      // tcflush(fd_, TCIFLUSH);
      // serial_port_.Close();
      // key_ = true;

      // serial_port_.ReadLine(read_string);

      std::vector<std::string> splited_string;
      boost::split(splited_string, read_string, boost::is_any_of(" "));

      uint32_t seqNum = strtoul(splited_string[0].c_str(), NULL, 0);
      uint32_t seqNum2 = strtoul(splited_string[1].c_str(), NULL, 0);
      double x = atof(splited_string[2].c_str());
      double y = atof(splited_string[3].c_str());
      double z = atof(splited_string[4].c_str());
      double qx = atof(splited_string[5].c_str());
      double qy = atof(splited_string[6].c_str());
      double qz = atof(splited_string[7].c_str());
      double qw = atof(splited_string[8].c_str());
      double vx = atof(splited_string[9].c_str());
      double vy = atof(splited_string[10].c_str());
      double vz = atof(splited_string[11].c_str());
      double tf = atof(splited_string[12].c_str());
      double xf = atof(splited_string[13].c_str());
      double yf = atof(splited_string[14].c_str());
      double zf = atof(splited_string[15].c_str());
      //double dtheta = atof(splited_string[13].c_str());
      //double dpsi = atof(splited_string[14].c_str());
      //double vx_filtered = atof(splited_string[15].c_str());
      //double vy_filtered = atof(splited_string[16].c_str());
      //double vz_filtered = atof(splited_string[17].c_str());

      //data.header.seq = seqNum;
      data.header.stamp.sec = seqNum;
      data.header.stamp.nsec = seqNum2;
      data.transform.translation.x = x;
      data.transform.translation.y = y;
      data.transform.translation.z = z;
      data.transform.rotation.x = qx;
      data.transform.rotation.y = qy;
      data.transform.rotation.z = qz;
      data.transform.rotation.w = qw;

      std::string seqNum_str = patch::to_string(seqNum);
      std::string seqNum_str2 = patch::to_string(seqNum2);
      std::string x_str = patch::to_string(x);
      std::string y_str = patch::to_string(y);
      std::string z_str = patch::to_string(z);
      std::string qx_str = patch::to_string(qx);
      std::string qy_str = patch::to_string(qy);
      std::string qz_str = patch::to_string(qz);
      std::string qw_str = patch::to_string(qw);
      std::string tf_str = patch::to_string(tf);
      std::string xf_str = patch::to_string(xf);
      std::string yf_str = patch::to_string(yf);
      std::string zf_str = patch::to_string(zf);

      std::string dataPublished;
      dataPublished = seqNum_str + " " + seqNum_str2 + " " + x_str + " " + y_str + " " + z_str +
                    " " + qx_str + " " + qy_str + " " + qz_str + " " + qw_str + " " + tf_str + " " + xf_str + " " + yf_str + " " + zf_str;

      std::cout << "Published : " << dataPublished << std::endl;
      
      //print laird latency
      ros::WallTime time_wall = ros::WallTime::now();
      double now_time = time_wall.toSec();
      double before_time = data.header.stamp.toSec();
      double diff_times = time_wall.toSec() - data.header.stamp.toSec();
      std::cout <<std::setprecision(5)<< "Lag laird only : " << diff_times << std::endl;// " before : " << seqNum << "," <<seqNum2 <<" beforetosec : " <<before_time<<" now : " <<now_time <<  std::endl;

      //publish transform
      lairdReceiver_pub_.publish(data);
      
      //publish odometry
      data_odo.header.stamp.sec = seqNum;
      data_odo.header.stamp.nsec = seqNum2;
      data_odo.pose.pose.position.x = x;
      data_odo.pose.pose.position.y = y;
      data_odo.pose.pose.position.z = z;
      data_odo.pose.pose.orientation.x = qx;
      data_odo.pose.pose.orientation.y = qy;
      data_odo.pose.pose.orientation.z = qz;
      data_odo.pose.pose.orientation.w = qw;

      data_odo.twist.twist.linear.x = vx;
      data_odo.twist.twist.linear.y = vy;
      data_odo.twist.twist.linear.z = vz;
      //data_odo.twist.twist.angular.x = zf;
      //data_odo.twist.twist.angular.y = dtheta;
      //data_odo.twist.twist.angular.z = dpsi;

      lairdReceiver_odo_pub_.publish(data_odo);
/*
      //publish odometry with filtered speeds
      data_odo_filtered.header.stamp.sec = seqNum;
      data_odo_filtered.header.stamp.nsec = seqNum2;
      data_odo_filtered.pose.pose.position.x = x;
      data_odo_filtered.pose.pose.position.y = y;
      data_odo_filtered.pose.pose.position.z = z;
      data_odo_filtered.pose.pose.orientation.x = qx;
      data_odo_filtered.pose.pose.orientation.y = qy;
      data_odo_filtered.pose.pose.orientation.z = qz;
      data_odo_filtered.pose.pose.orientation.w = qw;

      data_odo_filtered.twist.twist.linear.x = vx_filtered;
      data_odo_filtered.twist.twist.linear.y = vy_filtered;
      data_odo_filtered.twist.twist.linear.z = vz_filtered;
      data_odo_filtered.twist.twist.angular.x = zf;
      data_odo_filtered.twist.twist.angular.y = dtheta;
      data_odo_filtered.twist.twist.angular.z = dpsi;

      lairdReceiver_odo_filtered_pub_.publish(data_odo_filtered);
*/

      if (xf<=xf_max_ && xf>=xf_min_ &&
        yf<=yf_max_ && yf>=yf_min_ &&
        zf<=zf_max_ && zf>=zf_min_ &&
        tf<=tf_max_ && tf>=tf_min_)
      {
        //publish trajectory
        trajectory_msgs::MultiDOFJointTrajectoryPoint trajectoryMsg;

        geometry_msgs::Transform transformsMsg;
        transformsMsg.translation.x = xf - xf_impact_offset_;
        transformsMsg.translation.y = yf;
        transformsMsg.translation.z = zf;
        transformsMsg.rotation.x = 0;
        transformsMsg.rotation.y = 0;
        transformsMsg.rotation.z = 0;
        transformsMsg.rotation.w = 1;

        geometry_msgs::Twist velocitiesMsg;
        velocitiesMsg.linear.x = vx_impact_;
        velocitiesMsg.linear.y = 0;
        velocitiesMsg.linear.z = 0;
        velocitiesMsg.angular.x = 0;
        velocitiesMsg.angular.y = 0;
        velocitiesMsg.angular.z = 0;

        geometry_msgs::Twist accelerationsMsg;
        accelerationsMsg.linear.x = 0;
        accelerationsMsg.linear.y = 0;
        accelerationsMsg.linear.z = 0;
        accelerationsMsg.angular.x = 0;
        accelerationsMsg.angular.y = 0;
        accelerationsMsg.angular.z = 0;

        ros::Duration timeMsg = ros::Duration(tf);

        trajectoryMsg.transforms.push_back(transformsMsg);
        trajectoryMsg.velocities.push_back(velocitiesMsg);
        trajectoryMsg.accelerations.push_back(accelerationsMsg);
        trajectoryMsg.time_from_start = timeMsg;
        trajectory_pub_.publish(trajectoryMsg);
      }
    } catch (int e)
    {
      std::cout << "NO DATA! " << std::endl;
    }
}

// void LairdCommunicationReceiverNode::dataCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
// {
//   LairdCommunicationReceiverNode::readData();
//   uint32_t seqNum = msg->header.seq;
//   double x = msg->point.x;
//   double y = msg->point.y;
//   double z = msg->point.z;
//
//   std::string seqNum_str = patch::to_string(seqNum);
//   std::string x_str = patch::to_string(x);
//   std::string y_str = patch::to_string(y);
//   std::string z_str = patch::to_string(z);
//
//   std::string dataToSend;
//   dataToSend = seqNum_str + " " + x_str + " " + y_str + " " + z_str;
//
//   // if(key_==true)
//   // {
//   //   serial_port_.Open("/dev/ttyUSB1", std::ios::out);
//   //   serial_port_.SetBaudRate(LibSerial::SerialStreamBuf::BAUD_115200);
//   //   serial_port_.SetCharSize(LibSerial::SerialStreamBuf::CHAR_SIZE_8);
//   //   serial_port_.SetParity(LibSerial::SerialStreamBuf::PARITY_NONE);
//   //   serial_port_.SetNumOfStopBits(1);
//   //   serial_port_.SetFlowControl( LibSerial::SerialStreamBuf::FLOW_CONTROL_NONE ) ;
//   //   serial_port_.SetVTime(1);
//   //   serial_port_.SetVMin(100);
//   // }
//
//   serial_port_ << dataToSend << std::endl;
//
//   // serial_port_.Close();
//   // key_ = true;
//   // serial_port_.Write(dataToSend);
//   std::cout << "Sending IMU data: " << dataToSend <<std::endl;
//   // usleep(100);
//   // tcflush(fd_, TCIOFLUSH);
// }

int main(int argc, char** argv)
{
  ros::init(argc, argv, "laird_receiver_node");

  ros::NodeHandle nh;
  LairdCommunicationReceiverNode receiverNode(nh);
  // ros::NodeHandle nh;
  // LibSerial::SerialStream serial_port;
  //
  // ros::Rate loop_rate(100);

  while (ros::ok())
  {
    receiverNode.readData();

    // ros::spinOnce();

    // loop_rate.sleep();
    // ros::spin();
    // usleep(100);
  }
  // ros::spin();
  // serial_port.Close() ;

  return 0;
}

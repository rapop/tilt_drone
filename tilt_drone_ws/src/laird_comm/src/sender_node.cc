#include <laird_comm/sender_node.h>

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;

LairdCommunicationSenderNode::LairdCommunicationSenderNode(const ros::NodeHandle& nh) : nh_(nh)
{
  // lairdSender_pub_ = nh_.advertise<geometry_msgs::PointStamped>("laird/IMUReceived", 1);
  lairdSender_sub_ = nh_.subscribe<geometry_msgs::TransformStamped>("laird/dataToSend", 1,
                              &LairdCommunicationSenderNode::dataCallback, this, ros::TransportHints().tcpNoDelay());

  serial_port_.Open("/dev/ttyUSB0", std::ios::out);
  // serial_port_.SetBaudRate(LibSerial::SerialStreamBuf::BAUD_230400);
  serial_port_.SetCharSize(LibSerial::SerialStreamBuf::CHAR_SIZE_8);
  serial_port_.SetParity(LibSerial::SerialStreamBuf::PARITY_NONE);
  serial_port_.SetNumOfStopBits(1);
  serial_port_.SetFlowControl(LibSerial::SerialStreamBuf::FLOW_CONTROL_NONE);
  serial_port_.SetVTime(1);
  serial_port_.SetVMin(100);
  // fd_ = open("/dev/ttyUSB1", O_RDWR | O_NOCTTY | O_NDELAY);
  system("echo 'pop123123' | sudo -S stty -F /dev/ttyUSB0 115200");
  system("setserial /dev/ttyUSB0 low_latency");

  ROS_INFO_STREAM("Sender Connected to Port.");
  // key_ = false;
}

LairdCommunicationSenderNode::~LairdCommunicationSenderNode()
{
}

// void LairdCommunicationSenderNode::readData()
// {
//     try
//     {
//       // tcflush(serial_port_,TCIFLUSH);
//       geometry_msgs::PointStamped data;
//       std::string read_string = "" ;
//
//       // if(key_==true)
//       // {
//       //   serial_port_.Open("/dev/ttyUSB0");
//       //   serial_port_.SetBaudRate(LibSerial::SerialStreamBuf::BAUD_115200);
//       //   serial_port_.SetCharSize(LibSerial::SerialStreamBuf::CHAR_SIZE_8);
//       //   serial_port_.SetParity(LibSerial::SerialStreamBuf::PARITY_NONE);
//       //   serial_port_.SetNumOfStopBits(1);
//       //   serial_port_.SetFlowControl( LibSerial::SerialStreamBuf::FLOW_CONTROL_NONE ) ;
//       //   serial_port_.SetVTime(1);
//       //   serial_port_.SetVMin(100);
//       // }
//
//       std::getline(serial_port_, read_string) ;
//
//       // tcflush(fd_, TCIFLUSH);
//       // serial_port_.Close();
//       // key_ = true;
//
//       // serial_port_.ReadLine(read_string);
//
//       std::vector<std::string> splited_string;
//       boost::split(splited_string, read_string, boost::is_any_of(" "));
//
//       uint32_t seqNum = strtoul(splited_string[0].c_str(), NULL, 0);
//       double x = atof(splited_string[1].c_str());
//       double y = atof(splited_string[2].c_str());
//       double z = atof(splited_string[3].c_str());
//
//       data.header.seq = seqNum;
//       data.point.x = x;
//       data.point.y = y;
//       data.point.z = z;
//
//       std::string seqNum_str = patch::to_string(seqNum);
//       std::string x_str = patch::to_string(x);
//       std::string y_str = patch::to_string(y);
//       std::string z_str = patch::to_string(z);
//
//       std::string dataPublished;
//       dataPublished = seqNum_str + " " + x_str + " " + y_str + " " + z_str;
//
//       std::cout << "Published : " << dataPublished << std::endl;
//       lairdSender_pub_.publish(data);
//     } catch (int e)
//     {
//       std::cout << "NO DATA! " << std::endl;
//     }
// }

void LairdCommunicationSenderNode::dataCallback(const geometry_msgs::TransformStamped::ConstPtr& msg)
{
  auto now2 = std::chrono::high_resolution_clock::now();
  double sysTime = std::chrono::duration_cast<std::chrono::nanoseconds>(now2.time_since_epoch()).count() / 1e9;

	std::time_t secs = std::time(nullptr);
  int time_secs = sysTime;
  //std::cout << "here: " << secs << " "<<sysTime<< " "<<time_secs<<std::endl;
  //uint32_t seqNum = msg->header.seq;
  uint32_t seqNum = secs;//msg->header.stamp.sec;
  uint32_t seqNum2 = floor((sysTime - time_secs)*1e9);//msg->header.stamp.nsec;
  double x = msg->transform.translation.x;
  double y = msg->transform.translation.y;
  double z = msg->transform.translation.z;
  double qx = msg->transform.rotation.x;
  double qy = msg->transform.rotation.y;
  double qz = msg->transform.rotation.z;
  double qw = msg->transform.rotation.w;
  double timef(1.2);
  double xf(0.0);
  double yf(0.0);
  double zf(1.0);

  std::string seqNum_str = patch::to_string(seqNum);
  std::string seqNum_str2 = patch::to_string(seqNum2);
  std::string x_str = patch::to_string(x);
  std::string y_str = patch::to_string(y);
  std::string z_str = patch::to_string(z);
  std::string qx_str = patch::to_string(qx);
  std::string qy_str = patch::to_string(qy);
  std::string qz_str = patch::to_string(qz);
  std::string qw_str = patch::to_string(qw);
  std::string timef_str = patch::to_string(timef);
  std::string xf_str = patch::to_string(xf);
  std::string yf_str = patch::to_string(yf);
  std::string zf_str = patch::to_string(zf);

  std::string dataToSend;
  dataToSend = seqNum_str + " " + seqNum_str2 + " " + x_str + " " + y_str + " " + z_str +
                " " + qx_str + " " + qy_str + " " + qz_str + " " + qw_str +
                " " + timef_str + " " + xf_str + " " + yf_str + " " + zf_str;

  // if(key_==true)
  // {
  //   serial_port_.Open("/dev/ttyUSB1", std::ios::out);
  //   serial_port_.SetBaudRate(LibSerial::SerialStreamBuf::BAUD_115200);
  //   serial_port_.SetCharSize(LibSerial::SerialStreamBuf::CHAR_SIZE_8);
  //   serial_port_.SetParity(LibSerial::SerialStreamBuf::PARITY_NONE);
  //   serial_port_.SetNumOfStopBits(1);
  //   serial_port_.SetFlowControl( LibSerial::SerialStreamBuf::FLOW_CONTROL_NONE ) ;
  //   serial_port_.SetVTime(1);
  //   serial_port_.SetVMin(100);
  // }

  serial_port_ << dataToSend << std::endl;

  // serial_port_.Close();
  // key_ = true;
  // serial_port_.Write(dataToSend);
  std::cout << "Sending data: " << dataToSend <<std::endl;
  // usleep(10000);
  // tcflush(fd_, TCIOFLUSH);
  // LairdCommunicationSenderNode::readData();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "laird_sender_node");

  ros::NodeHandle nh;
  LairdCommunicationSenderNode senderNode(nh);
  // ros::NodeHandle nh;
  // LibSerial::SerialStream serial_port;
  //
  // ros::Rate loop_rate(100);
  // //
  // while (ros::ok())
  // {
  //   ros::spinOnce();
  //   loop_rate.sleep();
  // }
  // while (ros::ok())
  // {
  //
  //   senderNode.readData();
  //   ros::spinOnce();
  //
  //   loop_rate.sleep();
  //   // ros::spin();
  //
  //   // usleep(100);
  // }
  ros::spin();
  // serial_port.Close() ;

  return 0;
}

#ifndef INCLUDE_ITERATIVE_LINEAR_QUADRATIC_REGULATOR_NODE_H_
#define INCLUDE_ITERATIVE_LINEAR_QUADRATIC_REGULATOR_NODE_H_

#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <mav_msgs/conversions.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <tf/transform_datatypes.h>
#include <mav_ilqr_control/ilqr_control.h>
#include <dynamic_reconfigure/server.h>
#include <mav_ilqr_control/iLQRControllerConfig.h>
#include <ctime>
#include <time.h>
#include <chrono>

class IterativeLinearQuadraticRegulatorNode
{
public:
  IterativeLinearQuadraticRegulatorNode(const ros::NodeHandle& nh, const ros::NodeHandle private_nh, double dt = 0.01, int tN = 50);
  ~IterativeLinearQuadraticRegulatorNode();

private:
  void trajCallback(const trajectory_msgs::MultiDOFJointTrajectoryPoint::ConstPtr& msg);
  void posCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void odoCallback(const nav_msgs::Odometry::ConstPtr& odometry);
  void odoCallback_angular_rate(const nav_msgs::Odometry::ConstPtr& odometry);
  void imuCallback(const sensor_msgs::Imu::ConstPtr& imu);
  //void tiltCallback(const sensor_msgs::JointState::ConstPtr& tilt);
  void createTrajectoryBatch(double const& x,
    double const& y,
    double const& z,
    double const& vx,
    double const& vy,
    double const& vz,
    double const& ax,
    double const& ay,
    double const& az,
    double const& yaw,
    double const& t_impact);
  void getTrajectoryBatch(Eigen::MatrixXd &xd, Eigen::MatrixXd &ddz_ff, Eigen::MatrixXd &ddx_ff, Eigen::MatrixXd &ddy_ff);
  void commandRollPitchYawrateThrust(const ros::Time& odo_time);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  void DynConfigCallback(mav_ilqr_control::iLQRControllerConfig &config, uint32_t level);
  dynamic_reconfigure::Server<mav_ilqr_control::iLQRControllerConfig> dyn_config_server_;

void printTrajectoryPoint(double const& psi_des,
    double const& x,
    double const& dx,
    double const& y,
    double const& dy,
    double const& z,
    double const& dz);

  ros::Publisher commandRollPitchYawrateThrust_pub_;
  ros::Publisher commandFrontAngle_pub_;
  ros::Publisher commandBackAngle_pub_;
  ros::Publisher rawImu_pub_;
  ros::Subscriber odo_sub_;
  ros::Subscriber odo_ang_sub_;
  ros::Subscriber traj_sub_;
  //ros::Subscriber tilt_sub_;
  ros::Subscriber pos_sub_;
  ros::Subscriber imu_sub_;
  ros::Publisher traj_point_pub_;

  double x_;
  double y_;
  double z_;
  double phi_;
  double theta_;
  double psi_;
  double dx_;
  double dy_;
  double dz_;
  double dphi_;
  double dtheta_;
  double dpsi_;
  //double tilt_angle_;
  IterativeLinearQuadraticRegulator iLQR_;
  double dt_;
  int tN_;
  static constexpr int n_states_ = 9;
	static constexpr int n_actions_ = 5;
  Eigen::MatrixXd trajArray;
  int counter_;
  bool maintainPosition_;
  bool gotoPosition_;
  bool soft_gains_;
  clock_t begin_time_;
  double K_yaw_;
  double yaw_rate_limit_;
  Eigen::Vector3d position_error_integration_;
  Eigen::Vector4d position_desired_;

  bool entered_pos_limit_;

  bool speed_limit_on_;
  double vx_limit_;
  double vy_limit_;
  double vz_limit_;

  bool position_limit_on_;
  double x_limit_inf_;
  double x_limit_sup_;
  double y_limit_inf_;
  double y_limit_sup_;
  double z_limit_inf_;
  double z_limit_sup_;


};

#endif

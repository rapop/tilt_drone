#include <mav_ilqr_control/ilqr_control_node.h>
#include <mav_ilqr_control/traj_gen.h>

// constants
constexpr int IterativeLinearQuadraticRegulatorNode::n_states_;
constexpr int IterativeLinearQuadraticRegulatorNode::n_actions_;

IterativeLinearQuadraticRegulatorNode::IterativeLinearQuadraticRegulatorNode(const ros::NodeHandle& nh,
                                                                             const ros::NodeHandle private_nh,
                                                                           	 double dt,
	                                                                           int tN) :
                                                                             dt_(dt),
                                                                             tN_(tN),
																																						 nh_(nh),
																																				     private_nh_(private_nh),
																																						 iLQR_(nh, private_nh, dt, tN)
{
  commandRollPitchYawrateThrust_pub_ = nh_.advertise<mav_msgs::RollPitchYawrateThrust>("command/roll_pitch_yawrate_thrust", 1);
  commandFrontAngle_pub_ = nh_.advertise<std_msgs::Float64>("front_axle_joint_position_controller/command", 1);
  commandBackAngle_pub_ = nh_.advertise<std_msgs::Float64>("back_axle_joint_position_controller/command", 1);
  rawImu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu_raw_corrected", 1);
  traj_point_pub_ = nh_.advertise<nav_msgs::Odometry>("mav_ilqr_control/trajectoryReceived", 1);


  dynamic_reconfigure::Server<mav_ilqr_control::iLQRControllerConfig>::CallbackType f;
  f = boost::bind(&IterativeLinearQuadraticRegulatorNode::DynConfigCallback, this, _1, _2);
  dyn_config_server_.setCallback(f);

  odo_sub_ = nh_.subscribe<nav_msgs::Odometry>("ground_truth/odometry", 1,
                              &IterativeLinearQuadraticRegulatorNode::odoCallback, this, ros::TransportHints().tcpNoDelay());
  odo_ang_sub_ = nh_.subscribe<nav_msgs::Odometry>("ground_truth/odometry_ang", 1,
                              &IterativeLinearQuadraticRegulatorNode::odoCallback_angular_rate, this, ros::TransportHints().tcpNoDelay());
  imu_sub_ = nh_.subscribe<sensor_msgs::Imu>("ground_truth/imu", 1,
                              &IterativeLinearQuadraticRegulatorNode::imuCallback, this, ros::TransportHints().tcpNoDelay());
  traj_sub_ = nh_.subscribe<trajectory_msgs::MultiDOFJointTrajectoryPoint>("mav_ilqr_control/trajectory",
                              1, &IterativeLinearQuadraticRegulatorNode::trajCallback, this, ros::TransportHints().tcpNoDelay());
  pos_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("mav_ilqr_control/position",
                              1, &IterativeLinearQuadraticRegulatorNode::posCallback, this, ros::TransportHints().tcpNoDelay());
  //tilt_sub_ = nh_.subscribe<sensor_msgs::JointState>("joint_states",
  //                            1, &IterativeLinearQuadraticRegulatorNode::tiltCallback, this, ros::TransportHints().tcpNoDelay());

  maintainPosition_ = true;
  gotoPosition_ = false;
  soft_gains_ = true;
  position_desired_.setZero();
  position_error_integration_.setZero();
  begin_time_ = clock();
  entered_pos_limit_ = false;

}

IterativeLinearQuadraticRegulatorNode::~IterativeLinearQuadraticRegulatorNode()
{

}

void IterativeLinearQuadraticRegulatorNode::trajCallback(const trajectory_msgs::MultiDOFJointTrajectoryPoint::ConstPtr& msg)
{
  // get desired state
  double x = msg->transforms.front().translation.x;
  double y = msg->transforms.front().translation.y;
  double z = msg->transforms.front().translation.z;
  geometry_msgs::Quaternion quat = msg->transforms.front().rotation;
  tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
  tf::Matrix3x3 m(q);
  double phi, theta, psi;
  m.getRPY(phi, theta, psi);

  double vx = msg->velocities.front().linear.x;
  double vy = msg->velocities.front().linear.y;
  double vz = msg->velocities.front().linear.z;

  double ax = msg->accelerations.front().linear.x;
  double ay = msg->accelerations.front().linear.y;
  double az = msg->accelerations.front().linear.z;

  ros::Duration t = msg->time_from_start;
  double t_impact = t.toSec();

  IterativeLinearQuadraticRegulatorNode::createTrajectoryBatch(x, y, z, vx, vy, vz, ax, ay, az, psi, t_impact);
  maintainPosition_ = false;
  gotoPosition_ = false;
  position_desired_.setZero();
  counter_ = 0;

}

void IterativeLinearQuadraticRegulatorNode::posCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  // get desired state
  double x = msg->pose.position.x;
  double y = msg->pose.position.y;
  double z = msg->pose.position.z;
  geometry_msgs::Quaternion quat = msg->pose.orientation;
  tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
  tf::Matrix3x3 m(q);
  double phi, theta, psi; //roll pitch yaw
  m.getRPY(phi, theta, psi);
  if (std::isnan(psi)) {psi=0.0;}

  position_desired_ << x, y, z, psi;

  maintainPosition_ = false;
  gotoPosition_ = true;
  counter_ = 0;

}

void IterativeLinearQuadraticRegulatorNode::createTrajectoryBatch(double const& x,
  double const& y,
  double const& z,
  double const& vx,
  double const& vy,
  double const& vz,
  double const& ax,
  double const& ay,
  double const& az,
  double const& yaw,
  double const& t_impact)
{
    int wps(2);
    Eigen::MatrixXd constraints_x(5,wps+1);
    constraints_x << x_, x, x_, // position constraints
                      0, vx, 0, // velocity constraints
                      0, ax, 0, // acceleration constraints
                      0, NAN, 0, // jerk constraints
                      0, NAN, 0; // snap constraints
    Eigen::MatrixXd constraints_y(5,wps+1);
    constraints_y << y_, y, y_,
                     0, vy, 0,
                     0, ay, 0,
                     0, NAN, 0,
                     0, NAN, 0;
    Eigen::MatrixXd constraints_z(5,wps+1);
    constraints_z << z_, z, z_,
                     0, vz, 0,
                     0, az, 0,
                     0, NAN, 0,
                     0, NAN, 0;
    Eigen::MatrixXd constraints_psi(5,wps+1);
    constraints_psi << psi_, yaw, psi_,
                      0, 0, 0,
                      0, 0, 0,
                      0, NAN, 0,
                      0, NAN, 0;

    Eigen::VectorXd t(wps+1);
    t << 0, t_impact, 2;
    double dt2(0.01);
    TrajectoryGenerator trajObject(wps, t, dt2, constraints_x, constraints_y, constraints_z, constraints_psi);

    trajArray = trajObject.getFullTraj();

    //Adding repeatedly the point for iLQR
    int const tN2 = 50;
    Eigen::MatrixXd endConstStates=Eigen::MatrixXd::Zero(tN2,trajArray.cols());
    for (int i(0); i<tN2; i++)
    {
        endConstStates(i,0) = trajArray(trajArray.rows()-1,0) + dt2*(i+1);
        endConstStates.block(i,1,1,endConstStates.cols()-1) = trajArray.block(trajArray.rows()-1,1,1,trajArray.cols()-1);
    }
    trajArray.conservativeResize(trajArray.rows() + tN2, trajArray.cols());
    trajArray.bottomRows(tN2) = endConstStates;
    //std::cout << "Traj row : " << trajArray.row(0) << std::endl;
}

void IterativeLinearQuadraticRegulatorNode::getTrajectoryBatch(Eigen::MatrixXd &xd, Eigen::MatrixXd &ddz_ff, Eigen::MatrixXd &ddx_ff, Eigen::MatrixXd &ddy_ff)
{

  if (maintainPosition_) {

    Eigen::MatrixXd psi_des = Eigen::MatrixXd::Zero(tN_, 1);
    Eigen::MatrixXd x_des = Eigen::MatrixXd::Ones(tN_, 1);
    Eigen::MatrixXd dx_des = Eigen::MatrixXd::Zero(tN_, 1);
    Eigen::MatrixXd y_des = Eigen::MatrixXd::Ones(tN_, 1);
    Eigen::MatrixXd dy_des = Eigen::MatrixXd::Zero(tN_, 1);
    Eigen::MatrixXd z_des = Eigen::MatrixXd::Ones(tN_, 1);
    Eigen::MatrixXd dz_des = Eigen::MatrixXd::Zero(tN_, 1);
    Eigen::MatrixXd ddz_des = Eigen::MatrixXd::Zero(tN_, 1);

    xd.col(2) = psi_des;
    xd.col(3) = x_des*-1;
    xd.col(4) = dx_des;
    xd.col(5) = y_des*0.52;
    xd.col(6) = dy_des;
    xd.col(7) = z_des*1.1;
    xd.col(8) = dz_des;
    ddz_ff = ddz_des;
    ddx_ff = ddz_des;
    ddy_ff = ddz_des;
  }
  else if (gotoPosition_ && !position_desired_.norm()==0) {

    Eigen::MatrixXd psi_des = Eigen::MatrixXd::Ones(tN_, 1);
    Eigen::MatrixXd x_des = Eigen::MatrixXd::Ones(tN_, 1);
    Eigen::MatrixXd dx_des = Eigen::MatrixXd::Zero(tN_, 1);
    Eigen::MatrixXd y_des = Eigen::MatrixXd::Ones(tN_, 1);
    Eigen::MatrixXd dy_des = Eigen::MatrixXd::Zero(tN_, 1);
    Eigen::MatrixXd z_des = Eigen::MatrixXd::Ones(tN_, 1);
    Eigen::MatrixXd dz_des = Eigen::MatrixXd::Zero(tN_, 1);
    Eigen::MatrixXd ddz_des = Eigen::MatrixXd::Zero(tN_, 1);
    xd.col(2) = psi_des*position_desired_(3);
    xd.col(3) = x_des*position_desired_(0);
    xd.col(4) = dx_des;
    xd.col(5) = y_des*position_desired_(1);
    xd.col(6) = dy_des;
    xd.col(7) = z_des*position_desired_(2);
    xd.col(8) = dz_des;
    ddz_ff = ddz_des;
    ddx_ff = ddz_des;
    ddy_ff = ddz_des;
  }
  else {
    Eigen::MatrixXd psi_des = trajArray.block(counter_, 16, tN_, 1);
    Eigen::MatrixXd x_des = trajArray.block(counter_, 1, tN_, 1);
    Eigen::MatrixXd dx_des = trajArray.block(counter_, 2, tN_, 1);
    Eigen::MatrixXd ddx_des = trajArray.block(counter_, 3, tN_, 1);
    Eigen::MatrixXd y_des = trajArray.block(counter_, 6, tN_, 1);
    Eigen::MatrixXd dy_des = trajArray.block(counter_, 7, tN_, 1);
    Eigen::MatrixXd ddy_des = trajArray.block(counter_, 8, tN_, 1);
    Eigen::MatrixXd z_des = trajArray.block(counter_, 11, tN_, 1);
    Eigen::MatrixXd dz_des = trajArray.block(counter_, 12, tN_, 1);
    Eigen::MatrixXd ddz_des = trajArray.block(counter_, 13, tN_, 1);

    xd.col(2) = psi_des;
    xd.col(3) = x_des;
    xd.col(4) = dx_des;
    xd.col(5) = y_des;
    xd.col(6) = dy_des;
    xd.col(7) = z_des;
    xd.col(8) = dz_des;
    ddz_ff = ddz_des;
    ddx_ff = ddx_des;
    ddy_ff = ddy_des;

    //pub desired trajectory point
    IterativeLinearQuadraticRegulatorNode::printTrajectoryPoint(psi_des(0), x_des(0),
                      dx_des(0), y_des(0), dy_des(0), z_des(0), dz_des(0));

    ++counter_;
    //std::cout<< "counter " << counter_ <<std::endl;

    if (counter_ > 200){
    // if (counter_ > 200){
      maintainPosition_ = true;
    }
  }

}

void IterativeLinearQuadraticRegulatorNode::commandRollPitchYawrateThrust(const ros::Time& odo_time)
{

  ros::WallTime time1 = ros::WallTime::now();
	Eigen::MatrixXd x0 = Eigen::MatrixXd::Zero(1, n_states_);
	x0 << phi_, theta_, psi_, x_, dx_, y_, dy_, z_, dz_;
  // std::cout<< " x0 " << x0 << std::endl;
  Eigen::MatrixXd xd = Eigen::MatrixXd::Zero(tN_, n_states_);
  Eigen::MatrixXd ddz_ff = Eigen::MatrixXd::Zero(tN_, 1);
  Eigen::MatrixXd ddx_ff = Eigen::MatrixXd::Zero(tN_, 1);
  Eigen::MatrixXd ddy_ff = Eigen::MatrixXd::Zero(tN_, 1);
  IterativeLinearQuadraticRegulatorNode::getTrajectoryBatch(xd, ddz_ff, ddx_ff, ddy_ff);
  // std::cout<< " xd " << xd << std::endl;

	Eigen::MatrixXd xCommand = Eigen::MatrixXd::Zero(tN_, n_states_);
	Eigen::MatrixXd uCommand = Eigen::MatrixXd::Zero(tN_, n_actions_);

	double cost(0);
	//const clock_t begin_time = clock();

  //Safety speed limits
  if (((fabs(dx_) > vx_limit_)  || (fabs(dy_) > vy_limit_) || (fabs(dz_) > vz_limit_)) && speed_limit_on_) {
    std::cout << "---------------- Safety speed reached ----------------" << std::endl;
    std::cout << "dx_: " << dx_ <<  " vx_limit_: " << vx_limit_ << " dy_: " << dy_ <<  " vy_limit_: " << vy_limit_ << " dz_: " << dz_ <<  " vz_limit_: " << vz_limit_ << std::endl;
    std::cout << "---------------- -------------------- ----------------" << std::endl;
    position_desired_ << x_, y_, z_, psi_;
    maintainPosition_ = false;
    gotoPosition_ = true;
    counter_ = 0;
  }

  if (maintainPosition_ || gotoPosition_) {soft_gains_=true;} else {soft_gains_=false;}

	iLQR_.getControl(x0, xd, xCommand, uCommand, cost, soft_gains_);
	//std::cout << "time: " << float( clock () - begin_time ) /  CLOCKS_PER_SEC << std::endl;
	//std::cout<< uCommand(0,0)/cos(uCommand(0,4)) << " " << uCommand(0,1) << " " << uCommand(0,2) << " " << uCommand(0,3) << " " << uCommand(0,4) << std::endl;
  //std::cout<<  " xCommand " << xCommand << std::endl;
	// yaw controller
  double yaw_error = xd(0,2) - psi_;

  if (std::abs(yaw_error) > 3.1416) {
    if (yaw_error > 0.0) {
      yaw_error = yaw_error - 2.0 * 3.1416;
    } else {
      yaw_error = yaw_error + 2.0 * 3.1416;
    }
  }

  // double yaw_rate_cmd = K_yaw_ * yaw_error + 0.25*(uCommand(0,3) - dpsi_);  // feed-forward yaw_rate cmd
  double yaw_rate_cmd = K_yaw_ * yaw_error + uCommand(0,3);  // feed-forward yaw_rate cmd

  if (yaw_rate_cmd > yaw_rate_limit_) {
    yaw_rate_cmd = yaw_rate_limit_;
  }

  if (yaw_rate_cmd < -yaw_rate_limit_) {
    yaw_rate_cmd = -yaw_rate_limit_;
  }

  // Adding integrator ---------------------------------------------------------------------------------------------------------------------
  bool enable_integrator_ = true;
  double antiwindup_ball_ = 0.4;
  double sampling_time_ = 0.01;
  double position_error_integration_limit_ = 2;
  double Ki_xy_ = 0.3;
  double Ki_altitude_ = 3;

  Eigen::Vector3d estimated_disturbances = Eigen::Vector3d::Zero(3);
  Eigen::Vector3d position_error = Eigen::Vector3d::Zero(3);
  Eigen::MatrixXd error; //has to be Matrix because x0 is matrix and we cannot do the soustraction otherwise

  if (enable_integrator_) {
    error = xd.topRows(1) - x0;
    position_error << error(3),error(5),error(7);
    if (position_error.norm() < antiwindup_ball_) {
      position_error_integration_ += position_error * sampling_time_;
    } else {
      position_error_integration_.setZero();
    }

    position_error_integration_ = position_error_integration_.cwiseMax(
        Eigen::Vector3d(-position_error_integration_limit_, -position_error_integration_limit_,
                        -position_error_integration_limit_));

    position_error_integration_ = position_error_integration_.cwiseMin(
        Eigen::Vector3d(position_error_integration_limit_, position_error_integration_limit_,
                        position_error_integration_limit_));

    estimated_disturbances += Eigen::Vector3d(Ki_xy_, Ki_xy_, Ki_altitude_).asDiagonal()
        * position_error_integration_;

  }
  //--------------------------------------------------------------------------------------------------------------------------

  // Roll Pitch Yawrate Thrust command
  mav_msgs::RollPitchYawrateThrust commandRollPitchYawrateThrust;
  // uCommand(0,4) = std::max(-3.1416/5, std::min(uCommand(0,4), 3.1416/5));
  ros::Time time = ros::Time::now();

  //Time test //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   // ros::WallTime time_wall = ros::WallTime::now();
   //
   // auto now = std::chrono::high_resolution_clock::now();
   // double sysTime = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count()/1e9;
   //
   // int time_secs = sysTime;
   // int time_nanosecs = floor((sysTime-time_secs)*1e9);
   //
   // double diff_time_ros_machine = time_wall.toSec() - sysTime;
   // std::cout<<  std::setprecision (15) << " ROS " << time_wall.toSec() << " Machine " << sysTime << " diff ROS - Machine " << diff_time_ros_machine << std::endl;
   // std::cout<<  std::setprecision (15) << " Machine " << time_secs << " nano:  " << time_nanosecs << std::endl;
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

   // double diff_time_ros_machine2 = time.toSec() - odo_time.toSec();
   // std::cout<<  std::setprecision (15) << " lag odometry : " << diff_time_ros_machine2 << " command "  << time.toSec() << " odo : "  << odo_time.toSec() << std::endl;

  //Safety positions limits
  if (x_limit_inf_ > x_ || x_ > x_limit_sup_  || y_limit_inf_ > y_ || y_ > y_limit_sup_ || z_limit_inf_ > z_ || z_ > z_limit_sup_
      && position_limit_on_ || entered_pos_limit_) {
    std::cout << "---------------- Safety position limit reached ----------------" << std::endl;
    std::cout << "x_: " << x_ <<  " x_limit_inf_: " << x_limit_inf_ << " x_limit_sup_: " << x_limit_sup_  << std::endl;
    std::cout << "y_: " << y_ <<  " y_limit_inf_: " << y_limit_inf_ << " x_limit_sup_: " << y_limit_sup_  << std::endl;
    std::cout << "z_: " << z_ <<  " z_limit_inf_: " << z_limit_inf_ << " z_limit_sup_: " << z_limit_sup_  << std::endl;
    std::cout << "---------------- ----------------------------- ----------------" << std::endl;
    uCommand.setZero();
    yaw_rate_cmd = 0;
    entered_pos_limit_ = true;
  }

  commandRollPitchYawrateThrust.header.stamp = time;
  commandRollPitchYawrateThrust.roll = uCommand(0,1) - estimated_disturbances(1) ;//+ 1/9.81*(-cos(phi_)*sin(psi_)*ddx_ff(0) + cos(phi_)*cos(psi_)*ddy_ff(0) + sin(phi_)*ddz_ff(0));
  commandRollPitchYawrateThrust.pitch = uCommand(0,2) ;
  commandRollPitchYawrateThrust.yaw_rate = yaw_rate_cmd;
  commandRollPitchYawrateThrust.thrust.x = 0;
  commandRollPitchYawrateThrust.thrust.y = 0;
  ddz_ff(0)=0;
  //estimated_disturbances(2)=0;
  commandRollPitchYawrateThrust.thrust.z = (uCommand(0,0)+ddz_ff(0)+estimated_disturbances(2))/cos(uCommand(0,4));
  //std::cout << "v1: " << uCommand(0,0) << " thrust: " << (uCommand(0,0)+ddz_ff(0)+estimated_disturbances(2))/cos(tilt_angle_) << " ddz_ff(0): " << ddz_ff(0) << " estimated_disturbances(2): " << estimated_disturbances(2)<< " cos(tilt_angle_): " << cos(tilt_angle_)<< std::endl;

  std_msgs::Float64 U5;
  U5.data = uCommand(0,4);

  commandFrontAngle_pub_.publish(U5);
  commandBackAngle_pub_.publish(U5);
  commandRollPitchYawrateThrust_pub_.publish(commandRollPitchYawrateThrust);
  ros::WallTime time2 = ros::WallTime::now();
  double diff_time_ros_machine = time2.toSec() - time1.toSec();
  //std::cout<<  std::setprecision (15) << " lag : " << diff_time_ros_machine << " avant "  << time1.toSec() << " apres : "  << time2.toSec() << std::endl;
  //std::cout << "time: " << float( clock () - begin_time_ ) /  CLOCKS_PER_SEC << std::endl;
}

void IterativeLinearQuadraticRegulatorNode::odoCallback(const nav_msgs::Odometry::ConstPtr& odometry)
{
    //get states
    x_ = odometry->pose.pose.position.x;
    y_ = odometry->pose.pose.position.y;
    z_ = odometry->pose.pose.position.z;
    //geometry_msgs::Quaternion quat = odometry->pose.pose.orientation;
    //tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    //tf::Matrix3x3 m(q);
    //m.getRPY(phi_, theta_, psi_);

    dx_ = odometry->twist.twist.linear.x;
    dy_ = odometry->twist.twist.linear.y;
    dz_ = odometry->twist.twist.linear.z;
    //dphi_ = odometry->twist.twist.angular.x;
    //dtheta_ = odometry->twist.twist.angular.y;
    //dpsi_ = odometry->twist.twist.angular.z;

    //IterativeLinearQuadraticRegulatorNode::commandRollPitchYawrateThrust(odometry->header.stamp);
    //std::cout << "time: " << float( clock () - begin_time_ ) /  CLOCKS_PER_SEC << std::endl;
}

void IterativeLinearQuadraticRegulatorNode::odoCallback_angular_rate(const nav_msgs::Odometry::ConstPtr& odometry)
{
    //get states
    geometry_msgs::Quaternion quat = odometry->pose.pose.orientation;
    tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf::Matrix3x3 m(q);
    m.getRPY(phi_, theta_, psi_);
    dphi_ = odometry->twist.twist.angular.x;
    dtheta_ = odometry->twist.twist.angular.y;
    dpsi_ = odometry->twist.twist.angular.z;
    IterativeLinearQuadraticRegulatorNode::commandRollPitchYawrateThrust(odometry->header.stamp);
}

void IterativeLinearQuadraticRegulatorNode::imuCallback(const sensor_msgs::Imu::ConstPtr& imu)
{
  sensor_msgs::Imu imu_raw;
  //Put everything except the orientation in the raw
  imu_raw.header=imu->header;
  imu_raw.orientation_covariance=imu->orientation_covariance;
  imu_raw.angular_velocity=imu->angular_velocity;
  imu_raw.angular_velocity_covariance=imu->angular_velocity_covariance;
  imu_raw.linear_acceleration=imu->linear_acceleration;
  imu_raw.linear_acceleration_covariance=imu->linear_acceleration_covariance;
  rawImu_pub_.publish(imu_raw);
}
/*
void IterativeLinearQuadraticRegulatorNode::tiltCallback(const sensor_msgs::JointState::ConstPtr& tilt)
{
  if (!tilt->position.empty())
  {
    tilt_angle_=tilt->position[0];
  }
}
*/
void IterativeLinearQuadraticRegulatorNode::DynConfigCallback(mav_ilqr_control::iLQRControllerConfig &config,
                                                  uint32_t level)
{
  K_yaw_ = config.K_yaw;
  yaw_rate_limit_ = config.yaw_rate_limit;

  speed_limit_on_ = config.speed_limit_on;
  vx_limit_ = config.vx_limit;
  vy_limit_ = config.vy_limit;
  vz_limit_ = config.vz_limit;

  position_limit_on_ = config.position_limit_on;
  x_limit_inf_ = config.x_limit_inf;
  x_limit_sup_ = config.x_limit_sup;
  y_limit_inf_ = config.y_limit_inf;
  y_limit_sup_ = config.y_limit_sup;
  z_limit_inf_ = config.z_limit_inf;
  z_limit_sup_ = config.z_limit_sup;

  iLQR_.SetPDParameters(config.phi_gain, config.theta_gain, config.psi_gain, config.x_gain,
												 config.vx_gain, config.y_gain, config.vy_gain, config.z_gain, config.dz_gain,
											 config.thrust_gain, config.roll_ref_gain, config.pitch_ref_gain, config.yaw_rate_command_gain, config.alpha_gain);
}

void IterativeLinearQuadraticRegulatorNode::printTrajectoryPoint(double const& psi_des,
  double const& x,
  double const& dx,
  double const& y,
  double const& dy,
  double const& z,
  double const& dz)
{
  nav_msgs::Odometry odoMsg;
  odoMsg.header.stamp = ros::Time::now();
  odoMsg.pose.pose.position.x = x;
  odoMsg.pose.pose.position.y = y;
  odoMsg.pose.pose.position.z = z;
  odoMsg.pose.pose.orientation.x = 0;
  odoMsg.pose.pose.orientation.y = 0;
  odoMsg.pose.pose.orientation.z = 0;
  odoMsg.pose.pose.orientation.w = 1;
  odoMsg.twist.twist.linear.x = dx;
  odoMsg.twist.twist.linear.y = dy;
  odoMsg.twist.twist.linear.z = dz;

  traj_point_pub_.publish(odoMsg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mav_ilqr_control");

	ros::NodeHandle nh, private_nh("~");
  IterativeLinearQuadraticRegulatorNode iLQR(nh, private_nh);

  ros::spin();

  return 0;
}

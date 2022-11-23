/*
 * Copyright 2019 George Popescu, ASL, Polytechnic School, Montreal
 * Copyright 2019 Radu Popescu, ASL, Polytechnic School, Montreal
 *
 * This code can be used only for academic, non-commercial use.
 * This code cannot be redistributed under any license, open source or otherwise.
 *
 *
 */

/*
 * Implementation of the trajectory generation part of the paper Minimum Snap Trajectory Generation
 * and Control for Quadrotors by Daniel Mellinger and Vijay Kumar . With inspiration from
 * Adam P. Bry's PhD thesis and a paper by Richter and al. on Polynomial Trajectory Planning

 * Arguments:
 * wps -- Number of waypoints (not including initial conditions)
 * t -- time vector, starts with 0
 * constraints -- array of position, velocity, acc, jerk and snap constraints for every state

 * Returns:
 * trajectory -- a array of t, x_des, dx_des, ddx_des, dddx_des, ddddx_des,
 * y_des, dy_des, ddy_des, dddy_des, ddddy_des, z_des, dz_des, ddz_des, dddz_des,
 * ddddz_des, psi_des, dpsi_des,ddpsi_des, dddpsi_des, ddddpsi_des
 * for each discretization point.
 */

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Eigen>
#include <cmath>
#include <ctime>
#include <time.h>

#ifndef INCLUDE_MAV_TRAJ_GEN_H_
#define INCLUDE_MAV_TRAJ_GEN_H_

  class TrajectoryGenerator
  {
   public:
    TrajectoryGenerator(int wps, Eigen::VectorXd t, double dt, Eigen::MatrixXd constraints_x, Eigen::MatrixXd constraints_y, Eigen::MatrixXd constraints_z, Eigen::MatrixXd constraints_psi);
    ~TrajectoryGenerator();

    Eigen::VectorXd solve_kkt_onestate(Eigen::MatrixXd const& constraints);
    Eigen::MatrixXd buildHessien();
    static void rotateMatrix90deg(Eigen::MatrixXd& mat);
    void buildConstraints(Eigen::MatrixXd const& constraints, Eigen::MatrixXd &Aeq, Eigen::MatrixXd &beq);
    Eigen::MatrixXd getCoefficientMatrix();
    static void derivativePolynomial(Eigen::VectorXd& polynomial);
    Eigen::MatrixXd discretize_trajectory(Eigen::MatrixXd const& coeffs);
    static double evaluatePolynomial(Eigen::VectorXd const& polynomial, double const& x);
    Eigen::MatrixXd getFullTraj() const;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
   private:
     int n_; //Order of the polynomials of a trajectory
     int r_; // Order of derivative of the states
     int states_; // x,y,z,psi states
     int wps_; //number of waypoints in trajectory excluding the initial position
     Eigen::VectorXd t_; //total time to complete the trajectory
     double dt_; //discretization time
     Eigen::MatrixXd const constraints_x_; //2d array with the constaints for each states (r_+1)x(wps_)x(1)
     Eigen::MatrixXd const constraints_y_; //2d array with the constaints for each states (r_+1)x(wps_)x(1)
     Eigen::MatrixXd const constraints_z_; //2d array with the constaints for each states (r_+1)x(wps_)x(1)
     Eigen::MatrixXd const constraints_psi_; //2d array with the constaints for each states (r_+1)x(wps_)x(1)
     double solutionError_; //error on kkt resolution
     double alpha_; //Constant to rescale the trajectory
     Eigen::VectorXd tScaled_; //Constant to rescale the trajectory
     double dtScaled_; //Constant to rescale the trajectory
     Eigen::MatrixXd fullTrajTime_; //(t_(end)/dt+1)x((r_+1)*states_+1) ex. 201x21
     Eigen::VectorXd time_; //t_(end)/dt+1

  };

#endif /* INCLUDE_MAV_TRAJ_GEN_H_ */

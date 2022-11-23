#ifndef INCLUDE_ITERATIVE_LINEAR_QUADRATIC_REGULATOR_H_
#define INCLUDE_ITERATIVE_LINEAR_QUADRATIC_REGULATOR_H_

#include <math.h>
#include <algorithm>
#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <ros/ros.h>

class IterativeLinearQuadraticRegulator
{
	public:
		IterativeLinearQuadraticRegulator(const ros::NodeHandle& nh,
			                                const ros::NodeHandle private_nh,
																			double dt = 0.01,
																			int tN = 50,
																			int max_iter = 1);
		~IterativeLinearQuadraticRegulator();

		void getControl(Eigen::MatrixXd const& x0,
			Eigen::MatrixXd const& xd,
			Eigen::MatrixXd& xCommand,
			Eigen::MatrixXd& uCommand,
			double& cost,
			bool const& maintainPosition);

		void SetPDParameters(double phi_gain, double theta_gain, double psi_gain, double x_gain,
		                       double vx_gain, double y_gain, double vy_gain, double z_gain, double dz_gain,
												 double thrust_gain, double roll_ref_gain, double pitch_ref_gain, double yaw_rate_command_gain, double alpha_gain)
		 {
		   phi_gain_ = phi_gain;
		   theta_gain_ = theta_gain;
		   psi_gain_ = psi_gain;
		   x_gain_ = x_gain;
		   vx_gain_ = vx_gain;
		   y_gain_ = y_gain;
		   vy_gain_ = vy_gain;
			 z_gain_ = z_gain;
			 dz_gain_ = dz_gain;

			 thrust_gain_ = thrust_gain;
			 roll_ref_gain_ = roll_ref_gain;
			 pitch_ref_gain_ = pitch_ref_gain;
			 yaw_rate_command_gain_ = yaw_rate_command_gain;
			 alpha_gain_ = alpha_gain;
		 }

	private:
		double m_dt;
		int m_tN;
		int m_max_iter;
		ros::NodeHandle nh_;
		ros::NodeHandle private_nh_;
		static constexpr int n_states = 9;
		static constexpr int n_actions = 5;
		Eigen::MatrixXd Q_matrix;
		Eigen::MatrixXd Q_matrix_f;
		Eigen::MatrixXd R_matrix;
		Eigen::MatrixXd Q_terminal_matrix;

		static constexpr double eps_converge = 0.001;
		static constexpr double lamb_factor = 10;
		static constexpr double lamb_max = 10000;
		static constexpr double g = 9.81;
		static constexpr double m = 1.255;
		static constexpr double Ixx = 0.004;//3.676e-03;
		static constexpr double Iyy = 0.008;//7.031e-03;
		static constexpr double Izz = 0.01;//3.650e-03;
		static constexpr double PI  = 3.141592653589793238463;

		//Eigen::MatrixXd U;

		static constexpr double roll_tau = 0.225502;
		static constexpr double roll_gain = 1.145657;
		static constexpr double pitch_tau = 0.1666387;
		static constexpr double pitch_gain = 1.103003;

		double phi_gain_;
		double theta_gain_;
		double psi_gain_;
		double x_gain_;
		double vx_gain_;
		double y_gain_;
		double vy_gain_;
		double z_gain_;
		double dz_gain_;

		double thrust_gain_;
		double roll_ref_gain_;
		double pitch_ref_gain_;
		double yaw_rate_command_gain_;
		double alpha_gain_;

		void calculateCost2(Eigen::Matrix<double, 1, n_states> const& x,
			Eigen::Matrix<double, 1, n_actions> const& u,
			Eigen::Matrix<double, 1, n_states> const& xd,
			Eigen::MatrixXd& l,
			Eigen::MatrixXd& lx,
			Eigen::MatrixXd& lu,
			Eigen::MatrixXd& lxx,
			Eigen::MatrixXd& luu,
			Eigen::MatrixXd& lux);

		Eigen::MatrixXd calculateCost(Eigen::Matrix<double, 1, n_states> const& x,
				Eigen::Matrix<double, 1, n_actions> const& u,
				Eigen::Matrix<double, 1, n_states> const& xd);

		void calculateFinalCost(Eigen::Matrix<double, 1, n_states> const& x,
					Eigen::Matrix<double, 1, n_states> const& xd,
					Eigen::MatrixXd& l,
					Eigen::MatrixXd& lx,
					Eigen::MatrixXd& lxx);

		Eigen::MatrixXd calculateFinalCost(Eigen::Matrix<double, 1, n_states> const& x,
					Eigen::Matrix<double, 1, n_states> const& xd);

		void generateRollout(Eigen::MatrixXd const& x0,
				Eigen::MatrixXd const& U,
				Eigen::MatrixXd& X,
				double& cost,
				Eigen::MatrixXd const& xd);

		Eigen::MatrixXd stepDynamics(Eigen::MatrixXd const& Xt,
					Eigen::MatrixXd const& Ut);

		void getDerivatives(Eigen::Matrix<double, 1, n_states> const& x,
						Eigen::Matrix<double, 1, n_actions> const& u,
						Eigen::MatrixXd& A,
						Eigen::MatrixXd& B);

		void updateParams();
		void updateParamsPosition();

};

#endif

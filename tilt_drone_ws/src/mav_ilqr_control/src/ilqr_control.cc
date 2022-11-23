//#include "ilqr_control.h"
#include <mav_ilqr_control/ilqr_control.h>

// constants
constexpr int IterativeLinearQuadraticRegulator::n_states;
constexpr int IterativeLinearQuadraticRegulator::n_actions;
constexpr double IterativeLinearQuadraticRegulator::eps_converge;
constexpr double IterativeLinearQuadraticRegulator::lamb_factor;
constexpr double IterativeLinearQuadraticRegulator::lamb_max;
constexpr double IterativeLinearQuadraticRegulator::g;
constexpr double IterativeLinearQuadraticRegulator::m;
constexpr double IterativeLinearQuadraticRegulator::Ixx;
constexpr double IterativeLinearQuadraticRegulator::Iyy;
constexpr double IterativeLinearQuadraticRegulator::Izz;
constexpr double IterativeLinearQuadraticRegulator::roll_tau;
constexpr double IterativeLinearQuadraticRegulator::roll_gain;
constexpr double IterativeLinearQuadraticRegulator::pitch_tau;
constexpr double IterativeLinearQuadraticRegulator::pitch_gain;
constexpr double IterativeLinearQuadraticRegulator::PI;

IterativeLinearQuadraticRegulator::IterativeLinearQuadraticRegulator(const ros::NodeHandle& nh,
  const ros::NodeHandle private_nh,
	double dt,
	int tN,
	int max_iter) :
	m_dt(dt),
	m_tN(tN),
	m_max_iter(max_iter),
	nh_(nh),
  private_nh_(private_nh)
{

	Eigen::VectorXd Q_vector(n_states);

	Q_vector(0) = 1;//phi_gain_;//10; 		//phi
	Q_vector(1) = 1;//theta_gain_;//10;		//theta
	Q_vector(2) = 1;//psi_gain_;//10;	//psi
	Q_vector(3) = 1;//x_gain_;//1;		//x
	Q_vector(4) = 1;//vx_gain_;//1;		//vx
	Q_vector(5) = 1;//y_gain_;//1; 	//y
	Q_vector(6) = 1;//vy_gain_;//1;		//vy
	Q_vector(7) = 1;//z_gain_;//10;	//z
	Q_vector(8) = 1;//dz_gain_;//10;	//vz

	Q_matrix = Q_vector.asDiagonal();

	Q_matrix_f = Q_matrix;//*10;

	Eigen::VectorXd R_vector(n_actions);
	R_vector(0) = 1;//thrust_gain_;//0.1; 		//thrust
	R_vector(1) = 1;//roll_ref_gain_;//5; 		//roll_ref
	R_vector(2) = 1;//pitch_ref_gain_;//10; 		//pitch_ref
	R_vector(3) = 1;//yaw_rate_command_gain_;//10; 		//yaw_rate_command
	R_vector(4) = 1;//alpha_gain_;//1; 		//alpha
	R_matrix = R_vector.asDiagonal();
	// U.resize(m_tN, n_actions);
	// U.setZero();
  //
	// for (int i=0;i<U.rows();i++)
	// {
	// 	U(i,0) = g*m;
	// }

}

IterativeLinearQuadraticRegulator::~IterativeLinearQuadraticRegulator()
{

}

Eigen::MatrixXd IterativeLinearQuadraticRegulator::stepDynamics(Eigen::MatrixXd const& Xt,
	Eigen::MatrixXd const& Ut)
{

	Eigen::MatrixXd xNext(1,n_states);
	xNext(0) = 1/roll_tau*(roll_gain*Ut(1)-Xt(0))*m_dt;
	xNext(1) = 1/pitch_tau*(pitch_gain*Ut(2)-Xt(1))*m_dt;
	xNext(2) = Ut(3);
	xNext(3) = Xt(4)*m_dt;
	xNext(4) = Ut(0)/m*(cos(Xt(1))*sin(Xt(0))*sin(Xt(2)) + cos(Xt(2))*sin(Xt(1)) + cos(Xt(1))*cos(Xt(2))*tan(Ut(4)))*m_dt;
	xNext(5) = Xt(6)*m_dt;
	xNext(6) = Ut(0)/m*(-cos(Xt(1))*sin(Xt(0))*cos(Xt(2)) + sin(Xt(2))*sin(Xt(1)) + cos(Xt(1))*sin(Xt(2))*tan(Ut(4)))*m_dt;
	xNext(7) = Xt(8)*m_dt;
	xNext(8) = (-g + Ut(0)/m*(cos(Xt(0))*cos(Xt(1)) - sin(Xt(1))*tan(Ut(4))))*m_dt;
	xNext += Xt;

	return xNext;
}

void IterativeLinearQuadraticRegulator::generateRollout(Eigen::MatrixXd const& x0,
	Eigen::MatrixXd const& U,
	Eigen::MatrixXd& X,
	double& cost,
	Eigen::MatrixXd const& xd)
{
	// set first elements as x0
	X.row(0) = x0;

	for (int t = 0; t < m_tN-1; t++)
	{
		X.row(t+1) = IterativeLinearQuadraticRegulator::stepDynamics(X.row(t), U.row(t));

		Eigen::MatrixXd l = IterativeLinearQuadraticRegulator::calculateCost(X.row(t), U.row(t), xd.row(t));

		cost += m_dt*l(0);
	}

	// compute final cost
	Eigen::MatrixXd l_f = IterativeLinearQuadraticRegulator::calculateFinalCost(X.bottomRows(1), xd.bottomRows(1));

	cost += l_f(0);
}

Eigen::MatrixXd IterativeLinearQuadraticRegulator::calculateCost(Eigen::Matrix<double, 1, n_states> const& x,
	Eigen::Matrix<double, 1, n_actions> const& u,
	Eigen::Matrix<double, 1, n_states> const& xd)
{
	Eigen::Matrix<double, 1, n_states> xMat;

	xMat = x - xd;

	Eigen::MatrixXd l = 0.5*(xMat*Q_matrix*xMat.transpose() + u*R_matrix*u.transpose());


	return l;
}

Eigen::MatrixXd IterativeLinearQuadraticRegulator::calculateFinalCost(Eigen::Matrix<double, 1, n_states> const& x,
	Eigen::Matrix<double, 1, n_states> const& xd)
{
	Eigen::Matrix<double, 1, n_states> xMat;

	xMat = x - xd;

	Eigen::MatrixXd l = 0.5*(xMat*Q_matrix_f*xMat.transpose());

	return l;
}

void IterativeLinearQuadraticRegulator::calculateFinalCost(Eigen::Matrix<double, 1, n_states> const& x,
	Eigen::Matrix<double, 1, n_states> const& xd,
	Eigen::MatrixXd& l,
	Eigen::MatrixXd& lx,
	Eigen::MatrixXd& lxx)
{
	Eigen::Matrix<double, 1, n_states> xMat;

	xMat = x - xd;

	l = 0.5*(xMat*Q_matrix_f*xMat.transpose());
	lx = xMat*Q_matrix_f;
  lxx = Q_matrix_f;

}

void IterativeLinearQuadraticRegulator::calculateCost2(Eigen::Matrix<double, 1, n_states> const& x,
	Eigen::Matrix<double, 1, n_actions> const& u,
	Eigen::Matrix<double, 1, n_states> const& xd,
	Eigen::MatrixXd& l,
	Eigen::MatrixXd& lx,
	Eigen::MatrixXd& lu,
	Eigen::MatrixXd& lxx,
	Eigen::MatrixXd& luu,
	Eigen::MatrixXd& lux)
{
	Eigen::Matrix<double, 1, n_states> xMat;

	xMat = x - xd;

	l = 0.5*(xMat*Q_matrix*xMat.transpose() + u*R_matrix*u.transpose());
  lx = xMat*Q_matrix;
  lu = u*R_matrix;
  lxx = Q_matrix;
  luu = R_matrix;
	lux.resize(n_actions, n_states);
  lux.setZero();
}

void IterativeLinearQuadraticRegulator::getDerivatives(Eigen::Matrix<double, 1, n_states> const& x,
	Eigen::Matrix<double, 1, n_actions> const& u,
	Eigen::MatrixXd& A,
	Eigen::MatrixXd& B)
{
	A(0,0) = -1/roll_tau;
	B(0,1) = 1/roll_tau*roll_gain;
	A(1,1) = -1/pitch_tau;
	B(1,2) = 1/pitch_tau*pitch_gain;
	B(2,3) = 1;
	A(3,4) = 1;
	A(4,1) = u(0)/m*(-sin(x(1))*sin(x(0))*sin(x(2)) + cos(x(2))*cos(x(1)) - sin(x(1))*cos(x(2))*tan(u(4)));
	A(4,0) = u(0)/m*cos(x(1))*cos(x(0))*sin(x(2));
	A(4,2) = u(0)/m*(cos(x(1))*sin(x(0))*cos(x(2)) - sin(x(2))*sin(x(1)) - cos(x(1))*sin(x(2))*u(0)*tan(u(4)));
	B(4,0) = 1/m*(cos(x(1))*sin(x(0))*sin(x(2)) + cos(x(2))*sin(x(1)) + cos(x(1))*cos(x(2))*tan(u(4)));
	B(4,4) = u(0)/m*cos(x(1))*cos(x(2))*(1+tan(u(4))*tan(u(4)));
	A(5,6) = 1;
	A(6,1) = u(0)/m*(sin(x(1))*cos(x(2))*sin(x(0)) + sin(x(2))*cos(x(1)) - sin(x(1))*sin(x(2))*u(0)*tan(u(4)));
	A(6,2) = u(0)/m*(cos(x(1))*sin(x(2))*sin(x(0)) + cos(x(2))*sin(x(1)) + cos(x(1))*cos(x(2))*u(0)*tan(u(4)));
	A(6,0) = u(0)/m*(-cos(x(1))*cos(x(2))*cos(x(0)));
	B(6,0) = 1/m*(-cos(x(1))*cos(x(2))*sin(x(0)) + sin(x(2))*sin(x(1)) + cos(x(1))*sin(x(2))*tan(u(4)));
	B(6,4) = u(0)/m*cos(x(1))*sin(x(2))*(1+tan(u(4))*tan(u(4)));
	A(7,8) = 1;
	A(8,0) = u(0)/m*(-sin(x(0))*cos(x(1)));
	A(8,1) = u(0)/m*(-cos(x(0))*sin(x(1)) - cos(x(1))*u(0)*tan(u(4)));
	B(8,0) = 1/m*(cos(x(0))*cos(x(1)) - sin(x(1))*tan(u(4)));
	B(8,4) = -u(0)/m*sin(x(1))*(1+tan(u(4))*tan(u(4)));

}

void IterativeLinearQuadraticRegulator::getControl(Eigen::MatrixXd const& x0,
	Eigen::MatrixXd const& xd,
	Eigen::MatrixXd& xCommand,
	Eigen::MatrixXd& uCommand,
	double& cost,
  bool const& maintainPosition)
{
  if (maintainPosition){
    updateParamsPosition();
  }
  else{
    updateParams(); //updating params for dynamic_reconfigure
  }

	double lamb = 1.0;
	double alpha = 1.0;
	bool simulateTrajectory(true);

	Eigen::MatrixXd X = Eigen::MatrixXd::Zero(m_tN, n_states);

  Eigen::MatrixXd U;
  U.resize(m_tN, n_actions);
  U.setZero();

  for (int i=0;i<U.rows();i++)
  {
    U(i,0) = g*m;
  }
  uCommand = U;
	double old_cost;

	// derivatives storage
	std::vector<Eigen::Matrix<double, n_states, n_states>> Fx(m_tN);
	std::vector<Eigen::Matrix<double, n_states, n_actions>> Fu(m_tN);

	// cost storage
	Eigen::MatrixXd l = Eigen::MatrixXd::Zero(m_tN, 1);
	Eigen::MatrixXd lx = Eigen::MatrixXd::Zero(m_tN, n_states);
	std::vector<Eigen::Matrix<double, n_states, n_states>> lxx(m_tN);
	Eigen::MatrixXd lu = Eigen::MatrixXd::Zero(m_tN, n_actions);
	std::vector<Eigen::Matrix<double, n_actions, n_actions>> luu(m_tN);
	std::vector<Eigen::Matrix<double, n_actions, n_states>> lux(m_tN);

	for (int iter = 0; iter < m_max_iter; iter++)
	{

		if (simulateTrajectory == true)
		{
			// forward pass
			cost = 0;

			IterativeLinearQuadraticRegulator::generateRollout(x0, U, X, cost, xd);
			old_cost = cost;

			for (int t = 0; t < m_tN-1; t++)
			{
				Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n_states, n_states);
				Eigen::MatrixXd B = Eigen::MatrixXd::Zero(n_states, n_actions);

				IterativeLinearQuadraticRegulator::getDerivatives(X.row(t), U.row(t), A, B);

				Fx[t] = Eigen::Matrix<double, n_states, n_states>::Identity() + A*m_dt;
				Fu[t] = B*m_dt;

				Eigen::MatrixXd l_op = l.row(t);
				Eigen::MatrixXd lx_op = lx.row(t);
				Eigen::MatrixXd lu_op = lu.row(t);
				Eigen::MatrixXd lxx_op = lxx[t];
				Eigen::MatrixXd luu_op = luu[t];
				Eigen::MatrixXd lux_op = lux[t];

				IterativeLinearQuadraticRegulator::calculateCost2(X.row(t), U.row(t),
				xd.row(t), l_op, lx_op, lu_op, lxx_op, luu_op, lux_op);

				l.row(t) = l_op*m_dt;
				lx.row(t) = lx_op*m_dt;
				lu.row(t) = lu_op*m_dt;
				lxx[t] = lxx_op*m_dt;
				luu[t] = luu_op*m_dt;
				lux[t] = lux_op*m_dt;

			}

			Eigen::MatrixXd l_op = l.bottomRows(1);
			Eigen::MatrixXd lx_op = lx.bottomRows(1);
			Eigen::MatrixXd lxx_op = lxx[m_tN-1];

			IterativeLinearQuadraticRegulator::calculateFinalCost(X.bottomRows(1),
			xd.bottomRows(1), l_op, lx_op, lxx_op);

			l.bottomRows(1) = l_op;
			lx.bottomRows(1) = lx_op;
			lxx[m_tN-1] = lxx_op;

			simulateTrajectory = false;

		}

		Eigen::MatrixXd V = l.bottomRows(1); //state value function
		Eigen::MatrixXd Vx = lx.bottomRows(1);
		Eigen::MatrixXd Vxx = lxx[m_tN-1];

		Eigen::MatrixXd k = Eigen::MatrixXd::Zero(m_tN, n_actions); // feedforward
		std::vector<Eigen::Matrix<double, n_actions, n_states>> K(m_tN); // feedback gain

		// backward pass
		for (int t = m_tN-2; t >= 0; t-- )
		{

			Eigen::MatrixXd Qx = lx.row(t).transpose() + Fx[t].transpose()*Vx.transpose();
			Eigen::MatrixXd Qu = lu.row(t).transpose() + Fu[t].transpose()*Vx.transpose();
			Eigen::MatrixXd Qxx = lxx[t] + Fx[t].transpose()*Vxx*Fx[t];
			Eigen::MatrixXd Qux = lux[t] + Fu[t].transpose()*Vxx*Fx[t];
			Eigen::MatrixXd Quu = luu[t] + Fu[t].transpose()*Vxx*Fu[t];

			Eigen::MatrixXd Quu_inv = Quu.inverse();

			k.row(t) = -(Quu_inv*Qu).transpose();
			K[t] = -Quu_inv*Qux;


			Vx = (Qx - K[t].transpose()*Quu*k.row(t).transpose()).transpose();
			Vxx = (Qxx - K[t].transpose()*Quu*K[t]).transpose();

		}

		Eigen::MatrixXd Unew = Eigen::MatrixXd::Zero(m_tN, n_actions);

		Eigen::MatrixXd xnew = x0;

		for (int t = 0; t < m_tN-1; t++)
		{
			// std::cout<<"ici_Unew_row"<<k.row(t)<<std::endl;
			Unew.row(t) = U.row(t) + alpha*k.row(t) + (K[t]*(xnew-X.row(t)).transpose()).transpose();

			Unew(t,0) = std::max(0.0, std::min(Unew(t,0), m*g*4));
      Unew(t,1) = std::max(-PI/2, std::min(Unew(t,1), PI/2));
      Unew(t,2) = std::max(-PI/20, std::min(Unew(t,2), PI/20));
      Unew(t,3) = std::max(-PI/3, std::min(Unew(t,3), PI/3));
			Unew(t,4) = std::max(-PI/4, std::min(Unew(t,4), PI/4));

			xnew = IterativeLinearQuadraticRegulator::stepDynamics(xnew, Unew.row(t));
		}

		Eigen::MatrixXd Xnew = Eigen::MatrixXd::Zero(m_tN, n_states);

		double costnew(0);

		IterativeLinearQuadraticRegulator::generateRollout(x0, Unew, Xnew, costnew, xd);

		if (costnew < cost)
		{
			lamb/=lamb_factor;
			X = Xnew;
			U = Unew;
			uCommand = Unew; // for return
			xCommand = Xnew;
			old_cost = cost;
			cost = costnew;
			simulateTrajectory = true; // do another rollout
			if ((iter > 0) && (fabs((old_cost-cost)/cost) < eps_converge))
			{
				break;
			}
		}
		else
		{
			lamb *= lamb_factor;
			if (lamb > lamb_max)
			{
				break;
			}
		}
//std::cout << "iter: " << iter << std::endl;
	} // end for loop iterations

}

void IterativeLinearQuadraticRegulator::updateParams()
{

        /*
	Q_matrix(0,0) = phi_gain_;//10; 		//phi
	Q_matrix(1,1) = theta_gain_;//10;		//theta
	Q_matrix(2,2) = psi_gain_;//10;	//psi
	Q_matrix(3,3) = x_gain_;//1;		//x
	Q_matrix(4,4) = vx_gain_;//1;		//vx
	Q_matrix(5,5) = y_gain_;//1; 	//y
	Q_matrix(6,6) = vy_gain_;//1;		//vy
	Q_matrix(7,7) = z_gain_;//10;	//z
	Q_matrix(8,8) = dz_gain_;//10;	//vz

	Q_matrix_f = Q_matrix;//*10;

	R_matrix(0,0) = thrust_gain_;//0.1; 		//thrust
	R_matrix(1,1) = roll_ref_gain_;//5; 		//roll_ref
	R_matrix(2,2) = pitch_ref_gain_;//10; 		//pitch_ref
	R_matrix(3,3) = yaw_rate_command_gain_;//10; 		//yaw_rate_command
	R_matrix(4,4) = alpha_gain_;//1; 		//alpha
        */


	Q_matrix(0,0) = 1; 		//phi
	Q_matrix(1,1) = 20;		//theta
	Q_matrix(2,2) = 1;              //psi
	Q_matrix(3,3) = 2.7;		//x
	Q_matrix(4,4) = 0.5;		//vx
	Q_matrix(5,5) = 2.2; 	        //y
	Q_matrix(6,6) = 0.5;		//vy
	Q_matrix(7,7) = 3.5;	        //z
	Q_matrix(8,8) = 0.5;	       //vz

	Q_matrix_f = Q_matrix;

	R_matrix(0,0) = 0.1; 		//thrust
	R_matrix(1,1) = 5;		//roll_ref
	R_matrix(2,2) = 10;		//pitch_ref
	R_matrix(3,3) = 10; 		//yaw_rate_command
	R_matrix(4,4) = 5;  		//alpha

}

void IterativeLinearQuadraticRegulator::updateParamsPosition()
{

	Q_matrix(0,0) = 1;      //phi
	Q_matrix(1,1) = 20;	//theta
	Q_matrix(2,2) = 1;	//psi
	Q_matrix(3,3) = 1;	//x
	Q_matrix(4,4) = 0.5;	//vx
	Q_matrix(5,5) = 2;	//y
	Q_matrix(6,6) = 0.5;	//vy
	Q_matrix(7,7) = 2.5;	//z
	Q_matrix(8,8) = 0.5;	//vz

	Q_matrix_f = Q_matrix;//*10;

	R_matrix(0,0) = 0.1; 	//thrust
	R_matrix(1,1) = 5; 		//roll_ref
	R_matrix(2,2) = 10; 	//pitch_ref
	R_matrix(3,3) = 10; 	//yaw_rate_command
	R_matrix(4,4) = 5;//1;//alpha

}

// test
int main () {
	// std::cout << "Hello World" << std::endl;
	// double dt(0.01);
	// int tN(50);
	// int n_states(9);
	// int n_actions(5);
	//
	// IterativeLinearQuadraticRegulator iLQR(dt, tN);
	//
	// Eigen::MatrixXd x0 = Eigen::MatrixXd::Zero(1, n_states);
	// x0(3) = 1;
	// Eigen::MatrixXd xd = Eigen::MatrixXd::Zero(tN, n_states);
	// for (int t = 0; t < tN; t++)
	// {
	// 	// xd(t,0) = 1.0;
	// 	// xd(t,1) = 1.0;
	// 	// xd(t,2) = 1.0;
	// 	// xd(t,3) = 1.0;
	// 	// xd(t,4) = 1.0;
	// 	// xd(t,5) = 1.0;
	// 	// xd(t,6) = 1.0;
	// 	xd(t,7) = 1.0;
	// 	// xd(t,8) = 1.0;
	// }
	//
	// Eigen::MatrixXd xCommand = Eigen::MatrixXd::Zero(tN, n_states);
	// Eigen::MatrixXd uCommand = Eigen::MatrixXd::Zero(tN, n_actions);
	//
	// double cost(0);
	//
	// iLQR.getControl(x0, xd, xCommand, uCommand, cost);
	//
	// // std::cout << xCommand << std::endl;
	// std::cout << uCommand << std::endl;
	// std::cout << cost << std::endl;

	return 0;
}

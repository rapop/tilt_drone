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

//#include "traj_gen.h" //if in same folder
#include <mav_ilqr_control/traj_gen.h>

TrajectoryGenerator::TrajectoryGenerator(int wps, Eigen::VectorXd t, double dt, Eigen::MatrixXd constraints_x, Eigen::MatrixXd constraints_y, Eigen::MatrixXd constraints_z, Eigen::MatrixXd constraints_psi)
    : wps_(wps),
      t_(t),
      dt_(dt),
      constraints_x_(constraints_x),
      constraints_y_(constraints_y),
      constraints_z_(constraints_z),
      constraints_psi_(constraints_psi)
{
  n_ = 6;
  r_ = 4;
  states_ = 4;
  alpha_ = 1;

  //Initializing coefficients
  Eigen::VectorXd coeffs_x = solve_kkt_onestate(constraints_x_);
  Eigen::VectorXd coeffs_y = solve_kkt_onestate(constraints_y_); //coeffs not exactly the same (mostly y) and errors builds up when discretize but is of ~0.5 on wps 2 on snap
  Eigen::VectorXd coeffs_z = solve_kkt_onestate(constraints_z_);
  Eigen::VectorXd coeffs_psi = solve_kkt_onestate(constraints_psi_);
  Eigen::MatrixXd coeffs(wps_*(n_+1),states_);

  //putting each coeffs_i in a column in coeffs
  coeffs << coeffs_x, coeffs_y, coeffs_z, coeffs_psi;
  //std::cout << "coeffs_x " << coeffs_x << std::endl;

  fullTrajTime_ = discretize_trajectory(coeffs); //(t_(end)/dt+1)x((r_+1)*states_+1) ex. 201x21
}

TrajectoryGenerator::~TrajectoryGenerator()
{
}

Eigen::MatrixXd TrajectoryGenerator::getFullTraj() const
{
  return fullTrajTime_;
}

Eigen::VectorXd TrajectoryGenerator::solve_kkt_onestate(Eigen::MatrixXd const& constraints)
{
  Eigen::MatrixXd hessien = buildHessien(); //14x14
  Eigen::MatrixXd Aeq = Eigen::MatrixXd::Zero(16,14);
  Eigen::MatrixXd beq = Eigen::MatrixXd::Zero(16,1);
  buildConstraints(constraints, Aeq, beq); //Aeq 16x14, beq 16x1 --!-- not always 16, depends if NAN in conditions

  //remove last 2 constraints for psi state
  Aeq.conservativeResize(13, Aeq.cols()); //13x14
  beq.conservativeResize(13, beq.cols()); //13x1

  /*
  * build the KKT conditions
  * We want to solve
  *
  *  [ H A'   [  x*       = [0
  *    A 0 ]   lambda*]      b]
  */
  Eigen::MatrixXd K = Eigen::MatrixXd::Zero(hessien.rows()+Aeq.rows(), hessien.cols()+ Aeq.rows()); //27x27
  K.block(0,0,hessien.rows(),hessien.cols()) << hessien;
  K.block(hessien.rows(),0,Aeq.rows(),Aeq.cols()) << Aeq;
  K.block(0,hessien.cols(),Aeq.cols(),Aeq.rows()) << Aeq.transpose();
  Eigen::MatrixXd nulMatrix=Eigen::MatrixXd::Zero(Aeq.rows(),Aeq.rows());
  K.block(hessien.rows(),Aeq.cols(),nulMatrix.rows(),nulMatrix.cols()) << nulMatrix;
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(hessien.rows()+ beq.rows(), beq.cols());
  Eigen::MatrixXd nulMatrixB=Eigen::MatrixXd::Zero(hessien.rows(), beq.cols());
  B << nulMatrixB, //27x1
       beq;

  Eigen::VectorXd x = K.fullPivLu().solve(B); //there is also partialPivLu() in Eigen library which is faster and gives almost same result
  solutionError_ = (K*x-B).norm() / B.norm(); //relative error for the resolution
  Eigen::VectorXd coeffs = x.head((n_+1)*wps_);

  return coeffs;
}

Eigen::MatrixXd TrajectoryGenerator::discretize_trajectory(Eigen::MatrixXd const& coeffs)
{
  dtScaled_ = dt_*alpha_;
  tScaled_ = t_*alpha_;
  const int& n_coeffs = n_+1;
  Eigen::MatrixXd fullTrajTime = Eigen::MatrixXd::Zero(t_(t_.size()-1)/dt_+1,states_*(r_+1)+1);
  Eigen::MatrixXd polynomial[r_+1][states_]; //size is (r_+1)xstates_xn_coeffsxwps_
  for (int state(1) ; state<states_+1 ; state++)
  {
    Eigen::MatrixXd coeffsCol = coeffs.col(state-1);
    coeffsCol.resize(n_coeffs,wps_); //do not use conservativeResize here
    polynomial[0][state-1] = coeffsCol;
    Eigen::MatrixXd scale_vec(n_coeffs,wps_);
    for (int x(0) ; x<n_coeffs ; x++) {scale_vec(x,0)=pow(1/alpha_,x);}
    Eigen::MatrixXd b = Eigen::MatrixXd::Ones(n_coeffs,wps_-1);
    scale_vec.block(0,1,n_coeffs,wps_-1) << b;
    polynomial[0][state-1].cwiseProduct(scale_vec);

    for (int der(1) ; der<r_+1 ; der++) //to initialize inner matrixs
    {
      Eigen::MatrixXd innerMat= Eigen::MatrixXd::Zero(n_coeffs,wps_);
      polynomial[der][state-1]=innerMat;
    }

    for (int der(1) ; der<r_+1 ; der++)
    {
      for (int wp(1) ; wp<wps_+1 ; wp++)
      {

        Eigen::VectorXd poly_temp = polynomial[der-1][state-1].col(wp-1);
        TrajectoryGenerator::derivativePolynomial(poly_temp);

        Eigen::VectorXd a = Eigen::VectorXd::Zero(n_coeffs-poly_temp.size()+1);
        Eigen::VectorXd front=poly_temp.head(poly_temp.size()-a.size());
        poly_temp.tail(poly_temp.size()-a.size())=front;
        poly_temp.head(a.size())=a;
        polynomial[der][state-1].col(wp-1) = poly_temp;

      } //for on wp
    } //for on der
  } //for on state
  time_ = Eigen::VectorXd::LinSpaced(tScaled_(tScaled_.size()-1)/dtScaled_+1,0,tScaled_(tScaled_.size()-1));
  Eigen::MatrixXd traj[time_.size()]; //size is size(time)x(r_+1)xstates_
  for (int i(0) ; i<time_.size() ; i++) //to initialize inner matrixs
  {
    Eigen::MatrixXd innerMat= Eigen::MatrixXd::Zero(r_+1,states_);
    traj[i]=innerMat;
  }

  for (int state(1) ; state<states_+1 ; state++)
  {
    for (int t2(1) ; t2<time_.size()+1 ; t2++)
    {
      int wp;
      if (time_(t2-1)==0){wp=1;}
      else
      {
        Eigen::Matrix<bool,Eigen::Dynamic,1> bool_mat = t_.array()<time_(t2-1);
        wp=bool_mat.count();
      }

      for (int der(1) ; der<r_+2 ; der++)
      {
        if (wps_==1)
        {
          wp=1;
        }
        double scaledt = (time_(t2-1)-t_(wp-1)) / (t_(wp) - t_(wp-1)) * alpha_;
        Eigen::VectorXd poly_temp = polynomial[der-1][state-1].col(wp-1);
        traj[t2-1](der-1, state-1) = TrajectoryGenerator::evaluatePolynomial(poly_temp,scaledt);
      }

    } //for on t2
  } //for on state

  for (int i(0) ; i<time_.size() ; i++)
  {
    Eigen::VectorXd vecTimeStatesDer=Eigen::VectorXd::Zero(fullTrajTime.cols());
    vecTimeStatesDer(0)=time_(i);
    int count(1);
    for (int state(0) ; state<states_ ; state++)
    {
      for (int der(0) ; der<r_+1 ; der++)
      {
        vecTimeStatesDer(count)=traj[i](der,state); //putting all derivatives then moving to next states
        count++;
      }
    }
    fullTrajTime.row(i)=vecTimeStatesDer;
  }

  return fullTrajTime;
}

Eigen::MatrixXd TrajectoryGenerator::buildHessien()
{
  int num_coeffs(n_+1);

  Eigen::MatrixXd hessien = Eigen::MatrixXd::Zero(wps_*(n_+1),wps_*(n_+1));
  Eigen::MatrixXd hessien_R = Eigen::MatrixXd::Zero(num_coeffs,num_coeffs);

  for (int wp(1) ; wp<wps_+1 ; wp++)
  {
    for (int i(0) ; i<n_+1 ; i++)
    {
      for (int l(0) ; l<n_+1 ; l++)
      {
          if (i >= r_ && l >= r_)
          {
              int cum_mul(1);
              for (int m(0) ; m<r_ ; m++)
              {
                cum_mul = cum_mul * (i-m) * (l-m);
              }
              hessien_R(i,l) = cum_mul;
              hessien_R(i,l) /= (i+l-2*r_+1);
          }
          else
          {
            hessien_R(i,l) = 0;
          }

      } //l loop
    } //i loop


    hessien_R =  1/pow(t_(wp)-t_(wp-1),(2*r_-1)) * hessien_R; // scalar * matrix
    TrajectoryGenerator::rotateMatrix90deg(hessien_R);
    TrajectoryGenerator::rotateMatrix90deg(hessien_R);
    if (wp == 1)
    {
      hessien = hessien_R;
    }
    else
    {
      int size_H(hessien.rows());
      int size_Hr(hessien_R.rows());
      Eigen::MatrixXd hessien_augmented(size_H+size_Hr,size_H+size_Hr);
      hessien_augmented.block(0,0,size_H,size_H) << hessien;
      hessien_augmented.block(size_H,size_H,size_Hr,size_Hr) << hessien_R;
      hessien = hessien_augmented;
    }
    hessien_R.setZero(); //reseting the hessien_R, because it changes in every for loop

  } //for on wps

  return hessien;
}

void TrajectoryGenerator::buildConstraints(Eigen::MatrixXd const& constraints, Eigen::MatrixXd &Aeq, Eigen::MatrixXd &beq)
{
  int n_coeffs(n_+1);
  int n_wps(wps_+1);
  Eigen::MatrixXd coeffs = getCoefficientMatrix();
  Eigen::MatrixXd identityMatrix = Eigen::MatrixXd::Identity(n_coeffs,n_coeffs);
  TrajectoryGenerator::rotateMatrix90deg(identityMatrix);
  Eigen::MatrixXd a = Eigen::MatrixXd::Zero(1,(n_wps-1)*n_coeffs);
  Eigen::MatrixXd a_inelse = Eigen::MatrixXd::Zero(2,(n_wps-1)*n_coeffs);
  Eigen::MatrixXd A_0;
  Eigen::MatrixXd b_0;
  for (int wp(1) ; wp<n_wps+1 ; wp++)
  {
    for (int der(1) ; der<r_+1 ; der++)
    {
      if (!std::isnan(constraints(der-1,wp-1)))
      {
        if (wp == 1)
        {
          double int_t = 1/pow(t_(wp)-t_(wp-1),der-1);
          Eigen::VectorXd polynomial(coeffs.row(der-1).cwiseProduct(identityMatrix.row(der-1)*int_t));
          int idx = (wp-1) * n_coeffs + 1;
          for (int j(idx-1) ; j<idx+n_ ; j++)
          {
            a(0,j)=polynomial(j-idx+1);
          }
          double b = constraints(der-1, wp-1);
          A_0.conservativeResize(A_0.rows()+1, a.cols());
          A_0.row(A_0.rows()-1) = a;
          b_0.conservativeResize(1,b_0.cols()+1);
          b_0(0,b_0.cols()-1) = b;
          a.setZero();
        }
        else if(wp == n_wps)
        {
          double int_t_next = 1/pow(t_(wp-1)-t_(wp-2),der-1);
          Eigen::VectorXd polynomial = coeffs.row(der-1) * int_t_next;
          int idx = (wp-2) * n_coeffs + 1;
          for (int j(idx-1) ; j<idx+n_ ; j++)
          {
            a(0,j)=polynomial(j-idx+1);
          }
          double b = constraints(der-1, wp-1);
          A_0.conservativeResize(A_0.rows()+1, a.cols());
          A_0.row(A_0.rows()-1) = a;
          b_0.conservativeResize(1,b_0.cols()+1);
          b_0(0,b_0.cols()-1) = b;
          a.setZero();
        }
        else
        {
          double int_t_next = 1/pow(t_(wp-1) - t_(wp-2),der-1);
          Eigen::VectorXd polynomial = coeffs.row(der-1) * int_t_next;
          int idx = (wp-2) * n_coeffs + 1;
          for (int j(idx-1) ; j<idx+n_ ; j++)
          {
            a_inelse(0,j)=polynomial(j-idx+1);
          }
          double int_t = 1/pow(t_(wp)-t_(wp-1),der-1);
          Eigen::VectorXd polynomial2(coeffs.row(der-1).cwiseProduct(identityMatrix.row(der-1)*int_t));
          int idx2 = (wp-1) * n_coeffs + 1;
          for (int j(idx2-1) ; j<idx2+n_ ; j++)
          {
            a_inelse(1,j)=polynomial2(j-idx2+1);
          }
          Eigen::Vector2f b=Eigen::Vector2f::Ones(2);
          b *= constraints(der-1, wp-1);
          A_0.conservativeResize(A_0.rows()+2, a_inelse.cols());
          A_0.row(A_0.rows()-2) = a_inelse.row(0);
          A_0.row(A_0.rows()-1) = a_inelse.row(1);
          b_0.conservativeResize(1,b_0.cols()+2);
          b_0(0,b_0.cols()-2) = b(0);
          b_0(0,b_0.cols()-1) = b(1);
          a_inelse.setZero();
        }
      }
    } //der
  } //wp

  Eigen::MatrixXd A_t;
  Eigen::MatrixXd b_t;
  for (int wp(2) ; wp<n_wps ; wp++)
  {
    for (int der(1) ; der<r_+2 ; der++)
    {
      if (std::isnan(constraints(der-1,wp-1)))
      {
        Eigen::MatrixXd a(1,(n_wps-1)*n_coeffs);
        double int_t = 1/pow(t_(wp-1)-t_(wp-2),der-1);
        double int_t_next = 1/pow(t_(wp)-t_(wp-1),der-1);
        Eigen::VectorXd a_prev = coeffs.row(der-1) * int_t;
        int idx_prev = (wp-2) * n_coeffs + 1;
        for (int j(idx_prev-1) ; j<idx_prev+n_ ; j++)
        {
          a(0,j)=a_prev(j-idx_prev+1);
        }
        Eigen::VectorXd a_next(-coeffs.row(der-1).cwiseProduct(identityMatrix.row(der-1)*int_t_next));
        int idx_next = (wp-1) * n_coeffs + 1;
        for (int j(idx_next-1) ; j<idx_next+n_ ; j++)
        {
          a(0,j)=a_next(j-idx_next+1);
        }
        double b=0;
        A_t.conservativeResize(A_t.rows()+1, a.cols());
        A_t.row(A_t.rows()-1) = a;
        b_t.conservativeResize(1,b_t.cols()+1);
        b_t(0,b_t.cols()-1) = b;
      }
    }//der
  }//wp

  b_0.transposeInPlace(); //transform into a column vector
  b_t.transposeInPlace();
  //Redefining Aeq, beq (the purpose of this method)
  // Aeq.conservativeResize(A_0.rows()+A_t.rows(), A_0.cols());
  // Aeq << A_0,
  //        A_t;
  // beq.conservativeResize(b_0.rows()+A_t.rows(), b_0.cols());
  if (wps_>1)
  {

    Aeq.conservativeResize(A_0.rows()+A_t.rows(), A_0.cols());
    Aeq << A_0,
           A_t;
    beq.conservativeResize(b_0.rows()+A_t.rows(), b_0.cols());
    beq << b_0,
           b_t;
  }
  else
  {
    Aeq = A_0;
    beq = b_0;
  }

}

Eigen::MatrixXd TrajectoryGenerator::getCoefficientMatrix()
{
  int num_coeffs(n_+1);
  int num_poly(r_+1);
  Eigen::MatrixXd coefficients = Eigen::MatrixXd::Zero(num_poly,num_coeffs); //(num_poly,num_coeffs)
  Eigen::VectorXd polynomial=Eigen::VectorXd::Ones(num_coeffs);
  for (int i(0) ; i<num_poly ; i++)
  {
    //coefficients.block(i,0,1,num_coeffs-i)=polynomial; not working so need to do a loop
    for (int j(0) ; j<polynomial.size() ; j++)
    {
      coefficients(i,j)=polynomial(j);
    }
    TrajectoryGenerator::derivativePolynomial(polynomial);
    polynomial.conservativeResize(polynomial.size()-1); //removing the last element that is zero
  }

  return coefficients;
}

void TrajectoryGenerator::derivativePolynomial(Eigen::VectorXd& polynomial) //ref is essential to change mat inside this function
{
  //cnx^n + c(n-1)x^(n-1) + c(n-2)x^(n-2) + ... + c(1)x^(1) + c(0)

  for (int i(0) ; i<polynomial.size() ; i++)
  {
    polynomial(i)= (polynomial.size()-i-1)*polynomial(i);
  }
}

double TrajectoryGenerator::evaluatePolynomial(Eigen::VectorXd const& polynomial, double const& x)
{
  /* Horner’s method can be used to evaluate polynomial in O(n) time. To understand the method, let us consider the example of 2x3 – 6x2 + 2x – 1.
  The polynomial can be evaluated as ((2x – 6)x + 2)x – 1. The idea is to initialize result as coefficient of xn which is 2 in this case,
  repeatedly multiply result with x and add next coefficient to result. Finally return result.*/

  //cnx^n + c(n-1)x^(n-1) + c(n-2)x^(n-2) + ... + c(1)x^(1) + c(0)
  double result = polynomial(0);  // Initialize result
  for (int i(1) ; i<polynomial.size() ; i++)
  {
    result = result*x + polynomial(i);
  }

  return result;
}

void TrajectoryGenerator::rotateMatrix90deg(Eigen::MatrixXd& mat) //ref is essential to change mat inside this function
{
  int const N(mat.rows());
	// Consider all squares one by one
	for (int x = 0; x < N / 2; x++)
	{
		// Consider elements in group of N in
		// current square
		for (int y = x; y < N-x-1; y++)
		{
			// store current cell in temp variable
			int temp = mat(x,y); //1

			// move values from right to top
			mat(x,y) = mat(y,N-1-x);

			// move values from bottom to right
			mat(y,N-1-x) = mat(N-1-x,N-1-y);

			// move values from left to bottom
			mat(N-1-x,N-1-y) = mat(N-1-y,x);

			// assign temp to left
			mat(N-1-y,x) = temp;

		}
	}
}

int main()
{
  // int wps(1);
  // Eigen::MatrixXd constraints_x(5,wps+1);
  // constraints_x << 0.5, 3.199-0.25, // position constraints
  //                  0, 2.5, // velocity constraints
  //                  0, 0, // acceleration constraints
  //                  0, NAN, // jerk constraints
  //                  0, NAN; // snap constraints
  // Eigen::MatrixXd constraints_y(5,wps+1);
  // constraints_y << 0.65, 0.15,
  //                  0, 0,
  //                  0, 0,
  //                  0, NAN,
  //                  0, NAN;
  // Eigen::MatrixXd constraints_z(5,wps+1);
  // constraints_z << 1, 1.167-0.06,
  //                  0, 0,
  //                  0, 0,
  //                  0, NAN,
  //                  0, NAN;
  // Eigen::MatrixXd constraints_psi(5,wps+1);
  // constraints_psi << 0, 0,
  //                    0, 0,
  //                    0, 0,
  //                    0, NAN,
  //                    0, NAN;
  // Eigen::VectorXd t(wps+1);
  // t << 0, 0.84;
  // double dt(0.01);
  // const clock_t begin_time = clock();
  // TrajectoryGenerator trajObject(wps, t, dt, constraints_x, constraints_y, constraints_z, constraints_psi);

  // std::cout << "Here is the matrix constraints_x:\n" << constraints_x << std::endl;
  // std::cout << "\nis NAN: " << std::isnan(constraints_x(3,0)) << std::endl;
  // std::cout << "\nt : " << t(1) << std::endl;
  //
  //
  // Eigen::MatrixXd rotationmatrixtotest(2,2);
  // rotationmatrixtotest << 1, 2,
  //                         3, 4;
  // std::cout << "Before rotation:\n" << rotationmatrixtotest << std::endl;
  // TrajectoryGenerator::rotateMatrix90deg(rotationmatrixtotest);
  // std::cout << "After rotation:\n" << rotationmatrixtotest << std::endl;
  //
  // Eigen::VectorXd polynomial(4); //Needs to be Xd and with size defined
  // polynomial << 3,6,2,1;
  // std::cout << "The derivative of :\n" << polynomial << std::endl;
  // TrajectoryGenerator::derivativePolynomial(polynomial);
  // std::cout << "is :\n" << polynomial << std::endl;

  // Eigen::VectorXd polynomial(4); //Needs to be Xd and with size defined
  // polynomial << 2,1,6,4; double x=4.2;
  // std::cout << "The evaluation of : " << polynomial << "at : " << x << std::endl;
  // double result = TrajectoryGenerator::evaluatePolynomial(polynomial, x);
  // std::cout << "is : " << result << std::endl;

  //Eigen::MatrixXd trajArray=trajObject.getFullTraj();
  //std::cout << "time: " << float( clock () - begin_time ) /  CLOCKS_PER_SEC << std::endl;
  //std::cout << "Traj row : " << trajArray.row(50) << std::endl;

  //Adding repeatedly the point for iLQR
  // int const tN=50;
  // Eigen::MatrixXd endConstStates=Eigen::MatrixXd::Zero(tN,trajArray.cols());
  // for (int i(0); i<tN; i++)
  // {
  //   endConstStates(i,0) = trajArray(trajArray.rows()-1,0)+dt*(i+1);
  //   endConstStates.block(i,1,1,endConstStates.cols()-1) = trajArray.block(trajArray.rows()-1,1,1,trajArray.cols()-1);
  // }
  // trajArray.conservativeResize(trajArray.rows()+tN,trajArray.cols());
  // trajArray.bottomRows(tN)=endConstStates;
  //std::cout << "Traj row : " << trajArray.row(130) << std::endl;
}

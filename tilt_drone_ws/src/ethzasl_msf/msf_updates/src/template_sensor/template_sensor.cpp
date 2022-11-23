/*

Copyright (c) 2010, Stephan Weiss, ASL, ETH Zurich, Switzerland
You can contact the author at <stephan dot weiss at ieee dot org>

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
* Neither the name of ETHZ-ASL nor the
names of its contributors may be used to endorse or promote products
derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETHZ-ASL BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "template_sensor.h"
#include <ssf_core/eigen_utils.h>

#define N_MEAS /*N*/ // measurement size

TemplateSensorHandler::TemplateSensorHandler(ssf_core::Measurements* meas) :
  MeasurementHandler(meas)
{
  // read some parameters
  ros::NodeHandle pnh("~");
  pnh.param("measurement_world_sensor", measurement_world_sensor_, true);
  pnh.param("use_fixed_covariance", use_fixed_covariance_, false);

  ROS_INFO_COND(measurement_world_sensor_, "interpreting measurement as sensor w.r.t. world");
  ROS_INFO_COND(!measurement_world_sensor_, "interpreting measurement as world w.r.t. sensor (e.g. ethzasl_ptam)");

  ROS_INFO_COND(use_fixed_covariance_, "using fixed covariance");
  ROS_INFO_COND(!use_fixed_covariance_, "using covariance from sensor");

  subscribe();
}

void TemplateSensorHandler::subscribe()
{
  ros::NodeHandle nh("ssf_core");
  subMeasurement_ = nh.subscribe("template_measurement", 1, &TemplateSensorHandler::measurementCallback, this);

  measurements->ssf_core_.registerCallback(&TemplateSensorHandler::noiseConfig, this);

  nh.param("meas_noise1", n_zp_, 0.01);	// default position noise is for ethzasl_ptam
  nh.param("meas_noise2", n_zq_, 0.02);	// default attitude noise is for ethzasl_ptam
}

void TemplateSensorHandler::noiseConfig(ssf_core::SSF_CoreConfig& config, uint32_t level)
{
  //	if(level & ssf_core::SSF_Core_MISC)
  //	{
  this->n_zp_ = config.meas_noise1;
  this->n_zq_ = config.meas_noise2;
  //	}
}

void TemplateSensorHandler::measurementCallback(const /* measurement message type */ & msg)
{
  // init variables
  ssf_core::State state_old;
  ros::Time time_old = msg->header.stamp;
  Eigen::Matrix<double, N_MEAS, N_STATE> H_old;
  Eigen::Matrix<double, N_MEAS, 1> r_old;
  Eigen::Matrix<double, N_MEAS, N_MEAS> R;

  H_old.setZero();
  R.setZero();

  // get measurements
  z_p_ = /* read sensor information from msg: z_p_ is usually a position */
  z_q_ = /* read sensor information from msg: z_q_ is usually an attitude */

  // take covariance from sensor
  // fill the measurement covariance matrix R with the covariance values provided in msg

  //  alternatively take fix covariance from reconfigure GUI
  if (use_fixed_covariance_)
  {
    const double s_zp = n_zp_ * n_zp_;
    const double s_zq = n_zq_ * n_zq_;
    // fill the measurement covariance matrix R using the standard deviations n_zp_ (usually for positions) and n_zq_ (usually for attitudes)
  }

  if (!measurement_world_sensor_) // transform the readings and covariance to the sensor reference frame
  {
    Eigen::Matrix<double, 3, 3> C_zq = z_q_.toRotationMatrix();
    z_q_ = z_q_.conjugate();
    z_p_ = -C_zq.transpose() * z_p_;

    Eigen::Matrix<double, 6, 6> C_cov(Eigen::Matrix<double, 6, 6>::Zero());
    C_cov.block<3, 3> (0, 0) = C_zq;
    C_cov.block<3, 3> (3, 3) = C_zq;

    R.block<6, 6> (0, 0) = C_cov.transpose() * R.block<6, 6> (0, 0) * C_cov;
  }

  // feedback for init case
  measurements->p_vc_ = z_p_;	// these values are used as current sensor readings for state initialization
  measurements->q_cv_ = z_q_;	// these values are used as current sensor readings for state initialization

  // find closest predicted state in time which fits the measurement time
  unsigned char idx = measurements->ssf_core_.getClosestState(&state_old, time_old);
  if (state_old.time_ == -1)
    return; // // early abort // //

  // get rotation matrices
  Eigen::Matrix<double, 3, 3> C_wv = state_old.q_wv_.conjugate().toRotationMatrix();
  Eigen::Matrix<double, 3, 3> C_q = state_old.q_.conjugate().toRotationMatrix();
  Eigen::Matrix<double, 3, 3> C_ci = state_old.q_ci_.conjugate().toRotationMatrix();


  // construct measurement matrix H using H-blockx :-)
  // position:
  // H_old.block<..., ...> (..., ...) = ...

  // construct residuals y
  // r_old.block<..., ...> (..., ...) = ...

  // call update step in core class
  measurements->ssf_core_.applyMeasurement(idx, H_old, r_old, R);
}

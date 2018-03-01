/*
 * Copyright (c) 2017, James Jackson and Daniel Koch, BYU MAGICC Lab
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "estimator.h"
#include "rosflight.h"

#include <math.h>

namespace rosflight_firmware
{

static const Eigen::Matrix3f I_3x3 = [] {
  Eigen::Matrix3f tmp = Eigen::Matrix3f::Identity();
  return tmp;
}();

static const Eigen::Vector3f gravity = [] {
  Eigen::Vector3f tmp;
  tmp << 0, 0, 9.80665f;
  return tmp;
}();


Eigen::Matrix3f skew(const Eigen::Vector3f v)
{
  Eigen::Matrix3f mat;
  mat << 0.0, -v(2), v(1),
         v(2), 0.0, -v(0),
         -v(1), v(0), 0.0;
  return mat;
}

Eigen::Matrix<float, 3, 1> qrot(const Eigen::Matrix<float, 4, 1> &q, const Eigen::Matrix<float, 3, 1>& v)
{
  Eigen::Vector3f t = 2.0 * q.block<3,1>(1,0).cross(v);
  return v + q(0,0) * t + q.block<3,1>(1,0).cross(t);
}

Eigen::Matrix<float, 3, 1> qinvrot(const Eigen::Matrix<float, 4, 1> &q, const Eigen::Matrix<float, 3, 1>& v)
{
  Eigen::Vector3f t = 2.0 * q.block<3,1>(1,0).cross(v);
  return v - q(0,0) * t + q.block<3,1>(1,0).cross(t);
}

Eigen::Matrix<float, 3, 3> qR(const Eigen::Matrix<float, 4, 1> &q)
{
  float wx = q(0,0)*q(1,0);
  float wy = q(0,0)*q(2,0);
  float wz = q(0,0)*q(3,0);
  float xx = q(1,0)*q(1,0);
  float xy = q(1,0)*q(2,0);
  float xz = q(1,0)*q(3,0);
  float yy = q(2,0)*q(2,0);
  float yz = q(2,0)*q(3,0);
  float zz = q(3,0)*q(3,0);
  Eigen::Matrix<float, 3, 3> out;
  out << 1. - 2.*yy - 2.*zz, 2.*xy + 2.*wz, 2.*xz - 2.*wy,
      2.*xy - 2.*wz, 1. - 2.*xx - 2.*zz, 2.*yz + 2.*wx,
      2.*xz + 2.*wy, 2.*yz - 2.*wx, 1. - 2.*xx - 2.*yy;
  return out;
}

Estimator::Estimator(ROSflight &_rf):
  RF_(_rf)
{}

void Estimator::reset_state()
{
  x_ << 1.0, 0, 0, 0, // att
        0, 0, 0; // b_w
  
  dxVector Pdiag;
  Pdiag << 0.3, 0.3, 0.3,  // att
           0.01, 0.01, 0.1; // b_w
  P_ = Pdiag.asDiagonal();
  
  Qx_ << 0.01, 0.01, 0.01, // att
           0.001, 0.001, 0.001; // b_w
  
  Qu_ << 0.001, 0.001, 0.001; // gyro
  
  R_acc_ << 0.1, 0.1, 0.1;
  
  K_.setZero();
  H_.setZero();
  dx_.setZero();
  A_.setZero();
  G_.setZero();
  prev_t_us_ = 0;
}

void Estimator::reset_adaptive_bias()
{
  x_.block<3, 1>(xB_G, 0).setZero();
}

void Estimator::init()
{
  reset_state();
}

void Estimator::run_LPF()
{
  float alpha_acc = RF_.params_.get_param_float(PARAM_ACC_ALPHA);
  alpha_acc = 0.0f;
  const turbomath::Vector& raw_accel = RF_.sensors_.data().accel;
  accel_LPF_ = (1.0f-alpha_acc)*raw_accel + alpha_acc*accel_LPF_;

  float alpha_gyro = RF_.params_.get_param_float(PARAM_GYRO_ALPHA);
  alpha_gyro = 0.0f;
  const turbomath::Vector& raw_gyro = RF_.sensors_.data().gyro;
  gyro_LPF_ = (1.0f-alpha_gyro)*raw_gyro + alpha_gyro*gyro_LPF_;
}

void Estimator::run()
{
  uint64_t now_us = RF_.board_.clock_micros();
  
  if (prev_t_us_ < 1)
  {
    prev_t_us_ = now_us;
    return;
  }
  
  float dt = (now_us - prev_t_us_)*1e-6f;
  
  if (dt > 0.05 || dt <= 0.0001)
  {
    prev_t_us_ = now_us;
    RF_.state_manager_.set_error(StateManager::ERROR_UNHEALTHY_ESTIMATOR);
    return;
  }
  prev_t_us_ = now_us;
  
  // Run LPF
  run_LPF();
  
  // Load input
  u_(0) = gyro_LPF_.x;
  u_(1) = gyro_LPF_.y;
  u_(2) = gyro_LPF_.z;
  
  // Run dynamics  
  dynamics(x_, u_);
  boxplus(x_, dx_*dt, x_);
  
  // Propagate Covariance
  P_ += (A_ * P_ + P_ * A_.transpose() + G_ * Qu_.asDiagonal() * G_.transpose())*dt;  
  
  // Perform Accelerometer Measurement Update
  z_ << accel_LPF_.x, accel_LPF_.y, accel_LPF_.z;
  z_ *= 9.80665/z_.norm();
  update(z_, ACC, R_acc_.asDiagonal());
  
  // Clean up the quatenrion state
  x_.block<4,1>(xATT, 0) /= x_.block<4,1>(xATT, 0).norm();
  
  // Copy relvant data into state object for publishing to other modules
  state_.attitude.w = x_(xATT);
  state_.attitude.x = x_(xATT+1);
  state_.attitude.y = x_(xATT+2);
  state_.attitude.z = x_(xATT+3);
  state_.angular_velocity.x = u_(0) - x_(xB_G);
  state_.angular_velocity.y = u_(1) - x_(xB_G+1);
  state_.angular_velocity.z = u_(2) - x_(xB_G+2);
  state_.attitude.get_RPY(&state_.roll, &state_.pitch, &state_.yaw);
}

void Estimator::dynamics(const xVector &x, const uVector &u, dxVector &xdot, dxMatrix &dfdx, dxuMatrix &dfdu)
{
  dynamics(x, u);
  xdot = dx_;
  dfdx = A_;
  dfdu = G_;
}


void Estimator::dynamics(const xVector& x, const uVector &u)
{  
  // State Dynamics
  dx_.block<3,1>((int)dxATT, 0) = u.block<3,1>((int)uG, 0) - x.block<3,1>((int)xB_G, 0);
  
  // State Jacobian
  A_.block<3,3>((int)dxATT, (int)dxB_G) = -I_3x3;
  
  // Input Jacobian
  G_.block<3,3>((int)dxATT, (int)uG) = I_3x3;  
}

// Perform Quaternion integration
void Estimator::boxplus(const xVector& x, const dxVector& dx, xVector& out)
{  
  float norm_v = dx.block<3,1>((int)dxATT,0).norm();
  if (RF_.params_.get_param_int(PARAM_FILTER_USE_MAT_EXP) && norm_v > 1e-4f)
  {
    float v_scale = sinf(norm_v/2.0f)/norm_v;
    q1_ << cosf(norm_v/2.0f), v_scale*dx((int)dxATT + 0), v_scale*dx((int)dxATT + 1), v_scale*dx((int)dxATT + 2);
  }
  else
  {
    q1_ << 1.0f, dx((int)dxATT)/2.0f, dx((int)dxATT+1)/2.0f, dx((int)dxATT+2)/2.0f;
    q1_ /= q1_.norm();
  }
  out.block<4,1>((int)xATT, 0) << x(0,0) * q1_(0,0) - x(1,0) *q1_(1,0) - x(2,0) * q1_(2,0) - x(3,0) * q1_(3,0),
                      		  x(0,0) * q1_(1,0) + x(1,0) *q1_(0,0) + x(2,0) * q1_(3,0) - x(3,0) * q1_(2,0),
                      		  x(0,0) * q1_(2,0) - x(1,0) *q1_(3,0) + x(2,0) * q1_(0,0) + x(3,0) * q1_(1,0),
                      		  x(0,0) * q1_(3,0) + x(1,0) *q1_(2,0) - x(2,0) * q1_(1,0) + x(3,0) * q1_(0,0);
  
}

bool Estimator::update(const hVector& z, const measurement_type_t& meas_type, const Eigen::Matrix3f& R)
{
  // Call the appropriate measurement function
  (this->*(measurement_functions[meas_type]))(x_, zhat_, H_);
  
  hVector residual = zhat_ - z;
  
  // calculate Kalman Gain
  K_ = P_ * H_.transpose() * (R + H_*P_ * H_.transpose()).inverse();
  
  boxplus(x_, K_*residual, x_);
  P_ -= K_*H_*P_;  
}


void Estimator::h_acc(const xVector &x, hVector &h, HMatrix &H) const
{
  h = -qinvrot(x.block<4,1>(xATT, 0), gravity);
  
  H.setZero();
  H.block<3,3>(0, dxATT) = -skew(h);  
}

} // namespace rosflight_firmware

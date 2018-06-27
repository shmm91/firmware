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

namespace rosflight_firmware
{

Estimator::Estimator(ROSflight &_rf):
  RF_(_rf)
{}

void Estimator::reset_state()
{
  state_.position.setZero();
  state_.attitude = Quat::Identity();
  state_.linear_velocity.setZero();
  state_.angular_velocity.setZero();

  w1_.setZero();
  w2_.setZero();
  bias_.setZero();

  accel_LPF_ = 9.80665 * g_;
  gyro_LPF_.setZero();

  state_.timestamp_us = RF_.board_.clock_micros();

  // Clear the unhealthy estimator flag
  RF_.state_manager_.clear_error(StateManager::ERROR_UNHEALTHY_ESTIMATOR);
}

void Estimator::reset_adaptive_bias()
{
  bias_.setZero();
}

void Estimator::init()
{
  last_time_ = 0;
  last_acc_update_us_ = 0;
  reset_state();
}

void Estimator::run_LPF()
{
  float alpha_acc = RF_.params_.get_param_float(PARAM_ACC_ALPHA);
  const Vec3 raw_accel = RF_.sensors_.data().accel;
  accel_LPF_ = (1.0f - alpha_acc) * raw_accel + alpha_acc * accel_LPF_;

  float alpha_gyro = RF_.params_.get_param_float(PARAM_GYRO_ALPHA);
  const Vec3 raw_gyro = RF_.sensors_.data().gyro;
  gyro_LPF_ = (1.0f - alpha_gyro) * raw_gyro + alpha_gyro * gyro_LPF_;
}

void Estimator::run()
{
  if (RF_.board_.ins_present())
    run_ins();
  else
    run_mahony();
}

void Estimator::run_ins()
{
  // grab estimates from uINS
  state_.timestamp_us = RF_.sensors_.data().ins_time;
  state_.position = RF_.sensors_.data().ins_position;
  state_.attitude = RF_.sensors_.data().ins_attitude;
  state_.linear_velocity = RF_.sensors_.data().ins_linear_velocity;
  state_.angular_velocity = RF_.sensors_.data().gyro;
}

void Estimator::run_mahony()
{
  float kp, ki;
  uint64_t now_us = RF_.sensors_.data().imu_time;
  if (last_time_ == 0)
  {
    last_time_ = now_us;
    last_acc_update_us_ = last_time_;
    return;
  }
  else if (now_us < last_time_)
  {
    // this shouldn't happen
    RF_.state_manager_.set_error(StateManager::ERROR_TIME_GOING_BACKWARDS);
    last_time_ = now_us;
    return;
  }


  RF_.state_manager_.clear_error(StateManager::ERROR_TIME_GOING_BACKWARDS);

  float dt = (now_us - last_time_) * 1e-6f;
  last_time_ = now_us;
  state_.timestamp_us = now_us;

  // Crank up the gains for the first few seconds for quick convergence
  if (now_us < static_cast<uint64_t>(RF_.params_.get_param_int(PARAM_INIT_TIME))*1000)
  {
    kp = RF_.params_.get_param_float(PARAM_FILTER_KP)*10.0f;
    ki = RF_.params_.get_param_float(PARAM_FILTER_KI)*10.0f;
  }
  else
  {
    kp = RF_.params_.get_param_float(PARAM_FILTER_KP);
    ki = RF_.params_.get_param_float(PARAM_FILTER_KI);
  }

  // Run LPF to reject a lot of noise
  run_LPF();

  // add in accelerometer
  float a_sqrd_norm = accel_LPF_.squaredNorm();

  Vec3 w_acc;
  if (RF_.params_.get_param_int(PARAM_FILTER_USE_ACC)
      && a_sqrd_norm < 1.1f*1.1f*9.80665f*9.80665f && a_sqrd_norm > 0.9f*0.9f*9.80665f*9.80665f)
  {
    // Get error estimated by accelerometer measurement
    last_acc_update_us_ = now_us;
    // turn measurement into a unit vector
    Vec3 a = accel_LPF_.normalized();
    // Get the quaternion from accelerometer (low-frequency measure q)
    // (Not in either paper)
    Quat q_acc_inv = Quat::from_two_unit_vectors(g_, a);
    // Get the error quaternion between observer and low-freq q
    // Below Eq. 45 Mahony Paper
    Quat q_tilde = q_acc_inv * state_.attitude;
    // Correction Term of Eq. 47a and 47b Mahony Paper
    // w_acc = 2*s_tilde*v_tilde
    w_acc.x() = -2.0f * q_tilde.w() * q_tilde.x();
    w_acc.y() = -2.0f * q_tilde.w() * q_tilde.y();
    w_acc.z() = 0.0f; // Don't correct z, because it's unobservable from the accelerometer

    // integrate biases from accelerometer feedback
    // (eq 47b Mahony Paper, using correction term w_acc found above
    bias_.x() -= ki * w_acc.x() * dt;
    bias_.y() -= ki * w_acc.y() * dt;
    bias_.z() = 0.0;  // Don't integrate z bias, because it's unobservable
  }
  else
  {
    w_acc.setZero();
  }


  // Handle Gyro Measurements
  Vec3 wbar;
  if (RF_.params_.get_param_int(PARAM_FILTER_USE_QUAD_INT))
  {
    // Quadratic Interpolation (Eq. 14 Casey Paper)
    // this step adds 12 us on the STM32F10x chips
    wbar = (w2_/-12.0f) + w1_*(8.0f/12.0f) + gyro_LPF_ * (5.0f/12.0f);
    w2_ = w1_;
    w1_ = gyro_LPF_;
  }
  else
  {
    wbar = gyro_LPF_;
  }

  // Build the composite omega vector for kinematic propagation
  // This the stuff inside the p function in eq. 47a - Mahony Paper
  Vec3 wfinal = wbar - bias_ + w_acc * kp;

  // Propagate Dynamics (only if we've moved)
  float sqrd_norm_w = wfinal.squaredNorm();
  if (sqrd_norm_w > 0.0f)
  {
    float p = wfinal.x();
    float q = wfinal.y();
    float r = wfinal.z();

    if (RF_.params_.get_param_int(PARAM_FILTER_USE_MAT_EXP))
    {
      // Matrix Exponential Approximation (From Attitude Representation and Kinematic
      // Propagation for Low-Cost UAVs by Robert T. Casey)
      // (Eq. 12 Casey Paper)
      // This adds 90 us on STM32F10x chips
      float norm_w = sqrtf(sqrd_norm_w);
      Quat qhat_np1;
      float t1 = cosf((norm_w*dt)/2.0f);
      float t2 = 1.0f/norm_w * sinf((norm_w*dt)/2.0f);
      qhat_np1.setW(t1*state_.attitude.w() + t2*(-p*state_.attitude.x() - q*state_.attitude.y() - r*state_.attitude.z()));
      qhat_np1.setX(t1*state_.attitude.x() + t2*( p*state_.attitude.w() + r*state_.attitude.y() - q*state_.attitude.z()));
      qhat_np1.setY(t1*state_.attitude.y() + t2*( q*state_.attitude.w() - r*state_.attitude.x() + p*state_.attitude.z()));
      qhat_np1.setZ(t1*state_.attitude.z() + t2*( r*state_.attitude.w() + q*state_.attitude.x() - p*state_.attitude.y()));
      state_.attitude = qhat_np1.normalize();
    }
    else
    {
      // Euler Integration
      // (Eq. 47a Mahony Paper), but this is pretty straight-forward
      Quat qdot(0.5f * (-p*state_.attitude.x() - q*state_.attitude.y() - r*state_.attitude.z()),
                0.5f * ( p*state_.attitude.w() + r*state_.attitude.y() - q*state_.attitude.z()),
                0.5f * ( q*state_.attitude.w() - r*state_.attitude.x() + p*state_.attitude.z()),
                0.5f * ( r*state_.attitude.w() + q*state_.attitude.x() - p*state_.attitude.y()));
      state_.attitude.arr_ += qdot.arr_ * dt;
      state_.attitude.normalize();
    }
  }

  // Save off adjust gyro measurements with estimated biases for control
  state_.angular_velocity = gyro_LPF_ - bias_;

  // If it has been more than 0.5 seconds since the acc update ran and we are supposed to be getting them
  // then trigger an unhealthy estimator error
  if (RF_.params_.get_param_int(PARAM_FILTER_USE_ACC) && now_us > 500000 + last_acc_update_us_
      && !RF_.params_.get_param_int(PARAM_FIXED_WING))
  {
    RF_.state_manager_.set_error(StateManager::ERROR_UNHEALTHY_ESTIMATOR);
  }
  else
  {
    RF_.state_manager_.clear_error(StateManager::ERROR_UNHEALTHY_ESTIMATOR);
  }
}

} // namespace rosflight_firmware

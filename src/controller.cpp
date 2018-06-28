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

#include <stdint.h>
#include <stdbool.h>

#include "command_manager.h"
#include "estimator.h"
#include "rosflight.h"

#include "controller.h"

namespace rosflight_firmware
{

Controller::Controller(ROSflight& rf) :
  RF_(rf)
{
  RF_.params_.add_callback([this](uint16_t param_id){this->param_change_callback(param_id);}, PARAM_PID_ROLL_ANGLE_P);
  RF_.params_.add_callback([this](uint16_t param_id){this->param_change_callback(param_id);}, PARAM_PID_ROLL_ANGLE_I);
  RF_.params_.add_callback([this](uint16_t param_id){this->param_change_callback(param_id);}, PARAM_PID_ROLL_ANGLE_D);
  RF_.params_.add_callback([this](uint16_t param_id){this->param_change_callback(param_id);}, PARAM_PID_ROLL_RATE_P);
  RF_.params_.add_callback([this](uint16_t param_id){this->param_change_callback(param_id);}, PARAM_PID_ROLL_RATE_I);
  RF_.params_.add_callback([this](uint16_t param_id){this->param_change_callback(param_id);}, PARAM_PID_ROLL_RATE_D);
  RF_.params_.add_callback([this](uint16_t param_id){this->param_change_callback(param_id);}, PARAM_PID_PITCH_ANGLE_P);
  RF_.params_.add_callback([this](uint16_t param_id){this->param_change_callback(param_id);}, PARAM_PID_PITCH_ANGLE_I);
  RF_.params_.add_callback([this](uint16_t param_id){this->param_change_callback(param_id);}, PARAM_PID_PITCH_ANGLE_D);
  RF_.params_.add_callback([this](uint16_t param_id){this->param_change_callback(param_id);}, PARAM_PID_PITCH_RATE_P);
  RF_.params_.add_callback([this](uint16_t param_id){this->param_change_callback(param_id);}, PARAM_PID_PITCH_RATE_I);
  RF_.params_.add_callback([this](uint16_t param_id){this->param_change_callback(param_id);}, PARAM_PID_PITCH_RATE_D);
  RF_.params_.add_callback([this](uint16_t param_id){this->param_change_callback(param_id);}, PARAM_PID_YAW_RATE_P);
  RF_.params_.add_callback([this](uint16_t param_id){this->param_change_callback(param_id);}, PARAM_PID_YAW_RATE_I);
  RF_.params_.add_callback([this](uint16_t param_id){this->param_change_callback(param_id);}, PARAM_PID_YAW_RATE_D);
  RF_.params_.add_callback([this](uint16_t param_id){this->param_change_callback(param_id);}, PARAM_MAX_COMMAND);
  RF_.params_.add_callback([this](uint16_t param_id){this->param_change_callback(param_id);}, PARAM_PID_TAU);
  RF_.params_.add_callback([this](uint16_t param_id){this->param_change_callback(param_id);}, PARAM_NLC_POS_N);
  RF_.params_.add_callback([this](uint16_t param_id){this->param_change_callback(param_id);}, PARAM_NLC_POS_E);
  RF_.params_.add_callback([this](uint16_t param_id){this->param_change_callback(param_id);}, PARAM_NLC_POS_D);
  RF_.params_.add_callback([this](uint16_t param_id){this->param_change_callback(param_id);}, PARAM_NLC_VEL_X);
  RF_.params_.add_callback([this](uint16_t param_id){this->param_change_callback(param_id);}, PARAM_NLC_VEL_Y);
  RF_.params_.add_callback([this](uint16_t param_id){this->param_change_callback(param_id);}, PARAM_NLC_VEL_Z);
  RF_.params_.add_callback([this](uint16_t param_id){this->param_change_callback(param_id);}, PARAM_NLC_DIST_X);
  RF_.params_.add_callback([this](uint16_t param_id){this->param_change_callback(param_id);}, PARAM_NLC_DIST_Y);
  RF_.params_.add_callback([this](uint16_t param_id){this->param_change_callback(param_id);}, PARAM_NLC_DIST_Z);
  RF_.params_.add_callback([this](uint16_t param_id){this->param_change_callback(param_id);}, PARAM_EQ_THROTTLE);
  RF_.params_.add_callback([this](uint16_t param_id){this->param_change_callback(param_id);}, PARAM_NLC_MAX_VEL);
  RF_.params_.add_callback([this](uint16_t param_id){this->param_change_callback(param_id);}, PARAM_NLC_MAX_YAW_RATE);
  RF_.params_.add_callback([this](uint16_t param_id){this->param_change_callback(param_id);}, PARAM_NLC_ANGLE_P);
  RF_.params_.add_callback([this](uint16_t param_id){this->param_change_callback(param_id);}, PARAM_NLC_ANGLE_D);
}

void Controller::init()
{
  prev_time_us_ = 0;

  float max = RF_.params_.get_param_float(PARAM_MAX_COMMAND);
  float min = -max;
  float tau = RF_.params_.get_param_float(PARAM_PID_TAU);

  roll_.init(RF_.params_.get_param_float(PARAM_PID_ROLL_ANGLE_P),
             RF_.params_.get_param_float(PARAM_PID_ROLL_ANGLE_I),
             RF_.params_.get_param_float(PARAM_PID_ROLL_ANGLE_D),
             max, min, tau);
  roll_rate_.init(RF_.params_.get_param_float(PARAM_PID_ROLL_RATE_P),
                  RF_.params_.get_param_float(PARAM_PID_ROLL_RATE_I),
                  RF_.params_.get_param_float(PARAM_PID_ROLL_RATE_D),
                  max, min, tau);
  pitch_.init(RF_.params_.get_param_float(PARAM_PID_PITCH_ANGLE_P),
              RF_.params_.get_param_float(PARAM_PID_PITCH_ANGLE_I),
              RF_.params_.get_param_float(PARAM_PID_PITCH_ANGLE_D),
              max, min, tau);
  pitch_rate_.init(RF_.params_.get_param_float(PARAM_PID_PITCH_RATE_P),
                   RF_.params_.get_param_float(PARAM_PID_PITCH_RATE_I),
                   RF_.params_.get_param_float(PARAM_PID_PITCH_RATE_D),
                   max, min, tau);
  yaw_rate_.init(RF_.params_.get_param_float(PARAM_PID_YAW_RATE_P),
                 RF_.params_.get_param_float(PARAM_PID_YAW_RATE_I),
                 RF_.params_.get_param_float(PARAM_PID_YAW_RATE_D),
                 max, min, tau);

  dhat_.setZero();

  K_p_.setZero();
  K_p_(0, 0) = PARAM_NLC_POS_N;
  K_p_(1, 1) = PARAM_NLC_POS_E;
  K_p_(2, 2) = PARAM_NLC_POS_D;

  K_v_.setZero();
  K_v_(0, 0) = PARAM_NLC_VEL_X;
  K_v_(1, 1) = PARAM_NLC_VEL_Y;
  K_v_(2, 2) = PARAM_NLC_VEL_Z;

  K_d_.setZero();
  K_d_(0, 0) = PARAM_NLC_DIST_X;
  K_d_(1, 1) = PARAM_NLC_DIST_Y;
  K_d_(2, 2) = PARAM_NLC_DIST_Z;
}

void Controller::run()
{
  if (RF_.command_manager_.combined_control().F.type == POS)
  {
    run_position_control();
  }
  else if (RF_.command_manager_.combined_control().F.type == VEL)
  {
    run_velocity_control();
  }
  else if (RF_.command_manager_.combined_control().F.type == ANGLE)
  {
    run_angle_control();
  }
  else if (RF_.command_manager_.combined_control().F.type == RATE)
  {
    run_angular_rate_control();
  }
  else
  {
    RF_.comm_manager_.log(CommLink::LogSeverity::LOG_ERROR, "Invalid control type.");
  }
}

void Controller::run_position_control()
{
  // Time calculation
  if (prev_time_us_ == 0)
  {
    prev_time_us_ = RF_.estimator_.state().timestamp_us;
    return;
  }

  int32_t dt_us = (RF_.estimator_.state().timestamp_us - prev_time_us_);
  if ( dt_us < 0 )
  {
    RF_.state_manager_.set_error(StateManager::ERROR_TIME_GOING_BACKWARDS);
    return;
  }
  prev_time_us_ = RF_.estimator_.state().timestamp_us;

  // get time in seconds
  float dt = 1e-6*dt_us;

  // get state estimates
  Vec3 p = RF_.estimator_.state().position; // inertial to body position
  Quat q = RF_.estimator_.state().attitude; // inertial to body rotation
  Vec3 v = RF_.estimator_.state().linear_velocity; // body velocity
  Vec3 omega = RF_.estimator_.state().angular_velocity; // body angular rate

  // get commands
  Vec3 pc(RF_.command_manager_.combined_control().x.value,
          RF_.command_manager_.combined_control().y.value,
          RF_.command_manager_.combined_control().z.value); // position command
  float yaw_c = RF_.command_manager_.combined_control().F.value; // yaw command

  // define some constants
  static float g = 9.80665; // gravity magnitude
  static Vec3 k(0, 0, 1); // arbitrary z-axis unit vector

  // compute rotations between reference frames and vehicle-1 velocity
  Vec3 axis_angle = Quat::log(q); // axis-angle representation of attitude
  Vec3 axis_angle_v_to_v1(0, 0, axis_angle(2)); // axis-angle representation of vehicle ot vehicle-1 rotation
  Vec3 axis_angle_v1_to_b(axis_angle(0), axis_angle(1), 0); // axis-angle representation of vehicle-1 to body rotation
  Quat q_v_to_v1 = Quat::exp(axis_angle_v_to_v1); // quaternion rotation from vehicle to vehicle-1
  Quat q_v1_to_b = Quat::exp(axis_angle_v1_to_b); // quaternion rotation from vehicle-1 to body
  Vec3 v1 = q_v1_to_b.inverse().rotp(v); // vehicle-1 velocity estimate

  // compute saturated velocity command from position error
  Vec3 vc = q_v_to_v1.rotp(K_p_*(pc-p));
  float vmag = vc.norm();
  if (vmag > PARAM_NLC_MAX_VEL)
    vc *= PARAM_NLC_MAX_VEL/vmag;

  // update disturbance estimate and compute throttle command
  dhat_ = dhat_ - K_d_*(vc-v1)*dt;
  Vec3 k_tilde = PARAM_EQ_THROTTLE*(k-(1.0/g)*(K_v_*(vc-v1)-dhat_));
  output_.F = k.transpose()*q_v1_to_b.rotp(k_tilde);

  // get shortest rotation to desired tilt
  Vec3 kd = (1.0/output_.F)*k_tilde; // desired body z direction
  kd = kd/kd.norm(); // normalize to keep direction only
  float tilt_angle = acos(k.transpose()*kd); // desired tilt angle

  // build commanded attitude
  Quat q_c;
  Vec3 yaw_c_vec(0, 0, yaw_c);
  if (tilt_angle > 1e-6)
  {
    Vec3 k_cross_kd = skew(k)*kd;
    q_c = Quat::exp(tilt_angle*k_cross_kd/k_cross_kd.norm() + yaw_c_vec);
  }
  else
  {
    q_c = Quat::exp(yaw_c_vec);
  }

  // proportional-derivative control on difference between desired attitude
  // and current attitude to get commanded angular rate
  Vec3 omega_c = PARAM_NLC_ANGLE_P*Quat::log(q.inverse()*q_c) - PARAM_NLC_ANGLE_D*omega;

  // proportional control on angular rate error to achieve commanded angular rate
  Vec3 omega_err = omega_c - omega;
  output_.x = PARAM_PID_ROLL_RATE_P*omega_err(0) + RF_.params_.get_param_float(PARAM_X_EQ_TORQUE);
  output_.y = PARAM_PID_PITCH_RATE_P*omega_err(1) + RF_.params_.get_param_float(PARAM_Y_EQ_TORQUE);
  output_.z = PARAM_PID_YAW_RATE_P*omega_err(2) + RF_.params_.get_param_float(PARAM_Z_EQ_TORQUE);

  // too high yaw rate reduces controllability of multirotor, so saturate it
  if (output_.z > PARAM_NLC_MAX_YAW_RATE)
    output_.z = PARAM_NLC_MAX_YAW_RATE;
  else if (output_.z < -PARAM_NLC_MAX_YAW_RATE)
    output_.z = -PARAM_NLC_MAX_YAW_RATE;
}

void Controller::run_velocity_control()
{
  // Time calculation
  if (prev_time_us_ == 0)
  {
    prev_time_us_ = RF_.estimator_.state().timestamp_us;
    return;
  }

  int32_t dt_us = (RF_.estimator_.state().timestamp_us - prev_time_us_);
  if ( dt_us < 0 )
  {
    RF_.state_manager_.set_error(StateManager::ERROR_TIME_GOING_BACKWARDS);
    return;
  }
  prev_time_us_ = RF_.estimator_.state().timestamp_us;

  // get time in seconds
  float dt = 1e-6*dt_us;

  // get state estimates
  Quat q = RF_.estimator_.state().attitude; // inertial to body rotation
  Vec3 v = RF_.estimator_.state().linear_velocity; // body velocity
  Vec3 omega = RF_.estimator_.state().angular_velocity; // body angular rate

  // compute rotations between reference frames and vehicle-1 velocity
  Vec3 axis_angle = Quat::log(q); // axis-angle representation of attitude
  Vec3 axis_angle_v1_to_b(axis_angle(0), axis_angle(1), 0); // axis-angle representation of vehicle-1 to body rotation
  Quat q_v1_to_b = Quat::exp(axis_angle_v1_to_b); // quaternion rotation from vehicle-1 to body
  Vec3 v1 = q_v1_to_b.inverse().rotp(v); // vehicle-1 velocity estimate

  // get commanded velocities, yaw rate, and altitude rate
  float vx_c = RF_.command_manager_.combined_control().x.value;
  float vy_c = RF_.command_manager_.combined_control().y.value;
  float yaw_rate_c = RF_.command_manager_.combined_control().z.value;
  float alt_rate_c = RF_.command_manager_.combined_control().F.value;

  // update disturbance estimate and compute throttle command
  static float g = 9.80665; // gravity magnitude
  static Vec3 k(0, 0, 1); // arbitrary z-axis unit vector
  Vec3 vc(vx_c, vy_c, -alt_rate_c);
  dhat_ = dhat_ - K_d_*(vc-v1)*dt;
  Vec3 k_tilde = PARAM_EQ_THROTTLE*(k-(1.0/g)*(K_v_*(vc-v1)-dhat_));
  output_.F = k.transpose()*q_v1_to_b.rotp(k_tilde);

  // get shortest rotation to desired tilt
  Vec3 kd = (1.0/output_.F)*k_tilde; // desired body z direction
  kd = kd/kd.norm(); // normalize to keep direction only
  float tilt_angle = acos(k.transpose()*kd); // desired tilt angle

  // build commanded attitude
  Quat q_c(1, 0, 0, 0);
  if (tilt_angle > 1e-6)
  {
    Vec3 k_cross_kd = skew(k)*kd;
    q_c = Quat::exp(tilt_angle*k_cross_kd/k_cross_kd.norm());
  }

  // proportional-derivative control on difference between desired attitude
  // and current attitude to get commanded angular rate
  Vec3 omega_c = PARAM_NLC_ANGLE_P*Quat::log(q.inverse()*q_c) - PARAM_NLC_ANGLE_D*omega;
  omega_c(2) += yaw_rate_c; // add yaw rate to total angular rate command

  // proportional control on angular rate error to achieve commanded angular rate
  Vec3 omega_err = omega_c - omega;
  output_.x = PARAM_PID_ROLL_RATE_P*omega_err(0) + RF_.params_.get_param_float(PARAM_X_EQ_TORQUE);
  output_.y = PARAM_PID_PITCH_RATE_P*omega_err(1) + RF_.params_.get_param_float(PARAM_Y_EQ_TORQUE);
  output_.z = PARAM_PID_YAW_RATE_P*omega_err(2) + RF_.params_.get_param_float(PARAM_Z_EQ_TORQUE);
}

void Controller::run_angle_control()
{
  // get state estimates
  Quat q = RF_.estimator_.state().attitude; // inertial to body rotation
  Vec3 omega = RF_.estimator_.state().angular_velocity; // body angular rate

  // get commanded angles and yaw rate
  float roll_c = RF_.command_manager_.combined_control().x.value;
  float pitch_c = RF_.command_manager_.combined_control().y.value;
  float yaw_rate_c = RF_.command_manager_.combined_control().z.value;

  // build commanded attitude from commanded roll and pitch
  Quat q_c(1, 0, 0, 0);
  Vec3 tilt_c(roll_c,pitch_c,0);
  if (tilt_c.norm() > 1e-6)
    q_c = Quat::exp(tilt_c);

  // proportional-derivative control on difference between desired attitude and commanded
  Vec3 omega_c = PARAM_NLC_ANGLE_P*Quat::log(q.inverse()*q_c) - PARAM_NLC_ANGLE_D*omega;
  omega_c(2) += yaw_rate_c; // add yaw rate to total angular rate command

  // proportional control on angular rate error to achieve commanded angular rate
  Vec3 omega_err = omega_c - omega;
  output_.x = PARAM_PID_ROLL_RATE_P*omega_err(0) + RF_.params_.get_param_float(PARAM_X_EQ_TORQUE);
  output_.y = PARAM_PID_PITCH_RATE_P*omega_err(1) + RF_.params_.get_param_float(PARAM_Y_EQ_TORQUE);
  output_.z = PARAM_PID_YAW_RATE_P*omega_err(2) + RF_.params_.get_param_float(PARAM_Z_EQ_TORQUE);
  output_.F = RF_.command_manager_.combined_control().F.value; // passthrough throttle command
}

void Controller::run_angular_rate_control()
{
  // get estimated angular rate
  Vec3 omega = RF_.estimator_.state().angular_velocity;

  // get commanded angular rate
  Vec3 omega_c(RF_.command_manager_.combined_control().x.value,
               RF_.command_manager_.combined_control().y.value,
               RF_.command_manager_.combined_control().z.value);

  // proportional control on angular rate error to achieve commanded angular rate
  Vec3 omega_err = omega_c - omega;
  output_.x = PARAM_PID_ROLL_RATE_P*omega_err(0) + RF_.params_.get_param_float(PARAM_X_EQ_TORQUE);
  output_.y = PARAM_PID_PITCH_RATE_P*omega_err(1) + RF_.params_.get_param_float(PARAM_Y_EQ_TORQUE);
  output_.z = PARAM_PID_YAW_RATE_P*omega_err(2) + RF_.params_.get_param_float(PARAM_Z_EQ_TORQUE);
  output_.F = RF_.command_manager_.combined_control().F.value; // passthrough throttle command
}

void Controller::run_pid()
{
  // Time calculation
  if (prev_time_us_ == 0)
  {
    prev_time_us_ = RF_.estimator_.state().timestamp_us;
    return;
  }

  int32_t dt_us = (RF_.estimator_.state().timestamp_us - prev_time_us_);
  if ( dt_us < 0 )
  {
    RF_.state_manager_.set_error(StateManager::ERROR_TIME_GOING_BACKWARDS);
    return;
  }
  prev_time_us_ = RF_.estimator_.state().timestamp_us;

  // Check if integrators should be updated
  //! @todo better way to figure out if throttle is high
  bool update_integrators = (RF_.state_manager_.state().armed) && (RF_.command_manager_.combined_control().F.value > 0.1f) && dt_us < 10000;

  // Run the PID loops
  Vec3 pid_output = run_pid_loops(dt_us, RF_.estimator_.state(), RF_.command_manager_.combined_control(), update_integrators);

  // Add feedforward torques
  output_.x = pid_output.x() + RF_.params_.get_param_float(PARAM_X_EQ_TORQUE);
  output_.y = pid_output.y() + RF_.params_.get_param_float(PARAM_Y_EQ_TORQUE);
  output_.z = pid_output.z() + RF_.params_.get_param_float(PARAM_Z_EQ_TORQUE);
  output_.F = RF_.command_manager_.combined_control().F.value;
}

void Controller::calculate_equilbrium_torque_from_rc()
{
  // Make sure we are disarmed
  if (!(RF_.state_manager_.state().armed))
  {
    // Tell the user that we are doing a equilibrium torque calibration
    RF_.comm_manager_.log(CommLink::LogSeverity::LOG_WARNING, "Capturing equilbrium offsets from RC");

    // Prepare for calibration
    // artificially tell the flight controller it is leveled
    Estimator::State fake_state;
    fake_state.angular_velocity.setZero();

    fake_state.attitude = Quat::Identity();

    // pass the rc_control through the controller
    // dt is zero, so what this really does is applies the P gain with the settings
    // your RC transmitter, which if it flies level is a really good guess for
    // the static offset torques
    Vec3 pid_output = run_pid_loops(0, fake_state, RF_.command_manager_.rc_control(), false);

    // the output from the controller is going to be the static offsets
    RF_.params_.set_param_float(PARAM_X_EQ_TORQUE, pid_output.x() + RF_.params_.get_param_float(PARAM_X_EQ_TORQUE));
    RF_.params_.set_param_float(PARAM_Y_EQ_TORQUE, pid_output.y() + RF_.params_.get_param_float(PARAM_Y_EQ_TORQUE));
    RF_.params_.set_param_float(PARAM_Z_EQ_TORQUE, pid_output.z() + RF_.params_.get_param_float(PARAM_Z_EQ_TORQUE));

    RF_.comm_manager_.log(CommLink::LogSeverity::LOG_WARNING, "Equilibrium torques found and applied.");
    RF_.comm_manager_.log(CommLink::LogSeverity::LOG_WARNING, "Please zero out trims on your transmitter");
  }
  else
  {
    RF_.comm_manager_.log(CommLink::LogSeverity::LOG_WARNING, "Cannot perform equilibrium offset calibration while armed");
  }
}

void Controller::param_change_callback(uint16_t param_id)
{
  (void) param_id; // suppress unused parameter warning
  init();
}

Vec3 Controller::run_pid_loops(uint32_t dt_us, const Estimator::State& state, const control_t& command, bool update_integrators)
{
  // Based on the control types coming from the command manager, run the appropriate PID loops
  Vec3 out;

  float dt = 1e-6*dt_us;

  // ROLL
  if (command.x.type == RATE)
    out.x() = roll_rate_.run(dt, state.angular_velocity.x(), command.x.value, update_integrators);
  else if (command.x.type == ANGLE)
    out.x() = roll_.run(dt, state.attitude.roll(), command.x.value, update_integrators, state.angular_velocity.x());
  else
    out.x() = command.x.value;

  // PITCH
  if (command.y.type == RATE)
    out.y() = pitch_rate_.run(dt, state.angular_velocity.y(), command.y.value, update_integrators);
  else if (command.y.type == ANGLE)
    out.y() = pitch_.run(dt, state.attitude.pitch(), command.y.value, update_integrators, state.angular_velocity.y());
  else
    out.y() = command.y.value;

  // YAW
  if (command.z.type == RATE)
    out.z() = yaw_rate_.run(dt, state.angular_velocity.z(), command.z.value, update_integrators);
  else
    out.z() = command.z.value;

  return out;
}

Controller::PID::PID() :
  kp_(0.0f),
  ki_(0.0f),
  kd_(0.0f),
  max_(1.0f),
  min_(-1.0f),
  integrator_(0.0f),
  differentiator_(0.0f),
  prev_x_(0.0f),
  tau_(0.05)
{}

void Controller::PID::init(const float kp, const float ki, const float kd, const float max, const float min, const float tau)
{
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
  max_ = max;
  min_ = min;
  tau_ = tau;
}

float Controller::PID::run(const float dt, const float x, const float x_c, const bool update_integrator)
{
  float xdot;
  if (dt > 0.0001f)
  {
    // calculate D term (use dirty derivative if we don't have access to a measurement of the derivative)
    // The dirty derivative is a sort of low-pass filtered version of the derivative.
    //// (Include reference to Dr. Beard's notes here)
    differentiator_ = (2.0f * tau_ - dt) / (2.0f * tau_ + dt) * differentiator_
        + 2.0f / (2.0f * tau_ + dt) * (x - prev_x_);
    xdot = differentiator_;
  }
  else
  {
    xdot = 0.0f;
  }
  prev_x_ = x;

  return run(dt, x, x_c, update_integrator, xdot);
}

float Controller::PID::run(const float dt, const float x, const float x_c, const bool update_integrator, const float xdot)
{
  // Calculate Error
  float error = x_c - x;

  // Initialize Terms
  float p_term = error * kp_;
  float i_term = 0.0f;
  float d_term = 0.0f;

  // If there is a derivative term
  if (kd_ > 0.0f)
  {
    d_term = kd_ * xdot;
  }

  //If there is an integrator term and we are updating integrators
  if ((ki_ > 0.0f) && update_integrator)
  {
    // integrate
    integrator_ += error * dt;
    // calculate I term
    i_term = ki_ * integrator_;
  }

  // sum three terms
  float u = p_term - d_term + i_term;

  // Integrator anti-windup
  //// Include reference to Dr. Beard's notes here
  float u_sat = (u > max_) ? max_ : (u < min_) ? min_ : u;
  if (u != u_sat && fabs(i_term) > fabs(u - p_term + d_term) && ki_ > 0.0f)
    integrator_ = (u_sat - p_term + d_term)/ki_;

  // Set output
  return u_sat;
}

} // namespace rosflight_firmware

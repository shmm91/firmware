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

#ifndef ROSFLIGHT_FIRMWARE_CONTROLLER_H
#define ROSFLIGHT_FIRMWARE_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>

#include <turbomath/turbomath.h>

#include "command_manager.h"
#include "estimator.h"

using namespace rosflight_math;

namespace rosflight_firmware
{

class ROSflight;

class Controller
{
public:
  struct Output
  {
    float F;
    float x;
    float y;
    float z;
  };

  Controller(ROSflight& rf);

  inline const Output& output() const { return output_; }

  void init();
  void run();

  void calculate_equilbrium_torque_from_rc();
  void param_change_callback(uint16_t param_id);

private:
  class PID
  {
  public:
    PID();
    void init(const float kp, const float ki, const float kd, const float max, const float min, const float tau);
    float run(const float dt, const float x, const float x_c, const bool update_integrator);
    float run(const float dt, const float x, const float x_c, const bool update_integrator, const float xdot);

  private:
    float kp_;
    float ki_;
    float kd_;

    float max_;
    float min_;

    float integrator_;
    float differentiator_;
    float prev_x_;
    float tau_;
  };

  ROSflight& RF_;

  void run_position_control();
  void run_velocity_control();
  void run_angle_control();
  void run_angular_rate_control();
  void run_pid();
  Vec3 run_pid_loops(uint32_t dt, const Estimator::State& state, const control_t& command, bool update_integrators);

  Output output_;

  PID roll_;
  PID roll_rate_;
  PID pitch_;
  PID pitch_rate_;
  PID yaw_rate_;

  Vec3 dhat_;
  Mat33 K_p_;
  Mat33 K_v_;
  Mat33 K_d_;

  uint64_t prev_time_us_;
};

} // namespace rosflight_firmware

#endif // ROSFLIGHT_FIRMWARE_CONTROLLER_H

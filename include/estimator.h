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

#ifndef ROSFLIGHT_FIRMWARE_ESTIMATOR_H
#define ROSFLIGHT_FIRMWARE_ESTIMATOR_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include <turbomath/turbomath.h>
#include <math/quat.h>

using namespace rosflight_math;

namespace rosflight_firmware
{

class ROSflight;

class Estimator
{

public:
  struct State
  {
    Vec3 angular_velocity;
    Quat attitude;
    uint64_t timestamp_us;
  };

  Estimator(ROSflight& _rf);

  inline const State& state() const { return state_; }

  void init();
  void run();
  void reset_state();
  void reset_adaptive_bias();

private:
  const Vec3 g_ = {0.0f, 0.0f, -1.0f};

  ROSflight& RF_;
  State state_;

  uint64_t last_time_;
  uint64_t last_acc_update_us_;

  Vec3 w1_;
  Vec3 w2_;

  Vec3 bias_;

  Vec3 accel_LPF_;
  Vec3 gyro_LPF_;

  Vec3 w_acc_;

  void run_LPF();
};

} // namespace rosflight_firmware

#endif // ROSFLIGHT_FIRMWARE_ESTIMATOR_H

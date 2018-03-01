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
#include <map>

#include <turbomath/turbomath.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace rosflight_firmware
{

class ROSflight;

class Estimator
{

public:
  struct State
  {
    turbomath::Vector angular_velocity;
    turbomath::Quaternion attitude;
    float roll;
    float pitch;
    float yaw;
    uint64_t timestamp_us;
  };

  Estimator(ROSflight& _rf);

  inline const State& state() const { return state_; }

  void init();
  void run();
  void reset_state();
  void reset_adaptive_bias();
  
  typedef enum {
    ACC
  } measurement_type_t;

  const turbomath::Vector g_ = {0.0f, 0.0f, -1.0f};

  ROSflight& RF_;
  State state_;
  
  enum {
    xATT = 0,
    xB_G = 4,
    xZ = 7
  };
  
  enum {
    dxATT = 0,
    dxB_G = 3,
    dxZ = 6
  };
  
  enum {
    uG = 0,
    uZ = 3
  };
  
  typedef Eigen::Matrix<float, xZ, 1> xVector;
  typedef Eigen::Matrix<float, dxZ, 1> dxVector;
  typedef Eigen::Matrix<float, dxZ, dxZ> dxMatrix;
  typedef Eigen::Matrix<float, dxZ, uZ> dxuMatrix;
  typedef Eigen::Matrix<float, uZ, 1> uVector;
  typedef Eigen::Matrix<float, uZ, uZ> uMatrix;
  typedef Eigen::Matrix<float, 3, 1> hVector;
  typedef Eigen::Matrix<float, 3, dxZ> HMatrix;
  
  typedef void (Estimator::*measurement_function_ptr)(const xVector& x, hVector& h, HMatrix& H) const;
  
  
private:
  // State, Covariance and Process Noise Matrices
  xVector x_;
  uVector u_;
  dxMatrix P_;
  dxVector Qx_;
  uVector Qu_;
  
  // Matrix Workspace
  dxMatrix A_;
  dxuMatrix G_;
  dxVector dx_;
  xVector xp_;
  Eigen::Matrix<float, dxZ, 3>  K_;
  hVector zhat_;
  hVector z_;
  HMatrix H_;
  hVector R_acc_;
  Eigen::Matrix<float, 4, 1> q1_;
  
  uint64_t prev_t_us_;
  float dt_;
  
  const dxMatrix I_big_ = dxMatrix::Identity();

  turbomath::Vector accel_LPF_;
  turbomath::Vector gyro_LPF_;

  void run_LPF();
  
public:
  void dynamics(const xVector &x, const uVector &u, dxVector &xdot, dxMatrix &dfdx, dxuMatrix &dfdu);
  void dynamics(const xVector& x, const uVector &u);
  void boxplus(const xVector& x, const dxVector& dx, xVector& out);
  bool update(const hVector& z, const measurement_type_t& meas_type, const Eigen::Matrix3f &R);
  void h_acc(const xVector& x, hVector& h, HMatrix& H) const;
};

static std::map<Estimator::measurement_type_t, Estimator::measurement_function_ptr> measurement_functions = [] {
  std::map<Estimator::measurement_type_t, Estimator::measurement_function_ptr> tmp;
  tmp[Estimator::ACC] = &Estimator::h_acc;
  return tmp;
}();

} // namespace rosflight_firmware

#endif // ROSFLIGHT_FIRMWARE_ESTIMATOR_H

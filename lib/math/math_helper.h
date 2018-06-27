#pragma once

#include "math.h"
#include <math/defs.h>
#include <math/quat.h>

namespace rosflight_math
{

static const Eigen::Matrix<f_t, 2, 3> I_2x3 = [] {
  Eigen::Matrix<f_t, 2, 3> tmp;
  tmp << 1.0, 0, 0,
         0, 1.0, 0;
  return tmp;
}();

static const Mat33 I_3x3 = [] {
  Mat33 tmp;
  tmp << 1.0, 0, 0,
         0, 1.0, 0,
         0, 0, 1.0;
  return tmp;
}();

static const Eigen::Matrix<f_t, 2, 2> I_2x2 = [] {
  Eigen::Matrix<f_t, 2, 2> tmp;
  tmp << 1.0, 0,
         0, 1.0;
  return tmp;
}();


static const Vec3 e_x = [] {
  Vec3 tmp;
  tmp << 1.0, 0, 0;
  return tmp;
}();

static const Vec3 e_y = [] {
  Vec3 tmp;
  tmp << 0, 1.0, 0;
  return tmp;
}();

static const Vec3 e_z = [] {
  Vec3 tmp;
  tmp << 0, 0, 1.0;
  return tmp;
}();

// inverse of skew symmetric matrix
inline Vec3 vex(const Mat33 mat)
{
  Vec3 v;
  v << mat(0,2), mat(1,0), mat(2,1);
  return v;
}

// create skew symmetric matrix from vector
inline Mat33 skew(const Vec3 v)
{
  Mat33 mat;
  mat << 0.0, -v(2), v(1),
         v(2), 0.0, -v(0),
         -v(1), v(0), 0.0;
  return mat;
}

template <typename T>
int sign(T in)
{
  return (in >= 0) - (in < 0);
}

} // namespace rosflight_math

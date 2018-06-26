#ifndef ROSFLIGHT_FIRMWARE_MATH_DEFS_H
#define ROSFLIGHT_FIRMWARE_MATH_DEFS_H

#include "Eigen/Core"

namespace rosflight_math
{

// Use this to change all quaternion implemention to use float or f_t
#ifdef MATH_USE_f_t
    typedef double f_t;
#else
    typedef float f_t;
#endif
typedef Eigen::Matrix<f_t, 4, 1> Vec4;
typedef Eigen::Matrix<f_t, 3, 1> Vec3;
typedef Eigen::Matrix<f_t, 3, 3> Mat33;

#define DEG2RAD 0.01745327
#define RAD2DEG 57.295827

}

#endif // ROSFLIGHT_FIRMWARE_MATH_DEFS_H

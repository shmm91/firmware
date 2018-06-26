#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <math.h>
#include <iostream>
#include <math/math_helper.h>

using namespace Eigen;

namespace rosflight_math
{

class Quat
{

private:

public:
  Quat() {}
  Quat(Vec4 arr) : arr_(arr) {}
  Quat(f_t w, f_t x, f_t y, f_t z) : arr_(w, x, y, z) {}

  Vec4 arr_;
  
  inline f_t w() const { return arr_(0); }
  inline f_t x() const { return arr_(1); }
  inline f_t y() const { return arr_(2); }
  inline f_t z() const { return arr_(3); }
  inline void setW(f_t w) { arr_(0) = w; }
  inline void setX(f_t x) { arr_(1) = x; }
  inline void setY(f_t y) { arr_(2) = y; }
  inline void setZ(f_t z) { arr_(3) = z; }
  inline const Vec4& elements() const { return arr_;}

  Quat operator* (const Quat q) { return otimes(q); }
  Quat& operator *= (const Quat q)
  {
    arr_ <<  w() * q.w() - x() *q.x() - y() * q.y() - z() * q.z(),
             w() * q.x() + x() *q.w() + y() * q.z() - z() * q.y(),
             w() * q.y() - x() *q.z() + y() * q.w() + z() * q.x(),
             w() * q.z() + x() *q.y() - y() * q.x() + z() * q.w();
  }

  Quat& operator= (const Quat q) { arr_ = q.elements(); }
  Quat& operator= (const Vec4 in) {arr_ = in; }

  Quat operator+ (const Vec3 v) { return boxplus(v); }
  Quat& operator+= (const Vec3 v)
  {
    arr_ = boxplus(v).elements();
  }

  Vec3 operator- (const Quat q) {return boxminus(q);}

  static Quat exp(const Vec3 v)
  {
    f_t norm_v = v.norm();

    Vec4 q_arr;
    if (norm_v > 1e-4)
    {
      f_t v_scale = std::sin(norm_v/2.0)/norm_v;
      q_arr << std::cos(norm_v/2.0), v_scale*v(0), v_scale*v(1), v_scale*v(2);
    }
    else
    {
      q_arr << 1.0, v(0)/2.0, v(1)/2.0, v(2)/2.0;
      q_arr /= q_arr.norm();
    }
    return Quat(q_arr);
  }

  static Vec3 log(const Quat q)
  {
    Vec3 v = q.elements().block<3,1>(1, 0);
    f_t w = q.elements()(0,0);
    f_t norm_v = v.norm();

    Vec3 out;
    if (norm_v < 1e-8)
    {
      out.setZero();
    }
    else
    {
      out = 2.0*std::atan2(norm_v, w)*v/norm_v;
    }
    return out;
  }

  static Quat from_R(const Mat33 m)
  {
    Vec4 q;
    f_t tr = m.trace();

    if (tr > 0)
    {
      f_t S = std::sqrt(tr+1.0) * 2.;
      q << 0.25 * S,
           (m(1,2) - m(2,1)) / S,
           (m(2,0) - m(0,2)) / S,
           (m(0,1) - m(1,0)) / S;
    }
    else if ((m(0,0) > m(1,1)) && (m(0,0) > m(2,2)))
    {
      f_t S = std::sqrt(1.0 + m(0,0) - m(1,1) - m(2,2)) * 2.;
      q << (m(1,2) - m(2,1)) / S,
           0.25 * S,
           (m(1,0) + m(0,1)) / S,
           (m(2,0) + m(0,2)) / S;
    }
    else if (m(1,1) > m(2,2))
    {
      f_t S = std::sqrt(1.0 + m(1,1) - m(0,0) - m(2,2)) * 2.;
      q << (m(2,0) - m(0,2)) / S,
           (m(1,0) + m(0,1)) / S,
           0.25 * S,
           (m(2,1) + m(1,2)) / S;
    }
    else
    {
      f_t S = std::sqrt(1.0 + m(2,2) - m(0,0) - m(1,1)) * 2.;
      q << (m(0,1) - m(1,0)) / S,
           (m(2,0) + m(0,2)) / S,
           (m(2,1) + m(1,2)) / S,
           0.25 * S;
    }
    return Quat(q);
  }

  static Quat from_axis_angle(const Vec3 axis, const f_t angle)
  {
    f_t alpha_2 = angle/2.0;
    f_t sin_a2 = std::sin(alpha_2);
    Vec4 arr;
    arr << std::cos(alpha_2), axis(0)*sin_a2, axis(1)*sin_a2, axis(2)*sin_a2;
    arr /= arr.norm();
    return Quat(arr);
  }

  static Quat from_euler(const f_t roll, const f_t pitch, const f_t yaw)
  {
    f_t cp = std::cos(roll/2.0);
    f_t ct = std::cos(pitch/2.0);
    f_t cs = std::cos(yaw/2.0);
    f_t sp = std::sin(roll/2.0);
    f_t st = std::sin(pitch/2.0);
    f_t ss = std::sin(yaw/2.0);

    Vec4 arr;
    arr << cp*ct*cs + sp*st*ss,
           sp*ct*cs - cp*st*ss,
           cp*st*cs + sp*ct*ss,
           cp*ct*ss - sp*st*cs;
    return Quat(arr);
  }

  static Quat from_two_unit_vectors(const Vec3 u, const Vec3 v)
  {
    Vec4 q_arr;

    f_t d = u.dot(v);
    if (d < 1.0)
    {
      f_t invs = 1.0/std::sqrt((2.0*(1.0+d)));
      Vec3 xyz = skew(u)*v*invs;
      q_arr(0) = 0.5/invs;
      q_arr.block<3,1>(1,0)=xyz;
      q_arr /= q_arr.norm();
    }
    else
    {
      q_arr << 1, 0, 0, 0;
    }
    return Quat(q_arr);
  }

  static Quat Identity()
  {
    Vec4 q_arr;
    q_arr << 1.0, 0, 0, 0;
    return Quat(q_arr);
  }

  static Quat Random()
  {
    Vec4 q_arr;
    q_arr.setRandom();
    q_arr /= q_arr.norm();
    return Quat(q_arr);
  }

  Vec3 euler()
  {
    Vec3 out;
    out << std::atan2(2.0*(w()*x()+y()*z()), 1.0-2.0*(x()*x() + y()*y())),
        std::asin(2.0*(w()*y() - z()*x())),
        std::atan2(2.0*(w()*z()+x()*y()), 1.0-2.0*(y()*y() + z()*z()));
    return out;
  }
  
  f_t roll() const
  {
    return std::atan2(2.0*(w()*x()+y()*z()), 1.0-2.0*(x()*x() + y()*y()));
  }
  
  f_t pitch() const
  {
    return std::asin(2.0*(w()*y() - z()*x()));
  }
  
  f_t yaw() const
  {
    return std::atan2(2.0*(w()*z()+x()*y()), 1.0-2.0*(y()*y() + z()*z()));
  }

  Mat33 R()
  {
    f_t wx = w()*x();
    f_t wy = w()*y();
    f_t wz = w()*z();
    f_t xx = x()*x();
    f_t xy = x()*y();
    f_t xz = x()*z();
    f_t yy = y()*y();
    f_t yz = y()*z();
    f_t zz = z()*z();
    Mat33 out;
    out << 1. - 2.*yy - 2.*zz, 2.*xy + 2.*wz,      2.*xz - 2.*wy,
           2.*xy - 2.*wz,      1. - 2.*xx - 2.*zz, 2.*yz + 2.*wx,
           2.*xz + 2.*wy,      2.*yz - 2.*wx,      1. - 2.*xx - 2.*yy;
    return out;
  }

  Quat copy()
  {
    Vec4 tmp = arr_;
    return Quat(tmp);
  }

  Quat& normalize()
  {
    arr_ /= arr_.norm();
  }

  Matrix<f_t, 3, 2> f_trot(Matrix<f_t, 3, 2> v)
  {
    Matrix<f_t, 3, 2> out(3, v.cols());
    for (int i = 0; i < v.cols(); i++)
    {
       out.block<3,1>(0,i) = v.block<3,1>(0,i) + w() * (2.0 * arr_.block<3,1>(1,0).cross(v.block<3,1>(0,i))) + arr_.block<3,1>(1,0).cross((2.0 * arr_.block<3,1>(1,0).cross(v.block<3,1>(0,i))));
    }
    return out;
  }

  Matrix<f_t, 3, 2> f_tinvrot(Matrix<f_t, 3, 2> v)
  {
    Matrix<f_t, 3, 2> out(3, v.cols());
    Vec3 t;
    for (int i = 0; i < v.cols(); i++)
    {
       t = -2.0 * arr_.block<3,1>(1,0).cross(v.block<3,1>(0,i));
       out.block<3,1>(0,i) = v.block<3,1>(0,i) + w() * t - arr_.block<3,1>(1,0).cross(t);
    }
    return out;
  }


  // The same as R.T * v but faster
  Vec3 rota(Vec3 v)
  {
    Vec3 t = 2.0 * arr_.block<3,1>(1,0).cross(v);
    return v + w() * t + arr_.block<3,1>(1,0).cross(t);
  }

  // The same as R * v but faster
  Vec3 rotp(Vec3 v)
  {
    Vec3 t = -2.0 * arr_.block<3,1>(1,0).cross(v);
    return v + w() * t - arr_.block<3,1>(1,0).cross(t);
  }

  Quat& invert()
  {
    arr_.block<3,1>(1,0) *= -1.0;
  }

  Quat inverse() const
  {
    Vec4 tmp = arr_;
    tmp.block<3,1>(1,0) *= -1.0;
    return Quat(tmp);
  }

  Quat otimes(const Quat q)
  {
    Vec4 new_arr;
    new_arr <<  w() * q.w() - x() *q.x() - y() * q.y() - z() * q.z(),
                w() * q.x() + x() *q.w() + y() * q.z() - z() * q.y(),
                w() * q.y() - x() *q.z() + y() * q.w() + z() * q.x(),
                w() * q.z() + x() *q.y() - y() * q.x() + z() * q.w();
    return Quat(new_arr);
  }

  Quat boxplus(Vec3 delta)
  {
    return otimes(exp(delta));
  }
  
  Vec3 boxminus(Quat q)
  {
    Quat dq = q.inverse().otimes(*this);
    if (dq.w() < 0.0)
    {
      dq.arr_ *= -1.0;
    }
    return log(dq);
  }
};

inline std::ostream& operator<< (std::ostream& os, const Quat& q)
{
  os << "[ " << q.w() << ", " << q.x() << "i, " << q.y() << "j, " << q.z() << "k]";
  return os;
}

} // namespace rosflight_math


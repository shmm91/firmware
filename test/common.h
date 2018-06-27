#include <gtest/gtest.h>

#include <turbomath/turbomath.h>
#include <math/defs.h>
#include <math/quat.h>
#include <math/math_helper.h>

using namespace rosflight_math;

#define EXPECT_VEC3_SUPERCLOSE(vec, eig) \
  EXPECT_NEAR((vec).x(), (eig).x(), 0.0001);\
  EXPECT_NEAR((vec).y(), (eig).y(), 0.0001);\
  EXPECT_NEAR((vec).z(), (eig).z(), 0.0001)
#define EXPECT_QUAT_SUPERCLOSE(q, q_eig) { \
  double e1 = quaternion_error((q_eig), (q)); \
  Eigen::Quaternionf q_eig_neg = q_eig; \
  q_eig_neg.coeffs() *= -1.0; \
  double e2 = quaternion_error(q_eig_neg, (q)); \
  double error = (e1 < e2) ? e1 : e2; \
  EXPECT_LE(error, 0.0001); \
  }
#define ASSERT_QUAT_SUPERCLOSE(q, q_eig) { \
  double e1 = quaternion_error((q_eig), (q)); \
  Eigen::Quaternionf q_eig_neg = q_eig; \
  q_eig_neg.coeffs() *= -1.0; \
  double e2 = quaternion_error(q_eig_neg, (q)); \
  double error = (e1 < e2) ? e1 : e2; \
  EXPECT_LE(error, 0.0001); \
  }

#define ASSERT_TURBOQUAT_SUPERCLOSE(q, q2) { \
  double e1 = quaternion_error(q, q2); \
  Quat q2_neg = q2; \
  q2_neg.arr_ *= -1.0; \
  double e2 = quaternion_error(q, q2_neg); \
  double error = (e1 < e2) ? e1 : e2; \
  ASSERT_LE(error, 0.0001); \
  }


#define EXPECT_BASICALLYTHESAME(x, y) EXPECT_NEAR(x, y, 0.00001)
#define EXPECT_SUPERCLOSE(x, y) EXPECT_NEAR(x, y, 0.0001)
#define EXPECT_CLOSE(x, y) EXPECT_NEAR(x, y, 0.01)
#define EXPECT_INTHESAMEBALLPARK(x, y) EXPECT_NEAR(x, y, 0.1)

#define ASSERT_BASICALLYTHESAME(x, y) ASSERT_NEAR(x, y, 0.00001)
#define ASSERT_SUPERCLOSE(x, y) ASSERT_NEAR(x, y, 0.0001)
#define ASSERT_CLOSE(x, y) ASSERT_NEAR(x, y, 0.01)
#define ASSERT_INTHESAMEBALLPARK(x, y) ASSERT_NEAR(x, y, 0.1)

double quaternion_error(Quat q0, Quat q);
double quaternion_error(Eigen::Quaternionf q_eig, Quat q);

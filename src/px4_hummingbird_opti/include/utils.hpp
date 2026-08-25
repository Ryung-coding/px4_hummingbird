#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace utils {

inline Eigen::Matrix3d quatToRot(const Eigen::Vector4d& q_in)
{
  Eigen::Vector4d q = q_in;
  const double n = q.norm();

  if (n < 1.0e-9) {
    return Eigen::Matrix3d::Identity();
  }

  q /= n;

  const double w = q(0);
  const double x = q(1);
  const double y = q(2);
  const double z = q(3);

  Eigen::Matrix3d R;
  R << 1.0 - 2.0 * (y * y + z * z),       2.0 * (x * y - w * z),       2.0 * (x * z + w * y),
             2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z),       2.0 * (y * z - w * x),
             2.0 * (x * z - w * y),       2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y);
  return R;
}

inline Eigen::Vector4d rotToQuat(const Eigen::Matrix3d& R)
{
  Eigen::Quaterniond q(R);

  if (q.norm() < 1.0e-9) {
    return Eigen::Vector4d(1.0, 0.0, 0.0, 0.0);
  }

  q.normalize();
  return Eigen::Vector4d(q.w(), q.x(), q.y(), q.z());
}

struct Px4Pose
{
  Eigen::Vector3d pos;
  Eigen::Vector4d att;
};

inline Px4Pose mocapPoseToPx4(const Eigen::Vector3d& opti_pos, const Eigen::Vector4d& opti_att, const Eigen::Vector3d& opti_origin)
{
  Eigen::Matrix3d R_down;
  R_down << 1.0, 0.0, 0.0,
            0.0, -1.0, 0.0,
            0.0, 0.0, -1.0;

  const Eigen::Vector3d pos_px4 = R_down * (opti_pos - opti_origin);
  const Eigen::Matrix3d R_px4 = R_down * quatToRot(opti_att) * R_down;
  return Px4Pose{pos_px4, rotToQuat(R_px4)};
}

}  // namespace utils

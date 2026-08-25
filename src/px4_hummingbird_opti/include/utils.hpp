#pragma once

#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace utils {

// Math utils ========================================================
inline Eigen::Vector4d rpyToQuat(const Eigen::Vector3d& rpy)
{
  const double cr = std::cos(0.5 * rpy(0));
  const double sr = std::sin(0.5 * rpy(0));
  const double cp = std::cos(0.5 * rpy(1));
  const double sp = std::sin(0.5 * rpy(1));
  const double cy = std::cos(0.5 * rpy(2));
  const double sy = std::sin(0.5 * rpy(2));

  Eigen::Vector4d q;
  q << cr * cp * cy + sr * sp * sy,
       sr * cp * cy - cr * sp * sy,
       cr * sp * cy + sr * cp * sy,
       cr * cp * sy - sr * sp * cy;
  return q.normalized();
}

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

// Mocap utils =======================================================
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

// Rviz utils ========================================================
inline double wrapToPi(double angle)
{
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

inline double px4HeadingToRvizYaw(double heading_ned)
{
  return wrapToPi(0.5 * M_PI - heading_ned);
}

inline Eigen::Vector3d px4PosToRvizEnu(const Eigen::Vector3d& pos_ned)
{
  return Eigen::Vector3d(pos_ned.y(), pos_ned.x(), -pos_ned.z());
}

}  // namespace utils

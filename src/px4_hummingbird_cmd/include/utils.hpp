#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cmath>
#include <sstream>
#include <string>
#include <params.hpp>

namespace utils {

// struct.zip =======================================================
struct LPF {
  double y = 0.0;
  double alpha = 1.0;
  bool initialized = false;

  explicit LPF(double alpha_in = 1.0) : alpha(alpha_in) {}

  double update(double x)
  {
    if (!initialized) {
      y = x;
      initialized = true;
      return y;
    }

    y = alpha * x + (1.0 - alpha) * y;
    return y;
  }

  void reset(double x = 0.0)
  {
    y = x;
    initialized = true;
  }
};

struct TargetCMD {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
};

struct ManualHandoffFilters {
  LPF x{0.01};
  LPF y{0.01};
  LPF z{0.01};
  LPF roll{0.005};
  LPF pitch{0.005};
  LPF yaw{0.005};

  void reset(const TargetCMD& cmd)
  {
    x.reset(cmd.x);
    y.reset(cmd.y);
    z.reset(cmd.z);
    roll.reset(cmd.roll);
    pitch.reset(cmd.pitch);
    yaw.reset(cmd.yaw);
  }
};

// Math utils ========================================================
inline Eigen::Matrix3d hat(const Eigen::Vector3d& v)
{
  Eigen::Matrix3d m;
  m << 0.0, -v(2), v(1),
       v(2), 0.0, -v(0),
       -v(1), v(0), 0.0;
  return m;
}

inline Eigen::Vector3d vee(const Eigen::Matrix3d& m)
{
  Eigen::Vector3d v;
  v << m(2, 1), m(0, 2), m(1, 0);
  return v;
}

inline Eigen::Vector3d vec3(const std::array<double, 3>& a)
{
  return Eigen::Vector3d(a[0], a[1], a[2]);
}

inline Eigen::Matrix3d diag3(const std::array<double, 3>& a)
{
  Eigen::Matrix3d m;
  m << a[0], 0.0, 0.0,
       0.0, a[1], 0.0,
       0.0, 0.0, a[2];
  return m;
}

inline double meanAngle(double a, double b)
{
  return std::atan2(std::sin(a) + std::sin(b), std::cos(a) + std::cos(b));
}

inline Eigen::Vector3d clampVec3(const Eigen::Vector3d& x, const Eigen::Vector3d& lim)
{
  Eigen::Vector3d y;
  y << std::clamp(x(0), -lim(0), lim(0)),
       std::clamp(x(1), -lim(1), lim(1)),
       std::clamp(x(2), -lim(2), lim(2));
  return y;
}

inline Eigen::Matrix3d rpyToRot(const Eigen::Vector3d& rpy)
{
  const double r = rpy(0);
  const double p = rpy(1);
  const double y = rpy(2);

  const double sr = std::sin(r);
  const double cr = std::cos(r);
  const double sp = std::sin(p);
  const double cp = std::cos(p);
  const double sy = std::sin(y);
  const double cy = std::cos(y);

  Eigen::Matrix3d R;
  R << cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
       sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
           -sp,                  cp * sr,                  cp * cr;

  return R;
}

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

inline Eigen::Vector3d rotToRpy(const Eigen::Matrix3d& R)
{
  Eigen::Vector3d rpy;
  rpy << std::atan2(R(2, 1), R(2, 2)),
         std::asin(std::clamp(-R(2, 0), -1.0, 1.0)),
         std::atan2(R(1, 0), R(0, 0));
  return rpy;
}

inline Eigen::Matrix3d headingToRot(const Eigen::Vector3d& heading)
{
  Eigen::Vector3d b1 = heading;
  if (b1.norm() < 1.0e-6) b1 << 1.0, 0.0, 0.0;
  b1.normalize();

  Eigen::Vector3d b3;
  b3 << 0.0, 0.0, 1.0;

  Eigen::Vector3d b2 = b3.cross(b1);
  if (b2.norm() < 1.0e-6) b2 << 0.0, 1.0, 0.0;
  b2.normalize();

  b1 = b2.cross(b3);
  b1.normalize();

  Eigen::Matrix3d R;
  R.col(0) = b1;
  R.col(1) = b2;
  R.col(2) = b3;

  return R;
}

inline double unwrapNear(double target, double reference)
{
  while (target - reference > M_PI) target -= 2.0 * M_PI;
  while (target - reference < -M_PI) target += 2.0 * M_PI;
  return target;
}

inline TargetCMD manualHandoffCommand(const TargetCMD& last_cmd, double vehicle_x_ned, double vehicle_y_ned, double vehicle_z_ned, double vehicle_yaw_ned, double rc_roll, double rc_pitch, double rc_yaw, double position_offset_max, double yaw_offset_max, ManualHandoffFilters& filters)
{
  TargetCMD target;
  target.x = vehicle_x_ned + std::clamp(rc_pitch, -1.0, 1.0) * position_offset_max;
  target.y = vehicle_y_ned + std::clamp(rc_roll, -1.0, 1.0) * position_offset_max;
  target.z = -vehicle_z_ned;
  target.roll = 0.0;
  target.pitch = 0.0;
  target.yaw = unwrapNear(vehicle_yaw_ned + std::clamp(rc_yaw, -1.0, 1.0) * yaw_offset_max, last_cmd.yaw);

  TargetCMD cmd;
  cmd.x = filters.x.update(target.x);
  cmd.y = filters.y.update(target.y);
  cmd.z = filters.z.update(target.z);
  cmd.roll = filters.roll.update(target.roll);
  cmd.pitch = filters.pitch.update(target.pitch);
  cmd.yaw = filters.yaw.update(target.yaw);
  return cmd;
}

// Path utils =========================================================
inline TargetCMD trackApple(double t)
{
  TargetCMD cmd;

  const double w = 2.0 * M_PI / params::SCAN_PERIOD_SEC;
  const double yaw_phase = w * t;

  cmd.x = params::APPLE_X - params::RADIUS * std::cos(yaw_phase);
  cmd.y = params::APPLE_Y + params::RADIUS * std::sin(yaw_phase);
  cmd.z = params::APPLE_Z + params::RADIUS * std::sin(params::THETA_MAX) * std::sin(yaw_phase);

  const double dx = params::APPLE_X - cmd.x;
  const double dy = params::APPLE_Y - cmd.y;

  cmd.roll = 0.0;
  cmd.pitch = -params::THETA_MAX * std::sin(yaw_phase);
  cmd.yaw = std::atan2(dy, dx);

  return cmd;
}

inline TargetCMD takeApple(double t)
{
  TargetCMD cmd;

  const double cycle = params::SCAN_PERIOD_SEC;
  const double tc = std::fmod(t, cycle);

  const double tilt_sec = 0.25 * cycle;
  const double move_sec = 0.375 * cycle;
  const double theta = params::THETA_MAX;

  const double tilted_x = params::APPLE_X - params::RADIUS * std::cos(theta);
  const double tilted_z = params::APPLE_Z + params::RADIUS * std::sin(theta);

  const double approach_dist = params::RADIUS * 0.5;
  const double near_x = tilted_x + approach_dist * std::cos(theta);
  const double near_z = tilted_z - approach_dist * std::sin(theta);

  cmd.y = params::APPLE_Y;
  cmd.roll = 0.0;
  cmd.yaw = 0.0;

  if (tc < tilt_sec) {
    const double a = tc / tilt_sec;
    const double s = a * a * (3.0 - 2.0 * a);
    const double th = theta * s;

    cmd.x = params::APPLE_X - params::RADIUS * std::cos(th);
    cmd.z = params::APPLE_Z + params::RADIUS * std::sin(th);
    cmd.pitch = -th;

    return cmd;
  }

  if (tc < tilt_sec + move_sec) {
    const double a = (tc - tilt_sec) / move_sec;
    const double s = a * a * (3.0 - 2.0 * a);

    cmd.x = tilted_x + (near_x - tilted_x) * s;
    cmd.z = tilted_z + (near_z - tilted_z) * s;
    cmd.pitch = -theta;

    return cmd;
  }

  const double a = (tc - tilt_sec - move_sec) / move_sec;
  const double s = a * a * (3.0 - 2.0 * a);

  cmd.x = near_x + (tilted_x - near_x) * s;
  cmd.z = near_z + (tilted_z - near_z) * s;
  cmd.pitch = -theta;

  return cmd;
}

inline TargetCMD positionTuningPath(double t)
{
  static constexpr double SEG_SEC = 5.0;
  static constexpr double XY = 1.0;

  TargetCMD cmd;

  const double tm = t;
  const int phase = static_cast<int>(std::floor(tm / SEG_SEC)) % 4;
  const double a = std::fmod(tm, SEG_SEC) / SEG_SEC;
  const double s = a * a * (3.0 - 2.0 * a);

  const std::array<Eigen::Vector2d, 4> corners = {
    Eigen::Vector2d{XY, 0.0},
    Eigen::Vector2d{XY, XY},
    Eigen::Vector2d{0.0, XY},
    Eigen::Vector2d{0.0, 0.0}
  };

  const Eigen::Vector2d p0 = (phase == 0) ? Eigen::Vector2d{0.0, 0.0} : corners[phase - 1];
  const Eigen::Vector2d p1 = corners[phase];

  cmd.x = p0(0) + (p1(0) - p0(0)) * s;
  cmd.y = p0(1) + (p1(1) - p0(1)) * s;
  cmd.z = 0.0;
  cmd.roll = 0.0;
  cmd.pitch = 0.0;
  cmd.yaw = 0.0;

  return cmd;
}

inline TargetCMD attitudeTuningPath(double t)
{
  static constexpr double TUNE_SEC = 5.0;
  static constexpr double PITCH_AMP = 60.0 * M_PI / 180.0;

  TargetCMD cmd;

  const double tm = std::fmod(t, TUNE_SEC);
  const double axis_t = tm;
  const double w = 2.0 * M_PI / TUNE_SEC;

  cmd.x = 0.0;
  cmd.y = 0.0;
  cmd.z = 0.0;
  cmd.roll = 0.0;
  cmd.pitch = 0.0;
  cmd.yaw = 0.0;
  cmd.pitch = PITCH_AMP * std::sin(w * axis_t);
  // if (tm < TUNE_SEC) {
  //   cmd.pitch = PITCH_AMP * std::sin(w * axis_t);
  // } else if (tm < 2.0 * TUNE_SEC) {
  //   cmd.yaw = YAW_AMP * std::sin(w * axis_t);
  // } else {
  //   cmd.roll = ROLL_AMP * std::sin(w * axis_t);
  // }

  return cmd;
}

inline TargetCMD steppedAttitudePath(double t)
{
  static constexpr double HOVER_SEC = 2.0;
  static constexpr double ZERO_HOLD_SEC = 2.0;
  static constexpr double RAMP_SEC = 1.0;
  static constexpr double HOLD_SEC = 3.0;
  static constexpr double Z = 0.0;

  static constexpr double SWITCH_DEG = 60.0;
  static constexpr double FIRST_STEP_DEG = 10.0;
  static constexpr double SECOND_STEP_DEG = 1.0;
  static constexpr double MAX_DEG = 90.0;
  static constexpr double DEG2RAD = M_PI / 180.0;

  static constexpr int FIRST_STAGE_COUNT =
      static_cast<int>(SWITCH_DEG / FIRST_STEP_DEG);

  TargetCMD cmd;

  cmd.x = 0.0;
  cmd.y = 0.0;
  cmd.z = Z;
  cmd.roll = 0.0;
  cmd.pitch = 0.0;
  cmd.yaw = 0.0;

  if (t < HOVER_SEC) {
    const double a = t / HOVER_SEC;
    const double s = a * a * (3.0 - 2.0 * a);

    cmd.z = Z * s;

    return cmd;
  }

  const double tm = t - HOVER_SEC;

  if (tm < ZERO_HOLD_SEC) {
    return cmd;
  }

  const double ts = tm - ZERO_HOLD_SEC;
  const double stage_sec = RAMP_SEC + HOLD_SEC;
  const int stage = static_cast<int>(std::floor(ts / stage_sec));
  const double stage_t = std::fmod(ts, stage_sec);

  double start_deg;
  double target_deg;

  if (stage < FIRST_STAGE_COUNT) {
    start_deg = stage * FIRST_STEP_DEG;
    target_deg = start_deg + FIRST_STEP_DEG;
  } else {
    const int second_stage = stage - FIRST_STAGE_COUNT;

    start_deg = SWITCH_DEG + second_stage * SECOND_STEP_DEG;
    target_deg = start_deg + SECOND_STEP_DEG;
  }

  start_deg = std::min(start_deg, MAX_DEG);
  target_deg = std::min(target_deg, MAX_DEG);

  const double start_angle = start_deg * DEG2RAD;
  const double target_angle = target_deg * DEG2RAD;

  if (start_deg >= MAX_DEG) {
    cmd.pitch = MAX_DEG * DEG2RAD;
    return cmd;
  }

  if (stage_t < RAMP_SEC) {
    const double a = stage_t / RAMP_SEC;
    const double s = a * a * (3.0 - 2.0 * a);

    cmd.pitch = start_angle + (target_angle - start_angle) * s;
  } else {
    cmd.pitch = target_angle;
  }

  return cmd;
}

inline TargetCMD agilePath(double t)
{
  TargetCMD cmd;

  const double tm = t;
  const double w = 2.0 * M_PI / params::SCAN_PERIOD_SEC;
  const double p = w * tm;

  cmd.x = params::RADIUS * std::sin(p);
  cmd.y = params::RADIUS * std::sin(p) * std::cos(p);
  cmd.z = params::RADIUS * std::sin(params::THETA_MAX) * std::sin(p);

  cmd.roll = params::ROLL_MAX * std::sin(p);
  cmd.pitch = params::THETA_MAX * std::sin(p + 0.5 * M_PI);
  cmd.yaw = 0.5 * M_PI * std::sin(p);

  return cmd;
}

inline TargetCMD positionTrack(double t)
{
  static constexpr double distance_x = 1.0; // [m]
  static constexpr double distance_y = 1.0; // [m]

  static constexpr double vel_x = 1.0; // [m/s]
  static constexpr double vel_y = 1.0; // [m/s]

  const double X_SEG_SEC = distance_x / vel_x;
  const double Y_SEG_SEC = distance_y / vel_y;

  TargetCMD cmd;

  const double tm = t;
  const double cycle = 2.0 * X_SEG_SEC + 2.0 * Y_SEG_SEC;
  const double tc = std::fmod(tm, cycle);

  cmd.x = 0.0;
  cmd.y = 0.0;
  cmd.z = 0.0;
  cmd.roll = 0.0;
  cmd.pitch = 0.0;
  cmd.yaw = 0.0;

  if (tc < X_SEG_SEC) {
    const double a = tc / X_SEG_SEC;
    const double s = a * a * (3.0 - 2.0 * a);

    cmd.x = distance_x * s;
    cmd.y = 0.0;

    return cmd;
  }

  if (tc < X_SEG_SEC + Y_SEG_SEC) {
    const double a = (tc - X_SEG_SEC) / Y_SEG_SEC;
    const double s = a * a * (3.0 - 2.0 * a);

    cmd.x = distance_x;
    cmd.y = distance_y * s;

    return cmd;
  }

  if (tc < 2.0 * X_SEG_SEC + Y_SEG_SEC) {
    const double a = (tc - X_SEG_SEC - Y_SEG_SEC) / X_SEG_SEC;
    const double s = a * a * (3.0 - 2.0 * a);

    cmd.x = distance_x * (1.0 - s);
    cmd.y = distance_y;

    return cmd;
  }

  {
    const double a = (tc - 2.0 * X_SEG_SEC - Y_SEG_SEC) / Y_SEG_SEC;
    const double s = a * a * (3.0 - 2.0 * a);

    cmd.x = 0.0;
    cmd.y = distance_y * (1.0 - s);

    return cmd;
  }
}

inline TargetCMD hover(double)
{
  TargetCMD cmd;
  cmd.x = 0.0;
  cmd.y = 0.0;
  cmd.z = 0.0;
  cmd.roll = 0.0;
  cmd.pitch = 0.0;
  cmd.yaw = 0.0;
  return cmd;
}

}

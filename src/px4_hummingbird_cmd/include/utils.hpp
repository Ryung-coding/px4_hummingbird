#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cmath>
#include <params.hpp>

namespace utils {

// struct.zip =======================================================
struct LPF {
  double y = 0.0;
  double alpha = 1.0;
  bool initialized = false;

  explicit LPF(double convergence_time_sec = 0.0)
  {
    if (convergence_time_sec <= 0.0)
    {
      alpha = 1.0;
      return;
    }

    const double sample_count = convergence_time_sec * static_cast<double>(params::RATE_HZ);
    alpha = 1.0 - std::pow(0.01, 1.0 / sample_count);
  }

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

struct HandoffFilters {
  LPF x{params::handoff_runtime};
  LPF y{params::handoff_runtime};
  LPF z{params::handoff_runtime};
  LPF roll{params::handoff_runtime};
  LPF pitch{params::handoff_runtime};
  LPF yaw{params::handoff_runtime};

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

inline double unwrapNear(double target, double reference)
{
  while (target - reference > M_PI) target -= 2.0 * M_PI;
  while (target - reference < -M_PI) target += 2.0 * M_PI;
  return target;
}

inline double angleErrorAbs(double actual, double target)
{
  return std::abs(unwrapNear(actual, target) - target);
}

inline Eigen::Vector3d quatToRpy(const Eigen::Vector4d& q)
{
  const double w = q(0);
  const double x = q(1);
  const double y = q(2);
  const double z = q(3);

  const double sinr_cosp = 2.0 * (w * x + y * z);
  const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
  const double roll = std::atan2(sinr_cosp, cosr_cosp);

  const double sinp = 2.0 * (w * y - z * x);
  const double pitch = std::abs(sinp) >= 1.0 ? std::copysign(M_PI / 2.0, sinp) : std::asin(sinp);

  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  const double yaw = std::atan2(siny_cosp, cosy_cosp);

  return Eigen::Vector3d(roll, pitch, yaw);
}

inline TargetCMD initialPose()
{
  TargetCMD cmd;
  cmd.x = params::initial_pose_x_m;
  cmd.y = params::initial_pose_y_m;
  cmd.z = params::initial_pose_z_m;
  cmd.roll = params::initial_pose_roll_rad;
  cmd.pitch = params::initial_pose_pitch_rad;
  cmd.yaw = params::initial_pose_yaw_rad;
  return cmd;
}

inline TargetCMD RC2DDS_handoff(const TargetCMD& initial_pose, HandoffFilters& filters)
{
  TargetCMD target = initial_pose;
  target.yaw = unwrapNear(target.yaw, filters.yaw.y);

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
inline TargetCMD posPath(double t)
{
  static constexpr double SEG_SEC = 10.0;
  static constexpr double XY = 0.5;

  TargetCMD cmd;

  const double tm = t;
  const int phase = static_cast<int>(std::floor(tm / SEG_SEC)) % 4;
  const double a = std::fmod(tm, SEG_SEC) / SEG_SEC;
  const double s = a * a * (3.0 - 2.0 * a);

  const std::array<Eigen::Vector2d, 4> corners =
  {
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

inline TargetCMD attPath(double t)
{
  static constexpr double TUNE_SEC = 30.0;

  TargetCMD cmd;

  const double tm = std::fmod(t, TUNE_SEC);
  const double axis_t = tm;
  const double w = 2.0 * M_PI / TUNE_SEC;

  cmd.x = 0.0;
  cmd.y = 0.0;
  cmd.z = 0.0;
  cmd.roll = 0.0;
  cmd.pitch = params::PITCH_MAX * std::sin(w * axis_t);
  cmd.yaw = 0.0;

  return cmd;
}

inline TargetCMD stepAttPath(double t)
{
  static constexpr double ZERO_HOLD_SEC = 2.0;
  static constexpr double RAMP_SEC = 3.0;
  static constexpr double HOLD_SEC = 5.0;

  static constexpr double SWITCH_DEG = 60.0;
  static constexpr double FIRST_STEP_DEG = 5.0;
  static constexpr double SECOND_STEP_DEG = 1.0;
  static constexpr double MAX_DEG = 60.0;
  static constexpr double DEG2RAD = M_PI / 180.0;

  static constexpr int FIRST_STAGE_COUNT = static_cast<int>(SWITCH_DEG / FIRST_STEP_DEG);

  TargetCMD cmd;

  cmd.x = 0.0;
  cmd.y = 0.0;
  cmd.z = 0.0;
  cmd.roll = 0.0;
  cmd.pitch = 0.0;
  cmd.yaw = 0.0;

  const double stage_sec = RAMP_SEC + HOLD_SEC;
  const int UP_STAGE_COUNT = FIRST_STAGE_COUNT + static_cast<int>((MAX_DEG - SWITCH_DEG) / SECOND_STEP_DEG);
  const double cycle_sec = ZERO_HOLD_SEC + 2.0 * UP_STAGE_COUNT * stage_sec + ZERO_HOLD_SEC;
  const double cycle_t = std::fmod(t, cycle_sec);

  if (cycle_t < ZERO_HOLD_SEC) return cmd;

  const double ts = cycle_t - ZERO_HOLD_SEC;
  if (ts >= 2.0 * UP_STAGE_COUNT * stage_sec) return cmd;

  const int raw_stage = static_cast<int>(std::floor(ts / stage_sec));
  const double stage_t = std::fmod(ts, stage_sec);
  const bool down = raw_stage >= UP_STAGE_COUNT;
  const int stage = down ? 2 * UP_STAGE_COUNT - raw_stage - 1 : raw_stage;

  double start_deg;
  double target_deg;

  if (stage < FIRST_STAGE_COUNT)
  {
    start_deg = stage * FIRST_STEP_DEG;
    target_deg = start_deg + FIRST_STEP_DEG;
  }
  else
  {
    const int second_stage = stage - FIRST_STAGE_COUNT;

    start_deg = SWITCH_DEG + second_stage * SECOND_STEP_DEG;
    target_deg = start_deg + SECOND_STEP_DEG;
  }

  start_deg = std::min(start_deg, MAX_DEG);
  target_deg = std::min(target_deg, MAX_DEG);

  if (down) std::swap(start_deg, target_deg);

  const double start_angle = start_deg * DEG2RAD;
  const double target_angle = target_deg * DEG2RAD;

  if (stage_t < RAMP_SEC)
  {
    const double a = stage_t / RAMP_SEC;
    const double s = a * a * (3.0 - 2.0 * a);

    cmd.pitch = start_angle + (target_angle - start_angle) * s;
  }
  else cmd.pitch = target_angle;

  return cmd;
}

}  // namespace utils

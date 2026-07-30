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
  LPF x{params::handoff_run_time_sec};
  LPF y{params::handoff_run_time_sec};
  LPF z{params::handoff_run_time_sec};
  LPF roll{params::handoff_run_time_sec};
  LPF pitch{params::handoff_run_time_sec};
  LPF yaw{params::handoff_run_time_sec};

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

inline TargetCMD DDS2manual_handoff(const TargetCMD& last_cmd, double vehicle_x_ned, double vehicle_y_ned, double vehicle_z_ned, double vehicle_yaw_ned, double rc_roll, double rc_pitch, double rc_yaw, double position_offset_max, double yaw_offset_max, HandoffFilters& filters)
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
inline TargetCMD positionTuningPath(double t)
{
  static constexpr double SEG_SEC = 5.0;
  static constexpr double XY = 1.0;

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

inline TargetCMD attitudeTuningPath(double t)
{
  static constexpr double SINE_SEC = 50.0;
  static constexpr double HOLD_SEC = 10.0;
  static constexpr double TOTAL_SEC = SINE_SEC + HOLD_SEC;
  static constexpr double PEAK_TIME = SINE_SEC / 4.0;

  TargetCMD cmd;

  const double tm = std::fmod(t, TOTAL_SEC);
  const double w = 2.0 * M_PI / SINE_SEC;

  double attitude_scale = 0.0;

  if (tm < PEAK_TIME) attitude_scale = std::sin(w * tm);
  else if (tm < PEAK_TIME + HOLD_SEC) attitude_scale = 1.0;
  else 
  {
    const double sine_time = tm - HOLD_SEC;
    attitude_scale = std::sin(w * sine_time);
  }

  cmd.x = 0.0;
  cmd.y = 0.0;
  cmd.z = 0.0;

  cmd.roll = params::ROLL_MAX * attitude_scale;
  cmd.pitch = params::PITCH_MAX * attitude_scale;
  cmd.yaw = 0.0;

  return cmd;
}

inline TargetCMD steppedAttitudePath(double t)
{
  static constexpr double ZERO_HOLD_SEC = 2.0;
  static constexpr double RAMP_SEC = 1.0;
  static constexpr double HOLD_SEC = 1.0;

  static constexpr double SWITCH_DEG = 70.0;
  static constexpr double FIRST_STEP_DEG = 10.0;
  static constexpr double SECOND_STEP_DEG = 1.0;
  static constexpr double MAX_DEG = 90.0;
  static constexpr double DEG2RAD = M_PI / 180.0;

  static constexpr int FIRST_STAGE_COUNT = static_cast<int>(SWITCH_DEG / FIRST_STEP_DEG);

  TargetCMD cmd;

  cmd.x = 0.0;
  cmd.y = 0.0;
  cmd.z = 0.0;
  cmd.roll = 0.0;
  cmd.pitch = 0.0;
  cmd.yaw = 0.0;

  if (t < ZERO_HOLD_SEC) return cmd;

  const double ts = t - ZERO_HOLD_SEC;
  const double stage_sec = RAMP_SEC + HOLD_SEC;
  const int stage = static_cast<int>(std::floor(ts / stage_sec));
  const double stage_t = std::fmod(ts, stage_sec);

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

  const double start_angle = start_deg * DEG2RAD;
  const double target_angle = target_deg * DEG2RAD;

  if (start_deg >= MAX_DEG) 
  {
    cmd.pitch = MAX_DEG * DEG2RAD;
    return cmd;
  }

  if (stage_t < RAMP_SEC) 
  {
    const double a = stage_t / RAMP_SEC;
    const double s = a * a * (3.0 - 2.0 * a);

    cmd.pitch = start_angle + (target_angle - start_angle) * s;
  } 
  else cmd.pitch = target_angle;

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
  cmd.z = params::RADIUS * std::sin(params::PITCH_MAX) * std::sin(p);

  cmd.roll = params::ROLL_MAX * std::sin(p);
  cmd.pitch = params::PITCH_MAX * std::sin(p + 0.5 * M_PI);
  cmd.yaw = params::YAW_MAX * std::sin(p);

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

  if (tc < X_SEG_SEC) 
  {
    const double a = tc / X_SEG_SEC;
    const double s = a * a * (3.0 - 2.0 * a);

    cmd.x = distance_x * s;
    cmd.y = 0.0;

    return cmd;
  }

  if (tc < X_SEG_SEC + Y_SEG_SEC) 
  {
    const double a = (tc - X_SEG_SEC) / Y_SEG_SEC;
    const double s = a * a * (3.0 - 2.0 * a);

    cmd.x = distance_x;
    cmd.y = distance_y * s;

    return cmd;
  }

  if (tc < 2.0 * X_SEG_SEC + Y_SEG_SEC) 
  {
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

#pragma once

#include <cmath>
#include <array>
#include <cstdint>

namespace params {

// System Setting -----------------------------------------------
static constexpr bool USE_SO3_HEADING_CMD = false;
static constexpr int RATE_HZ = 400;                 // [Hz] DDS로 통신보내는 주기로 관측 후 설정최고치로 잡기

inline static constexpr double command_delay_sec = 2.0;
inline static constexpr double command_repeat_sec = 1.0;
inline static constexpr double message_timeout_sec = 1.0;

inline static constexpr double handoff_run_time_sec = 5.0;
inline static constexpr double handoff_position_offset_m = 0.6;
inline static constexpr double handoff_yaw_offset_rad = 0.35;

inline static constexpr uint8_t target_system_id = 1;
inline static constexpr uint8_t target_component_id = 1;
inline static constexpr uint8_t source_system_id = 1;
inline static constexpr uint16_t source_component_id = 1;

// Task parameters-----------------------------------------------
static constexpr double HOVER_SEC = 3.0;
static constexpr double SCAN_PERIOD_SEC = 30.0;

static constexpr double RADIUS = 1.5;
static constexpr double ROLL_MAX = 30.0 * M_PI / 180.0;
static constexpr double THETA_MAX = 60.0 * M_PI / 180.0;
static constexpr double YAW_MAX = 60.0 * M_PI / 180.0;

// Sim_servo parameters -----------------------------------------------
static constexpr double THETA_LIMIT_RAD = M_PI;
static constexpr double PHI_LIMIT_RAD = M_PI / 6.0;

static constexpr double SERVO_TIMEOUT_SEC = 0.2;
static constexpr double SERVO_RETURN_TIME_SEC = 5.0;

static constexpr int SERVO_PERIOD_US = static_cast<int>(1000000.0 / RATE_HZ);

// ActuatorServos control[0~3] = theta1~theta4
// ActuatorServos control[4~7] = phi1~phi4
static constexpr std::array<const char *, 8> SERVO_TOPICS =
{
  "joint_theta1", "joint_theta2", "joint_theta3", "joint_theta4",
  "joint_phi1", "joint_phi2", "joint_phi3", "joint_phi4"
};

// Real_servo parameters -----------------------------------------------

}

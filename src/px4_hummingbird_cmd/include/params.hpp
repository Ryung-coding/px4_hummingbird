#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace params {

// System Setting -----------------------------------------------
static constexpr bool USE_SO3_HEADING_CMD = false;
static constexpr int RATE_HZ = 400;

static constexpr double command_delay_sec = 2.0;
static constexpr double command_repeat_sec = 1.0;
static constexpr double message_timeout_sec = 1.0;

static constexpr double handoff_run_time_sec = 5.0;
static constexpr double handoff_position_offset_m = 0.6;
static constexpr double handoff_yaw_offset_rad = 0.35;

static constexpr uint8_t target_system_id = 1;
static constexpr uint8_t target_component_id = 1;
static constexpr uint8_t source_system_id = 1;
static constexpr uint16_t source_component_id = 1;

// Task parameters -----------------------------------------------
static constexpr double HOVER_SEC = 3.0;
static constexpr double SCAN_PERIOD_SEC = 30.0;

static constexpr double RADIUS = 1.5;
static constexpr double ROLL_MAX = 20.0 * M_PI / 180.0;
static constexpr double PITCH_MAX = 60.0 * M_PI / 180.0;
static constexpr double YAW_MAX = 60.0 * M_PI / 180.0;

// Common servo parameters -----------------------------------------------
static constexpr double THETA_LIMIT_RAD = M_PI;
static constexpr double PHI_LIMIT_RAD = M_PI / 6.0;

static constexpr double SERVO_TIMEOUT_SEC = 0.2;
static constexpr double SERVO_RETURN_TIME_SEC = 5.0;
static constexpr int SERVO_PERIOD_US = static_cast<int>(1000000.0 / RATE_HZ); // not tunable

static constexpr char DXL_PORT_NAME[] = "/dev/dynamixel";

static constexpr int DXL_BAUDRATE = 1000000;                // 1 Mbps setting1
static constexpr double STARTUP_RETURN_TIME_SEC = 2.0;
static constexpr int FINAL_ZERO_REPEAT_COUNT = 5;
static constexpr int FINAL_ZERO_REPEAT_INTERVAL_MS = 5;

static constexpr float DXL_PROTOCOL_VERSION = 2.0F;         //do not change
static constexpr int PORT_OPEN_RETRY_COUNT = 10;            //do not change
static constexpr int PING_RETRY_COUNT = 3;                  //do not change
static constexpr int REGISTER_WRITE_RETRY_COUNT = 3;        //do not change
static constexpr int PORT_RETRY_INTERVAL_MS = 500;          //do not change
static constexpr int REGISTER_RETRY_INTERVAL_MS = 20;       //do not change
static constexpr uint16_t ADDR_OPERATING_MODE = 11;         //do not change
static constexpr uint16_t ADDR_TORQUE_ENABLE = 64;          //do not change
static constexpr uint16_t ADDR_VELOCITY_I_GAIN = 76;        //do not change
static constexpr uint16_t ADDR_VELOCITY_P_GAIN = 78;        //do not change
static constexpr uint16_t ADDR_POSITION_D_GAIN = 80;        //do not change
static constexpr uint16_t ADDR_POSITION_I_GAIN = 82;        //do not change
static constexpr uint16_t ADDR_POSITION_P_GAIN = 84;        //do not change
static constexpr uint16_t ADDR_GOAL_POSITION = 116;         //do not change
static constexpr uint16_t ADDR_PRESENT_POSITION = 132;      //do not change
static constexpr uint8_t POSITION_CONTROL_MODE = 3;         //do not change
static constexpr uint8_t TORQUE_OFF = 0;                    //do not change
static constexpr uint8_t TORQUE_ON = 1;                     //do not change
static constexpr double PPR_PER_RAD = 4096.0 / (2.0 * M_PI);//do not change

struct ServoConfig
{
  uint8_t id;
  int direction;
  int32_t zero_ppr;
  int32_t min_ppr;
  int32_t max_ppr;
  double angle_limit_rad;

  uint16_t position_p;
  uint16_t position_i;
  uint16_t position_d;
  
  uint16_t velocity_p;
  uint16_t velocity_i;
};

  // control[0~1]: theta1~theta2
  // control[2~5]: phi1~phi4
static constexpr std::array<ServoConfig, 6> DXL_SERVOS{{
  {0, 1, 2048, 0, 4095, THETA_LIMIT_RAD, 1200, 0, 20, 150, 400},
  {1, -1, 2048, 0, 4095, THETA_LIMIT_RAD, 1200, 0, 20, 150, 400},
  {2, 1, 2048, 0, 4095, PHI_LIMIT_RAD,    800, 0,  0, 100, 300},
  {3, 1, 2048, 0, 4095, PHI_LIMIT_RAD,    800, 0,  0, 100, 300},
  {4, 1, 2048, 0, 4095, PHI_LIMIT_RAD,    800, 0,  0, 100, 300},
  {5, 1, 2048, 0, 4095, PHI_LIMIT_RAD,    800, 0,  0, 100, 300}
}};

}  // namespace params
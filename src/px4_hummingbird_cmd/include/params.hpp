#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace params {

// System Setting -----------------------------------------------
static constexpr bool USE_SO3_HEADING_CMD = false;
static constexpr int RATE_HZ = 400;

static constexpr double command_delay_sec = 0.5;      // offboard wating time after changing control mode
static constexpr double command_repeat_sec = 1.0;     // offboard/arm command repeat interval
static constexpr double message_timeout_sec = 1.0;    // offboard/arm command timeout for checking vehicle_status

static constexpr uint8_t target_system_id = 1;
static constexpr uint8_t target_component_id = 1;
static constexpr uint8_t source_system_id = 1;
static constexpr uint16_t source_component_id = 1;

// Planning Setting -----------------------------------------------
static constexpr double handoff_runtime = 8.0;

// Command frame: local NED position, yaw as NED heading.
static constexpr double initial_pose_x_m = 0.0;
static constexpr double initial_pose_y_m = 0.0;
static constexpr double initial_pose_z_m = -1.0;
static constexpr double initial_pose_roll_rad = 0.0;
static constexpr double initial_pose_pitch_rad = 0.0;
static constexpr double initial_pose_yaw_rad = 0.0;

static constexpr double pos_tol = 0.15;
static constexpr double yaw_tol = 5.0 * M_PI / 180.0;
static constexpr double att_tol = 5.0 * M_PI / 180.0;

// Task parameters -----------------------------------------------
static constexpr double ROLL_MAX = 20.0 * M_PI / 180.0;
static constexpr double PITCH_MAX = 60.0 * M_PI / 180.0;

// Common servo parameters -----------------------------------------------
static constexpr double BETA_LIMIT_RAD = M_PI;
static constexpr double ALPHA_LIMIT_RAD = M_PI / 6.0;

static constexpr double SERVO_TIMEOUT_SEC = 0.2;
static constexpr double SERVO_RETURN_TIME_SEC = 5.0;
static constexpr int SERVO_PERIOD_US = static_cast<int>(1000000.0 / RATE_HZ); // not tunable

// Sim
// ActuatorServos control[0~1] = beta_1~beta_2
// ActuatorServos control[2~5] = alpha_1~alpha_4
// Gazebo keeps four beta joints: beta_1 drives front pair 1/4, beta_2 drives rear pair 2/3.
static constexpr std::array<const char *, 8> SERVO_TOPICS{
  "joint_beta1", "joint_beta2", "joint_beta3", "joint_beta4",
  "joint_alpha1", "joint_alpha2", "joint_alpha3", "joint_alpha4"
};

static constexpr std::array<std::size_t, SERVO_TOPICS.size()> SERVO_CONTROL_INDEX{0, 1, 1, 0, 2, 3, 4, 5};

static constexpr std::array<double, SERVO_TOPICS.size()> SERVO_LIMIT_RAD{
  BETA_LIMIT_RAD, BETA_LIMIT_RAD, BETA_LIMIT_RAD, BETA_LIMIT_RAD,
  ALPHA_LIMIT_RAD, ALPHA_LIMIT_RAD, ALPHA_LIMIT_RAD, ALPHA_LIMIT_RAD
};

// Real 
// Dynamixel communication
static constexpr char DXL_PORT_NAME[] = "/dev/ttyUSB0";

static constexpr int DXL_BAUDRATE = 1000000;                // 1 Mbps setting
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

  // control[0~1]: beta_1~beta_2
  // control[2~5]: alpha_1~alpha_4
static constexpr std::array<ServoConfig, 6> DXL_SERVOS{{
  {0, 1, 2048, 0, 4095, BETA_LIMIT_RAD,  1200, 0, 20, 150, 400},
  {1, -1, 2048, 0, 4095, BETA_LIMIT_RAD, 1200, 0, 20, 150, 400},
  {2, 1, 2048, 0, 4095, ALPHA_LIMIT_RAD,  800, 0,  0, 100, 300},
  {3, 1, 2048, 0, 4095, ALPHA_LIMIT_RAD,  800, 0,  0, 100, 300},
  {4, 1, 2048, 0, 4095, ALPHA_LIMIT_RAD,  800, 0,  0, 100, 300},
  {5, 1, 2048, 0, 4095, ALPHA_LIMIT_RAD,  800, 0,  0, 100, 300}
}};

}  // namespace params

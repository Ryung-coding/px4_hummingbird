#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <thread>

#include <dynamixel_sdk/dynamixel_sdk.h>

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/actuator_servos.hpp>

#include "params.hpp"
#include "utils.hpp"

class Px4ServoToDynamixel : public rclcpp::Node
{
public:
  Px4ServoToDynamixel() : Node("px4_servo_to_dynamixel")
  {
    std::array<int32_t, params::DXL_SERVOS.size()> initial_ppr{};

    if (!initial_setting(initial_ppr) || !initial_zero(initial_ppr))
    {
      send_zero(params::FINAL_ZERO_REPEAT_COUNT, params::FINAL_ZERO_REPEAT_INTERVAL_MS);
      if (port_handler_) port_handler_->closePort();
      throw std::runtime_error("Dynamixel initialization failed");
    }

    last_dds_time_ = this->now();

    measured_pub_ = this->create_publisher<px4_msgs::msg::ActuatorServos>("/dynamixel_tilt_mea", rclcpp::SensorDataQoS());
    dds_sub_ = this->create_subscription<px4_msgs::msg::ActuatorServos>("/fmu/out/actuator_servos", rclcpp::SensorDataQoS(), std::bind(&Px4ServoToDynamixel::dds_callback, this, std::placeholders::_1));
    timeout_timer_ = this->create_wall_timer(std::chrono::microseconds(params::SERVO_PERIOD_US), std::bind(&Px4ServoToDynamixel::timeout_callback, this));
    measured_timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&Px4ServoToDynamixel::measured_callback, this));

    RCLCPP_INFO(this->get_logger(), "Dynamixel ready: port=%s, baud=%d", params::DXL_PORT_NAME, params::DXL_BAUDRATE);
  }

~Px4ServoToDynamixel() override
{
  if (timeout_timer_) timeout_timer_->cancel();
  if (measured_timer_) measured_timer_->cancel();

  dds_sub_.reset();

  send_zero(params::FINAL_ZERO_REPEAT_COUNT, params::FINAL_ZERO_REPEAT_INTERVAL_MS);

  if (torque_off_all()) RCLCPP_INFO(this->get_logger(), "Final zero sent. Torque disabled.");
  else RCLCPP_ERROR(this->get_logger(), "Final zero sent, but torque off failed.");

  sync_read_.reset();
  sync_write_.reset();

  if (port_handler_) port_handler_->closePort();
}

private:
  bool initial_setting(std::array<int32_t, params::DXL_SERVOS.size()> & initial_ppr)
  {
    port_handler_ = dynamixel::PortHandler::getPortHandler(params::DXL_PORT_NAME);
    packet_handler_ = dynamixel::PacketHandler::getPacketHandler(params::DXL_PROTOCOL_VERSION);

    if (!port_handler_ || !packet_handler_) return false;

    bool port_opened = false;

    for (int attempt = 1; attempt <= params::PORT_OPEN_RETRY_COUNT; ++attempt)
    {
      port_handler_->closePort();

      if (port_handler_->openPort() && port_handler_->setBaudRate(params::DXL_BAUDRATE))
      {
        port_opened = true;
        break;
      }

      RCLCPP_WARN(this->get_logger(), "Port connection failed: %d/%d", attempt, params::PORT_OPEN_RETRY_COUNT);
      std::this_thread::sleep_for(std::chrono::milliseconds(params::PORT_RETRY_INTERVAL_MS));
    }

    if (!port_opened) return false;

    sync_write_ = std::make_unique<dynamixel::GroupSyncWrite>(port_handler_, packet_handler_, params::ADDR_GOAL_POSITION, 4);
    sync_read_ = std::make_unique<dynamixel::GroupSyncRead>(port_handler_, packet_handler_, params::ADDR_PRESENT_POSITION, 4);

    for (const auto & servo : params::DXL_SERVOS)
    {
      if (!sync_read_->addParam(servo.id)) return false;
    }

    auto write_1byte = [&](uint8_t id, uint16_t address, uint8_t value)
    {
      for (int attempt = 0; attempt < params::REGISTER_WRITE_RETRY_COUNT; ++attempt)
      {
        uint8_t dxl_error = 0;
        const int result = packet_handler_->write1ByteTxRx(port_handler_, id, address, value, &dxl_error);

        if (result == COMM_SUCCESS && dxl_error == 0) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(params::REGISTER_RETRY_INTERVAL_MS));
      }

      RCLCPP_ERROR(this->get_logger(), "write1 failed: ID=%u, address=%u", static_cast<unsigned>(id), static_cast<unsigned>(address));
      return false;
    };

    auto write_2byte = [&](uint8_t id, uint16_t address, uint16_t value)
    {
      for (int attempt = 0; attempt < params::REGISTER_WRITE_RETRY_COUNT; ++attempt)
      {
        uint8_t dxl_error = 0;
        const int result = packet_handler_->write2ByteTxRx(port_handler_, id, address, value, &dxl_error);

        if (result == COMM_SUCCESS && dxl_error == 0) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(params::REGISTER_RETRY_INTERVAL_MS));
      }

      RCLCPP_ERROR(this->get_logger(), "write2 failed: ID=%u, address=%u", static_cast<unsigned>(id), static_cast<unsigned>(address));
      return false;
    };

    auto read_4byte = [&](uint8_t id, uint16_t address, uint32_t & value)
    {
      for (int attempt = 0; attempt < params::REGISTER_WRITE_RETRY_COUNT; ++attempt)
      {
        uint8_t dxl_error = 0;
        const int result = packet_handler_->read4ByteTxRx(port_handler_, id, address, &value, &dxl_error);

        if (result == COMM_SUCCESS && dxl_error == 0) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(params::REGISTER_RETRY_INTERVAL_MS));
      }

      RCLCPP_ERROR(this->get_logger(), "read4 failed: ID=%u, address=%u", static_cast<unsigned>(id), static_cast<unsigned>(address));
      return false;
    };

    // Motor connection check
    for (const auto & servo : params::DXL_SERVOS)
    {
      bool ping_ok = false;

      for (int attempt = 1; attempt <= params::PING_RETRY_COUNT; ++attempt)
      {
        uint16_t model_number = 0;
        uint8_t dxl_error = 0;

        const int result = packet_handler_->ping(
          port_handler_,
          servo.id,
          &model_number,
          &dxl_error
        );

        if (result == COMM_SUCCESS && dxl_error == 0)
        {
          RCLCPP_INFO(
            this->get_logger(),
            "Ping success: ID=%u, model=%u",
            static_cast<unsigned>(servo.id),
            static_cast<unsigned>(model_number)
          );

          ping_ok = true;
          break;
        }

        RCLCPP_WARN(
          this->get_logger(),
          "Ping attempt %d/%d failed: ID=%u, comm=%d (%s), dxl_error=%u (%s)",
          attempt,
          params::PING_RETRY_COUNT,
          static_cast<unsigned>(servo.id),
          result,
          packet_handler_->getTxRxResult(result),
          static_cast<unsigned>(dxl_error),
          packet_handler_->getRxPacketError(dxl_error)
        );

        std::this_thread::sleep_for(
          std::chrono::milliseconds(params::REGISTER_RETRY_INTERVAL_MS)
        );
      }

      if (!ping_ok)
      {
        RCLCPP_ERROR(
          this->get_logger(),
          "Ping failed permanently: ID=%u",
          static_cast<unsigned>(servo.id)
        );

        return false;
      }
    }

    // Torque off
    for (const auto & servo : params::DXL_SERVOS)
    {
      if (!write_1byte(servo.id, params::ADDR_TORQUE_ENABLE, params::TORQUE_OFF)) return false;
    }

    // Position mode
    for (const auto & servo : params::DXL_SERVOS)
    {
      if (!write_1byte(servo.id, params::ADDR_OPERATING_MODE, params::POSITION_CONTROL_MODE)) return false;
    }

    // Controller gains
    for (const auto & servo : params::DXL_SERVOS)
    {
      if (!write_2byte(servo.id, params::ADDR_POSITION_P_GAIN, servo.position_p)) return false;
      if (!write_2byte(servo.id, params::ADDR_POSITION_I_GAIN, servo.position_i)) return false;
      if (!write_2byte(servo.id, params::ADDR_POSITION_D_GAIN, servo.position_d)) return false;
      if (!write_2byte(servo.id, params::ADDR_VELOCITY_P_GAIN, servo.velocity_p)) return false;
      if (!write_2byte(servo.id, params::ADDR_VELOCITY_I_GAIN, servo.velocity_i)) return false;
    }

    // LPF initial position
    for (std::size_t i = 0; i < params::DXL_SERVOS.size(); ++i)
    {
      uint32_t value = 0;

      if (!read_4byte(params::DXL_SERVOS[i].id, params::ADDR_PRESENT_POSITION, value)) return false;
      initial_ppr[i] = static_cast<int32_t>(value);
    }

    // LPF 시작 위치를 Goal로 입력하고 Torque ON
    if (!write_goal(initial_ppr)) return false;

    for (const auto & servo : params::DXL_SERVOS)
    {
      if (!write_1byte(servo.id, params::ADDR_TORQUE_ENABLE, params::TORQUE_ON)) return false;
    }

    return true;
  }

  bool initial_zero(const std::array<int32_t, params::DXL_SERVOS.size()> & initial_ppr)
  {
    std::array<int32_t, params::DXL_SERVOS.size()> goal_ppr{};

    for (std::size_t i = 0; i < params::DXL_SERVOS.size(); ++i)
    {
      const auto & servo = params::DXL_SERVOS[i];
      const double initial_rad = servo.direction * (static_cast<double>(initial_ppr[i]) - servo.zero_ppr) / params::PPR_PER_RAD;

      startup_lpf_[i] = utils::LPF(params::STARTUP_RETURN_TIME_SEC);
      startup_lpf_[i].reset(initial_rad);
    }

    const auto start = std::chrono::steady_clock::now();
    auto next_tick = start;

    while (rclcpp::ok() && std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() < params::STARTUP_RETURN_TIME_SEC)
    {
      for (std::size_t i = 0; i < params::DXL_SERVOS.size(); ++i)
      {
        const auto & servo = params::DXL_SERVOS[i];
        const double rad = startup_lpf_[i].update(0.0);

        goal_ppr[i] = std::clamp(static_cast<int32_t>(std::lround(servo.zero_ppr + servo.direction * rad * params::PPR_PER_RAD)), servo.min_ppr, servo.max_ppr);
      }

      if (!write_goal(goal_ppr)) return false;

      next_tick += std::chrono::microseconds(params::SERVO_PERIOD_US);
      std::this_thread::sleep_until(next_tick);
    }

    for (std::size_t i = 0; i < params::DXL_SERVOS.size(); ++i) goal_ppr[i] = params::DXL_SERVOS[i].zero_ppr;

    if (!write_goal(goal_ppr)) return false;

    RCLCPP_INFO(this->get_logger(), "Initial zero completed");
    return true;
  }

  bool torque_off_all()
{
  if (!port_handler_ || !packet_handler_) return false;

  bool success = true;

  for (const auto & servo : params::DXL_SERVOS)
  {
    uint8_t dxl_error = 0;

    const int result = packet_handler_->write1ByteTxRx(
      port_handler_,
      servo.id,
      params::ADDR_TORQUE_ENABLE,
      params::TORQUE_OFF,
      &dxl_error
    );

    if (result != COMM_SUCCESS || dxl_error != 0)
    {
      success = false;

      RCLCPP_ERROR(
        this->get_logger(),
        "Torque off failed: ID=%u, comm=%d (%s), dxl_error=%u (%s)",
        static_cast<unsigned>(servo.id),
        result,
        packet_handler_->getTxRxResult(result),
        static_cast<unsigned>(dxl_error),
        packet_handler_->getRxPacketError(dxl_error)
      );
    }
  }

  return success;
}

  void dds_callback(const px4_msgs::msg::ActuatorServos::SharedPtr msg)
  {
    if (emergency_active_) return;

    std::array<int32_t, params::DXL_SERVOS.size()> goal_ppr{};

    for (std::size_t i = 0; i < params::DXL_SERVOS.size(); ++i)
    {
      const auto & servo = params::DXL_SERVOS[i];

      const std::size_t control_index = i;
      const double normalized = std::isfinite(msg->control[control_index]) ? std::clamp(static_cast<double>(msg->control[control_index]), -1.0, 1.0) : 0.0;
      const double rad = normalized * servo.angle_limit_rad;

      goal_ppr[i] = std::clamp(static_cast<int32_t>(std::lround(servo.zero_ppr + servo.direction * rad * params::PPR_PER_RAD)), servo.min_ppr, servo.max_ppr);
    }

    if (!write_goal(goal_ppr))
    {
      emergency_active_ = true;
      RCLCPP_ERROR(this->get_logger(), "DDS write failed. Sending zero.");
      send_zero(params::FINAL_ZERO_REPEAT_COUNT, params::FINAL_ZERO_REPEAT_INTERVAL_MS);
      if (rclcpp::ok()) rclcpp::shutdown();
      return;
    }

    last_dds_time_ = this->now();
    dds_received_ = true;
    timeout_zero_sent_ = false;
  }

  void measured_callback()
  {
    if (emergency_active_) return;
    publish_measured();
  }

  bool publish_measured()
  {
    if (!sync_read_ || !measured_pub_) return false;

    const int result = sync_read_->txRxPacket();

    if (result != COMM_SUCCESS)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Dynamixel measured position read failed");
      return false;
    }

    px4_msgs::msg::ActuatorServos msg{};
    msg.timestamp = static_cast<uint64_t>(this->now().nanoseconds() / 1000);
    msg.timestamp_sample = msg.timestamp;

    for (std::size_t i = 0; i < params::DXL_SERVOS.size(); ++i)
    {
      const auto & servo = params::DXL_SERVOS[i];

      if (!sync_read_->isAvailable(servo.id, params::ADDR_PRESENT_POSITION, 4))
      {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Dynamixel measured position unavailable: ID=%u", static_cast<unsigned>(servo.id));
        return false;
      }

      const int32_t present_ppr = static_cast<int32_t>(sync_read_->getData(servo.id, params::ADDR_PRESENT_POSITION, 4));
      const double rad = servo.direction * (static_cast<double>(present_ppr) - servo.zero_ppr) / params::PPR_PER_RAD;

      msg.control[i] = static_cast<float>(std::clamp(rad / servo.angle_limit_rad, -1.0, 1.0));
    }

    measured_pub_->publish(msg);
    return true;
  }

  void timeout_callback()
  {
    if (emergency_active_ || !dds_received_ || timeout_zero_sent_) return;

    const double timeout = (this->now() - last_dds_time_).seconds();

    if (timeout < params::SERVO_TIMEOUT_SEC) return;

    RCLCPP_WARN(this->get_logger(), "DDS timeout: %.3f sec", timeout);

    if (!send_zero())
    {
      emergency_active_ = true;
      RCLCPP_ERROR(this->get_logger(), "Timeout zero failed.");
      if (rclcpp::ok()) rclcpp::shutdown();
      return;
    }

    timeout_zero_sent_ = true;
  }

  bool send_zero(int repeat = 1, int interval_ms = 0)
  {
    if (!sync_write_) return false;

    std::array<int32_t, params::DXL_SERVOS.size()> zero_ppr{};

    for (std::size_t i = 0; i < params::DXL_SERVOS.size(); ++i) zero_ppr[i] = params::DXL_SERVOS[i].zero_ppr;

    bool success = true;

    for (int i = 0; i < repeat; ++i)
    {
      if (!write_goal(zero_ppr)) success = false;
      if (i + 1 < repeat) std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
    }

    return success;
  }

  bool write_goal(const std::array<int32_t, params::DXL_SERVOS.size()> & goal_ppr)
  {
    if (!sync_write_) return false;

    sync_write_->clearParam();

    for (std::size_t i = 0; i < params::DXL_SERVOS.size(); ++i)
    {
      const uint32_t value = static_cast<uint32_t>(goal_ppr[i]);

      uint8_t data[4]{
        DXL_LOBYTE(DXL_LOWORD(value)),
        DXL_HIBYTE(DXL_LOWORD(value)),
        DXL_LOBYTE(DXL_HIWORD(value)),
        DXL_HIBYTE(DXL_HIWORD(value))
      };

      if (!sync_write_->addParam(params::DXL_SERVOS[i].id, data))
      {
        sync_write_->clearParam();
        return false;
      }
    }

    const int result = sync_write_->txPacket();

    sync_write_->clearParam();
    return result == COMM_SUCCESS;
  }

  dynamixel::PortHandler * port_handler_ = nullptr;
  dynamixel::PacketHandler * packet_handler_ = nullptr;
  std::unique_ptr<dynamixel::GroupSyncWrite> sync_write_;
  std::unique_ptr<dynamixel::GroupSyncRead> sync_read_;

  std::array<utils::LPF, params::DXL_SERVOS.size()> startup_lpf_;

  rclcpp::Publisher<px4_msgs::msg::ActuatorServos>::SharedPtr measured_pub_;
  rclcpp::Subscription<px4_msgs::msg::ActuatorServos>::SharedPtr dds_sub_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;
  rclcpp::TimerBase::SharedPtr measured_timer_;
  rclcpp::Time last_dds_time_;

  bool dds_received_ = false;
  bool timeout_zero_sent_ = true;
  bool emergency_active_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try
  {
    auto node = std::make_shared<Px4ServoToDynamixel>();

    rclcpp::spin(node);
    node.reset();
  }
  catch (const std::exception & e)
  {
    RCLCPP_FATAL(rclcpp::get_logger("px4_servo_to_dynamixel"), "Node failed: %s", e.what());
  }

  if (rclcpp::ok()) rclcpp::shutdown();

  return 0;
}
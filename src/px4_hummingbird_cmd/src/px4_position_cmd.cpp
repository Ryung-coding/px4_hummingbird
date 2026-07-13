#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/hummingbird_status.hpp>
#include <px4_msgs/msg/manual_control_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint6dof.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include <params.hpp>
#include <utils.hpp>

class Px4PositionCmd : public rclcpp::Node
{
public:
  Px4PositionCmd()
  : rclcpp::Node("px4_position_cmd")
  {
    path_name_ = this->declare_parameter<std::string>("path", "position_tuning");
    auto_offboard_ = this->declare_parameter<bool>("auto_offboard", false);
    auto_arm_ = this->declare_parameter<bool>("auto_arm", false);
    require_vehicle_status_ = this->declare_parameter<bool>("require_vehicle_status", true);
    require_vehicle_local_position_ = this->declare_parameter<bool>("require_vehicle_local_position", true);
    require_hb_cmd_source_dds_ = this->declare_parameter<bool>("require_hb_cmd_source_dds", true);
    hummingbird_status_topic_ = this->declare_parameter<std::string>("hummingbird_status_topic", "/fmu/out/hummingbird_status");
    manual_control_topic_ = this->declare_parameter<std::string>("manual_control_topic", "/fmu/out/manual_control_setpoint");
    vehicle_local_position_topic_ = this->declare_parameter<std::string>("vehicle_local_position_topic", "/fmu/out/vehicle_local_position_v1");
    vehicle_status_topic_ = this->declare_parameter<std::string>("vehicle_status_topic", "/fmu/out/vehicle_status_v4");
    handoff_duration_sec_ = this->declare_parameter<double>("handoff_duration_sec", 2.0);
    handoff_position_offset_m_ = this->declare_parameter<double>("handoff_position_offset_m", 0.6);
    handoff_yaw_offset_rad_ = this->declare_parameter<double>("handoff_yaw_offset_rad", 0.35);
    command_delay_sec_ = this->declare_parameter<double>("command_delay_sec", 2.0);
    command_repeat_sec_ = this->declare_parameter<double>("command_repeat_sec", 1.0);
    status_timeout_sec_ = this->declare_parameter<double>("status_timeout_sec", 1.0);
    target_system_ = this->declare_parameter<int>("target_system", 1);
    target_component_ = this->declare_parameter<int>("target_component", 1);
    source_system_ = this->declare_parameter<int>("source_system", 1);
    source_component_ = this->declare_parameter<int>("source_component", 1);

    offboard_pub_        = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    trajectory6dof_pub_  = this->create_publisher<px4_msgs::msg::TrajectorySetpoint6dof>("/fmu/in/trajectory_setpoint6dof", 10);
    vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
    
    const auto status_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    
    hummingbird_status_sub_     = this->create_subscription<px4_msgs::msg::HummingbirdStatus>(hummingbird_status_topic_, status_qos, std::bind(&Px4PositionCmd::hummingbirdStatusCallback, this, std::placeholders::_1));
    manual_control_sub_         = this->create_subscription<px4_msgs::msg::ManualControlSetpoint>(manual_control_topic_, status_qos, std::bind(&Px4PositionCmd::manualControlCallback, this, std::placeholders::_1));
    vehicle_local_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(vehicle_local_position_topic_, status_qos, std::bind(&Px4PositionCmd::vehicleLocalPositionCallback, this, std::placeholders::_1));
    vehicle_status_sub_         = this->create_subscription<px4_msgs::msg::VehicleStatus>(vehicle_status_topic_, status_qos, std::bind(&Px4PositionCmd::vehicleStatusCallback, this, std::placeholders::_1));

    const auto period = std::chrono::nanoseconds(static_cast<int64_t>(1.0e9 / static_cast<double>(params::RATE_HZ)));

    t0_ = std::chrono::steady_clock::now();
    last_status_time_ = t0_;
    last_hummingbird_status_time_ = t0_;
    last_manual_control_time_ = t0_;
    last_vehicle_local_position_time_ = t0_;
    timer_ = this->create_wall_timer(period, std::bind(&Px4PositionCmd::onTick, this));
  }

private:
  enum class StartupState
  {
    WaitBeforeOffboard,
    WaitForOffboard,
    WaitForArm,
    PositionActive
  };

  uint64_t timestampMicros() const
  {
    const auto now = this->get_clock()->now();
    return static_cast<uint64_t>(now.nanoseconds() / 1000);
  }

  utils::TargetCMD pathCommand(double t) const
  {
    if (path_name_ == "track_apple") return utils::trackApple(t);
    if (path_name_ == "take_apple") return utils::takeApple(t);
    if (path_name_ == "attitude_tuning") return utils::attitudeTuningPath(t);
    if (path_name_ == "agile") return utils::agilePath(t);
    if (path_name_ == "position_track") return utils::positionTrack(t);
    if (path_name_ == "hover") return utils::hover(t);
    return utils::positionTuningPath(t);
  }

  void hummingbirdStatusCallback(const px4_msgs::msg::HummingbirdStatus::SharedPtr msg)
  {
    hummingbird_status_ = *msg;
    hummingbird_status_received_ = true;
    last_hummingbird_status_time_ = std::chrono::steady_clock::now();
  }

  void manualControlCallback(const px4_msgs::msg::ManualControlSetpoint::SharedPtr msg)
  {
    manual_control_ = *msg;
    manual_control_received_ = true;
    last_manual_control_time_ = std::chrono::steady_clock::now();
  }

  void vehicleLocalPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
  {
    vehicle_local_position_ = *msg;
    vehicle_local_position_received_ = true;
    last_vehicle_local_position_time_ = std::chrono::steady_clock::now();
  }

  void vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
  {
    vehicle_status_ = *msg;
    vehicle_status_received_ = true;
    last_status_time_ = std::chrono::steady_clock::now();
  }

  bool hbDdsCommandSourceFresh() const
  {
    if (!require_hb_cmd_source_dds_) return true;
    if (!hummingbird_status_received_) return false;

    const double age = std::chrono::duration<double>(std::chrono::steady_clock::now() - last_hummingbird_status_time_).count();
    
    return age <= status_timeout_sec_;
  }

  bool hbDdsCommandEnabled() const
  {
    return !require_hb_cmd_source_dds_ || (hbDdsCommandSourceFresh() && hummingbird_status_.hb_cmd_source == px4_msgs::msg::HummingbirdStatus::HB_CMD_SOURCE_DDS);
  }

  bool manualControlFresh() const
  {
    if (!manual_control_received_ || !manual_control_.valid) return false;

    const double age = std::chrono::duration<double>(std::chrono::steady_clock::now() - last_manual_control_time_).count();
    
    return age <= status_timeout_sec_;
  }

  bool vehicleLocalPositionFresh() const
  {
    if (!vehicle_local_position_received_ || !vehicle_local_position_.xy_valid || !vehicle_local_position_.z_valid) return false;

    const double age = std::chrono::duration<double>(std::chrono::steady_clock::now() - last_vehicle_local_position_time_).count();
    
    return age <= status_timeout_sec_;
  }

  bool localPositionReadyForTrajectoryOrigin() const
  {
    return !require_vehicle_local_position_ || vehicleLocalPositionFresh();
  }

  void captureTrajectoryOrigin(double t)
  {
    trajectory_origin_x_ned_ = vehicle_local_position_.x;
    trajectory_origin_y_ned_ = vehicle_local_position_.y;
    trajectory_origin_altitude_up_ = -vehicle_local_position_.z;
    trajectory_start_time_sec_ = t;
    trajectory_origin_valid_ = true;

    RCLCPP_INFO(
      this->get_logger(),
      "Trajectory origin captured: x=%.3f, y=%.3f, z_up=%.3f, t0=%.3f",
      trajectory_origin_x_ned_,
      trajectory_origin_y_ned_,
      trajectory_origin_altitude_up_,
      trajectory_start_time_sec_);
  }

  void captureCommandSourceStart(double t)
  {
    hb_cmd_source_enable_time_sec_ = t;
    hb_cmd_source_enable_time_valid_ = true;

    if (localPositionReadyForTrajectoryOrigin()) {
      captureTrajectoryOrigin(t);
    } else {
      trajectory_origin_valid_ = false;
      RCLCPP_WARN(
        this->get_logger(),
        "HB_CMD_SOURCE=1 detected, but vehicle_local_position is not fresh yet.");
    }
  }

  utils::TargetCMD applyTrajectoryOrigin(const utils::TargetCMD& relative_cmd) const
  {
    utils::TargetCMD absolute_cmd = relative_cmd;
    absolute_cmd.x = trajectory_origin_x_ned_ + relative_cmd.x;
    absolute_cmd.y = trajectory_origin_y_ned_ + relative_cmd.y;
    absolute_cmd.z = trajectory_origin_altitude_up_ + relative_cmd.z;
    return absolute_cmd;
  }

  bool vehicleStatusFresh(double t) const
  {
    if (!require_vehicle_status_) return true;
    if (!vehicle_status_received_) return false;

    const double age = std::chrono::duration<double>(std::chrono::steady_clock::now() - last_status_time_).count();
    
    return age <= status_timeout_sec_ && t >= command_delay_sec_;
  }

  bool offboardReady() const
  {
    return !require_vehicle_status_ || vehicle_status_.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
  }

  bool armReady() const
  {
    return !require_vehicle_status_ || vehicle_status_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
  }

  bool readyForPositionSetpoint() const
  {
    return offboardReady() && armReady();
  }

  bool commandIntervalElapsed(double t, double last_command_time) const
  {
    return (t - last_command_time) >= command_repeat_sec_;
  }

  void publishOffboardControlMode(uint64_t timestamp_us)
  {
    px4_msgs::msg::OffboardControlMode msg;
    msg.timestamp = timestamp_us;
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.thrust_and_torque = false;
    msg.direct_actuator = false;
    offboard_pub_->publish(msg);
  }

  void publishTrajectorySetpoint6dof(uint64_t timestamp_us, const utils::TargetCMD& cmd)
  {
    px4_msgs::msg::TrajectorySetpoint6dof msg;
    msg.timestamp = timestamp_us;

    msg.position = {
      static_cast<float>(cmd.x),
      static_cast<float>(cmd.y),
      static_cast<float>(-cmd.z)
    };

    const float nan = std::numeric_limits<float>::quiet_NaN();
    msg.velocity = {nan, nan, nan};
    msg.acceleration = {nan, nan, nan};
    msg.jerk = {nan, nan, nan};

    const Eigen::Vector4d q = utils::rpyToQuat(Eigen::Vector3d(cmd.roll, cmd.pitch, cmd.yaw));
    msg.quaternion = {
      static_cast<float>(q(0)),
      static_cast<float>(q(1)),
      static_cast<float>(q(2)),
      static_cast<float>(q(3))
    };
    msg.angular_velocity = {nan, nan, nan};

    trajectory6dof_pub_->publish(msg);
  }

  void publishVehicleCommand(uint64_t timestamp_us, uint32_t command, float param1, float param2 = 0.0F, float param3 = 0.0F)
  {
    px4_msgs::msg::VehicleCommand msg;
    msg.timestamp = timestamp_us;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.param3 = param3;
    msg.param4 = 0.0F;
    msg.param5 = 0.0;
    msg.param6 = 0.0;
    msg.param7 = 0.0F;
    msg.command = command;
    msg.target_system = static_cast<uint8_t>(target_system_);
    msg.target_component = static_cast<uint8_t>(target_component_);
    msg.source_system = static_cast<uint8_t>(source_system_);
    msg.source_component = static_cast<uint16_t>(source_component_);
    msg.confirmation = 0;
    msg.from_external = true;

    vehicle_command_pub_->publish(msg);
  }

  void beginManualHandoff(double t)
  {
    if (manual_handoff_active_) return;

    manual_handoff_active_ = true;
    manual_handoff_completed_ = false;
    manual_handoff_start_time_sec_ = t;
    handoff_filters_.reset(last_cmd_);
    RCLCPP_INFO(this->get_logger(), "HB_CMD_SOURCE changed to RC. Starting smooth manual handoff.");
  }

  void finishManualHandoff(uint64_t timestamp_us)
  {
    if (manual_handoff_completed_) return;

    publishVehicleCommand(
      timestamp_us,
      px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
      1.0F,
      3.0F);

    startup_state_ = StartupState::WaitBeforeOffboard;
    manual_handoff_active_ = false;
    manual_handoff_completed_ = true;
    RCLCPP_INFO(this->get_logger(), "Manual handoff complete. POSCTL command sent and offboard setpoints stopped.");
  }

  void publishManualHandoffSetpoint(uint64_t timestamp_us, double t)
  {
    const double roll = manualControlFresh() ? manual_control_.roll : 0.0;
    const double pitch = manualControlFresh() ? manual_control_.pitch : 0.0;
    const double yaw = manualControlFresh() ? manual_control_.yaw : 0.0;

    const double x = vehicleLocalPositionFresh() ? vehicle_local_position_.x : last_cmd_.x;
    const double y = vehicleLocalPositionFresh() ? vehicle_local_position_.y : last_cmd_.y;
    const double z = vehicleLocalPositionFresh() ? vehicle_local_position_.z : -last_cmd_.z;
    const double heading = vehicleLocalPositionFresh() ? vehicle_local_position_.heading : last_cmd_.yaw;

    const auto cmd = utils::manualHandoffCommand(
      last_cmd_,
      x,
      y,
      z,
      heading,
      roll,
      pitch,
      yaw,
      handoff_position_offset_m_,
      handoff_yaw_offset_rad_,
      handoff_filters_);

    last_cmd_ = cmd;
    publishTrajectorySetpoint6dof(timestamp_us, cmd);

    if ((t - manual_handoff_start_time_sec_) >= handoff_duration_sec_) finishManualHandoff(timestamp_us);
  }

  void updateStartupSequence(uint64_t timestamp_us, double t)
  {
    const bool hb_dds_enabled = hbDdsCommandEnabled();

    if (!previous_hb_dds_enabled_ && hb_dds_enabled) {
      captureCommandSourceStart(t);
    }

    if (previous_hb_dds_enabled_ && !hb_dds_enabled && startup_state_ == StartupState::PositionActive) beginManualHandoff(t);

    previous_hb_dds_enabled_ = hb_dds_enabled;

    if (manual_handoff_active_) return;

    if (!hb_dds_enabled) {
      if (startup_state_ == StartupState::PositionActive) {
        startup_state_ = StartupState::WaitBeforeOffboard;
        trajectory_origin_valid_ = false;
        hb_cmd_source_enable_time_valid_ = false;
      }

      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Waiting for HB_CMD_SOURCE=1 before sending offboard commands.");
      return;
    }

    if (!vehicleStatusFresh(t)) {
      if (startup_state_ == StartupState::PositionActive) {
        startup_state_ = StartupState::WaitForOffboard;
      }

      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Waiting for fresh vehicle_status before sending position setpoints.");
      return;
    }

    if (startup_state_ == StartupState::PositionActive && !readyForPositionSetpoint()) {
      startup_state_ = offboardReady() ? StartupState::WaitForArm : StartupState::WaitForOffboard;
      RCLCPP_WARN(this->get_logger(), "Vehicle left ready state. Position setpoints stopped.");
    }

    switch (startup_state_) {
      case StartupState::WaitBeforeOffboard:
        if (t < command_delay_sec_) {
          return;
        }
        startup_state_ = StartupState::WaitForOffboard;
        [[fallthrough]];

      case StartupState::WaitForOffboard:
        if (!offboardReady()) {
          if (auto_offboard_ && commandIntervalElapsed(t, last_offboard_command_time_sec_)) {
            publishVehicleCommand(
              timestamp_us,
              px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
              1.0F,
              6.0F);
            last_offboard_command_time_sec_ = t;
            RCLCPP_INFO(this->get_logger(), "Offboard mode command sent.");
          }
          return;
        }

        RCLCPP_INFO(this->get_logger(), "Offboard mode confirmed.");
        startup_state_ = StartupState::WaitForArm;
        [[fallthrough]];

      case StartupState::WaitForArm:
        if (!armReady()) {
          if (auto_arm_ && commandIntervalElapsed(t, last_arm_command_time_sec_)) {
            publishVehicleCommand(
              timestamp_us,
              px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
              1.0F);
            last_arm_command_time_sec_ = t;
            RCLCPP_INFO(this->get_logger(), "Arm command sent.");
          }
          return;
        }

        if (!trajectory_origin_valid_) {
          if (!localPositionReadyForTrajectoryOrigin()) {
            RCLCPP_WARN_THROTTLE(
              this->get_logger(), *this->get_clock(), 2000,
              "Waiting for fresh vehicle_local_position before capturing trajectory origin.");
            return;
          }

          captureTrajectoryOrigin(hb_cmd_source_enable_time_valid_ ? hb_cmd_source_enable_time_sec_ : t);
        }

        startup_state_ = StartupState::PositionActive;
        RCLCPP_INFO(this->get_logger(), "Vehicle armed. Position setpoint publishing started.");
        return;

      case StartupState::PositionActive:
        return;
    }
  }

  void onTick()
  {
    const double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0_).count();
    const uint64_t timestamp_us = timestampMicros();

    const bool publish_offboard = hbDdsCommandEnabled() || manual_handoff_active_;

    if (publish_offboard) publishOffboardControlMode(timestamp_us);

    updateStartupSequence(timestamp_us, t);

    if (manual_handoff_active_) {
      publishManualHandoffSetpoint(timestamp_us, t);
      return;
    }

    if (startup_state_ != StartupState::PositionActive || !readyForPositionSetpoint()) return;

    if (!trajectory_origin_valid_) return;

    const auto relative_cmd = pathCommand(t - trajectory_start_time_sec_);
    const auto cmd = applyTrajectoryOrigin(relative_cmd);
    last_cmd_ = cmd;
    publishTrajectorySetpoint6dof(timestamp_us, cmd);
  }

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint6dof>::SharedPtr trajectory6dof_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::Subscription<px4_msgs::msg::HummingbirdStatus>::SharedPtr hummingbird_status_sub_;
  rclcpp::Subscription<px4_msgs::msg::ManualControlSetpoint>::SharedPtr manual_control_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::steady_clock::time_point t0_;
  std::chrono::steady_clock::time_point last_status_time_;
  std::chrono::steady_clock::time_point last_hummingbird_status_time_;
  std::chrono::steady_clock::time_point last_manual_control_time_;
  std::chrono::steady_clock::time_point last_vehicle_local_position_time_;

  px4_msgs::msg::HummingbirdStatus hummingbird_status_{};
  px4_msgs::msg::ManualControlSetpoint manual_control_{};
  px4_msgs::msg::VehicleLocalPosition vehicle_local_position_{};
  px4_msgs::msg::VehicleStatus vehicle_status_{};
  utils::TargetCMD last_cmd_{};
  utils::ManualHandoffFilters handoff_filters_{};
  std::string path_name_;
  std::string hummingbird_status_topic_;
  std::string manual_control_topic_;
  std::string vehicle_local_position_topic_;
  std::string vehicle_status_topic_;
  bool auto_offboard_{false};
  bool auto_arm_{false};
  bool require_vehicle_status_{true};
  bool require_vehicle_local_position_{true};
  bool require_hb_cmd_source_dds_{true};
  bool hummingbird_status_received_{false};
  bool manual_control_received_{false};
  bool vehicle_local_position_received_{false};
  bool vehicle_status_received_{false};
  bool previous_hb_dds_enabled_{false};
  bool manual_handoff_active_{false};
  bool manual_handoff_completed_{false};
  bool trajectory_origin_valid_{false};
  bool hb_cmd_source_enable_time_valid_{false};
  StartupState startup_state_{StartupState::WaitBeforeOffboard};
  double command_delay_sec_{2.0};
  double command_repeat_sec_{1.0};
  double status_timeout_sec_{1.0};
  double handoff_duration_sec_{2.0};
  double handoff_position_offset_m_{0.6};
  double handoff_yaw_offset_rad_{0.35};
  double last_offboard_command_time_sec_{-std::numeric_limits<double>::infinity()};
  double last_arm_command_time_sec_{-std::numeric_limits<double>::infinity()};
  double trajectory_start_time_sec_{0.0};
  double hb_cmd_source_enable_time_sec_{0.0};
  double trajectory_origin_x_ned_{0.0};
  double trajectory_origin_y_ned_{0.0};
  double trajectory_origin_altitude_up_{0.0};
  double manual_handoff_start_time_sec_{0.0};
  int target_system_{1};
  int target_component_{1};
  int source_system_{1};
  int source_component_{1};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Px4PositionCmd>());
  rclcpp::shutdown();
  return 0;
}

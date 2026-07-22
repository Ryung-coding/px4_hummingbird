#include <chrono>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <Eigen/Core>

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


class Px4PositionCommand : public rclcpp::Node
{
public:
  Px4PositionCommand()
  : rclcpp::Node("px4_position_cmd")
  {
    trajectory_name_ = this->declare_parameter<std::string>("path", "position_tuning");
    auto_offboard_   = this->declare_parameter<bool>("auto_offboard", false);
    auto_arm_        = this->declare_parameter<bool>("auto_arm", false);

    offboard_mode_publisher_   = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    trajectory_publisher_      = this->create_publisher<px4_msgs::msg::TrajectorySetpoint6dof>("/fmu/in/trajectory_setpoint6dof", 10);
    vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);

    // 통신 품질 설정(QoS, Quality of Service)
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    // For px4_multirotor_viewer.py
    hummingbird_status_subscription_ = this->create_subscription<px4_msgs::msg::HummingbirdStatus>("/fmu/out/hummingbird_status", qos, std::bind(&Px4PositionCommand::controller_mode_callback, this, std::placeholders::_1));
    manual_control_subscription_     = this->create_subscription<px4_msgs::msg::ManualControlSetpoint>("/fmu/out/manual_control_setpoint", qos, std::bind(&Px4PositionCommand::RC_callback, this, std::placeholders::_1));
    local_position_subscription_     = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position_v1", qos, std::bind(&Px4PositionCommand::local_position_callback, this, std::placeholders::_1));
    vehicle_status_subscription_     = this->create_subscription<px4_msgs::msg::VehicleStatus>("/fmu/out/vehicle_status_v4", qos, std::bind(&Px4PositionCommand::vehicle_status_callback, this, std::placeholders::_1));


    const auto timer_period = std::chrono::nanoseconds(static_cast<int64_t>(1.0e9 / static_cast<double>(params::RATE_HZ)));

    // Initialize timestamps
    node_start_time_ = std::chrono::steady_clock::now();
    last_controller_mode_time_ = node_start_time_;
    last_RC_cmd_time_ = node_start_time_;
    last_local_position_time_ = node_start_time_;
    last_vehicle_status_time_ = node_start_time_;

    // Control Timer (Main Loop)
    control_timer_ = this->create_wall_timer(timer_period, std::bind(&Px4PositionCommand::on_tick, this));
  }

private:
  enum class ControlMode
  {
    wait_offboard,
    wait_arm,
    path_active,
    DDS2RC_handoff,
    RC2DDS_handoff
  };

  // DDS= or RC= (hb_cmd_source)
  void controller_mode_callback(const px4_msgs::msg::HummingbirdStatus::SharedPtr message)
  {
    controller_mode_ = *message;
    has_controller_mode_ = true;
    last_controller_mode_time_ = std::chrono::steady_clock::now();
  }

  // RC cmd
  void RC_callback(const px4_msgs::msg::ManualControlSetpoint::SharedPtr message)
  {
    RC_cmd = *message;
    has_RC_cmd_ = true;
    last_RC_cmd_time_ = std::chrono::steady_clock::now();
  }

  // Local Position (x,y,z,heading)
  void local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr message)
  {
    local_position_ = *message;
    has_local_position_ = true;
    last_local_position_time_ = std::chrono::steady_clock::now();
  }

  // PX4 status, Offboard on/off, Armming on/off, Navigation state
  void vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr message)
  {
    vehicle_status_ = *message;
    has_vehicle_status_ = true;
    last_vehicle_status_time_ = std::chrono::steady_clock::now();
  }

  void wait_offboard(uint64_t timestamp_us, double elapsed_sec)
  {
    if (elapsed_sec < params::command_delay_sec) return;

    const bool offboard_mode_active = vehicle_status_.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;

    // offboard request
    if (!offboard_mode_active) 
    {
      if (auto_offboard_ && elapsed_sec - last_offboard_request_time_sec_ >= params::command_repeat_sec) 
      {
        px4_msgs::msg::VehicleCommand message{};

        message.timestamp = timestamp_us;
        message.param1 = 1.0F;
        message.param2 = 6.0F;
        message.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
        message.target_system = params::target_system_id;
        message.target_component = params::target_component_id;
        message.source_system = params::source_system_id;
        message.source_component = params::source_component_id;
        message.confirmation = 0;
        message.from_external = true;

        vehicle_command_publisher_->publish(message);

        last_offboard_request_time_sec_ = elapsed_sec;

        RCLCPP_INFO(this->get_logger(), "Offboard mode command sent.");
      }
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Offboard mode confirmed.");

    control_mode_ = ControlMode::wait_arm;
    wait_arm(timestamp_us, elapsed_sec);
  }

  void wait_arm(uint64_t timestamp_us, double elapsed_sec)
  {
    const bool vehicle_armed = vehicle_status_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;

    // Arm request
    if (!vehicle_armed) 
    {
      if (auto_arm_ && elapsed_sec - last_arm_request_time_sec_ >= params::message_timeout_sec + params::command_repeat_sec) 
      {
        px4_msgs::msg::VehicleCommand message{};

        message.timestamp = timestamp_us;
        message.param1 = 1.0F;
        message.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
        message.target_system = params::target_system_id;
        message.target_component = params::target_component_id;
        message.source_system = params::source_system_id;
        message.source_component = params::source_component_id;
        message.confirmation = 0;
        message.from_external = true;

        vehicle_command_publisher_->publish(message);

        last_arm_request_time_sec_ = elapsed_sec;

        RCLCPP_INFO(this->get_logger(), "Arm command sent.");
      }

      return;
    }

    // update trajectory origin
    if (!has_trajectory_origin_) 
    {
      pos_x = local_position_.x;
      pos_y = local_position_.y;
      pos_z = -local_position_.z;
      trajectory_start_time_sec_ = has_dds_control_start_time_ ? dds_control_start_time_sec_ : elapsed_sec;
      has_trajectory_origin_ = true;
    }

    control_mode_ = ControlMode::path_active;

    RCLCPP_INFO(this->get_logger(), "Vehicle armed. Path setpoint publishing started.");
  }

  void path_active(uint64_t timestamp_us, double elapsed_sec)
  {
    const double trajectory_time_sec = elapsed_sec - trajectory_start_time_sec_;

    utils::TargetCMD path_target{};

    if      (trajectory_name_ == "position_tuning")  path_target = utils::positionTuningPath(trajectory_time_sec);
    else if (trajectory_name_ == "attitude_tuning")  path_target = utils::attitudeTuningPath(trajectory_time_sec);
    else if (trajectory_name_ == "stepped_attitude") path_target = utils::steppedAttitudePath(trajectory_time_sec);
    else if (trajectory_name_ == "agile")            path_target = utils::agilePath(trajectory_time_sec);
    else if (trajectory_name_ == "position_track")   path_target = utils::positionTrack(trajectory_time_sec);
    else if (trajectory_name_ == "hover")            path_target = utils::hover(trajectory_time_sec);
    else path_target = utils::hover(trajectory_time_sec);

    utils::TargetCMD target = path_target;

    target.x = pos_x + path_target.x;
    target.y = pos_y + path_target.y;
    target.z = pos_z + path_target.z;

    last_target_ = target;


    // Publish trajectory setpoint
    px4_msgs::msg::TrajectorySetpoint6dof message{};

    const float nan = std::numeric_limits<float>::quiet_NaN();
    const Eigen::Vector4d quaternion = utils::rpyToQuat(Eigen::Vector3d(target.roll, target.pitch, target.yaw));

    message.timestamp = timestamp_us;
    message.position = {static_cast<float>(target.x), static_cast<float>(target.y), static_cast<float>(-target.z)};
    message.velocity = {nan, nan, nan};
    message.acceleration = {nan, nan, nan};
    message.jerk = {nan, nan, nan};
    message.quaternion = {static_cast<float>(quaternion(0)), static_cast<float>(quaternion(1)),static_cast<float>(quaternion(2)),static_cast<float>(quaternion(3))};
    message.angular_velocity = {nan, nan, nan};

    trajectory_publisher_->publish(message);
  }

  void DDS2RC_handoff(uint64_t timestamp_us, double elapsed_sec)
  {
    const auto current_time = std::chrono::steady_clock::now();
    const double manual_control_age_sec = std::chrono::duration<double>(current_time - last_RC_cmd_time_).count();
    const double local_position_age_sec = std::chrono::duration<double>(current_time - last_local_position_time_).count();

    const bool manual_control_fresh = has_RC_cmd_ && RC_cmd.valid && manual_control_age_sec <= params::message_timeout_sec;
    const bool local_position_fresh = has_local_position_ && local_position_.xy_valid && local_position_.z_valid && local_position_age_sec <= params::message_timeout_sec;

    const double manual_roll = manual_control_fresh ? RC_cmd.roll : 0.0;
    const double manual_pitch = manual_control_fresh ? RC_cmd.pitch : 0.0;
    const double manual_yaw = manual_control_fresh ? RC_cmd.yaw : 0.0;

    const double current_x_ned = local_position_fresh ? local_position_.x : last_target_.x;
    const double current_y_ned = local_position_fresh ? local_position_.y : last_target_.y;
    const double current_z_ned = local_position_fresh ? local_position_.z : -last_target_.z;
    const double current_heading = local_position_fresh ? local_position_.heading : last_target_.yaw;

    last_target_ = utils::DDS2manual_handoff(last_target_, current_x_ned, current_y_ned, current_z_ned, current_heading, manual_roll, manual_pitch, manual_yaw, params::handoff_position_offset_m, params::handoff_yaw_offset_rad, dds2manual_handoff_filters_);

    px4_msgs::msg::TrajectorySetpoint6dof setpoint_message{};

    const float nan = std::numeric_limits<float>::quiet_NaN();
    const Eigen::Vector4d quaternion = utils::rpyToQuat(Eigen::Vector3d(last_target_.roll, last_target_.pitch, last_target_.yaw));

    setpoint_message.timestamp = timestamp_us;
    setpoint_message.position = {static_cast<float>(last_target_.x), static_cast<float>(last_target_.y), static_cast<float>(-last_target_.z)};
    setpoint_message.velocity = {nan, nan, nan};
    setpoint_message.acceleration = {nan, nan, nan};
    setpoint_message.jerk = {nan, nan, nan};
    setpoint_message.quaternion = {static_cast<float>(quaternion(0)), static_cast<float>(quaternion(1)), static_cast<float>(quaternion(2)), static_cast<float>(quaternion(3))};
    setpoint_message.angular_velocity = {nan, nan, nan};

    trajectory_publisher_->publish(setpoint_message);

    if (elapsed_sec - handoff_start_time_sec_ < params::handoff_run_time_sec) return;

    px4_msgs::msg::VehicleCommand mode_message{};

    mode_message.timestamp = timestamp_us;
    mode_message.param1 = 1.0F;
    mode_message.param2 = 3.0F;
    mode_message.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
    mode_message.target_system = params::target_system_id;
    mode_message.target_component = params::target_component_id;
    mode_message.source_system = params::source_system_id;
    mode_message.source_component = params::source_component_id;
    mode_message.confirmation = 0;
    mode_message.from_external = true;

    vehicle_command_publisher_->publish(mode_message);

    control_mode_ = ControlMode::wait_offboard;
    has_trajectory_origin_ = false;
    has_dds_control_start_time_ = false;

    RCLCPP_INFO(this->get_logger(),"Manual handoff complete. POSCTL command sent and offboard setpoints stopped.");
  }

  void on_tick()
  {
    const auto current_steady_time = std::chrono::steady_clock::now();
    
    const double elapsed_sec = std::chrono::duration<double>(current_steady_time - node_start_time_).count();
    
    const uint64_t timestamp_us = static_cast<uint64_t>(this->get_clock()->now().nanoseconds() / 1000);
    
    const double controller_mode_age_sec = std::chrono::duration<double>(current_steady_time - last_controller_mode_time_).count();
    const double local_position_age_sec = std::chrono::duration<double>(current_steady_time - last_local_position_time_).count();
    const double vehicle_status_age_sec = std::chrono::duration<double>(current_steady_time - last_vehicle_status_time_).count();

    const bool controller_mode_fresh = has_controller_mode_ && controller_mode_age_sec <= params::message_timeout_sec;
    const bool local_position_fresh  = has_local_position_  &&  local_position_age_sec <= params::message_timeout_sec && local_position_.xy_valid && local_position_.z_valid;
    const bool vehicle_status_fresh  = has_vehicle_status_  &&  vehicle_status_age_sec <= params::message_timeout_sec;
    
    const bool dds_control_enabled = controller_mode_fresh && controller_mode_.hb_cmd_source == px4_msgs::msg::HummingbirdStatus::HB_CMD_SOURCE_DDS;

    if (!previous_dds_control_enabled_ && dds_control_enabled) 
    {
      dds_control_start_time_sec_ = elapsed_sec;
      has_dds_control_start_time_ = true;

      if (local_position_fresh) 
      {
        pos_x = local_position_.x;
        pos_y = local_position_.y;
        pos_z = -local_position_.z;
        trajectory_start_time_sec_ = elapsed_sec;
        has_trajectory_origin_ = true;
        RCLCPP_INFO(this->get_logger(),"DDS control enabled. Trajectory origin set to current local position.");
      } 
      else 
      {
        has_trajectory_origin_ = false;
        RCLCPP_WARN(this->get_logger(), "DDS control enabled, but vehicle_local_position is not fresh.");
      }
    }

    if (previous_dds_control_enabled_ && !dds_control_enabled && control_mode_ == ControlMode::path_active)
    {
      control_mode_ = ControlMode::DDS2RC_handoff;
      handoff_start_time_sec_ = elapsed_sec;
      dds2manual_handoff_filters_.reset(last_target_);
      RCLCPP_INFO(this->get_logger(),"DDS control changed to RC. Starting smooth manual handoff.");
    }

    previous_dds_control_enabled_ = dds_control_enabled;

    if (dds_control_enabled || control_mode_ == ControlMode::DDS2RC_handoff) 
    {
      px4_msgs::msg::OffboardControlMode message{};

      message.timestamp = timestamp_us;
      message.position = true;
      message.velocity = false;
      message.acceleration = false;
      message.attitude = false;
      message.body_rate = false;
      message.thrust_and_torque = false;
      message.direct_actuator = false;

      offboard_mode_publisher_->publish(message);
    }

    if (control_mode_ == ControlMode::DDS2RC_handoff) 
    {
      DDS2RC_handoff(timestamp_us, elapsed_sec);
      return;
    }

    if (!dds_control_enabled) { RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for HB_CMD_SOURCE_DDS."); return; }

    if (!vehicle_status_fresh) 
    {
      if (control_mode_ == ControlMode::path_active) control_mode_ = ControlMode::wait_offboard;
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for vehicle_status");
      return;
    }

    const bool offboard_mode_active = vehicle_status_.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;

    const bool vehicle_armed = vehicle_status_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;

    if (control_mode_ == ControlMode::path_active && (!offboard_mode_active || !vehicle_armed))
    {
      control_mode_ = offboard_mode_active ? ControlMode::wait_arm : ControlMode::wait_offboard;
      RCLCPP_WARN(this->get_logger(),"Vehicle left active control state. Position setpoints stopped.");
    }

    switch (control_mode_) 
    {
      case ControlMode::wait_offboard:
        wait_offboard(timestamp_us, elapsed_sec);
        break;

      case ControlMode::wait_arm:
        wait_arm(timestamp_us, elapsed_sec);
        break;

      case ControlMode::path_active:
        path_active(timestamp_us, elapsed_sec);
        break;

      case ControlMode::DDS2RC_handoff:
        break;

      case ControlMode::RC2DDS_handoff:
        break;
    }
  }

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_publisher_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint6dof>::SharedPtr trajectory_publisher_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;

  rclcpp::Subscription<px4_msgs::msg::HummingbirdStatus>::SharedPtr hummingbird_status_subscription_;
  rclcpp::Subscription<px4_msgs::msg::ManualControlSetpoint>::SharedPtr manual_control_subscription_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_subscription_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_subscription_;

  rclcpp::TimerBase::SharedPtr control_timer_;

  std::chrono::steady_clock::time_point node_start_time_;
  std::chrono::steady_clock::time_point last_controller_mode_time_;
  std::chrono::steady_clock::time_point last_RC_cmd_time_;
  std::chrono::steady_clock::time_point last_local_position_time_;
  std::chrono::steady_clock::time_point last_vehicle_status_time_;

  px4_msgs::msg::HummingbirdStatus controller_mode_{};
  px4_msgs::msg::ManualControlSetpoint RC_cmd{};
  px4_msgs::msg::VehicleLocalPosition local_position_{};
  px4_msgs::msg::VehicleStatus vehicle_status_{};

  utils::TargetCMD last_target_{};
  utils::HandoffFilters dds2manual_handoff_filters_{};

  std::string trajectory_name_;

  bool auto_offboard_{false};
  bool auto_arm_{false};

  bool has_controller_mode_{false};
  bool has_RC_cmd_{false};
  bool has_local_position_{false};
  bool has_vehicle_status_{false};

  bool previous_dds_control_enabled_{false};
  bool has_trajectory_origin_{false};
  bool has_dds_control_start_time_{false};

  ControlMode control_mode_{ControlMode::wait_offboard};

  double last_offboard_request_time_sec_{-std::numeric_limits<double>::infinity()};
  double last_arm_request_time_sec_{-std::numeric_limits<double>::infinity()};

  double trajectory_start_time_sec_{0.0};
  double dds_control_start_time_sec_{0.0};
  double handoff_start_time_sec_{0.0};

  double pos_x{0.0};
  double pos_y{0.0};
  double pos_z{0.0};
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Px4PositionCommand>());
  rclcpp::shutdown();

  return 0;
}
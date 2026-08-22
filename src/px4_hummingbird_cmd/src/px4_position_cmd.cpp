#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <Eigen/Core>

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/hummingbird_status.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint6dof.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
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
    trajectory_name_ = this->declare_parameter<std::string>("path", "pos");
    auto_offboard_   = this->declare_parameter<bool>("auto_offboard", false);
    auto_arm_        = this->declare_parameter<bool>("auto_arm", false);

    offboard_mode_publisher_   = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    trajectory_publisher_      = this->create_publisher<px4_msgs::msg::TrajectorySetpoint6dof>("/fmu/in/trajectory_setpoint6dof", 10);
    vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);

    // Communication QoS
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    hummingbird_status_subscription_ = this->create_subscription<px4_msgs::msg::HummingbirdStatus>("/fmu/out/hummingbird_status_v1", qos, std::bind(&Px4PositionCommand::controller_mode_callback, this, std::placeholders::_1));
    local_position_subscription_     = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position_v1", qos, std::bind(&Px4PositionCommand::local_position_callback, this, std::placeholders::_1));
    vehicle_attitude_subscription_    = this->create_subscription<px4_msgs::msg::VehicleAttitude>("/fmu/out/vehicle_attitude", qos, std::bind(&Px4PositionCommand::vehicle_attitude_callback, this, std::placeholders::_1));
    vehicle_status_subscription_     = this->create_subscription<px4_msgs::msg::VehicleStatus>("/fmu/out/vehicle_status_v4", qos, std::bind(&Px4PositionCommand::vehicle_status_callback, this, std::placeholders::_1));


    const auto timer_period = std::chrono::nanoseconds(static_cast<int64_t>(1.0e9 / static_cast<double>(params::RATE_HZ)));

    // Initialize timestamps
    node_start_time_ = std::chrono::steady_clock::now();
    last_controller_mode_time_ = node_start_time_;
    last_local_position_time_ = node_start_time_;
    last_vehicle_attitude_time_ = node_start_time_;
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
    RC2DDS_handoff
  };

  // DDS= or RC= (hb_cmd_source)
  void controller_mode_callback(const px4_msgs::msg::HummingbirdStatus::SharedPtr message)
  {
    controller_mode_ = *message;
    has_controller_mode_ = true;
    last_controller_mode_time_ = std::chrono::steady_clock::now();
  }

  // Local Position (x,y,z,heading) in NED
  void local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr message)
  {
    local_position_ = *message;
    has_local_position_ = true;
    last_local_position_time_ = std::chrono::steady_clock::now();
  }

  // Attitude quaternion
  void vehicle_attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr message)
  {
    vehicle_attitude_ = *message;
    has_vehicle_attitude_ = true;
    last_vehicle_attitude_time_ = std::chrono::steady_clock::now();
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

  void start_RC2DDS_handoff()
  {
    const auto current_time = std::chrono::steady_clock::now();
    const double local_position_age_sec = std::chrono::duration<double>(current_time - last_local_position_time_).count();
    const bool local_position_fresh = has_local_position_ && local_position_.xy_valid && local_position_.z_valid && local_position_age_sec <= params::message_timeout_sec;

    utils::TargetCMD start = utils::initialPose();

    if (local_position_fresh) {
      start.x = local_position_.x;
      start.y = local_position_.y;
      start.z = local_position_.z;
      start.yaw = local_position_.heading;

    } else {
      RCLCPP_WARN(this->get_logger(), "Starting RC to DDS handoff without fresh vehicle_local_position. Using initial pose as start.");
    }

    last_target_ = start;
    rc2dds_handoff_filters_.reset(start);
    control_mode_ = ControlMode::RC2DDS_handoff;

    RCLCPP_INFO(this->get_logger(), "RC control changed to DDS. Moving to initial pose before starting path.");
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

    start_RC2DDS_handoff();
  }

  void path_active(uint64_t timestamp_us, double elapsed_sec)
  {
    const double trajectory_time_sec = elapsed_sec - trajectory_start_time_sec_;

    utils::TargetCMD path_target{};

    if      (trajectory_name_ == "pos")      path_target = utils::posPath(trajectory_time_sec);
    else if (trajectory_name_ == "att")      path_target = utils::attPath(trajectory_time_sec);
    else if (trajectory_name_ == "step_att") path_target = utils::stepAttPath(trajectory_time_sec);
    else RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Unknown path. Holding initial pose.");

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
    message.position = {static_cast<float>(target.x), static_cast<float>(target.y), static_cast<float>(target.z)};
    message.velocity = {nan, nan, nan};
    message.acceleration = {nan, nan, nan};
    message.jerk = {nan, nan, nan};
    message.quaternion = {static_cast<float>(quaternion(0)), static_cast<float>(quaternion(1)),static_cast<float>(quaternion(2)),static_cast<float>(quaternion(3))};
    message.angular_velocity = {nan, nan, nan};

    trajectory_publisher_->publish(message);
  }

  bool initial_pose_reached(const utils::TargetCMD& initial_target)
  {
    const auto current_time = std::chrono::steady_clock::now();
    const double local_position_age_sec = std::chrono::duration<double>(current_time - last_local_position_time_).count();
    const double vehicle_attitude_age_sec = std::chrono::duration<double>(current_time - last_vehicle_attitude_time_).count();

    const bool local_position_fresh = has_local_position_ && local_position_.xy_valid && local_position_.z_valid && local_position_age_sec <= params::message_timeout_sec;
    const bool vehicle_attitude_fresh = has_vehicle_attitude_ && vehicle_attitude_age_sec <= params::message_timeout_sec
      && std::isfinite(vehicle_attitude_.q[0]) && std::isfinite(vehicle_attitude_.q[1])
      && std::isfinite(vehicle_attitude_.q[2]) && std::isfinite(vehicle_attitude_.q[3]);

    if (!local_position_fresh || !vehicle_attitude_fresh) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for fresh pose to check initial pose arrival.");
      return false;
    }

    const Eigen::Vector3d pos_error(
      local_position_.x - initial_target.x,
      local_position_.y - initial_target.y,
      local_position_.z - initial_target.z);

    const Eigen::Vector4d q(
      vehicle_attitude_.q[0],
      vehicle_attitude_.q[1],
      vehicle_attitude_.q[2],
      vehicle_attitude_.q[3]);
    const Eigen::Vector3d rpy = utils::quatToRpy(q.normalized());

    const double roll_error = utils::angleErrorAbs(rpy(0), initial_target.roll);
    const double pitch_error = utils::angleErrorAbs(rpy(1), initial_target.pitch);
    const double att_error = std::max(roll_error, pitch_error);
    const double yaw_error = utils::angleErrorAbs(local_position_.heading, initial_target.yaw);

    return pos_error.norm() <= params::pos_tol
      && yaw_error <= params::yaw_tol
      && att_error <= params::att_tol;
  }

  void RC2DDS_handoff(uint64_t timestamp_us, double elapsed_sec)
  {
    utils::TargetCMD initial_target = utils::initialPose();
    initial_target.yaw = utils::unwrapNear(initial_target.yaw, last_target_.yaw);
    last_target_ = utils::RC2DDS_handoff(initial_target, rc2dds_handoff_filters_);

    px4_msgs::msg::TrajectorySetpoint6dof setpoint_message{};

    const float nan = std::numeric_limits<float>::quiet_NaN();
    const Eigen::Vector4d quaternion = utils::rpyToQuat(Eigen::Vector3d(last_target_.roll, last_target_.pitch, last_target_.yaw));

    setpoint_message.timestamp = timestamp_us;
    setpoint_message.position = {static_cast<float>(last_target_.x), static_cast<float>(last_target_.y), static_cast<float>(last_target_.z)};
    setpoint_message.velocity = {nan, nan, nan};
    setpoint_message.acceleration = {nan, nan, nan};
    setpoint_message.jerk = {nan, nan, nan};
    setpoint_message.quaternion = {static_cast<float>(quaternion(0)), static_cast<float>(quaternion(1)), static_cast<float>(quaternion(2)), static_cast<float>(quaternion(3))};
    setpoint_message.angular_velocity = {nan, nan, nan};

    trajectory_publisher_->publish(setpoint_message);

    if (!initial_pose_reached(initial_target)) return;

    pos_x = params::initial_pose_x_m;
    pos_y = params::initial_pose_y_m;
    pos_z = params::initial_pose_z_m;
    trajectory_start_time_sec_ = elapsed_sec;
    control_mode_ = ControlMode::path_active;
    last_target_ = initial_target;

    RCLCPP_INFO(this->get_logger(), "Initial pose reached. Path setpoint publishing started.");
  }

  void switch_to_position_control(uint64_t timestamp_us)
  {
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
      if (local_position_fresh)
      {
        RCLCPP_INFO(this->get_logger(),"DDS control enabled. Initial pose handoff will start after offboard and arm are ready.");
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "DDS control enabled, but vehicle_local_position is not fresh.");
      }
    }

    if (previous_dds_control_enabled_ && !dds_control_enabled && (control_mode_ == ControlMode::path_active || control_mode_ == ControlMode::RC2DDS_handoff))
    {
      switch_to_position_control(timestamp_us);
      RCLCPP_INFO(this->get_logger(), "DDS control changed to RC. POSCTL command sent and offboard setpoints stopped.");
    }

    previous_dds_control_enabled_ = dds_control_enabled;

    if (dds_control_enabled)
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

      case ControlMode::RC2DDS_handoff:
        RC2DDS_handoff(timestamp_us, elapsed_sec);
        break;
    }
  }

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_publisher_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint6dof>::SharedPtr trajectory_publisher_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;

  rclcpp::Subscription<px4_msgs::msg::HummingbirdStatus>::SharedPtr hummingbird_status_subscription_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_subscription_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr vehicle_attitude_subscription_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_subscription_;

  rclcpp::TimerBase::SharedPtr control_timer_;

  std::chrono::steady_clock::time_point node_start_time_;
  std::chrono::steady_clock::time_point last_controller_mode_time_;
  std::chrono::steady_clock::time_point last_local_position_time_;
  std::chrono::steady_clock::time_point last_vehicle_attitude_time_;
  std::chrono::steady_clock::time_point last_vehicle_status_time_;

  px4_msgs::msg::HummingbirdStatus controller_mode_{};
  px4_msgs::msg::VehicleLocalPosition local_position_{};
  px4_msgs::msg::VehicleAttitude vehicle_attitude_{};
  px4_msgs::msg::VehicleStatus vehicle_status_{};

  utils::TargetCMD last_target_{};
  utils::HandoffFilters rc2dds_handoff_filters_{};

  std::string trajectory_name_;

  bool auto_offboard_{false};
  bool auto_arm_{false};

  bool has_controller_mode_{false};
  bool has_local_position_{false};
  bool has_vehicle_attitude_{false};
  bool has_vehicle_status_{false};

  bool previous_dds_control_enabled_{false};
  ControlMode control_mode_{ControlMode::wait_offboard};

  double last_offboard_request_time_sec_{-std::numeric_limits<double>::infinity()};
  double last_arm_request_time_sec_{-std::numeric_limits<double>::infinity()};

  double trajectory_start_time_sec_{0.0};
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

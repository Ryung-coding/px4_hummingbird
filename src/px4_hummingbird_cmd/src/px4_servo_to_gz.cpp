#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>

#include <gz/msgs/double.pb.h>
#include <gz/transport/Node.hh>

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/actuator_servos.hpp>

namespace
{
constexpr int kNumTiltServos = 8;
constexpr double kDefaultThetaLimit = M_PI;
constexpr double kDefaultPhiLimit = M_PI / 6.0;
// HummingBird DDS convention: control[0..3] = theta1..theta4, control[4..7] = phi1..phi4.
constexpr std::array<const char*, kNumTiltServos> kServoTopics = {
  "joint_theta1", "joint_theta2", "joint_theta3", "joint_theta4",
  "joint_phi1", "joint_phi2", "joint_phi3", "joint_phi4"
};
}

class Px4ServoToGz : public rclcpp::Node
{
public:
  Px4ServoToGz()
  : rclcpp::Node("px4_servo_to_gz")
  {
    model_name_ = this->declare_parameter<std::string>("model_name", "hummingbird_0");
    theta_limit_rad_ = this->declare_parameter<double>("theta_limit_rad", kDefaultThetaLimit);
    phi_limit_rad_ = this->declare_parameter<double>("phi_limit_rad", kDefaultPhiLimit);

    for (int i = 0; i < kNumTiltServos; ++i) {
      const std::string topic = "/model/" + model_name_ + "/" + kServoTopics[i];
      servo_pubs_[i] = gz_node_.Advertise<gz::msgs::Double>(topic);

      if (!servo_pubs_[i].Valid()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to advertise Gazebo topic: %s", topic.c_str());
      }
    }

    last_actuator_msg_time_ = this->now();
    publish_zero_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&Px4ServoToGz::publishZeroIfInactive, this));

    actuator_servos_sub_ = this->create_subscription<px4_msgs::msg::ActuatorServos>(
      "/fmu/out/actuator_servos",
      rclcpp::SensorDataQoS(),
      std::bind(&Px4ServoToGz::actuatorServosCallback, this, std::placeholders::_1));

    publishZeroCommands();

    RCLCPP_INFO(
      this->get_logger(),
      "PX4 actuator_servos -> Gazebo HummingBird joint bridge started for model=%s",
      model_name_.c_str());
  }

private:
  static double clampNormalized(float value)
  {
    if (!std::isfinite(value)) {
      return 0.0;
    }

    return std::clamp(static_cast<double>(value), -1.0, 1.0);
  }

  void publishZeroCommands()
  {
    for (int i = 0; i < kNumTiltServos; ++i) {
      publishServo(i, 0.0);
    }
  }

  void publishZeroIfInactive()
  {
    if ((this->now() - last_actuator_msg_time_).seconds() < 0.2) {
      return;
    }

    publishZeroCommands();
  }

  void publishServo(int index, double angle_rad)
  {
    if (!std::isfinite(angle_rad) || !servo_pubs_[index].Valid()) {
      return;
    }

    gz::msgs::Double msg;
    msg.set_data(angle_rad);
    servo_pubs_[index].Publish(msg);
  }

  void actuatorServosCallback(const px4_msgs::msg::ActuatorServos::SharedPtr msg)
  {
    last_actuator_msg_time_ = this->now();

    for (int i = 0; i < 4; ++i) {
      const double theta = clampNormalized(msg->control[i]) * theta_limit_rad_;
      const double phi = clampNormalized(msg->control[i + 4]) * phi_limit_rad_;

      publishServo(i, theta);
      publishServo(i + 4, phi);
    }
  }

  gz::transport::Node gz_node_;
  std::array<gz::transport::Node::Publisher, kNumTiltServos> servo_pubs_;
  rclcpp::Subscription<px4_msgs::msg::ActuatorServos>::SharedPtr actuator_servos_sub_;
  rclcpp::TimerBase::SharedPtr publish_zero_timer_;

  std::string model_name_;
  rclcpp::Time last_actuator_msg_time_;
  double theta_limit_rad_{kDefaultThetaLimit};
  double phi_limit_rad_{kDefaultPhiLimit};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Px4ServoToGz>());
  rclcpp::shutdown();
  return 0;
}

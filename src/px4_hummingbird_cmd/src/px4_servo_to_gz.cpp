#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include <gz/msgs/double.pb.h>
#include <gz/transport/Node.hh>

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/actuator_servos.hpp>

#include "params.hpp"
#include "utils.hpp"

class Px4ServoToGz : public rclcpp::Node
{
public:
  Px4ServoToGz()
  : Node("px4_servo_to_gz")
  {
    for (std::size_t i = 0; i < params::SERVO_TOPICS.size(); ++i)
    {
      const std::string topic = "/model/hummingbird_0/" + std::string(params::SERVO_TOPICS[i]);
      servo_pubs_[i] = gz_node_.Advertise<gz::msgs::Double>(topic);

      if (!servo_pubs_[i].Valid()) RCLCPP_ERROR(this->get_logger(), "Failed to advertise Gazebo topic: %s", topic.c_str());

      servo_lpf_[i] = utils::LPF(params::SERVO_RETURN_TIME_SEC);
      servo_lpf_[i].reset(0.0);

      if (!servo_pubs_[i].Valid()) continue;

      gz::msgs::Double command;
      command.set_data(0.0);
      servo_pubs_[i].Publish(command);
    }

    last_actuator_msg_time_ = this->now();
    return_start_time_ = this->now();

    actuator_servos_sub_ = this->create_subscription<px4_msgs::msg::ActuatorServos>("/fmu/out/actuator_servos", rclcpp::SensorDataQoS(), std::bind(&Px4ServoToGz::actuatorServosCallback, this, std::placeholders::_1));

    publish_timer_ = this->create_wall_timer(std::chrono::microseconds(params::SERVO_PERIOD_US), [this]()
    {
      const rclcpp::Time now = this->now();
      const double inactive_time = (now - last_actuator_msg_time_).seconds();

      if (inactive_time < params::SERVO_TIMEOUT_SEC) { returning_to_zero_ = false; return; }
      if (!returning_to_zero_)                       { returning_to_zero_ = true; return_start_time_ = now;}

      const double return_elapsed = (now - return_start_time_).seconds();
      const bool return_finished = return_elapsed >= params::SERVO_RETURN_TIME_SEC;

      for (std::size_t i = 0; i < params::SERVO_TOPICS.size(); ++i)
      {
        double command = servo_lpf_[i].update(0.0);

        if (return_finished)
        {
          command = 0.0;
          servo_lpf_[i].reset(0.0);
        }

        if (!servo_pubs_[i].Valid()) continue;

        gz::msgs::Double servo_msg;
        servo_msg.set_data(command);
        servo_pubs_[i].Publish(servo_msg);
      }
    });

    RCLCPP_INFO(this->get_logger(), "PX4 actuator_servos -> Gazebo bridge started: model=hummingbird_0, rate=%d Hz, return=%.2f sec", params::RATE_HZ, params::SERVO_RETURN_TIME_SEC);
  }

  ~Px4ServoToGz() override
  {
    if (publish_timer_) publish_timer_->cancel();

    for (int repeat = 0; repeat < 5; ++repeat)
    {
      for (std::size_t i = 0; i < params::SERVO_TOPICS.size(); ++i)
      {
        if (!servo_pubs_[i].Valid()) continue;

        gz::msgs::Double command;
        command.set_data(0.0);
        servo_pubs_[i].Publish(command);
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    RCLCPP_INFO(this->get_logger(), "Final zero servo commands published");
  }

private:
  void actuatorServosCallback(const px4_msgs::msg::ActuatorServos::SharedPtr msg)
  {
    last_actuator_msg_time_ = this->now();
    returning_to_zero_ = false;

    for (std::size_t i = 0; i < params::SERVO_TOPICS.size() / 2; ++i)
    {
      const std::size_t phi_index = i + params::SERVO_TOPICS.size() / 2;

      const double theta_normalized = std::isfinite(msg->control[i]) ? std::clamp(static_cast<double>(msg->control[i]), -1.0, 1.0) : 0.0;
      const double phi_normalized = std::isfinite(msg->control[phi_index]) ? std::clamp(static_cast<double>(msg->control[phi_index]), -1.0, 1.0) : 0.0;

      const double theta_command = theta_normalized * params::THETA_LIMIT_RAD;
      const double phi_command = phi_normalized * params::PHI_LIMIT_RAD;

      servo_lpf_[i].reset(theta_command);
      servo_lpf_[phi_index].reset(phi_command);

      if (servo_pubs_[i].Valid())
      {
        gz::msgs::Double theta_msg;
        theta_msg.set_data(theta_command);
        servo_pubs_[i].Publish(theta_msg);
      }

      if (servo_pubs_[phi_index].Valid())
      {
        gz::msgs::Double phi_msg;
        phi_msg.set_data(phi_command);
        servo_pubs_[phi_index].Publish(phi_msg);
      }
    }
  }

  gz::transport::Node gz_node_;

  std::array<gz::transport::Node::Publisher, params::SERVO_TOPICS.size()> servo_pubs_;
  std::array<utils::LPF, params::SERVO_TOPICS.size()> servo_lpf_;

  rclcpp::Subscription<px4_msgs::msg::ActuatorServos>::SharedPtr actuator_servos_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  rclcpp::Time last_actuator_msg_time_;
  rclcpp::Time return_start_time_;

  bool returning_to_zero_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Px4ServoToGz>();
  rclcpp::spin(node);
  node.reset();
  if (rclcpp::ok()) rclcpp::shutdown();
  
  return 0;
}
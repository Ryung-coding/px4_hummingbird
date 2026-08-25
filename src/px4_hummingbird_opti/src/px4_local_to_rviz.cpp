#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include <Eigen/Core>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>

#include <params.hpp>
#include <utils.hpp>


class Px4LocalToRviz : public rclcpp::Node
{
public:
  Px4LocalToRviz()
  : rclcpp::Node("px4_local_to_rviz")
  {
    const std::string input_topic = this->declare_parameter<std::string>("input_topic", params::rviz_sub_name);
    const std::string output_topic = this->declare_parameter<std::string>("output_topic", params::rviz_pub_name);
    frame_id_ = this->declare_parameter<std::string>("frame_id", params::rviz_frame_id);

    px4_subscription_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      input_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&Px4LocalToRviz::px4_callback, this, std::placeholders::_1));

    rviz_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      output_topic,
      rclcpp::QoS(params::qos_depth));

    RCLCPP_INFO(this->get_logger(), "Subscribed: %s", input_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing : %s", output_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Frame ID   : %s", frame_id_.c_str());
  }

private:
  void px4_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
  {
    if (std::isnan(msg->x) || std::isnan(msg->y) || std::isnan(msg->z)) {
      return;
    }

    const Eigen::Vector3d pos_ned(msg->x, msg->y, msg->z);
    const Eigen::Vector3d pos_rviz = utils::px4PosToRvizEnu(pos_ned);

    Eigen::Vector4d quat_wxyz(1.0, 0.0, 0.0, 0.0);
    if (!std::isnan(msg->heading)) {
      const double yaw_rviz = utils::px4HeadingToRvizYaw(msg->heading);
      quat_wxyz = utils::rpyToQuat(Eigen::Vector3d(0.0, 0.0, yaw_rviz));
    }

    geometry_msgs::msg::PoseStamped pose{};
    pose.header.stamp = this->get_clock()->now();
    pose.header.frame_id = frame_id_;
    pose.pose.position.x = pos_rviz.x();
    pose.pose.position.y = pos_rviz.y();
    pose.pose.position.z = pos_rviz.z();
    pose.pose.orientation.x = quat_wxyz(1);
    pose.pose.orientation.y = quat_wxyz(2);
    pose.pose.orientation.z = quat_wxyz(3);
    pose.pose.orientation.w = quat_wxyz(0);

    rviz_publisher_->publish(pose);
  }

  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr px4_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr rviz_publisher_;

  std::string frame_id_{params::rviz_frame_id};
};


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Px4LocalToRviz>());
  rclcpp::shutdown();
  return 0;
}

#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <params.hpp>
#include <utils.hpp>


class MocapToPx4 : public rclcpp::Node
{
public:
  MocapToPx4()
  : rclcpp::Node("mocap_to_px4")
  {
    const std::string sub_mocap_topic = this->declare_parameter<std::string>("input_topic", params::mocap_pub_name);
    const std::string pub_px4_topic = this->declare_parameter<std::string>("output_topic", params::px4_dds_name);

    target_body_name_ = this->declare_parameter<std::string>("target_body_name", params::target_body_name);
    const std::vector<double> opti_origin = this->declare_parameter<std::vector<double>>("opti_origin", {params::opti_origin_m[0], params::opti_origin_m[1], params::opti_origin_m[2]});
    if (opti_origin.size() == 3) { opti_origin_ = Eigen::Vector3d(opti_origin[0], opti_origin[1], opti_origin[2]); }

    const double position_stddev_m = this->declare_parameter<double>("position_stddev", params::position_stddev_m);
    const double orientation_stddev_deg = this->declare_parameter<double>("orientation_stddev_deg", params::orientation_stddev_deg);

    quality_ = this->declare_parameter<int>("quality", params::quality);
    position_variance_ = static_cast<float>(position_stddev_m * position_stddev_m);
    const double orientation_stddev_rad = orientation_stddev_deg * M_PI / 180.0;
    orientation_variance_ = static_cast<float>(orientation_stddev_rad * orientation_stddev_rad);

    sub_mocap_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(sub_mocap_topic, rclcpp::QoS(params::qos_depth), std::bind(&MocapToPx4::onMocap, this, std::placeholders::_1));
    pub_px4_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(pub_px4_topic, rclcpp::QoS(params::qos_depth));

    RCLCPP_INFO(this->get_logger(), "Subscribed: %s", sub_mocap_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing : %s", pub_px4_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Target body: %s", target_body_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "OptiTrack origin offset (z-up): x=%.3f, y=%.3f, z=%.3f m", opti_origin_.x(), opti_origin_.y(), opti_origin_.z());
    RCLCPP_INFO(this->get_logger(), "Transform : pos_px4 = R_down * (opti_pos - opti_origin), R_px4 = R_down * R_opti * R_down");
  }

private:
  void onMocap(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!target_body_name_.empty() && msg->header.frame_id != target_body_name_) { return; }

    const rclcpp::Time now = this->get_clock()->now();
    const uint64_t timestamp_us = static_cast<uint64_t>(now.nanoseconds() / 1000);

    const Eigen::Vector3d opti_pos(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    const Eigen::Vector4d opti_att(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);

    const utils::Px4Pose pose_px4 = utils::mocapPoseToPx4(opti_pos, opti_att, opti_origin_);

    const float unknown = std::numeric_limits<float>::quiet_NaN();

    px4_msgs::msg::VehicleOdometry odom{};
    odom.timestamp = timestamp_us;
    odom.timestamp_sample = timestamp_us;
    odom.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
    odom.position = {static_cast<float>(pose_px4.pos.x()), static_cast<float>(pose_px4.pos.y()), static_cast<float>(pose_px4.pos.z())};
    odom.q = {static_cast<float>(pose_px4.att(0)), static_cast<float>(pose_px4.att(1)), static_cast<float>(pose_px4.att(2)), static_cast<float>(pose_px4.att(3))};
    odom.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_BODY_FRD;
    odom.velocity = {unknown, unknown, unknown};
    odom.angular_velocity = {unknown, unknown, unknown};
    odom.position_variance = {position_variance_, position_variance_, position_variance_};
    odom.orientation_variance = {orientation_variance_, orientation_variance_, orientation_variance_};
    odom.velocity_variance = {1.0e6F, 1.0e6F, 1.0e6F};
    odom.reset_counter = 0;
    odom.quality = static_cast<int8_t>(quality_);

    pub_px4_->publish(odom);
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_mocap_;
  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr pub_px4_;

  float position_variance_{0.0F};
  float orientation_variance_{0.0F};
  int quality_{params::quality};
  std::string target_body_name_{params::target_body_name};
  Eigen::Vector3d opti_origin_{params::opti_origin_m[0], params::opti_origin_m[1], params::opti_origin_m[2]};
};


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MocapToPx4>());
  rclcpp::shutdown();
  return 0;
}

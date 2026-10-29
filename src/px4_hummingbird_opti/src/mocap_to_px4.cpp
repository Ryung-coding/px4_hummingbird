#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <utils.hpp>


class MocapToPx4 : public rclcpp::Node
{
public:
  MocapToPx4()
  : rclcpp::Node("mocap_to_px4")
  {
    const std::string sub_mocap_topic = this->declare_parameter<std::string>("input_topic", "/nokov_raw/Hummingbird/pose");
    const std::string pub_mocap_topic = this->declare_parameter<std::string>("mocap_topic", "/opti_raw");
    const std::string pub_px4_topic = this->declare_parameter<std::string>("output_topic", "/fmu/in/vehicle_visual_odometry");

    target_body_name_ = this->declare_parameter<std::string>("target_body_name", "Hummingbird");
    position_scale_ = this->declare_parameter<double>("position_scale", 0.001);
    const std::vector<double> opti_origin = this->declare_parameter<std::vector<double>>("opti_origin", {0.0, 0.0, 0.0});
    if (opti_origin.size() == 3) { opti_origin_ = Eigen::Vector3d(opti_origin[0], opti_origin[1], opti_origin[2]); }

    const double position_stddev_m = this->declare_parameter<double>("position_stddev", 0.02);
    const double orientation_stddev_deg = this->declare_parameter<double>("orientation_stddev_deg", 3.0);

    quality_ = this->declare_parameter<int>("quality", 100);
    const int qos_depth = this->declare_parameter<int>("qos_depth", 10);
    const double output_rate_hz = this->declare_parameter<double>("output_rate_hz", 100.0);
    max_sample_age_sec_ = this->declare_parameter<double>("max_sample_age_sec", 0.1);
    if (output_rate_hz <= 0.0 || max_sample_age_sec_ <= 0.0) { throw std::invalid_argument("Invalid mocap output timing"); }
    position_variance_ = static_cast<float>(position_stddev_m * position_stddev_m);
    const double orientation_stddev_rad = orientation_stddev_deg * M_PI / 180.0;
    orientation_variance_ = static_cast<float>(orientation_stddev_rad * orientation_stddev_rad);

    sub_mocap_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(sub_mocap_topic, rclcpp::SensorDataQoS().keep_last(qos_depth), std::bind(&MocapToPx4::onMocap, this, std::placeholders::_1));
    pub_mocap_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(pub_mocap_topic, rclcpp::SensorDataQoS().keep_last(qos_depth));
    pub_px4_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(pub_px4_topic, rclcpp::QoS(qos_depth));
    output_timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / output_rate_hz), std::bind(&MocapToPx4::publishPx4, this));
    node_start_steady_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(this->get_logger(), "Subscribed: %s", sub_mocap_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing : %s", pub_mocap_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing : %s", pub_px4_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Target body: %s", target_body_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "OptiTrack origin offset (z-up): x=%.3f, y=%.3f, z=%.3f m", opti_origin_.x(), opti_origin_.y(), opti_origin_.z());
    RCLCPP_INFO(this->get_logger(), "Input position scale: %.6f", position_scale_);
    RCLCPP_INFO(this->get_logger(), "Transform : pos_px4 = R_down * (opti_pos - opti_origin), R_px4 = R_down * R_opti * R_down");
  }

private:
  void onMocap(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!target_body_name_.empty() && msg->header.frame_id != target_body_name_) { return; }

    const auto receive_steady = std::chrono::steady_clock::now();
    if (has_input_)
    {
      const double gap_sec = std::chrono::duration<double>(receive_steady - last_input_steady_).count();
      if (gap_sec >= 0.05)
      {
        RCLCPP_WARN(this->get_logger(), "OPTI_INPUT_GAP host=%.3fs", gap_sec);
      }
      if (input_missing_)
      {
        RCLCPP_INFO(this->get_logger(), "OPTI_INPUT_RECOVERED gap=%.3fs", gap_sec);
        input_missing_ = false;
      }
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "OPTI_INPUT_FIRST_SAMPLE");
      input_missing_ = false;
    }
    has_input_ = true;
    last_input_steady_ = receive_steady;

    const rclcpp::Time now = this->get_clock()->now();
    const rclcpp::Time sample_time(msg->header.stamp, now.get_clock_type());
    const bool valid_sample_time = sample_time.nanoseconds() > 0 && sample_time <= now;
    const rclcpp::Time measurement_time = valid_sample_time ? sample_time : now;

    const Eigen::Vector3d opti_pos = position_scale_ * Eigen::Vector3d(
      msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    const Eigen::Vector4d opti_att(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);

    if (!opti_pos.allFinite() || !opti_att.allFinite() || opti_att.norm() < 1.0e-6)
    {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "OPTI_INVALID_POSE nonfinite position/orientation or zero quaternion");
      return;
    }

    geometry_msgs::msg::PoseStamped opti_msg = *msg;
    opti_msg.pose.position.x = opti_pos.x();
    opti_msg.pose.position.y = opti_pos.y();
    opti_msg.pose.position.z = opti_pos.z();
    pub_mocap_->publish(opti_msg);

    const utils::Px4Pose pose_px4 = utils::mocapPoseToPx4(opti_pos, opti_att, opti_origin_);

    const float unknown = std::numeric_limits<float>::quiet_NaN();

    px4_msgs::msg::VehicleOdometry odom{};
    odom.timestamp_sample = static_cast<uint64_t>(measurement_time.nanoseconds() / 1000);
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

    pending_odom_ = odom;
  }

  void publishPx4()
  {
    if (std::chrono::duration<double>(std::chrono::steady_clock::now() -
        (has_input_ ? last_input_steady_ : node_start_steady_)).count() >= 0.2)
    {
      if (!input_missing_)
      {
        const double age_sec = std::chrono::duration<double>(std::chrono::steady_clock::now() -
          (has_input_ ? last_input_steady_ : node_start_steady_)).count();
        RCLCPP_ERROR(this->get_logger(), "OPTI_INPUT_TIMEOUT received=%d age=%.3fs; visual odometry cannot be refreshed",
          has_input_, age_sec);
        input_missing_ = true;
      }
    }
    if (!pending_odom_) return;

    const rclcpp::Time now = this->get_clock()->now();
    const rclcpp::Time sample_time(static_cast<int64_t>(pending_odom_->timestamp_sample) * 1000, now.get_clock_type());
    const double sample_age_sec = (now - sample_time).seconds();

    if (sample_age_sec >= 0.0 && sample_age_sec <= max_sample_age_sec_)
    {
      pending_odom_->timestamp = static_cast<uint64_t>(now.nanoseconds() / 1000);
      pub_px4_->publish(*pending_odom_);
    }
    else
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "OPTI_SAMPLE_STALE age=%.3fs; visual odometry not published", sample_age_sec);
    }

    pending_odom_.reset();
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_mocap_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_mocap_;
  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr pub_px4_;
  rclcpp::TimerBase::SharedPtr output_timer_;
  std::optional<px4_msgs::msg::VehicleOdometry> pending_odom_;
  double max_sample_age_sec_{0.1};
  std::chrono::steady_clock::time_point node_start_steady_{};
  std::chrono::steady_clock::time_point last_input_steady_{};
  bool has_input_{false};
  bool input_missing_{false};

  float position_variance_{0.0F};
  float orientation_variance_{0.0F};
  int quality_{100};
  double position_scale_{0.001};
  std::string target_body_name_{"Hummingbird"};
  Eigen::Vector3d opti_origin_{0.0, 0.0, 0.0};
};


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MocapToPx4>());
  rclcpp::shutdown();
  return 0;
}

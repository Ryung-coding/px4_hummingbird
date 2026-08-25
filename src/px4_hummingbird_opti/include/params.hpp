#pragma once

#include <array>

namespace params {

// Mocap input -------------------------------------------------------
static constexpr char mocap_pub_name[] = "/opti_raw";

// Raw OptiTrack z-up position measured when the drone is placed at initial position.
static constexpr std::array<double, 3> opti_origin_m{1.4, 1.4, 0.0};

// PX4 DDS input -----------------------------------------------------
static constexpr char px4_dds_name[] = "/fmu/in/vehicle_visual_odometry";
static constexpr char target_body_name[] = "hummingbird";
static constexpr int qos_depth = 10;

// Vision odometry quality ------------------------------------------
static constexpr double position_stddev_m = 0.02;
static constexpr double orientation_stddev_deg = 3.0;
static constexpr int quality = 100;

}  // namespace params

#pragma once

#include <cmath>
#include <array>

namespace params {

// System Setting -----------------------------------------------
static constexpr bool USE_SO3_HEADING_CMD = false;
static constexpr int RATE_HZ = 400;

// Task parameters-----------------------------------------------
static constexpr double HOVER_SEC = 3.0;
static constexpr double SCAN_PERIOD_SEC = 30.0;

static constexpr double APPLE_X = 1.0;
static constexpr double APPLE_Y = 0.0;
static constexpr double APPLE_Z = 3.0;
static constexpr double RADIUS = 1.5;
static constexpr double ROLL_MAX = 30.0 * M_PI / 180.0;
static constexpr double THETA_MAX = 60.0 * M_PI / 180.0;

}

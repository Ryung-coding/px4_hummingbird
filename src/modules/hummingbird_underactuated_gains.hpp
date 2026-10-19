/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be used to
 *    endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES ARE DISCLAIMED.
 *
 ****************************************************************************/

#pragma once

namespace hummingbird_underactuated_gains
{
// HB_CTRL_MODE=0 uses these values; HB_CTRL_MODE=1 uses the MPC_/MC_ parameters.
static constexpr float kTransitionTimeS = 0.1f;
static constexpr int kAirframe = 16;
static constexpr int kUnderactuatedMode = 0;

static constexpr float kPositionXyP = 0.95f;
static constexpr float kPositionZP = 1.0f;
static constexpr float kVelocityXyP = 1.8f;
static constexpr float kVelocityZP = 4.0f;
static constexpr float kVelocityXyI = 0.4f;
static constexpr float kVelocityZI = 2.0f;
static constexpr float kVelocityXyD = 0.2f;
static constexpr float kVelocityZD = 0.0f;

static constexpr float kAttitudeRollP = 4.0f;
static constexpr float kAttitudePitchP = 4.0f;
static constexpr float kAttitudeYawP = 2.8f;
static constexpr float kAttitudeYawWeight = 0.4f;

static constexpr float kRateRollP = 0.15f;
static constexpr float kRateRollI = 0.2f;
static constexpr float kRateRollD = 0.003f;
static constexpr float kRateRollFF = 0.0f;
static constexpr float kRateRollK = 1.0f;
static constexpr float kRatePitchP = 0.15f;
static constexpr float kRatePitchI = 0.2f;
static constexpr float kRatePitchD = 0.003f;
static constexpr float kRatePitchFF = 0.0f;
static constexpr float kRatePitchK = 1.0f;
static constexpr float kRateYawP = 0.2f;
static constexpr float kRateYawI = 0.1f;
static constexpr float kRateYawD = 0.0f;
static constexpr float kRateYawFF = 0.0f;
static constexpr float kRateYawK = 1.0f;
static constexpr float kRateRollIntegralLimit = 0.30f;
static constexpr float kRatePitchIntegralLimit = 0.30f;
static constexpr float kRateYawIntegralLimit = 0.30f;

inline float blend(float full_value, float underactuated_value, float underactuated_fraction)
{
	if (underactuated_fraction <= 0.f) {
		return full_value;
	}

	if (underactuated_fraction >= 1.f) {
		return underactuated_value;
	}

	return full_value + (underactuated_value - full_value) * underactuated_fraction;
}
}

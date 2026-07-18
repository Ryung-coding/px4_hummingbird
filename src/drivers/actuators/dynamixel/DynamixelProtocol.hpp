/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include <stdint.h>
#include <stddef.h>

namespace dynamixel
{

static constexpr uint8_t kMaxServos = 8;
static constexpr uint8_t kBroadcastId = 0xfe;
static constexpr uint8_t kInstructionSyncWrite = 0x83;
static constexpr uint16_t kGoalPositionAddress = 116;
static constexpr uint16_t kGoalPositionLength = 4;
static constexpr uint16_t kTorqueEnableAddress = 64;
static constexpr uint16_t kTorqueEnableLength = 1;
static constexpr size_t kSyncWriteGoalPositionMaxPacketSize = 14 + (kMaxServos * (1 + kGoalPositionLength));
static constexpr size_t kSyncWriteTorqueEnableMaxPacketSize = 14 + (kMaxServos * (1 + kTorqueEnableLength));

uint16_t crc16(const uint8_t *data, size_t length);

size_t makeSyncWriteGoalPositionPacket(const uint8_t *ids, const uint32_t *goals, uint8_t count, uint8_t *buffer,
				       size_t buffer_length);

size_t makeSyncWriteTorqueEnablePacket(const uint8_t *ids, uint8_t count, bool enable, uint8_t *buffer,
				       size_t buffer_length);

} // namespace dynamixel

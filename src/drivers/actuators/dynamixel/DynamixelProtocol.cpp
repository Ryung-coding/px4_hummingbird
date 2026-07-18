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
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "DynamixelProtocol.hpp"

namespace dynamixel
{

namespace
{

void putU16(uint8_t *buffer, size_t &offset, uint16_t value)
{
	buffer[offset++] = value & 0xff;
	buffer[offset++] = (value >> 8) & 0xff;
}

void putU32(uint8_t *buffer, size_t &offset, uint32_t value)
{
	buffer[offset++] = value & 0xff;
	buffer[offset++] = (value >> 8) & 0xff;
	buffer[offset++] = (value >> 16) & 0xff;
	buffer[offset++] = (value >> 24) & 0xff;
}

} // namespace

uint16_t crc16(const uint8_t *data, size_t length)
{
	uint16_t crc = 0;

	for (size_t i = 0; i < length; ++i) {
		crc ^= static_cast<uint16_t>(data[i]) << 8;

		for (int bit = 0; bit < 8; ++bit) {
			if (crc & 0x8000) {
				crc = static_cast<uint16_t>((crc << 1) ^ 0x8005);

			} else {
				crc = static_cast<uint16_t>(crc << 1);
			}
		}
	}

	return crc;
}

size_t makeSyncWriteGoalPositionPacket(const uint8_t *ids, const uint32_t *goals, uint8_t count, uint8_t *buffer,
				       size_t buffer_length)
{
	if (ids == nullptr || goals == nullptr || buffer == nullptr || count == 0 || count > kMaxServos) {
		return 0;
	}

	const size_t packet_length = 14 + (static_cast<size_t>(count) * (1 + kGoalPositionLength));

	if (buffer_length < packet_length) {
		return 0;
	}

	size_t offset = 0;
	buffer[offset++] = 0xff;
	buffer[offset++] = 0xff;
	buffer[offset++] = 0xfd;
	buffer[offset++] = 0x00;
	buffer[offset++] = kBroadcastId;

	const uint16_t parameter_length = 4 + (count * (1 + kGoalPositionLength));
	const uint16_t length_field = parameter_length + 3; // instruction + parameters + CRC
	putU16(buffer, offset, length_field);

	buffer[offset++] = kInstructionSyncWrite;
	putU16(buffer, offset, kGoalPositionAddress);
	putU16(buffer, offset, kGoalPositionLength);

	for (uint8_t i = 0; i < count; ++i) {
		buffer[offset++] = ids[i];
		putU32(buffer, offset, goals[i]);
	}

	const uint16_t crc = crc16(buffer, offset);
	putU16(buffer, offset, crc);

	return offset;
}

size_t makeSyncWriteTorqueEnablePacket(const uint8_t *ids, uint8_t count, bool enable, uint8_t *buffer,
				       size_t buffer_length)
{
	if (ids == nullptr || buffer == nullptr || count == 0 || count > kMaxServos) {
		return 0;
	}

	const size_t packet_length = 14 + (static_cast<size_t>(count) * (1 + kTorqueEnableLength));

	if (buffer_length < packet_length) {
		return 0;
	}

	size_t offset = 0;
	buffer[offset++] = 0xff;
	buffer[offset++] = 0xff;
	buffer[offset++] = 0xfd;
	buffer[offset++] = 0x00;
	buffer[offset++] = kBroadcastId;

	const uint16_t parameter_length = 4 + (count * (1 + kTorqueEnableLength));
	const uint16_t length_field = parameter_length + 3;
	putU16(buffer, offset, length_field);

	buffer[offset++] = kInstructionSyncWrite;
	putU16(buffer, offset, kTorqueEnableAddress);
	putU16(buffer, offset, kTorqueEnableLength);

	for (uint8_t i = 0; i < count; ++i) {
		buffer[offset++] = ids[i];
		buffer[offset++] = enable ? 1 : 0;
	}

	const uint16_t crc = crc16(buffer, offset);
	putU16(buffer, offset, crc);

	return offset;
}

} // namespace dynamixel

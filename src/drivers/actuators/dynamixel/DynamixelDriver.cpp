/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "DynamixelDriver.hpp"

#include <fcntl.h>
#include <math.h>
#include <mathlib/mathlib.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

ModuleBase::Descriptor DynamixelDriver::desc{DynamixelDriver::task_spawn, DynamixelDriver::custom_command,
					     DynamixelDriver::print_usage};

DynamixelDriver::DynamixelDriver(const char *device) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
	strncpy(_device, device, sizeof(_device) - 1);
	_device[sizeof(_device) - 1] = '\0';

	for (float &input : _last_input) {
		input = NAN;
	}
}

DynamixelDriver::~DynamixelDriver()
{
	ScheduleClear();
	closeUart();
}

bool DynamixelDriver::init()
{
	updateParams();
	updateServoConfig();

	if (!configureUart()) {
		return false;
	}

	ScheduleOnInterval(scheduleIntervalUs());
	return true;
}

void DynamixelDriver::Run()
{
	if (should_exit()) {
		exit_and_cleanup(desc);
		return;
	}

	updateParameters();

	vehicle_status_s vehicle_status{};

	if (_vehicle_status_sub.copy(&vehicle_status)) {
		_armed = vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
	}

	_hummingbird_mode = _param_sys_autostart.get() == _param_dxl_hb_autostart.get();

	actuator_servos_s actuator_servos{};

	if (_actuator_servos_sub.update(&actuator_servos)) {
		_last_actuator_servos = actuator_servos;
		_last_actuator_servos_time = actuator_servos.timestamp;
		_have_valid_actuator_servos = true;
	}

	if (_uart_fd < 0) {
		return;
	}

	const uint8_t count = _configured_count;

	if (count == 0) {
		return;
	}

	if (_param_dxl_kill.get()) {
		if (_torque_enabled) {
			sendTorqueEnable(false, count);
		}

		return;
	}

	if (!_param_dxl_enable.get()) {
		return;
	}

	if (_hummingbird_mode && _param_dxl_arm_only.get() && !_armed) {
		return;
	}

	if (_hummingbird_mode && !_have_valid_actuator_servos) {
		return;
	}

	uint8_t ids[dynamixel::kMaxServos]{};
	uint32_t goals[dynamixel::kMaxServos]{};

	if (!_torque_enabled && !sendTorqueEnable(true, count)) {
		return;
	}

	if (_hummingbird_mode) {
		if (!buildGoalPositions(_last_actuator_servos, count, ids, goals)) {
			return;
		}

	} else {
		buildCenterGoalPositions(count, ids, goals);
	}

	uint8_t packet[dynamixel::kSyncWriteGoalPositionMaxPacketSize]{};
	const size_t packet_length = dynamixel::makeSyncWriteGoalPositionPacket(ids, goals, count, packet, sizeof(packet));

	if (packet_length == 0) {
		++_write_fail_count;
		return;
	}

	if (writePacket(packet, packet_length)) {
		_last_tx_time = hrt_absolute_time();
		++_tx_packet_count;

	} else {
		++_write_fail_count;
	}
}

void DynamixelDriver::updateParameters()
{
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update{};
		_parameter_update_sub.copy(&param_update);
		updateParams();
		updateServoConfig();
		ScheduleOnInterval(scheduleIntervalUs());
	}
}

bool DynamixelDriver::configureUart()
{
	_uart_fd = ::open(_device, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (_uart_fd < 0) {
		PX4_ERR("open %s failed", _device);
		return false;
	}

	termios uart_config{};

	if (tcgetattr(_uart_fd, &uart_config) < 0) {
		PX4_ERR("tcgetattr failed");
		closeUart();
		return false;
	}

	cfmakeraw(&uart_config);
	uart_config.c_cflag |= (CLOCAL | CREAD);
	uart_config.c_cflag &= ~CRTSCTS;
	uart_config.c_cc[VMIN] = 0;
	uart_config.c_cc[VTIME] = 0;

	const speed_t speed = baudToSpeed(_param_dxl_baud.get());

	if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
		PX4_ERR("baud %ld not supported", static_cast<long>(_param_dxl_baud.get()));
		closeUart();
		return false;
	}

	if (tcsetattr(_uart_fd, TCSANOW, &uart_config) < 0) {
		PX4_ERR("tcsetattr failed");
		closeUart();
		return false;
	}

	return true;
}

void DynamixelDriver::closeUart()
{
	if (_uart_fd >= 0) {
		::close(_uart_fd);
		_uart_fd = -1;
	}
}

speed_t DynamixelDriver::baudToSpeed(int32_t baud) const
{
	switch (baud) {
	case 57600: return B57600;
	case 115200: return B115200;
	case 230400: return B230400;
	case 460800: return B460800;
	case 921600: return B921600;
#ifdef B1000000
	case 1000000: return B1000000;
#endif
#ifdef B1500000
	case 1500000: return B1500000;
#endif
#ifdef B2000000
	case 2000000: return B2000000;
#endif
	default:
#ifdef B1000000
		return B1000000;
#else
		return B115200;
#endif
	}
}

bool DynamixelDriver::updateServoConfig()
{
	const ServoConfig configs[dynamixel::kMaxServos] = {
		{_param_dxl_1_id.get(), _param_dxl_1_min.get(), _param_dxl_1_center.get(), _param_dxl_1_max.get(), _param_dxl_1_reverse.get()},
		{_param_dxl_2_id.get(), _param_dxl_2_min.get(), _param_dxl_2_center.get(), _param_dxl_2_max.get(), _param_dxl_2_reverse.get()},
		{_param_dxl_3_id.get(), _param_dxl_3_min.get(), _param_dxl_3_center.get(), _param_dxl_3_max.get(), _param_dxl_3_reverse.get()},
		{_param_dxl_4_id.get(), _param_dxl_4_min.get(), _param_dxl_4_center.get(), _param_dxl_4_max.get(), _param_dxl_4_reverse.get()},
		{_param_dxl_5_id.get(), _param_dxl_5_min.get(), _param_dxl_5_center.get(), _param_dxl_5_max.get(), _param_dxl_5_reverse.get()},
		{_param_dxl_6_id.get(), _param_dxl_6_min.get(), _param_dxl_6_center.get(), _param_dxl_6_max.get(), _param_dxl_6_reverse.get()},
		{_param_dxl_7_id.get(), _param_dxl_7_min.get(), _param_dxl_7_center.get(), _param_dxl_7_max.get(), _param_dxl_7_reverse.get()},
		{_param_dxl_8_id.get(), _param_dxl_8_min.get(), _param_dxl_8_center.get(), _param_dxl_8_max.get(), _param_dxl_8_reverse.get()},
	};

	const uint8_t count = static_cast<uint8_t>(math::constrain(_param_dxl_count.get(), static_cast<int32_t>(0), static_cast<int32_t>(dynamixel::kMaxServos)));

	for (uint8_t i = 0; i < count; ++i) {
		_servo_config[i] = configs[i];
		_servo_config[i].id = math::constrain(_servo_config[i].id, static_cast<int32_t>(1), static_cast<int32_t>(252));
		_servo_config[i].min = math::constrain(_servo_config[i].min, static_cast<int32_t>(0), static_cast<int32_t>(4095));
		_servo_config[i].center = math::constrain(_servo_config[i].center, static_cast<int32_t>(0), static_cast<int32_t>(4095));
		_servo_config[i].max = math::constrain(_servo_config[i].max, static_cast<int32_t>(0), static_cast<int32_t>(4095));

		if (_servo_config[i].min > _servo_config[i].max) {
			const int32_t min_value = _servo_config[i].max;
			_servo_config[i].max = _servo_config[i].min;
			_servo_config[i].min = min_value;
		}

		_servo_config[i].center = math::constrain(_servo_config[i].center, _servo_config[i].min, _servo_config[i].max);
	}

	_configured_count = count;
	return true;
}

bool DynamixelDriver::buildGoalPositions(const actuator_servos_s &servos, uint8_t count, uint8_t *ids, uint32_t *goals)
{
	for (uint8_t i = 0; i < count; ++i) {
		const float input = servos.control[i];

		if (!PX4_ISFINITE(input)) {
			return false;
		}

		ids[i] = static_cast<uint8_t>(_servo_config[i].id);
		goals[i] = normalizedToGoal(input, _servo_config[i]);
		_last_input[i] = math::constrain(input, -1.f, 1.f);
		_last_goal[i] = goals[i];
	}

	_last_count = count;
	return true;
}

void DynamixelDriver::buildCenterGoalPositions(uint8_t count, uint8_t *ids, uint32_t *goals)
{
	for (uint8_t i = 0; i < count; ++i) {
		ids[i] = static_cast<uint8_t>(_servo_config[i].id);
		goals[i] = normalizedToGoal(0.f, _servo_config[i]);
		_last_input[i] = 0.f;
		_last_goal[i] = goals[i];
	}

	_last_count = count;
}

uint32_t DynamixelDriver::normalizedToGoal(float input, const ServoConfig &config) const
{
	float u = math::constrain(input, -1.f, 1.f);

	if (config.reverse) {
		u = -u;
	}

	float goal = static_cast<float>(config.center);

	if (u >= 0.f) {
		goal += u * static_cast<float>(config.max - config.center);

	} else {
		goal += u * static_cast<float>(config.center - config.min);
	}

	return static_cast<uint32_t>(math::constrain(lroundf(goal), config.min, config.max));
}

bool DynamixelDriver::writePacket(const uint8_t *packet, size_t packet_length)
{
	const ssize_t ret = ::write(_uart_fd, packet, packet_length);
	return ret == static_cast<ssize_t>(packet_length);
}

bool DynamixelDriver::sendTorqueEnable(bool enable, uint8_t count)
{
	uint8_t ids[dynamixel::kMaxServos]{};

	for (uint8_t i = 0; i < count; ++i) {
		ids[i] = static_cast<uint8_t>(_servo_config[i].id);
	}

	uint8_t packet[dynamixel::kSyncWriteTorqueEnableMaxPacketSize]{};
	const size_t packet_length = dynamixel::makeSyncWriteTorqueEnablePacket(ids, count, enable, packet, sizeof(packet));

	if (packet_length == 0) {
		++_write_fail_count;
		return false;
	}

	if (!writePacket(packet, packet_length)) {
		++_write_fail_count;
		return false;
	}

	_torque_enabled = enable;
	_last_tx_time = hrt_absolute_time();
	++_torque_packet_count;
	return true;
}

hrt_abstime DynamixelDriver::scheduleIntervalUs() const
{
	const int32_t rate_hz = math::constrain(_param_dxl_rate.get(), static_cast<int32_t>(1), static_cast<int32_t>(400));
	return 1000000 / rate_hz;
}

int DynamixelDriver::print_status()
{
	PX4_INFO("running");
	PX4_INFO("device: %s", _device);
	PX4_INFO("baud: %ld", static_cast<long>(_param_dxl_baud.get()));
	PX4_INFO("rate: %ld Hz", static_cast<long>(_param_dxl_rate.get()));
	PX4_INFO("enable: %d", _param_dxl_enable.get());
	PX4_INFO("servo kill: %d torque enabled: %d", _param_dxl_kill.get(), _torque_enabled);
	PX4_INFO("hummingbird mode: %d SYS_AUTOSTART=%ld target=%ld",
		 _hummingbird_mode,
		 static_cast<long>(_param_sys_autostart.get()),
		 static_cast<long>(_param_dxl_hb_autostart.get()));
	PX4_INFO("arm only: %d armed: %d", _param_dxl_arm_only.get(), _armed);
	PX4_INFO("last actuator_servos: %llu us", static_cast<unsigned long long>(_last_actuator_servos_time));
	PX4_INFO("last tx: %llu us", static_cast<unsigned long long>(_last_tx_time));
	PX4_INFO("goal packets: %llu torque packets: %llu write failures: %llu",
		 static_cast<unsigned long long>(_tx_packet_count),
		 static_cast<unsigned long long>(_torque_packet_count),
		 static_cast<unsigned long long>(_write_fail_count));

	for (uint8_t i = 0; i < _last_count; ++i) {
		PX4_INFO("ch%u id=%ld input=%.4f goal=%lu",
			 static_cast<unsigned>(i + 1),
			 static_cast<long>(_servo_config[i].id),
			 static_cast<double>(_last_input[i]),
			 static_cast<unsigned long>(_last_goal[i]));
	}

	return 0;
}

int DynamixelDriver::task_spawn(int argc, char *argv[])
{
	const char *device_path = nullptr;
	int ch = 0;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device_path = myoptarg;
			break;

		default:
			return print_usage("unknown option");
		}
	}

	if (device_path == nullptr) {
		return print_usage("missing device");
	}

	DynamixelDriver *instance = new DynamixelDriver(device_path);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	desc.object.store(instance);
	desc.task_id = task_id_is_work_queue;

	if (instance->init()) {
		return PX4_OK;
	}

	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;

	return PX4_ERROR;
}

int DynamixelDriver::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int DynamixelDriver::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_USAGE_NAME("dynamixel", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS3", "<file:dev>", "UART device", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "print driver status");
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "stop driver");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int dynamixel_main(int argc, char *argv[])
{
	return ModuleBase::main(DynamixelDriver::desc, argc, argv);
}

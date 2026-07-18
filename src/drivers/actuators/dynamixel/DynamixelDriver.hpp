/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include "DynamixelProtocol.hpp"

#include <drivers/drv_hrt.h>
#include <termios.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/actuator_servos.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_status.h>

using namespace time_literals;

class DynamixelDriver : public ModuleBase, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	static Descriptor desc;

	explicit DynamixelDriver(const char *device);
	~DynamixelDriver() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	int print_status() override;

private:
	struct ServoConfig {
		int32_t id{0};
		int32_t min{0};
		int32_t center{2048};
		int32_t max{4095};
		bool reverse{false};
	};

	void Run() override;
	void updateParameters();
	bool configureUart();
	void closeUart();
	speed_t baudToSpeed(int32_t baud) const;
	bool updateServoConfig();
	bool buildGoalPositions(const actuator_servos_s &servos, uint8_t count, uint8_t *ids, uint32_t *goals);
	void buildCenterGoalPositions(uint8_t count, uint8_t *ids, uint32_t *goals);
	uint32_t normalizedToGoal(float input, const ServoConfig &config) const;
	bool writePacket(const uint8_t *packet, size_t packet_length);
	bool sendTorqueEnable(bool enable, uint8_t count);
	hrt_abstime scheduleIntervalUs() const;

	char _device[20]{};
	int _uart_fd{-1};

	uORB::Subscription _actuator_servos_sub{ORB_ID(actuator_servos)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	actuator_servos_s _last_actuator_servos{};

	ServoConfig _servo_config[dynamixel::kMaxServos]{};
	float _last_input[dynamixel::kMaxServos]{};
	uint32_t _last_goal[dynamixel::kMaxServos]{};
	uint8_t _last_count{0};
	uint8_t _configured_count{0};

	hrt_abstime _last_actuator_servos_time{0};
	hrt_abstime _last_tx_time{0};
	uint64_t _tx_packet_count{0};
	uint64_t _torque_packet_count{0};
	uint64_t _write_fail_count{0};
	bool _have_valid_actuator_servos{false};
	bool _armed{false};
	bool _torque_enabled{false};
	bool _hummingbird_mode{false};

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::DXL_ENABLE>) _param_dxl_enable,
		(ParamBool<px4::params::DXL_KILL>) _param_dxl_kill,
		(ParamInt<px4::params::DXL_BAUD>) _param_dxl_baud,
		(ParamInt<px4::params::DXL_RATE>) _param_dxl_rate,
		(ParamInt<px4::params::DXL_COUNT>) _param_dxl_count,
		(ParamBool<px4::params::DXL_ARM_ONLY>) _param_dxl_arm_only,
		(ParamInt<px4::params::DXL_HB_AUTOST>) _param_dxl_hb_autostart,
		(ParamInt<px4::params::SYS_AUTOSTART>) _param_sys_autostart,
		(ParamInt<px4::params::DXL_1_ID>) _param_dxl_1_id,
		(ParamInt<px4::params::DXL_1_MIN>) _param_dxl_1_min,
		(ParamInt<px4::params::DXL_1_CEN>) _param_dxl_1_center,
		(ParamInt<px4::params::DXL_1_MAX>) _param_dxl_1_max,
		(ParamBool<px4::params::DXL_1_REV>) _param_dxl_1_reverse,
		(ParamInt<px4::params::DXL_2_ID>) _param_dxl_2_id,
		(ParamInt<px4::params::DXL_2_MIN>) _param_dxl_2_min,
		(ParamInt<px4::params::DXL_2_CEN>) _param_dxl_2_center,
		(ParamInt<px4::params::DXL_2_MAX>) _param_dxl_2_max,
		(ParamBool<px4::params::DXL_2_REV>) _param_dxl_2_reverse,
		(ParamInt<px4::params::DXL_3_ID>) _param_dxl_3_id,
		(ParamInt<px4::params::DXL_3_MIN>) _param_dxl_3_min,
		(ParamInt<px4::params::DXL_3_CEN>) _param_dxl_3_center,
		(ParamInt<px4::params::DXL_3_MAX>) _param_dxl_3_max,
		(ParamBool<px4::params::DXL_3_REV>) _param_dxl_3_reverse,
		(ParamInt<px4::params::DXL_4_ID>) _param_dxl_4_id,
		(ParamInt<px4::params::DXL_4_MIN>) _param_dxl_4_min,
		(ParamInt<px4::params::DXL_4_CEN>) _param_dxl_4_center,
		(ParamInt<px4::params::DXL_4_MAX>) _param_dxl_4_max,
		(ParamBool<px4::params::DXL_4_REV>) _param_dxl_4_reverse,
		(ParamInt<px4::params::DXL_5_ID>) _param_dxl_5_id,
		(ParamInt<px4::params::DXL_5_MIN>) _param_dxl_5_min,
		(ParamInt<px4::params::DXL_5_CEN>) _param_dxl_5_center,
		(ParamInt<px4::params::DXL_5_MAX>) _param_dxl_5_max,
		(ParamBool<px4::params::DXL_5_REV>) _param_dxl_5_reverse,
		(ParamInt<px4::params::DXL_6_ID>) _param_dxl_6_id,
		(ParamInt<px4::params::DXL_6_MIN>) _param_dxl_6_min,
		(ParamInt<px4::params::DXL_6_CEN>) _param_dxl_6_center,
		(ParamInt<px4::params::DXL_6_MAX>) _param_dxl_6_max,
		(ParamBool<px4::params::DXL_6_REV>) _param_dxl_6_reverse,
		(ParamInt<px4::params::DXL_7_ID>) _param_dxl_7_id,
		(ParamInt<px4::params::DXL_7_MIN>) _param_dxl_7_min,
		(ParamInt<px4::params::DXL_7_CEN>) _param_dxl_7_center,
		(ParamInt<px4::params::DXL_7_MAX>) _param_dxl_7_max,
		(ParamBool<px4::params::DXL_7_REV>) _param_dxl_7_reverse,
		(ParamInt<px4::params::DXL_8_ID>) _param_dxl_8_id,
		(ParamInt<px4::params::DXL_8_MIN>) _param_dxl_8_min,
		(ParamInt<px4::params::DXL_8_CEN>) _param_dxl_8_center,
		(ParamInt<px4::params::DXL_8_MAX>) _param_dxl_8_max,
		(ParamBool<px4::params::DXL_8_REV>) _param_dxl_8_reverse
	)
};

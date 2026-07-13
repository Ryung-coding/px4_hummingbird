/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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

/**
 * @file ControlAllocator.cpp
 *
 * Control allocator.
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

#include "ControlAllocator.hpp"

#include <drivers/drv_hrt.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>

using namespace matrix;
using namespace time_literals;

ModuleBase::Descriptor ControlAllocator::desc{task_spawn, custom_command, print_usage};

namespace
{
constexpr float kHummingBirdLx = 0.1861f;
constexpr float kHummingBirdLy = 0.1861f;
constexpr float kHummingBirdD = 0.0500f;
constexpr float kHummingBirdZeta = 0.0200f;
constexpr float kHummingBirdVirtualLambda = 1.0e-4f;
constexpr float kHummingBirdFMin = 1.0e-3f;
constexpr float kHummingBirdMaxRotVelocity = 1000.0f;
constexpr float kHummingBirdMaxRotorThrust = 12.0f;
constexpr float kHummingBirdMaxTotalThrust = 4.0f * kHummingBirdMaxRotorThrust;
constexpr float kHummingBirdMaxMoment = kHummingBirdMaxTotalThrust * kHummingBirdLx;
constexpr float kHummingBirdThetaLimitRad = M_PI_F;
constexpr float kHummingBirdPhiLimitRad = M_PI_F / 6.0f;

struct HummingBirdAllocationOutput
{
	matrix::Vector<float, 4> f{};
	matrix::Vector<float, 4> theta{};
	matrix::Vector<float, 4> phi{};
};

float mean_angle(float a, float b)
{
	return atan2f(sinf(a) + sinf(b), cosf(a) + cosf(b));
}

HummingBirdAllocationOutput allocation_P2T2(const matrix::Vector3f &moment_cmd, const matrix::Vector3f &force_cmd,
		const matrix::Vector<float, 4> &theta_measured, const matrix::Vector<float, 4> &phi_measured)
{
	using Matrix68f = matrix::Matrix<float, 6, 8>;
	using Vector6f = matrix::Vector<float, 6>;
	using Vector8f = matrix::Vector<float, 8>;

	const float theta_front = mean_angle(theta_measured(0), theta_measured(3));
	const float theta_back = mean_angle(theta_measured(1), theta_measured(2));

	const float phi14 = 0.5f * (phi_measured(0) + phi_measured(3));
	const float phi23 = 0.5f * (phi_measured(1) + phi_measured(2));

	const float front_ex = -sinf(theta_front) * cosf(phi14);
	const float front_ey = sinf(phi14);
	const float front_ez = -cosf(theta_front) * cosf(phi14);

	const float back_ex = -sinf(theta_back) * cosf(phi23);
	const float back_ey = sinf(phi23);
	const float back_ez = -cosf(theta_back) * cosf(phi23);

	const float front_x = kHummingBirdLx - kHummingBirdD * cosf(phi14) * sinf(theta_front);
	const float front_y = kHummingBirdD * sinf(phi14);

	const float back_x = -kHummingBirdLx - kHummingBirdD * cosf(phi23) * sinf(theta_back);
	const float back_y = kHummingBirdD * sinf(phi23);

	Matrix68f A{};

	A(0, 2) = front_y;
	A(0, 5) = back_y;
	A(0, 6) = kHummingBirdLy * front_ez - kHummingBirdZeta * front_ex;
	A(0, 7) = kHummingBirdLy * back_ez + kHummingBirdZeta * back_ex;

	A(1, 2) = -front_x;
	A(1, 5) = -back_x;
	A(1, 6) = -kHummingBirdZeta * front_ey;
	A(1, 7) = kHummingBirdZeta * back_ey;

	A(2, 0) = -front_y;
	A(2, 1) = front_x;
	A(2, 3) = -back_y;
	A(2, 4) = back_x;
	A(2, 6) = -kHummingBirdLy * front_ex - kHummingBirdZeta * front_ez;
	A(2, 7) = -kHummingBirdLy * back_ex + kHummingBirdZeta * back_ez;

	A(3, 0) = 1.0f;
	A(3, 3) = 1.0f;

	A(4, 1) = 1.0f;
	A(4, 4) = 1.0f;

	A(5, 2) = 1.0f;
	A(5, 5) = 1.0f;

	Vector6f wrench{};
	wrench(0) = moment_cmd(0);
	wrench(1) = moment_cmd(1);
	wrench(2) = moment_cmd(2);
	wrench(3) = force_cmd(0);
	wrench(4) = force_cmd(1);
	wrench(5) = force_cmd(2);

	matrix::SquareMatrix<float, 6> H = A * A.T();

	for (int i = 0; i < 6; ++i) {
		H(i, i) += kHummingBirdVirtualLambda * kHummingBirdVirtualLambda;
	}

	const Vector8f virtual_force = A.T() * H.I() * wrench;

	matrix::Vector3f front_force{};
	front_force(0) = virtual_force(0);
	front_force(1) = virtual_force(1);
	front_force(2) = virtual_force(2);

	matrix::Vector3f back_force{};
	back_force(0) = virtual_force(3);
	back_force(1) = virtual_force(4);
	back_force(2) = virtual_force(5);

	const float front_force_norm = front_force.norm();
	const float back_force_norm = back_force.norm();
	const float f_front = math::max(front_force_norm, kHummingBirdFMin);
	const float f_back = math::max(back_force_norm, kHummingBirdFMin);

	matrix::Vector3f front_dir = front_force / f_front;
	matrix::Vector3f back_dir = back_force / f_back;

	if (front_force_norm <= kHummingBirdFMin) {
		front_dir(0) = 0.0f;
		front_dir(1) = 0.0f;
		front_dir(2) = -1.0f;
	}

	if (back_force_norm <= kHummingBirdFMin) {
		back_dir(0) = 0.0f;
		back_dir(1) = 0.0f;
		back_dir(2) = -1.0f;
	}

	const float theta1 = math::constrain(atan2f(-front_dir(0), -front_dir(2)), -kHummingBirdThetaLimitRad, kHummingBirdThetaLimitRad);
	const float theta2 = math::constrain(atan2f(-back_dir(0), -back_dir(2)), -kHummingBirdThetaLimitRad, kHummingBirdThetaLimitRad);
	const float phi14_cmd = math::constrain(asinf(math::constrain(front_dir(1), -1.0f, 1.0f)), -kHummingBirdPhiLimitRad, kHummingBirdPhiLimitRad);
	const float phi23_cmd = math::constrain(asinf(math::constrain(back_dir(1), -1.0f, 1.0f)), -kHummingBirdPhiLimitRad, kHummingBirdPhiLimitRad);

	const float df14 = math::constrain(virtual_force(6), -f_front + kHummingBirdFMin, f_front - kHummingBirdFMin);
	const float df23 = math::constrain(virtual_force(7), -f_back + kHummingBirdFMin, f_back - kHummingBirdFMin);

	HummingBirdAllocationOutput out{};
	out.f(0) = math::constrain(0.5f * (f_front + df14), 0.0f, kHummingBirdMaxRotorThrust);
	out.f(3) = math::constrain(0.5f * (f_front - df14), 0.0f, kHummingBirdMaxRotorThrust);
	out.f(1) = math::constrain(0.5f * (f_back + df23), 0.0f, kHummingBirdMaxRotorThrust);
	out.f(2) = math::constrain(0.5f * (f_back - df23), 0.0f, kHummingBirdMaxRotorThrust);

	out.theta(0) = theta1;
	out.theta(1) = theta2;
	out.theta(2) = theta2;
	out.theta(3) = theta1;

	out.phi(0) = phi14_cmd;
	out.phi(1) = phi23_cmd;
	out.phi(2) = phi23_cmd;
	out.phi(3) = phi14_cmd;

	return out;
}
}


ControlAllocator::ControlAllocator() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	_control_allocator_status_pub[0].advertise();
	_control_allocator_status_pub[1].advertise();

	_actuator_motors_pub.advertise();
	_actuator_servos_pub.advertise();
	_actuator_servos_trim_pub.advertise();

	for (int i = 0; i < MAX_NUM_MOTORS; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "CA_R%u_SLEW", i);
		_param_handles.slew_rate_motors[i] = param_find(buffer);
	}

	for (int i = 0; i < MAX_NUM_SERVOS; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "CA_SV%u_SLEW", i);
		_param_handles.slew_rate_servos[i] = param_find(buffer);
	}

	parameters_updated();
}

ControlAllocator::~ControlAllocator()
{
	for (int i = 0; i < ActuatorEffectiveness::MAX_NUM_MATRICES; ++i) {
		delete _control_allocation[i];
	}

	delete _actuator_effectiveness;

	perf_free(_loop_perf);
}

bool
ControlAllocator::init()
{
	if (!_vehicle_torque_setpoint_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

#ifndef ENABLE_LOCKSTEP_SCHEDULER // Backup schedule would interfere with lockstep
	ScheduleDelayed(50_ms);
#endif

	return true;
}

void
ControlAllocator::parameters_updated()
{
	_has_slew_rate = false;

	for (int i = 0; i < MAX_NUM_MOTORS; ++i) {
		param_get(_param_handles.slew_rate_motors[i], &_params.slew_rate_motors[i]);
		_has_slew_rate |= _params.slew_rate_motors[i] > FLT_EPSILON;
	}

	for (int i = 0; i < MAX_NUM_SERVOS; ++i) {
		param_get(_param_handles.slew_rate_servos[i], &_params.slew_rate_servos[i]);
		_has_slew_rate |= _params.slew_rate_servos[i] > FLT_EPSILON;
	}

	// Allocation method & effectiveness source
	// Do this first: in case a new method is loaded, it will be configured below
	bool updated = update_effectiveness_source();
	update_allocation_method(updated); // must be called after update_effectiveness_source()

	if (_num_control_allocation == 0) {
		return;
	}

	for (int i = 0; i < _num_control_allocation; ++i) {
		_control_allocation[i]->updateParameters();
	}

	update_effectiveness_matrix_if_needed(EffectivenessUpdateReason::CONFIGURATION_UPDATE);
}

void
ControlAllocator::update_allocation_method(bool force)
{
	AllocationMethod configured_method = (AllocationMethod)_param_ca_method.get();

	if (!_actuator_effectiveness) {
		PX4_ERR("_actuator_effectiveness null");
		return;
	}

	if (_allocation_method_id != configured_method || force) {

		ActuatorVector actuator_sp[ActuatorEffectiveness::MAX_NUM_MATRICES];

		// Cleanup first
		for (int i = 0; i < ActuatorEffectiveness::MAX_NUM_MATRICES; ++i) {
			// Save current state
			if (_control_allocation[i] != nullptr) {
				actuator_sp[i] = _control_allocation[i]->getActuatorSetpoint();
			}

			delete _control_allocation[i];
			_control_allocation[i] = nullptr;
		}

		_num_control_allocation = _actuator_effectiveness->numMatrices();

		AllocationMethod desired_methods[ActuatorEffectiveness::MAX_NUM_MATRICES];
		_actuator_effectiveness->getDesiredAllocationMethod(desired_methods);

		bool normalize_rpy[ActuatorEffectiveness::MAX_NUM_MATRICES];
		_actuator_effectiveness->getNormalizeRPY(normalize_rpy);

		for (int i = 0; i < _num_control_allocation; ++i) {
			AllocationMethod method = configured_method;

			if (configured_method == AllocationMethod::AUTO) {
				method = desired_methods[i];
			}

			switch (method) {
			case AllocationMethod::PSEUDO_INVERSE:
				_control_allocation[i] = new ControlAllocationPseudoInverse();
				break;

			case AllocationMethod::SEQUENTIAL_DESATURATION:
				_control_allocation[i] = new ControlAllocationSequentialDesaturation();
				break;

			default:
				PX4_ERR("Unknown allocation method");
				break;
			}

			if (_control_allocation[i] == nullptr) {
				PX4_ERR("alloc failed");
				_num_control_allocation = 0;

			} else {
				_control_allocation[i]->setNormalizeRPY(normalize_rpy[i]);
				_control_allocation[i]->setActuatorSetpoint(actuator_sp[i]);
			}
		}

		_allocation_method_id = configured_method;
	}
}

bool
ControlAllocator::update_effectiveness_source()
{
	const EffectivenessSource source = (EffectivenessSource)_param_ca_airframe.get();

	if (_effectiveness_source_id != source) {

		// try to instanciate new effectiveness source
		ActuatorEffectiveness *tmp = nullptr;

		switch (source) {
		case EffectivenessSource::NONE:
		case EffectivenessSource::MULTIROTOR:
			tmp = new ActuatorEffectivenessMultirotor(this);
			break;

		case EffectivenessSource::STANDARD_VTOL:
			tmp = new ActuatorEffectivenessStandardVTOL(this);
			break;

		case EffectivenessSource::TILTROTOR_VTOL:
			tmp = new ActuatorEffectivenessTiltrotorVTOL(this);
			break;

		case EffectivenessSource::TAILSITTER_VTOL:
			tmp = new ActuatorEffectivenessTailsitterVTOL(this);
			break;

		case EffectivenessSource::FIXED_WING:
			tmp = new ActuatorEffectivenessFixedWing(this);
			break;

		case EffectivenessSource::MOTORS_6DOF: // just a different UI from MULTIROTOR
			tmp = new ActuatorEffectivenessUUV(this);
			break;

		case EffectivenessSource::MULTIROTOR_WITH_TILT:
			tmp = new ActuatorEffectivenessMCTilt(this);
			break;

		case EffectivenessSource::CUSTOM:
			tmp = new ActuatorEffectivenessCustom(this);
			break;

		case EffectivenessSource::HELICOPTER_TAIL_ESC:
			tmp = new ActuatorEffectivenessHelicopter(this, ActuatorType::MOTORS);
			break;

		case EffectivenessSource::HELICOPTER_TAIL_SERVO:
			tmp = new ActuatorEffectivenessHelicopter(this, ActuatorType::SERVOS);
			break;

		case EffectivenessSource::HELICOPTER_COAXIAL:
			tmp = new ActuatorEffectivenessHelicopterCoaxial(this);
			break;

		case EffectivenessSource::SPACECRAFT_2D:
			tmp = new ActuatorEffectivenessSpacecraft(this);
			break;

		case EffectivenessSource::HUMMINGBIRD:
			tmp = new ActuatorEffectivenessHummingBird(this);
			break;

		case EffectivenessSource::ROVER_ACKERMANN: // Unreachable: Rover startup scripts don't load control_allocator. Controllers publish actuator_outputs directly.
		case EffectivenessSource::ROVER_DIFFERENTIAL:
		case EffectivenessSource::ROVER_MECANUM:
		default:
			PX4_ERR("Unknown airframe");
			break;
		}

		// Replace previous source with new one
		if (tmp == nullptr) {
			// It did not work, forget about it
			PX4_ERR("Actuator effectiveness init failed");
			_param_ca_airframe.set((int)_effectiveness_source_id);

		} else {
			// Swap effectiveness sources
			delete _actuator_effectiveness;
			_actuator_effectiveness = tmp;

			// Save source id
			_effectiveness_source_id = source;
		}

		return true;
	}

	return false;
}

void
ControlAllocator::Run()
{
	if (should_exit()) {
		_vehicle_torque_setpoint_sub.unregisterCallback();
		exit_and_cleanup(desc);
		return;
	}

	perf_begin(_loop_perf);

#ifndef ENABLE_LOCKSTEP_SCHEDULER // Backup schedule would interfere with lockstep
	// Push backup schedule
	ScheduleDelayed(50_ms);
#endif

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		if (_handled_motor_failure_bitmask == 0) {
			// We don't update the geometry after an actuator failure, as it could lead to unexpected results
			// (e.g. a user could add/remove motors, such that the bitmask isn't correct anymore)
			updateParams();
			parameters_updated();
		}
	}

	if (_num_control_allocation == 0 || _actuator_effectiveness == nullptr) {
		return;
	}

	{
		vehicle_status_s vehicle_status;

		if (_vehicle_status_sub.update(&vehicle_status)) {

			_armed = vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
			_is_vtol = vehicle_status.is_vtol;

			ActuatorEffectiveness::FlightPhase flight_phase{ActuatorEffectiveness::FlightPhase::HOVER_FLIGHT};

			// Check if the current flight phase is HOVER or FIXED_WING
			if (vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
				flight_phase = ActuatorEffectiveness::FlightPhase::HOVER_FLIGHT;

			} else {
				flight_phase = ActuatorEffectiveness::FlightPhase::FORWARD_FLIGHT;
			}

			// Special cases for VTOL in transition
			if (vehicle_status.is_vtol && vehicle_status.in_transition_mode) {
				if (vehicle_status.in_transition_to_fw) {
					flight_phase = ActuatorEffectiveness::FlightPhase::TRANSITION_HF_TO_FF;

				} else {
					flight_phase = ActuatorEffectiveness::FlightPhase::TRANSITION_FF_TO_HF;
				}
			}

			// Forward to effectiveness source
			_actuator_effectiveness->setFlightPhase(flight_phase);
		}
	}

	{
		vehicle_control_mode_s vehicle_control_mode;

		if (_vehicle_control_mode_sub.update(&vehicle_control_mode)) {
			_publish_controls = vehicle_control_mode.flag_control_allocation_enabled;
		}
	}

	// Guard against too small (< 0.2ms) and too large (> 20ms) dt's.
	const hrt_abstime now = hrt_absolute_time();
	const float dt = math::constrain(((now - _last_run) / 1e6f), 0.0002f, 0.02f);

	_actuator_group_preflight_check.handleCommand(now, _effectiveness_source_id == EffectivenessSource::TILTROTOR_VTOL);
	_actuator_group_preflight_check.updateState(now);

	bool do_update = false;
	vehicle_torque_setpoint_s vehicle_torque_setpoint;
	vehicle_thrust_setpoint_s vehicle_thrust_setpoint;

	// Run allocator on torque changes
	if (_vehicle_torque_setpoint_sub.update(&vehicle_torque_setpoint)) {
		_torque_sp = matrix::Vector3f(vehicle_torque_setpoint.xyz);

		do_update = true;
		_timestamp_sample = vehicle_torque_setpoint.timestamp_sample;

	}

	if (_vehicle_thrust_setpoint_sub.update(&vehicle_thrust_setpoint)) {
		_thrust_sp = matrix::Vector3f(vehicle_thrust_setpoint.xyz);
	}

	if (do_update) {
		_last_run = now;

		check_for_motor_failures();

		update_effectiveness_matrix_if_needed(EffectivenessUpdateReason::NO_EXTERNAL_UPDATE);

		// Set control setpoint vector(s)
		matrix::Vector<float, NUM_AXES> c[ActuatorEffectiveness::MAX_NUM_MATRICES];
		c[0](0) = _torque_sp(0);
		c[0](1) = _torque_sp(1);
		c[0](2) = _torque_sp(2);
		c[0](3) = _thrust_sp(0);
		c[0](4) = _thrust_sp(1);
		c[0](5) = _thrust_sp(2);

		if (_num_control_allocation > 1) {
			if (_vehicle_torque_setpoint1_sub.copy(&vehicle_torque_setpoint)) {
				c[1](0) = vehicle_torque_setpoint.xyz[0];
				c[1](1) = vehicle_torque_setpoint.xyz[1];
				c[1](2) = vehicle_torque_setpoint.xyz[2];
			}

			if (_vehicle_thrust_setpoint1_sub.copy(&vehicle_thrust_setpoint)) {
				c[1](3) = vehicle_thrust_setpoint.xyz[0];
				c[1](4) = vehicle_thrust_setpoint.xyz[1];
				c[1](5) = vehicle_thrust_setpoint.xyz[2];
			}
		}

		_actuator_group_preflight_check.applyOverrides(c, _is_vtol, *_actuator_effectiveness);

		for (int i = 0; i < _num_control_allocation; ++i) {

			_control_allocation[i]->setControlSetpoint(c[i]);

			// Do allocation
			_control_allocation[i]->allocate();
			_actuator_effectiveness->allocateAuxilaryControls(dt, i, _control_allocation[i]->_actuator_sp); //flaps and spoilers
			_actuator_effectiveness->updateSetpoint(c[i], i, _control_allocation[i]->_actuator_sp,
								_control_allocation[i]->getActuatorMin(), _control_allocation[i]->getActuatorMax());

			if (i == 0) {
				// The motors are always in allocation 0
				handle_stopped_motors(now);
			}

			if (_has_slew_rate) {
				_control_allocation[i]->applySlewRateLimit(dt);
			}

			_control_allocation[i]->clipActuatorSetpoint();
		}
	}

	// Publish actuator setpoint and allocator status
	publish_actuator_controls();

	// Publish status at limited rate, as it's somewhat expensive and we use it for slower dynamics
	// (i.e. anti-integrator windup)
	if (now - _last_status_pub >= 5_ms) {
		publish_control_allocator_status(0);

		if (_num_control_allocation > 1) {
			publish_control_allocator_status(1);
		}

		_last_status_pub = now;
	}

	perf_end(_loop_perf);
}

void
ControlAllocator::update_effectiveness_matrix_if_needed(EffectivenessUpdateReason reason)
{
	ActuatorEffectiveness::Configuration config{};

	if (reason == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE
	    && hrt_elapsed_time(&_last_effectiveness_update) < 100_ms) { // rate-limit updates
		return;
	}

	if (_actuator_effectiveness->getEffectivenessMatrix(config, reason)) {
		_last_effectiveness_update = hrt_absolute_time();

		memcpy(_control_allocation_selection_indexes, config.matrix_selection_indexes,
		       sizeof(_control_allocation_selection_indexes));

		// Get the minimum and maximum depending on type and configuration
		ActuatorEffectiveness::ActuatorVector minimum[ActuatorEffectiveness::MAX_NUM_MATRICES];
		ActuatorEffectiveness::ActuatorVector maximum[ActuatorEffectiveness::MAX_NUM_MATRICES];
		ActuatorEffectiveness::ActuatorVector slew_rate[ActuatorEffectiveness::MAX_NUM_MATRICES];
		int actuator_idx = 0;
		int actuator_idx_matrix[ActuatorEffectiveness::MAX_NUM_MATRICES] {};

		actuator_servos_trim_s trims{};
		static_assert(actuator_servos_trim_s::NUM_CONTROLS == actuator_servos_s::NUM_CONTROLS, "size mismatch");

		for (int actuator_type = 0; actuator_type < (int)ActuatorType::COUNT; ++actuator_type) {
			_num_actuators[actuator_type] = config.num_actuators[actuator_type];

			for (int actuator_type_idx = 0; actuator_type_idx < config.num_actuators[actuator_type]; ++actuator_type_idx) {
				if (actuator_idx >= NUM_ACTUATORS) {
					_num_actuators[actuator_type] = 0;
					PX4_ERR("Too many actuators");
					break;
				}

				int selected_matrix = _control_allocation_selection_indexes[actuator_idx];

				if ((ActuatorType)actuator_type == ActuatorType::MOTORS) {
					if (actuator_type_idx >= MAX_NUM_MOTORS) {
						PX4_ERR("Too many motors");
						_num_actuators[actuator_type] = 0;
						break;
					}

					if (_param_r_rev.get() & (1u << actuator_type_idx)) {
						minimum[selected_matrix](actuator_idx_matrix[selected_matrix]) = -1.f;

					} else {
						minimum[selected_matrix](actuator_idx_matrix[selected_matrix]) = 0.f;
					}

					slew_rate[selected_matrix](actuator_idx_matrix[selected_matrix]) = _params.slew_rate_motors[actuator_type_idx];

				} else if ((ActuatorType)actuator_type == ActuatorType::SERVOS) {
					if (actuator_type_idx >= MAX_NUM_SERVOS) {
						PX4_ERR("Too many servos");
						_num_actuators[actuator_type] = 0;
						break;
					}

					minimum[selected_matrix](actuator_idx_matrix[selected_matrix]) = -1.f;
					slew_rate[selected_matrix](actuator_idx_matrix[selected_matrix]) = _params.slew_rate_servos[actuator_type_idx];
					trims.trim[actuator_type_idx] = config.trim[selected_matrix](actuator_idx_matrix[selected_matrix]);

				} else {
					minimum[selected_matrix](actuator_idx_matrix[selected_matrix]) = -1.f;
				}

				maximum[selected_matrix](actuator_idx_matrix[selected_matrix]) = 1.f;

				++actuator_idx_matrix[selected_matrix];
				++actuator_idx;
			}
		}

		// Handle failed actuators
		if (_handled_motor_failure_bitmask) {
			actuator_idx = 0;
			memset(&actuator_idx_matrix, 0, sizeof(actuator_idx_matrix));

			for (int motors_idx = 0; motors_idx < _num_actuators[0] && motors_idx < actuator_motors_s::NUM_CONTROLS; motors_idx++) {
				int selected_matrix = _control_allocation_selection_indexes[actuator_idx];

				if (_handled_motor_failure_bitmask & (1 << motors_idx)) {
					ActuatorEffectiveness::EffectivenessMatrix &matrix = config.effectiveness_matrices[selected_matrix];

					for (int i = 0; i < NUM_AXES; i++) {
						matrix(i, actuator_idx_matrix[selected_matrix]) = 0.0f;
					}
				}

				++actuator_idx_matrix[selected_matrix];
				++actuator_idx;
			}
		}

		for (int i = 0; i < _num_control_allocation; ++i) {
			_control_allocation[i]->setActuatorMin(minimum[i]);
			_control_allocation[i]->setActuatorMax(maximum[i]);
			_control_allocation[i]->setSlewRateLimit(slew_rate[i]);

			// Set all the elements of a row to 0 if that row has weak authority.
			// That ensures that the algorithm doesn't try to control axes with only marginal control authority,
			// which in turn would degrade the control of the main axes that actually should and can be controlled.

			ActuatorEffectiveness::EffectivenessMatrix &matrix = config.effectiveness_matrices[i];

			for (int n = 0; n < NUM_AXES; n++) {
				bool all_entries_small = true;

				for (int m = 0; m < config.num_actuators_matrix[i]; m++) {
					if (fabsf(matrix(n, m)) > 0.05f) {
						all_entries_small = false;
					}
				}

				if (all_entries_small) {
					matrix.row(n) = 0.f;
				}
			}

			// Assign control effectiveness matrix
			int total_num_actuators = config.num_actuators_matrix[i];
			_control_allocation[i]->setEffectivenessMatrix(config.effectiveness_matrices[i], config.trim[i],
					config.linearization_point[i], total_num_actuators, reason == EffectivenessUpdateReason::CONFIGURATION_UPDATE);
		}

		trims.timestamp = hrt_absolute_time();
		_actuator_servos_trim_pub.publish(trims);
	}
}


void
ControlAllocator::handle_stopped_motors(const hrt_abstime now)
{
	const ActuatorBitmask stopped_motors_due_to_effectiveness = _actuator_effectiveness->getStoppedMotors();

	const ActuatorBitmask stopped_motors = stopped_motors_due_to_effectiveness
					       | _handled_motor_failure_bitmask
					       | _motor_stop_mask;

	// Handle stopped motors by setting NaN
	const unsigned int allocation_index = 0;  // Motors always in allocation 0
	_control_allocation[allocation_index]->applyNanToActuators(stopped_motors);

	// Apply ice shedding, which applies _only_ to stopped motors
	const bool any_stopped_motor_failed = 0 != (stopped_motors_due_to_effectiveness & (_handled_motor_failure_bitmask | _motor_stop_mask));
	const float ice_shedding_output = get_ice_shedding_output(now);

	if (ice_shedding_output > FLT_EPSILON && !any_stopped_motor_failed) {
		for (int motors_idx = 0; motors_idx < _num_actuators[allocation_index] && motors_idx < actuator_motors_s::NUM_CONTROLS; motors_idx++) {
			if (stopped_motors & 1u << motors_idx) {
				_control_allocation[allocation_index]->_actuator_sp(motors_idx) = ice_shedding_output;
			}
		}
	}
}

void
ControlAllocator::publish_control_allocator_status(int matrix_index)
{
	control_allocator_status_s control_allocator_status{};
	control_allocator_status.timestamp = hrt_absolute_time();
	control_allocator_status.actuator_group_preflight_check_active = _actuator_group_preflight_check.isActive();

	// TODO: disabled motors (?)

	// Allocated control
	const matrix::Vector<float, NUM_AXES> &allocated_control = _control_allocation[matrix_index]->getAllocatedControl();

	// Unallocated control
	const matrix::Vector<float, NUM_AXES> unallocated_control = _control_allocation[matrix_index]->getControlSetpoint() -
			allocated_control;
	control_allocator_status.unallocated_torque[0] = unallocated_control(0);
	control_allocator_status.unallocated_torque[1] = unallocated_control(1);
	control_allocator_status.unallocated_torque[2] = unallocated_control(2);
	control_allocator_status.unallocated_thrust[0] = unallocated_control(3);
	control_allocator_status.unallocated_thrust[1] = unallocated_control(4);
	control_allocator_status.unallocated_thrust[2] = unallocated_control(5);

	// override control_allocator_status in customized saturation logic for certain effectiveness types
	_actuator_effectiveness->getUnallocatedControl(matrix_index, control_allocator_status);

	// Allocation success flags
	control_allocator_status.torque_setpoint_achieved = (Vector3f(control_allocator_status.unallocated_torque[0],
			control_allocator_status.unallocated_torque[1],
			control_allocator_status.unallocated_torque[2]).norm_squared() < 1e-6f);
	control_allocator_status.thrust_setpoint_achieved = (Vector3f(control_allocator_status.unallocated_thrust[0],
			control_allocator_status.unallocated_thrust[1],
			control_allocator_status.unallocated_thrust[2]).norm_squared() < 1e-6f);

	// Actuator saturation
	const ActuatorVector &actuator_sp = _control_allocation[matrix_index]->getActuatorSetpoint();
	const ActuatorVector &actuator_min = _control_allocation[matrix_index]->getActuatorMin();
	const ActuatorVector &actuator_max = _control_allocation[matrix_index]->getActuatorMax();

	for (int i = 0; i < NUM_ACTUATORS; i++) {
		if (actuator_sp(i) > (actuator_max(i) - FLT_EPSILON)) {
			control_allocator_status.actuator_saturation[i] = control_allocator_status_s::ACTUATOR_SATURATION_UPPER;

		} else if (actuator_sp(i) < (actuator_min(i) + FLT_EPSILON)) {
			control_allocator_status.actuator_saturation[i] = control_allocator_status_s::ACTUATOR_SATURATION_LOWER;
		}
	}

	// Handled motor failures
	control_allocator_status.handled_motor_failure_mask = _handled_motor_failure_bitmask;
	control_allocator_status.motor_stop_mask = _motor_stop_mask;

	_control_allocator_status_pub[matrix_index].publish(control_allocator_status);
}

float
ControlAllocator::get_ice_shedding_output(hrt_abstime now)
{
	const float period_sec = _param_ice_shedding_period.get();

	const bool feature_disabled_by_param = period_sec <= FLT_EPSILON;
	const bool in_forward_flight = _actuator_effectiveness->getFlightPhase() == ActuatorEffectiveness::FlightPhase::FORWARD_FLIGHT;

	// If any stopped motor has failed, the feature will create much more
	// torque than in the nominal case, and becomes pointless anyway as we
	// cannot go back to multicopter
	const bool apply_shedding = _is_vtol && in_forward_flight;

	if (feature_disabled_by_param || !apply_shedding) {
		return 0.0f;

	} else {
		// Square wave output
		const float elapsed_in_period = fmodf(static_cast<float>(now) / 1_s, period_sec);
		const float ice_shedding_output = elapsed_in_period < ICE_SHEDDING_ON_SEC ? ICE_SHEDDING_OUTPUT : 0.0f;

		return ice_shedding_output;
	}
}

void
ControlAllocator::publish_hummingbird_actuator_controls()
{
	actuator_motors_s actuator_motors{};
	actuator_motors.timestamp = hrt_absolute_time();
	actuator_motors.timestamp_sample = _timestamp_sample;
	actuator_motors.reversible_flags = _param_r_rev.get();

	actuator_servos_s actuator_servos{};
	actuator_servos.timestamp = actuator_motors.timestamp;
	actuator_servos.timestamp_sample = _timestamp_sample;

	matrix::Vector3f moment_cmd = _torque_sp * kHummingBirdMaxMoment;
	matrix::Vector3f force_cmd = _thrust_sp * kHummingBirdMaxTotalThrust;

	const HummingBirdAllocationOutput allocation = allocation_P2T2(moment_cmd, force_cmd,
			_hummingbird_theta_sp, _hummingbird_phi_sp);

	_hummingbird_motor_sp = allocation.f;
	_hummingbird_theta_sp = allocation.theta;
	_hummingbird_phi_sp = allocation.phi;

	for (int i = 0; i < actuator_motors_s::NUM_CONTROLS; ++i) {
		actuator_motors.control[i] = NAN;
	}

	for (int i = 0; i < 4; ++i) {
		if (PX4_ISFINITE(_hummingbird_motor_sp(i))) {
			actuator_motors.control[i] = sqrtf(math::constrain(_hummingbird_motor_sp(i) / kHummingBirdMaxRotorThrust, 0.0f, 1.0f));

		} else {
			actuator_motors.control[i] = NAN;
		}
	}

	for (int i = 0; i < actuator_servos_s::NUM_CONTROLS; ++i) {
		actuator_servos.control[i] = 0.0f;
	}

	for (int i = 0; i < 4; ++i) {
		actuator_servos.control[i] = math::constrain(_hummingbird_theta_sp(i) / kHummingBirdThetaLimitRad, -1.0f, 1.0f);
		actuator_servos.control[i + 4] = math::constrain(_hummingbird_phi_sp(i) / kHummingBirdPhiLimitRad, -1.0f, 1.0f);
	}

	_actuator_motors_pub.publish(actuator_motors);
	_actuator_servos_pub.publish(actuator_servos);
}

void
ControlAllocator::publish_actuator_controls()
{
	if (!_publish_controls) {
		return;
	}

	if (_effectiveness_source_id == EffectivenessSource::HUMMINGBIRD && _param_hb_ctrl_mode.get() == 1) {
		publish_hummingbird_actuator_controls();
		return;
	}

	actuator_motors_s actuator_motors;
	actuator_motors.timestamp = hrt_absolute_time();
	actuator_motors.timestamp_sample = _timestamp_sample;

	actuator_servos_s actuator_servos;
	actuator_servos.timestamp = actuator_motors.timestamp;
	actuator_servos.timestamp_sample = _timestamp_sample;

	actuator_motors.reversible_flags = _param_r_rev.get();

	int actuator_idx = 0;
	int actuator_idx_matrix[ActuatorEffectiveness::MAX_NUM_MATRICES] {};

	// motors
	int motors_idx;

	for (motors_idx = 0; motors_idx < _num_actuators[0] && motors_idx < actuator_motors_s::NUM_CONTROLS; motors_idx++) {
		int selected_matrix = _control_allocation_selection_indexes[actuator_idx];
		float actuator_sp = _control_allocation[selected_matrix]->getActuatorSetpoint()(actuator_idx_matrix[selected_matrix]);
		actuator_motors.control[motors_idx] = PX4_ISFINITE(actuator_sp) ? actuator_sp : NAN;
		++actuator_idx_matrix[selected_matrix];
		++actuator_idx;
	}

	for (int i = motors_idx; i < actuator_motors_s::NUM_CONTROLS; i++) {
		actuator_motors.control[i] = NAN;
	}

	_actuator_motors_pub.publish(actuator_motors);

	// servos
	if (_num_actuators[1] > 0) {
		int servos_idx;

		for (servos_idx = 0; servos_idx < _num_actuators[1] && servos_idx < actuator_servos_s::NUM_CONTROLS; servos_idx++) {
			int selected_matrix = _control_allocation_selection_indexes[actuator_idx];
			float actuator_sp = _control_allocation[selected_matrix]->getActuatorSetpoint()(actuator_idx_matrix[selected_matrix]);
			actuator_servos.control[servos_idx] = PX4_ISFINITE(actuator_sp) ? actuator_sp : 0.f;
			++actuator_idx_matrix[selected_matrix];
			++actuator_idx;
		}

		for (int i = servos_idx; i < actuator_servos_s::NUM_CONTROLS; i++) {
			actuator_servos.control[i] = 0.f;
		}

	} else {
		for (int i = 0; i < actuator_servos_s::NUM_CONTROLS; i++) {
			actuator_servos.control[i] = 0.f;
		}
	}

	_actuator_servos_pub.publish(actuator_servos);
}

void
ControlAllocator::check_for_motor_failures()
{
	failure_detector_status_s failure_detector_status;

	if ((FailureMode)_param_ca_failure_mode.get() > FailureMode::IGNORE
	    && _failure_detector_status_sub.update(&failure_detector_status)) {

		if (_motor_stop_mask != failure_detector_status.motor_stop_mask) {
			_motor_stop_mask = failure_detector_status.motor_stop_mask;
			PX4_WARN("Stopping motors (%d)", _motor_stop_mask);
		}

		if (failure_detector_status.fd_motor) {
			if (_handled_motor_failure_bitmask != failure_detector_status.motor_failure_mask) {
				// motor failure bitmask changed
				switch ((FailureMode)_param_ca_failure_mode.get()) {
				case FailureMode::REMOVE_FIRST_FAILING_MOTOR: {
						// Count number of failed motors
						const int num_motors_failed = math::countSetBits(failure_detector_status.motor_failure_mask);

						// Only handle if it is the first failure
						if (_handled_motor_failure_bitmask == 0 && num_motors_failed == 1) {
							_handled_motor_failure_bitmask = failure_detector_status.motor_failure_mask;
							PX4_WARN("Removing motor from allocation (0x%x)", _handled_motor_failure_bitmask);

							for (int i = 0; i < _num_control_allocation; ++i) {
								_control_allocation[i]->setHadActuatorFailure(true);
							}

							update_effectiveness_matrix_if_needed(EffectivenessUpdateReason::MOTOR_ACTIVATION_UPDATE);
						}
					}
					break;

				default:
					break;
				}

			}

		} else if (_handled_motor_failure_bitmask != 0) {
			// Clear bitmask completely
			PX4_INFO("Restoring all motors");
			_handled_motor_failure_bitmask = 0;

			for (int i = 0; i < _num_control_allocation; ++i) {
				_control_allocation[i]->setHadActuatorFailure(false);
			}

			update_effectiveness_matrix_if_needed(EffectivenessUpdateReason::MOTOR_ACTIVATION_UPDATE);
		}
	}
}

int ControlAllocator::task_spawn(int argc, char *argv[])
{
	ControlAllocator *instance = new ControlAllocator();

	if (instance) {
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;

	return PX4_ERROR;
}

int ControlAllocator::print_status()
{
	PX4_INFO("Running");

	// Print current allocation method
	switch (_allocation_method_id) {
	case AllocationMethod::NONE:
		PX4_INFO("Method: None");
		break;

	case AllocationMethod::PSEUDO_INVERSE:
		PX4_INFO("Method: Pseudo-inverse");
		break;

	case AllocationMethod::SEQUENTIAL_DESATURATION:
		PX4_INFO("Method: Sequential desaturation");
		break;

	case AllocationMethod::AUTO:
		PX4_INFO("Method: Auto");
		break;
	}

	// Print current airframe
	if (_actuator_effectiveness != nullptr) {
		PX4_INFO("Effectiveness Source: %s", _actuator_effectiveness->name());
	}

	// Print current effectiveness matrix
	for (int i = 0; i < _num_control_allocation; ++i) {
		const ActuatorEffectiveness::EffectivenessMatrix &effectiveness = _control_allocation[i]->getEffectivenessMatrix();

		if (_num_control_allocation > 1) {
			PX4_INFO("Instance: %i", i);
		}

		PX4_INFO("  Effectiveness =");
		int num_configured = _control_allocation[i]->numConfiguredActuators();

		// print column numbering
		if (num_configured > 1) {
			printf("  ");

			for (int col = 0; col < num_configured; col++) {
				printf("|%2u      ", col);
			}

			printf("\n");
		}

		// Print effectiveness matrix with row labels
		const char *row_labels[] = {"Mx", "My", "Mz", "Fx", "Fy", "Fz"};

		for (int row = 0; row < 6; row++) {
			printf("%2s|", row_labels[row]);

			for (int col = 0; col < num_configured; col++) {
				double d = static_cast<double>(effectiveness(row, col));

				// avoid -0.0 for display
				if (fabs(d - 0.0) < 1e-9) {
					// print fixed width zero
					printf(" 0       ");

				} else if ((fabs(d) < 1e-4) || (fabs(d) >= 10.0)) {
					printf("% .1e ", d);

				} else {
					printf("% 6.5f ", d);
				}
			}

			printf("\n");
		}

		PX4_INFO("  minimum =");

		// print column numbering
		if (num_configured > 1) {
			printf("  ");

			for (int col = 0; col < num_configured; col++) {
				printf("|%2u      ", col);
			}

			printf("\n");
		}

		printf("  |");

		for (int col = 0; col < num_configured; col++) {
			double d = static_cast<double>(_control_allocation[i]->getActuatorMin()(col));

			// avoid -0.0 for display
			if (fabs(d - 0.0) < 1e-9) {
				// print fixed width zero
				printf(" 0       ");

			} else if ((fabs(d) < 1e-4) || (fabs(d) >= 10.0)) {
				printf("% .1e ", d);

			} else {
				printf("% 6.5f ", d);
			}
		}

		printf("\n");
		PX4_INFO("  maximum =");

		// print column numbering
		if (num_configured > 1) {
			printf("  ");

			for (int col = 0; col < num_configured; col++) {
				printf("|%2u      ", col);
			}

			printf("\n");
		}

		printf("  |");

		for (int col = 0; col < num_configured; col++) {
			double d = static_cast<double>(_control_allocation[i]->getActuatorMax()(col));

			// avoid -0.0 for display
			if (fabs(d - 0.0) < 1e-9) {
				// print fixed width zero
				printf(" 0       ");

			} else if ((fabs(d) < 1e-4) || (fabs(d) >= 10.0)) {
				printf("% .1e ", d);

			} else {
				printf("% 6.5f ", d);
			}
		}

		printf("\n");
		PX4_INFO("  Configured actuators: %i", num_configured);
	}

	if (_handled_motor_failure_bitmask) {
		PX4_INFO("Failed motors: %i (0x%x)", math::countSetBits(_handled_motor_failure_bitmask),
			 _handled_motor_failure_bitmask);
	}

	// Print perf
	perf_print_counter(_loop_perf);

	return 0;
}

int ControlAllocator::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int ControlAllocator::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements control allocation. It takes torque and thrust setpoints
as inputs and outputs actuator setpoint messages.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("control_allocator", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

/**
 * Control Allocator app start / stop handling function
 */
extern "C" __EXPORT int control_allocator_main(int argc, char *argv[]);

int control_allocator_main(int argc, char *argv[])
{
	return ModuleBase::main(ControlAllocator::desc, argc, argv);
}

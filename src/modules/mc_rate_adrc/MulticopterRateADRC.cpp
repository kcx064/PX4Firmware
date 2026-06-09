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

#include "MulticopterRateADRC.hpp"

#include <drivers/drv_hrt.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>
#include <px4_platform_common/events.h>

using namespace matrix;
using namespace time_literals;
using math::radians;

MulticopterRateADRC::MulticopterRateADRC(bool vtol) :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_vehicle_torque_setpoint_pub(vtol ? ORB_ID(vehicle_torque_setpoint_virtual_mc) : ORB_ID(vehicle_torque_setpoint)),
	_vehicle_thrust_setpoint_pub(vtol ? ORB_ID(vehicle_thrust_setpoint_virtual_mc) : ORB_ID(vehicle_thrust_setpoint)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	_vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;

	parameters_updated();
	_controller_status_pub.advertise();
}

MulticopterRateADRC::~MulticopterRateADRC()
{
	perf_free(_loop_perf);
}

bool
MulticopterRateADRC::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}
	// ladrc init
	adrc_step = _param_adrc_step.get();

	roll_bw_ctl = _param_roll_bw_ctl.get();
	roll_bw_obs = _param_roll_bw_obs.get();
	roll_gain_b = _param_roll_b.get();
	roll_sat_k = _param_roll_sat_k.get();
	roll_sat_tau = _param_roll_sat_tau.get();
	adrc_roll.init(roll_sat_tau, 2*roll_bw_obs, roll_bw_obs*roll_bw_obs, roll_bw_ctl, roll_gain_b, adrc_step, roll_sat_k, 0.0f, 3.0f, 0.0f);

	pitch_bw_ctl = _param_pitch_bw_ctl.get();
	pitch_bw_obs = _param_pitch_bw_obs.get();
	pitch_gain_b = _param_pitch_b.get();
	pitch_sat_k = _param_pitch_sat_k.get();
	pitch_sat_tau = _param_pitch_sat_tau.get();
	adrc_pitch.init(pitch_sat_tau, 2*pitch_bw_obs, pitch_bw_obs*pitch_bw_obs, pitch_bw_ctl, pitch_gain_b, adrc_step, pitch_sat_k, 0.0f, 3.0f, 0.0f);

	yaw_bw_ctl = _param_yaw_bw_ctl.get();
	yaw_bw_obs = _param_yaw_bw_obs.get();
	yaw_gain_b = _param_yaw_b.get();
	yaw_sat_k = _param_yaw_sat_k.get();
	yaw_sat_tau = _param_yaw_sat_tau.get();
	adrc_yaw.init(yaw_sat_tau, 2*yaw_bw_obs, yaw_bw_obs*yaw_bw_obs, yaw_bw_ctl, yaw_gain_b, adrc_step, yaw_sat_k, 0.0f, 3.0f, 0.0f);

	yaw_bw_ctl2 = _param_yaw_bw_ctl2.get();
	yaw_bw_obs2 = _param_yaw_bw_obs2.get();
	yaw_gain_b2 = _param_yaw_b2.get();
	yaw_sat_k2 = _param_yaw_sat_k2.get();
	yaw_sat_tau2 = _param_yaw_sat_tau2.get();
	adrc_yaw2.init(yaw_sat_tau2, 3*yaw_bw_obs2, 3*yaw_bw_obs2*yaw_bw_obs2, yaw_bw_obs2*yaw_bw_obs2*yaw_bw_obs2, yaw_bw_ctl2*yaw_bw_ctl2, 2.0f*yaw_bw_ctl2, yaw_gain_b2, adrc_step, yaw_sat_k2, 0, 3.0f, 0.0f);


	return true;
}

void
MulticopterRateADRC::parameters_updated()
{
	//update ladrc params
	adrc_step = _param_adrc_step.get();

	roll_bw_ctl = _param_roll_bw_ctl.get();
	roll_bw_obs = _param_roll_bw_obs.get();
	roll_gain_b = _param_roll_b.get();
	roll_sat_k = _param_roll_sat_k.get();
	roll_sat_tau = _param_roll_sat_tau.get();
	adrc_roll.param_update(roll_sat_tau, 2*roll_bw_obs, roll_bw_obs*roll_bw_obs, roll_bw_ctl, roll_gain_b, adrc_step, roll_sat_k);

	pitch_bw_ctl = _param_pitch_bw_ctl.get();
	pitch_bw_obs = _param_pitch_bw_obs.get();
	pitch_gain_b = _param_pitch_b.get();
	pitch_sat_k = _param_pitch_sat_k.get();
	pitch_sat_tau = _param_pitch_sat_tau.get();
	adrc_pitch.param_update(pitch_sat_tau, 2*pitch_bw_obs, pitch_bw_obs*pitch_bw_obs, pitch_bw_ctl, pitch_gain_b, adrc_step, pitch_sat_k);

	yaw_bw_ctl = _param_yaw_bw_ctl.get();
	yaw_bw_obs = _param_yaw_bw_obs.get();
	yaw_gain_b = _param_yaw_b.get();
	yaw_sat_k = _param_yaw_sat_k.get();
	yaw_sat_tau = _param_yaw_sat_tau.get();
	adrc_yaw.param_update(yaw_sat_tau, 2*yaw_bw_obs, yaw_bw_obs*yaw_bw_obs, yaw_bw_ctl, yaw_gain_b, adrc_step, yaw_sat_k);


	yaw_bw_ctl2 = _param_yaw_bw_ctl2.get();
	yaw_bw_obs2 = _param_yaw_bw_obs2.get();
	yaw_gain_b2 = _param_yaw_b2.get();
	yaw_sat_k2 = _param_yaw_sat_k2.get();
	yaw_sat_tau2 = _param_yaw_sat_tau2.get();
	adrc_yaw2.param_update(yaw_sat_tau2, 3*yaw_bw_obs2, 3*yaw_bw_obs2*yaw_bw_obs2, yaw_bw_obs2*yaw_bw_obs2*yaw_bw_obs2, yaw_bw_ctl2*yaw_bw_ctl2, 2.0f*yaw_bw_ctl2, yaw_gain_b2, adrc_step, yaw_sat_k2);

	// rate control parameters
	// The controller gain K is used to convert the parallel (P + I/s + sD) form
	// to the ideal (K * [1 + 1/sTi + sTd]) form
	const Vector3f rate_k = Vector3f(_param_mc_rollrate_k.get(), _param_mc_pitchrate_k.get(), _param_mc_yawrate_k.get());

	_rate_control.setPidGains(
		rate_k.emult(Vector3f(_param_mc_rollrate_p.get(), _param_mc_pitchrate_p.get(), _param_mc_yawrate_p.get())),
		rate_k.emult(Vector3f(_param_mc_rollrate_i.get(), _param_mc_pitchrate_i.get(), _param_mc_yawrate_i.get())),
		rate_k.emult(Vector3f(_param_mc_rollrate_d.get(), _param_mc_pitchrate_d.get(), _param_mc_yawrate_d.get())));

	_rate_control.setIntegratorLimit(
		Vector3f(_param_mc_rr_int_lim.get(), _param_mc_pr_int_lim.get(), _param_mc_yr_int_lim.get()));

	_rate_control.setFeedForwardGain(
		Vector3f(_param_mc_rollrate_ff.get(), _param_mc_pitchrate_ff.get(), _param_mc_yawrate_ff.get()));


	// manual rate control acro mode rate limits
	_acro_rate_max = Vector3f(radians(_param_mc_acro_r_max.get()), radians(_param_mc_acro_p_max.get()),
				  radians(_param_mc_acro_y_max.get()));
}

void
MulticopterRateADRC::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
		parameters_updated();
	}


	// vehicle_attitude_setpoint_s vehicle_attitude_setpoint;
	// if(_vehicle_attitude_setpoint_sub.update(&vehicle_attitude_setpoint))
	// {
	// 	// const Quatf q_d{vehicle_attitude_setpoint.q_d};
	// 	// const Eulerf euler_d(q_d);
	// 	// euler_yaw_d = euler_d.psi();
	// 	yaw_rate_sp = vehicle_attitude_setpoint.yaw_sp_move_rate;
	// 	euler_yaw_d += yaw_rate_sp
	// }

	// trajectory_setpoint_s traj_sp;
	// if(_vehicle_trajectory_setpoint_sub.update(&traj_sp))
	// {
	// 	euler_yaw_d = traj_sp.yaw;
	// }

	vehicle_attitude_s v_att;
	if(_vehicle_attitude_sub.update(&v_att))
	{
		const Quatf q{v_att.q};
		const Eulerf euler(q);
		euler_yaw = euler.psi();
	}

	/* run controller on gyro changes */


	if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {

		const hrt_abstime now = angular_velocity.timestamp_sample;

		// Guard against too small (< 0.125ms) and too large (> 20ms) dt's.
		const float dt = math::constrain(((now - _last_run) * 1e-6f), 0.000125f, 0.02f);
		_last_run = now;

		const Vector3f rates{angular_velocity.xyz};
		const Vector3f angular_accel{angular_velocity.xyz_derivative};

		/* check for updates in other topics */
		_vehicle_control_mode_sub.update(&_vehicle_control_mode);

		if (_vehicle_land_detected_sub.updated()) {
			vehicle_land_detected_s vehicle_land_detected;

			if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
				_landed = vehicle_land_detected.landed;
				_maybe_landed = vehicle_land_detected.maybe_landed;
			}
		}

		_vehicle_status_sub.update(&_vehicle_status);

		// use rates setpoint topic
		vehicle_rates_setpoint_s vehicle_rates_setpoint{};

		if (_vehicle_control_mode.flag_control_manual_enabled && !_vehicle_control_mode.flag_control_attitude_enabled) {
			// generate the rate setpoint from sticks
			manual_control_setpoint_s manual_control_setpoint;

			if (_manual_control_setpoint_sub.update(&manual_control_setpoint)) {
				// manual rates control - ACRO mode
				const Vector3f man_rate_sp{
					math::superexpo(manual_control_setpoint.roll, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
					math::superexpo(-manual_control_setpoint.pitch, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
					math::superexpo(manual_control_setpoint.yaw, _param_mc_acro_expo_y.get(), _param_mc_acro_supexpoy.get())};

				_rates_setpoint = man_rate_sp.emult(_acro_rate_max);
				_thrust_setpoint(2) = -(manual_control_setpoint.throttle + 1.f) * .5f;
				_thrust_setpoint(0) = _thrust_setpoint(1) = 0.f;

				// publish rate setpoint
				vehicle_rates_setpoint.roll = _rates_setpoint(0);
				vehicle_rates_setpoint.pitch = _rates_setpoint(1);
				vehicle_rates_setpoint.yaw = _rates_setpoint(2);
				_thrust_setpoint.copyTo(vehicle_rates_setpoint.thrust_body);
				vehicle_rates_setpoint.timestamp = hrt_absolute_time();

				_vehicle_rates_setpoint_pub.publish(vehicle_rates_setpoint);
			}

		} else if (_vehicle_rates_setpoint_sub.update(&vehicle_rates_setpoint)) {
			if (_vehicle_rates_setpoint_sub.copy(&vehicle_rates_setpoint)) {
				_rates_setpoint(0) = PX4_ISFINITE(vehicle_rates_setpoint.roll)  ? vehicle_rates_setpoint.roll  : rates(0);
				_rates_setpoint(1) = PX4_ISFINITE(vehicle_rates_setpoint.pitch) ? vehicle_rates_setpoint.pitch : rates(1);
				_rates_setpoint(2) = PX4_ISFINITE(vehicle_rates_setpoint.yaw)   ? vehicle_rates_setpoint.yaw   : rates(2);
				_thrust_setpoint = Vector3f(vehicle_rates_setpoint.thrust_body);
			}
		}

		// run the rate controller
		if (_vehicle_control_mode.flag_control_rates_enabled) {

			// reset integral if disarmed
			if (!_vehicle_control_mode.flag_armed || _vehicle_status.vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
				_rate_control.resetIntegral();

				//
				adrc_roll.reset(2*roll_bw_obs, roll_bw_obs*roll_bw_obs, roll_bw_ctl, roll_gain_b, adrc_step, roll_sat_k, 0.0f, 3.0f, 0.0f);
				adrc_pitch.reset(2*pitch_bw_obs, pitch_bw_obs*pitch_bw_obs, pitch_bw_ctl, pitch_gain_b, adrc_step, pitch_sat_k, 0.0f, 3.0f, 0.0f);
				adrc_yaw.reset(2*yaw_bw_obs, yaw_bw_obs*yaw_bw_obs, yaw_bw_ctl, yaw_gain_b, adrc_step, yaw_sat_k, 0.0f, 3.0f, 0.0f);
				adrc_yaw2.reset(euler_yaw, 0, 0, 0.0f, 3.0f, 0.0f);//二阶ADRC位置控制，重置的时候必须把当前真实角度作为初始值，而不是0作为初始值
			}

			// update saturation status from control allocation feedback
			control_allocator_status_s control_allocator_status;

			if (_control_allocator_status_sub.update(&control_allocator_status)) {
				Vector<bool, 3> saturation_positive;
				Vector<bool, 3> saturation_negative;

				if (!control_allocator_status.torque_setpoint_achieved) {
					for (size_t i = 0; i < 3; i++) {
						if (control_allocator_status.unallocated_torque[i] > FLT_EPSILON) {
							saturation_positive(i) = true;

						} else if (control_allocator_status.unallocated_torque[i] < -FLT_EPSILON) {
							saturation_negative(i) = true;
						}
					}
				}

				// TODO: send the unallocated value directly for better anti-windup
				_rate_control.setSaturationStatus(saturation_positive, saturation_negative);
			}

			// run rate controller
			// const Vector3f att_control = _rate_control.update(rates, _rates_setpoint, angular_accel, dt, _maybe_landed || _landed);
			adrc_control_roll = adrc_roll.calc(_rates_setpoint(0), rates(0), dt);
			adrc_control_pitch = adrc_pitch.calc(_rates_setpoint(1), rates(1), dt);
			adrc_control_yaw = adrc_yaw.calc(_rates_setpoint(2), rates(2), dt);

			// vehicle_attitude_setpoint_s vehicle_attitude_setpoint;
			// if(_vehicle_attitude_setpoint_sub.update(&vehicle_attitude_setpoint))
			// {
			// 	yaw_rate_sp = vehicle_attitude_setpoint.yaw_sp_move_rate;
			// 	euler_yaw_d += yaw_rate_sp * dt;
			// }

			// adrc_control_yaw2 = adrc_yaw2.calc(euler_yaw_d, euler_yaw, dt);

			// publish rate controller status
			rate_ctrl_status_s rate_ctrl_status{};
			_rate_control.getRateControlStatus(rate_ctrl_status);
			rate_ctrl_status.timestamp = hrt_absolute_time();
			_controller_status_pub.publish(rate_ctrl_status);

			// publish thrust and torque setpoints
			vehicle_thrust_setpoint_s vehicle_thrust_setpoint{};
			vehicle_torque_setpoint_s vehicle_torque_setpoint{};

			_thrust_setpoint.copyTo(vehicle_thrust_setpoint.xyz);
			// vehicle_torque_setpoint.xyz[0] = PX4_ISFINITE(att_control(0)) ? att_control(0) : 0.f;
			vehicle_torque_setpoint.xyz[0] = PX4_ISFINITE(adrc_control_roll) ? adrc_control_roll : 0.f; //ladrc

			// vehicle_torque_setpoint.xyz[1] = PX4_ISFINITE(att_control(1)) ? att_control(1) : 0.f;
			vehicle_torque_setpoint.xyz[1] = PX4_ISFINITE(adrc_control_pitch) ? adrc_control_pitch : 0.f; //ladrc

			// vehicle_torque_setpoint.xyz[2] = PX4_ISFINITE(att_control(2)) ? att_control(2) : 0.f;
			vehicle_torque_setpoint.xyz[2] = PX4_ISFINITE(adrc_control_yaw) ? adrc_control_yaw : 0.f; //ladrc
			// vehicle_torque_setpoint.xyz[2] = PX4_ISFINITE(adrc_control_yaw2) ? adrc_control_yaw2 : 0.f; //ladrc2

			// scale setpoints by battery status if enabled
			if (_param_mc_bat_scale_en.get()) {
				if (_battery_status_sub.updated()) {
					battery_status_s battery_status;

					if (_battery_status_sub.copy(&battery_status) && battery_status.connected && battery_status.scale > 0.f) {
						_battery_status_scale = battery_status.scale;
					}
				}

				if (_battery_status_scale > 0.f) {
					for (int i = 0; i < 3; i++) {
						vehicle_thrust_setpoint.xyz[i] = math::constrain(vehicle_thrust_setpoint.xyz[i] * _battery_status_scale, -1.f, 1.f);
						vehicle_torque_setpoint.xyz[i] = math::constrain(vehicle_torque_setpoint.xyz[i] * _battery_status_scale, -1.f, 1.f);
					}
				}
			}

			vehicle_thrust_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
			vehicle_thrust_setpoint.timestamp = hrt_absolute_time();
			_vehicle_thrust_setpoint_pub.publish(vehicle_thrust_setpoint);

			vehicle_torque_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
			vehicle_torque_setpoint.timestamp = hrt_absolute_time();
			_vehicle_torque_setpoint_pub.publish(vehicle_torque_setpoint);

			updateActuatorControlsStatus(vehicle_torque_setpoint, dt);

		}
	}

	perf_end(_loop_perf);
}

void MulticopterRateADRC::updateActuatorControlsStatus(const vehicle_torque_setpoint_s &vehicle_torque_setpoint,
		float dt)
{
	for (int i = 0; i < 3; i++) {
		_control_energy[i] += vehicle_torque_setpoint.xyz[i] * vehicle_torque_setpoint.xyz[i] * dt;
	}

	_energy_integration_time += dt;

	if (_energy_integration_time > 500e-3f) {

		actuator_controls_status_s status;
		status.timestamp = vehicle_torque_setpoint.timestamp;

		for (int i = 0; i < 3; i++) {
			status.control_power[i] = _control_energy[i] / _energy_integration_time;
			_control_energy[i] = 0.f;
		}

		_actuator_controls_status_pub.publish(status);
		_energy_integration_time = 0.f;
	}
}

int MulticopterRateADRC::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	MulticopterRateADRC *instance = new MulticopterRateADRC(vtol);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int MulticopterRateADRC::print_status()
{
	PX4_INFO("ADRC roll out %f", static_cast<double>(adrc_control_roll));
	PX4_INFO("ADRC roll rate_setpoint %f", static_cast<double>(_rates_setpoint(0)));
	PX4_INFO("ADRC roll rate %f", static_cast<double>(adrc_roll.ctl_param.measure));
	PX4_INFO("ADRC roll x1 (estimated roll rate) %f", static_cast<double>(adrc_roll.ctl_param.x1));
	PX4_INFO("ADRC roll error (roll rate - x1) %f", static_cast<double>(adrc_roll.ctl_param.error));
	PX4_INFO("ADRC roll x2 (lumped disturbance)%f", static_cast<double>(adrc_roll.ctl_param.x2));

	PX4_INFO("-------");

	PX4_INFO("ADRC pitch out %f", static_cast<double>(adrc_control_pitch));
	PX4_INFO("ADRC pitch rate_setpoint %f", static_cast<double>(_rates_setpoint(1)));
	PX4_INFO("ADRC pitch rate %f", static_cast<double>(adrc_pitch.ctl_param.measure));
	PX4_INFO("ADRC pitch x1 (estimated pitch rate)%f", static_cast<double>(adrc_pitch.ctl_param.x1));
	PX4_INFO("ADRC pitch error (pitch rate - x1) %f", static_cast<double>(adrc_pitch.ctl_param.error));
	PX4_INFO("ADRC pitch x2 (lumped disturbance)%f", static_cast<double>(adrc_pitch.ctl_param.x2));

	PX4_INFO("-------");

	PX4_INFO("ADRC yaw out %f", static_cast<double>(adrc_control_yaw));
	PX4_INFO("ADRC yaw rate_setpoint %f", static_cast<double>(_rates_setpoint(2)));
	PX4_INFO("ADRC yaw rate %f", static_cast<double>(adrc_yaw.ctl_param.measure));
	PX4_INFO("ADRC yaw x1 (estimated yaw rate)%f", static_cast<double>(adrc_yaw.ctl_param.x1));
	PX4_INFO("ADRC yaw error (yaw rate - x1) %f", static_cast<double>(adrc_yaw.ctl_param.error));
	PX4_INFO("ADRC yaw x2 (lumped disturbance)%f", static_cast<double>(adrc_yaw.ctl_param.x2));

	// PX4_INFO("-------");
	// PX4_INFO("ADRC yaw2 out %f", static_cast<double>(adrc_control_yaw2));
	// PX4_INFO("ADRC yaw2 yaw_sp %f", static_cast<double>(euler_yaw_d));
	// PX4_INFO("ADRC yaw2 yaw %f", static_cast<double>(adrc_yaw2.ctl_param.measure));
	// PX4_INFO("ADRC yaw2 x1 (estimated yaw)%f", static_cast<double>(adrc_yaw2.ctl_param.x1));
	// PX4_INFO("ADRC yaw2 error (yaw - x1) %f", static_cast<double>(adrc_yaw2.ctl_param.error));
	// PX4_INFO("ADRC yaw2 x2 (estimated yaw rate)%f", static_cast<double>(adrc_yaw2.ctl_param.x2));
	return 0;
}

int MulticopterRateADRC::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MulticopterRateADRC::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the multicopter rate controller. It takes rate setpoints (in acro mode
via `manual_control_setpoint` topic) as inputs and outputs actuator control messages.

The controller has a one order LADRC loop for angular rate error.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_rate_adrc", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int mc_rate_adrc_main(int argc, char *argv[])
{
	return MulticopterRateADRC::main(argc, argv);
}

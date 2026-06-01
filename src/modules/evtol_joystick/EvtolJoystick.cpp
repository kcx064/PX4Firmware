/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include "EvtolJoystick.hpp"

EvtolJoystick::EvtolJoystick() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::uavcan)
{
}

EvtolJoystick::~EvtolJoystick()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool EvtolJoystick::init()
{
	// execute Run() on every sensor_accel publication
	if (!_input_rc_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	// alternatively, Run on fixed interval
	// ScheduleOnInterval(50_ms); // 50 ms interval, 20 Hz rate

	return true;
}

void EvtolJoystick::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams(); // update module parameters (in DEFINE_PARAMETERS)

		stkarm_channel = _param_stkarm_channel.get();
	}

	if (_vehicle_status_sub.updated()) {
		_vehicle_status_sub.copy(&vehicle_status);
	}

	if (_input_rc_sub.updated()) {
		_input_rc_sub.copy(&_input_rc);
		// PX4_INFO("RC channel count: %d", _input_rc.channel_count);
		// PX4_INFO("RC channel %ld value: %d", shutdown_channel + 1, _input_rc.values[shutdown_channel]);

		vehicle_command_s cmd = {};
		cmd.command = vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM;

		uint16_t stk_arm_channel = _input_rc.values[stkarm_channel];

		switch (_key_state)
		{
		case key_state::waitaction:
			if(stk_arm_channel_last > 1070 && stk_arm_channel_last < 1200 && stk_arm_channel < 1070)
			{
				_key_state = key_state::short_press_key;//检测到短按
				time_short_press = hrt_absolute_time();
				mavlink_log_warning(&_mavlink_log_pub, "shot press.");
			}
			break;

		case key_state::short_press_key:
			if(stk_arm_channel_last > 1200 && stk_arm_channel <1070)
			{
				_key_state = key_state::long_press_key;
				PX4_INFO("long press.");
			}else if(stk_arm_channel_last > 1070 && stk_arm_channel_last < 1200 && stk_arm_channel < 1070){
				_key_state = key_state::waitaction;
				mavlink_log_warning(&_mavlink_log_pub, "shot press.");
			}

			if(hrt_absolute_time() - time_short_press >= 5_s)
			{
				_key_state = key_state::waitaction;
				PX4_INFO("long press timeout.");
			}
			break;

		case key_state::long_press_key:
			if(vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED)
			{//目前是解锁状态，那么上锁
				cmd.param1 = 0.0f;//上锁
				cmd.target_system = 1;
				cmd.target_component = 1;
				cmd.timestamp = hrt_absolute_time();
				// 发布到vehicle_command主题
				_vehicle_cmd_pub.publish(cmd);
			}else{//目前是上锁状态，那么解锁
				cmd.param1 = 1.0f;//1.0表示解锁
				cmd.target_system = 1;
				cmd.target_component = 1;
				cmd.timestamp = hrt_absolute_time();
				// 发布到vehicle_command主题
				_vehicle_cmd_pub.publish(cmd);
			}

			_key_state = key_state::waitaction;
			break;

		default:
			_key_state = key_state::waitaction;
			break;
		}

		stk_arm_channel_last = _input_rc.values[stkarm_channel];


	}

	perf_end(_loop_perf);
}

int EvtolJoystick::task_spawn(int argc, char *argv[])
{
	EvtolJoystick *instance = new EvtolJoystick();

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

int EvtolJoystick::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int EvtolJoystick::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int EvtolJoystick::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Example of a simple module running out of a work queue.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("evtol_joystick", "joystick");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int evtol_joystick_main(int argc, char *argv[])
{
	int32_t rc_port_config = 0;
	(void)param_get(param_find("RC_PORT_CONFIG"), &rc_port_config);
	if(rc_port_config){
		return EvtolJoystick::main(argc, argv);
	}else{
		return 0;
	}

}

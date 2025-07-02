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

#include "EvtolGpio.hpp"

EvtolGpio::EvtolGpio() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::uavcan)
{
}

EvtolGpio::~EvtolGpio()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool EvtolGpio::init()
{
	// execute Run() on every sensor_accel publication
	// if (!_sensor_accel_sub.registerCallback()) {
	// 	PX4_ERR("callback registration failed");
	// 	return false;
	// }

	// alternatively, Run on fixed interval
	ScheduleOnInterval(500_ms); // 500 ms interval, 2 Hz rate

	return true;
}

void EvtolGpio::Run()
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
#ifdef EVTOL_DB
#pragma message("Code under EVTOL_DB is being compiled.")
		BMS_POWER_EN(_param_bms_en.get());
		RC_SEL(_param_rc_sel.get());
#endif

#ifdef EVTOL_MIX
#pragma message("Code under EVTOL_MIX is being compiled.")
		start_precharge = _param_precharge.get();
		shutdown = _param_shutdown.get();
#endif
	}

/* state mechaine */
#ifdef EVTOL_MIX
	switch (_precharge_state)
	{
	case precharge_state::waitaction:
		if(start_precharge){
			AUX5_IO(true);
			_precharge_state = precharge_state::charging;
			timechargestart = hrt_absolute_time();
			mavlink_log_warning(&_mavlink_log_pub, "Precharge start");
		}else{
			AUX5_IO(false);
			AUX6_IO(false);
		}
		_param_shutdown.set(false);
		_param_shutdown.commit();
		break;

	case precharge_state::charging:
		/* code */
		if(hrt_absolute_time() - timechargestart >= 3_s){
			//使能AUX6输出将预充短路。在进入下一个状态后，再正式断开预充AUX5
			AUX6_IO(true);
			_precharge_state = precharge_state::complete;
			mavlink_log_warning(&_mavlink_log_pub, "Precharge complete");
		}
		break;

	case precharge_state::complete:
		_param_precharge.set(false);
		_param_precharge.commit();
		//拉低AUX5，结束预充
		AUX5_IO(false);
		_precharge_state = precharge_state::poweroff;
		break;

	case precharge_state::poweroff:
		if(shutdown){
			//如果关闭电源，拉低AUX6并且设置状态为waitaction，修改shutdown参数
			AUX6_IO(false);
			// ScheduleDelayed(500_us);
			_precharge_state = precharge_state::waitaction;
			mavlink_log_warning(&_mavlink_log_pub, "Power off");
		}
		_param_precharge.set(false);
		_param_precharge.commit();
		break;

	default:
		_precharge_state = precharge_state::waitaction;
		break;
	}
#endif

	perf_end(_loop_perf);
}

int EvtolGpio::task_spawn(int argc, char *argv[])
{
	EvtolGpio *instance = new EvtolGpio();

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

int EvtolGpio::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int EvtolGpio::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int EvtolGpio::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Example of a simple module running out of a work queue.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("evtol_gpio", "gpio");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int evtol_gpio_main(int argc, char *argv[])
{
	return EvtolGpio::main(argc, argv);
}

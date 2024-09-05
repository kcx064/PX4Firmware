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

#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>

#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/parameter_update.h>
// #include <uORB/topics/sensor_accel.h>
#include <uORB/topics/vehicle_status.h>

#include <uORB/topics/mixer_outputs.h>
#include <uORB/topics/can_actuator_test.h> //only for actuator debug when disarmed
#include <uORB/topics/servoinfo.h>
#include <uORB/topics/can_esc_report.h>
#include <uORB/topics/can_esc_ret.h>
#include <uORB/topics/can_servo_ret.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/esc_status.h>

#include "MW_PX4_CAN.h"

using namespace time_literals;

class WorkItemExample : public ModuleBase<WorkItemExample>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	WorkItemExample();
	~WorkItemExample() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

private:
	void Run() override;

	void set_servo_postion(uint8_t can_index, uint16_t *cmd);

	void set_esc_value(uint8_t can_index, int16_t *cmd, int16_t esc_frq);

	void collect_servo_report(uint8_t can_index);
	void collect_esc_report(uint8_t can_index);
	void collect_bms_report(uint8_t can_index);

	void decode_servo_report(uint8_t can_index, uint32_T id, uint8_t sevo_index);
	void decode_esc_report(uint8_t can_index, uint32_T id, uint8_t esc_index);

	// Publications
	// uORB::Publication<orb_test_s> _orb_test_pub{ORB_ID(orb_test)};
	uORB::PublicationMulti<esc_status_s> _esc_status_pub{ORB_ID(esc_status)};
	esc_status_s      _can_esc_status{};
	uORB::PublicationMulti<battery_status_s> _bms_status_pub{ORB_ID(battery_status)};
	battery_status_s  _can_bms_status{};

	uORB::PublicationMulti<battery_status_s> _24v_status_pub{ORB_ID(battery_status)};
	battery_status_s  _24v_status{};

	uORB::Publication<can_esc_ret_s> _can_esc_ret_pub{ORB_ID(can_esc_ret)};
	can_esc_ret_s    _can_esc_ret{};

	uORB::Publication<can_servo_ret_s> _can_servo_ret_pub{ORB_ID(can_servo_ret)};
	can_servo_ret_s _can_servo_ret{};

	// Subscriptions
	uORB::SubscriptionCallbackWorkItem _mixer_output_sub{this, ORB_ID(mixer_outputs)};        // subscription that schedules WorkItemExample when updated
	uORB::SubscriptionInterval         _parameter_update_sub{ORB_ID(parameter_update), 1_s};  // subscription limited to 1 Hz updates
	uORB::Subscription                 _vehicle_status_sub{ORB_ID(vehicle_status)};           // regular subscription for additional data
	uORB::Subscription                 _can_actuator_test_sub{ORB_ID(can_actuator_test)};

	// Performance (perf) counters
	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

	// Parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SYS_AUTOSTART>) _param_sys_autostart,   /**< example parameter */
		(ParamInt<px4::params::SYS_AUTOCONFIG>) _param_sys_autoconfig,  /**< another parameter */
		(ParamFloat<px4::params::BAT_CELL_V>) _param_bat_cell_v,
		(ParamInt<px4::params::DB_SERVO_BIAS>) _param_db_servo_bias
	)


	bool _armed{false};

	int servo_bias{0};

	int8_t retcanInit = -1;
	int8_t retcanInit_0 = -1;
	uint8_t servo_uavcan_msg_index = 0;
	//0:can1 port 1:can2 port
	uint8_t can_port_2 = 1; // servo
	uint8_t can_port_1 = 0; // esc
	int16_t used_esc_frq = 4000; //uint 0.1Hz
	int16_t esc_output[3] = {0,0,0};// 0-4095
	uint16_t servo_output[4] = {500,500,500,500};

	mixer_outputs_s mixer_outputs{};
	can_actuator_test_s can_actuator_test{};
	bool _can_test_run{false};

	uint8_t remote;
	uint8_t Length;
	uint8_t receiveData[8];

	int servoinfo_instance[4];
	orb_advert_t _servoinfo_sub[4];
	servoinfo_s servo_report[4]{};

	int esc_report_instance[3];
	orb_advert_t _esc_report_sub[3];
	can_esc_report_s esc_report[3]{};

	/*used for servoinfo report decode*/
	enum decode_servoinfo_state
	{
		PACK_H = 0,
		PACK_L,
	};
	uint8_t const servoinfo_uavcan_struct[9] = {5, 12, 16, 16, 12, 10, 10, 10, 5};

	typedef struct servo_decode_s{
		decode_servoinfo_state servoinfo_state = decode_servoinfo_state::PACK_H;
		uint8_t msg_SN{0};
		uint16_t servoinfo_raw_data[9] = {0};
		uint64_t receivePack = 0;
	}servo_decode_t;

	servo_decode_t servo_decode_state[4];

#pragma pack(push,1)
	typedef union bms_hcu_info_u
	{
		uint8_t data_raw[8];
		struct bms_hcu_info_s
		{
			uint16_t batVoltage;
			uint16_t batCurrent;
			uint8_t batSoc;//电池状态
			uint8_t batSoh;//健康状态
			uint8_t batState;
			uint8_t batLife;
		}data;
	}bms_hcu_info_t;

	typedef union bms_hcu_cellv_u
	{
		uint8_t data_raw[8];
		struct bms_hcu_cellv_s
		{
			uint16_t cellVoltage[4];
		}data;
	}bms_hcu_cellv_t;

	typedef union bms_hcu_cellt_u
	{
		uint8_t data_raw[8];
		struct bms_hcu_cellt_s
		{
			uint16_t cellTemp[8];
		}data;
	}bms_hcu_cellt_t;

	typedef union esc_report_1_u
	{
		uint8_t data_raw[8];
		struct esc_report_1_s
		{
			int16_t voltage_in;
			int16_t current_in;
			int16_t current_out;
			int16_t freq_out;
		}data;
	}esc_report_1_t;

	typedef union esc_report_2_u
	{
		uint8_t data_raw[8];
		struct esc_report_2_s
		{
			uint16_t throttle_right;
			uint16_t throttle_current;
			int16_t t_mcu;
			int16_t t_module;
		}data;
	}esc_report_2_t;

	typedef union esc_report_3_u
	{
		uint8_t data_raw[8];
		struct esc_report_3_s
		{
			uint32_t status;
			uint16_t theta;
			int16_t t_motor;
		}data;
	}esc_report_3_t;

#pragma pack(pop)

	bms_hcu_info_t bms_hcu_info;
	bms_hcu_cellv_t bms_hcu_cellv;
	bms_hcu_cellt_t bms_hcu_cellt;

	esc_report_1_t esc_status_1;
	esc_report_2_t esc_status_2;
	esc_report_3_t esc_status_3;

};

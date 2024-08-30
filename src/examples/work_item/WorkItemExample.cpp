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

#include "WorkItemExample.hpp"

#define DB_HIPOWER_EN  	/* PD11 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN11)
#define DB_RC_EN  	/* PG5  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTG|GPIO_PIN5)

#define CAN_NODE_ID 1U
// #define SEVONODE_1_ID 100

/* BMS can */
#define BMS_HCU_INFO_DATA_TYPE_ID  0x186040F3
#define BMS_HCU_ALARM_DATA_TYPE_ID 0x186540F3

#define BMS_HCU_CELLV_DATA_TYPE_ID 0x180050F3
#define BMS_HCU_CELLT_DATA_TYPE_ID 0x185050F3

/* servo can */
#define SERVO_UAVCAN_SOURCE_NODE_ID CAN_NODE_ID
#define SERVO_UAVCAN_DATATYPE_ID 2018U
#define SERVO_UAVCAN_PRIORITY 0x18
#define SERVO_UAVCAN_CONTROL_DATA_TYPE_ID (SERVO_UAVCAN_PRIORITY << 24 | SERVO_UAVCAN_DATATYPE_ID << 8 | SERVO_UAVCAN_SOURCE_NODE_ID)

#define SERVO1_CAN_REPORT_DATA_TYPE_ID 0x1807E364
#define SERVO2_CAN_REPORT_DATA_TYPE_ID 0x1807E365
#define SERVO3_CAN_REPORT_DATA_TYPE_ID 0x1807E366
#define SERVO4_CAN_REPORT_DATA_TYPE_ID 0x1807E367

#define SERVO_UAVCAN_SERVO_INFO_SIGNATURE 0xCA8F4B8F97D23B57
#define SERVO_CAN_CONTROL_DATA_MIN 0
#define SERVO_CAN_CONTROL_DATA_MAX 1000

/* esc can */
#define ESC1_CAN_CONTROL_DATA_TYPE_ID 1

#define ESC1_CAN_REPORT_1_DATA_TYPE_ID 0x1100
#define ESC2_CAN_REPORT_1_DATA_TYPE_ID 0x2100
#define ESC3_CAN_REPORT_1_DATA_TYPE_ID 0x3100

#define ESC1_CAN_REPORT_2_DATA_TYPE_ID 0x1110
#define ESC2_CAN_REPORT_2_DATA_TYPE_ID 0x2110
#define ESC3_CAN_REPORT_2_DATA_TYPE_ID 0x3110

#define ESC1_CAN_REPORT_3_DATA_TYPE_ID 0x1120
#define ESC2_CAN_REPORT_3_DATA_TYPE_ID 0x2120
#define ESC3_CAN_REPORT_3_DATA_TYPE_ID 0x3120

#define ESC1_CAN_REPORT_4_DATA_TYPE_ID 0x1130
#define ESC2_CAN_REPORT_4_DATA_TYPE_ID 0x2130
#define ESC3_CAN_REPORT_4_DATA_TYPE_ID 0x3130

#define MOTOR_POLES 10

#define ESC_CAN_CONTROL_DATA_MIN 0
#define ESC_CAN_CONTROL_DATA_MAX 4095

// #define ESC_CHANNEL_NONE "none"
// #define ESC_CHANNEL_PWM  "pwm"
// #define ESC_CHANNEL_CAN1 "can1"
// #define ESC_CHANNEL_CAN2 "can2"
// #define ESC_CHANNEL_UART "uart"

WorkItemExample::WorkItemExample() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1)
{
}

WorkItemExample::~WorkItemExample()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool WorkItemExample::init()
{
	// execute Run() on every mixer_output publication
	if (!_mixer_output_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	// alternatively, Run on fixed interval
	// ScheduleOnInterval(1000_ms); // 2000 us interval, 200 Hz rate
	// printf("WorkItemExample init success!\n");

	px4_arch_configgpio(DB_HIPOWER_EN);
	px4_arch_configgpio(DB_RC_EN);

	px4_arch_gpiowrite(DB_HIPOWER_EN, 1);//enable for BMS, for stander CUAV X7 Pro, this will enable UART 5V power
	px4_arch_gpiowrite(DB_RC_EN, 0);

	/* servo report */
	_servoinfo_sub[0] = orb_advertise_multi(ORB_ID(servoinfo), &servo_report[0], &servoinfo_instance[0]);
	_servoinfo_sub[1] = orb_advertise_multi(ORB_ID(servoinfo), &servo_report[1], &servoinfo_instance[1]);
	_servoinfo_sub[2] = orb_advertise_multi(ORB_ID(servoinfo), &servo_report[2], &servoinfo_instance[2]);
	_servoinfo_sub[3] = orb_advertise_multi(ORB_ID(servoinfo), &servo_report[3], &servoinfo_instance[3]);
	/* esc report */
	_esc_report_sub[0] = orb_advertise_multi(ORB_ID(can_esc_report), &esc_report[0], &esc_report_instance[0]);
	_esc_report_sub[1] = orb_advertise_multi(ORB_ID(can_esc_report), &esc_report[1], &esc_report_instance[1]);
	_esc_report_sub[2] = orb_advertise_multi(ORB_ID(can_esc_report), &esc_report[2], &esc_report_instance[2]);

	/* declare received esc report 1 */
	MW_CAN_AssignGlobalBufferForID(can_port_1, ESC1_CAN_REPORT_1_DATA_TYPE_ID, 1);
	MW_CAN_AssignGlobalBufferForID(can_port_1, ESC2_CAN_REPORT_1_DATA_TYPE_ID, 1);
	MW_CAN_AssignGlobalBufferForID(can_port_1, ESC3_CAN_REPORT_1_DATA_TYPE_ID, 1);
	/* declare received esc report 2 */
	MW_CAN_AssignGlobalBufferForID(can_port_1, ESC1_CAN_REPORT_2_DATA_TYPE_ID, 1);
	MW_CAN_AssignGlobalBufferForID(can_port_1, ESC2_CAN_REPORT_2_DATA_TYPE_ID, 1);
	MW_CAN_AssignGlobalBufferForID(can_port_1, ESC3_CAN_REPORT_2_DATA_TYPE_ID, 1);
	/* declare received esc report 3 */
	MW_CAN_AssignGlobalBufferForID(can_port_1, ESC1_CAN_REPORT_3_DATA_TYPE_ID, 1);
	MW_CAN_AssignGlobalBufferForID(can_port_1, ESC2_CAN_REPORT_3_DATA_TYPE_ID, 1);
	MW_CAN_AssignGlobalBufferForID(can_port_1, ESC3_CAN_REPORT_3_DATA_TYPE_ID, 1);
	/* declare received servo report */
	MW_CAN_AssignGlobalBufferForID(can_port_2, SERVO1_CAN_REPORT_DATA_TYPE_ID, 1);
	MW_CAN_AssignGlobalBufferForID(can_port_2, SERVO2_CAN_REPORT_DATA_TYPE_ID, 1);
	MW_CAN_AssignGlobalBufferForID(can_port_2, SERVO3_CAN_REPORT_DATA_TYPE_ID, 1);
	MW_CAN_AssignGlobalBufferForID(can_port_2, SERVO4_CAN_REPORT_DATA_TYPE_ID, 1);
	/* bms */
	MW_CAN_AssignGlobalBufferForID(can_port_2, BMS_HCU_INFO_DATA_TYPE_ID, 1);
	MW_CAN_AssignGlobalBufferForID(can_port_2, BMS_HCU_ALARM_DATA_TYPE_ID, 1);
	MW_CAN_AssignGlobalBufferForID(can_port_2, BMS_HCU_CELLV_DATA_TYPE_ID, 1);
	MW_CAN_AssignGlobalBufferForID(can_port_2, BMS_HCU_CELLT_DATA_TYPE_ID, 1);


	return true;
}

void WorkItemExample::Run()
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
	}


	// Example
	//  update vehicle_status to check arming state
	if (_vehicle_status_sub.updated()) {
		vehicle_status_s vehicle_status;

		if (_vehicle_status_sub.copy(&vehicle_status)) {

			const bool armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);

			if (armed && !_armed) {
				PX4_WARN("vehicle armed due to %d", vehicle_status.latest_arming_reason);

			} else if (!armed && _armed) {
				PX4_INFO("vehicle disarmed due to %d", vehicle_status.latest_disarming_reason);
			}

			_armed = armed;
		}
	}


	/* update mixer output */
	if (_mixer_output_sub.updated()) {

		if (_mixer_output_sub.copy(&mixer_outputs)) {
			// DO WORK

			// access parameter value (SYS_AUTOSTART)
			if (_param_sys_autostart.get() == 1234) {
				// do something if SYS_AUTOSTART is 1234
			}
		}
	}

	/* used for can test with diarmd */
	if (_can_actuator_test_sub.updated()) {

		if (_can_actuator_test_sub.copy(&can_actuator_test)){
			// DO WORK
			if(!_armed){
				if(can_actuator_test.is_run){//test is run
					if(can_actuator_test.actuator_index < 3){//0 1 2 for esc
						esc_output[0] = 0;
						esc_output[1] = 0;
						esc_output[2] = 0;
						esc_output[can_actuator_test.actuator_index] = can_actuator_test.cmd*(ESC_CAN_CONTROL_DATA_MAX - ESC_CAN_CONTROL_DATA_MIN)/2 + (ESC_CAN_CONTROL_DATA_MAX + ESC_CAN_CONTROL_DATA_MIN)/2;
						set_esc_value(can_port_1,  &esc_output[0], used_esc_frq);
					}else if(can_actuator_test.actuator_index==7){
						servo_output[0] = - can_actuator_test.cmd*(SERVO_CAN_CONTROL_DATA_MAX - SERVO_CAN_CONTROL_DATA_MIN)/2 + (SERVO_CAN_CONTROL_DATA_MAX + SERVO_CAN_CONTROL_DATA_MIN)/2;
						servo_output[1] = servo_output[0];
						servo_output[2] = servo_output[0];
						servo_output[3] = servo_output[0];
						set_servo_postion(can_port_2, &servo_output[0]);
					}else if(can_actuator_test.actuator_index==8){//set all esc at same time
						esc_output[0] = can_actuator_test.cmd*(ESC_CAN_CONTROL_DATA_MAX - ESC_CAN_CONTROL_DATA_MIN)/2 + (ESC_CAN_CONTROL_DATA_MAX + ESC_CAN_CONTROL_DATA_MIN)/2;
						esc_output[1] = esc_output[0];
						esc_output[2] = esc_output[0];
						set_esc_value(can_port_1,  &esc_output[0], used_esc_frq);
					}
					else{//3 4 5 6 for servo
						servo_output[0] = 500;
						servo_output[1] = 500;
						servo_output[2] = 500;
						servo_output[3] = 500;
						// reverse servo output
						servo_output[can_actuator_test.actuator_index-3] = - can_actuator_test.cmd*(SERVO_CAN_CONTROL_DATA_MAX - SERVO_CAN_CONTROL_DATA_MIN)/2 + (SERVO_CAN_CONTROL_DATA_MAX + SERVO_CAN_CONTROL_DATA_MIN)/2;
						set_servo_postion(can_port_2, &servo_output[0]);
					}
				}else{//stop test
					esc_output[0] = 0;
					esc_output[1] = 0;
					esc_output[2] = 0;
					set_esc_value(can_port_1,  &esc_output[0], used_esc_frq);
					servo_output[0] = 500;
					servo_output[1] = 500;
					servo_output[2] = 500;
					servo_output[3] = 500;
					set_servo_postion(can_port_2, &servo_output[0]);
				}
			}
		}
	}


	/* send can data */
	if (retcanInit < 0 || retcanInit_0 < 0)/* not init can port */
	{
		retcanInit_0 = MW_CAN_Open(can_port_1, 1000000U, 0);
		PX4_INFO("can port 1 open, ret: %d", (int)retcanInit_0);

		retcanInit = MW_CAN_Open(can_port_2, 1000000U, 0);
		PX4_INFO("can port 2 open, ret: %d", (int)retcanInit);
	}else{/* can port has been initialized, therefore send data */
		collect_esc_report(can_port_1);
		collect_servo_report(can_port_2);
		collect_bms_report(can_port_2);
		/* set esc cmd msg */
		for (int i = 0; i < 3; i++)
		{
			esc_output[i] = mixer_outputs.output[i]*(ESC_CAN_CONTROL_DATA_MAX - ESC_CAN_CONTROL_DATA_MIN)/2 + (ESC_CAN_CONTROL_DATA_MAX + ESC_CAN_CONTROL_DATA_MIN)/2;
		}
		if(_armed){
			set_esc_value(can_port_1,  &esc_output[0], used_esc_frq);
		}else{
			if(!can_actuator_test.is_run){
				esc_output[0] = 0;
				esc_output[1] = 0;
				esc_output[2] = 0;
				set_esc_value(can_port_1,  &esc_output[0], used_esc_frq);
			}
		}

		/* set servo cmd msg */
		for (int i = 3; i < 7; i++)
		{
			// reverse servo output
			servo_output[i-3] = - mixer_outputs.output[i]*(SERVO_CAN_CONTROL_DATA_MAX - SERVO_CAN_CONTROL_DATA_MIN)/2 + (SERVO_CAN_CONTROL_DATA_MAX + SERVO_CAN_CONTROL_DATA_MIN)/2;
		}
		if(_armed){
			set_servo_postion(can_port_2, &servo_output[0]);
		}else{
			if(!can_actuator_test.is_run){
				servo_output[0] = 500;
				servo_output[1] = 500;
				servo_output[2] = 500;
				servo_output[3] = 500;
				set_servo_postion(can_port_2, &servo_output[0]);
			}
		}
	}


	perf_end(_loop_perf);
}

void WorkItemExample::collect_bms_report(uint8_t can_index){
	if(!MW_CAN_ReceiveMessages_By_ID(can_index, bms_hcu_info.data_raw, BMS_HCU_INFO_DATA_TYPE_ID, 1, &remote, &Length)
	&& !MW_CAN_ReceiveMessages_By_ID(can_index, bms_hcu_cellv.data_raw, BMS_HCU_CELLV_DATA_TYPE_ID, 1, &remote, &Length)
	&& !MW_CAN_ReceiveMessages_By_ID(can_index, bms_hcu_cellt.data_raw, BMS_HCU_CELLT_DATA_TYPE_ID, 1, &remote, &Length))
	{

		_can_bms_status.timestamp = hrt_absolute_time();
		_can_bms_status.voltage_v = bms_hcu_info.data.batVoltage*0.1f;
		_can_bms_status.voltage_filtered_v = bms_hcu_info.data.batVoltage*0.1f;
		_can_bms_status.cell_count = 12;
		_can_bms_status.scale = 1;
		_can_bms_status.voltage_cell_v[0] = _can_bms_status.voltage_v/12;//max: 53.4 equivalent 12cell * 4.45Vmax
		_can_bms_status.voltage_cell_v[1] = _can_bms_status.voltage_v/12;
		_can_bms_status.voltage_cell_v[2] = _can_bms_status.voltage_v/12;
		_can_bms_status.voltage_cell_v[3] = _can_bms_status.voltage_v/12;
		_can_bms_status.voltage_cell_v[4] = _can_bms_status.voltage_v/12;
		_can_bms_status.voltage_cell_v[5] = _can_bms_status.voltage_v/12;
		_can_bms_status.voltage_cell_v[6] = _can_bms_status.voltage_v/12;
		_can_bms_status.voltage_cell_v[7] = _can_bms_status.voltage_v/12;
		_can_bms_status.voltage_cell_v[8] = _can_bms_status.voltage_v/12;
		_can_bms_status.voltage_cell_v[9] = _can_bms_status.voltage_v/12;
		_can_bms_status.voltage_cell_v[10] = _can_bms_status.voltage_v/12;
		_can_bms_status.voltage_cell_v[11] = _can_bms_status.voltage_v/12;
		_can_bms_status.remaining = (_can_bms_status.voltage_v - 504.0f)/(640.8f - 504.0f);
		_can_bms_status.id = 4;
		_can_bms_status.temperature = 30;
		_can_bms_status.connected = true;
		if(_can_bms_status.voltage_v < 640.8f)_can_bms_status.warning = battery_status_s::BATTERY_WARNING_NONE;//4.45
		if(_can_bms_status.voltage_v < 561.60f)_can_bms_status.warning = battery_status_s::BATTERY_WARNING_LOW;//3.9
		if(_can_bms_status.voltage_v < 547.2f)_can_bms_status.warning = battery_status_s::BATTERY_WARNING_CRITICAL;//3.8
		if(_can_bms_status.voltage_v < 532.8f)_can_bms_status.warning = battery_status_s::BATTERY_WARNING_EMERGENCY;//3.7
		if(_can_bms_status.voltage_v < 504.0f)_can_bms_status.warning = battery_status_s::BATTERY_WARNING_FAILED;//3.5

		_bms_status_pub.publish(_can_bms_status);
	}


}

void WorkItemExample::collect_servo_report(uint8_t can_index){
	decode_servo_report(can_index, SERVO1_CAN_REPORT_DATA_TYPE_ID, 0);
	decode_servo_report(can_index, SERVO2_CAN_REPORT_DATA_TYPE_ID, 1);
	decode_servo_report(can_index, SERVO3_CAN_REPORT_DATA_TYPE_ID, 2);
	decode_servo_report(can_index, SERVO4_CAN_REPORT_DATA_TYPE_ID, 3);
}

void WorkItemExample::decode_servo_report(uint8_t can_index, uint32_T id, uint8_t sevo_index){
	memset(&receiveData[0], 0, sizeof(receiveData));

	if(!MW_CAN_ReceiveMessages_By_ID(can_index, &receiveData[0], id, 1, &remote, &Length))
	{
		// PX4_WARN("receive data start %d",sevo_index);
		switch (servo_decode_state[sevo_index].servoinfo_state)
		{
		case PACK_H:
			if(receiveData[7] & 0x80){
				servo_decode_state[sevo_index].servoinfo_state = decode_servoinfo_state::PACK_L;
				servo_decode_state[sevo_index].msg_SN = receiveData[7] & 0x1F;
				//低字节放置到高位
				servo_decode_state[sevo_index].receivePack = 0;
				servo_decode_state[sevo_index].receivePack = (uint64_t)receiveData[2] << 8*7 | (uint64_t)receiveData[3] << 8*6 | (uint64_t)receiveData[4] << 8*5 | (uint64_t)receiveData[5] << 8*4 | (uint64_t)receiveData[6] << 8*3;
				// PX4_INFO("received data 1 is %llX",servo_decode_state[sevo_index].receivePack);
			}
			break;

		case PACK_L:
			if( (receiveData[7] & 0x40) && ((receiveData[7] & 0x1F) == servo_decode_state[sevo_index].msg_SN) ){
				//TODO: 增加签名校验部分
				//第一阶段
				servo_decode_state[sevo_index].receivePack = servo_decode_state[sevo_index].receivePack | (uint64_t)receiveData[0] << 8*2 | (uint64_t)receiveData[1] << 8*1 | (uint64_t)receiveData[2];
				// PX4_INFO("received data 2 is %llX",servo_decode_state[sevo_index].receivePack);
				for (uint8_t i = 0; i < 5; i++)
				{
					if(servoinfo_uavcan_struct[i] <= 8){//目标数据小于等于8位，那么一次提取即可
						//提取数据
						servo_decode_state[sevo_index].servoinfo_raw_data[i] = servo_decode_state[sevo_index].receivePack >> (64 - servoinfo_uavcan_struct[i]);
						//左移，去除已经提取的数据
						servo_decode_state[sevo_index].receivePack = servo_decode_state[sevo_index].receivePack << servoinfo_uavcan_struct[i];
					}else{//目标数据大于8位，根据厂家给出的定义，消息字段最长有16位，那么二次提取即可
						//提取低八位数据
						servo_decode_state[sevo_index].servoinfo_raw_data[i] = servo_decode_state[sevo_index].receivePack >> (64 - 8);
						//左移，去除已经提取的低八位数据
						servo_decode_state[sevo_index].receivePack = servo_decode_state[sevo_index].receivePack << 8;

						//提取高位数据，可能不足八位
						servo_decode_state[sevo_index].servoinfo_raw_data[i] = servo_decode_state[sevo_index].servoinfo_raw_data[i] | (( servo_decode_state[sevo_index].receivePack >> (64 - (servoinfo_uavcan_struct[i] - 8)) ) << 8);
						//左移，去除已经提取的数据
						servo_decode_state[sevo_index].receivePack = servo_decode_state[sevo_index].receivePack << (servoinfo_uavcan_struct[i] - 8);
					}
				}
				//第二阶段
				servo_decode_state[sevo_index].receivePack = 0;
				servo_decode_state[sevo_index].receivePack = (uint64_t)receiveData[2] << 8*7 | (uint64_t)receiveData[3] << 8*6 | (uint64_t)receiveData[4] << 8*5 | (uint64_t)receiveData[5] << 8*4 | (uint64_t)receiveData[6] << 8*3;
				// PX4_INFO("received data 3 is %llX",servo_decode_state[sevo_index].receivePack);
				servo_decode_state[sevo_index].receivePack = servo_decode_state[sevo_index].receivePack << 5;//手动计算得到的5，目的是消除上一个字段的高5位，该高五位在第一阶段已经提取
				for (uint8_t i = 5; i < 9; i++)
				{
					if(servoinfo_uavcan_struct[i] <= 8){//目标数据小于8字节，那么一次提取即可
						//提取数据
						servo_decode_state[sevo_index].servoinfo_raw_data[i] = servo_decode_state[sevo_index].receivePack >> (64 - servoinfo_uavcan_struct[i]);
						//左移，去除已经提取的数据
						servo_decode_state[sevo_index].receivePack = servo_decode_state[sevo_index].receivePack << servoinfo_uavcan_struct[i];
					}else{//目标数据大于8位，根据厂家给出的定义，消息字段最长有16位，那么二次提取即可
						//提取低八位数据
						servo_decode_state[sevo_index].servoinfo_raw_data[i] = servo_decode_state[sevo_index].receivePack >> (64 - 8);
						//左移，去除已经提取的低八位数据
						servo_decode_state[sevo_index].receivePack = servo_decode_state[sevo_index].receivePack << 8;

						//提取高位数据，可能不足八位
						servo_decode_state[sevo_index].servoinfo_raw_data[i] = servo_decode_state[sevo_index].servoinfo_raw_data[i] | (( servo_decode_state[sevo_index].receivePack >> (64 - (servoinfo_uavcan_struct[i] - 8)) ) << 8);
						//左移，去除已经提取的数据
						servo_decode_state[sevo_index].receivePack = servo_decode_state[sevo_index].receivePack << (servoinfo_uavcan_struct[i] - 8);
					}
				}
				//第三阶段整理数据，并发送
				servo_report[sevo_index].timestamp = hrt_absolute_time();
				servo_report[sevo_index].servo_id = servo_decode_state[sevo_index].servoinfo_raw_data[0];//uint5
				servo_report[sevo_index].pwm_input = servo_decode_state[sevo_index].servoinfo_raw_data[1];//uint12
				servo_report[sevo_index].pos_cmd = (int16_t)servo_decode_state[sevo_index].servoinfo_raw_data[2];//int16
				servo_report[sevo_index].pos_sensor = (int16_t)servo_decode_state[sevo_index].servoinfo_raw_data[3];//int16
				servo_report[sevo_index].voltage = servo_decode_state[sevo_index].servoinfo_raw_data[4];//uint12
				servo_report[sevo_index].current = servo_decode_state[sevo_index].servoinfo_raw_data[5];//uint10
				servo_report[sevo_index].pcb_temp = servo_decode_state[sevo_index].servoinfo_raw_data[6];//uint10
				servo_report[sevo_index].motor_temp = servo_decode_state[sevo_index].servoinfo_raw_data[7];//uint10
				servo_report[sevo_index].statusinfo = servo_decode_state[sevo_index].servoinfo_raw_data[8];//uint5
				orb_publish(ORB_ID(servoinfo), _servoinfo_sub[sevo_index], &servo_report[sevo_index]);

				if(sevo_index == 0){
					_24v_status.timestamp = hrt_absolute_time();
					_24v_status.voltage_v = servo_report[sevo_index].voltage*0.01;
					_24v_status.scale = 1;
					_24v_status.cell_count = 6;
					_24v_status.voltage_cell_v[0] = _24v_status.voltage_v/6;
					_24v_status.voltage_cell_v[1] = _24v_status.voltage_v/6;
					_24v_status.voltage_cell_v[2] = _24v_status.voltage_v/6;
					_24v_status.voltage_cell_v[3] = _24v_status.voltage_v/6;
					_24v_status.voltage_cell_v[4] = _24v_status.voltage_v/6;
					_24v_status.voltage_cell_v[5] = _24v_status.voltage_v/6;
					_24v_status.remaining = (servo_report[sevo_index].voltage*0.01 - 22.2)/(25.2-22.2);//4.2*6 - 3.7*6
					_24v_status.id = 3;
					// _24v_status.temperature = 26;
					if(_24v_status.voltage_v < 25.2f)_24v_status.warning = battery_status_s::BATTERY_WARNING_NONE;//4.2*6
					if(_24v_status.voltage_v < 23.4f)_24v_status.warning = battery_status_s::BATTERY_WARNING_LOW;//3.9
					if(_24v_status.voltage_v < 22.8f)_24v_status.warning = battery_status_s::BATTERY_WARNING_CRITICAL;//3.8
					if(_24v_status.voltage_v < 22.2f)_24v_status.warning = battery_status_s::BATTERY_WARNING_EMERGENCY;//3.7
					if(_24v_status.voltage_v < 21.0f)_24v_status.warning = battery_status_s::BATTERY_WARNING_FAILED;//3.5
					_24v_status.connected = true;
					_24v_status_pub.publish(_24v_status);
				}

			}
			servo_decode_state[sevo_index].servoinfo_state = decode_servoinfo_state::PACK_H;
			break;

		default:
			break;
		}
	}else{
		// PX4_INFO("receive data error %d",sevo_index);
	}
}

void WorkItemExample::collect_esc_report(uint8_t can_index){
	decode_esc_report(can_index, ESC1_CAN_REPORT_1_DATA_TYPE_ID, 0);
	decode_esc_report(can_index, ESC2_CAN_REPORT_1_DATA_TYPE_ID, 1);
	decode_esc_report(can_index, ESC3_CAN_REPORT_1_DATA_TYPE_ID, 2);
}

void WorkItemExample::decode_esc_report(uint8_t can_index, uint32_T id, uint8_t esc_index){
	// uint8_t report_1_raw[8] = {0,};
	// uint8_t report_2_raw[8] = {0,};
	// uint8_t report_3_raw[8] = {0,};
	if(!MW_CAN_ReceiveMessages_By_ID(can_index, esc_status_1.data_raw, id, 1, &remote, &Length)
	&& !MW_CAN_ReceiveMessages_By_ID(can_index, esc_status_2.data_raw, id + 0x10, 1, &remote, &Length)
	&& !MW_CAN_ReceiveMessages_By_ID(can_index, esc_status_3.data_raw, id + 0x20, 1, &remote, &Length))
	{
		esc_report[esc_index].timestamp = hrt_absolute_time();

		/* report 1 */
		esc_report[esc_index].voltage_in  = esc_status_1.data.voltage_in*0.1;//(report_1_raw[1] << 8 | report_1_raw[0])*0.1;
		esc_report[esc_index].current_in  = esc_status_1.data.current_in*0.1;//(report_1_raw[3] << 8 | report_1_raw[2])*0.1;
		esc_report[esc_index].current_out = esc_status_1.data.current_out*0.1;//(report_1_raw[5] << 8 | report_1_raw[4])*0.1;
		esc_report[esc_index].freq_out    = esc_status_1.data.freq_out*0.1;//(report_1_raw[7] << 8 | report_1_raw[6])*0.1;
		esc_report[esc_index].rpm = uint16_t(esc_report[esc_index].freq_out*60/MOTOR_POLES);

		/* report 2 */
		uint16_t temp = esc_status_2.data.throttle_right;//(report_2_raw[1] << 8 | report_2_raw[0]);
		uint8_t _channel = (temp & 0xF000) >> 12;

		esc_report[esc_index].throttle_desired = temp & 0x0FFF;
		esc_report[esc_index].pwm_channel = false;
		esc_report[esc_index].can1_channel = false;
		esc_report[esc_index].can2_channel = false;
		esc_report[esc_index].uart_channel = false;
		switch (_channel){
			case 1:
				esc_report[esc_index].pwm_channel = true;
				break;
			case 2:
				esc_report[esc_index].can1_channel = true;
				break;
			case 3:
				esc_report[esc_index].can2_channel = true;
				break;
			case 4:
				esc_report[esc_index].uart_channel = true;
				break;
			default:
				break;

		}
		esc_report[esc_index].throttle_current = esc_status_2.data.throttle_current;//(report_2_raw[3] << 8 | report_2_raw[2]);
		esc_report[esc_index].t_mcu = esc_status_2.data.t_mcu;//(report_2_raw[5] << 8 | report_2_raw[4]);
		esc_report[esc_index].t_module = esc_status_2.data.t_module;//(report_2_raw[7] << 8 | report_2_raw[6]);

		/* report 3 */
		esc_report[esc_index].status = esc_status_3.data.status;//(report_3_raw[3] << 24 | report_3_raw[2] << 16 | report_3_raw[1] << 8 | report_3_raw[0]);
		esc_report[esc_index].theta = esc_status_3.data.theta;//(report_3_raw[5] << 8 | report_3_raw[4]);
		esc_report[esc_index].t_motor = esc_status_3.data.t_motor;//(report_3_raw[7] << 8 | report_3_raw[6]);

		/* publish */
		orb_publish(ORB_ID(can_esc_report), _esc_report_sub[esc_index], &esc_report[esc_index]);

		/* publish only used for QGC display */
		_can_esc_status.esc[esc_index].timestamp = hrt_absolute_time();
		_can_esc_status.esc[esc_index].esc_rpm = esc_report[esc_index].rpm;
		_can_esc_status.esc[esc_index].esc_voltage = esc_report[esc_index].voltage_in;
		_can_esc_status.esc[esc_index].esc_current = esc_report[esc_index].current_in;
		_can_esc_status.esc[esc_index].esc_state = 0;
		_can_esc_status.esc[esc_index].failures = 0;
		_can_esc_status.esc[esc_index].esc_temperature = esc_report[esc_index].t_mcu;
		_can_esc_status.esc[esc_index].esc_errorcount = 0;

		_can_esc_status.timestamp = hrt_absolute_time();
		_can_esc_status.counter++;
		_can_esc_status.esc_count = 3;
		_can_esc_status.esc_connectiontype = esc_status_s::ESC_CONNECTION_TYPE_CAN;
		_can_esc_status.esc_online_flags |= 1 << esc_index;
		_can_esc_status.esc_armed_flags |= 1 << esc_index;
		_esc_status_pub.publish(_can_esc_status);
	}

}

void WorkItemExample::set_servo_postion(uint8_t can_index, uint16_t *cmd){
	/* convert cmd data to txData for uavcan servo */
	uint8_t txData[6] ={0,};
	/* for 1st servo */
	txData[0] = uint8_t(cmd[0] & 0b0011111111);
	txData[1] = uint8_t((cmd[0] & 0b1100000000)>>2);

	/* for 2nd servo */
	txData[1] |= uint8_t(cmd[1] & 0b0011111100)>>2;
	txData[2] |= uint8_t(cmd[1] & 0b0000000011)<<6;
	txData[2] |= uint8_t((cmd[1] & 0b1100000000)>>4);

	/* for 3rd servo */
	txData[2] |= uint8_t(cmd[2] & 0b0011110000)>>4;
	txData[3] |= uint8_t(cmd[2] & 0b0000001111)<<4;
	txData[3] |= uint8_t((cmd[2] & 0b1100000000)>>6);

	/* for 4th servo */
	txData[3] |= uint8_t(cmd[3] & 0b0011000000)>>6;
	txData[4] |= uint8_t(cmd[3] & 0b0000111111)<<2;
	txData[4] |= uint8_t((cmd[3] & 0b1100000000)>>8);

	/* for 5th servo */
	// txData[5] |= uint8_t(cmd[4] & 0b0011111111);
	// txData[6] |= uint8_t((cmd[4] & 0b1100000000)>>2);

	/* for tail byte */
	txData[5] = 0b11000000;
	txData[5] |= uint8_t(servo_uavcan_msg_index >> 3);
	servo_uavcan_msg_index += 8;

	/* send servo control msg */
	MW_CAN_TransmitMessage(can_index, &txData[0], SERVO_UAVCAN_CONTROL_DATA_TYPE_ID, 1, 0, 6);
}

void WorkItemExample::set_esc_value(uint8_t can_index, int16_t *cmd, int16_t esc_frq){
	uint8_t esc_msg_data[8] = {0,};
	for (int j = 0; j < 3; j++)
	{
		memset(esc_msg_data, 0, sizeof(esc_msg_data));

		esc_msg_data[0] = uint8_t(cmd[j] & 0xFF);
		esc_msg_data[1] = uint8_t((cmd[j] >> 8) & 0xFF);

		esc_msg_data[4] = uint8_t(esc_frq & 0xFF);
		esc_msg_data[5] = uint8_t((esc_frq >> 8) & 0xFF);

		for(int i=0; i<6 ; i++){
			esc_msg_data[6] += esc_msg_data[i];
		}

		esc_msg_data[7] = ~esc_msg_data[6];

		MW_CAN_TransmitMessage(can_index, &esc_msg_data[0], ESC1_CAN_CONTROL_DATA_TYPE_ID + j, 1, 0, 8);
	}
}


int WorkItemExample::task_spawn(int argc, char *argv[])
{
	WorkItemExample *instance = new WorkItemExample();

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

int WorkItemExample::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int WorkItemExample::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int WorkItemExample::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Example of a simple module running out of a work queue.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("work_item_example", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int work_item_example_main(int argc, char *argv[])
{
	return WorkItemExample::main(argc, argv);
}

#pragma once

#include "../can_driver/MW_PX4_CAN_DEVICE.h"
#include "throttle_dronecan_msg.hpp"
#include "tmotor_dronecan_msg.hpp"

// #include <uavcan/uavcan.hpp>
// #include <uavcan/equipment/esc/RawCommand.hpp>
// #include <uavcan/equipment/esc/Status.hpp>
#include <lib/perf/perf_counter.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/esc_status.h>
#include <drivers/drv_hrt.h>
#include <lib/mixer_module/mixer_module.hpp>

#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/debug_value.h>
#include <uORB/topics/redundancy_detector.h>

class canesc
{

public:
	static constexpr int MAX_ACTUATORS = esc_status_s::CONNECTED_ESC_MAX;
	canesc(MW_H7CAN_DEVICE& h7can_device_ref):
		_h7can_device(h7can_device_ref),
		sinemotion_esc(sinemotion_esc_throttle_signature,8),
		uavcan_esc(uavcan_esc_signature,8)
	{
	}

	~canesc() = default;

	void update_outputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS], unsigned num_outputs);

	int32_t _rotor_num{8};
	uint32_t _throttle_2_id{BROADCAST_THROTTLE_2_ID};
	uint32_t _uavcan_cmd_id{RAW_COMMAND_ID};

	void set_rotor_num(int32_t rotor_num)
	{
		_rotor_num = rotor_num;

		//设置uavcan消息长度
		sinemotion_esc.set_esc_num(_rotor_num);
		uavcan_esc.set_esc_num(_rotor_num);
	}

	uint8_t enable_backup{0};
	/*
	 * 根据local_node_id更新消息id的节点id部分
	*/
	void set_node_id(uint8_t node_id)
	{
		_throttle_2_id |= node_id;
		_uavcan_cmd_id |= node_id;
		/* 如果node id等1，那么本飞控为主飞控，需要直接开启控制输出，即enable_backup = 1*/
		if(node_id == 1)enable_backup = 1;
	}

private:
	MW_H7CAN_DEVICE 		&_h7can_device;

	uint64_t 			sinemotion_esc_throttle_signature{0x1437AC612DC2C691};
	throttle_pwm 			sinemotion_esc;
	uint64_t 			uavcan_esc_signature{0x217F5C87D7EC951D};
	raw_command 			uavcan_esc;

	uint8_t 			_CANModule{0};


	uORB::PublicationMulti<debug_value_s> _debug_pub{ORB_ID(debug_value)};
	//
	uORB::PublicationMulti<redundancy_detector_s> _redundancy_detector_2nd_pub{ORB_ID(redundancy_detector_second)};

	uORB::Subscription		_redundancy_detector_sub{ORB_ID(redundancy_detector)};

	hrt_abstime last_received_timestamp{0};
};


void
canesc::update_outputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS], unsigned num_outputs)
{
	debug_value_s debug_value{};
	debug_value.timestamp = hrt_absolute_time();
	debug_value.value = _rotor_num;
	_debug_pub.publish(debug_value);

	if (_redundancy_detector_sub.updated()) {

		redundancy_detector_s r_detector;
		_redundancy_detector_sub.copy(&r_detector);
		last_received_timestamp = hrt_absolute_time();
	}

	/* 如果检测到超时，且last_received_timestamp = 0, 那么使能输出*/
	redundancy_detector_s r_detector_2nd;
	r_detector_2nd.timestamp = hrt_absolute_time();
	r_detector_2nd.receive_interval = hrt_absolute_time() - last_received_timestamp;
	_redundancy_detector_2nd_pub.publish(r_detector_2nd);
	if (r_detector_2nd.receive_interval >= 30000 && last_received_timestamp != 0)
	{/* 超时30ms以上，认为主飞控失效 */
		enable_backup = 1;
	}

	for(int i=0; i<_rotor_num; i++){
		sinemotion_esc.add_esc_cmd(0x20+i,outputs[i]);
		uavcan_esc.add_esc_cmd(1000);
	}

	uint8_t esc_msg_data[8] = {0,};
	uint8_t len = 0;
	while (!sinemotion_esc.get_package(&esc_msg_data[0], &len))
	{
		// if(enable_backup){/*对于主飞控，enable_backup默认为1，备份飞控默认为0，当备飞控检测到主飞控异常后，enable_backup会变为1 */
		// 	if(_h7can_device.transmitMessage(_CANModule, &esc_msg_data[0], _throttle_2_id, 1, 0, len))
		// 	{/* _CANModule 为0或者1， 如果当前通道发送失败就向另一个通道发送， 即当前的否值， !0 = 1  !1=0*/
		// 		_h7can_device.transmitMessage(!_CANModule, &esc_msg_data[0], _throttle_2_id, 1, 0, len);
		// 	}
		// }
	}
	while(!uavcan_esc.get_package(&esc_msg_data[0], &len))
	{
		if(enable_backup){/*对于主飞控，enable_backup默认为1，备份飞控默认为0，当备飞控检测到主飞控异常后，enable_backup会变为1 */
			if(_h7can_device.transmitMessage(_CANModule, &esc_msg_data[0], _uavcan_cmd_id, 1, 0, len))
			{/* _CANModule 为0或者1， 如果当前通道发送失败就向另一个通道发送， 即当前的否值， !0 = 1  !1=0*/
				_h7can_device.transmitMessage(!_CANModule, &esc_msg_data[0], _uavcan_cmd_id, 1, 0, len);
			}
		}
	}
	sinemotion_esc.clear_esc_cmds();
	uavcan_esc.clear_esc_cmds();
}


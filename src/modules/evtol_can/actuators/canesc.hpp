#pragma once

#include "../can_driver/MW_PX4_CAN_DEVICE.h"
#include "throttle_dronecan_msg.hpp"

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

class canesc
{

public:
	static constexpr int MAX_ACTUATORS = esc_status_s::CONNECTED_ESC_MAX;
	canesc(MW_H7CAN_DEVICE& h7can_device_ref):
		_h7can_device(h7can_device_ref),
		sinemotion_esc(sinemotion_esc_throttle_signature,8)
	{
	}

	~canesc() = default;

	void update_outputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS], unsigned num_outputs);

	int32_t _rotor_num{8};

	void set_rotor_num(int32_t rotor_num)
	{
		_rotor_num = rotor_num;

		//设置uavcan消息长度以及是否需要CRC字段
		sinemotion_esc.set_esc_num(_rotor_num);
	}

private:
	MW_H7CAN_DEVICE 		&_h7can_device;

	uint64_t 			sinemotion_esc_throttle_signature{0x1437AC612DC2C691};
	throttle_pwm 			sinemotion_esc;

	uint8_t 			_CANModule{0};


	uORB::PublicationMulti<debug_value_s> _debug_pub{ORB_ID(debug_value)};

};


void
canesc::update_outputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS], unsigned num_outputs)
{
	debug_value_s debug_value{};
	debug_value.timestamp = hrt_absolute_time();
	debug_value.value = _rotor_num;
	_debug_pub.publish(debug_value);


	for(int i=0; i<_rotor_num; i++){
		sinemotion_esc.add_esc_cmd(0x20+i,outputs[i]);
	}

	uint8_t esc_msg_data[8] = {0,};
	uint8_t len = 0;
	while (!sinemotion_esc.get_package(&esc_msg_data[0], &len))
	{
		_h7can_device.transmitMessage(_CANModule, &esc_msg_data[0], BROADCAST_THROTTLE_2_ID, 1, 0, len);
	}
	sinemotion_esc.clear_esc_cmds();
}


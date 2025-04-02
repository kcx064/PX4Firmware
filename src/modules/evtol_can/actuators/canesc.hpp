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

class canesc
{

public:
	static constexpr int MAX_ACTUATORS = esc_status_s::CONNECTED_ESC_MAX;
	canesc(MW_H7CAN_DEVICE& h7can_device_ref);
	~canesc() = default;

	void update_outputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS], unsigned num_outputs);

private:
	MW_H7CAN_DEVICE 		&_h7can_device;

	uint64_t sinemotion_esc_throttle_signature{0x1437AC612DC2C691};
	throttle_pwm<8> 		sinemotion_esc;

};

canesc::canesc(MW_H7CAN_DEVICE& h7can_device_ref):
	_h7can_device(h7can_device_ref),
	sinemotion_esc(sinemotion_esc_throttle_signature)
{
}

void
canesc::update_outputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS], unsigned num_outputs)
{

	uint8_t esc_msg_data[8] = {0,};
	sinemotion_esc.add_esc_cmd(0x21,outputs[0]);
	sinemotion_esc.add_esc_cmd(0x22,outputs[1]);
	sinemotion_esc.add_esc_cmd(0x23,outputs[2]);
	sinemotion_esc.add_esc_cmd(0x24,outputs[3]);
	sinemotion_esc.add_esc_cmd(0x25,outputs[4]);
	sinemotion_esc.add_esc_cmd(0x26,outputs[5]);
	sinemotion_esc.add_esc_cmd(0x27,outputs[6]);
	sinemotion_esc.add_esc_cmd(0x28,outputs[7]);

	uint8_t len = 0;
	while (!sinemotion_esc.get_package(&esc_msg_data[0], &len))
	{
		_h7can_device.transmitMessage(0, &esc_msg_data[0], BROADCAST_THROTTLE_2_ID, 1, 0, len);
	}
	sinemotion_esc.clear_esc_cmds();
}


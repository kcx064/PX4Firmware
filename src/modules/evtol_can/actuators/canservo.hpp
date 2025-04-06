#pragma once

#include "../can_driver/MW_PX4_CAN_DEVICE.h"
// #include "throttle_dronecan_msg.hpp"

// #include <uavcan/uavcan.hpp>
// #include <uavcan/equipment/esc/RawCommand.hpp>
// #include <uavcan/equipment/esc/Status.hpp>
#include <lib/perf/perf_counter.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/esc_status.h>
#include <drivers/drv_hrt.h>
#include <lib/mixer_module/mixer_module.hpp>

class canservo
{

public:
	static constexpr int MAX_ACTUATORS = 8;
	canservo(MW_H7CAN_DEVICE& h7can_device_ref):
		_h7can_device(h7can_device_ref)
	{
	};
	~canservo() = default;

	void update_outputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS], unsigned num_outputs);

private:
	MW_H7CAN_DEVICE 		&_h7can_device;
};


void
canservo::update_outputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS], unsigned num_outputs)
{

}


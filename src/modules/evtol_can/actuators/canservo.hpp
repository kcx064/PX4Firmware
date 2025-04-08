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

#define CAN_NODE_ID 1U
/* servo can */
#define SERVO_UAVCAN_SOURCE_NODE_ID CAN_NODE_ID
#define SERVO_UAVCAN_DATATYPE_ID 2018U
#define SERVO_UAVCAN_PRIORITY 0x18
#define SERVO_UAVCAN_CONTROL_DATA_TYPE_ID (SERVO_UAVCAN_PRIORITY << 24 | SERVO_UAVCAN_DATATYPE_ID << 8 | SERVO_UAVCAN_SOURCE_NODE_ID)

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
	uint8_t 			servo_uavcan_msg_index{0};
	uint8_t 			_CANModule{1};
};


void
canservo::update_outputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS], unsigned num_outputs)
{
	/* convert cmd data to txData for uavcan servo */
	uint8_t txData[6] ={0,};
	/* for 1st servo */
	txData[0] = uint8_t(outputs[0] & 0b0011111111);
	txData[1] = uint8_t((outputs[0] & 0b1100000000)>>2);

	/* for 2nd servo */
	txData[1] |= uint8_t(outputs[1] & 0b0011111100)>>2;
	txData[2] |= uint8_t(outputs[1] & 0b0000000011)<<6;
	txData[2] |= uint8_t((outputs[1] & 0b1100000000)>>4);

	/* for 3rd servo */
	txData[2] |= uint8_t(outputs[2] & 0b0011110000)>>4;
	txData[3] |= uint8_t(outputs[2] & 0b0000001111)<<4;
	txData[3] |= uint8_t((outputs[2] & 0b1100000000)>>6);

	/* for 4th servo */
	txData[3] |= uint8_t(outputs[3] & 0b0011000000)>>6;
	txData[4] |= uint8_t(outputs[3] & 0b0000111111)<<2;
	txData[4] |= uint8_t((outputs[3] & 0b1100000000)>>8);

	/* for 5th servo */
	// txData[5] |= uint8_t(outputs[4] & 0b0011111111);
	// txData[6] |= uint8_t((outputs[4] & 0b1100000000)>>2);

	/* for tail byte */
	txData[5] = 0b11000000;
	txData[5] |= uint8_t(servo_uavcan_msg_index >> 3);
	servo_uavcan_msg_index += 8;

	/* send servo control msg */
	// _can_servo_ret.timestamp = hrt_absolute_time();
	// _can_servo_ret.send_ret = MW_CAN_TransmitMessage(can_index, &txData[0], SERVO_UAVCAN_CONTROL_DATA_TYPE_ID, 1, 0, 6);
	_h7can_device.transmitMessage(_CANModule, &txData[0], SERVO_UAVCAN_CONTROL_DATA_TYPE_ID, 1, 0, 6);
	// _can_servo_ret_pub.publish(_can_servo_ret);
}


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
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/parameter_update.h>
#include <drivers/drv_hrt.h>
#include <lib/mixer_module/mixer_module.hpp>

#define CAN_NODE_ID 1U
/* servo can */
#define SERVO_UAVCAN_SOURCE_NODE_ID CAN_NODE_ID
#define SERVO_UAVCAN_DATATYPE_ID 2018U
#define SERVO_UAVCAN_PRIORITY 0x18
#define SERVO_UAVCAN_CONTROL_DATA_TYPE_ID (SERVO_UAVCAN_PRIORITY << 24 | SERVO_UAVCAN_DATATYPE_ID << 8 | SERVO_UAVCAN_SOURCE_NODE_ID)

class canservo : public ModuleParams
{

public:
	static constexpr int MAX_ACTUATORS = 8;
	canservo(MW_H7CAN_DEVICE& h7can_device_ref):
		ModuleParams(nullptr),
		_h7can_device(h7can_device_ref)
	{
	};
	~canservo() = default;

	void update_outputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS], unsigned num_outputs);

private:
	MW_H7CAN_DEVICE 		&_h7can_device;
	uint8_t 			servo_uavcan_msg_index{0};
	uint8_t 			_CANModule{1};
	uORB::Subscription              _vehicle_status_sub{ORB_ID(vehicle_status)};
	bool 				_armed{false};
	float_t 			_servo_bias{0};
	int8_t 				servo_check_status{0};
	bool 				enable_servo_check{false};
	int16_t				servo_output[4] = {0,};

	uORB::SubscriptionInterval		_parameter_update_sub{ORB_ID(parameter_update), 1_s};  // subscription limited to 1 Hz updates
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::DB_SRV_CHK>) _param_sv_check
	)
};


void
canservo::update_outputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS], unsigned num_outputs)
{
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams(); // update module parameters (in DEFINE_PARAMETERS)
		enable_servo_check = _param_sv_check.get();
		PX4_INFO("servo check set to %d",enable_servo_check);
	}

	if (_vehicle_status_sub.updated()) {
		vehicle_status_s vehicle_status;

		if (_vehicle_status_sub.copy(&vehicle_status)) {

			const bool armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
			_armed = armed;
		}
	}

	if(enable_servo_check && !_armed){
		if(_servo_bias >= 200)servo_check_status = 1;
		if(_servo_bias <= -200)servo_check_status = 0;

		if(servo_check_status == 0)
		{
			_servo_bias += 0.1f;
		}else{
			_servo_bias -= 0.1f;
		}
	}else{
		_servo_bias = 0;
	}

	servo_output[0] = outputs[0] + _servo_bias;
	servo_output[1] = outputs[1] + _servo_bias;
	servo_output[2] = outputs[2] + _servo_bias;
	servo_output[3] = outputs[3] + _servo_bias;
	// PX4_INFO("servo: %d %d %f", servo_output[0],outputs[0],static_cast<double>(_servo_bias));
	/* convert cmd data to txData for uavcan servo */
	uint8_t txData[6] ={0,};
	/* for 1st servo */
	txData[0] = uint8_t(servo_output[0] & 0b0011111111);
	txData[1] = uint8_t((servo_output[0] & 0b1100000000)>>2);

	/* for 2nd servo */
	txData[1] |= uint8_t(servo_output[1] & 0b0011111100)>>2;
	txData[2] |= uint8_t(servo_output[1] & 0b0000000011)<<6;
	txData[2] |= uint8_t((servo_output[1] & 0b1100000000)>>4);

	/* for 3rd servo */
	txData[2] |= uint8_t(servo_output[2] & 0b0011110000)>>4;
	txData[3] |= uint8_t(servo_output[2] & 0b0000001111)<<4;
	txData[3] |= uint8_t((servo_output[2] & 0b1100000000)>>6);

	/* for 4th servo */
	txData[3] |= uint8_t(servo_output[3] & 0b0011000000)>>6;
	txData[4] |= uint8_t(servo_output[3] & 0b0000111111)<<2;
	txData[4] |= uint8_t((servo_output[3] & 0b1100000000)>>8);

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


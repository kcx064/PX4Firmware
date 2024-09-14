/****************************************************************************
 *
 *   Copyright (c) 2014, 2015 PX4 Development Team. All rights reserved.
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

/**
 * @author Ke Chenxu <kcx064@163.com>
 * ::uavcan::DataTypeSignature signature(0xCA8F4B8F97D23B57);
 */

#include "servoinfo.hpp"

#include <drivers/drv_hrt.h>
#include <systemlib/err.h>

// #include <lib/drivers/magnetometer/PX4Magnetometer.hpp>

const char *const UavcanServoInfoBridge::NAME = "servoinfo";

UavcanServoInfoBridge::UavcanServoInfoBridge(uavcan::INode &node) :
	UavcanSensorBridgeBase("uavcan_servoinfo", ORB_ID(servoinfo)),
	_sub_servoinfo(node)
{
}

int UavcanServoInfoBridge::init()
{
	int res = _sub_servoinfo.start(ServoInfoCbBinder(this, &UavcanServoInfoBridge::servoinfo_sub_cb));

	if (res < 0) {
		PX4_ERR("failed to start uavcan sub: %d", res);
		return res;
	}
	PX4_INFO("servoinfo init success!");
	return 0;
}

void UavcanServoInfoBridge::servoinfo_sub_cb(const
		uavcan::ReceivedDataStructure<com::himark::servo::ServoInfo> &msg)
{
	// uavcan_bridge::Channel *channel = get_channel_for_node(msg.getSrcNodeID().get());

	// if (channel == nullptr) {
	// 	// Something went wrong - no channel to publish on; return
	// 	return;
	// }

	// Cast our generic CDev pointer to the sensor-specific driver class
	// PX4Magnetometer *mag = (PX4Magnetometer *)channel->h_driver;

	// if (mag == nullptr) {
	// 	return;
	// }

	// const float x = msg.magnetic_field_ga[0];
	// const float y = msg.magnetic_field_ga[1];
	// const float z = msg.magnetic_field_ga[2];

	// mag->update(hrt_absolute_time(), x, y, z);

	servoinfo_s report{};
	report.timestamp = hrt_absolute_time();
	report.pwm_input = msg.pwm_input;
	report.pos_cmd = msg.pos_cmd;
	report.pos_sensor = msg.pos_sensor;
	report.voltage = msg.voltage;
	report.current = msg.current;
	report.pcb_temp = msg.pcb_temp;
	report.motor_temp = msg.motor_temp;
	report.servo_id = msg.servo_id;
	report.statusinfo_flags = msg.error_status;

	publish(msg.getSrcNodeID().get(), &report);
}

// int UavcanServoInfoBridge::init_driver(uavcan_bridge::Channel *channel)
// {
// 	// update device id as we now know our device node_id
// 	DeviceId device_id{_device_id};

// 	device_id.devid_s.devtype = DRV_MAG_DEVTYPE_UAVCAN;
// 	device_id.devid_s.address = static_cast<uint8_t>(channel->node_id);

// 	channel->h_driver = new PX4Magnetometer(device_id.devid, ROTATION_NONE);

// 	if (channel->h_driver == nullptr) {
// 		return PX4_ERROR;
// 	}

// 	PX4Magnetometer *mag = (PX4Magnetometer *)channel->h_driver;

// 	channel->instance = mag->get_instance();

// 	if (channel->instance < 0) {
// 		PX4_ERR("UavcanMag: Unable to get an instance");
// 		delete mag;
// 		channel->h_driver = nullptr;
// 		return PX4_ERROR;
// 	}

// 	return PX4_OK;
// }

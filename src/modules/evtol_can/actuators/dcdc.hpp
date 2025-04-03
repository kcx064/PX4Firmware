#pragma once
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/parameter_update.h>

#include "../can_driver/MW_PX4_CAN_DEVICE.h"
#include "CanDeviceInterface.hpp"

class dcdc : public CanDeviceInterface
{
public:
	dcdc(pthread_mutex_t &node_mutex, MW_H7CAN_DEVICE &h7can_device):
	CanDeviceInterface(MODULE_NAME "-dcdc", px4::wq_configurations::test1, node_mutex),
	_h7can_device(h7can_device)
	{
		dcdc_addr = _param_dcdc_addr.get();
		DCDC_POWER_CMD_ID = ((DCDC_POWER_CMD_ID & ~ADDR_MASK) | (static_cast<uint32_t>(dcdc_addr)<<16));
		DCDC_INQUIRE_ID = ((DCDC_INQUIRE_ID & ~ADDR_MASK) | (static_cast<uint32_t>(dcdc_addr)<<16));
	}
	~dcdc();

	bool updateOutputs() override;
private:
	MW_H7CAN_DEVICE 		&_h7can_device;

	uint8_t len = 8;
	uint32_t ADDR_MASK = 0x00FF0000;

    	uint8_t dcdc_addr = 0x00;    		// DCDC power address: 0x01~0xFE, broadcast 0xFF
    	uint8_t SENDER_ADDRESS = 0xE0;       	// Formal host: 0xE0, debug host: 0xD0


    	uint8_t INQUIRE = 0x01;      	 	// Inquire
	uint8_t REMOTE_CONTROL = 0x04;       	// Placeholder for PF value
    	uint8_t POW_ID_SET = 0xA0;       	// Power ID setting

    	uint32_t DCDC_POWER_CMD_ID =
		((0x07u << 24) | (dcdc_addr << 16) | (SENDER_ADDRESS << 8) | REMOTE_CONTROL);
    	uint32_t DCDC_INQUIRE_ID =
		((0x07u << 24) | (dcdc_addr << 16) | (SENDER_ADDRESS << 8) | INQUIRE);
	uint32_t DCDC_SET_ID =
		((0x07u << 24) | (static_cast<uint32_t>(0xFF) << 16) | (SENDER_ADDRESS << 8) | POW_ID_SET);

	//声明一个枚举类型，成员分别表示开机0x01、关机0x02、和复位0x04
	enum POWER_STATE
	{
		WAITE = 0x00,
		POWER_OFF = 0x01,
		POWER_ON = 0x02,
		POWER_RESET = 0x04
	};

	enum POWER_STATE powerState = WAITE;

	uORB::SubscriptionInterval		_parameter_update_sub{ORB_ID(parameter_update), 1_s};  // subscription limited to 1 Hz updates
	// Parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::DCDC_POW>) _param_dcdc_power,
		(ParamInt<px4::params::DCDC_ADDR>) _param_dcdc_addr
	)//最后一行没有逗号

};

dcdc::~dcdc()
{
}

bool
dcdc::updateOutputs()
{
	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams(); // update module parameters (in DEFINE_PARAMETERS)
	}

	uint8_t txData[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	_h7can_device.transmitMessage(0, &txData[0], DCDC_INQUIRE_ID, 1, 0, len);

	switch (powerState)
	{
	case WAITE:
		powerState = static_cast<enum POWER_STATE>(_param_dcdc_power.get());
		break;
	case POWER_OFF:
		txData[0] = POWER_STATE::POWER_OFF;
		break;

	case POWER_ON:
		txData[0] = POWER_STATE::POWER_ON;
		break;

	case POWER_RESET:
		txData[0] = POWER_STATE::POWER_RESET;
		break;

	default:
		powerState = POWER_STATE::WAITE;
		break;
	}

	if(powerState == POWER_STATE::POWER_OFF || powerState == POWER_STATE::POWER_ON || powerState == POWER_STATE::POWER_RESET)
	{
		txData[0] = powerState;
		_h7can_device.transmitMessage(0, &txData[0], DCDC_POWER_CMD_ID, 1, 0, len);
		powerState = POWER_STATE::WAITE;
		_param_dcdc_power.set(POWER_STATE::WAITE);
		_param_dcdc_power.commit();
	}

	if(dcdc_addr != _param_dcdc_addr.get())
	{
		txData[0] = _param_dcdc_addr.get();
		_h7can_device.transmitMessage(0, &txData[0], DCDC_SET_ID, 1, 0, len);

		dcdc_addr = txData[0];
		DCDC_POWER_CMD_ID = ((DCDC_POWER_CMD_ID & ~ADDR_MASK) | (static_cast<uint32_t>(dcdc_addr)<<16));
		DCDC_INQUIRE_ID = ((DCDC_INQUIRE_ID & ~ADDR_MASK) | (static_cast<uint32_t>(dcdc_addr)<<16));
	}

	return true;
}




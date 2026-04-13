#pragma once
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/parameter_update.h>

#include "../can_driver/MW_PX4_CAN_DEVICE.h"
#include "CanDeviceInterface.hpp"
#include "lib_uavcan_packager.hpp"

#ifdef __cplusplus
extern "C" {
#endif
#include <lib/crc/crc.h>
#ifdef __cplusplus
}
#endif

#define PARAM_CFG_PRIORITY 0x18u
#define PARAM_CFG_DATETYPE_ID 1033u
#define PARAM_CFG_LOCALNODE_ID 0x01u
#define PARAM_CFG_ID ((PARAM_CFG_PRIORITY << 24) | (PARAM_CFG_DATETYPE_ID << 8) | PARAM_CFG_LOCALNODE_ID)

#pragma pack(push,1)
typedef union param_cfg
{
	uint8_t buffer[27];
	struct{
		uint8_t current_esc_id; //电调ID
		uint32_t uuid; //电调唯一32位ID
		uint16_t tgt_esc_id;//要设置的ID  current_esc_id变为tgt_esc_id
		uint16_t voltage_protect;
		uint16_t current_protect;
		uint16_t temp_protect;
		uint16_t throttle_acc_limit;
		uint16_t throttle_decent_limit;
		uint16_t rotation_direction;
		uint8_t _angle; //进角设置 1-29  对应 1-29°
		uint8_t priority_and_fixed_propeller; //高四位： 开启定桨 1000 关闭定桨 0000     低四位：PWM 0001  CAN 0010
		uint16_t led;  // 0-2bit:RGB 灯使能开关，3bit:灯闪烁开关，4-15bit:灯闪烁频率(0.1HZ)
		uint8_t bus_rate; // 0-1Mbps 1-500Kbps 2-250Kbps 3-125Kbps 4-100Kbps 5-50Kbps
		uint16_t feedback_rate; // 0-400Hz
		uint8_t param_save; // 0-临时设置 1-永久设置
	};
}param_cfg_u;
#pragma pack(pop)
/* 功能：
 * 1.修改参数特别是ID（仅连接一个esc的时候可以操作）
 */
class esc_paramset : public CanDeviceInterface
{
public:
	esc_paramset(pthread_mutex_t &node_mutex, MW_H7CAN_DEVICE &h7can_device):
	CanDeviceInterface(MODULE_NAME "-esc_paramset", px4::wq_configurations::test1, node_mutex),
	_h7can_device(h7can_device)
	{
		// dcdc_addr = _param_dcdc_addr.get();
	}
	~esc_paramset();

	bool updateOutputs() override;
private:
	#pragma pack(push,1)
	typedef union crc16
	{
		uint16_t crc_val;
		uint8_t crc16_byte[2];
	}crc16_u;

	typedef union{
		uint64_t signature;
		uint8_t buffer[8];
	}signature_u;
	#pragma pack(pop)
	crc16_u _crc16{.crc_val=0};
	MW_H7CAN_DEVICE 		&_h7can_device;

	param_cfg_u paramCfg{.buffer{0xff,}};
	signature_u _signature{.signature{0x948F5E0B33E0EDEE}};
	uavcan_packager _uavcan_packager{paramCfg.buffer, sizeof(paramCfg)};
	uint8_t set_node_id{1};
	uint8_t current_node_id{1};
	uint8_t set_esc_param{0};

	uORB::SubscriptionInterval		_parameter_update_sub{ORB_ID(parameter_update), 1_s};  // subscription limited to 1 Hz updates

	uint8_t 				_CANModule{0};
	// Parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::CUT_NODE_ID>) _param_current_node_id,
		(ParamInt<px4::params::SET_NODE_ID>) _param_set_node_id,
		(ParamInt<px4::params::SET_PARAM>) _param_esc_set
	)//最后一行没有逗号

};

esc_paramset::~esc_paramset()
{
}

bool
esc_paramset::updateOutputs()
{
	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams(); // update module parameters (in DEFINE_PARAMETERS)
		current_node_id = _param_current_node_id.get();
		set_node_id = _param_set_node_id.get();
		set_esc_param = _param_esc_set.get();
	}

	if(set_esc_param)
	{
		paramCfg.current_esc_id = current_node_id;
		paramCfg.tgt_esc_id = set_node_id;
		_crc16.crc_val = crc16_signature(0xFFFF, 8, _signature.buffer);
		_crc16.crc_val = crc16_signature(_crc16.crc_val, sizeof(paramCfg), paramCfg.buffer);

		_uavcan_packager.set_crc(_crc16.crc_val);
		_uavcan_packager.init_packager();

		uint8_t txData[8] = {0,};
		uint8_t len = 0;
		while(!_uavcan_packager.get_package(txData, &len))
		{
			_h7can_device.transmitMessage(_CANModule, &txData[0], PARAM_CFG_ID, 1, 0, len);
			PX4_INFO("Sent parameter package");
		}
		PX4_INFO("Sent Completed");
		_param_esc_set.set(0);
		_param_esc_set.commit();
		set_esc_param = false;
	}

	return true;
}




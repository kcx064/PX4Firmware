#include "CanSensorBridge.hpp"

#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/dcdc_status.h>

#include <lib/systemlib/mavlink_log.h>

namespace AKD202A2871_dcdc
{
	//数据0~1：输出电压
	//数据2~3：输出电流
	//数据4~5：输入电压
	//数据6：温度数据
	//数据7：状态字
	struct dcdc_status_t
	{
		uint16_t output_voltage;
		uint16_t output_current;
		uint16_t input_voltage;
		int8_t temperature;
		uint8_t status_byte;
	};

	union status_u
	{
		dcdc_status_t status_s;
		uint8_t bytes[8];
	};

}



class dcdc_status : public CanSensorBridgeBase
{
public:
	static const char *const NAME;

	dcdc_status(){};

	const char *get_name() const override { return NAME; }

	int init() override;

	void msg_cb(uint8_t canModule, uint32_t msg_id, uint8_t *rxData, uint8_t len) override;

	const uint32_t *get_msg_id() override
	{
		return msg_id_list;
	}

	size_t get_msg_id_num() override
	{
		return MSG_ID_COUNT;
	}

	uint8_t get_can_module() override
	{
		return _CANModule;
	}

	static constexpr uint32_t msg_id_list[] ={
		(0x07E00101),//电池0x01返回状态
		(0x07E00104),//电池0x01返回的遥控指令
	};
	static constexpr size_t MSG_ID_COUNT = sizeof(msg_id_list)/sizeof(msg_id_list[0]);

	dcdc_status_s _dcdc_status{};
	uORB::PublicationMulti<dcdc_status_s> _esc_status_pub{ORB_ID(dcdc_status)};

	orb_advert_t 			_mavlink_log_pub{nullptr};
};

const char *const dcdc_status::NAME = "DCDC_STATUS";
constexpr uint32_t dcdc_status::msg_id_list[];
constexpr size_t dcdc_status::MSG_ID_COUNT;

int dcdc_status::init()
{
	return 0;
}

void dcdc_status::msg_cb(uint8_t canModule, uint32_t msg_id, uint8_t *rxData, uint8_t len)
{
	// perf_count(_count_perf);
	perf_count_interval(_count_perf, 1_s);
	_CANModule = canModule;
	AKD202A2871_dcdc::status_u akd_status;
	if (msg_id == msg_id_list[0])
	{
		memcpy(&akd_status.bytes, rxData, 8);
		_dcdc_status.timestamp = hrt_absolute_time();
		_dcdc_status.output_voltage_v = static_cast<float>(akd_status.status_s.output_voltage) * 0.1f;
		_dcdc_status.output_current_a = static_cast<float>(akd_status.status_s.output_current) * 0.1f;
		_dcdc_status.temperature_c = static_cast<float>(akd_status.status_s.temperature) - 40.0f;
		_dcdc_status.status = akd_status.status_s.status_byte;
		_dcdc_status.input_voltage_v = static_cast<float>(akd_status.status_s.input_voltage) * 0.1f;

		_esc_status_pub.publish(_dcdc_status);
	}

	if(msg_id == msg_id_list[1])
	{
		if(rxData[0] == 0x01)
		mavlink_log_warning(&_mavlink_log_pub, "DC converter 1 shutdown");

		if(rxData[0] == 0x02)
		mavlink_log_warning(&_mavlink_log_pub, "DC converter 1 power on");

		if(rxData[0] == 0x04)
		mavlink_log_warning(&_mavlink_log_pub, "DC converter 1 reset");
	}

}

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
		uint8_t output_voltage_h;
		uint8_t output_voltage_l;
		uint8_t output_current_h;
		uint8_t output_current_l;
		uint8_t input_voltage_h;
		uint8_t input_voltage_l;
		int8_t temperature;
		uint8_t status_byte;
	};

	union status_u
	{
		dcdc_status_t status_s;
		uint8_t bytes[8];
	};

	//定义按位与运算位，bit0表示输入欠压报警，bit1表示输入过压报警，bit2表示过温报警，bit3表示输出过压报警，bit4表示输出过流报警
	constexpr uint8_t STATUS_UNDER_VOLTAGE_INPUT = 1 << 0; 	// 输入欠压报警
	constexpr uint8_t STATUS_OVER_VOLTAGE_INPUT = 1 << 1; 	// 输入过压报警
	constexpr uint8_t STATUS_OVER_TEMPERATURE = 1 << 2; 	// 过温报警
	constexpr uint8_t STATUS_OVER_VOLTAGE_OUTPUT = 1 << 3; 	// 输出过压报警
	constexpr uint8_t STATUS_OVER_CURRENT_OUTPUT = 1 << 4; 	// 输出过流报警


}



class dcdc_status : public CanSensorBridgeBase, public ModuleParams
{
public:
	static const char *const NAME;

	dcdc_status():
		ModuleParams(nullptr)
	{};

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

	void print_status() const override
	{
		printf("channel: %d(can port: %d)\n", _CANModule, (_CANModule+1));
		perf_print_counter(_count_perf);
		printf("can sensor message id list :\n");
		for (size_t i = 0; i < MSG_ID_COUNT; i++)
		{
			printf("[%d]: 0x%08lX \n", i+1, msg_id_list[i]);
		}
	}

	static constexpr uint32_t msg_id_list[] ={
		(0x07E00101),//电池0x01返回状态
		(0x07E00104),//电池0x01返回的遥控指令
		(0x07E00201),//电池0x02返回状态
		(0x07E00204),//电池0x02返回的遥控指令
	};
	static constexpr size_t MSG_ID_COUNT = sizeof(msg_id_list)/sizeof(msg_id_list[0]);

	dcdc_status_s _dcdc_status{};
	uORB::PublicationMulti<dcdc_status_s> _esc_status_pub{ORB_ID(dcdc_status)};
	uORB::SubscriptionInterval		_parameter_update_sub{ORB_ID(parameter_update), 1_s};  // subscription limited to 1 Hz updates


	orb_advert_t 			_mavlink_log_pub{nullptr};

	// Parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::DCDC_ADDR>) _param_dcdc_addr
	)//最后一行没有逗号
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
	perf_count(_count_perf);
	_CANModule = canModule;
	AKD202A2871_dcdc::status_u akd_status;

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams(); // update module parameters (in DEFINE_PARAMETERS)
	}

	uint8_t dcdc_id = (msg_id & 0x0000FF00) >> 8;

	if (msg_id == msg_id_list[0] || msg_id == msg_id_list[2])
	{
		memcpy(&akd_status.bytes, rxData, 8);
		_dcdc_status.timestamp = hrt_absolute_time();
		_dcdc_status.output_voltage_v = static_cast<float>(akd_status.status_s.output_voltage_h << 8 | akd_status.status_s.output_voltage_l) * 0.1f;
		_dcdc_status.output_current_a = static_cast<float>(akd_status.status_s.output_current_h << 8 | akd_status.status_s.output_current_l) * 0.1f;
		_dcdc_status.input_voltage_v = static_cast<float>(akd_status.status_s.input_voltage_h << 8 | akd_status.status_s.input_voltage_l) * 0.1f;
		_dcdc_status.temperature_c = static_cast<float>(akd_status.status_s.temperature) - 40.0f;
		_dcdc_status.status_flags = akd_status.status_s.status_byte;
		_dcdc_status.dcdc_id = dcdc_id;
		_esc_status_pub.publish(_dcdc_status);
		// 根据status_flags状态向地面站发出警报
		if(_dcdc_status.status_flags & AKD202A2871_dcdc::STATUS_UNDER_VOLTAGE_INPUT)
		{
			//输入欠压报警
			mavlink_log_warning(&_mavlink_log_pub, "DC-DC %u undervoltage input", dcdc_id);
		}
		if(_dcdc_status.status_flags & AKD202A2871_dcdc::STATUS_OVER_VOLTAGE_INPUT)
		{
			//输入过压报警
			mavlink_log_warning(&_mavlink_log_pub, "DC-DC %u overvoltage input", dcdc_id);
		}
		if(_dcdc_status.status_flags & AKD202A2871_dcdc::STATUS_OVER_TEMPERATURE)
		{
			//过温报警
			mavlink_log_warning(&_mavlink_log_pub, "DC-DC %u over temperature", dcdc_id);
		}
		if(_dcdc_status.status_flags & AKD202A2871_dcdc::STATUS_OVER_VOLTAGE_OUTPUT)
		{
			//输出过压报警
			mavlink_log_warning(&_mavlink_log_pub, "DC-DC %u overvoltage output", dcdc_id);
		}
		if(_dcdc_status.status_flags & AKD202A2871_dcdc::STATUS_OVER_CURRENT_OUTPUT)
		{
			//输出过流报警
			mavlink_log_warning(&_mavlink_log_pub, "DC-DC %u overcurrent output", dcdc_id);
		}

	}

	if(msg_id == msg_id_list[1] || msg_id == msg_id_list[3])
	{
		if(rxData[0] == 0x01)
		mavlink_log_warning(&_mavlink_log_pub, "DC-DC %u shutdown", dcdc_id);

		if(rxData[0] == 0x02)
		mavlink_log_warning(&_mavlink_log_pub, "DC-DC %u power on", dcdc_id);

		if(rxData[0] == 0x04)
		mavlink_log_warning(&_mavlink_log_pub, "DC-DC %u reset", dcdc_id);
	}

}

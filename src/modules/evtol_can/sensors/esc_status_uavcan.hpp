#include "CanSensorBridge.hpp"

#include <px4_platform_common/defines.h>

#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/esc_status.h>

#include "lib_uavcan_buffer.hpp"

class esc_status_uavcan : public CanSensorBridgeBase, public ModuleParams
{
public:
	static const char *const NAME;

	esc_status_uavcan() :
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
		(0x18040A01),
		(0x18040A01),
		(0x18040A01),//电调1 每个电调状态需要三个连续帧才能发送完毕
		(0x18040A02),
		(0x18040A02),
		(0x18040A02),//电调2
		(0x18040A03),
		(0x18040A03),
		(0x18040A03),//电调3
		(0x18040A04),
		(0x18040A04),
		(0x18040A04),//电调4
	};
	static constexpr size_t MSG_ID_COUNT = sizeof(msg_id_list)/sizeof(msg_id_list[0]);

private:

#pragma pack(push,1)
	typedef union esc_status_msg {
		uint8_t buffer[14] = {0,};
		struct {
			uint32_t error_count;
			uint16_t voltage;
			uint16_t current;
			uint16_t temperature;
			uint8_t buffer1; // int18 rpm | uint7 power_rating_pct | uint5 esc_index
			uint8_t buffer2;
			uint8_t buffer3;
			uint8_t buffer4;
		};
	} esc_status_msg_t;
#pragma pack(pop)
	uint8_t buff_len_max = sizeof(esc_status_msg_t);
	//初始化buffer合成器
	lib_uavcan_buffer _uavcan_buffer[4] = {buff_len_max, buff_len_max, buff_len_max, buff_len_max};

	//用于存储CAN多帧负载组合的完整消息
	esc_status_msg_t _esc_status_msg[4];


	enum sub_state {
		IDLE = 1,
		START_FRAME,//该状态表示**已经**处理过起始帧 而非 当前是起始帧！
	};

	sub_state _sub_state[4] = {sub_state::IDLE,};
	uint8_t buff_len{0};

	esc_status_s					_esc_status{};
	uORB::PublicationMulti<esc_status_s> 		_esc_status_pub{ORB_ID(esc_status)};

	int32_t rotor_num{esc_status_s::CONNECTED_ESC_MAX};
	uORB::SubscriptionInterval	_parameter_update_sub{ORB_ID(parameter_update), 1_s};  // subscription limited to 1 Hz updates
	// Parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::CA_ROTOR_COUNT>) _ca_rotor_count
	)//最后一行没有逗号

	uint8_t check_escs_status();
	float float16_to_float32(uint16_t f);

};

const char *const esc_status_uavcan::NAME = "Template";
constexpr uint32_t esc_status_uavcan::msg_id_list[];
constexpr size_t esc_status_uavcan::MSG_ID_COUNT;

int esc_status_uavcan::init()
{
	return 0;
}

void esc_status_uavcan::msg_cb(uint8_t canModule, uint32_t msg_id, uint8_t *rxData, uint8_t len)
{
	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams(); // update module parameters (in DEFINE_PARAMETERS)

		//获取旋翼-电调数量
		rotor_num = _ca_rotor_count.get();
		PX4_INFO("assume esc number %ld",rotor_num);
	}

	perf_count(_count_perf);
	_CANModule = canModule;

	//确定消息来自于哪个电调
	uint8_t esc_index = 0;
	if((msg_id & 0xFF) == 1)esc_index = 0;
	if((msg_id & 0xFF) == 2)esc_index = 1;
	if((msg_id & 0xFF) == 3)esc_index = 2;
	if((msg_id & 0xFF) == 4)esc_index = 3;

	//用于存储uorb消息数据
	auto &ref = _esc_status.esc[esc_index];

	//方案一 如果接收到结束帧，那么if中返回1则执行if分支，否则不执行
	if(_uavcan_buffer[esc_index].run(_esc_status_msg[esc_index].buffer, rxData, len))
	{
		// 这里数据全部拷贝完毕，位于 _esc_status_msg[esc_index] 中
		ref.timestamp = hrt_absolute_time();
		ref.esc_errorcount  = 0;
		ref.esc_rpm = (_esc_status_msg[esc_index].buffer1) + (_esc_status_msg[esc_index].buffer2 << 8) + ((_esc_status_msg[esc_index].buffer3 & 0b11000000) << 10);
		ref.esc_voltage = static_cast<float_t>(float16_to_float32(_esc_status_msg[esc_index].voltage));
		ref.esc_current = static_cast<float_t>(float16_to_float32(_esc_status_msg[esc_index].current));
		ref.esc_temperature = static_cast<int16_t>(float16_to_float32(_esc_status_msg[esc_index].temperature));

		//每更新一个电调都推送一次，此时意味着其他电调的状态是旧的
		_esc_status.esc_count = rotor_num;
		_esc_status.counter += 1;
		_esc_status.esc_connectiontype = esc_status_s::ESC_CONNECTION_TYPE_CAN;
		_esc_status.esc_online_flags = check_escs_status();
		_esc_status.esc_armed_flags = (1 << rotor_num) - 1;
		_esc_status.timestamp = hrt_absolute_time();
		_esc_status_pub.publish(_esc_status);
	}

	// //方案二
	// switch (_sub_state[esc_index])//这个状态机逻辑可以改编为一个专门的类 与 src/modules/evtol_can/actuators/lib_uavcan_packager.hpp 的目标正好相反
	// {
	// 	case sub_state::IDLE:
	// 		if ( (rxData[len-1] >> 6) == 2 ) { //仅起始帧
	// 			_sub_state[esc_index] = sub_state::START_FRAME;
	// 			memcpy(&_esc_status_msg[esc_index].buffer[buff_len], &rxData[2], 5);
	// 			buff_len = 5;
	// 			// PX4_INFO("pmu start msg");

	// 		} else if ((rxData[len-1] >> 6) == 3) { //起始帧 + 结束帧
	// 			_sub_state[esc_index] = sub_state::IDLE;
	// 			memcpy(&_esc_status_msg[esc_index].buffer[buff_len], &rxData[0], len - 1);
	// 			buff_len = len - 1;
	// 			// PX4_INFO("pmu start_end msg");
	// 		}else{}

	// 		break;

	// 	case sub_state::START_FRAME:
	// 		if ((rxData[len-1] >> 6) == 0) { //中间帧
	// 			// _sub_state[esc_index] = sub_state::START_FRAME;
	// 			if(buff_len+7 <= buff_len_max){
	// 				memcpy(&_esc_status_msg[esc_index].buffer[buff_len], &rxData[0], 7);
	// 				buff_len += 7;
	// 			}
	// 			// PX4_INFO("pmu mid msg");

	// 		} else if ( (rxData[len-1] >> 6) == 1 ) { //仅结束帧
	// 			_sub_state[esc_index] = sub_state::IDLE;
	// 			if(buff_len + len - 1 <= buff_len_max){
	// 				memcpy(&_esc_status_msg[esc_index].buffer[buff_len], &rxData[0], len - 1);
	// 				buff_len += (len - 1);
	// 			}
	// 			// 这里数据全部拷贝完毕，位于 _esc_status_msg[esc_index] 中
	// 			ref.timestamp = hrt_absolute_time();
	// 			ref.esc_errorcount  = 0;
	// 			ref.esc_rpm = (_esc_status_msg[esc_index].buffer1) + (_esc_status_msg[esc_index].buffer2 << 8) + ((_esc_status_msg[esc_index].buffer3 & 0b11000000) << 10);
	// 			ref.esc_voltage = static_cast<float_t>(float16_to_float32(_esc_status_msg[esc_index].voltage));
	// 			ref.esc_current = static_cast<float_t>(float16_to_float32(_esc_status_msg[esc_index].current));
	// 			ref.esc_temperature = static_cast<int16_t>(float16_to_float32(_esc_status_msg[esc_index].temperature));

	// 			//每更新一个电调都推送一次，此时意味着其他电调的状态是旧的
	// 			_esc_status.esc_count = rotor_num;
	// 			_esc_status.counter += 1;
	// 			_esc_status.esc_connectiontype = esc_status_s::ESC_CONNECTION_TYPE_CAN;
	// 			_esc_status.esc_online_flags = check_escs_status();
	// 			_esc_status.esc_armed_flags = (1 << rotor_num) - 1;
	// 			_esc_status.timestamp = hrt_absolute_time();
	// 			_esc_status_pub.publish(_esc_status);

	// 			// PX4_INFO("pmu end msg");

	// 		}else{}
	// 		break;

	// 	default:
	// 		PX4_INFO("default: %d", rxData[len-1] >> 6);
	// 		_sub_state[esc_index] = sub_state::IDLE;
	// 		break;
	// }


}

uint8_t esc_status_uavcan::check_escs_status()
{
	int esc_status_flags = 0;
	const hrt_abstime now = hrt_absolute_time();

	for (int index = 0; index < rotor_num; index++) {

		if (_esc_status.esc[index].timestamp > 0 && now - _esc_status.esc[index].timestamp < 1200_ms) {
			esc_status_flags |= (1 << index);
		}

	}

	return esc_status_flags;
}

float esc_status_uavcan::float16_to_float32(uint16_t f)
{
	uint32_t sign = (f >> 15) & 0x1;
	uint32_t exponent = (f >> 10) & 0x1F;
	uint32_t mantissa = f & 0x3FF;

	// 处理特殊值
	if (exponent == 0x1F) { // 指数全1
		if (mantissa == 0) {
			// 无穷大
			uint32_t result = (sign << 31) | 0x7F800000;
			return *(float *)&result;

		} else {
			// NaN
			uint32_t result = (sign << 31) | 0x7F800000 | (mantissa << 13);
			return *(float *)&result;
		}
	}

	if (exponent == 0) {
		// 处理零和非规格化数 (此处简化，将非规格化数也视为0)
		uint32_t result = (sign << 31);
		return *(float *)&result;
	}

	// 正常转换
	exponent = exponent - 15 + 127; // 调整偏置
	mantissa = mantissa << 13; // 左移13位，低位补0

	uint32_t result = (sign << 31) | (exponent << 23) | mantissa;
	return *(float *)&result;
}

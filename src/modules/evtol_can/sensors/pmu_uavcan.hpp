#include "CanSensorBridge.hpp"

#include <px4_platform_common/defines.h>
#include <battery/battery.h>
#include <uORB/topics/battery_status.h>

#include "lib_uavcan_buffer.hpp"

class pmu_uavcan : public CanSensorBridgeBase, public ModuleParams
{
public:
	static const char *const NAME;

	pmu_uavcan():
		ModuleParams(nullptr),
		_battery(static_cast<int>(1), this, 200_ms, battery_status_s::BATTERY_SOURCE_EXTERNAL)
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
		printf("channel: %d(can port: %d)\n", _CANModule, (_CANModule + 1));
		perf_print_counter(_count_perf);
		printf("can sensor message id list :\n");

		for (size_t i = 0; i < MSG_ID_COUNT; i++) {
			printf("[%d]: 0x%08lX \n", i + 1, msg_id_list[i]);
		}
	}
	//uavcan消息比较特殊，连续帧会一口气发送。参考uavcan himark舵机的接收逻辑，pmu的uavcan消息一组由6个连续帧组成，因此需要一口气注册六个相同id的消息
	static constexpr uint32_t msg_id_list[] = {
		(0x18044433),
		(0x18044433),
		(0x18044433),
		(0x18044433),
		(0x18044433),
		(0x18044433),
	};
	static constexpr size_t MSG_ID_COUNT = sizeof(msg_id_list) / sizeof(msg_id_list[0]);
	Battery _battery;
private:

#pragma pack(push,1)
	typedef union pmu_msg {
		uint8_t buffer[48] = {0,};
		struct {
			uint16_t temperature;
			uint16_t voltage;
			uint16_t current;
			uint16_t average_power_10sec;
			uint16_t remaining_capacity_wh;
			uint16_t full_charge_capacity_wh;
			uint16_t hours_to_full_charge;
		};
	} pmu_msg_t;
#pragma pack(pop)

	pmu_msg_t _pmu_msg;
	uint8_t buff_len{0};
	uint8_t buff_len_max = sizeof(pmu_msg_t);

	//初始化buffer合成器
	lib_uavcan_buffer _uavcan_buffer{buff_len_max};

	enum sub_state {
		IDLE = 1,
		START_FRAME,//该状态表示**已经**处理过起始帧 而非 当前是起始帧！
	};

	sub_state _sub_state = sub_state::IDLE;

	float float16_to_float32(uint16_t f);

};

const char *const pmu_uavcan::NAME = "pmu_uavcan";
constexpr uint32_t pmu_uavcan::msg_id_list[];
constexpr size_t pmu_uavcan::MSG_ID_COUNT;

int pmu_uavcan::init()
{
	return 0;
}

void pmu_uavcan::msg_cb(uint8_t canModule, uint32_t msg_id, uint8_t *rxData, uint8_t len)
{
	perf_count(_count_perf);
	_CANModule = canModule;
	//msg_id_list仅仅考虑一个消息id，这里不需要根据id分类。

	if(_uavcan_buffer.run(_pmu_msg.buffer, rxData, len))
	{
		_battery.setConnected(true);
		_battery.updateVoltage(float16_to_float32(_pmu_msg.voltage));
		_battery.updateCurrent(float16_to_float32(_pmu_msg.current));
		_battery.updateAndPublishBatteryStatus(hrt_absolute_time());
	}
}

float pmu_uavcan::float16_to_float32(uint16_t f)
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

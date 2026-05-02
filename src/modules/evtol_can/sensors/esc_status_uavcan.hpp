#include "CanSensorBridge.hpp"

class esc_status_uavcan : public CanSensorBridgeBase
{
public:
	static const char *const NAME;

	esc_status_uavcan(){};

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
		(0x18040A01),//
		(0x18040A02),
		(0x18040A02),
		(0x18040A02),//
		(0x18040A03),
		(0x18040A03),
		(0x18040A03),//
		(0x18040A04),
		(0x18040A04),
		(0x18040A04),//
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
			uint32_t combination; // int18 rpm | uint7 power_rating_pct | uint5 esc_index
		};
	} esc_status_msg_t;
#pragma pack(pop)
	esc_status_msg_t _esc_status_msg[4];
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
	perf_count(_count_perf);
	_CANModule = canModule;
}

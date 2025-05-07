#include "CanSensorBridge.hpp"

class sensor_template : public CanSensorBridgeBase
{
public:
	static const char *const NAME;

	sensor_template(){};

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
		(0x01),
		(0x02),
		(0x03),
	};
	static constexpr size_t MSG_ID_COUNT = sizeof(msg_id_list)/sizeof(msg_id_list[0]);
};

const char *const sensor_template::NAME = "Template";
constexpr uint32_t sensor_template::msg_id_list[];
constexpr size_t sensor_template::MSG_ID_COUNT;

int sensor_template::init()
{
	return 0;
}

void sensor_template::msg_cb(uint8_t canModule, uint32_t msg_id, uint8_t *rxData, uint8_t len)
{
	perf_count(_count_perf);
	_CANModule = canModule;
}

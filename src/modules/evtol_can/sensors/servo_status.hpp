#include "CanSensorBridge.hpp"

#define SERVO1_CAN_REPORT_DATA_TYPE_ID 0x1807E364
#define SERVO1_CAN_HEARTBEAT_DATA_TYPE_ID 0x18015564

#define SERVO2_CAN_REPORT_DATA_TYPE_ID 0x1807E365
#define SERVO2_CAN_HEARTBEAT_DATA_TYPE_ID 0x18015565

#define SERVO3_CAN_REPORT_DATA_TYPE_ID 0x1807E366
#define SERVO3_CAN_HEARTBEAT_DATA_TYPE_ID 0x18015566

#define SERVO4_CAN_REPORT_DATA_TYPE_ID 0x1807E367
#define SERVO4_CAN_HEARTBEAT_DATA_TYPE_ID 0x18015567

class servo_status : public CanSensorBridgeBase
{
public:
	static const char *const NAME;

	servo_status(){};

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
		(SERVO1_CAN_REPORT_DATA_TYPE_ID),
		(SERVO1_CAN_REPORT_DATA_TYPE_ID),
		(SERVO2_CAN_REPORT_DATA_TYPE_ID),
		(SERVO2_CAN_REPORT_DATA_TYPE_ID),
		(SERVO3_CAN_REPORT_DATA_TYPE_ID),
		(SERVO3_CAN_REPORT_DATA_TYPE_ID),
		(SERVO4_CAN_REPORT_DATA_TYPE_ID),
		(SERVO4_CAN_REPORT_DATA_TYPE_ID),
	};
	static constexpr size_t MSG_ID_COUNT = sizeof(msg_id_list)/sizeof(msg_id_list[0]);
};

const char *const servo_status::NAME = "SERVO_STATUS";
constexpr uint32_t servo_status::msg_id_list[];
constexpr size_t servo_status::MSG_ID_COUNT;

int servo_status::init()
{
	return 0;
}

void servo_status::msg_cb(uint8_t canModule, uint32_t msg_id, uint8_t *rxData, uint8_t len)
{
	// perf_count(_count_perf);
	perf_count_interval(_count_perf, 1_s);
	_CANModule = canModule;
}

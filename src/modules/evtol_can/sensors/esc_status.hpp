#include "CanSensorBridge.hpp"

class esc_status : public CanSensorBridgeBase
{
public:
	static const char *const NAME;

	esc_status(){};

	const char *get_name() const override { return NAME; }

	int init() override;

	void msg_cb(uint32_t msg_id, uint8_t *rxData, uint8_t len) override;

	static constexpr uint32_t msg_id_list[] ={
		(0x01),
		(0x02),
		(0x03),
	};

	const uint32_t *get_msg_id() override
	{
		return msg_id_list;
	}

	size_t get_msg_id_num() override
	{
		return MSG_ID_COUNT;
	}

	static constexpr size_t MSG_ID_COUNT = sizeof(msg_id_list)/sizeof(msg_id_list[0]);
};

const char *const esc_status::NAME = "ESC_STATUS1";
constexpr uint32_t esc_status::msg_id_list[];
constexpr size_t esc_status::MSG_ID_COUNT;

int esc_status::init()
{
	return 0;
}

void esc_status::msg_cb(uint32_t msg_id, uint8_t *rxData, uint8_t len)
{

}

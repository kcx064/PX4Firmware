#include "CanSensorBridge.hpp"

#include <uORB/topics/redundancy_detector.h>

class redundancy_detector : public CanSensorBridgeBase, public ModuleParams
{
public:
	static const char *const NAME;

	redundancy_detector() :
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

	uint32_t msg_id_list[1]={
		// (0x004E2E01)
		(0x00040600)
	};
	uint8_t MSG_ID_COUNT = 1;
private:

	redundancy_detector_s _redundancy_detector{};
	uORB::PublicationMulti<redundancy_detector_s> _redundancy_detector_pub{ORB_ID(redundancy_detector)};

	hrt_abstime last_receive_time{0};
	hrt_abstime receive_interval{0};

	// Parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::EVTOL_NODE_ID>) _evtol_node_id
	)//最后一行没有逗号
};

const char *const redundancy_detector::NAME = "REDUNDANCY_DETECTOR";
// constexpr uint32_t redundancy_detector::msg_id_list[];
// constexpr size_t redundancy_detector::MSG_ID_COUNT;

int redundancy_detector::init()
{
	if(_evtol_node_id.get() == 1)
	{
		//如果节点id==1，本飞控是主飞控，那么需要监听备飞控的控制输出,这里要求备飞控的node id == 2
		msg_id_list[0] |= 0x02;
	}else{
		//否则认为节点id等于2，那么需要监听主飞控的控制输出
		msg_id_list[0] |= 0x01;
	}
	return 0;
}

void redundancy_detector::msg_cb(uint8_t canModule, uint32_t msg_id, uint8_t *rxData, uint8_t len)
{
	perf_count(_count_perf);
	_CANModule = canModule;

	if(msg_id == msg_id_list[0])
	{
		_redundancy_detector.timestamp = hrt_absolute_time();
		_redundancy_detector.receive_interval = hrt_absolute_time() - last_receive_time;
		last_receive_time = _redundancy_detector.timestamp;

		_redundancy_detector_pub.publish(_redundancy_detector);
	}
}

#include "CanSensorBridge.hpp"


#include <uORB/topics/redundancy_detector.h>
#include <stdint.h>
#include "lib_uavcan_parser.hpp"

class redundancy_detector : public CanSensorBridgeBase, public ModuleParams
{
public:
	static const char *const NAME;

	redundancy_detector() :
		ModuleParams(nullptr)
	{
	};

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

	uint32_t msg_id_list[2]={//不同类型电调id不同，这里id的顺序要求与参数 ESC_TYPE 一一对应，该参数定义见src/modules/evtol_can/params.c
		(0x004E2E00), //弦动电调 索引0
		(0x00040600) //uavcan电调 索引1
	};
	uint8_t MSG_ID_COUNT = 2; //与msg_id_list长度匹配
private:
	uint8_t 			esc_type{0};

	//用于sinemotion esc消息解析
	uint8_t 			sine_buffer[30] = {0,};
	uint8_t 			sine_buff_len_max{30}; //要求 sine_buff_len_max 与 sine_buffer 长度匹配
	uavcan_field_info_s 		_sine_field_info[16] = {
						{8,0},
						{16,0},//1
						{8,0},
						{16,0},//2
						{8,0},
						{16,0},//3
						{8,0},
						{16,0},//4
						{8,0},
						{16,0},//5
						{8,0},
						{16,0},//6
						{8,0},
						{16,0},//7
						{8,0},
						{16,0} //8
							};
	uint8_t 			_sine_field_info_size{16}; //要求与_sine_field_info长度匹配
	lib_uavcan_parser 		_sine_uavcan_parser{sine_buffer, sine_buff_len_max, _sine_field_info, _sine_field_info_size};

	//用于uavcan esc消息解析
	uint8_t 			buffer[30] = {0,};
	uint8_t 			buff_len_max{30}; //要求buff_len_max与buffer长度匹配 buffer总bit容量需要大于等于_field_info中bit_width总和
	uavcan_field_info_s 		_field_info[8] = {
						{14,0},
						{14,0},
						{14,0},
						{14,0},
						{14,0},
						{14,0},
						{14,0},
						{14,0}};
	uint8_t 			_field_info_size{8}; //要求与_field_info长度匹配
	lib_uavcan_parser 		_uavcan_parser{buffer, buff_len_max, _field_info, _field_info_size};

	//uorb消息相关定义
	redundancy_detector_s 		_redundancy_detector{};
	uORB::PublicationMulti<redundancy_detector_s> _redundancy_detector_pub{ORB_ID(redundancy_detector)};

	hrt_abstime last_receive_time{0};
	hrt_abstime receive_interval{0};

	// Parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::EVTOL_NODE_ID>) _evtol_node_id,
		(ParamInt<px4::params::ESC_TYPE>) _param_esc_type
	)//最后一行没有逗号
};

const char *const redundancy_detector::NAME = "REDUNDANCY_DETECTOR";
// constexpr uint32_t redundancy_detector::msg_id_list[];
// constexpr size_t redundancy_detector::MSG_ID_COUNT;

int redundancy_detector::init()
{
	esc_type = _param_esc_type.get();
	if(_evtol_node_id.get() == 1)//@todo 注意备飞控id!=2（id!=2就会认为是备飞控）的时候，这里会暂时失效。id!=2的情况仅供调试
	{
		//如果节点id==1，本飞控是主飞控，那么需要监听备飞控的控制输出,这里要求备飞控的node id == 2
		msg_id_list[0] |= 0x02;
		msg_id_list[1] |= 0x02;
	}else{
		//否则认为节点id等于2，那么需要监听主飞控的控制输出
		msg_id_list[0] |= 0x01;
		msg_id_list[1] |= 0x01;
	}
	return 0;
}

void redundancy_detector::msg_cb(uint8_t canModule, uint32_t msg_id, uint8_t *rxData, uint8_t len)
{
	perf_count(_count_perf);
	_CANModule = canModule;

	if(msg_id == msg_id_list[0]) //收到彼飞控发送的匹配弦动电调的油门指令
	{
		if(_sine_uavcan_parser.run(rxData, len))
		{
			_redundancy_detector.raw_command[0] = _sine_field_info[0].field_val;
			_redundancy_detector.raw_command[1] = _sine_field_info[2].field_val;
			_redundancy_detector.raw_command[2] = _sine_field_info[4].field_val;
			_redundancy_detector.raw_command[3] = _sine_field_info[6].field_val;
			_redundancy_detector.raw_command[4] = _sine_field_info[8].field_val;
			_redundancy_detector.raw_command[5] = _sine_field_info[10].field_val;
			_redundancy_detector.raw_command[6] = _sine_field_info[12].field_val;
			_redundancy_detector.raw_command[7] = _sine_field_info[14].field_val;

			//发布消息
			_redundancy_detector.timestamp = hrt_absolute_time();
			_redundancy_detector.receive_interval = hrt_absolute_time() - last_receive_time;
			last_receive_time = _redundancy_detector.timestamp;

			_redundancy_detector_pub.publish(_redundancy_detector);
		}
	}
	if(msg_id == msg_id_list[1]) //收到彼飞控发送的匹配标准uavcan esc的油门指令
	{
		if(_uavcan_parser.run(rxData, len))
		{
			_redundancy_detector.raw_command[0] = _field_info[0].field_val;
			_redundancy_detector.raw_command[1] = _field_info[1].field_val;
			_redundancy_detector.raw_command[2] = _field_info[2].field_val;
			_redundancy_detector.raw_command[3] = _field_info[3].field_val;
			_redundancy_detector.raw_command[4] = _field_info[4].field_val;
			_redundancy_detector.raw_command[5] = _field_info[5].field_val;
			_redundancy_detector.raw_command[6] = _field_info[6].field_val;
			_redundancy_detector.raw_command[7] = _field_info[7].field_val;

			//发布消息
			_redundancy_detector.timestamp = hrt_absolute_time();
			_redundancy_detector.receive_interval = hrt_absolute_time() - last_receive_time;
			last_receive_time = _redundancy_detector.timestamp;

			_redundancy_detector_pub.publish(_redundancy_detector);
		}

	}
}

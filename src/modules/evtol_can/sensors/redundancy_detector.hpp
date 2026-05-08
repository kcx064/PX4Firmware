#include "CanSensorBridge.hpp"


#include <uORB/topics/redundancy_detector.h>
#include <stdint.h>
#include "lib_uavcan_buffer.hpp"

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
		// (0x004E2E01)//弦动电调
		(0x00040600) //uavcan电调
	};
	uint8_t MSG_ID_COUNT = 1;
private:
	//用于消息解析
	uint8_t const raw_cmd_struct[20] = {14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14};

	uint8_t buffer[20];
	uint8_t buff_len_max = sizeof(buffer);

	lib_uavcan_buffer _uavcan_buffer{buff_len_max};
	uint8_t uavcan_get_bit(uint8_t *recv_buffer, uint8_t buffer_len, uint16_t bit_pos);
	uint32_t uavcan_get_value(uint8_t *recv_buffer, uint8_t buffer_len, uint16_t start_bit_pos, uint8_t bit_width);

	typedef union uavcan_field
	{
		uint32_t val;
		uint8_t byte[4];
	}field_u;

	#pragma pack(push, 1)
	typedef struct uavcan_field_info
	{
		uint8_t bit_width;
		uint32_t field_val;
	}uavcan_field_info_s;
	#pragma pack(pop)

	uavcan_field_info_s _field_info[4] = {
		{14,0},
		{14,0},
		{14,0},
		{14,0}
	};

	//uorb消息相关定义
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

		//buffer_len为0 说明完整的连续帧尚未全部收到
		uint8_t buffer_len = _uavcan_buffer.run(buffer, rxData, len);
		if(buffer_len)
		{//读取完毕uavcan连续帧中的消息，存储在buffer中
			uint16_t bit_pos_start = 0;
			for(uint16_t i = 0; i<sizeof(_field_info)/sizeof(uavcan_field_info_s); i++)//TODO 这部分同样整理成为类
			{
				//每次运行下面函数，都能够得到目标数据的值，存储在一个uint32的变量中
				/**
				 * @param buffer 已经保存了消息原始数据的数组，不含CRC和尾字节部分
				 * @param bit_pos_start bit起始位置
				 * @param raw_cmd_struct 当前目标数据的bit宽度，需要用户提前定义好每个数据宽度，以数组的形式提供
				*/
				_field_info[i].field_val = uavcan_get_value(buffer, buffer_len, bit_pos_start, _field_info[i].bit_width);
				bit_pos_start += _field_info[i].bit_width;
			}

		}
		/** @todo 并在另外的地方增加平滑过渡逻辑 */
		_redundancy_detector.raw_command[0] = _field_info[0].field_val;
		_redundancy_detector.raw_command[1] = _field_info[1].field_val;
		_redundancy_detector.raw_command[2] = _field_info[2].field_val;
		_redundancy_detector.raw_command[3] = _field_info[3].field_val;

		//发布消息
		_redundancy_detector.timestamp = hrt_absolute_time();
		_redundancy_detector.receive_interval = hrt_absolute_time() - last_receive_time;
		last_receive_time = _redundancy_detector.timestamp;

		_redundancy_detector_pub.publish(_redundancy_detector);
	}
}

/**
 * @brief 从接收的recv_buffer中提取定义好的消息数据，每次运行获取消息中的一个数据。需要多次运行才能
 * @param recv_buffer 接收到的UAVCAN帧或者UAVCAN连续帧的有效payload，不包含CRC校验和尾字节部分。
 * @param buffer_len recv_buffer的数据长度
 * @param start_bit_pos 目标数据在buffer的起始位置，排列方式为uavcan的数据排列方式
 * 起始 bit 位置-注意这里bit_pos应看作是从第0个字节高位往低位数，接着从第1个字节高位往低位数，依次类推。
 * 这也与数组从左往右从低到高写的习惯一致，也与UAVCAN数据紧凑排列习惯一致。
 * @param bit_width 目标数据在buffer的宽度
*/
uint32_t redundancy_detector::uavcan_get_value(uint8_t *recv_buffer, uint8_t buffer_len, uint16_t start_bit_pos, uint8_t bit_width)
{
	uint8_t sub_i = 0;
	uint8_t byte_index = 0;
	field_u _field{.val{0}};
	for(uint8_t i = 0; i < bit_width; i++)
	{/* 从字节的高位开始逐个bit读取 */
		uint8_t bit_val = uavcan_get_bit(recv_buffer, buffer_len, start_bit_pos+i);//TODO处理返回的值
		//根据i的值确定返回值赋值的逻辑。比方案二的代码更加简洁，结构更加清晰
		if(sub_i>=8)
		{//效果：sub_i的值会按照循环次数在这几个数值间变换 0 1 2 3 4 5 6 7 0 ··· ···
			sub_i -= 8; //因为要切换下一个字节填充，减去8，使得重新重零开始
			byte_index +=1; //字节顺序加一，因为要继续填充高一个地址的字节
		}
		_field.byte[byte_index] = (_field.byte[byte_index] << 1) | bit_val;//这行代码执行8次则填满整个字节
		sub_i++;
	}
	return _field.val;
}

/**
 * @brief 从接收的UAVCAN 字节缓存中获取指定位置的bit值
 * @param recv_buffer 接收到的UAVCAN帧或者UAVCAN连续帧的有效payload，不包含CRC校验和尾字节部分。
 * @param buffer_len recv_buffer的数据长度
 * @param bit_pos 目标bit位置，从0开始。
 * 起始 bit 位置-注意这里bit_pos应看作是从第0个字节高位往低位数，接着从第1个字节高位往低位数，依次类推。
 * 这也与数组从左往右从低到高写的习惯一致，也与UAVCAN数据紧凑排列习惯一致。
 * @return 返回bit_pos对应的bit值: 0或1, 如果出现recv_buffer访问越界，不会返回异常只会返回0
*/
uint8_t redundancy_detector::uavcan_get_bit(uint8_t *recv_buffer, uint8_t buffer_len, uint16_t bit_pos)
{
	uint16_t byte_idx = bit_pos / 8; //从哪个字节开始写入
	/* bit_offset转换为了符合芯片逻辑的字节bit位置，即最低位为0，最高位为7 */
	uint8_t bit_offset = 7 - (bit_pos % 8); //获取哪个字节的哪一位值, bit_pos % 8得到的索引等同于从高位往低位数的索引，因此用7 - bit_pos % 8得到的索引等同于从低位往高位数的索引

	if(byte_idx < buffer_len){
		return (recv_buffer[byte_idx] & (0x01 << bit_offset)) == 0x00 ? 0 : 1;
	}else{
		return 0;
	}


}

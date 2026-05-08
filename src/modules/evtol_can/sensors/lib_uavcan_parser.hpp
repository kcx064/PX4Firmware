#pragma once
#include <cstdint>

#include "lib_uavcan_buffer.hpp"
#include "lib_uavcan_field_extractor.hpp"

#pragma pack(push, 1)
typedef struct uavcan_field_info
{
	uint8_t bit_width;
	uint32_t field_val;
}uavcan_field_info_s;
#pragma pack(pop)

class lib_uavcan_parser
{
private:
	uint8_t *buff;
	uint8_t buff_len_max;
	uavcan_field_info_s *field_info;
	uint8_t field_info_size;  // 记录数组大小


	lib_uavcan_buffer _uavcan_buffer{buff_len_max}; // 获取连续帧的完整payload，需要指定最大长度
	lib_uavcan_field_extractor _uavcan_field_extractor; //获取

public:
	lib_uavcan_parser(uint8_t *buff, uint8_t _buff_len_max, uavcan_field_info_s *_field_info, uint8_t _size);
	~lib_uavcan_parser() = default;
	uint8_t run(uint8_t *rxData, uint8_t len);
};

lib_uavcan_parser::lib_uavcan_parser(uint8_t *_buff, uint8_t _buff_len_max, uavcan_field_info_s *_field_info, uint8_t _size) :
	buff{_buff},
	buff_len_max{_buff_len_max},
	field_info{_field_info},
	field_info_size{_size}
{
}

/**
 * @brief 得到解析后的uavcan定义的数据
 * @return 解析完毕返回1 否则返回0
*/
uint8_t lib_uavcan_parser::run(uint8_t *rxData, uint8_t len)
{
	//buffer_len为0 说明完整的连续帧尚未全部收到
	uint8_t buffer_len = _uavcan_buffer.run(buff, rxData, len);
	if(buffer_len)
	{//读取完毕uavcan连续帧中的消息，存储在buffer中
		uint16_t bit_pos_start = 0;
		for(uint16_t i = 0; i<field_info_size; i++)
		{
			//每次运行下面函数，都能够得到目标数据的值，存储在一个uint32的变量中
			/**
			 * @param buffer 已经保存了消息原始数据的数组，不含CRC和尾字节部分
			 * @param bit_pos_start bit起始位置
			 * @param raw_cmd_struct 当前目标数据的bit宽度，需要用户提前定义好每个数据宽度，以数组的形式提供
			*/
			field_info[i].field_val = _uavcan_field_extractor.uavcan_get_value(buff, buffer_len, bit_pos_start, field_info[i].bit_width);
			bit_pos_start += field_info[i].bit_width;

		}
		return 1;
	}
	return 0;
}

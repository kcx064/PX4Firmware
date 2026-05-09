#pragma once
#include <cstdint>

#include "lib_uavcan_buffer.hpp"
#include "lib_uavcan_field_extractor.hpp"


class lib_uavcan_parser
{
private:
	uint8_t *buff;
	uint8_t buff_len_max;
	uavcan_field_info_s *field_info;
	uint8_t field_info_size;


	lib_uavcan_buffer _uavcan_buffer{buff_len_max}; // 获取连续帧的完整payload，需要指定最大长度
	lib_uavcan_field_extractor _uavcan_field_extractor{field_info, field_info_size};//指定field_info，用于存储解析后的数据

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
		_uavcan_field_extractor.uavcan_get_values(buff, buffer_len);//数据存储在field_info中
		return 1;
	}
	return 0;
}

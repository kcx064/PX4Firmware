#pragma once

#include <cstdint>

//将连续帧的消息重新组合为完整的buffer
class lib_uavcan_buffer
{
private:
	enum sub_state {
		IDLE = 1,
		START_FRAME,//该状态表示**已经**处理过起始帧 而非 当前是起始帧！
	};
	sub_state _sub_state;
	uint8_t buff_len{0};
	uint8_t &buff_len_max;
public:
	lib_uavcan_buffer(uint8_t &_buff_len_max);
	~lib_uavcan_buffer();
	uint8_t run(uint8_t *buffer, uint8_t *rxData, uint8_t len);
};

lib_uavcan_buffer::lib_uavcan_buffer(uint8_t &_buff_len_max) :
	buff_len_max{_buff_len_max}
{
}

lib_uavcan_buffer::~lib_uavcan_buffer()
{
}

uint8_t lib_uavcan_buffer::run(uint8_t *buffer, uint8_t *rxData, uint8_t len)
{
	switch (_sub_state)//这个状态机逻辑 与 src/modules/evtol_can/actuators/lib_uavcan_packager.hpp 的目标正好相反， 用于将连续帧转换为
	{
		case sub_state::IDLE:
			if ( (rxData[len-1] >> 6) == 2 ) { //仅起始帧
				_sub_state = sub_state::START_FRAME;
				memcpy(&buffer[buff_len], &rxData[2], 5);
				buff_len = 5;
				// PX4_INFO("pmu start msg");

			} else if ((rxData[len-1] >> 6) == 3) { //起始帧 + 结束帧
				_sub_state = sub_state::IDLE;
				memcpy(&buffer[buff_len], &rxData[0], len - 1);
				buff_len = len - 1;
				// PX4_INFO("pmu start_end msg");
				return 1;
			}else{}

			break;

		case sub_state::START_FRAME:
			if ((rxData[len-1] >> 6) == 0) { //中间帧
				// _sub_state = sub_state::START_FRAME;
				if(buff_len+7 <= buff_len_max){
					memcpy(&buffer[buff_len], &rxData[0], 7);
					buff_len += 7;
				}
				// PX4_INFO("pmu mid msg");

			} else if ( (rxData[len-1] >> 6) == 1 ) { //仅结束帧
				_sub_state = sub_state::IDLE;
				if(buff_len + len - 1 <= buff_len_max){
					memcpy(&buffer[buff_len], &rxData[0], len - 1);
					buff_len += (len - 1);
				}

				// PX4_INFO("pmu end msg");
				return 1;

			}else{}
			break;

		default:
			// PX4_INFO("default: %d", rxData[len-1] >> 6);
			_sub_state = sub_state::IDLE;
			break;
	}
	return 0;
}

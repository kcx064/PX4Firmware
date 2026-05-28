#pragma once
#include <cstdint>

#pragma pack(push, 1)
typedef struct uavcan_field_info
{
	uint8_t bit_width;
	uint32_t field_val;
}uavcan_field_info_s;
#pragma pack(pop)

class lib_uavcan_field_extractor
{
private:
	/**
	 * 假设uavcan一个数据定义最高32bit, 也是4个字节
	 **/
	typedef union uavcan_field
	{
		uint32_t val;
		uint8_t byte[4];
	}field_u;
	int8_t uavcan_get_bit(uint8_t *recv_buffer, uint8_t buffer_len, uint16_t bit_pos);
	uint32_t uavcan_get_value(uint8_t *recv_buffer, uint8_t buffer_len, uint16_t start_bit_pos, uint8_t bit_width);

	uavcan_field_info_s *field_info;
	uint8_t field_info_size;

public:
	lib_uavcan_field_extractor(uavcan_field_info_s *_field_info, uint8_t _field_info_size);
	~lib_uavcan_field_extractor() = default;
	void uavcan_get_values(uint8_t *buff, uint8_t buffer_len);
};

lib_uavcan_field_extractor::lib_uavcan_field_extractor(uavcan_field_info_s *_field_info, uint8_t _field_info_size) :
	field_info{_field_info},
	field_info_size{_field_info_size}
{}

/**
 * @param buff uavcan连续帧有效数据合并后的数据
 * @param buffer_len buff指向数据的有效长度
 * @result 超过buffer_len对应的bit长度，则提前结束该方法。因为有效数据已经全部处理完毕。数据存储在field_info中
 */
void lib_uavcan_field_extractor::uavcan_get_values(uint8_t *buff, uint8_t buffer_len)
{
	uint16_t bit_pos_start = 0;
	for(uint16_t i = 0; i<field_info_size; i++)
	{
		/** 如果已处理的bit＋即将处理的bit > buffer_len*8的有效bit，说明全部数据提前处理完毕。
		 *  典型情况是: 定义了16组电调，实际只使用了3组，那么会出现有效buffer_len*8小于field_info中所有bit_width的和。
		 *  这里就提前判断，提前结束循环，节省处理器时间
		 */
		if(bit_pos_start + field_info[i].bit_width > buffer_len*8)break;

		//每次运行下面函数，都能够得到目标数据的值，存储在一个uint32的变量中
		/**
		 * @param buff 已经保存了消息原始数据的数组，不含CRC和尾字节部分
		 * @param buffer_len buff指向数据的有效长度
		 * @param bit_pos_start bit起始位置
		 * @param bit_width 当前目标数据的bit宽度，需要用户提前定义好每个数据宽度，以数组的形式提供
		*/
		field_info[i].field_val = uavcan_get_value(buff, buffer_len, bit_pos_start, field_info[i].bit_width);
		bit_pos_start += field_info[i].bit_width;
	}
}

/**
 * @brief 从接收的recv_buffer中提取定义好的消息数据，每次运行获取消息中的一个field_u数据！
 * @param recv_buffer 接收到的UAVCAN帧或者UAVCAN连续帧的有效payload，不包含CRC校验和尾字节部分。
 * @param buffer_len recv_buffer的数据长度
 * @param start_bit_pos 目标数据在buffer的起始位置，排列方式为UAVCAN数据紧凑排列方式
 * 起始 bit 位置-注意这里bit_pos应看作是从第0个字节高位往低位数，接着从第1个字节高位往低位数，依次类推。
 * 这也与数组从左往右从低到高写的习惯一致，也就是UAVCAN数据紧凑排列方式。
 * @param bit_width 目标数据在buffer的宽度
 * @return 返回目标值
*/
uint32_t lib_uavcan_field_extractor::uavcan_get_value(uint8_t *recv_buffer, uint8_t buffer_len, uint16_t start_bit_pos, uint8_t bit_width)
{
	uint8_t sub_i = 0;
	uint8_t byte_index = 0;
	field_u _field{.val{0}};
	for(uint8_t i = 0; i < bit_width; i++)
	{/* 从字节的高位开始逐个bit读取 */
		int8_t bit_val = uavcan_get_bit(recv_buffer, buffer_len, start_bit_pos+i);
		if(bit_val == -1)break; //有效数据提前处理完毕，提前结束循环，return返回值为0
		//根据i的值确定返回值赋值的逻辑。
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
 * @param buffer_len recv_buffer的数据长度，如果 buffer_len 小于 recv_buffer 在内存中开辟的长度，则本方法每次返回0，不会造成越界访问内存
 * @param bit_pos 目标bit位置，从0开始。
 * 起始 bit 位置-注意这里bit_pos应看作是从第0个字节高位往低位数，接着从第1个字节高位往低位数，依次类推。
 * 这也与数组从左往右从低到高写的习惯一致，也与UAVCAN数据紧凑排列习惯一致。
 * @return 返回bit_pos对应的bit值: 0或1, 如果出现recv_buffer被访问越界，返回异常-1。提醒上层调用者.
 *
 * 例如：出于兼容不同数量电调的机型，定义了16组电调油门消息。实际如果是3组电调就会出现buffer_len小于上层调用需要的数据长度
*/
int8_t lib_uavcan_field_extractor::uavcan_get_bit(uint8_t *recv_buffer, uint8_t buffer_len, uint16_t bit_pos)
{
	uint16_t byte_idx = bit_pos / 8; //从哪个字节开始写入
	/* bit_offset转换为了符合芯片逻辑的字节bit位置，即最低位为0，最高位为7 */
	uint8_t bit_offset = 7 - (bit_pos % 8); //获取哪个字节的哪一位值, bit_pos % 8得到的索引等同于从高位往低位数的索引，因此用7 - bit_pos % 8得到的索引等同于从低位往高位数的索引

	if(byte_idx < buffer_len){
		return (recv_buffer[byte_idx] & (0x01 << bit_offset)) == 0x00 ? 0 : 1;
	}else{//超出recv_buffer有效长度，返回-1
		return -1;
	}


}

#pragma once

#include <cstdint>
// #include <containers/Array.hpp>
#include <px4_platform_common/log.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <mathlib/math/Limits.hpp>

#ifdef __cplusplus
extern "C" {
#endif
#include <lib/crc/crc.h>
#ifdef __cplusplus
}
#endif

#define THROTTLE_COUNT	20
#define THROTTLE_BYTE_COUNT 35
#define THROTTLE_14BIT_MAX   ((uint16_t)0x3FFF)  /* 16383 */

/* tmotor esc can */
#define RAW_COMMAND_PRIORITY 0x18u
#define RAW_COMMAND_LOCALNODE_ID 0x00u
#define RAW_COMMAND_DATETYPE_ID 1030
#define RAW_COMMAND_ID ((RAW_COMMAND_PRIORITY << 24) | (RAW_COMMAND_DATETYPE_ID << 8) | RAW_COMMAND_LOCALNODE_ID)

using namespace px4;

#pragma pack(push,1)

typedef struct raw_command_data{
    uint8_t byte_array[THROTTLE_BYTE_COUNT];
} raw_command_data_u;

typedef union signature
{
	uint64_t signature;
	uint8_t buffer[8];
}signature_t;

typedef union crc16
{
	uint16_t crc_val;
	uint8_t crc16_byte[2];
}crc16_u;

#pragma pack(pop)


class raw_command
{
private:
	static constexpr uint8_t CRC_THRESHOLD = 7;
	uint8_t _esc_num{8};
	uint8_t _need_crc{0};
	uint8_t _index{0};
	crc16_u _crc16{.crc_val=0};
	uint8_t thr_index{0};
	raw_command_data_u raw_cmd{.byte_array{0,}};
	uint8_t byte_len{0};

	/* Tail byte的标记位 */
	bool start_of_transfer{false};
	bool end_of_transfer{false};
	uint8_t toggle{0};
	uint8_t transfer_id{0};//width 5bit

	/*已经传输的数据，需要CRC时显示包含CRC在内*/
	uint8_t transferred_data{0};
	/* 剩余应传输的数据*/
	uint8_t remain_byte{0};

	uint64_t get_signature(){
		return _signature.signature;
	}

	uint16_t get_crc(){
		return _crc16.crc_val;
	}

	/**
	 * @brief 设置 35 字节数组中指定 bit 位置的值
	 * @param data    字节数组指针
	 * @param bit_pos 起始 bit 位置 (0-279)
	 * @param value   要写入的值
	 */
	static inline void uavcan_set_bits(uint8_t* data, uint16_t bit_pos, uint16_t value)
	{
		uint16_t inti_mask = 0;
		//下面两步骤可以交换顺序
		//先处理高6位
		inti_mask = 1 << 13;
		for(int i=8; i<14; i++)
		{
			uavcan_set_bit(data, bit_pos+i, value&inti_mask);
			// PX4_INFO_RAW("%d--%d,", bit_pos+8+i, value & inti_mask);
			inti_mask = inti_mask >> 1;
		}

		//再处理低字节8位
		inti_mask=1 << 7;
		for(int i=0; i<8; i++)
		{
			uavcan_set_bit(data, bit_pos+i, value&inti_mask);
			inti_mask = inti_mask >> 1;
		}
	}

	/**
	 * @brief 设置 35 字节数组中指定 bit 位置的值
	 * @param data    字节数组指针
	 * @param bit_pos 起始 bit 位置-注意这里bit_pos应看作是从第0个字节高位往低位数，接着从第1个字节高位往低位数，依次类推。这也与数组从左往右从第到高写的习惯一致，也与uavcan数据紧凑排列习惯一致。
	 * @param bit_val 目标bit值 (0或1)
	 */
	static inline void uavcan_set_bit(uint8_t* data, uint16_t bit_pos, uint16_t bit_val)
	{
		uint16_t byte_idx = bit_pos / 8; //从哪个字节开始写入
		uint8_t bit_offset = 7 - (bit_pos % 8); //从哪个字节的哪一位开始写入, bit_pos % 8得到的索引等同于从高位往低位数，因此用7 - bit_pos % 8得到的索引等同于从低位往高位数

		if(bit_val != 0){//置1
			data[byte_idx] |= static_cast<uint8_t>(1 << bit_offset);
			// PX4_INFO_RAW("|= %d,",static_cast<uint8_t>(bit_offset));
		}else{//置0
			data[byte_idx] &= ~static_cast<uint8_t>(1 << bit_offset);
			// PX4_INFO_RAW("&= %d,",static_cast<uint8_t>(bit_offset));
		}
	}

	/**
	 * @brief 设置指定索引的油门量值
	 */
	int throttle_set(raw_command_data_u* u, uint8_t idx, uint16_t val)
	{
		if (u == NULL || idx >= THROTTLE_COUNT) {
			return -1;
		}

		/* 每个油门量占 14bit，计算起始 bit 位置 */
		uint16_t bit_pos = idx * 14;

		/* 限制值范围在 14bit 内 */
		val &= THROTTLE_14BIT_MAX;

		/* 使用位操作写入 */
		uavcan_set_bits(u->byte_array, bit_pos, val);

		return 0;
	}


	/* 计算有效数据的CRC
	* Reference https://legacy.uavcan.org/Specification/4._CAN_bus_transport_layer/
	* The transfer CRC algorithm is specified as follows:
	* Name: CRC-16-CCITT-FALSE
	* Description: http://reveng.sourceforge.net/crc-catalogue/16.htm#crc.cat.crc-16-ccitt-false
	* Initial value: 0xFFFF
	* Poly: 0x1021
	* Reverse: no
	* Output XOR: 0
	* Check: 0x29B1
	*/
	uint16_t crcAddByte(uint16_t crc_val, uint8_t byte)
	{
		crc_val ^= (uint16_t) ((uint16_t) (byte) << 8);
		for (uint8_t j = 0; j < 8; j++)
		{
			if (crc_val & 0x8000U)
			{
				crc_val = (uint16_t) ((uint16_t) (crc_val << 1) ^ 0x1021U);
			}
			else
			{
				crc_val = (uint16_t) (crc_val << 1);
			}
		}
		return crc_val;
	}

	uint16_t crcAddSignature(uint16_t crc_val, uint64_t data_type_signature)
	{
		for (uint16_t shift_val = 0; shift_val < 64; shift_val = (uint16_t)(shift_val + 8U))
		{
			crc_val = crcAddByte(crc_val, (uint8_t) (data_type_signature >> shift_val));
		}
		return crc_val;
	}

	uint16_t crcAdd(uint16_t crc_val, const uint8_t* bytes, size_t len)
	{
		while (len--)
		{
			crc_val = crcAddByte(crc_val, *bytes++);
		}
		return crc_val;
	}


	// uint16_t cal_uavcan_crc(){
	// 	uint8_t data[sizeof(uint64_t) + RAW_CMD_LENGTH*8];
	// 	/* 添加签名数据 */
	// 	memcpy(data, &_signature_and_buffer.signature, sizeof(uint64_t));
	// 	/* 添加pwm数据，长度等于 单个电调数据长度*电调数量 */
	// 	memcpy(&data[sizeof(uint64_t)], &_signature_and_buffer.buffer[2], RAW_CMD_LENGTH*_esc_num);
	// 	_crc16 = crc16_signature(0xFFFF, 8 + _esc_num * RAW_CMD_LENGTH, data);
	// 	return _crc16;
	// }
public:

	signature_t _signature;

	/**
	 * @brief 初始化 raw_command 类
	 *
	 * 初始化 raw_command 类，根据参数 signature 设置内部状态。
	 *
	 * @param signature 签名值，用于初始化 _signature 成员变量
	 */
	raw_command(uint64_t signature, uint8_t esc_num) :
		_signature{signature}
	{
		_esc_num = esc_num;
	}

	void set_esc_num(uint8_t esc_num)
	{
		_esc_num = esc_num;
	}



	/**
	 * @brief 向 ESC 命令缓冲区添加一个 ESC 命令
	 *
	 * 将给定的 PWM 信号添加到 ESC 命令缓冲区中。
	 *
	 * @param thr PWM 信号宽度，单位为微秒
	 * @return 0 表示成功，1 表示命令缓冲区溢出
	 */
	uint8_t add_esc_cmd(uint16_t thr){

		// raw_cmd.data[thr_index].throttle = thr; //0~8191
		throttle_set(&raw_cmd, thr_index, thr); //0~8191
		thr_index += 1;

		// 油门填充完毕，计算CRC校验
		if(thr_index == _esc_num){

			//计算油门消息一共占用多少字节
			if((thr_index*14)%8 == 0)
			{
				byte_len = (thr_index*14)/8;
			}else{
				//取整
				byte_len = (thr_index*14)/8 + 1;
			}

			// 大于7字节则需要计算 CRC16
			if(byte_len > 7)
			{
				// _crc16.crc_val = crc16_signature(0xFFFF, 8, _signature.buffer);
				// _crc16.crc_val = crc16_signature(_crc16.crc_val, byte_len, raw_cmd.byte_array);

				_crc16.crc_val = crcAddSignature(0xFFFF, _signature.signature);
				_crc16.crc_val = crcAdd(_crc16.crc_val, raw_cmd.byte_array, byte_len);

				_need_crc = 1;
			}

			// 填充完毕设置开始传输标志
			start_of_transfer = true;
			transferred_data = 0;
			// PX4_INFO("raw_cmd byte %x, %x, %x, %x, %x, %x, %x, %x, %x",
			// 	raw_cmd.byte_array[0],
			// 	raw_cmd.byte_array[1],
			// 	raw_cmd.byte_array[2],
			// 	raw_cmd.byte_array[3],
			// 	raw_cmd.byte_array[4],
			// 	raw_cmd.byte_array[5],
			// 	raw_cmd.byte_array[6],
			// 	raw_cmd.byte_array[7],
			// 	raw_cmd.byte_array[8]);
			return 1;
		}

		if(thr_index > _esc_num){
			PX4_WARN("Overflow in ESC commands");
			return 1;
		}
		return 0;
	}

	void clear_esc_cmds(){
		thr_index = 0;
	}

	int8_t get_package(uint8_t *buffer, uint8_t *len){
		if (buffer == nullptr || len == nullptr) {
			return 1;
		}

		if(transferred_data == byte_len)
		{
			PX4_INFO("transferred_data == byte_len");
			return 1;
		}

		if(_need_crc){//需要CRC的时候说明数据包字节数大于7字节，需要多帧, 且计算CRC
			if(start_of_transfer)
			{
				memcpy(buffer, _crc16.crc16_byte, 2); //赋值CRC校验码
				memcpy(&buffer[2], raw_cmd.byte_array, 5);
				transferred_data = 5;

				end_of_transfer = false;
				toggle = false;
				transfer_id += 8;
				buffer[7] = static_cast<uint8_t>(
					(start_of_transfer << 7) |
					(end_of_transfer << 6) |
					(toggle << 5) |
					(transfer_id >> 3)
				);

				*len = 8;
				start_of_transfer = false;
				return 0;//返回0表示数据未全部打包完毕
			}else{//TODO: 开始处理第二帧第三帧...
				remain_byte = byte_len - transferred_data;
				if(remain_byte > 7){//说明不是最后一帧
					memcpy(buffer, &raw_cmd.byte_array[transferred_data], 7);
					transferred_data += 7;


					end_of_transfer = false;
					buffer[7] = static_cast<uint8_t>(
						(start_of_transfer << 7) |
						(end_of_transfer << 6) |
						(toggle << 5) |
						(transfer_id >> 3)
					);
					toggle = !toggle;
					*len = 8;
					return 0;//返回0表示数据未全部打包完毕
				}else{//说明是最后一帧
					memcpy(buffer, &raw_cmd.byte_array[transferred_data], remain_byte);
					transferred_data += remain_byte;

					end_of_transfer = true;

					buffer[remain_byte] = static_cast<uint8_t>(
						(start_of_transfer << 7) |
						(end_of_transfer << 6) |
						(toggle << 5) |
						(transfer_id >> 3)
					);
					toggle = !toggle;
					*len = remain_byte + 1;
					return 0;
				}
			}

		}else{//不需要CRC时候，说明数据包小于等于7字节，那么数据包内容为负载+尾字节
			memcpy(buffer, raw_cmd.byte_array, byte_len);
			transferred_data = byte_len;

			transfer_id += 8;
			end_of_transfer = true;

			buffer[byte_len] = static_cast<uint8_t>(
				(start_of_transfer << 7) |
				(end_of_transfer << 6) |
				(toggle << 5) |
				(transfer_id >> 3)
			);
			toggle = !toggle;
			*len = byte_len + 1;
			start_of_transfer = false;

			return 0;
		}
	}
};

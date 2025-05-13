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

/* sinemotion esc can */
#define PRIORITY 0x00u
#define LOCALNODE_ID 0x01u
#define DATETYPE_ID 0x4E2E
#define BROADCAST_THROTTLE_2_ID ((PRIORITY << 24) | (DATETYPE_ID << 8) | LOCALNODE_ID)

using namespace px4;

#pragma pack(push,1)
typedef union throttle_pwm_data
{
	struct {
		uint8_t esc_id;
		uint16_t pwm_us;//0.1us
	};
	uint8_t raw[sizeof(uint8_t) + sizeof(uint16_t)];
}throttle_pwm_data_t;
#pragma pack(pop)

constexpr uint8_t CMD_LENGTH = sizeof(throttle_pwm_data_t);
constexpr uint8_t buffer_size = CMD_LENGTH*8 + 2; // 预留最多8个电调的数据长度+2个CRC

class throttle_pwm
{
private:
	static_assert(CMD_LENGTH > 0, "CMD_LENGTH must be greater than 0");

	static constexpr uint8_t CRC_THRESHOLD = 7;
	uint8_t _esc_num{8};
	uint8_t _need_crc{1};
	uint8_t _index{0};
	uint16_t _crc16{0};

	/* Tail byte的标记位 */
	bool start_of_transfer{false};
	bool end_of_transfer{false};
	uint8_t toggle{0};
	uint8_t transfer_id{0};//width 5bit

	/*已经传输的数据，需要CRC时显示包含CRC在内*/
	uint8_t transfered_data{0};
	/* 剩余应传输的数据，需要CRC时包含CRC两个字节在内*/
	uint8_t remain_data{0};
public:
	#pragma pack(push,1)
	typedef union signature_and_buffer
	{
		struct {
			uint64_t signature;
			uint8_t buffer[buffer_size];
		};
		uint8_t raw[sizeof(uint64_t) + buffer_size];
	}signature_and_buffer_t;
	#pragma pack(pop)
	signature_and_buffer_t _signature_and_buffer;
	uint8_t used_buffer_size;

	/**
	 * @brief 初始化 throttle_pwm 类
	 *
	 * 初始化 throttle_pwm 类，根据参数 signature 设置内部状态。
	 *
	 * @param signature 签名值，用于初始化 _signature_and_buffer 成员变量
	 */
	throttle_pwm(uint64_t signature, uint8_t esc_num) :
		_signature_and_buffer{signature,}
	{
		used_buffer_size = (_esc_num * CMD_LENGTH > 7) ? _esc_num * CMD_LENGTH + 2 : _esc_num * CMD_LENGTH;
	}

	void set_esc_num(uint8_t esc_num)
	{
		_esc_num = esc_num;
		_need_crc = ((_esc_num * CMD_LENGTH) > CRC_THRESHOLD);
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
	uint16_t cal_uavcan_crc(){
		uint8_t data[sizeof(uint64_t) + CMD_LENGTH*8];
		/* 添加签名数据 */
		memcpy(data, &_signature_and_buffer.signature, sizeof(uint64_t));
		/* 添加pwm数据，长度等于 单个电调数据长度*电调数量 */
		memcpy(&data[sizeof(uint64_t)], &_signature_and_buffer.buffer[2], CMD_LENGTH*_esc_num);
		_crc16 = crc16_signature(0xFFFF, 8 + _esc_num * CMD_LENGTH, data);
		return _crc16;
	}

	/**
	 * @brief 向 ESC 命令缓冲区添加一个 ESC 命令
	 *
	 * 将给定的 ESC ID 和 PWM 信号添加到 ESC 命令缓冲区中。
	 *
	 * @param esc_id ESC 的 ID
	 * @param pwm_us PWM 信号宽度，单位为微秒
	 * @return 0 表示成功，1 表示命令缓冲区溢出
	 */
	uint8_t add_esc_cmd(uint8_t esc_id, uint16_t pwm_us){
		throttle_pwm_data_t pwm_cmd;
		pwm_cmd.esc_id = esc_id;
		pwm_cmd.pwm_us = pwm_us;

		if(_index == _esc_num){
			PX4_WARN("Overflow in ESC commands");
			return 1;
		}

		if(_index < _esc_num){
			memcpy(&_signature_and_buffer.buffer[CMD_LENGTH*_index + _need_crc*2], pwm_cmd.raw, sizeof(throttle_pwm_data_t));
			_index++;
			if(_index==_esc_num){//数据填满则设置开始传输标志
				start_of_transfer = true;
				remain_data = _need_crc*2 + _esc_num * CMD_LENGTH;
				// 如果需要CRC，计算并将CRC写入缓冲区的前两个字节中
				if(_need_crc){
					cal_uavcan_crc();
					_signature_and_buffer.buffer[0] = _crc16 & 0xFF;
					_signature_and_buffer.buffer[1] = _crc16 >> 8;
				}
			}
		}
		return 0;
	}

	// uint16_t get_esc_cmd(uint8_t index){
	// 	return 0;
	// }

	void clear_esc_cmds(){
		_index = 0;
		_crc16 = 0;
		toggle = 0;
		transfered_data = 0;
	}

	uint64_t get_signature(){
		return _signature_and_buffer.signature;
	}

	uint16_t get_crc(){
		return _crc16;
	}

	int8_t get_package(uint8_t *buffer, uint8_t *len){
		if (buffer == nullptr || len == nullptr) {
			return 1;
		}
		if(remain_data ==0)
		{
			return 1;
		}

		constexpr uint8_t DATA_FRAME_SIZE = 7;
		constexpr uint8_t TAILBYTE_SIZE = 1;

		end_of_transfer = (remain_data <= DATA_FRAME_SIZE);
		uint8_t data_to_copy = math::min(remain_data, DATA_FRAME_SIZE);

		if (transfered_data + data_to_copy > sizeof(_signature_and_buffer.buffer)) {
			return 1;
		}

		memcpy(buffer, &_signature_and_buffer.buffer[transfered_data], data_to_copy);

		transfered_data += data_to_copy;
		remain_data -= data_to_copy;

		if (start_of_transfer)transfer_id += 8;
		buffer[data_to_copy] = static_cast<uint8_t>(
			(start_of_transfer << 7) |
			(end_of_transfer << 6) |
			(toggle << 5) |
			(transfer_id >> 3)
		);
		*len = data_to_copy + TAILBYTE_SIZE;

		toggle = !toggle;
		start_of_transfer = false;


		return 0;
	}
};

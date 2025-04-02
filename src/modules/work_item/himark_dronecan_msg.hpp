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


using namespace px4;

#pragma pack(push,1)
typedef union himark_servo_data_u
{
	struct himark_servo_s{
		uint64_t servo0 : 10;
		uint64_t servo1 : 10;
		uint64_t servo2 : 10;
		uint64_t servo3 : 10;
	}data;
	uint8_t raw[5];
}himark_servo_data_t;
#pragma pack(pop)

static_assert(sizeof(himark_servo_data_t) == 5, "Invalid himark_servo_data_t size");

constexpr uint8_t SERVO_CMD_LENGTH = sizeof(himark_servo_data_t::raw);

template <uint8_t N>
struct himark_signature_and_buffer_helper {
	static constexpr uint8_t buffer_size = (N * SERVO_CMD_LENGTH > 7) ? N * SERVO_CMD_LENGTH + 2 : N * SERVO_CMD_LENGTH;
};

template <uint8_t N>
class himark_servo
{
private:

#pragma pack(push,1)
	typedef union signature_and_buffer
	{
		struct {
			uint64_t signature;
			uint8_t buffer[himark_signature_and_buffer_helper<N>::buffer_size];
		};
		uint8_t raw[sizeof(uint64_t) + himark_signature_and_buffer_helper<N>::buffer_size];
	}signature_and_buffer_t;
#pragma pack(pop)

static_assert(SERVO_CMD_LENGTH > 0, "SERVO_CMD_LENGTH must be greater than 0");
static_assert(N>0, "N must be greater than 0");

static constexpr uint8_t CRC_THRESHOLD = 7;
uint8_t _need_crc{0};
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
	signature_and_buffer_t _signature_and_buffer;
	/**
	 * @brief 初始化 himark_servo 类
	 *
	 * 初始化 himark_servo 类，根据参数 signature 设置内部状态。
	 *
	 * @param signature 签名值，用于初始化 _signature_and_buffer 成员变量
	 */
	himark_servo(uint64_t signature) :
	_need_crc((N * SERVO_CMD_LENGTH) > CRC_THRESHOLD),
	_index{0},
	_signature_and_buffer{signature,}
	{
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
		_crc16 = crc16_signature(0xFFFF,N * SERVO_CMD_LENGTH, &_signature_and_buffer.buffer[2]);
		return _crc16;
	}


	uint8_t add_servo_cmd(uint16_t *cmd){
		himark_servo_data_t servo_cmd;
		servo_cmd.data.servo0 = cmd[0];
		servo_cmd.data.servo1 = cmd[1];
		servo_cmd.data.servo2 = cmd[2];
		servo_cmd.data.servo3 = cmd[3];

		if(_index == N){
			PX4_WARN("Overflow in Servo commands");
			return 1;
		}

		if(_index < N){
			memcpy(&_signature_and_buffer.buffer[SERVO_CMD_LENGTH*_index + _need_crc*2], servo_cmd.raw, sizeof(himark_servo_data_t));
			_index++;
			if(_index==N){//数据填满则设置开始传输标志
				start_of_transfer = 1;
				remain_data = _need_crc*2 + N * SERVO_CMD_LENGTH;
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

	void clear_servo_cmds(){
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

		buffer[data_to_copy] = static_cast<uint8_t>(
			(start_of_transfer << 7) |
			(end_of_transfer << 6) |
			(toggle << 5) |
			(transfer_id >> 3)
		);
		*len = data_to_copy + TAILBYTE_SIZE;

		toggle = !toggle;
		start_of_transfer = false;
		transfer_id += 8;

		return 0;
	}
};

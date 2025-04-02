/**
 * 该类用于处理弦动电调状态信息1的解析和封装，
 * 输入为对应帧的原始数据，并根据尾字节判断当前帧是否为最后一帧，如果完成一次接收就发布一次uorb消息
 * 返回解析后的结构体数据
 */
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

#pragma pack(push, 1)
struct sinemotion_dronecan_status3_t
{
    uint8_t esc_index;   // 电调编号
    uint8_t mos_temp;    // mos温度偏置-50
    uint8_t cap_temp;    // 电容温度偏置-50
    uint8_t mcu_temp;    // mcu温度偏置-50
    uint8_t motor_temp;  // 电机温度偏置-50
    uint16_t running_error;  // 运行错误码
    uint8_t tail_byte;
};

static_assert(sizeof(sinemotion_dronecan_status3_t) == 8, "Invalid struct size");

#define SINEMOTION_DRONECAN_STATUS3_SIZE sizeof(sinemotion_dronecan_status3_t)

union sinemotion_dronecan_status3_u
{
	sinemotion_dronecan_status3_t status;
	uint8_t bytes[SINEMOTION_DRONECAN_STATUS3_SIZE];
	static_assert(sizeof(bytes) == sizeof(status), "Union size mismatch");
};
#pragma pack(pop)

template <uint8_t PRI, uint8_t NODE_ID>
class sinemotion_dronecan_status3_decoder
{
private:
	static constexpr uint32_t type_id = 20024;
public:
	uint32_t can_id{0};
	sinemotion_dronecan_status3_u can_data;
	explicit sinemotion_dronecan_status3_decoder()
    		: can_id(static_cast<uint32_t>((static_cast<uint32_t>(PRI) << 24) |
                                 (static_cast<uint32_t>(type_id) << 8) |
                                 static_cast<uint32_t>(NODE_ID)))
	{
		static_assert(PRI <= 0x1F, "Invalid priority value");
		static_assert(NODE_ID <= 0x7F, "Invalid node ID");
	}
	~sinemotion_dronecan_status3_decoder() = default;
	int8_t decode(uint8_t *data, uint8_t len){
		if (len != SINEMOTION_DRONECAN_STATUS3_SIZE) {
			return -1;
		}
		memcpy(can_data.bytes, data, len);
		return 0;
	}

};

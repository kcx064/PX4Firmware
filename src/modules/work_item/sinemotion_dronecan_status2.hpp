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
struct sinemotion_dronecan_status2_t
{
    uint16_t voltge; // 0.1v 母线电压
    int16_t bus_current;  // 0.1A 母线电流
    int16_t current;      // 0.1A 电机线电流
    uint8_t v_modulation; //调制比
    uint8_t tail_byte;
};

static_assert(sizeof(sinemotion_dronecan_status2_t) == 8, "Invalid struct size");

#define SINEMOTION_DRONECAN_STATUS2_SIZE sizeof(sinemotion_dronecan_status2_t)

union sinemotion_dronecan_status2_u
{
	sinemotion_dronecan_status2_t status;
	uint8_t bytes[SINEMOTION_DRONECAN_STATUS2_SIZE];
	static_assert(sizeof(bytes) == sizeof(status), "Union size mismatch");
};
#pragma pack(pop)

template <uint8_t PRI, uint8_t NODE_ID>
class sinemotion_dronecan_status2_decoder
{
private:
	static constexpr uint32_t type_id = 20023;
public:
	uint32_t can_id{0};
	sinemotion_dronecan_status2_u can_data;
	explicit sinemotion_dronecan_status2_decoder()
    		: can_id(static_cast<uint32_t>((static_cast<uint32_t>(PRI) << 24) |
                                 (static_cast<uint32_t>(type_id) << 8) |
                                 static_cast<uint32_t>(NODE_ID)))
	{
		static_assert(PRI <= 0x1F, "Invalid priority value");
		static_assert(NODE_ID <= 0x7F, "Invalid node ID");
	}
	~sinemotion_dronecan_status2_decoder() = default;
	int8_t decode(uint8_t *data, uint8_t len){
		if (len != SINEMOTION_DRONECAN_STATUS2_SIZE) {
			return -1;
		}
		memcpy(can_data.bytes, data, len);
		return 0;
	}

};

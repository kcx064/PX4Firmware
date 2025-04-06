#include "CanSensorBridge.hpp"

#include <lib/perf/perf_counter.h>
#include <lib/systemlib/mavlink_log.h>

#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include "uORB/topics/esc_status.h"

using namespace time_literals;

namespace SinemotionESC {
	constexpr uint8_t Priority = 0x18;
	constexpr uint8_t BaseId = 0x20;
	constexpr uint8_t Count = 8;

	// Status codes for Sinemotion SinemotionESC
	constexpr uint16_t SinemotionStatus1 = 20022;  // Primary status
	constexpr uint16_t SinemotionStatus2 = 20023;  // Secondary status
	constexpr uint16_t SinemotionStatus3 = 20024;  // Tertiary status

	constexpr uint32_t makeStatusId(uint8_t escIndex, uint16_t status) {
	return (static_cast<uint32_t>(Priority) << 24) |
		(static_cast<uint32_t>(status) << 8) |
		static_cast<uint32_t>(BaseId + escIndex);
	}

	template<uint8_t Index>
	struct EscStatusIds {
	static_assert(Index < Count, "SinemotionESC index out of range");
	static constexpr uint32_t Status1 = makeStatusId(Index, SinemotionStatus1);
	static constexpr uint32_t Status2 = makeStatusId(Index, SinemotionStatus2);
	static constexpr uint32_t Status3 = makeStatusId(Index, SinemotionStatus3);
	};

#pragma pack(push, 1)
	struct status1_t
	{
	int32_t speed : 24; // rpm
	uint16_t recv_pwm;  // 0.1us
	uint16_t comm_pwm;  // 0.1us
	uint8_t tail_byte;
	};

	struct status2_t
	{
	uint16_t voltge; // 0.1v 母线电压
	int16_t bus_current;  // 0.1A 母线电流
	int16_t current;      // 0.1A 电机线电流
	uint8_t v_modulation; //调制比
	uint8_t tail_byte;
	};

	struct status3_t
	{
	uint8_t esc_index;   // 电调编号
	uint8_t mos_temp;    // mos温度偏置-50
	uint8_t cap_temp;    // 电容温度偏置-50
	uint8_t mcu_temp;    // mcu温度偏置-50
	uint8_t motor_temp;  // 电机温度偏置-50
	uint16_t running_error;  // 运行错误码
	uint8_t tail_byte;
	};

	union status_u
	{
		status1_t status1;
		status2_t status2;
		status3_t status3;
		uint8_t bytes[8];
	};
#pragma pack(pop)
}

// ESC0-ESC7 status IDs
constexpr uint32_t Esc0Status1 = SinemotionESC::EscStatusIds<0>::Status1;
constexpr uint32_t Esc0Status2 = SinemotionESC::EscStatusIds<0>::Status2;
constexpr uint32_t Esc0Status3 = SinemotionESC::EscStatusIds<0>::Status3;

constexpr uint32_t Esc1Status1 = SinemotionESC::EscStatusIds<1>::Status1;
constexpr uint32_t Esc1Status2 = SinemotionESC::EscStatusIds<1>::Status2;
constexpr uint32_t Esc1Status3 = SinemotionESC::EscStatusIds<1>::Status3;

constexpr uint32_t Esc2Status1 = SinemotionESC::EscStatusIds<2>::Status1;
constexpr uint32_t Esc2Status2 = SinemotionESC::EscStatusIds<2>::Status2;
constexpr uint32_t Esc2Status3 = SinemotionESC::EscStatusIds<2>::Status3;

constexpr uint32_t Esc3Status1 = SinemotionESC::EscStatusIds<3>::Status1;
constexpr uint32_t Esc3Status2 = SinemotionESC::EscStatusIds<3>::Status2;
constexpr uint32_t Esc3Status3 = SinemotionESC::EscStatusIds<3>::Status3;

constexpr uint32_t Esc4Status1 = SinemotionESC::EscStatusIds<4>::Status1;
constexpr uint32_t Esc4Status2 = SinemotionESC::EscStatusIds<4>::Status2;
constexpr uint32_t Esc4Status3 = SinemotionESC::EscStatusIds<4>::Status3;

constexpr uint32_t Esc5Status1 = SinemotionESC::EscStatusIds<5>::Status1;
constexpr uint32_t Esc5Status2 = SinemotionESC::EscStatusIds<5>::Status2;
constexpr uint32_t Esc5Status3 = SinemotionESC::EscStatusIds<5>::Status3;

constexpr uint32_t Esc6Status1 = SinemotionESC::EscStatusIds<6>::Status1;
constexpr uint32_t Esc6Status2 = SinemotionESC::EscStatusIds<6>::Status2;
constexpr uint32_t Esc6Status3 = SinemotionESC::EscStatusIds<6>::Status3;

constexpr uint32_t Esc7Status1 = SinemotionESC::EscStatusIds<7>::Status1;
constexpr uint32_t Esc7Status2 = SinemotionESC::EscStatusIds<7>::Status2;
constexpr uint32_t Esc7Status3 = SinemotionESC::EscStatusIds<7>::Status3;


class esc_status : public CanSensorBridgeBase
{
public:
	static const char *const NAME;

	esc_status(uint8_t CANModule):
		_CANModule(CANModule){};

	const char *get_name() const override { return NAME; }

	int init() override;

	void msg_cb(uint32_t msg_id, uint8_t *rxData, uint8_t len) override;

	uint8_t check_escs_status();

	esc_status_s	_esc_status{};
	uORB::PublicationMulti<esc_status_s> _esc_status_pub{ORB_ID(esc_status)};
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
	uint8_t _CANModule{0};
	static constexpr uint32_t msg_id_list[] ={
		Esc0Status1,
		Esc0Status2,
		Esc0Status3,
		Esc1Status1,
		Esc1Status2,
		Esc1Status3,
		Esc2Status1,
		Esc2Status2,
		Esc2Status3,
		Esc3Status1,
		Esc3Status2,
		Esc3Status3,
		Esc4Status1,
		Esc4Status2,
		Esc4Status3,
		Esc5Status1,
		Esc5Status2,
		Esc5Status3,
		Esc6Status1,
		Esc6Status2,
		Esc6Status3,
		Esc7Status1,
		Esc7Status2,
		Esc7Status3,
	};
	static constexpr size_t MSG_ID_COUNT = sizeof(msg_id_list)/sizeof(msg_id_list[0]);
};

const char *const esc_status::NAME = "ESC_STATUS1";
constexpr uint32_t esc_status::msg_id_list[];
constexpr size_t esc_status::MSG_ID_COUNT;

int esc_status::init()
{
	return 0;
}

void esc_status::msg_cb(uint32_t msg_id, uint8_t *rxData, uint8_t len)
{
	SinemotionESC::status_u status;
	//根据msg_id,将rxData存入相应联合体中
	memcpy(&status, rxData, 8);

	//对uint32_t msg_id取中间两个字节作为uint16_t uavcan_msg_id
	uint16_t uavcan_msg_id = (msg_id >> 8) & 0xFFFF;
	//对uint32_t msg_id取最后一个字节作为uint8_t source_node_id
	uint8_t esc_index = (msg_id & 0xFF) - SinemotionESC::BaseId;

	//TODO 此处有待优化，目前只支持8个ESC, 改为动态适应涵道和混动
	uint8_t _rotor_count = esc_status_s::CONNECTED_ESC_MAX;

	auto &ref = _esc_status.esc[esc_index];

	if (esc_index < esc_status_s::CONNECTED_ESC_MAX)
	{
		ref.timestamp = hrt_absolute_time();
		ref.esc_errorcount  = 0;
		if(uavcan_msg_id == SinemotionESC::SinemotionStatus1)
		{
			// status.status1.comm_pwm;
			// status.status1.recv_pwm;
			ref.esc_rpm = status.status1.speed;
		}

		if(uavcan_msg_id == SinemotionESC::SinemotionStatus2)
		{
			ref.esc_voltage = status.status2.voltge;
			ref.esc_current = status.status2.current;
			// status.status2.bus_current;
			// status.status2.v_modulation;
		}
		if(uavcan_msg_id == SinemotionESC::SinemotionStatus3)
		{
			ref.esc_address = status.status3.esc_index;
			ref.esc_temperature = status.status3.mos_temp;
			// status.status3.cap_temp;
			// status.status3.mcu_temp;
			// status.status3.motor_temp;
			// status.status3.running_error;
		}
	}

	_esc_status.esc_count = _rotor_count;
	_esc_status.counter += 1;
	_esc_status.esc_connectiontype = esc_status_s::ESC_CONNECTION_TYPE_CAN;
	_esc_status.esc_online_flags = check_escs_status();
	_esc_status.esc_armed_flags = (1 << _rotor_count) - 1;
	_esc_status.timestamp = hrt_absolute_time();
	_esc_status_pub.publish(_esc_status);


}

uint8_t esc_status::check_escs_status()
{
	int esc_status_flags = 0;
	const hrt_abstime now = hrt_absolute_time();

	for (int index = 0; index < esc_status_s::CONNECTED_ESC_MAX; index++) {

		if (_esc_status.esc[index].timestamp > 0 && now - _esc_status.esc[index].timestamp < 1200_ms) {
			esc_status_flags |= (1 << index);
		}

	}

	return esc_status_flags;
}

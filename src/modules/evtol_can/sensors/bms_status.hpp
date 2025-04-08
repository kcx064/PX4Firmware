#include "CanSensorBridge.hpp"

#include <uORB/topics/battery_status.h>

/* BMS can */
#define BMS_HCU_INFO_DATA_TYPE_ID  0x186040F3
#define BMS_HCU_ALARM_DATA_TYPE_ID 0x186540F3

#define BMS_HCU_CELLV_DATA_TYPE_ID 0x180050F3
#define BMS_HCU_CELLT_DATA_TYPE_ID 0x185050F3

namespace fullymaxbms
{
#pragma pack(push,1)
	typedef union bms_hcu_info_u
	{
		uint8_t data_raw[8];
		struct bms_hcu_info_s
		{
			// 电池电压，高8位和低8位组合
			uint8_t batVoltage_H : 8;
			uint8_t batVoltage_L : 8;

			// 电池电流，高8位和低8位组合
			uint8_t batCurrent_H : 8;
			uint8_t batCurrent_L : 8;

			// 电池状态量
			uint8_t batSoc : 8; //电池荷电状态
			uint8_t batSoh : 8; //电池健康状态
			uint8_t batAlmLv : 4;//电池警告级别
			uint8_t batState : 4;//电池状态

			// 电池寿命
			uint8_t batLife : 8;
		}data;
	}bms_hcu_info_t;

	typedef union bms_hcu_alarm_u
	{
		uint8_t data_raw[8];
		struct bms_hcu_alarm_s
		{
			//byte 0
			uint8_t alm_cell_ut : 2;
			uint8_t alm_cell_ot : 2;
			uint8_t alm_cell_uv : 2;
			uint8_t alm_cell_ov : 2;
			//byte 1
			uint8_t alm_batt_ov : 2;
			uint8_t alm_batt_dt : 2;
			uint8_t alm_batt_dv : 2;
			uint8_t alm_cell_tbk : 1;
			uint8_t alm_cell_lbk : 1;
			//byte 2
			uint8_t alm_chrg_ocs : 2;
			uint8_t alm_batt_uc : 2;
			uint8_t alm_batt_oc : 2;
			uint8_t alm_batt_uv : 2;
			//byte 3
			uint8_t alm_bsu_offline : 2;
			uint8_t alm_dsch_oct : 2;
			uint8_t alm_chrg_oct : 2;
			uint8_t alm_dsch_ocs : 2;
			//byte 4
			uint8_t alm_vcu_offline : 2;
			uint8_t alm_bmu_fail : 1;
			uint8_t alm_aux_fail : 1;
			uint8_t alm_prechrg_fail : 1;
			uint8_t alm_leak_oc : 2;
			uint8_t alm_bsu_fault : 1;
			//byte 5
			uint8_t reserved_0 : 6;
			uint8_t alm_hvrel_fail : 1;
			uint8_t alm_hall_break : 1;
			//byte 6
			uint8_t reserved_1 : 8;
			//byte 7
			uint8_t reserved_2 : 8;
		}data;
	}bms_hcu_alarm_t;

	typedef union bms_hcu_cellv_u
	{
		uint8_t data_raw[8];
		struct bms_hcu_cellv_s
		{
			uint16_t cellVoltage[4];
		}data;
	}bms_hcu_cellv_t;

	typedef union bms_hcu_cellt_u
	{
		uint8_t data_raw[8];
		struct bms_hcu_cellt_s
		{
			uint16_t cellTemp[8];
		}data;
	}bms_hcu_cellt_t;
#pragma pack(pop)
}

class bms_status : public CanSensorBridgeBase
{
public:
	static const char *const NAME;

	bms_status(){};

	const char *get_name() const override { return NAME; }

	int init() override;

	void msg_cb(uint8_t canModule, uint32_t msg_id, uint8_t *rxData, uint8_t len) override;

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

	static constexpr uint32_t msg_id_list[] ={
		(BMS_HCU_INFO_DATA_TYPE_ID),
	};
	static constexpr size_t MSG_ID_COUNT = sizeof(msg_id_list)/sizeof(msg_id_list[0]);

	uORB::PublicationMulti<battery_status_s> _bms_status_pub{ORB_ID(battery_status)};
	battery_status_s  _can_bms_status{};

	fullymaxbms::bms_hcu_info_t bms_hcu_info;

	const float BMS_VOLTAGE_SCALE = 0.1f;
};

const char *const bms_status::NAME = "BMS_STATUS";
constexpr uint32_t bms_status::msg_id_list[];
constexpr size_t bms_status::MSG_ID_COUNT;

int bms_status::init()
{
	return 0;
}

void bms_status::msg_cb(uint8_t canModule, uint32_t msg_id, uint8_t *rxData, uint8_t len)
{
	perf_count(_count_perf);
	_CANModule = canModule;

	if(msg_id == msg_id_list[0])
	{
		_can_bms_status.timestamp = hrt_absolute_time();
		_can_bms_status.voltage_v = (static_cast<uint16_t>((bms_hcu_info.data.batVoltage_H << 8) | bms_hcu_info.data.batVoltage_L))*BMS_VOLTAGE_SCALE;
		_can_bms_status.voltage_filtered_v = _can_bms_status.voltage_v;

		_can_bms_status.current_a = -((static_cast<float>((bms_hcu_info.data.batCurrent_H << 8) | bms_hcu_info.data.batCurrent_L))*BMS_VOLTAGE_SCALE-1000.0f);//BMS_VOLTAGE_SCALE same as CURRENT_SCALE
		//厂家设置为放电电流为负值，充电电流为正值，且原始数据带有1000A偏置量。因此原始数据乘以电流转换系数，减去1000偏置量，取负值，即为实际电流值
		_can_bms_status.current_filtered_a = _can_bms_status.current_a;
		_can_bms_status.current_average_a = -1;

		_can_bms_status.cell_count = 12;
		_can_bms_status.scale = 1;
		_can_bms_status.voltage_cell_v[0] = _can_bms_status.voltage_v/12;//max: 53.4 equivalent 12cell * 4.45Vmax
		_can_bms_status.voltage_cell_v[1] = _can_bms_status.voltage_v/12;
		_can_bms_status.voltage_cell_v[2] = _can_bms_status.voltage_v/12;
		_can_bms_status.voltage_cell_v[3] = _can_bms_status.voltage_v/12;
		_can_bms_status.voltage_cell_v[4] = _can_bms_status.voltage_v/12;
		_can_bms_status.voltage_cell_v[5] = _can_bms_status.voltage_v/12;
		_can_bms_status.voltage_cell_v[6] = _can_bms_status.voltage_v/12;
		_can_bms_status.voltage_cell_v[7] = _can_bms_status.voltage_v/12;
		_can_bms_status.voltage_cell_v[8] = _can_bms_status.voltage_v/12;
		_can_bms_status.voltage_cell_v[9] = _can_bms_status.voltage_v/12;
		_can_bms_status.voltage_cell_v[10] = _can_bms_status.voltage_v/12;
		_can_bms_status.voltage_cell_v[11] = _can_bms_status.voltage_v/12;
		_can_bms_status.remaining = (_can_bms_status.voltage_v - 504.0f)/(640.8f - 504.0f);
		_can_bms_status.id = 4;
		_can_bms_status.temperature = NAN;
		_can_bms_status.time_remaining_s = NAN;
		_can_bms_status.connected = true;
		//144cells, MAX 4.45V/cell
		if(_can_bms_status.voltage_v < 640.8f)_can_bms_status.warning = battery_status_s::BATTERY_WARNING_NONE;//4.45
		if(_can_bms_status.voltage_v < 561.6f)_can_bms_status.warning = battery_status_s::BATTERY_WARNING_NONE;//3.9
		if(_can_bms_status.voltage_v < 547.2f)_can_bms_status.warning = battery_status_s::BATTERY_WARNING_NONE;//3.8
		if(_can_bms_status.voltage_v < 532.8f)_can_bms_status.warning = battery_status_s::BATTERY_WARNING_LOW;//3.7
		if(_can_bms_status.voltage_v < 504.0f)_can_bms_status.warning = battery_status_s::BATTERY_WARNING_EMERGENCY;//3.5

		_bms_status_pub.publish(_can_bms_status);
	}
}

#include "CanSensorBridge.hpp"

#include <px4_platform_common/defines.h>
#include <battery/battery.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/fullymax_battery_status.h>
#include <uORB/topics/fullymax_battery_alarm.h>

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
			uint8_t batVoltage_H;
			uint8_t batVoltage_L;

			// 电池电流，高8位和低8位组合
			uint8_t batCurrent_H;
			uint8_t batCurrent_L;

			// 电池状态量
			uint8_t batSoc; //电池荷电状态
			uint8_t batSoh; //电池健康状态
			uint8_t batAlmLv : 4;//电池警告级别
			uint8_t batState : 4;//电池状态

			// 电池寿命
			uint8_t batLife;
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
			uint8_t alm_hall_break : 1;
			uint8_t alm_hvrel_fail : 1;
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

class bms_status : public CanSensorBridgeBase, public ModuleParams
{
public:
	static const char *const NAME;

	bms_status():
		ModuleParams(nullptr),
		_battery(static_cast<int>(2), this, 500_ms, battery_status_s::BATTERY_SOURCE_EXTERNAL)
	{};

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

	void print_status() const override
	{
		printf("channel: %d(can port: %d)\n", _CANModule, (_CANModule+1));
		perf_print_counter(_count_perf);
		printf("can sensor message id list :\n");
		for (size_t i = 0; i < MSG_ID_COUNT; i++)
		{
			printf("[%d]: 0x%08lX \n", i+1, msg_id_list[i]);
		}
	}

	static constexpr uint32_t msg_id_list[] ={
		(BMS_HCU_INFO_DATA_TYPE_ID),
		(BMS_HCU_ALARM_DATA_TYPE_ID),
	};
	static constexpr size_t MSG_ID_COUNT = sizeof(msg_id_list)/sizeof(msg_id_list[0]);

	// uORB::PublicationMulti<battery_status_s> _bms_status_pub{ORB_ID(battery_status)};
	// battery_status_s  _can_bms_status{};

	fullymax_battery_status_s _fullymax_status{};
	uORB::PublicationMulti<fullymax_battery_status_s> _fullymax_status_pub{ORB_ID(fullymax_battery_status)};

	fullymax_battery_alarm_s _fullymax_alarm{};
	uORB::PublicationMulti<fullymax_battery_alarm_s> _fullymax_alarm_pub{ORB_ID(fullymax_battery_alarm)};

	fullymaxbms::bms_hcu_info_t bms_hcu_info;
	fullymaxbms::bms_hcu_alarm_t bms_hcu_alarm;

	const float BMS_VOLTAGE_SCALE = 0.1f;

	Battery _battery;
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
		memcpy(&bms_hcu_info.data_raw, rxData, 8);


		float_t bms_voltage = (static_cast<uint16_t>((bms_hcu_info.data.batVoltage_H << 8) | bms_hcu_info.data.batVoltage_L))*BMS_VOLTAGE_SCALE;
		float_t bms_current = -((static_cast<float>((bms_hcu_info.data.batCurrent_H << 8) | bms_hcu_info.data.batCurrent_L))*BMS_VOLTAGE_SCALE - 1000.0f);
		_battery.setConnected(true);
		_battery.updateVoltage(bms_voltage);
		_battery.updateCurrent(bms_current);
		_battery.updateAndPublishBatteryStatus(hrt_absolute_time());

		// 将剩余数据发布到自定义消息中
		_fullymax_status.timestamp = hrt_absolute_time();
		_fullymax_status.batsoc = bms_hcu_info.data.batSoc;
		_fullymax_status.batsoh = bms_hcu_info.data.batSoh;
		_fullymax_status.batalmlv = bms_hcu_info.data.batAlmLv;
		_fullymax_status.batstate = bms_hcu_info.data.batState;
		_fullymax_status.batlife = bms_hcu_info.data.batLife;
		_fullymax_status_pub.publish(_fullymax_status);

	}
	if(msg_id == msg_id_list[1])
	{
		memcpy(&bms_hcu_alarm.data_raw, rxData, 8);

		_fullymax_alarm.timestamp = hrt_absolute_time();
		_fullymax_alarm.alm_cell_ov = bms_hcu_alarm.data.alm_cell_ov;
		_fullymax_alarm.alm_cell_uv = bms_hcu_alarm.data.alm_cell_uv;
		_fullymax_alarm.alm_cell_ot = bms_hcu_alarm.data.alm_cell_ot;
		_fullymax_alarm.alm_cell_ut = bms_hcu_alarm.data.alm_cell_ut;
		_fullymax_alarm.alm_cell_lbk = bms_hcu_alarm.data.alm_cell_lbk;
		_fullymax_alarm.alm_cell_tbk = bms_hcu_alarm.data.alm_cell_tbk;
		_fullymax_alarm.alm_batt_dv = bms_hcu_alarm.data.alm_batt_dv;
		_fullymax_alarm.alm_batt_dt = bms_hcu_alarm.data.alm_batt_dt;
		_fullymax_alarm.alm_batt_ov = bms_hcu_alarm.data.alm_batt_ov;
		_fullymax_alarm.alm_batt_uv = bms_hcu_alarm.data.alm_batt_uv;
		_fullymax_alarm.alm_batt_oc = bms_hcu_alarm.data.alm_batt_oc;
		_fullymax_alarm.alm_batt_uc = bms_hcu_alarm.data.alm_batt_uc;
		_fullymax_alarm.alm_chrg_ocs = bms_hcu_alarm.data.alm_chrg_ocs;
		_fullymax_alarm.alm_dsch_ocs = bms_hcu_alarm.data.alm_dsch_ocs;
		_fullymax_alarm.alm_chrg_oct = bms_hcu_alarm.data.alm_chrg_oct;
		_fullymax_alarm.alm_dsch_oct = bms_hcu_alarm.data.alm_dsch_oct;
		_fullymax_alarm.alm_bsu_offline = bms_hcu_alarm.data.alm_bsu_offline;
		_fullymax_alarm.alm_bsu_fault = bms_hcu_alarm.data.alm_bsu_fault;
		_fullymax_alarm.alm_leak_oc = bms_hcu_alarm.data.alm_leak_oc;
		_fullymax_alarm.alm_prechrg_fail = bms_hcu_alarm.data.alm_prechrg_fail;
		_fullymax_alarm.alm_aux_fail = bms_hcu_alarm.data.alm_aux_fail;
		_fullymax_alarm.alm_bmu_fail = bms_hcu_alarm.data.alm_bmu_fail;
		_fullymax_alarm.alm_vcu_offline = bms_hcu_alarm.data.alm_vcu_offline;
		_fullymax_alarm.alm_hvrel_fail = bms_hcu_alarm.data.alm_hvrel_fail;
		_fullymax_alarm.alm_hall_break = bms_hcu_alarm.data.alm_hall_break;
		_fullymax_alarm_pub.publish(_fullymax_alarm);
	}
}

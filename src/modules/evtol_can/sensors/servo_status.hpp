#include "CanSensorBridge.hpp"

#include <battery/battery.h>
#include <uORB/topics/servoinfo.h>
#include <uORB/topics/battery_status.h>

#define SERVO1_CAN_REPORT_DATA_TYPE_ID 0x1807E364
#define SERVO1_CAN_HEARTBEAT_DATA_TYPE_ID 0x18015564

#define SERVO2_CAN_REPORT_DATA_TYPE_ID 0x1807E365
#define SERVO2_CAN_HEARTBEAT_DATA_TYPE_ID 0x18015565

#define SERVO3_CAN_REPORT_DATA_TYPE_ID 0x1807E366
#define SERVO3_CAN_HEARTBEAT_DATA_TYPE_ID 0x18015566

#define SERVO4_CAN_REPORT_DATA_TYPE_ID 0x1807E367
#define SERVO4_CAN_HEARTBEAT_DATA_TYPE_ID 0x18015567

using namespace time_literals;

namespace himark_servo
{
	/*used for servoinfo report decode*/
	enum decode_servoinfo_state
	{
		PACK_H = 0,
		PACK_L,
	};
	uint8_t const servoinfo_uavcan_struct[9] = {5, 12, 16, 16, 12, 10, 10, 10, 5};

	typedef struct servo_decode_s{
		decode_servoinfo_state servoinfo_state = decode_servoinfo_state::PACK_H;
		uint8_t msg_SN{0};
		uint16_t servoinfo_raw_data[9] = {0};
		uint64_t receivePack = 0;
	}servo_decode_t;
}

class servo_status : public CanSensorBridgeBase, public ModuleParams
{
public:
	static const char *const NAME;

	servo_status():
		ModuleParams(nullptr),
		_battery(static_cast<int>(1), this, 500_ms, battery_status_s::BATTERY_SOURCE_POWER_MODULE)
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
		(SERVO1_CAN_REPORT_DATA_TYPE_ID),
		(SERVO1_CAN_REPORT_DATA_TYPE_ID),
		(SERVO2_CAN_REPORT_DATA_TYPE_ID),
		(SERVO2_CAN_REPORT_DATA_TYPE_ID),
		(SERVO3_CAN_REPORT_DATA_TYPE_ID),
		(SERVO3_CAN_REPORT_DATA_TYPE_ID),
		(SERVO4_CAN_REPORT_DATA_TYPE_ID),
		(SERVO4_CAN_REPORT_DATA_TYPE_ID),
	};
	static constexpr size_t MSG_ID_COUNT = sizeof(msg_id_list)/sizeof(msg_id_list[0]);

	himark_servo::servo_decode_t servo_decode_state[4];

	int servoinfo_instance[4];
	orb_advert_t _servoinfo_sub[4];
	servoinfo_s servo_report[4]{};

	// uORB::PublicationMulti<battery_status_s> _LV_status_pub{ORB_ID(battery_status)};
	// battery_status_s  _LV_status{};

	Battery _battery;
};

const char *const servo_status::NAME = "SERVO_STATUS";
constexpr uint32_t servo_status::msg_id_list[];
constexpr size_t servo_status::MSG_ID_COUNT;

int servo_status::init()
{
	/* servo report */
	_servoinfo_sub[0] = orb_advertise_multi(ORB_ID(servoinfo), &servo_report[0], &servoinfo_instance[0]);
	_servoinfo_sub[1] = orb_advertise_multi(ORB_ID(servoinfo), &servo_report[1], &servoinfo_instance[1]);
	_servoinfo_sub[2] = orb_advertise_multi(ORB_ID(servoinfo), &servo_report[2], &servoinfo_instance[2]);
	_servoinfo_sub[3] = orb_advertise_multi(ORB_ID(servoinfo), &servo_report[3], &servoinfo_instance[3]);
	return 0;
}

void servo_status::msg_cb(uint8_t canModule, uint32_t msg_id, uint8_t *rxData, uint8_t len)
{
	perf_count(_count_perf);
	_CANModule = canModule;

	uint8_t servo_index = 0;
	if(msg_id == SERVO1_CAN_REPORT_DATA_TYPE_ID){servo_index = 0;}
	if(msg_id == SERVO2_CAN_REPORT_DATA_TYPE_ID){servo_index = 1;}
	if(msg_id == SERVO3_CAN_REPORT_DATA_TYPE_ID){servo_index = 2;}
	if(msg_id == SERVO4_CAN_REPORT_DATA_TYPE_ID){servo_index = 3;}

	// PX4_WARN("receive data start %d",servo_index);
	switch (servo_decode_state[servo_index].servoinfo_state)
	{
	case himark_servo::PACK_H:
		if(rxData[7] & 0x80){
			servo_decode_state[servo_index].servoinfo_state = himark_servo::decode_servoinfo_state::PACK_L;
			servo_decode_state[servo_index].msg_SN = rxData[7] & 0x1F;
			//低字节放置到高位
			servo_decode_state[servo_index].receivePack = 0;
			servo_decode_state[servo_index].receivePack = (uint64_t)rxData[2] << 8*7 | (uint64_t)rxData[3] << 8*6 | (uint64_t)rxData[4] << 8*5 | (uint64_t)rxData[5] << 8*4 | (uint64_t)rxData[6] << 8*3;
			// PX4_INFO("received data 1 is %llX",servo_decode_state[servo_index].receivePack);
		}
		break;

	case himark_servo::PACK_L:
		if( (rxData[7] & 0x40) && ((rxData[7] & 0x1F) == servo_decode_state[servo_index].msg_SN) ){
			//TODO: 增加签名校验部分
			//第一阶段
			servo_decode_state[servo_index].receivePack = servo_decode_state[servo_index].receivePack | (uint64_t)rxData[0] << 8*2 | (uint64_t)rxData[1] << 8*1 | (uint64_t)rxData[2];
			// PX4_INFO("received data 2 is %llX",servo_decode_state[servo_index].receivePack);
			for (uint8_t i = 0; i < 5; i++)
			{
				if(himark_servo::servoinfo_uavcan_struct[i] <= 8){//目标数据小于等于8位，那么一次提取即可
					//提取数据
					servo_decode_state[servo_index].servoinfo_raw_data[i] = servo_decode_state[servo_index].receivePack >> (64 - himark_servo::servoinfo_uavcan_struct[i]);
					//左移，去除已经提取的数据
					servo_decode_state[servo_index].receivePack = servo_decode_state[servo_index].receivePack << himark_servo::servoinfo_uavcan_struct[i];
				}else{//目标数据大于8位，根据厂家给出的定义，消息字段最长有16位，那么二次提取即可
					//提取低八位数据
					servo_decode_state[servo_index].servoinfo_raw_data[i] = servo_decode_state[servo_index].receivePack >> (64 - 8);
					//左移，去除已经提取的低八位数据
					servo_decode_state[servo_index].receivePack = servo_decode_state[servo_index].receivePack << 8;

					//提取高位数据，可能不足八位
					servo_decode_state[servo_index].servoinfo_raw_data[i] = servo_decode_state[servo_index].servoinfo_raw_data[i] | (( servo_decode_state[servo_index].receivePack >> (64 - (himark_servo::servoinfo_uavcan_struct[i] - 8)) ) << 8);
					//左移，去除已经提取的数据
					servo_decode_state[servo_index].receivePack = servo_decode_state[servo_index].receivePack << (himark_servo::servoinfo_uavcan_struct[i] - 8);
				}
			}
			//第二阶段
			servo_decode_state[servo_index].receivePack = 0;
			servo_decode_state[servo_index].receivePack = (uint64_t)rxData[2] << 8*7 | (uint64_t)rxData[3] << 8*6 | (uint64_t)rxData[4] << 8*5 | (uint64_t)rxData[5] << 8*4 | (uint64_t)rxData[6] << 8*3;
			// PX4_INFO("received data 3 is %llX",servo_decode_state[servo_index].receivePack);
			servo_decode_state[servo_index].receivePack = servo_decode_state[servo_index].receivePack << 5;//手动计算得到的5，目的是消除上一个字段的高5位，该高五位在第一阶段已经提取
			for (uint8_t i = 5; i < 9; i++)
			{
				if(himark_servo::servoinfo_uavcan_struct[i] <= 8){//目标数据小于8字节，那么一次提取即可
					//提取数据
					servo_decode_state[servo_index].servoinfo_raw_data[i] = servo_decode_state[servo_index].receivePack >> (64 - himark_servo::servoinfo_uavcan_struct[i]);
					//左移，去除已经提取的数据
					servo_decode_state[servo_index].receivePack = servo_decode_state[servo_index].receivePack << himark_servo::servoinfo_uavcan_struct[i];
				}else{//目标数据大于8位，根据厂家给出的定义，消息字段最长有16位，那么二次提取即可
					//提取低八位数据
					servo_decode_state[servo_index].servoinfo_raw_data[i] = servo_decode_state[servo_index].receivePack >> (64 - 8);
					//左移，去除已经提取的低八位数据
					servo_decode_state[servo_index].receivePack = servo_decode_state[servo_index].receivePack << 8;

					//提取高位数据，可能不足八位
					servo_decode_state[servo_index].servoinfo_raw_data[i] = servo_decode_state[servo_index].servoinfo_raw_data[i] | (( servo_decode_state[servo_index].receivePack >> (64 - (himark_servo::servoinfo_uavcan_struct[i] - 8)) ) << 8);
					//左移，去除已经提取的数据
					servo_decode_state[servo_index].receivePack = servo_decode_state[servo_index].receivePack << (himark_servo::servoinfo_uavcan_struct[i] - 8);
				}
			}
			//第三阶段整理数据，并发送
			servo_report[servo_index].timestamp = hrt_absolute_time();
			servo_report[servo_index].servo_id = servo_decode_state[servo_index].servoinfo_raw_data[0];//uint5
			servo_report[servo_index].pwm_input = servo_decode_state[servo_index].servoinfo_raw_data[1];//uint12
			servo_report[servo_index].pos_cmd = (int16_t)servo_decode_state[servo_index].servoinfo_raw_data[2];//int16
			servo_report[servo_index].pos_sensor = (int16_t)servo_decode_state[servo_index].servoinfo_raw_data[3];//int16
			servo_report[servo_index].voltage = servo_decode_state[servo_index].servoinfo_raw_data[4];//uint12
			servo_report[servo_index].current = servo_decode_state[servo_index].servoinfo_raw_data[5];//uint10
			servo_report[servo_index].pcb_temp = servo_decode_state[servo_index].servoinfo_raw_data[6];//uint10
			servo_report[servo_index].motor_temp = servo_decode_state[servo_index].servoinfo_raw_data[7];//uint10
			servo_report[servo_index].statusinfo_flags = servo_decode_state[servo_index].servoinfo_raw_data[8];//uint5
			orb_publish(ORB_ID(servoinfo), _servoinfo_sub[servo_index], &servo_report[servo_index]);

			if(servo_index == 0){
				_battery.setConnected(true);
				_battery.updateVoltage(servo_report[servo_index].voltage*0.01);
				_battery.updateCurrent(-1);
				_battery.updateAndPublishBatteryStatus(hrt_absolute_time());


				// _LV_status.timestamp = hrt_absolute_time();

				// _LV_status.voltage_v = servo_report[servo_index].voltage*0.01;
				// _LV_status.voltage_filtered_v = servo_report[servo_index].voltage*0.01;

				// _LV_status.current_a = -1;
				// _LV_status.current_filtered_a = 0;
				// _LV_status.current_average_a = -1;

				// _LV_status.discharged_mah = -1;
				// _LV_status.time_remaining_s = NAN;
				// _LV_status.temperature = NAN;
				// _LV_status.is_powering_off = false;

				// _LV_status.scale = 1;
				// _LV_status.cell_count = 6;


				// _LV_status.voltage_cell_v[0] = _LV_status.voltage_v/6;
				// _LV_status.voltage_cell_v[1] = _LV_status.voltage_v/6;
				// _LV_status.voltage_cell_v[2] = _LV_status.voltage_v/6;
				// _LV_status.voltage_cell_v[3] = _LV_status.voltage_v/6;
				// _LV_status.voltage_cell_v[4] = _LV_status.voltage_v/6;
				// _LV_status.voltage_cell_v[5] = _LV_status.voltage_v/6;

				// _LV_status.remaining = (servo_report[servo_index].voltage*0.01 - 22.2)/(25.2-22.2);//4.2*6 - 3.7*6

				// _LV_status.id = 3;

				// if(_LV_status.voltage_v < 25.2f)_LV_status.warning = battery_status_s::BATTERY_WARNING_NONE;//4.2*6
				// // if(_LV_status.voltage_v < 23.4f)_LV_status.warning = battery_status_s::BATTERY_WARNING_LOW;//3.9
				// if(_LV_status.voltage_v < 22.8f)_LV_status.warning = battery_status_s::BATTERY_WARNING_LOW;//3.8
				// if(_LV_status.voltage_v < 22.2f)_LV_status.warning = battery_status_s::BATTERY_WARNING_EMERGENCY;//3.7
				// if(_LV_status.voltage_v < 21.0f)_LV_status.warning = battery_status_s::BATTERY_WARNING_FAILED;//3.5

				// _LV_status.connected = true;

				// _LV_status_pub.publish(_LV_status);
			}

		}
		servo_decode_state[servo_index].servoinfo_state = himark_servo::decode_servoinfo_state::PACK_H;
		break;

	default:
		break;
	}
}

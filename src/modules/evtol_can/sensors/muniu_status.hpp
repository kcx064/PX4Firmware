#include "CanSensorBridge.hpp"
#include <lib/drivers/device/Device.hpp>
#include <drivers/rangefinder/PX4Rangefinder.hpp>

#include <uORB/topics/distance_sensor.h>

// using namespace device;

class muniu_status : public CanSensorBridgeBase, public ModuleParams
{
public:
	static const char *const NAME;

	muniu_status() :
		ModuleParams(nullptr)
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

	uint8_t get_frame_type() override
	{
		return 0; //覆盖基类，声明本订阅帧类型为标准帧。基类方法默认返回标准帧
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
		(0x055),
		(0x056),
		(0x057),
		(0x058),
	};
	static constexpr size_t MSG_ID_COUNT = sizeof(msg_id_list)/sizeof(msg_id_list[0]);

private:
	// CAN数据包结构（单帧8字节）
	typedef struct {
		uint8_t data[8];
	} CAN_Packet_t;

	// 32字节CAN数据联合体
	typedef union {
		uint8_t raw_data[32];  // 原始32字节数据
		CAN_Packet_t packets[4];  // 4帧CAN数据包

		struct {
			// 帧头
			uint8_t header_high;     // data1: 0xEB
			uint8_t header_low;      // data2: 0x90

			// 设备信息（data3）
			uint8_t device_id;       // data3: 设备识别码

			// 包信息长度（data4）
			uint8_t packet_length;   // data4: 0x1C

			// 告警信息（data5-data6）
			uint8_t alarm_high;      // data5: 告警高字节
			uint8_t alarm_low;       // data6: 告警低字节

			// 24G雷达数据（data7-data11）
			uint8_t high1_h;         // data7: 高度1高字节
			uint8_t high1_l;         // data8: 高度1低字节
			uint8_t snr1;            // data9: 高度1信噪比
			uint8_t speed1_h;        // data10: 速度1高字节
			uint8_t speed1_l;        // data11: 速度1低字节

			// 60G雷达数据（data12-data16）
			uint8_t high2_h;         // data12: 高度2高字节
			uint8_t high2_l;         // data13: 高度2低字节
			uint8_t snr2;            // data14: 高度2信噪比
			uint8_t speed2_h;        // data15: 速度2高字节
			uint8_t speed2_l;        // data16: 速度2低字节

			// 融合雷达数据（data17-data21）
			uint8_t high3_h;         // data17: 高度3高字节
			uint8_t high3_l;         // data18: 高度3低字节
			uint8_t snr3;            // data19: 高度3信噪比
			uint8_t speed3_h;        // data20: 速度3高字节
			uint8_t speed3_l;        // data21: 速度3低字节

			// 高度4数据（data22-data26）
			uint8_t high4_h;         // data22: 高度4高字节
			uint8_t high4_l;         // data23: 高度4低字节
			uint8_t snr4;            // data24: 高度4信噪比
			uint8_t speed4_h;        // data25: 速度4高字节
			uint8_t speed4_l;        // data26: 速度4低字节

			// 高度5数据（data27-data31）
			uint8_t high5_h;         // data27: 高度5高字节
			uint8_t high5_l;         // data28: 高度5低字节
			uint8_t snr5;            // data29: 高度5信噪比
			uint8_t speed5_h;        // data30: 速度5高字节
			uint8_t speed5_l;        // data31: 速度5低字节

			// 校验和（data32）
			uint8_t checksum;        // data32: 校验和
		} fields;
	} ULandingPRO_Data_t;

	ULandingPRO_Data_t can_data;

	typedef enum receive_order {
		WAIT = 0,
		FIRST,        // 第一个CAN帧
		SECOND,       // 第二个CAN帧
		THIRD,        // 第三个CAN帧
		FOURTH        // 第四个CAN帧
	} receive_order_t;

	receive_order_t current_frame = FIRST;

	void process_can_data();
	bool verify_checksum();
	uint8_t calculate_checksum();
	float parse_height(uint8_t high_byte, uint8_t low_byte);
	float parse_speed(uint8_t high_byte, uint8_t low_byte);

	// distance_sensor_s _distance_sensor{};
	// uORB::PublicationMulti<distance_sensor_s> _distance_sensor_pub{ORB_ID(distance_sensor)};

	PX4Rangefinder rangefinder{0 ,distance_sensor_s::ROTATION_DOWNWARD_FACING};
	device::Device::DeviceId device_id{};

	// Parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::EN_MUNIU>) _param_en_muniu
	)//最后一行没有逗号

};

const char *const muniu_status::NAME = "muniu_radar";
constexpr uint32_t muniu_status::msg_id_list[];
constexpr size_t muniu_status::MSG_ID_COUNT;

int muniu_status::init()
{
	device_id.devid_s.devtype = DRV_DIST_DEVTYPE_UAVCAN;
	device_id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_UAVCAN;
	device_id.devid_s.address = static_cast<uint8_t>(7);//地址就是node id但是该设备不是uavcan，所以这里的数值是随便设置的
	rangefinder.set_device_id(device_id.devid);
	rangefinder.set_rangefinder_type(distance_sensor_s::MAV_DISTANCE_SENSOR_RADAR);
	rangefinder.set_hfov(0.75);//0.75rad  == 方位43°
	rangefinder.set_vfov(0.52);//0.52rad  == 俯仰30°
	rangefinder.set_min_distance(0.3);
	rangefinder.set_max_distance(500.0);
	return 0;
}

void muniu_status::msg_cb(uint8_t canModule, uint32_t msg_id, uint8_t *rxData, uint8_t len)
{
	perf_count(_count_perf);
	_CANModule = canModule;

	switch (current_frame)
	{
		case receive_order_t::FIRST:
			if(msg_id == 0x55 ){
				memcpy(&can_data.packets[0].data[0], rxData, 8);
				current_frame = receive_order_t::SECOND;
			}
			break;

		case receive_order_t::SECOND:
			if(msg_id == 0x56 ){
				memcpy(&can_data.packets[1].data[0], rxData, 8);
				current_frame = receive_order_t::THIRD;
			}
			break;

		case receive_order_t::THIRD:
			if(msg_id == 0x57 ){
				memcpy(&can_data.packets[2].data[0], rxData, 8);
				current_frame = receive_order_t::FOURTH;
			}
			break;

		case receive_order_t::FOURTH:
			if(msg_id == 0x58 )
			{
				memcpy(&can_data.packets[3].data[0], rxData, 8);
				//完整的帧处理完毕，开始提取有效数据
				process_can_data();
				current_frame = receive_order_t::FIRST;
			}
			break;

		default:
			current_frame = receive_order_t::FIRST;

	}

}

void muniu_status::process_can_data()
{
	// 验证帧头
	if (can_data.fields.header_high != 0xEB ||
	can_data.fields.header_low != 0x90)
	{
		PX4_INFO("header error\n");
		return;
	}

	// 验证校验和
	if (!verify_checksum()) {
		PX4_INFO("checksum error\n");
		return;
	}

	// 解析数据
	// float altitude_24g = parse_height(can_data.fields.high1_h, can_data.fields.high1_l);
	// float speed_24g = parse_speed(can_data.fields.speed1_h, can_data.fields.speed1_l);

	// float altitude_60g = parse_height(can_data.fields.high2_h, can_data.fields.high2_l);
	// float speed_60g = parse_speed(can_data.fields.speed2_h, can_data.fields.speed2_l);

	float altitude_fusion = parse_height(can_data.fields.high3_h, can_data.fields.high3_l);
	// float speed_fusion = parse_speed(can_data.fields.speed3_h, can_data.fields.speed3_l);

	// 输出结果
	// PX4_INFO("24GHz height: %.2f m, speed: %.2f m/s\n", static_cast<double>(altitude_24g), static_cast<double>(speed_24g));
	// PX4_INFO("60GHz height: %.2f m, speed: %.2f m/s\n", static_cast<double>(altitude_60g), static_cast<double>(speed_60g));
	// PX4_INFO("fused height: %.2f m, speed: %.2f m/s\n", static_cast<double>(altitude_fusion), static_cast<double>(speed_fusion));

	// _distance_sensor.timestamp = hrt_absolute_time();
	rangefinder.update(hrt_absolute_time(), altitude_fusion, (can_data.fields.snr3 > 0 ? 100:0)* _param_en_muniu.get());
}

// 校验函数
uint8_t muniu_status::calculate_checksum()
{
    uint8_t sum = 0;
    uint8_t *ptr = &can_data.raw_data[3];  // 从data4开始（索引3）

    // data4到data31 共28个字节
    for (int i = 0; i < 28; i++) {
        sum += ptr[i];
    }

    return sum & 0xFF;  // 与0xFF进行与操作
}

// 校验数据完整性
bool muniu_status::verify_checksum()
{
    return can_data.fields.checksum == calculate_checksum();
}

// 解析高度值（单位：0.01m）
float muniu_status::parse_height(uint8_t high_byte, uint8_t low_byte) {
    uint16_t raw = ((uint16_t)high_byte << 8) | low_byte;
    return raw * 0.01f;  // 单位：米
}

// 解析速度值（单位：0.01m/s，有符号）
float muniu_status::parse_speed(uint8_t high_byte, uint8_t low_byte) {
    int16_t raw = (int16_t)((high_byte << 8) | low_byte);
    return raw * 0.01f;  // 单位：米/秒
}

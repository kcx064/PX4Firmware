#include "CanSensorBridge.hpp"

#include "lib_uavcan_parser.hpp"
#include <drivers/rangefinder/PX4Rangefinder.hpp>

#include <uORB/topics/distance_sensor.h>

class nalei_status : public CanSensorBridgeBase, public ModuleParams
{
public:
	static const char *const NAME;

	nalei_status() :
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
		(0x18041A3C),
		(0x18041A3C),
		(0x18041A3C),
	};
	static constexpr size_t MSG_ID_COUNT = sizeof(msg_id_list)/sizeof(msg_id_list[0]);

private:
	//用于uavcan esc消息解析
	uint8_t 			buffer[30] = {0,};
	uint8_t 			buff_len_max{30}; //要求buff_len_max与buffer长度匹配 buffer总bit容量需要大于等于_field_info中bit_width总和
	uavcan_field_info_s 		_field_info[8] = {
						{32,0},
						{24,0},
						{8,0},
						{16,0},
						{16,0},
						{5,0},
						{3,0},
						{16,0}};
	uint8_t 			_field_info_size{8}; //要求与_field_info长度匹配
	lib_uavcan_parser 		_uavcan_parser{buffer, buff_len_max, _field_info, _field_info_size};

	//
	PX4Rangefinder rangefinder{0 ,distance_sensor_s::ROTATION_DOWNWARD_FACING};
	device::Device::DeviceId device_id{};

	float float16_to_float32(uint16_t f);

	// Parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::EN_NALEI>) _param_en_nalei
	)//最后一行没有逗号

};

const char *const nalei_status::NAME = "nalei_radar";
constexpr uint32_t nalei_status::msg_id_list[];
constexpr size_t nalei_status::MSG_ID_COUNT;

int nalei_status::init()
{
	device_id.devid_s.devtype = DRV_DIST_DEVTYPE_UAVCAN;
	device_id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_UAVCAN;
	device_id.devid_s.address = static_cast<uint8_t>(0x3C);//地址就是node id但是该设备不是uavcan，所以这里的数值是随便设置的
	rangefinder.set_device_id(device_id.devid);
	rangefinder.set_rangefinder_type(distance_sensor_s::MAV_DISTANCE_SENSOR_RADAR);
	rangefinder.set_hfov(0.75);//0.75rad  == 方位43°
	rangefinder.set_vfov(0.52);//0.52rad  == 俯仰30°
	rangefinder.set_min_distance(0.5);
	rangefinder.set_max_distance(200.0);
	return 0;
}

void nalei_status::msg_cb(uint8_t canModule, uint32_t msg_id, uint8_t *rxData, uint8_t len)
{
	perf_count(_count_perf);
	_CANModule = canModule;

	if(_uavcan_parser.run(rxData, len))
	{
		float_t range = float16_to_float32(static_cast<uint16_t>(_field_info[7].field_val));//提取距离

		rangefinder.set_fov(float16_to_float32(static_cast<uint16_t>(_field_info[4].field_val)));//设置fov
		rangefinder.update(hrt_absolute_time(), range, (_field_info[6].field_val==1? 100:0)*_param_en_nalei.get()); //发送并设置可信度 为READING_TYPE_VALID_RANGE才设置为1否则为0
	}
}


float nalei_status::float16_to_float32(uint16_t f)
{
	uint32_t sign = (f >> 15) & 0x1;
	uint32_t exponent = (f >> 10) & 0x1F;
	uint32_t mantissa = f & 0x3FF;

	// 处理特殊值
	if (exponent == 0x1F) { // 指数全1
		if (mantissa == 0) {
			// 无穷大
			uint32_t result = (sign << 31) | 0x7F800000;
			return *(float *)&result;

		} else {
			// NaN
			uint32_t result = (sign << 31) | 0x7F800000 | (mantissa << 13);
			return *(float *)&result;
		}
	}

	if (exponent == 0) {
		// 处理零和非规格化数 (此处简化，将非规格化数也视为0)
		uint32_t result = (sign << 31);
		return *(float *)&result;
	}

	// 正常转换
	exponent = exponent - 15 + 127; // 调整偏置
	mantissa = mantissa << 13; // 左移13位，低位补0

	uint32_t result = (sign << 31) | (exponent << 23) | mantissa;
	return *(float *)&result;
}

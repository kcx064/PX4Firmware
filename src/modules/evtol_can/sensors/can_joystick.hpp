#include "CanSensorBridge.hpp"

#include <lib/parameters/param.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/input_rc.h>

/**
 * @brief TS45F9-743ZB 三轴多功能操作杆 CAN 驱动
 *
 * 通信参数：CAN 2.0A 标准帧，默认波特率 1000Kbps，周期 20ms(50Hz)，默认帧ID 0x35(可通过参数 JY_CAN_ID 修改)
 *
 * 数据帧格式（8 字节）：
 *  Byte0: 开关状态字  bit0=SW1  bit1=SW2  bit2=SW3 (1=开关闭合)
 *  Byte1: 保留
 *  Byte2: X轴数据  有符号  左~中~右 = 80H~00H~7FH (-128~0~127)
 *  Byte3: Y轴数据  有符号  后~中~前 = 80H~00H~7FH
 *  Byte4: Z1轴数据 有符号  下~中~上 = 80H~00H~7FH
 *  Byte5: Z2轴数据 有符号  左旋~中~右旋 = 80H~00H~7FH
 *  Byte6: 保留
 *  Byte7: 保留
 *
 * 杆量映射到 input_rc 通道（PWM 脉宽格式，1000~2000us，中位1500）：
 *  ch1 = roll     ( X ) 左负右正
 *  ch2 = pitch    (-Y ) 前推为正 -> 反转后前推杆为负 pitch(nose down)
 *  ch3 = throttle ( Z1) 下负上正
 *  ch4 = yaw      ( Z2) 左旋负右旋正(顺时针为正)
 *  ch5 = SW1 开关
 *  ch6 = SW2 开关
 *  ch7 = SW3 开关
 *
 * 发布：input_rc 主题（instance 0），由 rc_update 模块订阅，经 RC_MAP_* 通道映射后
 * 发布 manual_control_input 及 manual_control_switches，无需直接发布 manual_control_setpoint。
 */
class can_joystick : public CanSensorBridgeBase, public ModuleParams
{
public:
	static const char *const NAME;

	can_joystick() :
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
		return 0; //覆盖基类，声明本订阅帧类型为CAN2.0A标准帧
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

	// 帧ID可在 init() 中根据参数 JY_CAN_ID 配置，默认 0x35
	static uint32_t msg_id_list[1];
	static constexpr size_t MSG_ID_COUNT = 1;

private:
	// 输出通道数（4杆量 + 3开关）
	static constexpr uint8_t CHANNEL_COUNT = 7;

	// PWM 脉宽范围（默认 RC_MIN/TRIM/MAX = 1000/1500/2000）
	static constexpr int32_t PWM_MIN = 1000;
	static constexpr int32_t PWM_TRIM = 1500;
	static constexpr int32_t PWM_MAX = 2000;

	// 杆量(int8, -128~127) -> PWM(1000~2000)，中位1500，可选 dir 反转(±1)
	static uint16_t pwm_from_stick(int8_t raw, int32_t dir = 1)
	{
		int32_t pwm = PWM_TRIM + dir * (raw * (PWM_MAX - PWM_TRIM)) / 127;

		if (pwm < PWM_MIN) {
			pwm = PWM_MIN;
		}

		if (pwm > PWM_MAX) {
			pwm = PWM_MAX;
		}

		return (uint16_t)pwm;
	}

	// 开关(1=闭合) -> PWM
	static uint16_t pwm_from_switch(bool closed)
	{
		return closed ? (uint16_t)PWM_MAX : (uint16_t)PWM_MIN;
	}

	// 发布 input_rc（instance 0），rc_update 模块订阅并转换为 manual_control_input
	uORB::Publication<input_rc_s> _input_rc_pub{ORB_ID(input_rc)};

	// 接收帧计数（调试用，发布到 rc_total_frame_count）
	uint16_t _frame_count{0};

	// Parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::JY_CAN_ID>) _param_jy_can_id // 操纵杆CAN帧ID，默认0x35
	)
};

const char *const can_joystick::NAME = "can_joystick";
uint32_t can_joystick::msg_id_list[1] = {0x35};
constexpr size_t can_joystick::MSG_ID_COUNT;

int can_joystick::init()
{
	msg_id_list[0] = (uint32_t)_param_jy_can_id.get();
	return 0;
}

void can_joystick::msg_cb(uint8_t canModule, uint32_t msg_id, uint8_t *rxData, uint8_t len)
{
	perf_count(_count_perf);
	_CANModule = canModule;

	// 数据帧需至少包含 Byte0(开关)+Byte2~5(四路杆量)，即6字节
	if (len < 6) {
		return;
	}

	// Byte0: 开关状态字（bit0=SW1, bit1=SW2, bit2=SW3）
	const uint8_t switches = rxData[0];

	// Byte2~5: 四路杆量（有符号 int8）
	const int8_t x_raw  = (int8_t)rxData[2];
	const int8_t y_raw  = (int8_t)rxData[3];
	const int8_t z1_raw = (int8_t)rxData[4];
	const int8_t z2_raw = (int8_t)rxData[5];

	input_rc_s rc{};
	rc.timestamp = hrt_absolute_time();
	rc.timestamp_last_signal = rc.timestamp;

	// 杆量 -> PWM 脉宽；pitch 前推(正)需要反转以符合 PX4 前推为负的约定
	rc.values[0] = pwm_from_stick(x_raw);                         // ch1 roll
	rc.values[1] = pwm_from_stick(y_raw, -1);                     // ch2 pitch (反转)
	rc.values[2] = pwm_from_stick(z1_raw);                        // ch3 throttle
	rc.values[3] = pwm_from_stick(z2_raw);                        // ch4 yaw
	rc.values[4] = pwm_from_switch(switches & 0x01);              // ch5 SW1
	rc.values[5] = pwm_from_switch(switches & 0x02);              // ch6 SW2
	rc.values[6] = pwm_from_switch(switches & 0x04);              // ch7 SW3

	rc.channel_count = CHANNEL_COUNT;
	rc.rssi = 100;                              // 满信号
	rc.rc_failsafe = false;
	rc.rc_lost = false;
	rc.rc_lost_frame_count = 0;
	rc.rc_total_frame_count = _frame_count++;
	rc.rc_ppm_frame_length = 0;
	rc.input_source = input_rc_s::RC_INPUT_SOURCE_UNKNOWN; // 与 rc_update 初始值一致，保持源稳定
	rc.link_quality = 100;
	rc.rssi_dbm = NAN;

	_input_rc_pub.publish(rc);
}

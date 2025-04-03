#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <lib/systemlib/mavlink_log.h>

// #include <drivers/device/device.h>
// #include <drivers/drv_hrt.h>
// #include <drivers/drv_pwm_output.h>
#include <lib/mixer_module/mixer_module.hpp>
// #include <px4_platform_common/px4_config.h>
// #include <px4_platform_common/tasks.h>
// #include <px4_platform_common/time.h>

#include "can_driver/MW_PX4_CAN_DEVICE.h"
#include "actuators/dcdc.hpp"
#include "actuators/canesc.hpp"
#include "actuators/canservo.hpp"
#include "actuators/CanMixingInterfaceEsc.hpp"
#include "actuators/CanMixingInterfaceServo.hpp"

#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/parameter_update.h>
// #include <uORB/topics/sensor_accel.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/mixer_outputs.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/can_actuator_test.h> //only for actuator debug when disarmed
#include <uORB/topics/servoinfo.h>
#include <uORB/topics/can_esc_report.h>
#include <uORB/topics/can_esc_ret.h>
#include <uORB/topics/can_servo_ret.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/distance_sensor.h>
// #include <uORB/topics/actuator_controls.h>
#include <uORB/topics/manual_control_switches.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/db_value.h>

using namespace time_literals;

class EvtolCan : public ModuleParams, public px4::ScheduledWorkItem
{
public:
	EvtolCan(MW_H7CAN_DEVICE& h7can_device);
	~EvtolCan();

	static int start();

	void print_info();

	bool init();

	static EvtolCan	*instance() { return _instance; }


private:
	void Run() override;

	MW_H7CAN_DEVICE 		&_h7can_device;
	canesc				_canesc;
	canservo			_canservo;


	pthread_mutex_t			_node_mutex;
	CanMixingInterfaceEsc 		_can_interface_esc{_node_mutex, _canesc};
	CanMixingInterfaceServo		_can_interface_servo{_node_mutex, _canservo};

	dcdc				_dcdc;


	static EvtolCan			*_instance;			///< singleton pointer

	// Parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::DB_INTERVAL>) _param_db_interval,
		// (ParamFloat<px4::params::BAT_CELL_V>) _param_bat_cell_v,
		// (ParamInt<px4::params::DB_SERVO_BIAS>) _param_db_servo_bias,
		(ParamInt<px4::params::DB_CAN_RATE>) _param_db_can_rate
		// (ParamInt<px4::params::DB_BMS_EN>) _param_db_bms_en,
		// (ParamInt<px4::params::DB_RC_SEL>) _param_db_rc_sel,
		// (ParamInt<px4::params::DB_SRV_CHK>) _param_db_srv_chk,
		// (ParamInt<px4::params::DB_ESC_SEND>) _param_db_esc_send,
		// (ParamInt<px4::params::MPC_POS_MODE>) _param_mpc_pos_mode,
		// (ParamInt<px4::params::DB_ESC_VDR>) _param_db_esc_vendor,
		// (ParamInt<px4::params::DB_AW_EN>) _param_db_aw_en
	)//最后一行没有逗号

	perf_counter_t	_cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle time")};
	perf_counter_t	_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": cycle interval")};

};

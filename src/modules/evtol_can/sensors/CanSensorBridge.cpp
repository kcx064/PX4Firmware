#include "CanSensorBridge.hpp"
#include "lib/parameters/param.h"
#include "esc_status.hpp"
#include "esc_status_uavcan.hpp"
#include "dcdc_status.hpp"
#include "servo_status.hpp"
#include "bms_status.hpp"
#include "redundancy_detector.hpp"
#include "pmu_uavcan.hpp"
#include "muniu_status.hpp"
#include "nalei_status.hpp"
#include "can_joystick.hpp"

void ICanSensorBridge::make_all(List<ICanSensorBridge *> &list)
{
	int32_t can_sub_esc = 1;
	param_get(param_find("SUB_SM_ESC"), &can_sub_esc);
	if(can_sub_esc != 0){
		list.add(new esc_status());
	}

	int32_t can_sub_dcdc = 1;
	param_get(param_find("SUB_DCDC"), &can_sub_dcdc);
	if(can_sub_dcdc != 0){
		list.add(new dcdc_status());
	}

	int32_t can_sub_servo = 1;
	param_get(param_find("SUB_HMARK_SERVO"), &can_sub_servo);
	if(can_sub_servo != 0){
		list.add(new servo_status());
	}

	int32_t can_sub_bms = 1;
	param_get(param_find("SUB_FM_BATT"), &can_sub_bms);
	if(can_sub_bms != 0){
		list.add(new bms_status());
	}

	int32_t can_sub_redundency_detector = 1;
	param_get(param_find("SUB_REDU_DETEC"), &can_sub_redundency_detector);
	if (can_sub_redundency_detector != 0)
	{
		list.add(new redundancy_detector());
	}

	int32_t uavcan_sub_pmu = 1;
	param_get(param_find("SUB_PMU"), &uavcan_sub_pmu);
	if (uavcan_sub_pmu != 0)
	{
		list.add(new pmu_uavcan());
	}

	int32_t uavcan_sub_esc = 1;
	param_get(param_find("SUB_UAVCAN_ESC"), &uavcan_sub_esc);
	if (uavcan_sub_esc != 0)
	{
		list.add(new esc_status_uavcan());
	}

	int32_t can_sub_muniu = 1;
	param_get(param_find("SUB_MUNIU"), &can_sub_muniu);
	if (can_sub_muniu != 0)
	{
		list.add(new muniu_status());
	}

	int32_t can_sub_malei = 1;
	param_get(param_find("SUB_NALEI"), &can_sub_malei);
	if (can_sub_malei != 0)
	{
		list.add(new nalei_status());
	}

	int32_t can_sub_joystick = 1;
	param_get(param_find("SUB_JOYSTICK"), &can_sub_joystick);
	if (can_sub_joystick != 0)
	{
		list.add(new can_joystick());
	}

}




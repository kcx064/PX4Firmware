#include "CanSensorBridge.hpp"
#include "lib/parameters/param.h"
#include "esc_status.hpp"
#include "dcdc_status.hpp"
#include "servo_status.hpp"

void ICanSensorBridge::make_all(List<ICanSensorBridge *> &list)
{
	int32_t can_sub_esc = 1;
	param_get(param_find("SUB_SM_ESC"), &can_sub_esc);
	if(can_sub_esc!=0){
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


}




#include "CanSensorBridge.hpp"
#include "lib/parameters/param.h"
#include "esc_status.hpp"
#include "dcdc_status.hpp"


void ICanSensorBridge::make_all(List<ICanSensorBridge *> &list)
{
	list.add(new esc_status(0));

	int32_t can_sub_dcdc = 1;
	param_get(param_find("SUB_DCDC"), &can_sub_dcdc);
	if(can_sub_dcdc != 0){
		list.add(new dcdc_status(0));
	}

}

CanSensorBridgeBase::~CanSensorBridgeBase()
{
}




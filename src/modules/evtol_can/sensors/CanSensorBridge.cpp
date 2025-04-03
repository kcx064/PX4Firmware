#include "CanSensorBridge.hpp"

#include "esc_status.hpp"


void ICanSensorBridge::make_all(List<ICanSensorBridge *> &list)
{
	list.add(new esc_status(0));
}

CanSensorBridgeBase::~CanSensorBridgeBase()
{
}




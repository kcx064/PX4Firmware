#pragma once

#include <containers/List.hpp>
#include <string.h>

class ICanSensorBridge : public ListNode<ICanSensorBridge *>
{
public:
	// ICanSensorBridge(uint32_t &_msg_id_list_ref) :
	// 	msg_id_list_ref(_msg_id_list_ref)
	// {}
	virtual ~ICanSensorBridge() = default;

	/**
	 * Returns ASCII name of the bridge.
	 */
	virtual const char *get_name() const = 0;

	// =0,派生类中必须实现此函数，用以初始化传感器桥接器。
	virtual int init() = 0;

	virtual const uint32_t* get_msg_id() = 0;
	virtual size_t get_msg_id_num() = 0;
	virtual uint8_t get_can_module() = 0;//CANModule

	virtual void msg_cb(uint32_t msg_id, uint8_t *rxData, uint8_t len) {};

	static void make_all(List<ICanSensorBridge *> &list);

};


class CanSensorBridgeBase : public ICanSensorBridge
{
protected:
	CanSensorBridgeBase(){};
public:

	virtual ~CanSensorBridgeBase();
};



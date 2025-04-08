#pragma once

#include <drivers/drv_hrt.h>
#include <containers/List.hpp>
#include <string.h>
#include <lib/perf/perf_counter.h>
#include <stdio.h>


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

	virtual void msg_cb(uint8_t canModule, uint32_t msg_id, uint8_t *rxData, uint8_t len) {};

	/**
	 * Prints current status in a human readable format to stdout.
	 */
	virtual void print_status() const = 0;

	static void make_all(List<ICanSensorBridge *> &list);

};


class CanSensorBridgeBase : public ICanSensorBridge
{
protected:
	CanSensorBridgeBase(){
		perf_begin(_count_perf);
	};
public:
	uint8_t _CANModule{0};
	~CanSensorBridgeBase(){
		perf_free(_count_perf);
	};

	perf_counter_t	_count_perf{perf_alloc(PC_COUNT, MODULE_NAME": received count")};

	void print_status() const override
	{
		printf("channel: %d(can port: %d)\n", _CANModule, (_CANModule+1));
		perf_print_counter(_count_perf);

		// for (unsigned i = 0; i < _max_channels; i++) {
		// 	if (_channels[i].node_id >= 0) {
		// 		printf("channel %d: node id %d --> instance %d\n",
		// 		i, _channels[i].node_id, _channels[i].instance);
		// 	}
		// }
	}
};



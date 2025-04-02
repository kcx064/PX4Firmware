#include "EvtolCan.hpp"

static MW_H7CAN_DEVICE h7can;

EvtolCan *EvtolCan::_instance;


EvtolCan::EvtolCan(MW_H7CAN_DEVICE& h7can_device) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::uavcan),
	_h7can_device(h7can_device),
	_canesc(_h7can_device),
	_canservo(_h7can_device)
{
	int res = pthread_mutex_init(&_node_mutex, nullptr);
	_h7can_device.init(0, _param_db_can_rate.get(), 0);

	if (res < 0) {
		std::abort();
	}
}

EvtolCan::~EvtolCan()
{
	pthread_mutex_destroy(&_node_mutex);

	perf_free(_cycle_perf);
	perf_free(_interval_perf);
}
bool EvtolCan::init()
{
	// execute Run() on every mixer_output publication
	// if (!_mixer_output_sub.registerCallback()) {
	// 	PX4_ERR("callback registration failed");
	// 	return false;
	// }

	// alternatively, Run on fixed interval
	// ScheduleOnInterval(_param_db_interval.get()); // 10 ms interval, 100 Hz rate
	printf("EvtolCan init success!\n");

	return true;
}

void EvtolCan::print_info()
{
	(void)pthread_mutex_lock(&_node_mutex);

	perf_print_counter(_cycle_perf);
	perf_print_counter(_interval_perf);

	(void)pthread_mutex_unlock(&_node_mutex);
}

void EvtolCan::Run()
{

	perf_begin(_cycle_perf);
	perf_count(_interval_perf);
}

int EvtolCan::start()
{
	if (_instance != nullptr) {
		PX4_WARN("Already started");
		return -1;
	}

	// if (h7can == nullptr)
	// {
	// 	h7can = new MW_H7CAN_DEVICE();

	// 	if (h7can == nullptr) {  // We don't have exceptions so bad_alloc cannot be thrown
	// 		PX4_ERR("Out of memory");
	// 		return -1;
	// 	}
	// }

	_instance = new EvtolCan(h7can);

	if (_instance == nullptr) {
		PX4_ERR("Out of memory");
		return -1;
	}

	_instance->ScheduleOnInterval(10_ms);

	_instance->_can_interface_esc.ScheduleNow();
	_instance->_can_interface_servo.ScheduleNow();
	return 0;
}

/*
 * App entry point
 */
static void print_usage()
{
	PX4_INFO("usage: \n"
		 "\tevtol_can {start|status|stop}\n");
}

extern "C" __EXPORT int evtol_can_main(int argc, char *argv[])
{
	if (argc < 2) {
		print_usage();
		::exit(1);
	}

	int32_t uavcan_enable = 1;
	(void)param_get(param_find("UAVCAN_ENABLE"), &uavcan_enable);

	if (!std::strcmp(argv[1], "start") && uavcan_enable==0) {
		if (EvtolCan::instance()) {
			// Already running, no error
			PX4_INFO("already started");
			::exit(0);
		}

		// // Node ID
		// int32_t node_id = 1;
		// (void)param_get(param_find("UAVCAN_NODE_ID"), &node_id);

		// if (node_id < 0 || node_id > uavcan::NodeID::Max || !uavcan::NodeID(node_id).isUnicast()) {
		// 	PX4_ERR("Invalid Node ID %" PRId32, node_id);
		// 	::exit(1);
		// }

		// // CAN bitrate
		// int32_t bitrate = 1000000;
		// (void)param_get(param_find("UAVCAN_BITRATE"), &bitrate);

		// Start
		// PX4_INFO("Node ID %" PRIu32 ", bitrate %" PRIu32, node_id, bitrate);
		return EvtolCan::start();
	}

	/* commands below require the app to be started */
	EvtolCan *const inst = EvtolCan::instance();

	if (!inst) {
		errx(1, "application not running");
	}

	if (!std::strcmp(argv[1], "status") || !std::strcmp(argv[1], "info")) {
		inst->print_info();
		::exit(0);
	}

	if (!std::strcmp(argv[1], "stop")) {
		delete inst;
		::exit(0);
	}

	print_usage();
	::exit(1);
}

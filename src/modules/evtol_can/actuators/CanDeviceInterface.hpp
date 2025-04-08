#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

class CanDeviceInterface : public px4::ScheduledWorkItem, public ModuleParams
{
public:
	CanDeviceInterface(const char *name, const px4::wq_config_t &config, pthread_mutex_t& node_mutex):
		px4::ScheduledWorkItem(name, config),
		ModuleParams(nullptr),
		_node_mutex(node_mutex)
	{}

	~CanDeviceInterface() {
		perf_free(_cycle_perf);
		perf_free(_interval_perf);
	};

	virtual bool updateOutputs() = 0;

	void print_status();

	pthread_mutex_t &_node_mutex;
private:
	void Run() override;

	perf_counter_t	_cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle time")};
	perf_counter_t	_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": cycle interval")};

};

inline void CanDeviceInterface::Run()
{
	pthread_mutex_lock(&_node_mutex);
	updateOutputs();
	pthread_mutex_unlock(&_node_mutex);

	perf_begin(_cycle_perf);
	perf_count(_interval_perf);
}

void CanDeviceInterface::print_status()
{
	perf_print_counter(_cycle_perf);
	perf_print_counter(_interval_perf);
}

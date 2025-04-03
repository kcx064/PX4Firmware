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

	virtual bool updateOutputs() = 0;

	pthread_mutex_t &_node_mutex;
private:
	void Run() override;

};

inline void CanDeviceInterface::Run()
{
	pthread_mutex_lock(&_node_mutex);
	updateOutputs();
	pthread_mutex_unlock(&_node_mutex);
}

#include <lib/mixer_module/mixer_module.hpp>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/debug_value.h>

#include "canesc.hpp"

class CanMixingInterfaceEsc : public OutputModuleInterface
{
public:
	CanMixingInterfaceEsc(pthread_mutex_t &node_mutex, canesc &can_esc_controller):
		OutputModuleInterface(MODULE_NAME "-esc", px4::wq_configurations::test1),
  	  	_node_mutex(node_mutex),
		_can_esc_controller(can_esc_controller)
		{
			_mixing_output.setMaxNumOutputs(can_esc_controller._rotor_num);
		}

	bool updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
		unsigned num_outputs, unsigned num_control_groups_updated) override;

	void mixerChanged() override;

	static constexpr unsigned MAX_RATE_HZ = 400;

private:
	void Run() override;

	pthread_mutex_t &_node_mutex;
	canesc &_can_esc_controller;

	MixingOutput _mixing_output{"CAN_EC", canesc::MAX_ACTUATORS, *this, MixingOutput::SchedulingPolicy::Auto, false, false};

	uORB::PublicationMulti<actuator_outputs_s> _actuator_outputs_esc_pub{ORB_ID(actuator_outputs_can_esc)};

	// Parameters
	// DEFINE_PARAMETERS(
	// 	(ParamInt<px4::params::CA_ROTOR_COUNT>) _ca_rotor_count // decided by the current airframe file
	// )//最后一行没有逗号
};

bool CanMixingInterfaceEsc::updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS], unsigned num_outputs,
	unsigned num_control_groups_updated)
{
	// publish actuator outputs if any control group has been updated
	if (num_control_groups_updated > 0)
	{
		_can_esc_controller.update_outputs(stop_motors, outputs, num_outputs);
		actuator_outputs_s actuator_outputs{};
		actuator_outputs.noutputs = num_outputs;
		for (int i = 0; i < 8; i++)
		{
			actuator_outputs.output[i] = outputs[i];
		}

		actuator_outputs.timestamp = hrt_absolute_time();
		_actuator_outputs_esc_pub.publish(actuator_outputs);
		return true;
	}
	return false;
}

void CanMixingInterfaceEsc::mixerChanged()
{

}

void CanMixingInterfaceEsc::Run()
{
	pthread_mutex_lock(&_node_mutex);
	_mixing_output.update();
	_mixing_output.updateSubscriptions(false);
	pthread_mutex_unlock(&_node_mutex);
}

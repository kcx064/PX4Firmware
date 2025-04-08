#include <lib/mixer_module/mixer_module.hpp>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>

#include "canservo.hpp"

class CanMixingInterfaceServo : public OutputModuleInterface
{
public:
	CanMixingInterfaceServo(pthread_mutex_t &node_mutex, canservo &can_servo_controller):
		OutputModuleInterface(MODULE_NAME "-servo", px4::wq_configurations::test1),
  	  	_node_mutex(node_mutex),
		_can_servo_controller(can_servo_controller)
		{}
	~CanMixingInterfaceServo() {
		perf_free(_cycle_perf);
		perf_free(_interval_perf);
	};

	bool updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
		unsigned num_outputs, unsigned num_control_groups_updated) override;

	void mixerChanged() override;

	void print_status();

	static constexpr unsigned MAX_RATE_HZ = 400;
private:
	void Run() override;

	pthread_mutex_t &_node_mutex;
	canservo &_can_servo_controller;

	MixingOutput _mixing_output{"CAN_SV", canservo::MAX_ACTUATORS, *this, MixingOutput::SchedulingPolicy::Auto, false, false};

	uORB::PublicationMulti<actuator_outputs_s> _actuator_outputs_servo_pub{ORB_ID(actuator_outputs_can_servo)};

	perf_counter_t	_cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle time")};
	perf_counter_t	_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": cycle interval")};
};

bool CanMixingInterfaceServo::updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS], unsigned num_outputs,
	unsigned num_control_groups_updated)
{
	// publish actuator outputs if any control group has been updated
	if (num_control_groups_updated > 0)
	{
		_can_servo_controller.update_outputs(stop_motors, outputs, num_outputs);
		actuator_outputs_s actuator_outputs{};
		actuator_outputs.noutputs = num_outputs;
		for (int i = 0; i < 8; i++)
		{
			actuator_outputs.output[i] = outputs[i];
		}

		actuator_outputs.timestamp = hrt_absolute_time();
		_actuator_outputs_servo_pub.publish(actuator_outputs);
		return true;
	}
	return false;
}

void CanMixingInterfaceServo::mixerChanged()
{

}

void CanMixingInterfaceServo::Run()
{


	pthread_mutex_lock(&_node_mutex);

	perf_begin(_cycle_perf);
	perf_count(_interval_perf);

	_mixing_output.update();
	_mixing_output.updateSubscriptions(false);

	perf_end(_cycle_perf);

	pthread_mutex_unlock(&_node_mutex);



}

void CanMixingInterfaceServo::print_status()
{
	perf_print_counter(_cycle_perf);
	perf_print_counter(_interval_perf);
}

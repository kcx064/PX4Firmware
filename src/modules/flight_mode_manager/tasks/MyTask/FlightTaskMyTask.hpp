#pragma once

#include <lib/collision_prevention/CollisionPrevention.hpp>
#include "FlightTaskManualAltitude.hpp"

class FlightTaskMyTask : public FlightTaskManualAltitude
{
public:
	FlightTaskMyTask();

	virtual ~FlightTaskMyTask() = default;
	bool activate(const vehicle_local_position_setpoint_s &last_setpoint) override;
	bool updateInitialize() override;

	/**
	 * Sets an external yaw handler which can be used to implement a different yaw control strategy.
	 */
	void setYawHandler(WeatherVane *yaw_handler) override { _weathervane_yaw_handler = yaw_handler; }

protected:
	void _updateXYlock(); /**< applies position lock based on stick and velocity */
	void _updateSetpoints() override;
	void _scaleSticks() override;

	DEFINE_PARAMETERS_CUSTOM_PARENT(FlightTaskManualAltitude,
					(ParamFloat<px4::params::MPC_VEL_MANUAL>) _param_mpc_vel_manual,
					(ParamFloat<px4::params::MPC_ACC_HOR_MAX>) _param_mpc_acc_hor_max,
					(ParamFloat<px4::params::MPC_HOLD_MAX_XY>) _param_mpc_hold_max_xy
				       )
private:
	uint8_t _reset_counter{0}; /**< counter for estimator resets in xy-direction */

	WeatherVane *_weathervane_yaw_handler =
		nullptr;	/**< external weathervane library, used to implement a yaw control law that turns the vehicle nose into the wind */

	CollisionPrevention _collision_prevention;	/**< collision avoidance setpoint amendment */
};

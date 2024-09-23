/****************************************************************************
 *
 *   Copyright (C) 2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file dbactuator_step.cpp
 *
 * CLI to publish the actuator_test msg
 */

#include <drivers/drv_hrt.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/actuator_test.h>
#include <uORB/topics/can_actuator_test.h>
#include <uORB/topics/can_esc_report.h>
#include <uORB/topics/servoinfo.h>

#include <math.h>

extern "C" __EXPORT int dbactuator_step_main(int argc, char *argv[]);

static void actuator_test(int function, float value, int timeout_ms, bool release_control);
static void actuator_test_can(uint8_t index, float cmd, bool is_run);

static void usage(const char *reason);

void actuator_test(int function, float value, int timeout_ms, bool release_control)
{
	actuator_test_s actuator_test{};

	actuator_test.timestamp = hrt_absolute_time();
	actuator_test.function = function;
	actuator_test.value = value;
	actuator_test.action = release_control ? actuator_test_s::ACTION_RELEASE_CONTROL : actuator_test_s::ACTION_DO_CONTROL;
	actuator_test.timeout_ms = timeout_ms;

	uORB::Publication<actuator_test_s> actuator_test_pub{ORB_ID(actuator_test)};
	actuator_test_pub.publish(actuator_test);
}

void actuator_test_can(uint8_t index, float cmd, bool is_run)
{
	can_actuator_test_s can_actuator_test{};

	can_actuator_test.timestamp = hrt_absolute_time();
	can_actuator_test.actuator_index = index;
	can_actuator_test.cmd = cmd;
	can_actuator_test.is_run = is_run;

	float thrust = ((cmd + 1)/2) * 100;

	//subscribe esc or servo report
	if(index<=2){
		can_esc_report_s can_esc_report{};
		int esc_sub = orb_subscribe_multi(ORB_ID(can_esc_report), index);
		orb_copy(ORB_ID(can_esc_report), esc_sub, &can_esc_report);

		PX4_INFO("index: %d, cmd: %f(%.2f%%), run status: %d, rpm: %d", index, static_cast<double>(cmd), static_cast<double>(thrust), is_run, can_esc_report.rpm);
	}else if(index>2 && index<=6){
		servoinfo_s servo_report{};
		int servo_sub = orb_subscribe_multi(ORB_ID(servoinfo), index-2);
		orb_copy(ORB_ID(servoinfo), servo_sub, &servo_report);

		PX4_INFO("index: %d, cmd: %f(%.2f%%), run status: %d, pos: %d", index, static_cast<double>(cmd), static_cast<double>(thrust), is_run, servo_report.pos_sensor);
	}else{
		PX4_INFO("index: %d, cmd: %f(%.2f%%), run status: %d", index, static_cast<double>(cmd), static_cast<double>(thrust), is_run);
	}

	uORB::Publication<can_actuator_test_s> can_actuator_test_pub{ORB_ID(can_actuator_test)};
	bool ret = can_actuator_test_pub.publish(can_actuator_test);

	//TODO：舵机换向集中到一个函数中
	px4_usleep(100);
	PX4_INFO("publish can_actuator_test return %d", ret);
}

static void usage(const char *reason)
{
	if (reason != nullptr) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
Utility to test actuators.

Note: this is only used in combination with SYS_CTRL_ALLOC=1.

WARNING: remove all props before using this command.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("dbactuator_step", "command");
	PRINT_MODULE_USAGE_COMMAND_DESCR("set", "Set an actuator to a specific output value");
	PRINT_MODULE_USAGE_PARAM_COMMENT("The actuator can be specified by motor, servo or function directly:");
	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 1, 8, "Motor to test (1...8)", true);
	PRINT_MODULE_USAGE_PARAM_INT('s', -1, 1, 8, "Servo to test (1...8)", true);
	PRINT_MODULE_USAGE_PARAM_INT('f', -1, 1, 8, "Specify function directly", true);

	PRINT_MODULE_USAGE_PARAM_FLOAT('v', 0, -1, 1, "value (-1...1)", false);
	PRINT_MODULE_USAGE_PARAM_INT('t', 0, 0, 100, "Timeout in seconds (run interactive if not set)", true);

	PRINT_MODULE_USAGE_COMMAND_DESCR("iterate-motors", "Iterate all motors starting and stopping one after the other");
	PRINT_MODULE_USAGE_COMMAND_DESCR("iterate-servos", "Iterate all servos deflecting one after the other");

	PRINT_MODULE_USAGE_COMMAND_DESCR("ramp", "Ramp response of motor");
	PRINT_MODULE_USAGE_PARAM_COMMENT("Example: dbactuator_step ramp -m 1 -w 15000");
	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 1, 8, "Motor to test (1...8)", false);
	PRINT_MODULE_USAGE_PARAM_INT('w', -1, 10, 1000, "Wait time in microsecond(us) to ramp motor", false);

	PRINT_MODULE_USAGE_COMMAND_DESCR("step", "step response of motor");
	PRINT_MODULE_USAGE_PARAM_COMMENT("Example: dbactuator_step step -m 1 -v 0.2 -u 0.3");
	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 1, 8, "Motor to test (1...8)", false);
	PRINT_MODULE_USAGE_PARAM_FLOAT('v', 0, -1, 1, "value (-1...1)", false);
	PRINT_MODULE_USAGE_PARAM_FLOAT('u', 0, -1, 1, "Step value (-1...1)", false);

	PRINT_MODULE_USAGE_COMMAND_DESCR("can", "can actuator test, esc: 1 2 3, servo: 4 5 6 7, all servo: 8, all esc: 9");
	PRINT_MODULE_USAGE_PARAM_COMMENT("Example: dbactuator_step can -m 1 -w 5 -v 0.2");
	PRINT_MODULE_USAGE_PARAM_FLOAT('v', 0, -1, 1, "value (-1...1)", false);
	PRINT_MODULE_USAGE_PARAM_INT('w', -1, 0, 1000, "Timeout in seconds", false);

}

int dbactuator_step_main(int argc, char *argv[])
{
	int function = 0;

	float step_vaule = 10.0f;
	int wait_time_s = -1;

	float value = 10.0f;
	int ch;
	int timeout_ms = 0;

	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "m:w:s:f:v:u:t", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {

		case 'm':
			function = actuator_test_s::FUNCTION_MOTOR1 + (int)strtol(myoptarg, nullptr, 0) - 1;
			break;

		/* Wait time (s) */
		case 'w':
			wait_time_s = (int)strtol(myoptarg, nullptr, 0);
			PX4_INFO("wait time is %d s", wait_time_s);
			// wait_time_s = strtof(myoptarg, nullptr);
			break;

		case 's':
			function = actuator_test_s::FUNCTION_SERVO1 + (int)strtol(myoptarg, nullptr, 0) - 1;
			break;

		case 'f':
			function = (int)strtol(myoptarg, nullptr, 0);
			break;

		case 'v':
			value = strtof(myoptarg, nullptr);
			PX4_INFO("value is %f", (double)value);
			if (value < -1.f || value > 1.f) {
				usage("value invalid");
				return 1;
			}
			break;

		/* Step value */
		case 'u':
			step_vaule = strtof(myoptarg, nullptr);

			PX4_INFO("step value is %f", (double)step_vaule);

			if (step_vaule < -1.f || step_vaule > 1.f) {
				usage("step vaule invalid");
				return 1;
			}
			break;

		case 't':
			timeout_ms = strtof(myoptarg, nullptr) * 1000.f;
			PX4_INFO("time out is %d ms", timeout_ms);
			break;

		default:
			usage(nullptr);
			return 1;
		}
	}


	if (myoptind >= 0 && myoptind < argc) {
		if (strcmp("set", argv[myoptind]) == 0) {

			if (value > 9.f) {
				usage("Missing argument: value");
				return 1;
			}

			if (function == 0) {
				usage("Missing argument: function");
				return 1;
			}

			if (timeout_ms == 0) {
				// interactive
				actuator_test(function, value, 0, false);

				/* stop on any user request */
				PX4_INFO("Press Enter to stop");
				char c;
				ssize_t ret = read(0, &c, 1);

				if (ret < 0) {
					PX4_ERR("read failed: %i", errno);
				}

				actuator_test(function, NAN, 0, true);
			} else {
				actuator_test(function, value, timeout_ms, false);
			}
			return 0;

		} else if (strcmp("iterate-motors", argv[myoptind]) == 0) {
			value = 0.15f;
			for (int i = 0; i < actuator_test_s::MAX_NUM_MOTORS; ++i) {
				PX4_INFO("Motor %i (%.0f%%)", i, (double)(value*100.f));
				actuator_test(actuator_test_s::FUNCTION_MOTOR1+i, value, 400, false);
				px4_usleep(600000);
			}
			return 0;

		} else if (strcmp("iterate-servos", argv[myoptind]) == 0) {
			value = 0.3f;
			for (int i = 0; i < actuator_test_s::MAX_NUM_SERVOS; ++i) {
				PX4_INFO("Servo %i (%.0f%%)", i, (double)(value*100.f));
				actuator_test(actuator_test_s::FUNCTION_SERVO1+i, value, 800, false);
				px4_usleep(1000000);
			}
			return 0;
		} else if (strcmp("ramp", argv[myoptind]) == 0) {
			value = 0.1f;
			// 逐渐增加至0.1-0.3，4s
			for(int i=0; i<4; i++)
			{
				value += 0.05f;
				actuator_test(function, value, 0, false);
				PX4_INFO("Motor test point %i (%.2f%%)", i, (double)(value*100.f));
				px4_sleep(1);

			}

			//维持指定时间
			PX4_INFO("Wait for 5s");
			px4_sleep(5);

			// 希望在0.3s内提升至0.7的油门
			for (int i = 0; i < 20; i++) {
				value += 0.02f;
				actuator_test(function, value, 0, false);
				PX4_INFO("Motor test point %i (%.2f%%)", i, (double)(value*100.f));
				px4_usleep(wait_time_s);//15000us
			}

			//维持指定时间
			PX4_INFO("Wait for 5s");
			px4_sleep(5);


			//10s内逐渐降低至0.2
			for (int i = 0; i<10; i++)
			{
				value -= 0.05f;
				actuator_test(function, value, 0, false);
				PX4_INFO("Motor test point %i (%.2f%%)", i, (double)(value*100.f));
				px4_sleep(1);
			}


			actuator_test(function, NAN, 0, true);
			return 0;
		} else if (strcmp("step", argv[myoptind]) == 0) {
			actuator_test(function, value, 0, false);
			PX4_INFO("Motor test value (%.0f%%)", (double)(value*100.f));
			px4_sleep(3);//3s
			actuator_test(function, step_vaule, 0, false);
			PX4_INFO("Motor test value (%.0f%%)", (double)(step_vaule*100.f));
			px4_sleep(3);//3s
			actuator_test(function, NAN, 0, true);
			return 0;
		} else if (strcmp("can", argv[myoptind]) == 0) {
			/* 转速增加阶段 */
			float temp_value = -1.0;
			do{
				actuator_test_can(function - actuator_test_s::FUNCTION_MOTOR1, temp_value, true);
				px4_usleep(50000);// 50ms
				PX4_WARN("Motor ramp value (%.2f%%)", static_cast<double>((temp_value+1)*50.f));
				temp_value += 0.005f;
			}while(temp_value < value);
			/* 转速平稳阶段 */
			hrt_abstime start_time = hrt_absolute_time();
			float timeused = 0;
			do{
				actuator_test_can(function - actuator_test_s::FUNCTION_MOTOR1, value, true);
				px4_usleep(50000);// 50ms
				timeused = (float)((hrt_absolute_time() - start_time)/1000)/1000;
				PX4_WARN("timeused %.3fs", static_cast<double>(timeused));
			}while(timeused < wait_time_s);

			hrt_abstime during_time = hrt_absolute_time() - start_time;
			PX4_INFO("During time %.3fs", (double)(during_time/1000)/1000);
			/* 转速下降阶段 */
			temp_value = value;
			do{
				actuator_test_can(function - actuator_test_s::FUNCTION_MOTOR1, temp_value, true);
				px4_usleep(50000);// 50ms
				PX4_WARN("Motor ramp value (%.2f%%)", static_cast<double>((temp_value+1)*50.f));
				temp_value -= 0.005f;
			}while(temp_value > -1.0f);
			/* 停止阶段 */
			px4_usleep(1000*5);
			actuator_test_can(function - actuator_test_s::FUNCTION_MOTOR1, 0, false);
			px4_usleep(1000*5);
			actuator_test_can(function - actuator_test_s::FUNCTION_MOTOR1, 0, false);
			return 0;
		}
	}

	usage(nullptr);
	return 0;
}

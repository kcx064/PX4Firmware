############################################################################
#
#   Copyright (c) 2021 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################
set(LIBUAVCAN_DIR ${PX4_SOURCE_DIR}/src/drivers/uavcan/libuavcan)
set(LIBUAVCAN_DIR_DRIVERS ${PX4_SOURCE_DIR}/src/drivers/uavcan/uavcan_drivers)

px4_add_module(
	MODULE modules__evtol_can
	MAIN evtol_can
	COMPILE_FLAGS
		-DMW_CAN_UAVCAN
		-DUAVCAN_CPP_VERSION=UAVCAN_CPP03
		-DUAVCAN_STM32H7_NUTTX
		-DUAVCAN_STM32H7_NUM_IFACES=2
		-DMW_NUM_CAN_RECEIVE_RAW=39
	SRCS
		EvtolCan.cpp
		EvtolCan.hpp
		can_driver/MW_H7CAN.cpp
		can_driver/MW_PX4_CAN_DEVICE.h
		can_driver/rtwtypes.h
		actuators/canesc.hpp
		actuators/throttle_dronecan_msg.hpp
		actuators/CanMixingInterfaceEsc.hpp
		actuators/CanMixingInterfaceServo.hpp
		actuators/dcdc.hpp
		actuators/CanDeviceInterface.hpp
		sensors/CanSensorBridge.cpp
		sensors/CanSensorBridge.hpp
		sensors/esc_status.hpp
		sensors/dcdc_status.hpp
		sensors/servo_status.hpp
		sensors/bms_status.hpp
		sensors/redundancy_detector.hpp
	INCLUDES
		${LIBUAVCAN_DIR}/libuavcan/include
		${LIBUAVCAN_DIR_DRIVERS}/stm32h7/driver/include
		${LIBUAVCAN_DIR}/libuavcan/include/dsdlc_generated
	MODULE_CONFIG
		module.yaml
	DEPENDS
		px4_work_queue
		mixer_module
	)

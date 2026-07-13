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

############################################################################
#
# DSDL code generation via the dronecan_dsdlc submodule
#
# The dronecan_dsdlc.py script parses the DSDL definitions under dsdl/ and
# emits C sources/headers into the dsdlc_generated/ subdirectory. The build
# automatically re-runs the generator whenever a .uavcan file changes.
#
set(DSDLC_COMPILER ${CMAKE_CURRENT_SOURCE_DIR}/dronecan_dsdlc/dronecan_dsdlc.py)
set(DSDLC_INPUTS
	${CMAKE_CURRENT_SOURCE_DIR}/dsdl/ardupilot
	${CMAKE_CURRENT_SOURCE_DIR}/dsdl/com
	${CMAKE_CURRENT_SOURCE_DIR}/dsdl/cuav
	${CMAKE_CURRENT_SOURCE_DIR}/dsdl/dronecan
	${CMAKE_CURRENT_SOURCE_DIR}/dsdl/uavcan
)
set(DSDLC_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/dsdlc_generated)

# Collect all DSDL input files so the custom command re-runs when they change.
file(GLOB_RECURSE DSDLC_INPUT_FILES ${CMAKE_CURRENT_SOURCE_DIR}/dsdl *.uavcan)

# Stamp/marker file produced by the generator; used as the custom command output.
set(DSDLC_STAMP ${DSDLC_OUTPUT_PATH}/include/dronecan_msgs.h)

add_custom_command(
	OUTPUT ${DSDLC_STAMP}
	COMMAND ${CMAKE_COMMAND} -E make_directory ${DSDLC_OUTPUT_PATH}
	COMMAND ${PYTHON_EXECUTABLE} ${DSDLC_COMPILER}
		--output ${DSDLC_OUTPUT_PATH} ${DSDLC_INPUTS}
	DEPENDS ${DSDLC_INPUT_FILES} ${DSDLC_COMPILER}
	COMMENT "Generating DSDL code with dronecan_dsdlc into dsdlc_generated"
	VERBATIM
)
add_custom_target(evtol_canard_dsdlc DEPENDS ${DSDLC_STAMP})

# Generated C sources (compiled as part of the module). Globbed so newly
# generated files are picked up; a reconfigure may be required after the
# first generation run to include files created during the build.
file(GLOB_RECURSE DSDLC_GENERATED_SRCS_FILES ${DSDLC_OUTPUT_PATH}/src/*.c)

px4_add_module(
	MODULE modules__evtol_canard
	MAIN evtol_canard
	COMPILE_FLAGS
		-DMW_CAN_UAVCAN
		-DUAVCAN_CPP_VERSION=UAVCAN_CPP03
		-DUAVCAN_STM32H7_NUTTX
		-DUAVCAN_STM32H7_NUM_IFACES=2
		-DMW_NUM_CAN_RECEIVE_RAW=48
	SRCS
		EvtolCanard.cpp
		EvtolCanard.hpp
		canard.c
		canard.h
		canard_internals.h
		can_driver/MW_H7CAN.cpp
		can_driver/MW_PX4_CAN_DEVICE.h
		can_driver/rtwtypes.h
		canard_stm32h7.hpp
		${DSDLC_GENERATED_SRCS_FILES}

	INCLUDES
		${CMAKE_CURRENT_SOURCE_DIR}
		${LIBUAVCAN_DIR}/libuavcan/include
		${LIBUAVCAN_DIR_DRIVERS}/stm32h7/driver/include
		${LIBUAVCAN_DIR}/libuavcan/include/dsdlc_generated
		${DSDLC_OUTPUT_PATH}/include
	DEPENDS
		evtol_canard_dsdlc
		px4_work_queue
		mixer_module
	)

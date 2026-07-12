#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <assert.h>

#include "canard.h"
#include "can_driver/MW_PX4_CAN_DEVICE.h"


class canard_stm32h7 : public MW_H7CAN_DEVICE
{
private:
	/* data */
public:
	canard_stm32h7(/* args */);
	~canard_stm32h7();
	uint16_t can_init();
};

canard_stm32h7::canard_stm32h7(/* args */)
{
}

canard_stm32h7::~canard_stm32h7()
{
}

uint16_t canard_stm32h7::can_init()
{
	::init(0, 1000000, 0);

}



// int16_t STM32h7_CANInit(void);








/*
 * interrupts.c
 *
 *  Created on: 16 нояб. 2026 г.
 *      Author: user
 */

#include "Init.h"


void OTG_HS_IRQHandler(void)
{

    tud_int_handler(1);
}

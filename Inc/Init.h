/*
 * Init.h
 *
 *  Created on: 2 нояб. 2026 г.
 *      Author: user
 */

#ifndef INIT_H_
#define INIT_H_

#include "stm32f4xx.h"
#include "tusb.h"
#include "core_cm4.h"
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stddef.h>

//#include "bsp/board_api.h"
//#include "tusb.h"
#include "usb_descriptors.h"

int ClockInit(void);
void InitPin(void);
void Init_USB_HS(void);
void tud_vendor_rx_cb(uint8_t itf, uint8_t const *buffer, uint16_t bufsize);
void buf_16_out(uint8_t  *buf);
void buf_65k_out(void);
void procedure (void);
void SysTick_init(void);



#endif /* INIT_H_ */

/*
 * timer_tic.c
 *
 *  Created on: 19 нояб. 2026 г.
 *      Author: user
 */
#include "Init.h"
volatile uint8_t tic_usb_in = 0;       // флаг начала приема по usb
//volatile uint8_t usb_16_time = 0;       // таймер для приема 16 байт
volatile uint8_t usb_64k_time = 0;      // таймер приема 64кбайт


void time_tic(void)
{
	if(tic_usb_in)
	{
		usb_64k_time;
	//	if(usb_16_time >50) usb_64k_time++;


	}
}

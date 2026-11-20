/*
 * func_gen.c
 *
 *  Created on: 16 нояб. 2026 г.
 *      Author: user
 */

#include "Init.h"
extern uint8_t exp_oper;
extern uint8_t buf_16[16];
uint8_t buf_er[16] = {0xF0,0xF0,0xF0,0xF0,0,0,0,0,0,0,0,0,0,0,0,0};

uint8_t crc8_smbus(const uint8_t *data, size_t length)
{
    uint8_t crc = 0x00;

    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];  // Добавляем очередной байт через XOR

        for (unsigned bit = 0; bit < 8; bit++) {
            uint8_t top_bit = crc & 0x80;  // Старший бит ДО сдвига
            crc = (uint8_t)(crc << 1);    // Оставляем младшие 8 бит

            if (top_bit != 0) {
                crc ^= 0x07;              // Шаг деления по модулю 2
            }
        }
    }

    return crc;
}

void procedure (void)
{
	switch(exp_oper)
	{
	case 0x10: exp_oper = 0x20; break;
	case 0x11: exp_oper = 0x21; break;
	case 0x20: buf_16_out(buf_16);    break;
	case 0x21: buf_65k_out();   break;
	case 0xF0:
		buf_16_out(buf_16);
		exp_oper = 0xF0;
		break;
	}
}

/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#ifdef __cplusplus
 extern "C" {
#endif

//--------------------------------------------------------------------+
//Конфигурация, специфичная для платы
//--------------------------------------------------------------------+

//  Номер порта RH, используемый для устройства, может быть задан в файле board.mk; по умолчанию используется порт 0.
#ifndef BOARD_TUD_RHPORT
#define BOARD_TUD_RHPORT      1
#endif

// Максимальная рабочая скорость RHPort может быть задана в файле board.mk.
#ifndef BOARD_TUD_MAX_SPEED
#define BOARD_TUD_MAX_SPEED   OPT_MODE_HIGH_SPEED
#endif

//--------------------------------------------------------------------
// Общая конфигурация
//--------------------------------------------------------------------

// определяется с помощью флагов компилятора для обеспечения гибкости
#ifndef CFG_TUSB_MCU
#define CFG_TUSB_MCU OPT_MCU_STM32F4
#endif

#ifndef CFG_TUSB_OS
#define CFG_TUSB_OS           OPT_OS_NONE
#endif

#ifndef CFG_TUSB_DEBUG
#define CFG_TUSB_DEBUG        0
#endif

// Включить стек устройств
#define CFG_TUD_ENABLED       1

// По умолчанию используется максимальная скорость, которую может поддерживать аппаратный контроллер с помощью встроенного в кристалл PHY-блока.
#define CFG_TUD_MAX_SPEED     BOARD_TUD_MAX_SPEED

/*  В некоторых микроконтроллерах (MCU) механизм USB DMA может обращаться только к определённой области SRAM, причём к ней предъявляются требования по выравниванию.
 * В библиотеке TinyUSB используются следующие макросы для объявления памяти, предназначенной для передачи данных, что позволяет разместить её
 * в указанной области.
 * Примеры:
 * - CFG_TUSB_MEM_SECTION : __attribute__ (( section(".usb_ram") ))
 * - CFG_TUSB_MEM_ALIGN   : __attribute__ ((aligned(4)))
 */
#ifndef CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_SECTION
#endif

#ifndef CFG_TUSB_MEM_ALIGN
#define CFG_TUSB_MEM_ALIGN        __attribute__ ((aligned(4)))
#endif

//--------------------------------------------------------------------
// КОНФИГУРАЦИЯ УСТРОЙСТВА
//--------------------------------------------------------------------

#ifndef CFG_TUD_ENDPOINT0_SIZE
#define CFG_TUD_ENDPOINT0_SIZE    64
#endif

//------------- CLASS -------------//
#define CFG_TUD_CDC               0
#define CFG_TUD_MSC               0
#define CFG_TUD_HID               0
#define CFG_TUD_MIDI              0
#define CFG_TUD_VENDOR            1

// CDC FIFO size of TX and RX
//#define CFG_TUD_CDC_RX_BUFSIZE    (TUD_OPT_HIGH_SPEED ? 512 : 64)
//#define CFG_TUD_CDC_TX_BUFSIZE    (TUD_OPT_HIGH_SPEED ? 512 : 64)

//  Размер FIFO для конечных точек поставщика (vendor endpoints) при передаче (TX) и приеме (RX)
 // Если задано нулевое значение: для конечных точек поставщика буферизация использоваться не будет
#define CFG_TUD_VENDOR_RX_BUFSIZE 0
#define CFG_TUD_VENDOR_TX_BUFSIZE 512
#define CFG_TUD_VENDOR_EPSIZE  512


#ifdef __cplusplus
 }
#endif

#endif /* _TUSB_CONFIG_H_ */

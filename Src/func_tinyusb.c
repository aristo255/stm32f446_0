/*
 * func_tinyusb.c
 *
 *  Created on: 16 нояб. 2026 г.
 *      Author: user
 */
#include "Init.h"

#define VENDOR_REQUEST_RESET_PROTOCOL  0x30
#define PROTOCOL_READY                 0x01

extern uint8_t in_full;
extern uint8_t buf_16[16];
extern uint8_t buf_65k[65536];
volatile uint16_t count_in = 0;
extern uint32_t count_in_65k;
extern uint8_t exp_oper;
extern uint32_t count_out_65k;

volatile uint8_t rx_call = 0;

//  ///////////// функция отправки части дескрипторов /////////

bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const* request)
{
  // nothing to with DATA & ACK stage
  if (stage != CONTROL_STAGE_SETUP) return true;

  if((request->bmRequestType_bit.type==TUSB_REQ_TYPE_VENDOR)&&(request->bRequest==VENDOR_REQUEST_MICROSOFT)&&(request->wIndex == 7))
  {
	  uint16_t total_len;
	  memcpy(&total_len, desc_ms_os_20 + 8, 2);
	  return tud_control_xfer(rhport, request, (void*)(uintptr_t)desc_ms_os_20, total_len);
  }

  if ((request->bmRequestType == 0xC0) &&
         (request->bRequest == VENDOR_REQUEST_RESET_PROTOCOL) &&
         (request->wValue == 0) &&
         (request->wIndex == 0) &&
         (request->wLength == 1))
     {
         static uint8_t ready = PROTOCOL_READY;  // отправка на ПК статуса сброса протокола 0x01


         count_in_65k = 0;                     // сброс счетчиков
         count_out_65k = 0;
         exp_oper = 0;                         // ожидание 16 байт


         return tud_control_xfer(rhport, request, &ready, sizeof(ready));
     }

  return false;
}

// ////////////  функция приема пакета от ПК   ///////////////////////////

void tud_vendor_rx_cb(uint8_t itf, uint8_t const *buffer, uint16_t bufsize)
{

    (void)itf;
    count_in = bufsize;                                                   // количество принятых байт

    if(exp_oper == 0)    // если ожидается 16 байт
    {
    for(int i=0; i<count_in; i++)  { buf_16[i] = buffer[i]; }              // копирование данных из приемного буфера
    exp_oper = 0x10;                                                              // ожидается обработка 16 байт
    }

    if(exp_oper == 0x1)    //  если ожидается 64кбайт
    {
    	rx_call++;
    	for(int i=0; i<count_in; i++)  { buf_65k[count_in_65k+i] = buffer[i];}  // копирование данных из приемного буфера
    	count_in_65k +=count_in;                                               // приращение счетчика принятых байт

    	if(count_in_65k == 65536)
    	{
    	exp_oper = 0x11;                                                      // ожидается обработка 64к
    	count_in_65k = 0;                                                     // сброс счетчика принятого
    	}
    }
}

// ///////////////  // отправка 16 байт на ПК   ////////////////////

void buf_16_out (uint8_t  *buf)
{
	    tud_vendor_write(buf, 16);                                  // загрузка данных для отправки
		tud_vendor_write_flush();                                    //  отправка данных
		exp_oper = 0x0;                                             // переход на ожидание 16 байт
		// если отправлено сообщение с ошибкой о неполной длине ожидание ответа через EP0
}

// /////////// отправка 64 к на ПК //////////////////////

void buf_65k_out(void)
{
   uint32_t remaining = sizeof(buf_65k)-count_out_65k;                      // осталось передать из 65к
   uint32_t sent = 0;                                                       // нужно передать байт
   if(remaining>512) sent = 512;
   else sent = remaining;
   uint32_t written = tud_vendor_write(&buf_65k[count_out_65k], sent);      // передача с определенного адреса массива
   count_out_65k += written;                                                // приращение счетчика переданных байт
   if(count_out_65k==sizeof(buf_65k))                                       // если передан весь массив
   {
	   exp_oper = 0x0;                                                        // переход ожидание 16 байт
	   count_out_65k =0;                                                      // сброс счетчика отправленного
   }
}

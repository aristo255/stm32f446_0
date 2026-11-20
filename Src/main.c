

#include "Init.h"

volatile uint8_t test = 1;
volatile uint8_t buf_65k[65536];       // буфер для пакета 65кбайт
volatile uint32_t count_in_65k = 0;    // счетчик пришедших байт из ожидаемых 65к
volatile uint32_t count_out_65k = 0;    // счетчик отправленных байт из  65к
volatile uint8_t buf_16[16];            // буфер для пакета 16 байт
volatile uint8_t in_full = 0;          // пришедший пакет
volatile uint8_t exp_oper = 0x1;       // ожидаемая операция 0-прием 16 байт         1 -прием 64к
                                     //                    10-обработка 16 байт    11-обработка 64к
                                     //                    20-отправка 16 байт     21- отправка 64к

int main(void)
{

	test = ClockInit();
	InitPin();
	Init_USB_HS();
    tusb_rhport_init_t dev_init = {
        .role  = TUSB_ROLE_DEVICE,
        .speed = TUSB_SPEED_HIGH
    };

    bool usb_init_ok = tusb_init(BOARD_TUD_RHPORT, &dev_init);
    if((test==0)&&(usb_init_ok !=0)) GPIOC->ODR |= 1<<GPIO_ODR_OD10_Pos;

    while(1)
    {
    	tud_task();
        procedure();
    }
}








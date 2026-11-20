/*
 * Init.c
 *
 *  Created on: 2 нояб. 2026 г.
 *      Author: user
 */

#include "Init.h"


int ClockInit(void)
{
	RCC->CR |= (1<<RCC_CR_HSEON_Pos);                 // запуск внешнего генератора
		uint16_t startUP = 0;

		for(startUP = 0;;startUP++)
		{
			if(RCC->CR & (1<<RCC_CR_HSERDY_Pos)) break;  // если генератор запустился выход из цикла проверки

			if(startUP > 0x1000)                         // если превышено количество попыток
			{
				RCC->CR &= ~(1<<RCC_CR_HSEON_Pos);      // остановка генератора
				return 1;                               // выход из функции с ошибкой
			}
		}

		// Включаем тактирование блока управления питанием.
		RCC->APB1ENR |= RCC_APB1ENR_PWREN;

		// Чтение обеспечивает задержку после включения тактирования.
		(void)RCC->APB1ENR;

		// Scale 1: поле VOS = 0b11.
		PWR->CR = (PWR->CR & ~PWR_CR_VOS_Msk)
		        | (0b11U << PWR_CR_VOS_Pos);

		RCC->PLLCFGR =0; // сброс битовых полей, котрые по дефолту не подходят
			RCC->PLLCFGR |= (0b10<<RCC_PLLCFGR_PLLR_Pos)|(0b100<<RCC_PLLCFGR_PLLQ_Pos)|
					        (1<<RCC_PLLCFGR_PLLSRC_Pos)|(0b0<<RCC_PLLCFGR_PLLP_Pos)|
			                (0b10110100<<RCC_PLLCFGR_PLLN_Pos)|(0b000100<<RCC_PLLCFGR_PLLM_Pos); // выбор делителя PLLM = 8,PLLQ = 2
			                                                      // остальные по дефолту(можно не дублировать)
			                                                      //и источника тактирования для него (HSE)
			RCC->CR |= 1<<RCC_CR_PLLON_Pos;

			for(startUP = 0;;startUP++)
					{
						if(RCC->CR & (1<<RCC_CR_PLLRDY_Pos)) break; // если генератор запустился выход из цикла проверки

						if(startUP > 0x1000)                        // если превышено количество попыток
						{
							RCC->CR &= ~(1<<RCC_CR_PLLON_Pos);      // остановка  PLL
							return 2;                               // выход из функции с ошибкой
						}
					}
			FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY_Msk)
			           | FLASH_ACR_LATENCY_5WS;

			while ((FLASH->ACR & FLASH_ACR_LATENCY_Msk)
			       != FLASH_ACR_LATENCY_5WS) {
			}
		         	RCC->CFGR &= ~(RCC_CFGR_HPRE_Msk
	                | RCC_CFGR_PPRE1_Msk
	                | RCC_CFGR_PPRE2_Msk);
					RCC->CFGR |= (0b100<<RCC_CFGR_PPRE2_Pos)|           // шина APB2 деление 2
							(0b101<<RCC_CFGR_PPRE1_Pos);              // шина APB1 деление 4
							                // шина AHB без деления

					// Ожидаем готовности выбранного режима питания.
					while ((PWR->CSR & PWR_CSR_VOSRDY) == 0U) {
					}

					// Включаем Over-drive.
					PWR->CR |= PWR_CR_ODEN;

					while ((PWR->CSR & PWR_CSR_ODRDY) == 0U) {
					}

					// Переключаем регулятор в режим Over-drive.
					PWR->CR |= PWR_CR_ODSWEN;

					while ((PWR->CSR & PWR_CSR_ODSWRDY) == 0U) {
					}

					RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW_Msk)
					          | (0b10U << RCC_CFGR_SW_Pos);

					while ((RCC->CFGR & RCC_CFGR_SWS_Msk)
					       != (0b10U << RCC_CFGR_SWS_Pos)) {
					}              // переключение на PLL

					RCC->CR &= ~RCC_CR_HSION;

					SystemCoreClockUpdate();



					RCC->AHB1ENR |= 1<<RCC_AHB1ENR_GPIOAEN_Pos;
					RCC->AHB1ENR |= 1<<RCC_AHB1ENR_GPIOBEN_Pos;
					RCC->AHB1ENR |= 1<<RCC_AHB1ENR_GPIOCEN_Pos;
					RCC->AHB1ENR |= 1<<RCC_AHB1ENR_GPIODEN_Pos;
					RCC->AHB1ENR |= 1<<RCC_AHB1ENR_GPIOEEN_Pos;
					RCC->AHB1ENR |= 1<<RCC_AHB1ENR_GPIOGEN_Pos;
			//		RCC->APB1ENR |= 1<<RCC_APB1ENR_TIM2EN_Pos;


					return 0;
}

void InitPin(void)
{
	GPIOC->MODER |= 1<<GPIO_MODER_MODE10_Pos;           // PC10-LED
	GPIOC->OTYPER &= ~(1<<GPIO_OTYPER_OT10_Pos);
	GPIOC->PUPDR |= 0x2<<GPIO_PUPDR_PUPD10_Pos;
	GPIOC->ODR &= ~(1<<GPIO_ODR_OD10_Pos);
}

void Init_USB_HS(void)
{

	RCC->AHB1ENR |= 1<<RCC_AHB1ENR_OTGHSEN_Pos;
	RCC->AHB1ENR |= 1<<RCC_AHB1ENR_OTGHSULPIEN_Pos;

    GPIOC->MODER |= 0b10<<GPIO_MODER_MODE2_Pos;            // OTG_HS_ULPI_DIR
    GPIOC->OTYPER &= ~(1<<GPIO_OTYPER_OT2_Pos);
    GPIOC->PUPDR &= ~(0b11<<GPIO_PUPDR_PUPD2_Pos);
    GPIOC->OSPEEDR |= 0b11<<GPIO_OSPEEDR_OSPEED2_Pos;
    GPIOC->AFR[0] |= 0b1010<<GPIO_AFRL_AFSEL2_Pos;

    GPIOC->MODER |= 0b10<<GPIO_MODER_MODE3_Pos;            // OTG_HS_ULPI_NXP
    GPIOC->OTYPER &= ~(1<<GPIO_OTYPER_OT3_Pos);
    GPIOC->PUPDR &= ~(0b11<<GPIO_PUPDR_PUPD3_Pos);
    GPIOC->OSPEEDR |= 0b11<<GPIO_OSPEEDR_OSPEED3_Pos;
    GPIOC->AFR[0] |= 0b1010<<GPIO_AFRL_AFSEL3_Pos;

    GPIOA->MODER |= 0b10<<GPIO_MODER_MODE3_Pos;            // OTG_HS_ULPI_D0
    GPIOA->OTYPER &= ~(1<<GPIO_OTYPER_OT3_Pos);
    GPIOA->PUPDR &= ~(0b11<<GPIO_PUPDR_PUPD3_Pos);
    GPIOA->OSPEEDR |= 0b11<<GPIO_OSPEEDR_OSPEED3_Pos;
    GPIOA->AFR[0] |= 0b1010<<GPIO_AFRL_AFSEL3_Pos;

    GPIOB->MODER |= 0b10<<GPIO_MODER_MODE0_Pos;            // OTG_HS_ULPI_D1
    GPIOB->OTYPER &= ~(1<<GPIO_OTYPER_OT0_Pos);
    GPIOB->PUPDR &= ~(0b11<<GPIO_PUPDR_PUPD0_Pos);
    GPIOB->OSPEEDR |= 0b11<<GPIO_OSPEEDR_OSPEED0_Pos;
    GPIOB->AFR[0] |= 0b1010<<GPIO_AFRL_AFSEL0_Pos;

    GPIOB->MODER |= 0b10<<GPIO_MODER_MODE1_Pos;            // OTG_HS_ULPI_D2
    GPIOB->OTYPER &= ~(1<<GPIO_OTYPER_OT1_Pos);
    GPIOB->PUPDR &= ~(0b11<<GPIO_PUPDR_PUPD1_Pos);
    GPIOB->OSPEEDR |= 0b11<<GPIO_OSPEEDR_OSPEED1_Pos;
    GPIOB->AFR[0] |= 0b1010<<GPIO_AFRL_AFSEL1_Pos;

    GPIOB->MODER |= 0b10<<GPIO_MODER_MODE10_Pos;            // OTG_HS_ULPI_D3
    GPIOB->OTYPER &= ~(1<<GPIO_OTYPER_OT10_Pos);
    GPIOB->PUPDR &= ~(0b11<<GPIO_PUPDR_PUPD10_Pos);
    GPIOB->OSPEEDR |= 0b11<<GPIO_OSPEEDR_OSPEED10_Pos;
    GPIOB->AFR[1] |= 0b1010<<GPIO_AFRH_AFSEL10_Pos;

    GPIOB->MODER |= 0b10<<GPIO_MODER_MODE2_Pos;            // OTG_HS_ULPI_D4
    GPIOB->OTYPER &= ~(1<<GPIO_OTYPER_OT2_Pos);
    GPIOB->PUPDR &= ~(0b11<<GPIO_PUPDR_PUPD2_Pos);
    GPIOB->OSPEEDR |= 0b11<<GPIO_OSPEEDR_OSPEED2_Pos;
    GPIOB->AFR[0] |= 0b1010<<GPIO_AFRL_AFSEL2_Pos;

    GPIOB->MODER |= 0b10<<GPIO_MODER_MODE12_Pos;            // OTG_HS_ULPI_D5
    GPIOB->OTYPER &= ~(1<<GPIO_OTYPER_OT12_Pos);
    GPIOB->PUPDR &= ~(0b11<<GPIO_PUPDR_PUPD12_Pos);
    GPIOB->OSPEEDR |= 0b11<<GPIO_OSPEEDR_OSPEED12_Pos;
    GPIOB->AFR[1] |= 0b1010<<GPIO_AFRH_AFSEL12_Pos;

    GPIOB->MODER |= 0b10<<GPIO_MODER_MODE13_Pos;            // OTG_HS_ULPI_D6
    GPIOB->OTYPER &= ~(1<<GPIO_OTYPER_OT13_Pos);
    GPIOB->PUPDR &= ~(0b11<<GPIO_PUPDR_PUPD13_Pos);
    GPIOB->OSPEEDR |= 0b11<<GPIO_OSPEEDR_OSPEED13_Pos;
    GPIOB->AFR[1] |= 0b1010<<GPIO_AFRH_AFSEL13_Pos;

    GPIOB->MODER |= 0b10<<GPIO_MODER_MODE5_Pos;            // OTG_HS_ULPI_D7
    GPIOB->OTYPER &= ~(1<<GPIO_OTYPER_OT5_Pos);
    GPIOB->PUPDR &= ~(0b11<<GPIO_PUPDR_PUPD5_Pos);
    GPIOB->OSPEEDR |= 0b11<<GPIO_OSPEEDR_OSPEED5_Pos;
    GPIOB->AFR[0] |= 0b1010<<GPIO_AFRL_AFSEL5_Pos;


    GPIOA->OTYPER &= ~(1<<GPIO_OTYPER_OT5_Pos);
    GPIOA->PUPDR &= ~(0b11<<GPIO_PUPDR_PUPD5_Pos);
    GPIOA->OSPEEDR |= 0b11<<GPIO_OSPEEDR_OSPEED5_Pos;
    GPIOA->AFR[0] |= 0b1010<<GPIO_AFRL_AFSEL5_Pos;
    GPIOA->MODER |= 0b10<<GPIO_MODER_MODE5_Pos;            // OTG_HS_ULPI_CK


    GPIOC->OTYPER &= ~(1<<GPIO_OTYPER_OT0_Pos);
    GPIOC->PUPDR &= ~(0b11<<GPIO_PUPDR_PUPD0_Pos);
    GPIOC->OSPEEDR |= 0b11<<GPIO_OSPEEDR_OSPEED0_Pos;
    GPIOC->AFR[0] |= 0b1010<<GPIO_AFRL_AFSEL0_Pos;
    GPIOC->MODER |= 0b10<<GPIO_MODER_MODE0_Pos;            // OTG_HS_ULPI_STP

}

void SysTick_init(void)
{
	SysTick->LOAD = 900000;                                       // 5ms
	    SysTick->CTRL |= 1<<SysTick_CTRL_TICKINT_Pos;
		SysTick->CTRL |= 1<<SysTick_CTRL_CLKSOURCE_Pos;
		SysTick->CTRL |= 1<<SysTick_CTRL_ENABLE_Pos;
		NVIC_EnableIRQ(SysTick_IRQn);
}

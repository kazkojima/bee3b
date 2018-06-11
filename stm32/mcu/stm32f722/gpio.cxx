// Copyright (C) 2018 kaz Kojima
//
// This file is part of ef runtime library.  This library is free
// software; you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the
// Free Software Foundation; either version 3, or (at your option)
// any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// Under Section 7 of GPL version 3, you are granted additional
// permissions described in the EF Runtime Library Exception.

// You should have received a copy of the GNU General Public License and
// a copy of the EF Runtime Library Exception along with this program;
// see the files COPYING and EXCEPTION respectively.

/*
 * GPIO init
 */

/*
 * Port A setup.
 * PA0  - INT input
 * PA1  - FSYNC output(Default low)
 * PA2  - UART2 TX(AF7)
 * PA3  - UART2 RX(AF7)
 * PA4  - /CS SPI1(AF5)
 * PA5  - SCLK SPI1(AF5)
 * PA6  - MISO SPI1(AF5)
 * PA7  - MOSI SPI1(AF5)
 * PA13, PA14 - SWD(AF0)
 */
#define GPIOA_MODER   0x2800aaa4 // AF Pin14,13,7-2 Output Pin1 Input Pin0
#define GPIOA_OTYPER  0x00000000 // Push-Pull
#define GPIOA_OSPEEDR 0x0000ff00 // High speed
#define GPIOA_PUPDR   0x65555908 // Pin14,1 pull-down Pin13,0 floating
#define GPIOA_ODR     0x00000000
#define GPIOA_AFR0    0x55557700
#define GPIOA_AFR1    0x00000000 // AF0 Pin14,13

/*
 * Port B setup.
 * PB6  - PWM0 TIM4_CH1(AF2)
 * PB7  - PWM1 TIM4_CH2(AF2)
 * PB8  - PWM2 TIM4_CH3(AF2)
 * PB9  - PWM3 TIM4_CH4(AF2)
 * PB12 - /CS SPI2(AF5)
 * PB13 - SCLK SPI2(AF5)
 * PB14 - MISO SPI2(AF5)
 * PB15 - MOSI SPI2(AF5)
 */
#define GPIOB_MODER   0xaa0aa000 // AF Pin15-12,9-6
#define GPIOB_OTYPER  0x00000000 // Push-Pull
#define GPIOB_OSPEEDR 0xff000000 // High speed
#define GPIOB_PUPDR   0x595aa555 // Pin9-6 pull-down
#define GPIOB_ODR     0x00000000
#define GPIOB_AFR0    0x22000000
#define GPIOB_AFR1    0x55550022

/*
 * Port C setup.
 * PC6  - Notify input
 * PC12 - LED
 */
#define GPIOC_MODER   0x01000000 // Output Pin12 Input Pin6
#define GPIOC_OTYPER  0x00000000 // Push-Pull
#define GPIOC_OSPEEDR 0x00003000 // High speed: Pin6
#define GPIOC_PUPDR   0x54555555 // Pin12 no pull-up
#define GPIOC_ODR     0x00000000
#define GPIOC_AFR0    0x00000000
#define GPIOC_AFR1    0x00000000

/*
 * Port D setup.
 */
#define GPIOD_MODER   0x00000000
#define GPIOD_OTYPER  0x00000000 // Push-Pull
#define GPIOD_OSPEEDR 0x00000000 // High speed
#define GPIOD_PUPDR   0x55555555
#define GPIOD_ODR     0x00000000
#define GPIOD_AFR0    0x00000000
#define GPIOD_AFR1    0x00000000

/*
 * Port E setup.
 */
#define GPIOE_MODER   0x00000000
#define GPIOE_OTYPER  0x00000000 // Push-Pull
#define GPIOE_OSPEEDR 0x00000000
#define GPIOE_PUPDR   0x55555555
#define GPIOE_ODR     0x00000000
#define GPIOE_AFR0    0x00000000
#define GPIOE_AFR1    0x00000000

/*
 * Port F setup.
 */
#define GPIOF_MODER   0x00000000 // no AF
#define GPIOF_OTYPER  0x00000000 // Push-Pull
#define GPIOF_OSPEEDR 0x00000000
#define GPIOF_PUPDR   0x55555555
#define GPIOF_ODR     0x00000000
#define GPIOF_AFR0    0x00000000
#define GPIOF_AFR1    0x00000000

/*
 * Port G setup.
 */
#define GPIOG_MODER   0x00000000 // no AF
#define GPIOG_OTYPER  0x00000000 // Push-Pull
#define GPIOG_OSPEEDR 0x00000000
#define GPIOG_PUPDR   0x51155555 // Pin13,11 floating
#define GPIOG_ODR     0x00000000
#define GPIOG_AFR0    0x00000000
#define GPIOG_AFR1    0x00000000

/*
 * Port H setup.
 */
#define GPIOH_MODER   0x00000000
#define GPIOH_OTYPER  0x00000000 // Push-Pull
#define GPIOH_OSPEEDR 0x00000000
#define GPIOH_PUPDR   0x55555555
#define GPIOH_ODR     0x00000000
#define GPIOH_AFR0    0x00000000
#define GPIOH_AFR1    0x00000000

/*
 * Port I setup.
 */
#define GPIOI_MODER   0x00000000
#define GPIOI_OTYPER  0x00000000 // Push-Pull
#define GPIOI_OSPEEDR 0x00000000
#define GPIOI_PUPDR   0x55555555
#define GPIOI_ODR     0x00000000
#define GPIOI_AFR0    0x00000000
#define GPIOI_AFR1    0x00000000

#define GPIO_LED GPIOC
#define GPIO_LED_SET_TO_EMIT    12

void
ef::mcu::gpio_init (void)
{
  // Then enable GPIO clock.
  RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN
		   | RCC_AHB1ENR_GPIOBEN
		   | RCC_AHB1ENR_GPIOCEN
		   | RCC_AHB1ENR_GPIODEN
		   | RCC_AHB1ENR_GPIOEEN
		   | RCC_AHB1ENR_GPIOFEN
		   | RCC_AHB1ENR_GPIOGEN);
  RCC->AHB1RSTR = (RCC_AHB1RSTR_GPIOARST
		   | RCC_AHB1RSTR_GPIOBRST
		   | RCC_AHB1RSTR_GPIOCRST
		   | RCC_AHB1RSTR_GPIODRST
		   | RCC_AHB1RSTR_GPIOERST
		   | RCC_AHB1RSTR_GPIOFRST
		   | RCC_AHB1RSTR_GPIOGRST);
  RCC->AHB1RSTR = 0;

  GPIOA->AFR[0]  = GPIOA_AFR0;
  GPIOA->AFR[1]  = GPIOA_AFR1;
  GPIOA->OSPEEDR = GPIOA_OSPEEDR;
  GPIOA->OTYPER  = GPIOA_OTYPER;
  GPIOA->ODR     = GPIOA_ODR;
  GPIOA->MODER   = GPIOA_MODER;
  GPIOA->PUPDR   = GPIOA_PUPDR;

  GPIOB->AFR[0]  = GPIOB_AFR0;
  GPIOB->AFR[1]  = GPIOB_AFR1;
  GPIOB->OSPEEDR = GPIOB_OSPEEDR;
  GPIOB->OTYPER  = GPIOB_OTYPER;
  GPIOB->ODR     = GPIOB_ODR;
  GPIOB->MODER   = GPIOB_MODER;
  GPIOB->PUPDR   = GPIOB_PUPDR;

  GPIOC->AFR[0]  = GPIOC_AFR0;
  GPIOC->AFR[1]  = GPIOC_AFR1;
  GPIOC->OSPEEDR = GPIOC_OSPEEDR;
  GPIOC->OTYPER  = GPIOC_OTYPER;
  GPIOC->ODR     = GPIOC_ODR;
  GPIOC->MODER   = GPIOC_MODER;
  GPIOC->PUPDR   = GPIOC_PUPDR;

  GPIOD->AFR[0]  = GPIOD_AFR0;
  GPIOD->AFR[1]  = GPIOD_AFR1;
  GPIOD->OSPEEDR = GPIOD_OSPEEDR;
  GPIOD->OTYPER  = GPIOD_OTYPER;
  GPIOD->ODR     = GPIOD_ODR;
  GPIOD->MODER   = GPIOD_MODER;
  GPIOD->PUPDR   = GPIOD_PUPDR;

  GPIOE->AFR[0]  = GPIOE_AFR0;
  GPIOE->AFR[1]  = GPIOE_AFR1;
  GPIOE->OSPEEDR = GPIOE_OSPEEDR;
  GPIOE->OTYPER  = GPIOE_OTYPER;
  GPIOE->ODR     = GPIOE_ODR;
  GPIOE->MODER   = GPIOE_MODER;
  GPIOE->PUPDR   = GPIOE_PUPDR;

  GPIOF->AFR[0]  = GPIOF_AFR0;
  GPIOF->AFR[1]  = GPIOF_AFR1;
  GPIOF->OSPEEDR = GPIOF_OSPEEDR;
  GPIOF->OTYPER  = GPIOF_OTYPER;
  GPIOF->ODR     = GPIOF_ODR;
  GPIOF->MODER   = GPIOF_MODER;
  GPIOF->PUPDR   = GPIOF_PUPDR;

  GPIOG->AFR[0]  = GPIOG_AFR0;
  GPIOG->AFR[1]  = GPIOG_AFR1;
  GPIOG->OSPEEDR = GPIOG_OSPEEDR;
  GPIOG->OTYPER  = GPIOG_OTYPER;
  GPIOG->ODR     = GPIOG_ODR;
  GPIOG->MODER   = GPIOG_MODER;
  GPIOG->PUPDR   = GPIOG_PUPDR;
}

void
ef::board::set_led (bool on)
{
  if (on)
    GPIO_LED->BSRR = (1 << GPIO_LED_SET_TO_EMIT);
  else
    GPIO_LED->BSRR = (1 << GPIO_LED_SET_TO_EMIT) << 16;
}

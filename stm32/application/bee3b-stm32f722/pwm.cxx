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

#include "ef.h"
#include "mcu.h"

using namespace ef;

#define CR1_UIFREMAP	(1 << 11)
#define CR1_CKD_SHIFT	8
#define CR1_ARPE	(1 << 7)
#define CR1_CMS_SHIFT	5
#define CR1_DIR		(1 << 4)
#define CR1_OPM		(1 << 3)
#define CR1_URS		(1 << 2)
#define CR1_UDIS	(1 << 1)
#define CR1_CEN		(1 << 0)

#define CR2_TI1S	(1 << 7)
#define CR2_MMS_SHIFT	4
#define CR2_CCDS	(1 << 3)

#define SR_CC4OF	(1 << 12)
#define SR_CC3OF	(1 << 11)
#define SR_CC2OF	(1 << 10)
#define SR_CC1OF	(1 << 9)
#define SR_TIF		(1 << 6)
#define SR_CC4IF	(1 << 4)
#define SR_CC3IF	(1 << 3)
#define SR_CC2IF	(1 << 2)
#define SR_CC1IF	(1 << 1)
#define SR_UIF		(1 << 0)

#define EGR_TG		(1 << 6)
#define EGR_CC4G	(1 << 4)
#define EGR_CC3G	(1 << 3)
#define EGR_CC2G	(1 << 2)
#define EGR_CC1G	(1 << 1)
#define EGR_UG		(1 << 0)

#define CCMR1_OC2M3	(1 << 24)
#define CCMR1_OC1M3	(1 << 16)
#define CCMR1_OC2CE	(1 << 15)
#define CCMR1_OC2M_SHIFT 12
#define CCMR1_OC2PE	(1 << 11)
#define CCMR1_OC2FE	(1 << 10)
#define CCMR1_CC2S_SHIFT 8
#define CCMR1_OC1CE	(1 << 7)
#define CCMR1_OC1M_SHIFT 4
#define CCMR1_OC1PE	(1 << 3)
#define CCMR1_OC1FE	(1 << 2)
#define CCMR1_CC1S_SHIFT 0

#define CCMR2_OC4M3	(1 << 24)
#define CCMR2_OC3M3	(1 << 16)
#define CCMR2_OC4CE	(1 << 15)
#define CCMR2_OC4M_SHIFT 12
#define CCMR2_OC4PE	(1 << 11)
#define CCMR2_OC4FE	(1 << 10)
#define CCMR2_CC4S_SHIFT 8
#define CCMR2_OC3CE	(1 << 7)
#define CCMR2_OC3M_SHIFT 4
#define CCMR2_OC3PE	(1 << 3)
#define CCMR2_OC3FE	(1 << 2)
#define CCMR2_CC3S_SHIFT 0

#define CCMR_PWM_MODE1	6
#define CCMR_PWM_MODE2	7

#define CCER_CC4NP	(1 << 15)
#define CCER_CC4P	(1 << 13)
#define CCER_CC4E	(1 << 12)
#define CCER_CC3NP	(1 << 11)
#define CCER_CC3P	(1 << 9)
#define CCER_CC3E	(1 << 8)
#define CCER_CC2NP	(1 << 7)
#define CCER_CC2P	(1 << 5)
#define CCER_CC2E	(1 << 4)
#define CCER_CC1NP	(1 << 3)
#define CCER_CC1P	(1 << 1)
#define CCER_CC1E	(1 << 0)

void
tim4_init (void)
{
  // Enable TIM4 clock.
  RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
  RCC->APB1RSTR = RCC_APB1RSTR_TIM4RST;
  RCC->APB1RSTR = 0;

  // Period 12.9kHz 12-bit precision
  TIM4->PSC = 1;
  TIM4->ARR = 4095;
  TIM4->CCR1 = 0;
  TIM4->CCR2 = 0;
  TIM4->CCR3 = 0;
  TIM4->CCR4 = 0;
  // Auto reload, edge-aligned, upcounting
  TIM4->CR1 = CR1_ARPE;
  // No slave mode, no DMA, no interrupt
  TIM4->CR2 = 0;
  TIM4->SMCR = 0;
  TIM4->DIER = 0;
  TIM4->DCR = 0;
  TIM4->DMAR = 0;
  //
  TIM4->CCER = CCER_CC4E | CCER_CC3E | CCER_CC2E | CCER_CC1E;
  // PWM mode
  TIM4->CCMR1 = (CCMR1_OC2CE
		 | (CCMR_PWM_MODE1 << CCMR1_OC2M_SHIFT)
		 | CCMR1_OC2PE
		 | CCMR1_OC1CE
		 | (CCMR_PWM_MODE1 << CCMR1_OC1M_SHIFT)
		 | CCMR1_OC1PE);

  TIM4->CCMR2 = (CCMR2_OC4CE
		 | (CCMR_PWM_MODE1 << CCMR2_OC4M_SHIFT)
		 | CCMR2_OC4PE
		 | CCMR2_OC3CE
		 | (CCMR_PWM_MODE1 << CCMR2_OC3M_SHIFT)
		 | CCMR2_OC3PE);
  
  TIM4->CR1 |= CR1_CEN;
}

bool
tim4_set_pwm_width (uint16_t *width)
{
  TIM4->CCR1 = width[0];
  TIM4->CCR2 = width[1];
  TIM4->CCR3 = width[2];
  TIM4->CCR4 = width[3];
  return true;
}

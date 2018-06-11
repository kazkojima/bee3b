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
 * RCC init
 */

void
ef::mcu::rcc_init (void)
{
  // This is NOT clock setting but perhaps good time for it.
  SCB->CPACR |= (0xf << 20);
  asm volatile ("dsb\n\t"
		"isb" : : : "memory");
#if defined(USE_FPU_AUTO_SAVE)
  // Auto save/restore fp context on exceptions.
  FPU->FPCCR = (1 << 31);
#else
  // No auto save/restore.
  FPU->FPCCR = 0;
#endif

  // Power clock on
  RCC->APB1ENR = RCC_APB1ENR_PWREN;
  // Setup PWR
  PWR->CR1 = (3 << PWR_CR1_VOS_SHIFT);

  // HSI setup
  RCC->CR |= RCC_CR_HSION;
  while (!(RCC->CR & RCC_CR_HSIRDY))
    ;
  // Reset CR
  RCC->CR = 0x83;
  RCC->CFGR = 0;
  while (((RCC->CFGR >> RCC_CFGR_SWS_SHIFT) & 3) != RCC_CFGR_SWS_HSI)
    ;
  // Reset PLLCFGR
  RCC->PLLCFGR = 0x24003010;
  // Reset PLLI2SCFGR
  RCC->PLLSAICFGR = 0x24003000;
  // Reset PLLSAICFGR
  RCC->PLLSAICFGR = 0x24003000;

  // Disable all interrupts
  RCC->CIR = 0;

  // HSE setup
  RCC->CR |= RCC_CR_HSEON;
  while (!(RCC->CR & RCC_CR_HSERDY))
    ;

  // PLL setup
  RCC->PLLCFGR = (RCC_PLLCFGR_PLLSRC_HSE
		  | (8 << 0)	// PLLM=8
		  | (432 << 6)	// PLLN=432
		  | (0 << 16)	// PLLP=DIV2 0:DIV2 1:DIV4 2:DIV6 3:DIV8
		  | (9 << 24)	// PLLQ=9
		  | (2 << 28));	// Must be kept reset value
  RCC->CR |= RCC_CR_PLLON;

  // Ensure regulator setling
  while ((PWR->CSR1 & PWR_CSR1_VOSRDY) == 0)
    ;
  // Overdrive
  PWR->CR1 |= PWR_CR1_ODEN;
  while (!(PWR->CSR1 & PWR_CSR1_ODRDY))
      ;
  PWR->CR1 |= PWR_CR1_ODSWEN;
  while (!(PWR->CSR1 & PWR_CSR1_ODSWRDY))
      ;

  while (!(RCC->CR & RCC_CR_PLLRDY))
    ;

  RCC->DCKCFGR1 = 0;

  // PLLI2S setup
  RCC->PLLI2SCFGR = ((192 << 6)		// PLLN=192 */
		     | (1 << 16)	// PLLP=DIV4 0:DIV2 1:DIV4 2:DIV6 3:DIV8
		     | (4 << 24)	// PLLQ=4
		     | (2 << 28));	// PLLR=2
  RCC->CR |= RCC_CR_PLLI2SON;
  while (!(RCC->CR & RCC_CR_PLLI2SRDY))
    ;

  // PLLSAI setup
  RCC->PLLSAICFGR = ((192 << 6)		// PLLN=192
		     | (1 << 16)	// PLLP=DIV4 0:DIV2 1:DIV4 2:DIV6 3:DIV8
		     | (4 << 24)	// PLLQ=4
		     | (4 << 28));	// PLLR=4
  RCC->CR |= RCC_CR_PLLSAION;
  while (!(RCC->CR & RCC_CR_PLLSAIRDY))
    ;

  // Clock settings
  RCC->CFGR = ((0 << 30)		// MCO2 0:SYS 1:PLLI2S 2:HSE 3:PLL
	       | (6 << 27)	// MCO2PRE 4:DIV2 5:DIV3 6:DIV4 7:DIV5
	       | (4 << 24)	// MCO1PRE 4:DIV2 5:DIV3 6:DIV4 7:DIV5
	       | (0 << 23)	// I2SSRC 0:PLLI2S 1:I2S_CKIN
	       | (0 << 21)	// MCO1 0:HSI 1:LSE 2:HSE 3:PLL
	       | (0 << 16)	// RTCPRE HSE division factor
	       | (4 << RCC_CFGR_PPRE2_SHIFT)	// 4:DIV2 5:DIV4 6:DIV8 7:DIV16
	       | (5 << RCC_CFGR_PPRE1_SHIFT)
	       | (0 << 4)		// HPRE 0xxx:DIV1 4:DIV2 ... 15:DIV512
	       | (0 << 2)		// SWS read only bits
	       | (0 << 0));	// SW 0:HSI 1:HSE 2:PLL

  RCC->DCKCFGR2 = RCC_DCKCFGR2_CK48MSEL;	// CK48MSEL 0:PLL_Q 1:PLLSAI_P

  /*
   * We don't touch RCC->CR2, RCC->CFGR2, RCC->CFGR3, and RCC->CIR.
   */

  // Flash setup
  FLASH->ACR = (FLASH_ACR_LATENCY_7WS	// 8 CPU cycle wait
		| FLASH_ACR_PRFTEN
		| FLASH_ACR_ARTRST);
  FLASH->ACR = (FLASH_ACR_LATENCY_7WS
		| FLASH_ACR_PRFTEN
		| FLASH_ACR_ARTEN);

  // CRC
  RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;

  // Switching on the configured clock source.
  RCC->CFGR &= ~(3 << RCC_CFGR_SW_SHIFT);
  RCC->CFGR |= (RCC_CFGR_SW_PLL << RCC_CFGR_SW_SHIFT);
  while (((RCC->CFGR >> RCC_CFGR_SW_SHIFT) & 3) != RCC_CFGR_SW_PLL)
    ;

  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
  RCC->APB2RSTR = RCC_APB2RSTR_SYSCFGRST;
  RCC->APB2RSTR = 0;

  // Use vectors on RAM
  // SYSCFG->MEMRMP = (SYSCFG->MEMRMP & ~SYSCFG_MEMRMP_MEM_MODE) | 0x3;

  // Enable caches
  asm volatile ("dsb\n\t"
		"isb" : : : "memory");
  SCB->ICIALLU = 0UL;
  // Enable I-Cache
  SCB->CCR |=  SCB_CCR_IC_MASK;
  asm volatile ("dsb\n\t"
		"isb" : : : "memory");

  uint32_t ccsidr;
  uint32_t sets;
  uint32_t ways;
  // Level 1 data cache
  SCB->CSSELR = (0U << 1U) | 0U;
  asm volatile ("dsb" : : : "memory");
  ccsidr = SCB->CCSIDR;

  // invalidate D-Cache
  sets = (uint32_t)(CCSIDR_SETS(ccsidr));
  do {
    ways = (uint32_t)(CCSIDR_WAYS(ccsidr));
    do {
      SCB->DCISW = SBC_DCISW(sets, ways);
      asm volatile ("" : : : "memory");
    } while (ways--);
  } while(sets--);

  asm volatile ("dsb" : : : "memory");

  // Enable D-Cache
  SCB->CCR |= SCB_CCR_DC_MASK;
  asm volatile ("dsb\n\t"
		"isb" : : : "memory");
}

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

/* Simple STM32F7xx definitions.  Not complete. */

/* Cotex-M7 system controll block definition. */

struct SCB
{
  volatile uint32_t CPUID;
  volatile uint32_t ICSR;
  volatile uint32_t VTOR;
  volatile uint32_t AIRCR;
  volatile uint32_t SCR;
  volatile uint32_t CCR;
  volatile uint8_t  SHP[12];
  volatile uint32_t SHCSR;
  volatile uint32_t CFSR;
  volatile uint32_t HFSR;
  volatile uint32_t DFSR;
  volatile uint32_t MMFAR;
  volatile uint32_t BFAR;
  volatile uint32_t AFSR;
  volatile uint32_t PFR[2];
  volatile uint32_t DFR;
  volatile uint32_t AFR;
  volatile uint32_t MMFR[4];
  volatile uint32_t ISAR[5];
  uint32_t dummy0[1];
  volatile uint32_t CLIDR;
  volatile uint32_t CTR;
  volatile uint32_t CCSIDR;
  volatile uint32_t CSSELR;
  volatile uint32_t CPACR;
  uint32_t dummy3[93];
  volatile uint32_t STIR;
  uint32_t dummy4[15];
  volatile uint32_t MVFR0;
  volatile uint32_t MVFR1;
  volatile uint32_t MVFR2;
  uint32_t dummy5[1];
  volatile uint32_t ICIALLU;
  uint32_t dummy6[1];
  volatile uint32_t ICIMVAU;
  volatile uint32_t DCIMVAC;
  volatile uint32_t DCISW;
  volatile uint32_t DCCMVAU;
  volatile uint32_t DCCMVAC;
  volatile uint32_t DCCSW;
  volatile uint32_t DCCIMVAC;
  volatile uint32_t DCCISW;
  uint32_t dummy7[6];
  volatile uint32_t ITCMCR;
  volatile uint32_t DTCMCR;
  volatile uint32_t AHBPCR;
  volatile uint32_t CACR;
  volatile uint32_t AHBSCR;
  uint32_t dummy8[1];
  volatile uint32_t ABFSR;
};

#define SCB_CCR_DC_MASK		(1 << 16)
#define SCB_CCR_IC_MASK		(1 << 17)

#define CCSIDR_SETS(x)		(((x) >> 13) & 0x7FFF)
#define CCSIDR_WAYS(x)		(((x) >> 3) & 0x3FF)
#define SBC_DCISW(s,w)		((((s) & 0x1ff) << 5)|(((w) & 0x3) << 30))

#define SCS_BASE	(0xE000E000)
#define SCB_BASE	(SCS_BASE +  0x0D00)
static struct SCB *const SCB = ((struct SCB *const) SCB_BASE);

struct FPU
{
  uint32_t dummy0[1];
  volatile uint32_t FPCCR;
  volatile uint32_t FPCAR;
  volatile uint32_t FPDSCR;
  volatile uint32_t MVFR0;
  volatile uint32_t MVFR1;
  volatile uint32_t MVFR2;
};

#define FPU_BASE	(SCS_BASE + 0x0F30)
static struct FPU *const FPU = ((struct FPU *const) FPU_BASE);

/* Some STM32F722 peripheral definitions.  */

#define PERIPH_BASE		0x40000000
#define APB1PERIPH_BASE  	PERIPH_BASE
#define APB2PERIPH_BASE		(PERIPH_BASE + 0x10000)
#define AHB1PERIPH_BASE		(PERIPH_BASE + 0x20000)
#define AHB2PERIPH_BASE		(PERIPH_BASE + 0x10000000)
#define AHB3PERIPH_BASE		(PERIPH_BASE + 0x60000000)

/* RCC */
struct RCC {
  volatile uint32_t CR;
  volatile uint32_t PLLCFGR;
  volatile uint32_t CFGR;
  volatile uint32_t CIR;
  volatile uint32_t AHB1RSTR;
  volatile uint32_t AHB2RSTR;
  volatile uint32_t AHB3RSTR;
  uint32_t dummy0[1];
  volatile uint32_t APB1RSTR;
  volatile uint32_t APB2RSTR;
  uint32_t dummy1[2];
  volatile uint32_t AHB1ENR;
  volatile uint32_t AHB2ENR;
  volatile uint32_t AHB3ENR;
  uint32_t dummy2[1];
  volatile uint32_t APB1ENR;
  volatile uint32_t APB2ENR;
  uint32_t dummy3[2];
  volatile uint32_t AHB1LPENR;
  volatile uint32_t AHB2LPENR;
  volatile uint32_t AHB3LPENR;
  uint32_t dummy4[1];
  volatile uint32_t APB1LPENR;
  volatile uint32_t APB2LPENR;
  uint32_t dummy5[2];
  volatile uint32_t BDCR;
  volatile uint32_t CSR;
  uint32_t dummy6[2];
  volatile uint32_t SSCGR;
  volatile uint32_t PLLI2SCFGR;
  volatile uint32_t PLLSAICFGR;
  volatile uint32_t DCKCFGR1;
  volatile uint32_t DCKCFGR2;
};

#define RCC_CR_HSION		(1 << 0)
#define RCC_CR_HSIRDY		(1 << 1)
#define RCC_CR_HSEON		(1 << 16)
#define RCC_CR_HSERDY		(1 << 17)
#define RCC_CR_CSSON		(1 << 19)
#define RCC_CR_PLLON		(1 << 24)
#define RCC_CR_PLLRDY		(1 << 25)
#define RCC_CR_PLLI2SON		(1 << 26)
#define RCC_CR_PLLI2SRDY	(1 << 27)
#define RCC_CR_PLLSAION		(1 << 28)
#define RCC_CR_PLLSAIRDY	(1 << 29)

#define RCC_PLLCFGR_PLLM_SHIFT	0
#define RCC_PLLCFGR_PLLN_SHIFT	6
#define RCC_PLLCFGR_PLLP_SHIFT	16
#define RCC_PLLCFGR_PLLSRC_HSE	(1 << 22)
#define RCC_PLLCFGR_PLLQ_SHIFT	24
#define RCC_PLLCFGR_PLLR_SHIFT	28

#define RCC_CFGR_SW_SHIFT	0
#define RCC_CFGR_SW_PLL		2
#define RCC_CFGR_SWS_SHIFT	2
#define RCC_CFGR_SWS_HSI	0
#define RCC_CFGR_HPRE_SHIFT	4
#define RCC_CFGR_PPRE1_SHIFT	10
#define RCC_CFGR_PPRE1_DIV4	5
#define RCC_CFGR_PPRE2_SHIFT	13
#define RCC_CFGR_PPRE2_DIV2	4
#define RCC_CFGR_RTCPRE_SHIFT	16
#define RCC_CFGR_MCO1_SHIFT	21
#define RCC_CFGR_I2SSRC_SHIFT	23
#define RCC_CFGR_MCO1PRE_SHIFT	24
#define RCC_CFGR_MCO2PRE_SHIFT	27
#define RCC_CFGR_MCO2_SHIFT	30

#define RCC_AHB1RSTR_GPIOARST	(1 << 0)
#define RCC_AHB1RSTR_GPIOBRST	(1 << 1)
#define RCC_AHB1RSTR_GPIOCRST	(1 << 2)
#define RCC_AHB1RSTR_GPIODRST	(1 << 3)
#define RCC_AHB1RSTR_GPIOERST	(1 << 4)
#define RCC_AHB1RSTR_GPIOFRST	(1 << 5)
#define RCC_AHB1RSTR_GPIOGRST	(1 << 6)
#define RCC_AHB1RSTR_GPIOHRST	(1 << 7)
#define RCC_AHB1RSTR_GPIOIRST	(1 << 8)
#define RCC_AHB1RSTR_GPIOJRST	(1 << 9)
#define RCC_AHB1RSTR_GPIOKRST	(1 << 10)
#define RCC_AHB1RSTR_CRCRST	(1 << 12)
#define RCC_AHB1RSTR_DMA1RST	(1 << 21)
#define RCC_AHB1RSTR_DMA2RST	(1 << 22)
#define RCC_AHB1RSTR_DMA2DRST	(1 << 23)
#define RCC_AHB1RSTR_ETHMACRST	(1 << 25)
#define RCC_AHB1RSTR_OTGHSRST	(1 << 29)

#define RCC_AHB2RSTR_DCMIRST	(1 << 0)
#define RCC_AHB2RSTR_CRYPRST	(1 << 4)
#define RCC_AHB2RSTR_HASHRST	(1 << 5)
#define RCC_AHB2RSTR_RNGRST	(1 << 6)
#define RCC_AHB2RSTR_OTGFSRST	(1 << 7)

#define RCC_AHB3RSTR_FMCRST	(1 << 0)
#define RCC_AHB3RSTR_QSPIRST	(1 << 1)

#define RCC_APB1RSTR_TIM2RST	(1 << 0)
#define RCC_APB1RSTR_TIM3RST	(1 << 1)
#define RCC_APB1RSTR_TIM4RST	(1 << 2)
#define RCC_APB1RSTR_TIM5RST	(1 << 3)
#define RCC_APB1RSTR_TIM6RST	(1 << 4)
#define RCC_APB1RSTR_TIM7RST	(1 << 5)
#define RCC_APB1RSTR_TIM12RST	(1 << 6)
#define RCC_APB1RSTR_TIM13RST	(1 << 7)
#define RCC_APB1RSTR_TIM14RST	(1 << 8)
#define RCC_APB1RSTR_LPTIM1RST	(1 << 9)
#define RCC_APB1RSTR_WWDGRST	(1 << 11)
#define RCC_APB1RSTR_SPI2RST	(1 << 14)
#define RCC_APB1RSTR_SPI3RST	(1 << 15)
#define RCC_APB1RSTR_SPDIFRXRST	(1 << 16)
#define RCC_APB1RSTR_USART2RST	(1 << 17)
#define RCC_APB1RSTR_USART3RST	(1 << 18)
#define RCC_APB1RSTR_UART4RST	(1 << 19)
#define RCC_APB1RSTR_UART5RST	(1 << 20)
#define RCC_APB1RSTR_I2C1RST	(1 << 21)
#define RCC_APB1RSTR_I2C2RST	(1 << 22)
#define RCC_APB1RSTR_I2C3RST	(1 << 23)
#define RCC_APB1RSTR_I2C4RST	(1 << 24)
#define RCC_APB1RSTR_CAN1RST	(1 << 25)
#define RCC_APB1RSTR_CAN2RST	(1 << 26)
#define RCC_APB1RSTR_CECRST	(1 << 27)
#define RCC_APB1RSTR_PWRRST	(1 << 28)
#define RCC_APB1RSTR_DACRST	(1 << 29)
#define RCC_APB1RSTR_UART7RST	(1 << 30)
#define RCC_APB1RSTR_UART85RST	(1 << 31)

#define RCC_APB2RSTR_TIM1RST	(1 << 0)
#define RCC_APB2RSTR_TIM8RST	(1 << 1)
#define RCC_APB2RSTR_USART1RST	(1 << 4)
#define RCC_APB2RSTR_USART6RST	(1 << 5)
#define RCC_APB2RSTR_ADCRST	(1 << 8)
#define RCC_APB2RSTR_SDMMC1RST	(1 << 11)
#define RCC_APB2RSTR_SPI1RST	(1 << 12)
#define RCC_APB2RSTR_SPI4RST	(1 << 13)
#define RCC_APB2RSTR_SYSCFGRST	(1 << 14)
#define RCC_APB2RSTR_TIM9RST	(1 << 16)
#define RCC_APB2RSTR_TIM10RST	(1 << 17)
#define RCC_APB2RSTR_TIM11RST	(1 << 18)
#define RCC_APB2RSTR_SPI5RST	(1 << 20)
#define RCC_APB2RSTR_SPI6RST	(1 << 21)
#define RCC_APB2RSTR_SAI1RST	(1 << 22)
#define RCC_APB2RSTR_SAI2RST	(1 << 23)
#define RCC_APB2RSTR_LTDCRST	(1 << 26)

#define RCC_AHB1ENR_GPIOAEN	(1 << 0)
#define RCC_AHB1ENR_GPIOBEN	(1 << 1)
#define RCC_AHB1ENR_GPIOCEN	(1 << 2)
#define RCC_AHB1ENR_GPIODEN	(1 << 3)
#define RCC_AHB1ENR_GPIOEEN	(1 << 4)
#define RCC_AHB1ENR_GPIOFEN	(1 << 5)
#define RCC_AHB1ENR_GPIOGEN	(1 << 6)
#define RCC_AHB1ENR_GPIOHEN	(1 << 7)
#define RCC_AHB1ENR_GPIOIEN	(1 << 8)
#define RCC_AHB1ENR_GPIOJEN	(1 << 9)
#define RCC_AHB1ENR_GPIOKEN	(1 << 10)
#define RCC_AHB1ENR_CRCEN	(1 << 12)
#define RCC_AHB1ENR_BKPSRAMEN	(1 << 18)
#define RCC_AHB1ENR_DTCMRAMEN	(1 << 20)
#define RCC_AHB1ENR_DMA1EN	(1 << 21)
#define RCC_AHB1ENR_DMA2EN	(1 << 22)
#define RCC_AHB1ENR_DMA2DEN	(1 << 23)
#define RCC_AHB1ENR_ETHMACEN	(1 << 25)
#define RCC_AHB1ENR_ETHMACTXEN	(1 << 26)
#define RCC_AHB1ENR_ETHMACRXEN	(1 << 27)
#define RCC_AHB1ENR_ETHMACPTPEN	(1 << 28)
#define RCC_AHB1ENR_OTGHSEN	(1 << 29)
#define RCC_AHB1ENR_OTGHSULPIEN	(1 << 30)

#define RCC_AHB2ENR_DCMIEN	(1 << 0)
#define RCC_AHB2ENR_CRYPEN	(1 << 4)
#define RCC_AHB2ENR_HASHEN	(1 << 5)
#define RCC_AHB2ENR_RNGEN	(1 << 6)
#define RCC_AHB2ENR_OTGFSEN	(1 << 7)

#define RCC_AHB3ENR_FMCEN	(1 << 0)
#define RCC_AHB3ENR_QSPIEN	(1 << 1)

#define RCC_APB1ENR_TIM2EN	(1 << 0)
#define RCC_APB1ENR_TIM3EN	(1 << 1)
#define RCC_APB1ENR_TIM4EN	(1 << 2)
#define RCC_APB1ENR_TIM5EN	(1 << 3)
#define RCC_APB1ENR_TIM6EN	(1 << 4)
#define RCC_APB1ENR_TIM7EN	(1 << 5)
#define RCC_APB1ENR_TIM12EN	(1 << 6)
#define RCC_APB1ENR_TIM13EN	(1 << 7)
#define RCC_APB1ENR_TIM14EN	(1 << 8)
#define RCC_APB1ENR_LPTIM1EN	(1 << 9)
#define RCC_APB1ENR_WWDGEN	(1 << 11)
#define RCC_APB1ENR_SPI2EN	(1 << 14)
#define RCC_APB1ENR_SPI3EN	(1 << 15)
#define RCC_APB1ENR_SPDIFRXEN	(1 << 16)
#define RCC_APB1ENR_USART2EN	(1 << 17)
#define RCC_APB1ENR_USART3EN	(1 << 18)
#define RCC_APB1ENR_UART4EN	(1 << 19)
#define RCC_APB1ENR_UART5EN	(1 << 20)
#define RCC_APB1ENR_I2C1EN	(1 << 21)
#define RCC_APB1ENR_I2C2EN	(1 << 22)
#define RCC_APB1ENR_I2C3EN	(1 << 23)
#define RCC_APB1ENR_I2C4EN	(1 << 24)
#define RCC_APB1ENR_CAN1EN	(1 << 25)
#define RCC_APB1ENR_CAN2EN	(1 << 26)
#define RCC_APB1ENR_CECEN	(1 << 27)
#define RCC_APB1ENR_PWREN	(1 << 28)
#define RCC_APB1ENR_DACEN	(1 << 29)
#define RCC_APB1ENR_UART7EN	(1 << 30)
#define RCC_APB1ENR_UART85EN	(1 << 31)

#define RCC_APB2ENR_TIM1EN	(1 << 0)
#define RCC_APB2ENR_TIM8EN	(1 << 1)
#define RCC_APB2ENR_USART1EN	(1 << 4)
#define RCC_APB2ENR_USART6EN	(1 << 5)
#define RCC_APB2ENR_ADC1EN	(1 << 8)
#define RCC_APB2ENR_ADC2EN	(1 << 9)
#define RCC_APB2ENR_ADC3EN	(1 << 10)
#define RCC_APB2ENR_SDMMC1EN	(1 << 11)
#define RCC_APB2ENR_SPI1EN	(1 << 12)
#define RCC_APB2ENR_SPI4EN	(1 << 13)
#define RCC_APB2ENR_SYSCFGEN	(1 << 14)
#define RCC_APB2ENR_TIM9EN	(1 << 16)
#define RCC_APB2ENR_TIM10EN	(1 << 17)
#define RCC_APB2ENR_TIM11EN	(1 << 18)
#define RCC_APB2ENR_SPI5EN	(1 << 20)
#define RCC_APB2ENR_SPI6EN	(1 << 21)
#define RCC_APB2ENR_SAI1EN	(1 << 22)
#define RCC_APB2ENR_SAI2EN	(1 << 23)
#define RCC_APB2ENR_LTDCEN	(1 << 26)

#define RCC_BDCR_LSEON		(1 << 0)
#define RCC_BDCR_LSERDY		(1 << 1)
#define RCC_BDCR_LSEBYP		(1 << 2)
#define RCC_BDCR_LSEDRV_SHIFT	3
#define RCC_BDCR_RTCSEL_SHIFT	8
#define RCC_BDCR_RTCEN		(1 << 15)
#define RCC_BDCR_BDRST		(1 << 16)

#define RCC_CSR_LSION		(1 << 0)
#define RCC_CSR_LSIRDY		(1 << 1)
#define RCC_CSR_RMVF		(1 << 24)
#define RCC_CSR_BORRSTF		(1 << 25)
#define RCC_CSR_PINRSTF		(1 << 26)
#define RCC_CSR_PORRSTF		(1 << 27)
#define RCC_CSR_SFTRSTF		(1 << 28)
#define RCC_CSR_IWDGRSTF	(1 << 29)
#define RCC_CSR_WWDGRSTF	(1 << 30)
#define RCC_CSR_LPWRRSTF	(1 << 31)

#define RCC_SSCGR_MODPER_SHIFT	0
#define RCC_SSCGR_INCSTEP_SHIFT	13
#define RCC_SSGCR_SPREDSEL	(1 << 30)
#define RCC_SSGCR_SSCGEN	(1 << 31)

// PLL for I2S and SAI
#define RCC_PLLXCFGR_PLLN_SHIFT	6
#define RCC_PLLXCFGR_PLLP_SHIFT	16
#define RCC_PLLXCFGR_PLLQ_SHIFT	24
#define RCC_PLLXCFGR_PLLR_SHIFT	28

#define RCC_DCKCFGR1_PLLI2SDIVQ_SHIFT	0
#define RCC_DCKCFGR1_PLLSAIDIVQ_SHIFT	8
#define RCC_DCKCFGR1_PLLSAIDIVR_SHIFT	16
#define RCC_DCKCFGR1_PLLSAI1SEL_SHIFT	20
#define RCC_DCKCFGR1_PLLSAI2SEL_SHIFT	22
#define RCC_DCKCFGR1_TIMPRE		(1 << 24)

#define RCC_DCKCFGR2_UART1SEL_SHIFT	0
#define RCC_DCKCFGR2_UART2SEL_SHIFT	2
#define RCC_DCKCFGR2_UART3SEL_SHIFT	4
#define RCC_DCKCFGR2_UART4SEL_SHIFT	6
#define RCC_DCKCFGR2_UART5SEL_SHIFT	8
#define RCC_DCKCFGR2_UART6SEL_SHIFT	10
#define RCC_DCKCFGR2_UART7SEL_SHIFT	12
#define RCC_DCKCFGR2_UART8SEL_SHIFT	14
#define RCC_DCKCFGR2_I2C1SEL_SHIFT	16
#define RCC_DCKCFGR2_I2C2SEL_SHIFT	18
#define RCC_DCKCFGR2_I2C3SEL_SHIFT	20
#define RCC_DCKCFGR2_I2C4SEL_SHIFT	22
#define RCC_DCKCFGR2_LPTIM1SEL_SHIFT	24
#define RCC_DCKCFGR2_CECSEL		(1 << 26)
#define RCC_DCKCFGR2_CK48MSEL		(1 << 27)
#define RCC_DCKCFGR2_SDMMC1SEL		(1 << 28)

/* SYSCFG */
struct SYSCFG {
  volatile uint32_t MEMRMP;
  volatile uint32_t PMC;
  volatile uint32_t EXTICR[4];
  uint32_t dummy0[2];
  volatile uint32_t CMPCR;
};

#define SYSCFG_MEMRMP_MEM_BOOT 0x1

/* EXTI */
struct EXTI {
  volatile uint32_t IMR;
  volatile uint32_t EMR;
  volatile uint32_t RTSR;
  volatile uint32_t FTSR;
  volatile uint32_t SWIER;
  volatile uint32_t PR;
};

/* POWER */
struct PWR {
  volatile uint32_t CR1;
  volatile uint32_t CSR1;
  volatile uint32_t CR2;
  volatile uint32_t CSR2;
};

#define PWR_CR1_CSBF		(1 << 3)
#define PWR_CR1_DBP		(1 << 8)
#define PWR_CR1_VOS_SHIFT	14
#define PWR_CR1_ODEN		(1 << 16)
#define PWR_CR1_ODSWEN		(1 << 17)

#define PWR_CSR1_VOSRDY		(1 << 14)
#define PWR_CSR1_ODRDY		(1 << 16)
#define PWR_CSR1_ODSWRDY	(1 << 17)

/* FLASH */
struct FLASH {
  volatile uint32_t ACR;
  volatile uint32_t KEYR;
  volatile uint32_t OPTKEYR;
  volatile uint32_t SR;
  volatile uint32_t CR;
  volatile uint32_t AR;
  volatile uint32_t RESERVED;
  volatile uint32_t OBR;
  volatile uint32_t WRPR;
};

#define FLASH_ACR_LATENCY_7WS	7
#define FLASH_ACR_PRFTEN	(1 << 8)
#define FLASH_ACR_ARTEN		(1 << 9)
#define FLASH_ACR_ARTRST	(1 << 11)

/* GPIO */
struct GPIO {
  volatile uint32_t MODER;
  volatile uint32_t OTYPER;
  volatile uint32_t OSPEEDR;
  volatile uint32_t PUPDR;
  volatile uint32_t IDR;
  volatile uint32_t ODR;
  volatile uint32_t BSRR;
  volatile uint32_t LCKR;
  volatile uint32_t AFR[2];
};

/* Timer */
struct TIM
{
  volatile uint32_t CR1;
  volatile uint32_t CR2;
  volatile uint32_t SMCR;
  volatile uint32_t DIER;
  volatile uint32_t SR;
  volatile uint32_t EGR;
  volatile uint32_t CCMR1;
  volatile uint32_t CCMR2;
  volatile uint32_t CCER;
  volatile uint32_t CNT;
  volatile uint32_t PSC;
  volatile uint32_t ARR;
  volatile uint32_t RCR;
  volatile uint32_t CCR1;
  volatile uint32_t CCR2;
  volatile uint32_t CCR3;
  volatile uint32_t CCR4;
  volatile uint32_t BDTR;
  volatile uint32_t DCR;
  volatile uint32_t DMAR;
  uint32_t dummy0[1];
  volatile uint32_t CCMR3;
  volatile uint32_t CCMR5;
  volatile uint32_t CCMR6;
};

/* USART */
struct USART
{
  volatile uint32_t CR1;
  volatile uint32_t CR2;
  volatile uint32_t CR3;
  volatile uint32_t BRR;
  volatile uint32_t GTPR;
  volatile uint32_t RTOR;
  volatile uint32_t RQR;
  volatile uint32_t ISR;
  volatile uint32_t ICR;
  volatile uint32_t RDR;
  volatile uint32_t TDR;
};

/* bxCAN */
struct TXMB
{
  volatile uint32_t TIR;
  volatile uint32_t TDTR;
  volatile uint32_t TDLR;
  volatile uint32_t TDHR;
};

struct RXMB
{
  volatile uint32_t RIR;
  volatile uint32_t RDTR;
  volatile uint32_t RDLR;
  volatile uint32_t RDHR;
};

struct FILT
{
  volatile uint32_t FR1;
  volatile uint32_t FR2;
};

struct CAN
{
  volatile uint32_t MCR;
  volatile uint32_t MSR;
  volatile uint32_t TSR;
  volatile uint32_t RF0R;
  volatile uint32_t RF1R;
  volatile uint32_t IER;
  volatile uint32_t ESR;
  volatile uint32_t BTR;
  uint32_t dummy0[88];
  struct TXMB TXMB[3];
  struct RXMB RXMB[2];
  uint32_t dummy1[12];
  volatile uint32_t FMR;
  volatile uint32_t FM1R;
  uint32_t dummy2;
  volatile uint32_t FS1R;
  uint32_t dummy3;
  volatile uint32_t FFA1R;
  uint32_t dummy4;
  volatile uint32_t FA1R;
  uint32_t dummy5[8];
  struct FILT BANK[28];
};

/* SPI/I2S */
struct SPI
{
  volatile uint32_t CR1;
  volatile uint32_t CR2;
  volatile uint32_t SR;
  volatile uint32_t DR;
  volatile uint32_t CRCPR;
  volatile uint32_t RXCRCR;
  volatile uint32_t TXCRCR;
  volatile uint32_t I2SCFGR;
  volatile uint32_t I2SPR;
};

/* I2C */

struct I2C
{
  volatile uint32_t CR1;
  volatile uint32_t CR2;
  volatile uint32_t OAR1;
  volatile uint32_t OAR2;
  volatile uint32_t DR;
  volatile uint32_t SR1;
  volatile uint32_t SR2;
  volatile uint32_t CCR;
  volatile uint32_t TRISE;
  volatile uint32_t FLTR;
};

/* ADC */

struct ADC
{
  volatile uint32_t SR;
  volatile uint32_t CR1;
  volatile uint32_t CR2;
  volatile uint32_t SMPR1;
  volatile uint32_t SMPR2;
  volatile uint32_t JOFR1;
  volatile uint32_t JOFR2;
  volatile uint32_t JOFR3;
  volatile uint32_t JOFR4;
  volatile uint32_t HTR;
  volatile uint32_t LTR;
  volatile uint32_t SQR1;
  volatile uint32_t SQR2;
  volatile uint32_t SQR3;
  volatile uint32_t JSQR;
  volatile uint32_t JDR1;
  volatile uint32_t JDR2;
  volatile uint32_t JDR3;
  volatile uint32_t JDR4;
  volatile uint32_t DR;
};

/* common ADC registers */

struct ADCC
{
  volatile uint32_t CSR;
  volatile uint32_t CCR;
  volatile uint32_t CDR;
};

#if 0
/* ethernet MAC */

struct ETH
{
  volatile uint32_t MACCR;
  volatile uint32_t MACFFR;
  volatile uint32_t MACHTHR;
  volatile uint32_t MACHTLR;
  volatile uint32_t MACMIIAR;
  volatile uint32_t MACMIIDR;
  volatile uint32_t MACFCR;
  volatile uint32_t MACVLANTR;
  uint32_t dummy0[2];
  volatile uint32_t MACRWUFFR;
  volatile uint32_t MACPMTCSR;
  uint32_t dummy1[2];
  volatile uint32_t MACSR;
  volatile uint32_t MACIMR;
  volatile uint32_t MACA0HR;
  volatile uint32_t MACA0LR;
  volatile uint32_t MACA1HR;
  volatile uint32_t MACA1LR;
  volatile uint32_t MACA2HR;
  volatile uint32_t MACA2LR;
  volatile uint32_t MACA3HR;
  volatile uint32_t MACA3LR;
  uint32_t dummy2[40];
  volatile uint32_t MMCCR;
  volatile uint32_t MMCRIR;
  volatile uint32_t MMCTIR;
  volatile uint32_t MMCRIMR;
  volatile uint32_t MMCTIMR;
  uint32_t dummy3[14];
  volatile uint32_t MMCTGFSCCR;
  volatile uint32_t MMCTGFMSCCR;
  uint32_t dummy4[5];
  volatile uint32_t MMCTGFCR;
  uint32_t dummy5[10];
  volatile uint32_t MMCRFCECR;
  volatile uint32_t MMCRFAECR;
  uint32_t dummy6[10];
  volatile uint32_t MMCRGUFCR;
  uint32_t dummy7[334];
  volatile uint32_t PTPTSCR;
  volatile uint32_t PTPSSIR;
  volatile uint32_t PTPTSHR;
  volatile uint32_t PTPTSLR;
  volatile uint32_t PTPTSHUR;
  volatile uint32_t PTPTSLUR;
  volatile uint32_t PTPTSAR;
  volatile uint32_t PTPTTHR;
  volatile uint32_t PTPTTLR;
  volatile uint32_t dummy8;
  volatile uint32_t PTPTSSR;
  uint32_t dummy9[565];
  volatile uint32_t DMABMR;
  volatile uint32_t DMATPDR;
  volatile uint32_t DMARPDR;
  volatile uint32_t DMARDLAR;
  volatile uint32_t DMATDLAR;
  volatile uint32_t DMASR;
  volatile uint32_t DMAOMR;
  volatile uint32_t DMAIER;
  volatile uint32_t DMAMFBOCR;
  volatile uint32_t DMARSWTR;
  uint32_t dummy10[8];
  volatile uint32_t DMACHTDR;
  volatile uint32_t DMACHRDR;
  volatile uint32_t DMACHTBAR;
  volatile uint32_t DMACHRBAR;
};
#endif

/* DMA */

struct DMA
{
  volatile uint32_t LISR;
  volatile uint32_t HISR;
  volatile uint32_t LIFCR;
  volatile uint32_t HIFCR;
};

#define DMA_TCIF3		(1 << 27)
#define DMA_HTIF3		(1 << 26)
#define DMA_TEIF3		(1 << 25)
#define DMA_DMEIF3		(1 << 24)
#define DMA_FEIF3		(1 << 22)
#define DMA_TCIF2		(1 << 21)
#define DMA_HTIF2		(1 << 20)
#define DMA_TEIF2		(1 << 19)
#define DMA_DMEIF2		(1 << 18)
#define DMA_FEIF2		(1 << 16)
#define DMA_TCIF1		(1 << 11)
#define DMA_HTIF1		(1 << 10)
#define DMA_TEIF1		(1 << 9)
#define DMA_DMEIF1		(1 << 8)
#define DMA_FEIF1		(1 << 6)
#define DMA_TCIF0		(1 << 5)
#define DMA_HTIF0		(1 << 4)
#define DMA_TEIF0		(1 << 3)
#define DMA_DMEIF0		(1 << 2)
#define DMA_FEIF0		(1 << 0)

#define DMA_TCIF7		(1 << 27)
#define DMA_HTIF7		(1 << 26)
#define DMA_TEIF7		(1 << 25)
#define DMA_DMEIF7		(1 << 24)
#define DMA_FEIF7		(1 << 22)
#define DMA_TCIF6		(1 << 21)
#define DMA_HTIF6		(1 << 20)
#define DMA_TEIF6		(1 << 19)
#define DMA_DMEIF6		(1 << 18)
#define DMA_FEIF6		(1 << 16)
#define DMA_TCIF5		(1 << 11)
#define DMA_HTIF5		(1 << 10)
#define DMA_TEIF5		(1 << 9)
#define DMA_DMEIF5		(1 << 8)
#define DMA_FEIF5		(1 << 6)
#define DMA_TCIF4		(1 << 5)
#define DMA_HTIF4		(1 << 4)
#define DMA_TEIF4		(1 << 3)
#define DMA_DMEIF4		(1 << 2)
#define DMA_FEIF4		(1 << 0)

/* DMA stream */
struct DMA_Stream
{
  volatile uint32_t CR;
  volatile uint32_t NDTR;
  volatile uint32_t PAR;
  volatile uint32_t M0AR;
  volatile uint32_t M1AR;
  volatile uint32_t FCR;
};

#define DMA_SxCR_CHSEL_shift	25
#define DMA_SxCR_MBURST_shift	23
#define DMA_SxCR_PBURST_shift	21
#define DMA_SxCR_BURST_SINGLE	0
#define DMA_SxCR_BURST_INCR4	1
#define DMA_SxCR_BURST_INCR8	2
#define DMA_SxCR_BURST_INCR16	3
#define DMA_SxCR_CT		(1 << 19)
#define DMA_SxCR_DBM		(1 << 18)
#define DMA_SxCR_PL_shift	16
#define DMA_SxCR_PL_LOW		0
#define DMA_SxCR_PL_MEDIUM	1
#define DMA_SxCR_PL_HIGH	2
#define DMA_SxCR_PL_VHIGH	3
#define DMA_SxCR_PINCOS		(1 << 15)
#define DMA_SxCR_MSIZE_shift	13
#define DMA_SxCR_PSIZE_shift	11
#define DMA_SxCR_SIZE_BYTE	0
#define DMA_SxCR_SIZE_HALF	1
#define DMA_SxCR_SIZE_WORD	2
#define DMA_SxCR_MINC		(1 << 10)
#define DMA_SxCR_PINC		(1 << 9)
#define DMA_SxCR_CIRC		(1 << 8)
#define DMA_SxCR_DIR_shift	6
#define DMA_SxCR_DIR_P2M	0
#define DMA_SxCR_DIR_M2P	1
#define DMA_SxCR_DIR_M2M	2
#define DMA_SxCR_PFCTRL		(1 << 5)
#define DMA_SxCR_TCIE		(1 << 4)
#define DMA_SxCR_HTIE		(1 << 3)
#define DMA_SxCR_TEIE		(1 << 2)
#define DMA_SxCR_DMEIE		(1 << 1)
#define DMA_SxCR_EN		(1 << 0)

#define DMA_SxFCR_FEIE		(1 << 7)
#define DMA_SxFCR_FS_shift	3
#define DMA_SxFCR_FS_1BY4	0
#define DMA_SxFCR_FS_1BY2	1
#define DMA_SxFCR_FS_3BY4	2
#define DMA_SxFCR_FS_ALMOST	3
#define DMA_SxFCR_FS_ENMPT	4
#define DMA_SxFCR_FS_FULL	5
#define DMA_SxFCR_DMDIS		(1 << 2)
#define DMA_SxFCR_FTH_shift	0
#define DMA_SxFCR_FTH_1BY4	0
#define DMA_SxFCR_FTH_1BY2	1
#define DMA_SxFCR_FTH_3BY4	2
#define DMA_SxFCR_FTH_FULL	3

/* Physical addresses */
#define RCC_BASE		(AHB1PERIPH_BASE + 0x3800)
#define SYSCFG_BASE		(APB2PERIPH_BASE + 0x3800)
#define EXTI_BASE		(APB2PERIPH_BASE + 0x3C00)
#define PWR_BASE		(APB1PERIPH_BASE + 0x7000)
#define FLASH_R_BASE		(AHB1PERIPH_BASE + 0x3C00)
#define GPIOA_BASE		(AHB1PERIPH_BASE + 0x0000)
#define GPIOB_BASE		(AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASE		(AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASE		(AHB1PERIPH_BASE + 0x0C00)
#define GPIOE_BASE		(AHB1PERIPH_BASE + 0x1000)
#define GPIOF_BASE		(AHB1PERIPH_BASE + 0x1400)
#define GPIOG_BASE		(AHB1PERIPH_BASE + 0x1800)
#define GPIOH_BASE		(AHB1PERIPH_BASE + 0x1C00)
#define GPIOI_BASE		(AHB1PERIPH_BASE + 0x2000)
#define GPIOJ_BASE		(AHB1PERIPH_BASE + 0x2400)
#define GPIOK_BASE		(AHB1PERIPH_BASE + 0x2800)
#define TIM2_BASE		(APB1PERIPH_BASE + 0x0000)
#define TIM3_BASE		(APB1PERIPH_BASE + 0x0400)
#define TIM4_BASE		(APB1PERIPH_BASE + 0x0800)
#define TIM5_BASE		(APB1PERIPH_BASE + 0x0C00)
#define TIM6_BASE		(APB1PERIPH_BASE + 0x1000)
#define TIM7_BASE		(APB1PERIPH_BASE + 0x1400)
#define USART1_BASE		(APB2PERIPH_BASE + 0x1000)
#define USART2_BASE		(APB1PERIPH_BASE + 0x4400)
#define USART3_BASE		(APB1PERIPH_BASE + 0x4800)
#define UART4_BASE		(APB1PERIPH_BASE + 0x4C00)
#define UART5_BASE		(APB1PERIPH_BASE + 0x5000)
#define USART6_BASE		(APB2PERIPH_BASE + 0x1400)
#define CAN1_BASE		(APB1PERIPH_BASE + 0x6400)
#define CAN2_BASE		(APB1PERIPH_BASE + 0x6800)
#define SPI1_BASE		(APB2PERIPH_BASE + 0x3000)
#define SPI2_BASE		(APB1PERIPH_BASE + 0x3800)
#define SPI3_BASE		(APB1PERIPH_BASE + 0x3C00)
#define I2C1_BASE		(APB1PERIPH_BASE + 0x5400)
#define I2C2_BASE		(APB1PERIPH_BASE + 0x5800)
#define I2C3_BASE		(APB1PERIPH_BASE + 0x5C00)
#define ADC1_BASE		(APB2PERIPH_BASE + 0x2000)
#define ADC2_BASE		(APB2PERIPH_BASE + 0x2100)
#define ADC3_BASE		(APB2PERIPH_BASE + 0x2200)
#define ADCC_BASE		(APB2PERIPH_BASE + 0x2300)
#define DMA1_BASE		(AHB1PERIPH_BASE + 0x6000)
#define DMA2_BASE		(AHB1PERIPH_BASE + 0x6400)
#if 0
#define ETH_BASE		(AHB1PERIPH_BASE + 0x8000)
#define ETH_MAC_BASE		(ETH_BASE)
#define ETH_MMC_BASE		(ETH_BASE + 0x0100U)
#define ETH_PTP_BASE		(ETH_BASE + 0x0700U)
#define ETH_DMA_BASE		(ETH_BASE + 0x1000U)
#endif

static struct RCC *const RCC = ((struct RCC *const)RCC_BASE);
static struct SYSCFG *const SYSCFG = ((struct SYSCFG *const) SYSCFG_BASE);
static struct EXTI *const EXTI = ((struct EXTI *const) EXTI_BASE);
static struct PWR *const PWR = ((struct PWR *const) PWR_BASE);
static struct FLASH *const FLASH = ((struct FLASH *const) FLASH_R_BASE);
static struct GPIO *const GPIOA = ((struct GPIO *const) GPIOA_BASE);
static struct GPIO *const GPIOB = ((struct GPIO *const) GPIOB_BASE);
static struct GPIO *const GPIOC = ((struct GPIO *const) GPIOC_BASE);
static struct GPIO *const GPIOD = ((struct GPIO *const) GPIOD_BASE);
static struct GPIO *const GPIOE = ((struct GPIO *const) GPIOE_BASE);
static struct GPIO *const GPIOF = ((struct GPIO *const) GPIOF_BASE);
static struct GPIO *const GPIOG = ((struct GPIO *const) GPIOG_BASE);
static struct TIM *const TIM2 = ((struct TIM *const) TIM2_BASE);
static struct TIM *const TIM3 = ((struct TIM *const) TIM3_BASE);
static struct TIM *const TIM4 = ((struct TIM *const) TIM4_BASE);
static struct TIM *const TIM5 = ((struct TIM *const) TIM5_BASE);
static struct TIM *const TIM6 = ((struct TIM *const) TIM6_BASE);
static struct TIM *const TIM7 = ((struct TIM *const) TIM7_BASE);
static struct USART *const USART1 = ((struct USART *const) USART1_BASE);
static struct USART *const USART2 = ((struct USART *const) USART2_BASE);
static struct USART *const USART3 = ((struct USART *const) USART3_BASE);
static struct USART *const UART4 = ((struct USART *const) UART4_BASE);
static struct USART *const UART5 = ((struct USART *const) UART5_BASE);
static struct USART *const USART6 = ((struct USART *const) USART6_BASE);
static struct CAN *const CAN1 = ((struct CAN *const) CAN1_BASE);
static struct CAN *const CAN2 = ((struct CAN *const) CAN2_BASE);
static struct SPI *const I2S1 = ((struct SPI *const) SPI1_BASE);
static struct SPI *const I2S2 = ((struct SPI *const) SPI2_BASE);
static struct SPI *const I2S3 = ((struct SPI *const) SPI3_BASE);
static struct I2C *const I2C1 = ((struct I2C *const) I2C1_BASE);
static struct I2C *const I2C2 = ((struct I2C *const) I2C2_BASE);
static struct I2C *const I2C3 = ((struct I2C *const) I2C3_BASE);
static struct ADC *const ADC1 = ((struct ADC *const) ADC1_BASE);
static struct ADC *const ADC2 = ((struct ADC *const) ADC2_BASE);
static struct ADC *const ADC3 = ((struct ADC *const) ADC3_BASE);
static struct ADCC *const ADCC = ((struct ADCC *const) ADCC_BASE);
#if 0
static struct ETH *const ETH = ((struct ETH *const) ETH_BASE);
#endif
static struct DMA *const DMA1 = ((struct DMA *const) DMA1_BASE);
static struct DMA *const DMA2 = ((struct DMA *const) DMA2_BASE);
static struct DMA_Stream *const DMA1_Stream __attribute__((unused)) =
  ((struct DMA_Stream *const) (DMA1_BASE + sizeof (struct DMA)));
static struct DMA_Stream *const DMA2_Stream __attribute__((unused)) =
  ((struct DMA_Stream *const) (DMA2_BASE + sizeof (struct DMA)));

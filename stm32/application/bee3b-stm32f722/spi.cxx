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

#define INTR_REQ_SPI1		35
// SPI1_RX DMA2 CH3 Stream0
#define INTR_REQ_DMA2_Stream0	56
// SPI2_TX DMA2 CH3 Stream3
#define INTR_REQ_DMA2_Stream3	59

#define INTR_REQ_SPI2		36
// SPI2_RX DMA1 CH0 Stream3
#define INTR_REQ_DMA1_Stream3	14
// SPI2_TX DMA1 CH0 Stream4
#define INTR_REQ_DMA1_Stream4	15

#define CR1_DFF		(1 << 11)
#define CR1_SSM		(1 << 9)
#define CR1_SPE		(1 << 6)
#define CR1_BR_SHIFT	3
#define CR1_MSTR	(1 << 2)
#define CR1_CPOL	(1 << 1)
#define CR1_CPHA	(1 << 0)

#define CR2_RXDMAEN	(1 << 0)
#define CR2_TXDMAEN	(1 << 1)
#define CR2_SSOE	(1 << 2)
#define CR2_NSSP	(1 << 3)
#define CR2_ERRIE	(1 << 5)
#define CR2_RXNEIE	(1 << 6)
#define CR2_TXEIE	(1 << 7)
#define CR2_DS_SHIFT	8
#define CR2_FRXTH	(1 << 12)

#define SR_RXNE		(1 << 0)
#define SR_TXE		(1 << 1)
#define SR_CHSIDE	(1 << 2)
#define SR_UDR		(1 << 3)
#define SR_OVR		(1 << 6)
#define SR_BSY		(1 << 7)
#define SR_FRE		(1 << 8)
#define SR_FRLVL_SHIFT	9
#define SR_FTLVL_SHIFT	11

#define CFGR_CHLEN	(1 << 0)
#define CFGR_DATALEN_16	(0 << 1)
#define CFGR_DATALEN_24	(1 << 1)
#define CFGR_DATALEN_32	(2 << 1)
#define CFGR_CKPOL	(1 << 3)
#define CFGR_I2SSTD_I2S	(0 << 4)
#define CFGR_I2SSTD_MSB	(1 << 4)
#define CFGR_I2SSTD_LSB	(2 << 4)
#define CFGR_I2SSTD_PCM	(3 << 4)
#define CFGR_PCMSYNC	(1 << 7)
#define CFGR_RECIEVE	(1 << 8)
#define CFGR_MASTER	(1 << 9)
#define CFGR_I2SE	(1 << 10)
#define CFGR_I2SMOD	(1 << 11)
#define CFGR_ASTREN	(1 << 12)

#define PR_ODD		(1 << 8)
#define PR_MCKOE	(1 << 9)

static struct SPI *const SPI1 = ((struct SPI *const) SPI1_BASE);
static struct SPI *const SPI2 = ((struct SPI *const) SPI2_BASE);

void
spi_init (void)
{
  // Enable DMA2
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
  RCC->AHB1RSTR = RCC_AHB1RSTR_DMA2RST;
  RCC->AHB1RSTR = 0;

  // Enable SPI1 clock.
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
  RCC->APB2RSTR |= RCC_APB2RSTR_SPI1RST;
  RCC->APB2RSTR &=~ RCC_APB2RSTR_SPI1RST;

  // BR[1:0]: 3 (pclk/16), Main
  SPI1->CR1 = (3 << CR1_BR_SHIFT)| CR1_MSTR;
  // 8-bit, NSS asserted when SPI is enabled
  SPI1->CR2 = CR2_FRXTH | (7 << CR2_DS_SHIFT) | CR2_SSOE;

  // SPI1_RX: DMA2 stream 0 channel 3
  // SPI1_TX: DMA2 stream 3 channel 3
  // Stop stream first
  DMA2_Stream[0].CR &= ~DMA_SxCR_EN;
  while (DMA2_Stream[0].CR & DMA_SxCR_EN)
    ;
  DMA2_Stream[3].CR &= ~DMA_SxCR_EN;
  while (DMA2_Stream[3].CR & DMA_SxCR_EN)
    ;
  DMA2->LIFCR = (DMA2->LISR
		 & (DMA_TCIF3|DMA_HTIF3|DMA_TEIF3|DMA_DMEIF3|DMA_FEIF3));
  DMA2_Stream[0].PAR = (uint32_t) (void *) &(SPI1->DR);
  // stream0: ch3, single burst, prio high, byte to byte, minc, p2m
  DMA2_Stream[0].CR = ((3 << DMA_SxCR_CHSEL_shift)
		       | (DMA_SxCR_BURST_SINGLE << DMA_SxCR_MBURST_shift)
		       | (DMA_SxCR_BURST_SINGLE << DMA_SxCR_PBURST_shift)
		       | (DMA_SxCR_PL_HIGH << DMA_SxCR_PL_shift)
		       | (DMA_SxCR_SIZE_BYTE << DMA_SxCR_MSIZE_shift)
		       | (DMA_SxCR_SIZE_BYTE << DMA_SxCR_PSIZE_shift)
		       | DMA_SxCR_MINC
		       | (DMA_SxCR_DIR_P2M << DMA_SxCR_DIR_shift)
		       | (DMA_SxCR_TCIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE));
  DMA2_Stream[3].PAR = (uint32_t) (void *) &(SPI1->DR);
  // stream3: ch3, single burst, prio high, byte to byte, minc, m2p
  DMA2_Stream[3].CR = ((3 << DMA_SxCR_CHSEL_shift)
		       | (DMA_SxCR_BURST_SINGLE << DMA_SxCR_MBURST_shift)
		       | (DMA_SxCR_BURST_SINGLE << DMA_SxCR_PBURST_shift)
		       | (DMA_SxCR_PL_HIGH << DMA_SxCR_PL_shift)
		       | (DMA_SxCR_SIZE_BYTE << DMA_SxCR_MSIZE_shift)
		       | (DMA_SxCR_SIZE_BYTE << DMA_SxCR_PSIZE_shift)
		       | DMA_SxCR_MINC
		       | (DMA_SxCR_DIR_M2P << DMA_SxCR_DIR_shift)
		       | (DMA_SxCR_TCIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE));

  // Enable DMA1
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
  RCC->AHB1RSTR |= RCC_AHB1RSTR_DMA1RST;
  RCC->AHB1RSTR &=~ RCC_AHB1RSTR_DMA1RST;

  // Enable SPI2 clock.
  RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
  RCC->APB1RSTR = RCC_APB1RSTR_SPI2RST;
  RCC->APB1RSTR = 0;

  // BR[1:0]: 3 (pclk/16), Main
  SPI2->CR1 = (3 << CR1_BR_SHIFT)| CR1_MSTR;
  // 8-bit, NSS  asserted when SPI is enabled
  SPI2->CR2 = CR2_FRXTH | (7 << CR2_DS_SHIFT) | CR2_SSOE;

  // SPI2_RX: DMA1 stream 3 channel 0
  // SPI2_TX: DMA1 stream 4 channel 0
  // Stop stream first
  DMA1_Stream[3].CR &= ~DMA_SxCR_EN;
  while (DMA1_Stream[3].CR & DMA_SxCR_EN)
    ;
  DMA1_Stream[4].CR &= ~DMA_SxCR_EN;
  while (DMA1_Stream[4].CR & DMA_SxCR_EN)
    ;
  DMA1->LIFCR = (DMA1->LISR
		 & (DMA_TCIF3|DMA_HTIF3|DMA_TEIF3|DMA_DMEIF3|DMA_FEIF3));
  DMA1_Stream[3].PAR = (uint32_t) (void *) &(SPI2->DR);
  // stream3: ch0, single burst, prio high, byte to byte, minc, p2m
  DMA1_Stream[3].CR = ((0 << DMA_SxCR_CHSEL_shift)
		       | (DMA_SxCR_BURST_SINGLE << DMA_SxCR_MBURST_shift)
		       | (DMA_SxCR_BURST_SINGLE << DMA_SxCR_PBURST_shift)
		       | (DMA_SxCR_PL_HIGH << DMA_SxCR_PL_shift)
		       | (DMA_SxCR_SIZE_BYTE << DMA_SxCR_MSIZE_shift)
		       | (DMA_SxCR_SIZE_BYTE << DMA_SxCR_PSIZE_shift)
		       | DMA_SxCR_MINC
		       | (DMA_SxCR_DIR_P2M << DMA_SxCR_DIR_shift)
		       | (DMA_SxCR_TCIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE));
  DMA1_Stream[4].PAR = (uint32_t) (void *) &(SPI2->DR);
  // stream4: ch0, single burst, prio high, byte to byte, minc, m2p
  DMA1_Stream[4].CR = ((0 << DMA_SxCR_CHSEL_shift)
		       | (DMA_SxCR_BURST_SINGLE << DMA_SxCR_MBURST_shift)
		       | (DMA_SxCR_BURST_SINGLE << DMA_SxCR_PBURST_shift)
		       | (DMA_SxCR_PL_HIGH << DMA_SxCR_PL_shift)
		       | (DMA_SxCR_SIZE_BYTE << DMA_SxCR_MSIZE_shift)
		       | (DMA_SxCR_SIZE_BYTE << DMA_SxCR_PSIZE_shift)
		       | DMA_SxCR_MINC
		       | (DMA_SxCR_DIR_M2P << DMA_SxCR_DIR_shift)
		       | (DMA_SxCR_TCIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE));
}

static volatile uint32_t dummem[1];

bool
spi1_transfer (uint8_t *sbuf, uint8_t *rbuf, size_t n)
{
  DMA2_Stream[0].M0AR = (uint32_t) (void *) rbuf;
  DMA2_Stream[0].NDTR = n;
  DMA2_Stream[3].M0AR = (uint32_t) (void *) sbuf;
  DMA2_Stream[3].NDTR = n;

  SPI1->CR2 |= CR2_RXDMAEN;

  DMA2_Stream[0].CR |= DMA_SxCR_EN;
  DMA2_Stream[3].CR |= DMA_SxCR_EN;

  SPI1->CR2 |= CR2_TXDMAEN;

  // Wait DMA2 Stream0 interrupt
  bitset flags;
  thread::poll_section ();
  flags.clear ();
  id_t irq_id = eventflag::irq_event (INTR_REQ_DMA2_Stream0);
  flags.add (irq_id);

  SPI1->CR1 |= CR1_SPE;
  id_t id = thread::poll (flags);
  if (id != irq_id)
    goto fail;

  // Get status and handle errors
  uint32_t isr;
  isr = DMA2->LISR;
  if (isr & (DMA_TEIF0|DMA_DMEIF0|DMA_FEIF0))
    {
      // Report error and restart dma - NOTYET
      // Clear these flags
      goto fail;
    }
  while ((SPI1->SR >> SR_FTLVL_SHIFT) & 0x3)
    ;
  while (SPI1->SR & SR_BSY)
    ;
  SPI1->CR1 &= ~CR1_SPE;
  while ((SPI1->SR >> SR_FRLVL_SHIFT) & 0x3)
    ;
  DMA2->LIFCR = DMA2->LISR;
  DMA2->HIFCR = DMA2->HISR;
  DMA2_Stream[0].CR &= ~DMA_SxCR_EN;
  DMA2_Stream[3].CR &= ~DMA_SxCR_EN;
  SPI1->CR2 &= ~(CR2_RXDMAEN | CR2_TXDMAEN);
  return true;

 fail:
  while ((SPI1->SR >> SR_FTLVL_SHIFT) & 0x3)
    ;
  while (SPI1->SR & SR_BSY)
    ;
  SPI1->CR1 &= ~CR1_SPE;
  uint8_t dummy __attribute__((unused));
  while ((SPI1->SR >> SR_FRLVL_SHIFT) & 0x3)
    dummy = SPI1->DR;
  DMA2->LIFCR = DMA2->LISR;
  DMA2->HIFCR = DMA2->HISR;
  DMA2_Stream[0].CR &= ~DMA_SxCR_EN;
  DMA2_Stream[3].CR &= ~DMA_SxCR_EN;
  SPI1->CR2 &= ~(CR2_RXDMAEN | CR2_TXDMAEN);
  return false;
}

bool
spi2_transfer (uint8_t *sbuf, uint8_t *rbuf, size_t n)
{
  DMA1_Stream[3].M0AR = (uint32_t) (void *) rbuf;
  DMA1_Stream[3].NDTR = n;
  DMA1_Stream[4].M0AR = (uint32_t) (void *) sbuf;
  DMA1_Stream[4].NDTR = n;

  SPI2->CR2 |= CR2_RXDMAEN;

  DMA1_Stream[3].CR |= DMA_SxCR_EN;
  DMA1_Stream[4].CR |= DMA_SxCR_EN;

  SPI2->CR2 |= CR2_TXDMAEN;

  // Wait DMA1 Stream3 interrupt
  bitset flags;
  thread::poll_section ();
  flags.clear ();
  id_t irq_id = eventflag::irq_event (INTR_REQ_DMA1_Stream3);
  flags.add (irq_id);

  SPI2->CR1 |= CR1_SPE;
  id_t id = thread::poll (flags);
  if (id != irq_id)
    goto fail;

  // Get status and handle errors
  uint32_t isr;
  isr = DMA1->LISR;
  if (isr & (DMA_TEIF3|DMA_DMEIF3|DMA_FEIF3))
    {
      // Report error and restart dma - NOTYET
      // Clear these flags
      goto fail;
    }

  while ((SPI2->SR >> SR_FTLVL_SHIFT) & 0x3)
    ;
  while (SPI2->SR & SR_BSY)
    ;
  SPI2->CR1 &= ~CR1_SPE;
  while ((SPI2->SR >> SR_FRLVL_SHIFT) & 0x3)
    ;
  DMA1->LIFCR = DMA1->LISR;
  DMA1->HIFCR = DMA1->HISR;
  DMA1_Stream[3].CR &= ~DMA_SxCR_EN;
  DMA1_Stream[4].CR &= ~DMA_SxCR_EN;
  SPI2->CR2 &= ~(CR2_RXDMAEN | CR2_TXDMAEN);
  return true;

 fail:
  while ((SPI2->SR >> SR_FTLVL_SHIFT) & 0x3)
    ;
  while (SPI2->SR & SR_BSY)
    ;
  SPI2->CR1 &= ~CR1_SPE;
  uint8_t dummy __attribute__((unused));
  while ((SPI2->SR >> SR_FRLVL_SHIFT) & 0x3)
    dummy = SPI2->DR;
  DMA1->LIFCR = DMA1->LISR;
  DMA1->HIFCR = DMA1->HISR;
  DMA1_Stream[3].CR &= ~DMA_SxCR_EN;
  DMA1_Stream[4].CR &= ~DMA_SxCR_EN;
  SPI2->CR2 &= ~(CR2_RXDMAEN | CR2_TXDMAEN);
  return false;
}

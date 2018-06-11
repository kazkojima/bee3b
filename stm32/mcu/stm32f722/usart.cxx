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

// USART
// USART2 at 38400 baud rate for system use

#define INTR_REQ_USART2		38
// USART2_RX DMA1 CH4 Stream5
#define INTR_REQ_DMA1_Stream5	16
// USART2_TX DMA1 CH4 Stream6
#define INTR_REQ_DMA1_Stream6	17

#define ISR_TXE		(1 << 7)
#define ISR_TC		(1 << 6)
#define ISR_RXNE	(1 << 5)
#define ISR_IDLE	(1 << 4)
#define ISR_ORE		(1 << 3)
#define ISR_NF		(1 << 2)
#define ISR_FE		(1 << 1)
#define ISR_PE		(1 << 0)

#define CR1_UE		(1 << 0)
#define CR1_PRIE	(1 << 8)
#define CR1_TXEIE	(1 << 7)
#define CR1_TCIE	(1 << 6)
#define CR1_RXNEIE	(1 << 5)
#define CR1_IDLEIE	(1 << 4)
#define CR1_TE		(1 << 3)
#define CR1_RE		(1 << 2)

#define CR2_STOP2	(2 << 12)

#define CR3_DMAT	(1 << 7)
#define CR3_DMAR	(1 << 6)
#define CR3_EIE		(1 << 0)

void
ef::board::usart_init (void)
{
  // Enable USART2 clock.
  RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
  RCC->APB1RSTR = RCC_APB1RSTR_USART2RST;
  RCC->APB1RSTR = 0;

  USART2->CR1 = 0;
  USART2->CR2 = CR2_STOP2;
  USART2->CR3 = 0;
  
  // PCLK=54MHz Baud=38.4kHz div=1406
  USART2->BRR = 1406;
  USART2->CR1 = CR1_RXNEIE | CR1_TE | CR1_RE | CR1_UE;
}

void
ef::board::usart_send (const uint8_t ch)
{
  while (!(USART2->ISR & ISR_TXE))
    ;
  USART2->TDR = ch;
  while (!(USART2->ISR & ISR_TC))
    ;
}

// Copyright (C) 2016 kaz Kojima
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

// STM32 SYST

struct SYST
{
  volatile uint32_t CSR;
  volatile uint32_t RVR;
  volatile uint32_t CVR;
  volatile uint32_t CR;
};
  
static struct SYST *const SYST = (struct SYST *const) 0xe000e010;

void
ef::systimer::init (void)
{
  SYST->RVR = 0;
  SYST->CVR = 0;
  SYST->CSR = 7;
}

void
ef::systimer::reload (uint32_t ticks)
{
  SYST->RVR = ticks;
  SYST->CVR = 0;
  SYST->RVR = 0;
}

uint32_t
ef::systimer::get (void)
{
  return SYST->CVR;
}

#ifndef MHZ
#define MHZ 48
#endif

uint32_t
ef::systimer::usec_to_ticks (uint32_t usec)
{
  return usec * MHZ;
}

uint32_t
ef::systimer::ticks_to_usec (uint32_t usec)
{
  return usec / MHZ;
}

uint32_t
ef::systimer::max_ticks (void)
{
  // stm32 SysTick timer is 24-bit.
  return (0xffffff / MHZ) * MHZ;
}

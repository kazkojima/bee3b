// Copyright (C) 2016, 2018 kaz Kojima
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

#include "irq.h"
#include "ef.h"

#if defined(MCU_STM32F722)
# include "mcu/irq.cxx"
#else
# error "Unsupported MCU"
#endif

uint8_t irq::ids[MAX_IRQn];
uint8_t irq::irqs[MAX_ID];
bitset irq::iall;

irq::irq (irqnum_t n) : irq_num(n)
{
}

void
irq::claim (id_t id)
{
  if (iall.has (id))
    return;

  ids[irq_num] = id;
  irqs[id] = irq_num;
  iall.add (id);
}

id_t
irq::id (void)
{
  return ids[irq_num];
}

irqnum_t
irq::irqn (id_t id)
{
  return irqs[id];
}

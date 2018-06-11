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

#pragma once

#include <cstdint>

namespace ef
{

  // Generate software interrupt for delayed scheduling
  static inline void  __attribute__ ((unused))
    request_preemption (void)
  {
    *(volatile uint32_t *const)0xE000ED04 = (1 << 28);
    asm ("" : : : "memory");
  }

  static inline void  __attribute__ ((unused))
    prio_init (void)
  {
    // AIRCR: PRIGROUP = 5, 2-bit:2-bit
    *(volatile uint32_t *const)0xE000ED0C = 0x05FA0000 | ( 5 << 8);
    // SHPR2:
    *(volatile uint32_t *const)0xE000ED1C = PRIO_SVC << 24;
    // SHPR3:
    *(volatile uint32_t *const)0xE000ED20 = ((PRIO_IRQ << 24)
					     |(PRIO_PENDSV << 16));
  }

  // exception handlers
  class exc
  {
  public:
    static void svc ();
    static void preempt ();
    static void systick ();
    static void irq_handler ();
    // Not real handler
    // Switch thread immediately
    static void sched ()
    {
      asm ("svc	#0" : : : "memory");
    }
  };
}

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
#include <cstring>

namespace ef
{
  struct context_on_stack
  {
    uint32_t r0, r1, r2, r3, r12, lr, pc, xpsr;
#if defined(__ARM_ARCH_7EM__) && defined(__ARM_FP) && (__ARM_FP == 4)
    uint32_t sreg[32];
    uint32_t fpscr;
#endif
  };

  enum cpu_prio
  {
    PRIO_SVC = 0x30,
    PRIO_CRITICAL = 0x40,
    PRIO_IRQ = 0xb0,
    PRIO_PENDSV = 0xc0
  };

  class context
  {
    friend class sys;
    friend class exc;

  private:
    uint32_t r4, r5, r6, r7, r8, r9, r10, r11, sp, xlr, basepri;

    void restore ();

  public:
    void init (uint32_t stack, uint32_t entry, uint32_t arg,
	       uint32_t caller, uint32_t bpri)
      {
	r4 = r5 = r6 = r7 = r8 = r9 = r10 = r11 = 0;
	sp = stack;
	xlr = 0xfffffffd;
	basepri = bpri;

	context_on_stack *pos = reinterpret_cast<context_on_stack *>(stack);
	pos->r0 = arg;
	pos->pc = entry;
	pos->lr = caller;
	pos->xpsr = 0x01000000; // T:1 other:0
      }
  };

  static inline uint32_t __attribute__ ((unused))
    enter_critical_section (void)
  {
    uint32_t opri;
    
    asm ("mrs	%0, BASEPRI" : "=r" (opri) : : "memory");
    if (opri == 0 || opri > PRIO_CRITICAL)
      asm ("msr	BASEPRI, %0" : : "r" (PRIO_CRITICAL) : "memory");
    return opri;
  }

  static inline void __attribute__ ((unused))
    exit_critical_section (uint32_t opri)
  {
    if (opri == 0 || opri > PRIO_CRITICAL)
      asm ("msr	BASEPRI, %0" : : "r" (opri) : "memory");
  }
}

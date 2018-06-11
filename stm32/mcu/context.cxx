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

// STM32 dependant context and support routines

void __attribute__ ((naked))
ef::context::restore (void)
  {
    asm volatile ("ldm	r0!, {r4, r5, r6, r7}\n\t"
		  "ldr	r8, [r0], #4\n\t"
		  "ldr	r9, [r0], #4\n\t"
		  "ldr	r10, [r0], #4\n\t"
		  "ldr	r11, [r0], #4\n\t"
		  "ldr	r2, [r0], #4\n\t"
		  "msr	PSP, r2\n\t"
		  "ldr	lr, [r0], #4\n\t"
		  "ldr	r2, [r0], #4\n\t"
		  "msr	BASEPRI, r2\n\t"
		  "bx	lr"
		  : /* no output */
		  : "l" (this)
		  : "r1", "r2", "r3", "r4", "r5", "r6", "r7", "memory");
  }

// Exception handlers

// SVCall
void __attribute__ ((naked))
ef::exc::svc (void)
{
  asm volatile (// Get ef::thread::running and save context
		"ldr	r1, =_ZN2ef6thread7runningE\n\t"
		"ldr	r0, [r1]\n\t"
		"stm	r0!, {r4, r5, r6, r7}\n\t"
		"mov	r2, r8\n\t"
		"mov	r3, r9\n\t"
		"mov	r4, r10\n\t"
		"mov	r5, r11\n\t"
		"mrs	r6, PSP\n\t"
		"stm	r0!, {r2, r3, r4, r5, r6, lr}"
		: /* no output */
		: /* no input */
		: "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7", "memory");

  bitset r = thread::runnable;
  id_t id = r.pop ();
  thread *np = thread::pvec[id];
  thread::running = np;

  ((context *) np)->restore ();
}

// PendSV
void __attribute__ ((naked))
ef::exc::preempt (void)
{
  register thread *tp asm ("r0");

  asm volatile (// Get ef::thread::running and save context
		"ldr	r1, =_ZN2ef6thread7runningE\n\t"
		"ldr	r0, [r1]\n\t"
		"stm	r0!, {r4, r5, r6, r7}\n\t"
		"mov	r2, r8\n\t"
		"mov	r3, r9\n\t"
		"mov	r4, r10\n\t"
		"mov	r5, r11\n\t"
		"mrs	r6, PSP\n\t"
		"stm	r0!, {r2, r3, r4, r5, r6, lr}"
		: "=r" (tp)
		: /* no input */
		: "r1", "r2", "r3", "r4", "r5", "r6", "r7", "memory");

  bitset r = thread::runnable;
  id_t id = r.pop ();
  thread *np = thread::pvec[id];
  thread::running = np;

  ((context *) np)->restore ();
}

// SysTick
void
ef::exc::systick (void)
{
  timers::update ();
}

// Generic IRQ handler
void
ef::exc::irq_handler (void)
{
  uint32_t n;

  asm ("mrs    %0, IPSR" : "=r" (n) : /* no input */ : "memory");
  // Exception # - 16 = interrupt number
  n -= 16;

  irq ir(n);
  ir.disable ();
  id_t id = ir.id ();
  if (irq::irqn (id) == n)
    eventflag::signal (id);
}

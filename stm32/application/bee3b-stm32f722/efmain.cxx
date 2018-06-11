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

#include "ef.h"
#include "mcu.h"
#include "spi.h"
#include "inven.h"
#include "pwm.h"

using namespace ef;

static void
exti_init (void)
{
  // PA0 PA:0
  SYSCFG->EXTICR[0] = 0;
  // PC6 PC:2
  SYSCFG->EXTICR[1] = 2 << (6 - 4);
  SYSCFG->CMPCR = 1;

  EXTI->IMR = (1 << 0) | (1 << 6);
  EXTI->EMR = (1 << 0) | (1 << 6);
  // Capture rising edge
  EXTI->RTSR = (1 << 0) | (1 << 6);
  EXTI->FTSR = 0;
  // Clear with writing 1s
  EXTI->PR = (1 << 0) | (1 << 6);
}

#define INTR_REQ_EXTI0		6
#define INTR_REQ_EXTI9_5	23

static uint8_t send2[32];
static uint8_t recv2[32];

void
bee3b_test (void *arg __attribute__ ((unused)))
{
  std::memset (recv2, 0, 32);
  std::memset (send2, 0, 32);

  icm20602_init ();

  bitset flags;
  thread::poll_section ();
  flags.clear ();
  flags.add (eventflag::timeout_event (1000));
  thread::poll (flags);

  while (1)
    {
#if 1
      //WAIT INT
      if (!(EXTI->PR & (1 << 0)))
	{
	  thread::poll_section ();
	  flags.clear ();
	  id_t inven_irq_id = eventflag::irq_event (INTR_REQ_EXTI0);
	  flags.add (inven_irq_id);
	  id_t id = thread::poll (flags);
	}
      EXTI->PR = (1 << 0);

      int fifo_count = icm20602_fifo_count ();
      if (fifo_count != 0)
	{
	  int n_sample = fifo_count / ICM20602_SIZE_OF_SAMPLE;
	  while(n_sample--)
	    {
	      float gx, gy, gz, ax, ay, az, t;
	      if (icm20602_get_values (gx, gy, gz, ax, ay, az, t))
		{
		  // Filter

		  // If slow output is updated
		  if (!(EXTI->PR & (1 << 6)))
		    {
		      thread::poll_section ();
		      flags.clear ();
		      id_t hs_irq_id = eventflag::irq_event (INTR_REQ_EXTI9_5);
		      flags.add (hs_irq_id);
		      id_t id = thread::poll (flags);
		    }
		  EXTI->PR = (1 << 6);
		  union { float f; uint8_t bytes[sizeof(float)];} ux, uy, uz;
		  union { float f; uint8_t bytes[sizeof(float)];} ut;
		  ux.f = ax; uy.f = ay; uz.f = az;
		  memcpy (&send2[0], ux.bytes, sizeof(float));
		  memcpy (&send2[4], uy.bytes, sizeof(float));
		  memcpy (&send2[8], uz.bytes, sizeof(float));
		  ux.f = gx; uy.f = gy; uz.f = gz;
		  memcpy (&send2[12], ux.bytes, sizeof(float));
		  memcpy (&send2[16], uy.bytes, sizeof(float));
		  memcpy (&send2[20], uz.bytes, sizeof(float));
		  ut.f = t;
		  memcpy (&send2[24], ut.bytes, sizeof(float));
		  spi2_transfer (send2, recv2, 32);
		  tim4_set_pwm_width ((uint16_t *) recv2);
		}
	    }
	}
#else
      board::set_led (true);
      thread::poll_section ();
      flags.clear ();
      flags.add (eventflag::timeout_event (100*1000));
      thread::poll (flags);
      board::set_led (false);
      thread::poll_section ();
      flags.clear ();
      flags.add (eventflag::timeout_event (100*1000));
      thread::poll (flags);
#endif
    }
}

void
ef::main (void *arg  __attribute__ ((unused)))
{
  spi_init ();
  exti_init ();
  tim4_init ();

  board::set_led (true);

  thread *tp = thread::create (5, bee3b_test, NULL, NULL, 8192, 0);
  tp->run ();

  while(1)
    ;
}



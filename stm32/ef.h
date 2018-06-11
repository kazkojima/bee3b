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
#include "id.h"
#include "bitset.h"
#include "context.h"
#include "systimer.h"
#include "irq.h"

namespace ef
{
#define MAX_THREAD_ID (ef::MAX_ID)
#define MAX_EVENTFLAG_ID (ef::MAX_ID)

  class eventflag
  {
    friend class thread;

  private:
    bitset threads;

    static bitset all;	// Set of all flags
    static eventflag vec[MAX_EVENTFLAG_ID];

    static id_t new_id ();

  public:
    static id_t soft_event ();
    static id_t irq_event (irqnum_t irqnum);
    static id_t timeout_event (uint32_t usec);
    static void signal (id_t id);
    static void release_id (id_t id);
  };

  class mutex
  {
  private:
    id_t owner;
    id_t event;
  public:
    mutex ();
    virtual ~mutex ();
    void lock ();
    void unlock ();
  };

  class thread
    : public context
  {
    friend class eventflag;
    friend class mutex;
    friend class exc;
    friend class sys;

  private:
    id_t id;
    id_t cause;
    bitset eventflags;

    static bitset all;		// Set of all threads
    static bitset runnable;	// Runnable threads
    static bitset waiting;	// Threads waiting some flags
    static thread *pvec[MAX_THREAD_ID];
    static thread *running;

  public:
    static thread* create (id_t id, void (*fp) (void *), void *arg,
			   void (*return_fp) (void *), 
			   uint32_t stack_size, uint32_t pri);
    void run ();
    static void poll_section ();
    static id_t poll (bitset flags);
  };

  void main (void *arg);

  // misc
  void fatal (const char *msg);
}

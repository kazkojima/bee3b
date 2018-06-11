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

#include "ef.h"
#include <cstdlib>

using namespace ef;

void *__dso_handle;

bitset eventflag::all;	// Set of all flags
eventflag eventflag::vec[MAX_EVENTFLAG_ID];
bitset thread::all;		// Set of all threads
bitset thread::runnable;	// Runnable threads
bitset thread::waiting;	// Threads waiting some flags
thread *thread::pvec[MAX_THREAD_ID];
thread *thread::running;

void
eventflag::signal (id_t sig)
{
  bitset thds = eventflag::vec[sig].threads;
  // return immediately if no waiters
  if (thds.empty ())
    return;

  while (!thds.empty ())
    {
      id_t th = thds.pop ();
      bitset efs = thread::pvec[th]->eventflags;
      while (!efs.empty ())
	{
	  id_t ef = efs.pop ();
	  eventflag::vec[ef].threads.remove (th);
	}
      thread::pvec[th]->eventflags.clear ();
      thread::pvec[th]->cause = sig;
      // make thread::pvec[th] runnable
      thread::waiting.remove (th);
      thread::runnable.add (th);
    }
  
  request_preemption ();
}

id_t
eventflag::new_id (void)
{
  bitset unused = ~eventflag::all;
  if (unused.empty ())
    fatal ("eventflag id exhausted");

  id_t id = unused.pop ();
  eventflag::all.add (id);
  eventflag::vec[id].threads.clear ();
  return id;
}

void
eventflag::release_id (id_t id)
{
  eventflag::all.remove (id);
}

id_t
eventflag::soft_event (void)
{
  id_t id = eventflag::new_id ();
  return id;
}

id_t
eventflag::irq_event (irqnum_t irqnum)
{
  irq ir = irq (irqnum);
  id_t id = ir.id ();
  if (irqnum != irq::irqn (id))
    {
      id = eventflag::new_id ();
      ir.claim (id);
    }
  ir.disable ();
  ir.set_prio ();
  return id;
}
 
id_t
eventflag::timeout_event (uint32_t usec)
{
  id_t id = eventflag::new_id ();
  timers::add (usec, id);
  return id;
}

mutex::mutex (void)
  : owner (MAX_THREAD_ID)
{
  event = eventflag::soft_event ();
}

mutex::~mutex (void)
{
  eventflag::release_id (event);
}

void
mutex::lock (void)
{
  uint32_t opri = ef::enter_critical_section ();
  while (1)
    {
      if (owner == MAX_THREAD_ID)
	{
	  owner = ef::thread::running->id;
	  break;
	}
      else
	{
	  bitset flags;
	  flags.clear ();
	  flags.add (event);
	  thread::poll (flags);
	}
    }
  ef::exit_critical_section (opri);
}

void
mutex::unlock (void)
{
  uint32_t opri = ef::enter_critical_section ();
  if (owner == thread::running->id)
    {
      owner = MAX_THREAD_ID;
      eventflag::signal (event);
    }
  ef::exit_critical_section (opri);
}

thread *
thread::create (id_t id, void (*fp) (void *), void *arg,
		void (*return_fp) (void *),
		uint32_t stack_size, uint32_t pri)
{
  void *mp = std::malloc (stack_size);
  uint32_t stack = (reinterpret_cast<uint32_t> (mp) + stack_size
		    - sizeof (context_on_stack));
  thread *tp = static_cast<thread *> (std::malloc (sizeof (thread)));
  ((context *) tp)->init (stack,
			  reinterpret_cast<uint32_t> (fp),
			  reinterpret_cast<uint32_t> (arg),
			  reinterpret_cast<uint32_t> (return_fp),
			  pri);
  tp->id = id;
  tp->cause = MAX_ID;
  tp->eventflags.clear ();
  thread::pvec[id] = tp;
  thread::all.add (id);
  return tp;
}

void
thread::run (void)
{
  thread::runnable.add (id);
  exc::sched ();
}

void
thread::poll_section (void)
{
  // Disable irq/systick
  ef::enter_critical_section ();
}

id_t
thread::poll (bitset flags)
{
  // check flags???

  thread *tp = thread::running;
  tp->eventflags = flags;
  if (flags.empty ())
    return MAX_ID;

  id_t tid = tp->id;
  bitset iflags = flags & irq::iall;
  while (!flags.empty ())
    {
      id_t eid = flags.pop ();
      eventflag::vec[eid].threads.add (tid);
      if (iflags.has (eid))
	{
	  irq ir = irq (irq::irqn (eid));
	  ir.enable ();
	}
    }

  thread::runnable.remove (tid);
  thread::waiting.add (tid);

  exc::sched ();

  return tp->cause;
}

void
irq::handle (void)
{
  disable ();
  eventflag::signal (irq::ids[irq_num]);
}

void
ef::fatal (const char *msg)
{
  (void) msg;
  for (;;);
}

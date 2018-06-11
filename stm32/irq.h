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

namespace ef
{
  // These are target dependant, but perhaps OK for almost targets ATM.
#define MAX_IRQn 256
  typedef uint8_t irqnum_t;

  class irq
  {
    friend class thread;
    friend class exc;

  private:
    irqnum_t irq_num;

    static uint8_t ids[MAX_IRQn];
    static uint8_t irqs[MAX_ID];
    static bitset iall;

  public:
    irq (irqnum_t n);

    void claim (id_t id);
    id_t id ();
    void enable ();
    void clear ();
    void disable ();
    void set_prio ();
    void handle ();

    static irqnum_t irqn (id_t id);
  };
}

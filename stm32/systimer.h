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
#include <list>

namespace ef
{
  class systimer
  {
  public:
    static void init ();
    static void reload (uint32_t ticks);
    static uint32_t get ();
    static uint32_t usec_to_ticks (uint32_t usec);
    static uint32_t ticks_to_usec (uint32_t usec);
    static uint32_t max_ticks ();
  };

  class timers
  {
  public:
    static void add (uint32_t usec, uint32_t id);
    static void update ();
  };
}

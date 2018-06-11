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

// small bit set on long long
#pragma once

#include <cstdint>
#include "id.h"

namespace ef
{
  typedef uint64_t bitset_t;

  class bitset
  {
  private:
    bitset_t m_set;
    static const bitset_t msb = ((bitset_t) 1) << (MAX_ID - 1);

  public:
    void clear () { m_set = (bitset_t) 0; }
    void add (id_t e) { m_set |= msb >> e; }
    void remove (id_t e) { m_set &= ~(msb >> e); }
    bool empty () { return (m_set == (bitset_t) 0); }
    id_t pop ()
    {
      id_t e = (id_t) __builtin_clzll (m_set);
      remove (e);
      return e;
    }
    bitset operator~ () { bitset b; b.m_set = ~m_set; return b; }
    bitset operator& (bitset other)
    {
      bitset b; b.m_set = m_set & other.m_set; return b;
    }
    bool has (id_t e) { return (m_set & (msb >> e)) != (bitset_t) 0; }
    // void show ();
  };
}

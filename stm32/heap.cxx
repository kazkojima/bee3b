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

/* We overload newlib implementation of_sbrk so to avoid checking
   if heap pointer is less than stack pointer.  */

#include "context.h"

extern "C"
{
#include <errno.h>

  // Heap symbols defined with linker script.
#include "ldsym.h"


  // Modified version of newlib implementation of _sbrk.
  void *
  _sbrk (int incr)
  {
    static char* heap_end;
    char* prev_heap_end;

    if (heap_end == NULL)
      heap_end = &__heap_base__;

    prev_heap_end = heap_end;

    if (heap_end + incr > &__heap_end__)
      {
        /* Some of the libstdc++-v3 tests rely upon detecting
           out of memory errors, so do not abort here.  */
        errno = ENOMEM;
        return (void *) -1;
      }
    heap_end += incr;

    return (void *) prev_heap_end;
  }

  static uint32_t __malloc_lock_flags;

  // Override malloc_lock/unlock so as to make malloc thread-safe.
  void __malloc_lock ()
  {
    __malloc_lock_flags = ef::enter_critical_section ();
  }

  void __malloc_unlock ()
  {
    ef::exit_critical_section (__malloc_lock_flags);
  }
} // extern "C"

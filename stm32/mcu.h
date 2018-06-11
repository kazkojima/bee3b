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

#pragma once

extern "C" {
#if defined(MCU_STM32F722)
# include "mcu/stm32f722/mcu.h"
#else
# error "Unsupported MCU"
#endif
}

#include "context.h"

namespace ef
{
  class mcu
  {
    friend class sys;

    static void rcc_init ();
    static void gpio_init ();
  };

  class sys
  {
    typedef void (*handler)(void);
    static void entry ();
    static handler _vector[];
    static handler vector[];
  };

  class board
  {
  public:
    static void set_led (bool);
    static void usart_init ();
    static void usart_send (const uint8_t ch);
  };
}

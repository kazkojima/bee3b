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

#if defined(__ARM_ARCH_7M__) || defined (__ARM_ARCH_7EM__)
# include "core/context-7mx.h"
#elif defined(__ARM_ARCH_6M__)
# include "core/context-6m.h"
#else
# error "Unsupported architecture"
#endif

#if defined(MCU_STM32F722)
# include "mcu/context.cxx"
#else
# error "Unsupported MCU"
#endif

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

// Dummy syscalls

#pragma GCC diagnostic ignored "-Wunused-parameter"

int
_close(int fd)
{
  return -1;
}

int
_fstat(int fd, void *s)
{
  return -1;
}


void
_exit(int s)
{
  while (1)
    asm ("swi %a0" :: "i" (0x11));
}

int
_kill (int pid, int sig)
{
  return -1;
}

int
_write(int fd, void *b, int nb)
{
  return -1;
}

int
_isatty (int c)
{
    return -1;
}

int
_lseek (int fd, int off, int w)
{
  return -1;
}

int
_read (int fd, void *b, int nb)
{
  return -1;
}

int
_getpid (void)
{
  return -1;
}

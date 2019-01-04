// Copyright (C) 2018 kaz Kojima
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

template <typename T> T
sat (T x, T lo, T hi)
{
  if (x < lo)
    return lo;
  else if (x > hi)
    return hi;
  return x;
}

inline static uint16_t
chmix (int chval, int adjval, int lim)
{
  if (chval == 0)
    return chval;
  adjval = sat<int> (adjval, -lim, lim);
  return sat<int> (chval + adjval, 0, 4096);
}

// Channel matrix
inline static void
chmat (float pitch, float roll, float yaw, float gain,
       float& ch1, float& ch2, float& ch3, float& ch4)
{
  static const float m[4][3] =
    {
#define CH1GAIN 1.0
#define CH2GAIN 1.0
#define CH3GAIN 1.0
#define CH4GAIN 1.0
#define DRGAIN 1.0
#define DPGAIN 1.2
#define DYGAIN 1.0
     {  CH1GAIN*DRGAIN, -CH1GAIN*DPGAIN,  CH1GAIN*DYGAIN, },
     { -CH2GAIN*DRGAIN,  CH2GAIN*DPGAIN,  CH2GAIN*DYGAIN, },
     { -CH3GAIN*DRGAIN, -CH3GAIN*DPGAIN, -CH3GAIN*DYGAIN, },
     {  CH4GAIN*DRGAIN,  CH4GAIN*DPGAIN, -CH4GAIN*DYGAIN, },
    };
  ch1 = gain*(m[0][0]*roll + m[0][1]*pitch + m[0][2]*yaw);
  ch2 = gain*(m[1][0]*roll + m[1][1]*pitch + m[1][2]*yaw);
  ch3 = gain*(m[2][0]*roll + m[2][1]*pitch + m[2][2]*yaw);
  ch4 = gain*(m[3][0]*roll + m[3][1]*pitch + m[3][2]*yaw);
}

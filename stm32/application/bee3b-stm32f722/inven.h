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

#define ICM20602_SIZE_OF_SAMPLE 14
#define ICM20602_MAX_SAMPLE 48
struct icm20602_sample {
  uint8_t reg;
  uint8_t d[ICM20602_SIZE_OF_SAMPLE*ICM20602_MAX_SAMPLE];
};

bool icm20602_init (void);
int icm20602_fifo_count (void);
bool icm20602_get_sample (uint8_t *p, float& gx, float& gy, float& gz,
			  float& ax, float& ay, float& az, float &t);
bool icm20602_read_fifo (int n, struct icm20602_sample *rxp);

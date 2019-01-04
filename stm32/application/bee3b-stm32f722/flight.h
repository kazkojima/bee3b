// Copyright (C) 2018, 2019 kaz Kojima
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

// Very simple landing controller

#include <cstdint>

// controller state
enum flightState { ST_START, ST_SAS, ST_HOLD, ST_LANDING, ST_CRASH, ST_HALT };

class flightController
{
public:
  flightController ();
  void reset (void);
  void hedge (float c, float si, float sj, float sk,
	      float ax, float ay, float az,
	      float posz, float ofx, float ofy,
	      uint16_t ch1, uint16_t ch2, uint16_t ch3, uint16_t ch4);
  void update (float c, float si, float sj, float sk,
	       float oi, float oj, float ok,
	       float ax, float ay, float az,
	       float posz, float ofx, float ofy,
	       float kp, float kd, float kf, float kv,
	       uint16_t& mo1, uint16_t& mo2, uint16_t& mo3, uint16_t& mo4);
  void dump (float& broll, float& bpitch, float& byaw,
	     float& posx, float& posy, float& posz,
	     float& velx, float& vely, float& velz);
private:
  // altitude bias
  float m_broll, m_bpitch, m_byaw;
  // estimated horizontal position&speed
  float m_posx, m_posy;
  float m_velx, m_vely;
  // estimated vertical position, speed, acc offset and filter constants
  float m_posz, m_velz, m_ialt;
  const float m_igain = 1.0f/4096.0f;
  const float m_max_ialt = 10000.0f;
  // target horizontal position
  float m_tposx, m_tposy;
  // target vertical position
  float m_tposz;
  // gains
  const float m_kp = 1.414213f*10.0f;
  const float m_kv = 10.0f*10.0f;
  // pwm mean values
  float m_ch1, m_ch2, m_ch3, m_ch4;
  // failsafe flight mode
  enum flightState m_state;
};

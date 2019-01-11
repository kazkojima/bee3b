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

// Very simple flight controller

#include <cstdint>

#include "flight.h"
#include "motmat.h"

inline static float Absf(float x) { return (x < 0.0f)? -x : x; }

#define EPSILON 0.001f
#define GRAVITY_MSS 9.80665f

flightController::flightController (void)
{
  m_broll = m_bpitch = m_byaw = 0.0f;
  m_posz = m_posx = m_posy = 0.0f;
  m_velz = m_velx = m_vely = 0.0f;
  m_tposx = m_tposy = m_tposz = 0.0f;
  m_ch1 = m_ch2 = m_ch3 = m_ch4 = 0.0f;
  m_ialt = 0.0f;
  m_state = ST_LANDING;
}

void
flightController::reset (void)
{
  m_posz = m_posx = m_posy = 0.0f;
  m_velz = m_velx = m_vely = 0.0f;
  m_tposx = m_tposy = m_tposz = 0.0f;
  m_ch1 = m_ch2 = m_ch3 = m_ch4 = 0.0f;
}

void
flightController::hedge (float c, float si, float sj, float sk,
			  float ax, float ay, float az,
			  float posz, float flowx, float flowy,
			  uint16_t ch1, uint16_t ch2, uint16_t ch3,
			  uint16_t ch4)
{
#if 1
  //m_bpitch = (1.0f - EPSILON)*m_bpitch + EPSILON*(-(c*si - sk*sj));
  //m_broll = (1.0f - EPSILON)*m_broll + EPSILON*(-(c*sj + sk*si));
  m_bpitch = 0;
  m_broll = 0;
  m_byaw = (1.0f - EPSILON)*m_byaw + EPSILON*(-(c*sk + si*sj));
#endif
  
  m_ch1 = (1.0f - EPSILON)*m_ch1 + EPSILON*ch1;
  m_ch2 = (1.0f - EPSILON)*m_ch2 + EPSILON*ch2;
  m_ch3 = (1.0f - EPSILON)*m_ch3 + EPSILON*ch3;
  m_ch4 = (1.0f - EPSILON)*m_ch4 + EPSILON*ch4;

  // inertial frame vertical acc
  float accz = (c*(az*c - ay*si + ax*sj) - si*(ay*c + az*si - ax*sk)
		+ sj*(ax*c - az*sj + ay*sk) + sk*(ax*si + ay*sj + az*sk));
  accz += GRAVITY_MSS;
  // update altitude filter
  float xp = m_posz;
  float xv = m_velz;
  float xdelta = posz - xp;
  const float dt = 1.0f/4096;
  m_ialt -= xdelta;
  m_ialt = sat<float> (m_ialt, -m_max_ialt, m_max_ialt);
  accz = accz - m_ialt*m_igain;
  float vdelta = accz*dt;
  m_posz = xp + xv*dt + (m_kp+0.5f*m_kv*dt)*dt*xdelta + 0.5f*dt*vdelta;
  m_velz = xv + m_kv*dt*xdelta + vdelta;

  // estimate horizontal position and speed
  float ovx = m_velx, ovy = m_vely;
  m_velx = flowx*posz;
  m_vely = flowy*posz;
  m_posx += 0.5f*(ovx + m_velx)*dt;
  m_posy += 0.5f*(ovy + m_vely)*dt;
  m_tposx = m_posx;
  m_tposy = m_posy;
}

inline static
float lean (float p, float v, float kp, float kv)
{
  return sat<float> (kp*p + kv*v, -0.5f, 0.5f);
}

inline static float
accel (float tpos, float pos, float v, float ke, float kv)
{
  float perr = tpos - pos;
  float adj = 0.0f;
  if (perr > 0.1f)
    adj = sat<float> (0.3f - v, 0.0f, 0.5f);
  else if (perr < -0.1f)
    adj = -sat<float> (0.3f + v, 0.0f, 0.5f);
  else
    adj = ke*perr + kv*v;
  return adj;
}

void
flightController::update (float c, float si, float sj, float sk,
			  float oi, float oj, float ok,
			  float ax, float ay, float az,
			  float posz, float flowx, float flowy,
			  float Kp, float Kd, float Kf, float kv,
			  uint16_t& ch1, uint16_t& ch2,
			  uint16_t& ch3, uint16_t& ch4)
{
  // These are 1/2 of linear estimations.
  float pitch = (-(c*si - sk*sj)) - m_bpitch;
  float roll = (-(c*sj + sk*si)) - m_broll;
  float yaw = (-(c*sk + si*sj)) - m_byaw;

  // Flight log says that ~4% pwm change made ~3deg attitude change
  // in 0.2s for each axis when hovering.
  // Kp = ~1250 (2*4096*0.04/(15/57.29)) looks ok for the initial gain.

  // -Kp*P
  float m1, m2, m3, m4;
  chmat (pitch, roll, yaw, Kp*kv, m1, m2, m3, m4);
  // -Kd*D
  float a1, a2, a3, a4;
  chmat (oi, oj, ok, Kd*kv, a1, a2, a3, a4);

  // inertial frame vertical acc
  float accz = (c*(az*c - ay*si + ax*sj) - si*(ay*c + az*si - ax*sk)
		+ sj*(ax*c - az*sj + ay*sk) + sk*(ax*si + ay*sj + az*sk));
  accz += GRAVITY_MSS;
  // update altitude filter
  float xp = m_posz;
  float xv = m_velz;
  float xdelta = posz - xp;
  const float dt = 1.0f/4096;
  m_ialt -= xdelta;
  m_ialt = sat<float> (m_ialt, -m_max_ialt, m_max_ialt);
  accz = accz - m_ialt*m_igain;
  float vdelta = accz*dt;
  m_posz = xp + xv*dt + (m_kp+0.5f*m_kv*dt)*dt*xdelta + 0.5f*dt*vdelta;
  m_velz = xv + m_kv*dt*xdelta + vdelta;

  // estimate horizontal position and speed
  float ovx = m_velx, ovy = m_vely;
  m_velx = flowx*posz;
  m_vely = flowy*posz;
  m_posx += 0.5f*(ovx + m_velx)*dt;
  m_posy += 0.5f*(ovy + m_vely)*dt;
  m_tposx = (1-EPSILON)*m_tposx + EPSILON*m_posx;
  m_tposy = (1-EPSILON)*m_tposy + EPSILON*m_posy;

  float leani, leanj, l1, l2, l3, l4;
  leani = -lean ((m_posy - m_tposy), m_vely, 0.2f, 2.0f);
  leanj = lean ((m_posx - m_tposx), m_velx, 0.2f, 2.0f);
  chmat (leani, leanj, 0, Kf, l1, l2, l3, l4);
  
  if (m_state == ST_LANDING)
    {
      // brake
      float brake = -0.25f*sat<float> (m_velz + 0.2f, -0.05f, 0.25f);
      if (m_posz > 0.15f)
	brake += 0.15f*accel (0.1f, m_posz, m_velz, 1.0f, -0.02f);
      else if (m_posz > 0.05f)
	brake -= 0.2f;
      else
	brake -= 0.5f;
      brake *= kv;
      // limit at 70%
      m_ch1 = sat<float> (m_ch1 + brake, 0, 2800);
      m_ch2 = sat<float> (m_ch2 + brake, 0, 2800);
      m_ch3 = sat<float> (m_ch3 + brake, 0, 2800);
      m_ch4 = sat<float> (m_ch4 + brake, 0, 2800);
    }
  else if (m_state == ST_HOLD)
    {
      float adj = kv*50.0f*accel (m_tposz, m_posz, m_velz, 1.0f, -0.02f);
      // limit at 70%
      m_ch1 = sat<float> (m_ch1 + adj, 0, 2800);
      m_ch2 = sat<float> (m_ch2 + adj, 0, 2800);
      m_ch3 = sat<float> (m_ch3 + adj, 0, 2800);
      m_ch4 = sat<float> (m_ch4 + adj, 0, 2800);
    }

  // feedback
  ch1 = chmix (m_ch1, m1 + a1 + l1, 1200);
  ch2 = chmix (m_ch2, m2 + a2 + l2, 1200);
  ch3 = chmix (m_ch3, m3 + a3 + l3, 1200);
  ch4 = chmix (m_ch4, m4 + a4 + l4, 1200);
}

void
flightController::dump (float& broll, float& bpitch, float& byaw,
			float& posx, float& posy, float& posz,
			float& velx, float& vely, float& velz)
{
  broll = m_broll; bpitch = m_bpitch; byaw = m_byaw;
  posx = m_posx; posy = m_posy; posz = m_posz;
  velx = m_velx; vely = m_vely; velz = m_velz;
}

// Copyright (C) 2016, 2018, 2019 kaz Kojima
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
#include "mcu.h"
#include "spi.h"
#include "inven.h"
#include "pwm.h"
#include "rotor.h"
#include "biquadFilter.h"
#include "motmat.h"
#include "flight.h"

using namespace ef;

static void
exti_init (void)
{
  // PA0 PA:0
  SYSCFG->EXTICR[0] = 0;
  // PC6 PC:2
  SYSCFG->EXTICR[1] = 2 << (6 - 4);
  SYSCFG->CMPCR = 1;

  EXTI->IMR = (1 << 0) | (1 << 6);
  EXTI->EMR = (1 << 0) | (1 << 6);
  // Capture rising edge
  EXTI->RTSR = (1 << 0) | (1 << 6);
  EXTI->FTSR = 0;
  // Clear with writing 1s
  EXTI->PR = (1 << 0) | (1 << 6);
}

#define INTR_REQ_EXTI0		6
#define INTR_REQ_EXTI9_5	23

// On DCTM
static uint8_t send2[32];
static uint8_t recv2[32];
static struct icm20602_sample rx;

// 5 second
#define IN_CALIB (4*5000)
#define CALIB_ALPHA 0.001f

// biquad Filter q
#define BUTTERWORTH_Q (1.0f/1.414213562f)

// Moving avarage
template <int N>
class sma
{
public:
  sma (void);
  float Update(float input)
  {
    avr += (input - buf[cur % N])/N;
    buf[cur] = input;
    cur = (cur + 1) % N;
    return avr;
  }
private:
  float buf[N];
  float avr;
  int cur;
};

template <int N> sma<N>::sma(void) :avr(0), cur(0.0)
{
  for (int i=0; i < N; i++)
    buf[i] = 0;
}

// Simple lag LPF 188Hz + avarage of 8 values
// 188*2\pi = 1181.24, dotx = (u - x)*(1181.24/(1181.24 + 1/dt))
class plpf
{
public:
  void Reset (float value)
  {
    x = y = z = value;
  }
  void Update (float& u, float& v, float& w, int count)
  {
    static const float alpha = 1181.24f/(1181.24f + 32*1024);
    x += alpha*(u - x);
    y += alpha*(v - y);
    z += alpha*(w - z);
    if (count == 0)
      sx = sy = sz = 0.0f;
    sx += x;
    sy += y;
    sz += z;
    if (count == (8-1))
      {
	u = sx/8;
	v = sy/8;
	w = sz/8;
      }
  }
private:
  float x = 0.0f, y = 0.0f, z = 0.0f;
  float sx = 0.0f, sy = 0.0f, sz = 0.0f;
};

// Simple crash checker
class crashChecker
{
public:
  flightState update (flightState status, float c, float si, float sj, float sk);
  float show ();
private:
  flightState m_state = ST_START;
  float m_I = 1.0f;
};

inline static float Absf(float x) { return (x < 0.0f)? -x : x; }

flightState
crashChecker::update (flightState state, float c, float si, float sj, float sk)
{
  // z-component of S e3 /S
  float p = c*c - si*si - sj*sj + sk*sk;
  m_I = (1-0.01f)*m_I + 0.01f*p;
  if (m_state != ST_CRASH && m_I < 0.7f)
    m_state = ST_CRASH;
  else if (m_state == ST_CRASH && m_I > 0.96f)
    m_state = ST_HALT;
  else
    m_state = state;
  return m_state;
}

float crashChecker::show () { return m_I; }

// Filters and parameters
static sma<32> smat;
static sma<4> smagx, smagy, smagz, smaax, smaay, smaaz;
static plpf pfg, pfa;
static RotorIyVS RI (0.01, 0.25e-3, 1.0e-6);
static biquadLPF DLPFi (40.0, BUTTERWORTH_Q, 4096.0);
static biquadLPF DLPFj (40.0, BUTTERWORTH_Q, 4096.0);
static biquadLPF DLPFk (40.0, BUTTERWORTH_Q, 4096.0);
static biquadLPF ALPFx (80.0, BUTTERWORTH_Q, 4096.0);
static biquadLPF ALPFy (80.0, BUTTERWORTH_Q, 4096.0);
static biquadLPF ALPFz (80.0, BUTTERWORTH_Q, 4096.0);
static biquadLPF FLPFx (2.0, BUTTERWORTH_Q, 25.0);
static biquadLPF FLPFy (2.0, BUTTERWORTH_Q, 25.0);
static biquadNOTCHF GNoFx (120.0, 0.167248f, 4096.0);
static biquadNOTCHF GNoFy (120.0, 0.167248f, 4096.0);
static biquadNOTCHF GNoFz (120.0, 0.167248f, 4096.0);
static biquadNOTCHF ANoFx (120.0, 0.167248f, 4096.0);
static biquadNOTCHF ANoFy (120.0, 0.167248f, 4096.0);
static biquadNOTCHF ANoFz (120.0, 0.167248f, 4096.0);
static float cgain = 1.0f;
static bool enable_notch_g = true;
static bool enable_notch_a = true;
static float cmd_adj = 0.0f;

// Accelerometer calibration
// It sholud be configurable with some packet, shouldn't be?
static const float Acalv[3] = { 0.562, -0.018, 0.0 };
static const float Acal[3][4] = {
  {  0.9421728,  0.0036101,  0.0234709,  0.2174708 },
  { -0.0042346,  0.9753865,  0.0143261,  0.1286936 },
  {  0.0746991, -0.0156619,  0.9565443, -0.3946332 }
};

inline static void
calibrate_acc (float& ax, float& ay, float& az)
{
  ax += Acalv[0]; ay = Acalv[1]; ay += Acalv[2];
  // Multiply calib. matrix
  float cax, cay, caz;
  cax = Acal[0][0]*ax + Acal[0][1]*ay + Acal[0][2]*az + Acal[0][3];
  cay = Acal[1][0]*ax + Acal[1][1]*ay + Acal[1][2]*az + Acal[1][3];
  caz = Acal[2][0]*ax + Acal[2][1]*ay + Acal[2][2]*az + Acal[2][3];
  ax = cax; ay = cay; az = caz;
}

// Channel definition
// ch1-4: 0-3, ch6(volume): 5, ch8: 7, pwm rate: 11,
// posz(ToF): 11, flowq: 12, flowx: 13, flowy: 14,
// mark: 15
#define CH_ADJ 5
#define CH_ALT 7
#define CH_RATE 8
#define CH_BAT 9
#define CH_POSZ 11
#define CH_OFQ 12
#define CH_OFX 13
#define CH_OFY 14
#define CH_MARK 15

// OF coefficient to convert 16-bit data to rad/s
#define OF_COEFF (0.7f/4000)

// minimum safe packet rate. If rate is less than this, failsafe
// will be run.
#define MIN_RATE 40

// Main loop task
void
bee3b (void *arg __attribute__ ((unused)))
{
  std::memset (recv2, 0, 32);
  std::memset (send2, 0, 32);

  icm20602_init ();

  bitset flags;
  thread::poll_section ();
  flags.clear ();
  flags.add (eventflag::timeout_event (1000));
  thread::poll (flags);

  float gx_offs = 0, gy_offs = 0, gz_offs = 0;
  int calib_count = 0;
  int sample_count = 0;

  float rcrate = 0.0f;
  float vbat = 0.0f;
  float badj = 1.0f;
  float posz = 0.0f;
  float flowx = 0.0f, flowy = 0.0f;
  int flow_quality = 0;

  enum flightState state = ST_START;
  flightController FC;
  crashChecker CC;

  float c, si, sj, sk;

  float ch1 = 0.0f, ch2 = 0.0f, ch3 = 0.0f, ch4 = 0.0f;
  uint16_t mo1 = 0, mo2 = 0, mo3 = 0, mo4 = 0;
  uint16_t width[4];
  width[0] = width[1] = width[2] = width[3] = 0;
  uint16_t *command = (uint16_t *) recv2;
  int16_t *icommand = (int16_t *) recv2;

  while (1)
    {
      //WAIT INT
      if (!(EXTI->PR & (1 << 0)))
	{
	  thread::poll_section ();
	  flags.clear ();
	  id_t inven_irq_id = eventflag::irq_event (INTR_REQ_EXTI0);
	  flags.add (inven_irq_id);
	  id_t id = thread::poll (flags);
	}
      EXTI->PR = (1 << 0);

      int fifo_count = icm20602_fifo_count ();
      if (fifo_count != 0)
	{
	  int n = fifo_count / ICM20602_SIZE_OF_SAMPLE;
	  icm20602_read_fifo (n, &rx);

	  uint8_t *rp = rx.d;
	  for (int i = 0; i < n; i++, rp += ICM20602_SIZE_OF_SAMPLE)
	    {
	      float gx, gy, gz, ax, ay, az, t;
	      if (!icm20602_get_sample (rp, gx, gy, gz, ax, ay, az, t))
		{
		  sample_count = 0;
		  pfg.Reset (0.0f);
		  pfa.Reset (0.0f);
		  break;
		}
	      // Primary lowpass filter
	      pfg.Update (gx, gy, gz, sample_count % 8);
	      pfa.Update (ax, ay, az, sample_count % 8);
	      t = smat.Update(t);

	      sample_count = (sample_count + 1) % 32;

	      if ((sample_count) % 8 == 0)
		{
		  // Cancell initial bias
		  if (calib_count < IN_CALIB)
		    {
		      gx_offs = (1-CALIB_ALPHA)*gx_offs + CALIB_ALPHA*gx;
		      gy_offs = (1-CALIB_ALPHA)*gy_offs + CALIB_ALPHA*gy;
		      gz_offs = (1-CALIB_ALPHA)*gz_offs + CALIB_ALPHA*gz;
		      ++calib_count;
		      continue;
		    }
		  gx -= gx_offs; gy -= gy_offs; gz -= gz_offs;

		  if (enable_notch_g)
		    {
		      gx = GNoFx.Update (gx);
		      gy = GNoFy.Update (gy);
		      gz = GNoFz.Update (gz);
		    }
		  if (enable_notch_a)
		    {
		      ax = ANoFx.Update (ax);
		      ay = ANoFy.Update (ay);
		      az = ANoFz.Update (az);
		    }
		  // Filtered acc.
		  float axf = ALPFx.Update (ax);
		  float ayf = ALPFy.Update (ay);
		  float azf = ALPFz.Update (az);
		  // Acc. calibration.
		  calibrate_acc (axf, ayf, azf);
		  // Update rotor estimation at every 8 samples
		  //float c, si, sj, sk;
		  float oi, oj, ok;
		  // Convert NED frame into the normal frame.
		  // Since gz will be used as the coefficient of bivector
		  // e1^e2 e1 <-> e2 makes the of gz minus.
		  RI.Update (gy, gx, -gz, -ayf, -axf, azf,
			     c, si, sj, sk, oi, oj, ok);
		  // LPF 40Hz
		  oi = DLPFi.Update (oi);
		  oj = DLPFj.Update (oj);
		  ok = DLPFk.Update (ok);

		  if (command[CH_MARK] == 0xbe3b)
		    {
#if 0
		      // Set omega (D-term) compensator values
		      if (command[CH_ALT] == 1400)
			enable_notch_g = false;
		      else if (command[CH_ALT] == 1401)
			enable_notch_g = true;
		      else if (command[CH_ALT] == 1500)
			enable_notch_a = false;
		      else if (command[CH_ALT] == 1501)
			enable_notch_a = true;
#endif
		      cmd_adj = sat<float> ((command[CH_ADJ] - 965)/1100.0f,
					    0.0f, 1.0f);
		      cgain = cmd_adj;

		      rcrate = command[CH_RATE];
		      vbat = (1.0f/1024)*command[CH_BAT];
		      posz = 0.001f*command[CH_POSZ]; // mm -> m
		      // scale rad/s and extract horizontal movements
		      flow_quality = command[CH_OFQ]; // 0-255
		      if (flow_quality > 100)
			{
			  flowx = -(OF_COEFF*icommand[CH_OFX] - oj); // rad/s
			  flowy = (OF_COEFF*icommand[CH_OFY] - oi); // rad/s
			  flowx = FLPFx.Update(flowx);
			  flowy = FLPFy.Update(flowy);
			}
		      else
			{
			  flowx = 0;
			  flowy = 0;
			}

		      if (rcrate >= MIN_RATE)
			{
			  if (state == ST_START)
			    state = ST_SAS;
			  // Restart
			  if (state == ST_HALT
			      && command[0] == 0
			      && command[1] == 0
			      && command[2] == 0
			      && command[3] == 0)
			    state = ST_START;
			}
		      else
			{
			  if (state == ST_SAS)
			    state = ST_LANDING;
			}
#if 1
		      if (state == ST_LANDING)
			{
			  if (posz < 0.1f)
			    {
			      // Cut motors where 0.1m > hight anyway.
			      state = ST_HALT;
			    }
			}
		      state = CC.update (state, c, si, sj, sk);
#endif
		      if (state == ST_SAS)
			{
			  chmat (oi, oj, ok, 40.0f*cgain, ch1, ch2, ch3, ch4);
			  // motor-channel skewed. 0:ch4, 1:ch1, 2:ch2, 3:ch3
			  mo1=width[0] = chmix (command[0], (int) ch4, 50);
			  width[1] = chmix (command[1], (int) ch1, 50);
			  width[2] = chmix (command[2], (int) ch2, 50);
			  width[3] = chmix (command[3], (int) ch3, 50);
			  FC.hedge (c, si, sj, sk, -ayf, -axf, azf,
				    posz, flowx, flowy,
				    width[1], width[2], width[3], width[0]);
			}
		      else if (state == ST_LANDING)
			{
			  float ratio = sat<float>(2.0f-vbat/3.6f, 0.5f, 1.0f);
			  badj = (1-0.001f)*badj + 0.001f*(ratio*ratio);
			  // failsafe:
			  //  Try to keep level attitude and go to land
			  //  with a constant vertical velocity.
			  FC.update (c, si, sj, sk, oi, oj, ok,
				     -ayf, -axf, azf,
				     posz, flowx, flowy,
				     1250.0f, 320.0f, 80.0f, badj,
				     mo1, mo2, mo3, mo4);
			  width[0] = mo4;
			  width[1] = mo1;
			  width[2] = mo2;
			  width[3] = mo3;
			}
		      else
			{
			  if (state == ST_HALT)
			    FC.reset ();
			  width[0] = 0;
			  width[1] = 0;
			  width[2] = 0;
			  width[3] = 0;
			}
#if 1
		      tim4_set_pwm_width (width);
#endif
		    }
		  // Moving avarage for downsampling
		  gx = smagx.Update (gx);
		  gy = smagy.Update (gy);
		  gz = smagz.Update (gz);
		  ax = smaax.Update (ax);
		  ay = smaay.Update (ay);
		  az = smaaz.Update (az);
		}
	      if (sample_count == 0)
		{
		  // If slow output is updated
		  if (!(EXTI->PR & (1 << 6)))
		    {
		      thread::poll_section ();
		      flags.clear ();
		      id_t hs_irq_id = eventflag::irq_event (INTR_REQ_EXTI9_5);
		      flags.add (hs_irq_id);
		      id_t id = thread::poll (flags);
		    }
		  EXTI->PR = (1 << 6);

		  // Exchange packet at every ~1ms
		  union { float f; uint8_t bytes[sizeof(float)];} ux, uy, uz;
		  union { float f; uint8_t bytes[sizeof(float)];} ut;
		  ux.f = ax; uy.f = ay; uz.f = az;
		  memcpy (&send2[0], ux.bytes, sizeof(float));
		  memcpy (&send2[4], uy.bytes, sizeof(float));
		  memcpy (&send2[8], uz.bytes, sizeof(float));
		  ux.f = gx; uy.f = gy; uz.f = gz;
		  memcpy (&send2[12], ux.bytes, sizeof(float));
		  memcpy (&send2[16], uy.bytes, sizeof(float));
		  memcpy (&send2[20], uz.bytes, sizeof(float));
		  ut.f = t;
		  memcpy (&send2[24], ut.bytes, sizeof(float));
		  float br, bp, by, px, py, pz, vx, vy, vz;
		  FC.dump (br, bp, by, px, py, pz, vx, vy, vz);
		  send2[28] = (int8_t) (state);
		  send2[29] = (int8_t) (px*10);
		  send2[30] = (int8_t) (py*10);
		  send2[31] = (int8_t) (pz*10);
		  spi2_transfer (send2, recv2, 32);
		}
	    }
	}
    }
}

void
ef::main (void *arg  __attribute__ ((unused)))
{
  spi_init ();
  exti_init ();
  tim4_init ();

  board::set_led (true);

  thread *tp = thread::create (5, bee3b, NULL, NULL, 8192, 0);
  tp->run ();

  while(1)
    ;
}

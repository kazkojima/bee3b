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

#include "ef.h"
#include "mcu.h"
#include "spi.h"
#include <cstdlib>

using namespace ef;

//#define ROTATION_YAW	180
#define  ROTATION_YAW	0

// ICM20602
#define ICM20602_ID     0x12

// ICM20602 registers
#define XG_OFFSET_H     0x13
#define XG_OFFSET_L     0x14
#define YG_OFFSET_H     0x15
#define YG_OFFSET_L     0x16
#define ZG_OFFSET_H     0x17
#define ZG_OFFSET_L     0x18

#define SMPLRT_DIV      0x19
#define MPU_CONFIG      0x1A
#define GYRO_CONFIG     0x1B
#define ACCEL_CONFIG    0x1C
#define ACCEL_CONFIG2   0x1D

#define FIFO_EN         0x23

#define I2C_MST_CTRL    0x24
#define I2C_SLV0_ADDR   0x25
#define I2C_SLV0_REG    0x26
#define I2C_SLV0_CTRL   0x27

#define I2C_SLV4_CTRL   0x34

#define INT_PIN_CFG     0x37
#define INT_ENABLE      0x38
#define INT_STATUS      0x3A

#define ACCEL_XOUT_H    0x3B
#define ACCEL_XOUT_L    0x3C
#define ACCEL_YOUT_H    0x3D
#define ACCEL_YOUT_L    0x3E
#define ACCEL_ZOUT_H    0x3F
#define ACCEL_ZOUT_L    0x40
#define TEMP_OUT_H      0x41
#define TEMP_OUT_L      0x42
#define GYRO_XOUT_H     0x43
#define GYRO_XOUT_L     0x44
#define GYRO_YOUT_H     0x45
#define GYRO_YOUT_L     0x46
#define GYRO_ZOUT_H     0x47
#define GYRO_ZOUT_L     0x48

#define EXT_SENS_DATA_00 0x49
#define I2C_SLV0_DO     0x63
#define I2C_MST_DELAY_CTRL 0x67

#define USER_CTRL       0x6A
#define PWR_MGMT_1      0x6B
#define PWR_MGMT_2      0x6C

#define FIFO_COUNTH     0x72
#define FIFO_COUNTL     0x73
#define FIFO_R_W        0x74

#define WHO_IM_I        0x75

#define XA_OFFSET_H     0x77
#define XA_OFFSET_L     0x78
#define YA_OFFSET_H     0x7A
#define YA_OFFSET_L     0x7B
#define ZA_OFFSET_H     0x7D
#define ZA_OFFSET_L     0x7E

#define MPUREG_ICM_UNDOC1       0x11
#define MPUREG_ICM_UNDOC1_VALUE 0xc9

#define GRAVITY_MSS     9.80665f

#define FILTER_CONVERGE_COUNT 2000

// accelerometer scaling for 16g range
#define ICM20602_ACCEL_SCALE_1G    (GRAVITY_MSS / 2048.0f)

/*
 *  gyro as 16.4 LSB/DPS at scale factor of +/- 2000dps (FS_SEL==3)
 */
static const float GYRO_SCALE = 0.0174532f / 16.4f;

static const float TEMP_SCALE = 1.0f / 326.8f;
#define TEMP_OFFSET 25.0f

// ICM20602 IMU data are big endian
#define be16_val(v, idx) ((int16_t)(((uint16_t)(v)[2*idx] << 8) | (v)[2*idx+1]))

static void
wait_ms (uint32_t ms)
{
  if (!ms)
    return;

  bitset flags;
  thread::poll_section ();
  flags.clear ();
  flags.add (eventflag::timeout_event (ms * 1000));
  thread::poll (flags);
}

// buffers on DCTM
static uint8_t sendbuf[4];
static uint8_t recvbuf[4];

static uint8_t
icm20602_read (uint8_t reg)
{
  sendbuf[0] = reg | 0x80;
  sendbuf[1] = 0;
  bool rv = spi1_transfer(sendbuf, recvbuf, 2);
  return recvbuf[1];
}

static bool
icm20602_write (uint8_t reg, uint8_t val)
{
  sendbuf[0] = reg;
  sendbuf[1] = val;
  bool rv = spi1_transfer(sendbuf, recvbuf, 2);
  return rv;
}

static bool
icm20602_ready (void)
{
  uint8_t val = icm20602_read (INT_STATUS);
  return (val & 1);
}

struct sample {
  uint8_t reg;
  uint8_t d[14];
};

int
icm20602_fifo_count (void)
{
  bool rv;
  sendbuf[0] = 0x80 | FIFO_COUNTH;
  rv = spi1_transfer (sendbuf, recvbuf, 3);
  return rv ? be16_val(&recvbuf[1], 0) : 0;
}

// on DCTM
static struct sample sx;

static bool
icm20602_read_fifo (struct sample *rxp)
{
  bool rv;
  memset (&sx, 0, sizeof(struct sample));
  sx.reg = 0x80 | FIFO_R_W;
  rv = spi1_transfer ((uint8_t *)&sx, (uint8_t *)rxp, sizeof(struct sample));
  return rv;
}

// Check fifo entry with temperature
static bool check_fifo(int t)
{
  static int raw = -400;
  if (t - raw > -400 && t - raw < 400)
    return true;

  sendbuf[0] = 0x80 | TEMP_OUT_H;
  if (spi1_transfer (sendbuf, recvbuf, 3))
    raw = be16_val(&recvbuf[1], 0);

  return (t - raw > -400 && t - raw < 400);
}

static void
icm20602_fifo_reset (void)
{
    uint8_t val = icm20602_read (USER_CTRL);
    val &= ~0x44;
    icm20602_write (FIFO_EN, 0);
    icm20602_write (USER_CTRL, val);
    icm20602_write (USER_CTRL, val|0x04);
    icm20602_write (USER_CTRL, val|0x40);
    // All except external sensors
    icm20602_write (FIFO_EN, 0xf8);
    wait_ms (1);
}

static void
icm20602_start(void)
{
  icm20602_write (PWR_MGMT_2, 0);
  wait_ms (1);

  // 1: Set LPF to 184Hz 0: No LPF
  icm20602_write (MPU_CONFIG, /*(1<<6)|*/1);
  wait_ms (1);

  // Sample rate 1000Hz
  icm20602_write (SMPLRT_DIV, 0);
  wait_ms (1);

  // Gyro 2000dps
  icm20602_write (GYRO_CONFIG, 3<<3);
  wait_ms (1);

  // Accel full scale 16g
  icm20602_write (ACCEL_CONFIG, 3<<3);
  wait_ms (1);

  icm20602_write (MPUREG_ICM_UNDOC1, MPUREG_ICM_UNDOC1_VALUE);
  wait_ms (1);
    
  // Set LPF to 218Hz BW
  icm20602_write (ACCEL_CONFIG2, 1);
  wait_ms (1);

  uint8_t val;
  // INT enable on RDY
  icm20602_write (INT_ENABLE, 1);
  wait_ms (1);

  val = icm20602_read (INT_PIN_CFG);
  val |= 0x30;
  icm20602_write (INT_PIN_CFG, val);
  wait_ms (1);
}

bool
icm20602_init (void)
{
  uint8_t rv;
  rv = icm20602_read (WHO_IM_I);
  if (rv != ICM20602_ID) {
    return false;
  }

  uint8_t tries;
  for (tries = 0; tries < 5; tries++) {
    // Disable master I2C here
    if ((rv = icm20602_read (USER_CTRL)) & (1<<5))
      {
	icm20602_write (USER_CTRL, rv &~ (1<<5));
	wait_ms (10);
      }

    // Reset
    icm20602_write (PWR_MGMT_1, 0x80);
    wait_ms (100);

    // Disable I2C interface
    icm20602_write (USER_CTRL, 0x10);
    wait_ms (100);

    // Wake up with appropriate clock
    icm20602_write (PWR_MGMT_1, 0x03);
    wait_ms (5);
    if (icm20602_read (PWR_MGMT_1) == 0x03)
      break;

    wait_ms (10);
    if (icm20602_ready ())
      break;
  }

  if (tries == 5)
    {
      //printf("Failed to boot ICM20602 5 times");
      return false;
    }

  icm20602_start ();
  icm20602_fifo_reset ();

#if 0
  // Configure slaves
  // Set I2C_MST_EN, MST_P_NSR and set bus speed to 400kHz
  rv = icm20602_read (USER_CTRL);
  icm20602_write (USER_CTRL, rv | (1<<5));
  icm20602_write (I2C_MST_CTRL, (1<<4)|13);
  // Sample rate 100Hz
  icm20602_write (I2C_SLV4_CTRL, 9);
  icm20602_write (I2C_MST_DELAY_CTRL, 0x0f);
#endif
  return true;
}

// on DCTM
static struct sample rx;

bool
icm20602_get_values (float& gx, float& gy, float& gz,
		     float& ax, float& ay, float& az, float &temp)
{

  icm20602_read_fifo (&rx);

  int16_t t = be16_val(rx.d, 3);
  if (!check_fifo (t))
    {
      icm20602_fifo_reset();
      //printf("temp reset fifo %04x %d\n", t, fifo_count);
      return false;
    }

  // adjust and serialize floats into packet bytes
  // skew accel/gyro frames so to match AK8963 NED frame
#if (ROTATION_YAW == 0)
  ax = ((float)be16_val(rx.d, 1)) * ICM20602_ACCEL_SCALE_1G;
  ay = ((float)be16_val(rx.d, 0)) * ICM20602_ACCEL_SCALE_1G;
  az = -((float)be16_val(rx.d, 2)) * ICM20602_ACCEL_SCALE_1G;
#elif (ROTATION_YAW == 90)
  ax = -((float)be16_val(rx.d, 0)) * ICM20602_ACCEL_SCALE_1G;
  ay = ((float)be16_val(rx.d, 1)) * ICM20602_ACCEL_SCALE_1G;
  az = -((float)be16_val(rx.d, 2)) * ICM20602_ACCEL_SCALE_1G;
#elif (ROTATION_YAW == 180)
  ax = -((float)be16_val(rx.d, 1)) * ICM20602_ACCEL_SCALE_1G;
  ay = -((float)be16_val(rx.d, 0)) * ICM20602_ACCEL_SCALE_1G;
  az = -((float)be16_val(rx.d, 2)) * ICM20602_ACCEL_SCALE_1G;
#elif (ROTATION_YAW == 270)
  ax = ((float)be16_val(rx.d, 0)) * ICM20602_ACCEL_SCALE_1G;
  ay = -((float)be16_val(rx.d, 1)) * ICM20602_ACCEL_SCALE_1G;
  az = -((float)be16_val(rx.d, 2)) * ICM20602_ACCEL_SCALE_1G;
#else
#error "bad ROTATION_YAW value"
#endif

  temp = ((float)be16_val(rx.d, 3)) * TEMP_SCALE + TEMP_OFFSET;

#if (ROTATION_YAW == 0)
  gx = ((float)be16_val(rx.d, 5)) * GYRO_SCALE;
  gy = ((float)be16_val(rx.d, 4)) * GYRO_SCALE;
  gz = -((float)be16_val(rx.d, 6)) * GYRO_SCALE;
#elif (ROTATION_YAW == 90)
  gx = -((float)be16_val(rx.d, 4)) * GYRO_SCALE;
  gy = ((float)be16_val(rx.d, 5)) * GYRO_SCALE;
  gz = -((float)be16_val(rx.d, 6)) * GYRO_SCALE;
#elif (ROTATION_YAW == 180)
  gx = -((float)be16_val(rx.d, 5)) * GYRO_SCALE;
  gy = -((float)be16_val(rx.d, 4)) * GYRO_SCALE;
  gz = -((float)be16_val(rx.d, 6)) * GYRO_SCALE;
#elif (ROTATION_YAW == 270)
  gx = ((float)be16_val(rx.d, 4)) * GYRO_SCALE;
  gy = -((float)be16_val(rx.d, 5)) * GYRO_SCALE;
  gz = -((float)be16_val(rx.d, 6)) * GYRO_SCALE;
#else
#error "bad ROTATION_YAW value"
#endif
  return true;
}

#if 0
iven_uses ()
{
  while (1)
    {
      uint32_t gpio_num;
      //WAIT INT

      int fifo_count = icm20602_fifo_count ();
      if (fifo_count == 0)
	continue;

      int n_sample = fifo_count / sizeof(rx);
      while(n_sample--)
	{
	  float gx, gy, gz, ax, ay, az, t;
	  if (icm20602_get_values (gx, gy, gz, ax, ay, az, t))
	    {
	    }
	}
}
#endif

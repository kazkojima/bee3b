# Makefile

PROJECT = sample

### This is a LED example for Nucleo STM32F722

EFROOT=../..

LDSCRIPT= $(EFROOT)/mcu/stm32f722/ef.ld
CXXSRC = \
	$(EFROOT)/sys.cxx \
	$(EFROOT)/rcc.cxx \
	$(EFROOT)/gpio.cxx \
	$(EFROOT)/systimer.cxx \
	$(EFROOT)/usart.cxx \
	$(EFROOT)/irq.cxx \
	$(EFROOT)/heap.cxx \
	$(EFROOT)/context.cxx \
	$(EFROOT)/ef.cxx \
	spi.cxx \
	inven.cxx \
	pwm.cxx \
	rotor.cxx \
	biquadFilter.cxx \
	flight.cxx \
	efmain.cxx

CSRC = \
        $(EFROOT)/dummy-syscalls.c

###################################
CROSS = arm-none-eabi-
CC   = $(CROSS)gcc
CXX  = $(CROSS)g++
LD   = $(CROSS)g++
OBJCOPY   = $(CROSS)objcopy

MCU   = cortex-m7
CWARN = -Wall -Wextra
DEFS  = -DFREE_STANDING -DMHZ=216 -DMCU_STM32F722 -DUSE_FPU_AUTO_SAVE
OPT   = -O3 -Os -g -mfloat-abi=hard -mfpu=fpv4-sp-d16 -ffast-math
EXTRA_LDFLAGS = -mfloat-abi=hard -mfpu=fpv5-sp-d16 -ffast-math
LIBS  = -lm

include $(EFROOT)/rules.mk

distclean: clean

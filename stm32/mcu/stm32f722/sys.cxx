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

#include "ldsym.h"
#include "systimer.h"
#include "context.h"

typedef void (*initfp)(void);

static void
loop (void *arg __attribute__ ((unused)))
{
  while (1)
    ;
}

void __attribute__ ((naked, section(".startup.entry")))
ef::sys::entry (void)
{
  asm volatile ("cpsid	i" : : : "memory");
  mcu::rcc_init ();
  memset (&_bss_start, 0, &_bss_end - &_bss_start);
  memcpy (&_data, &_textdata, &_edata - &_data);
  for (initfp *fp = (initfp *)&__init_array_start;
       fp < (initfp *)&__init_array_end; fp++)
    (*fp) ();
  memcpy (sys::vector, sys::_vector, 1024);

  prio_init ();
  systimer::init ();
  mcu::gpio_init ();

  // Create idle thread
  thread::create (MAX_ID-1, loop, NULL, NULL, 256, 0);
  thread::runnable.add (MAX_ID-1);
  // Create main thread
  thread *tp = thread::create (MAX_ID-2, ef::main, NULL, loop, 512, 0);
  thread::runnable.add (MAX_ID-2);
  thread::running = tp;
  uint32_t psp = ((context *) tp)->sp;

  psp += sizeof (context_on_stack);

  // Use PSP for threads 
  asm ("msr	PSP, %0\n\t"
       "msr	CONTROL, %1\n\t"
       "isb"
       : : "r" (psp), "r" (2) : "memory");

  // Enable interrupts
  asm volatile ("cpsie	i" : : : "memory");

  ef::main (NULL);

  for (;;);
}

static void nmi (void)
{
  for (;;);
}

static void hard_fault (void)
{
  for (;;);
}

static void mem_manage (void)
{
  for (;;);
}

static void bus_fault (void)
{
  for (;;);
}

static void usage_fault (void)
{
  for (;;);
}

static void none (void)
{
}

typedef void (*handler)(void);

handler ef::sys::_vector[256] __attribute__ ((section(".startup.vectors"))) = {
  (handler)(&__initial_sp__),
  ef::sys::entry,
  nmi,			// -14:nmi
  hard_fault,		// -13:hard fault
  mem_manage,		// -12:mem manage
  bus_fault,		// -11:bus fault
  usage_fault,		// -10:usage fault
  none,			//  -9: reserved
  none,			//  -8: reserved
  none,			//  -7: reserved
  none,			//  -6: reserved
  ef::exc::svc,		//  -5: SVCall
  none,			//  -4: Debug
  none,			//  -3: reserved
  ef::exc::preempt,	//  -2: PendSV
  ef::exc::systick,	//  -1: SysTick
  ef::exc::irq_handler,	//   0: IRQ 0
  ef::exc::irq_handler, //   1: IRQ 1
  ef::exc::irq_handler, //   2: IRQ 2
  ef::exc::irq_handler, //   3: IRQ 3
  ef::exc::irq_handler, //   4: IRQ 4
  ef::exc::irq_handler, //   5: IRQ 5
  ef::exc::irq_handler, //   6: IRQ 6
  ef::exc::irq_handler, //   7: IRQ 7
  ef::exc::irq_handler, //   8: IRQ 8
  ef::exc::irq_handler, //   9: IRQ 9
  ef::exc::irq_handler, //  10: IRQ 10
  ef::exc::irq_handler, //  11: IRQ 11
  ef::exc::irq_handler, //  12: IRQ 12
  ef::exc::irq_handler, //  13: IRQ 13
  ef::exc::irq_handler, //  14: IRQ 14
  ef::exc::irq_handler, //  15: IRQ 15
  ef::exc::irq_handler, //  16: IRQ 16
  ef::exc::irq_handler, //  17: IRQ 17
  ef::exc::irq_handler, //  18: IRQ 18
  ef::exc::irq_handler, //  19: IRQ 19
  ef::exc::irq_handler, //  20: IRQ 20
  ef::exc::irq_handler, //  21: IRQ 21
  ef::exc::irq_handler, //  22: IRQ 22
  ef::exc::irq_handler, //  23: IRQ 23
  ef::exc::irq_handler, //  24: IRQ 24
  ef::exc::irq_handler, //  25: IRQ 25
  ef::exc::irq_handler, //  26: IRQ 26
  ef::exc::irq_handler, //  27: IRQ 27
  ef::exc::irq_handler, //  28: IRQ 28
  ef::exc::irq_handler, //  29: IRQ 29
  ef::exc::irq_handler, //  30: IRQ 30
  ef::exc::irq_handler, //  31: IRQ 31
  ef::exc::irq_handler, //  32: IRQ 32
  ef::exc::irq_handler, //  33: IRQ 33
  ef::exc::irq_handler, //  34: IRQ 34
  ef::exc::irq_handler, //  35: IRQ 35
  ef::exc::irq_handler, //  36: IRQ 36
  ef::exc::irq_handler, //  37: IRQ 37
  ef::exc::irq_handler, //  38: IRQ 38
  ef::exc::irq_handler, //  39: IRQ 39
  ef::exc::irq_handler, //  40: IRQ 40
  ef::exc::irq_handler, //  41: IRQ 41
  ef::exc::irq_handler, //  42: IRQ 42
  ef::exc::irq_handler, //  43: IRQ 43
  ef::exc::irq_handler, //  44: IRQ 44
  ef::exc::irq_handler, //  45: IRQ 45
  ef::exc::irq_handler, //  46: IRQ 46
  ef::exc::irq_handler, //  47: IRQ 47
  ef::exc::irq_handler, //  48: IRQ 48
  ef::exc::irq_handler, //  49: IRQ 49
  ef::exc::irq_handler, //  50: IRQ 50
  ef::exc::irq_handler, //  51: IRQ 51
  ef::exc::irq_handler, //  52: IRQ 52
  ef::exc::irq_handler, //  53: IRQ 53
  ef::exc::irq_handler, //  54: IRQ 54
  ef::exc::irq_handler, //  55: IRQ 55
  ef::exc::irq_handler, //  56: IRQ 56
  ef::exc::irq_handler, //  57: IRQ 57
  ef::exc::irq_handler, //  58: IRQ 58
  ef::exc::irq_handler, //  59: IRQ 59
  ef::exc::irq_handler, //  60: IRQ 60
  ef::exc::irq_handler, //  61: IRQ 61
  ef::exc::irq_handler, //  62: IRQ 62
  ef::exc::irq_handler, //  63: IRQ 63
  ef::exc::irq_handler, //  64: IRQ 64
  ef::exc::irq_handler, //  65: IRQ 65
  ef::exc::irq_handler, //  66: IRQ 66
  ef::exc::irq_handler, //  67: IRQ 67
  ef::exc::irq_handler, //  68: IRQ 68
  ef::exc::irq_handler, //  69: IRQ 69
  ef::exc::irq_handler, //  70: IRQ 70
  ef::exc::irq_handler, //  71: IRQ 71
  ef::exc::irq_handler, //  72: IRQ 72
  ef::exc::irq_handler, //  73: IRQ 73
  ef::exc::irq_handler, //  74: IRQ 74
  ef::exc::irq_handler, //  75: IRQ 75
  ef::exc::irq_handler, //  76: IRQ 76
  ef::exc::irq_handler, //  77: IRQ 77
  ef::exc::irq_handler, //  78: IRQ 78
  ef::exc::irq_handler, //  79: IRQ 79
  ef::exc::irq_handler, //  80: IRQ 80
  ef::exc::irq_handler, //  81: IRQ 81
  ef::exc::irq_handler, //  82: IRQ 82
  ef::exc::irq_handler, //  83: IRQ 83
  ef::exc::irq_handler, //  84: IRQ 84
  ef::exc::irq_handler, //  85: IRQ 85
  ef::exc::irq_handler, //  86: IRQ 86
  ef::exc::irq_handler, //  87: IRQ 87
  ef::exc::irq_handler, //  88: IRQ 88
  ef::exc::irq_handler, //  89: IRQ 89
  ef::exc::irq_handler, //  90: IRQ 90
  ef::exc::irq_handler, //  91: IRQ 91
  ef::exc::irq_handler, //  92: IRQ 92
  ef::exc::irq_handler, //  93: IRQ 93
  ef::exc::irq_handler, //  94: IRQ 94
  ef::exc::irq_handler, //  95: IRQ 95
  ef::exc::irq_handler, //  96: IRQ 96
  ef::exc::irq_handler, //  97: IRQ 97
  ef::exc::irq_handler, //  98: IRQ 98
  ef::exc::irq_handler, //  99: IRQ 99
  ef::exc::irq_handler, // 100: IRQ 100
  ef::exc::irq_handler, // 101: IRQ 101
  ef::exc::irq_handler, // 102: IRQ 102
  ef::exc::irq_handler, // 103: IRQ 103
  ef::exc::irq_handler, // 104: IRQ 104
  ef::exc::irq_handler, // 105: IRQ 105
  ef::exc::irq_handler, // 106: IRQ 106
  ef::exc::irq_handler, // 107: IRQ 107
  ef::exc::irq_handler, // 108: IRQ 108
  ef::exc::irq_handler, // 109: IRQ 109
  ef::exc::irq_handler, // 110: IRQ 110
  ef::exc::irq_handler, // 111: IRQ 111
  ef::exc::irq_handler, // 112: IRQ 112
  ef::exc::irq_handler, // 113: IRQ 113
  ef::exc::irq_handler, // 114: IRQ 114
  ef::exc::irq_handler, // 115: IRQ 115
  ef::exc::irq_handler, // 116: IRQ 116
  ef::exc::irq_handler, // 117: IRQ 117
  ef::exc::irq_handler, // 118: IRQ 118
  ef::exc::irq_handler, // 119: IRQ 119
  ef::exc::irq_handler, // 120: IRQ 120
  ef::exc::irq_handler, // 121: IRQ 121
  ef::exc::irq_handler, // 122: IRQ 122
  ef::exc::irq_handler, // 123: IRQ 123
  ef::exc::irq_handler, // 124: IRQ 124
  ef::exc::irq_handler, // 125: IRQ 125
  ef::exc::irq_handler, // 126: IRQ 126
  ef::exc::irq_handler, // 127: IRQ 127
  ef::exc::irq_handler, // 128: IRQ 128
  ef::exc::irq_handler, // 129: IRQ 129
  ef::exc::irq_handler, // 130: IRQ 130
  ef::exc::irq_handler, // 131: IRQ 131
  ef::exc::irq_handler, // 132: IRQ 132
  ef::exc::irq_handler, // 133: IRQ 133
  ef::exc::irq_handler, // 134: IRQ 134
  ef::exc::irq_handler, // 135: IRQ 135
  ef::exc::irq_handler, // 136: IRQ 136
  ef::exc::irq_handler, // 137: IRQ 137
  ef::exc::irq_handler, // 138: IRQ 138
  ef::exc::irq_handler, // 139: IRQ 139
  ef::exc::irq_handler, // 140: IRQ 140
  ef::exc::irq_handler, // 141: IRQ 141
  ef::exc::irq_handler, // 142: IRQ 142
  ef::exc::irq_handler, // 143: IRQ 143
  ef::exc::irq_handler, // 144: IRQ 144
  ef::exc::irq_handler, // 145: IRQ 145
  ef::exc::irq_handler, // 146: IRQ 146
  ef::exc::irq_handler, // 147: IRQ 147
  ef::exc::irq_handler, // 148: IRQ 148
  ef::exc::irq_handler, // 149: IRQ 149
  ef::exc::irq_handler, // 150: IRQ 150
  ef::exc::irq_handler, // 151: IRQ 151
  ef::exc::irq_handler, // 152: IRQ 152
  ef::exc::irq_handler, // 153: IRQ 153
  ef::exc::irq_handler, // 154: IRQ 154
  ef::exc::irq_handler, // 155: IRQ 155
  ef::exc::irq_handler, // 156: IRQ 156
  ef::exc::irq_handler, // 157: IRQ 157
  ef::exc::irq_handler, // 158: IRQ 158
  ef::exc::irq_handler, // 159: IRQ 159
  ef::exc::irq_handler, // 160: IRQ 160
  ef::exc::irq_handler, // 161: IRQ 161
  ef::exc::irq_handler, // 162: IRQ 162
  ef::exc::irq_handler, // 163: IRQ 163
  ef::exc::irq_handler, // 164: IRQ 164
  ef::exc::irq_handler, // 165: IRQ 165
  ef::exc::irq_handler, // 166: IRQ 166
  ef::exc::irq_handler, // 167: IRQ 167
  ef::exc::irq_handler, // 168: IRQ 168
  ef::exc::irq_handler, // 169: IRQ 169
  ef::exc::irq_handler, // 170: IRQ 170
  ef::exc::irq_handler, // 171: IRQ 171
  ef::exc::irq_handler, // 172: IRQ 172
  ef::exc::irq_handler, // 173: IRQ 173
  ef::exc::irq_handler, // 174: IRQ 174
  ef::exc::irq_handler, // 175: IRQ 175
  ef::exc::irq_handler, // 176: IRQ 176
  ef::exc::irq_handler, // 177: IRQ 177
  ef::exc::irq_handler, // 178: IRQ 178
  ef::exc::irq_handler, // 179: IRQ 179
  ef::exc::irq_handler, // 180: IRQ 180
  ef::exc::irq_handler, // 181: IRQ 181
  ef::exc::irq_handler, // 182: IRQ 182
  ef::exc::irq_handler, // 183: IRQ 183
  ef::exc::irq_handler, // 184: IRQ 184
  ef::exc::irq_handler, // 185: IRQ 185
  ef::exc::irq_handler, // 186: IRQ 186
  ef::exc::irq_handler, // 187: IRQ 187
  ef::exc::irq_handler, // 188: IRQ 188
  ef::exc::irq_handler, // 189: IRQ 189
  ef::exc::irq_handler, // 190: IRQ 190
  ef::exc::irq_handler, // 191: IRQ 191
  ef::exc::irq_handler, // 192: IRQ 192
  ef::exc::irq_handler, // 193: IRQ 193
  ef::exc::irq_handler, // 194: IRQ 194
  ef::exc::irq_handler, // 195: IRQ 195
  ef::exc::irq_handler, // 196: IRQ 196
  ef::exc::irq_handler, // 197: IRQ 197
  ef::exc::irq_handler, // 198: IRQ 198
  ef::exc::irq_handler, // 199: IRQ 199
  ef::exc::irq_handler, // 200: IRQ 200
  ef::exc::irq_handler, // 201: IRQ 201
  ef::exc::irq_handler, // 202: IRQ 202
  ef::exc::irq_handler, // 203: IRQ 203
  ef::exc::irq_handler, // 204: IRQ 204
  ef::exc::irq_handler, // 205: IRQ 205
  ef::exc::irq_handler, // 206: IRQ 206
  ef::exc::irq_handler, // 207: IRQ 207
  ef::exc::irq_handler, // 208: IRQ 208
  ef::exc::irq_handler, // 209: IRQ 209
  ef::exc::irq_handler, // 210: IRQ 210
  ef::exc::irq_handler, // 211: IRQ 211
  ef::exc::irq_handler, // 212: IRQ 212
  ef::exc::irq_handler, // 213: IRQ 213
  ef::exc::irq_handler, // 214: IRQ 214
  ef::exc::irq_handler, // 215: IRQ 215
  ef::exc::irq_handler, // 216: IRQ 216
  ef::exc::irq_handler, // 217: IRQ 217
  ef::exc::irq_handler, // 218: IRQ 218
  ef::exc::irq_handler, // 219: IRQ 219
  ef::exc::irq_handler, // 220: IRQ 220
  ef::exc::irq_handler, // 221: IRQ 221
  ef::exc::irq_handler, // 222: IRQ 222
  ef::exc::irq_handler, // 223: IRQ 223
  ef::exc::irq_handler, // 224: IRQ 224
  ef::exc::irq_handler, // 225: IRQ 225
  ef::exc::irq_handler, // 226: IRQ 226
  ef::exc::irq_handler, // 227: IRQ 227
  ef::exc::irq_handler, // 228: IRQ 228
  ef::exc::irq_handler, // 229: IRQ 229
  ef::exc::irq_handler, // 230: IRQ 230
  ef::exc::irq_handler, // 231: IRQ 231
  ef::exc::irq_handler, // 232: IRQ 232
  ef::exc::irq_handler, // 233: IRQ 233
  ef::exc::irq_handler, // 234: IRQ 234
  ef::exc::irq_handler, // 235: IRQ 235
  ef::exc::irq_handler, // 236: IRQ 236
  ef::exc::irq_handler, // 237: IRQ 237
  ef::exc::irq_handler, // 238: IRQ 238
  ef::exc::irq_handler, // 239: IRQ 239
};

handler ef::sys::vector[256] __attribute__ ((section(".vectors")));

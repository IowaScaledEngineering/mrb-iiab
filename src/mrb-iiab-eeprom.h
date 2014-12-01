/*************************************************************************
Title:    MRB-IIAB-EEPROM
Authors:  Michael D. Petersen <railfan@drgw.net>
File:     $Id: $
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2013 Michael Petersen & Nathan Holmes

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

*************************************************************************/

#ifndef MRB_IIAB_EEPROM
#define MRB_IIAB_EEPROM

// Timeout = debounce applied to positive detection when that detection goes away (used to "coast" through point-style optical detectors)
// Lockout = time after interlocking clears during which the train from the opposite direction (or same train leaving) cannot get the interlocking (lets train finish leaving)
// Timelock = delay between signal going red and any other train getting a proceed signal
// Debouce = debounce applied to interlocking block detection going low.  Filters momentary non-detects to prevent premature switch from OCCUPIED to CLEAR state.
#define EE_TIMEOUT_SECONDS      0x10
// 0x10 - 0x13 = 4 sets of timeouts, one for each direction
#define EE_LOCKOUT_SECONDS      0x14
#define EE_TIMELOCK_SECONDS     0x15
#define EE_DEBOUNCE_SECONDS     0x16
#define EE_CLOCK_SOURCE_ADDRESS 0x18
#define EE_MAX_DEAD_RECKONING   0x19
#define EE_SIM_TRAIN_WINDOW     0x1A
#define EE_BLINKY_DECISECS      0x1F
#define EE_INPUT_POLARITY0      0x20
#define EE_INPUT_POLARITY1      0x21
#define EE_OUTPUT_POLARITY0     0x22
#define EE_OUTPUT_POLARITY1     0x23
#define EE_OUTPUT_POLARITY2     0x24
#define EE_OUTPUT_POLARITY3     0x25
#define EE_OUTPUT_POLARITY4     0x26
#define EE_MISC_CONFIG          0x30
#define EE_SIM_TRAINS           0x40

#endif // MRB_IIAB_EEPROM

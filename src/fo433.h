#ifndef FO433_H_
#define FO433_H_ 1
/*
 * STC1000+, improved firmware and Arduino based firmware uploader for the STC-1000 dual stage thermostat.
 *
 * Copyright 2014 Mats Staffansson
 *
 * This file is part of STC1000+.
 *
 * STC1000+ is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * STC1000+ is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with STC1000+.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
#include "stc1000p.h"
#include "stc1000p_lib.h"
#include <stdint.h>

#define FO433_SEC       (60) /* 48 */
#define FO433_MAX_BYTES  (6)

#define FO433_PREAMBLE  (0xFF)
#define FO433_DEVICE_ID (0x45)

#define FO433_INIT       (0)
#define FO433_RDY_SEND   (1)
#define FO433_SEND_BIT   (2)
#define FO433_SEND_1     (3)
#define FO433_SEND_0     (4)
#define FO433_SEND_SPACE (5)

void    fo433_isr(void);
uint8_t fo433_crc8(uint8_t data, uint8_t crc);
void    fo433_fsm(void);

#endif

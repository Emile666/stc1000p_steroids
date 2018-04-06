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
#include "fo433.h"
#include <stdbool.h>
#include <stdint.h>

uint8_t fo433_data[FO433_MAX_BYTES];               // data bytes to send
uint8_t fo433_std                   = FO433_INIT;  // FSM state number  
uint8_t fo433_bit                   = 0;           // index of bit to send
uint8_t fo433_byte                  = 0;           // index of byte to send
uint8_t fo433_start                 = 0;           // 1 = start sending 6 bytes
uint16_t fo433_sec_count             = FO433_SEC/2; // second timer for fo433
uint8_t fo433_tmr                   = 0;           // FO433 timer

extern int16_t temp_ntc1; // The temperature in E-1 °C from NTC probe 1

/*-----------------------------------------------------------------------------
  Purpose  : This is the interrupt routine for the Fine Offset Protocol.
             It runs at 2 kHz (0.5 msec.) to ensure proper timing.
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
void fo433_isr(void)
{
	switch (fo433_std)
	{
		case FO433_INIT:
                PB_ODR    &= ~LED3; // Led on frontpanel off
				fo433_byte = 0;
				if (fo433_start) 
					fo433_std   = FO433_RDY_SEND;
				break;
		case FO433_RDY_SEND: // ready to send 1 byte
                PB_ODR    |= LED3; // Led on frontpanel on
				fo433_bit  = 0x80;
				if (++fo433_byte > FO433_MAX_BYTES)
				{
					fo433_start = 0; // reset flag for main program
					fo433_std   = FO433_INIT;
				} // if
				else fo433_std = FO433_SEND_BIT;
				break;
		case FO433_SEND_BIT: // prepare to send 1 bit
				PA_ODR &= ~D433_OUT;
				if (!fo433_bit) fo433_std = FO433_RDY_SEND; // send next byte
				else if (fo433_data[fo433_byte-1] & fo433_bit) 
                {
                    fo433_std = FO433_SEND_1; // send 1
                }
				else                        
				{
					fo433_tmr = 0;            // init. timer
					fo433_std = FO433_SEND_0; // send 0
				} // else
				break;
		case FO433_SEND_1: // send 1 (0.5 msec.)
				PA_ODR |= D433_OUT;
				fo433_tmr = 0;
				fo433_std = FO433_SEND_SPACE; // send space
				break;
		case FO433_SEND_0: // send 0 (1.5 msec.)
				PA_ODR    |= D433_OUT;
				if (++fo433_tmr > 2) 
				{
					fo433_tmr = 0;
					fo433_std = FO433_SEND_SPACE; // send space
				} // if
				break;
		case FO433_SEND_SPACE: // send space (1 msec)
				PA_ODR &= ~D433_OUT;
				fo433_bit >>= 1;            // next bit
				fo433_std = FO433_SEND_BIT; // adds another 0.5 msec.
				break;
	} // switch
} // fo433_isr()

/*-----------------------------------------------------------------------------
  Purpose  : This function calculates the CRC8 value: x^8 + x^5 + x^4 + 1
  Variables: data: the new data-byte to include in the crc
             crc : the current crc value
  Returns  : the new crc value
  ---------------------------------------------------------------------------*/
uint8_t fo433_crc8(uint8_t data, uint8_t crc)
{
	uint8_t i = 8;
	uint8_t mix;
	
	while (i-- > 0)
	{    
		mix = (crc ^ data) & 0x80;
		crc <<= 1;
		if (mix)
		{
			crc ^= 0x31;
		}
		data <<= 1;
	} // while
	return crc;
} // fo433_crc8()

/*-----------------------------------------------------------------------------
  Purpose  : This functions contains the main part of the Fine Offset protocol.
             It should be called every second to ensure proper timing.
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
void fo433_fsm(void)
{
	int16_t t;
	uint8_t crc, di;
	
	if (--fo433_sec_count == 0)
	{
		fo433_sec_count = FO433_SEC; // init. timer
	} // if
	if (!fo433_start && (fo433_sec_count < 3))
	{
	   fo433_data[0] = FO433_PREAMBLE;  // Pre-amble
	   fo433_data[1] = FO433_DEVICE_ID; // Device-ID MSB
	   di            = ((unsigned char)eeprom_read_config(EEADR_MENU_ITEM(dI))) << 4;
	   t             = temp_ntc1;
	   fo433_data[2] = (di | ((t >> 8) & 0xf));
	   crc           = fo433_crc8(fo433_data[2],0x00);
	   fo433_data[3] = (uint8_t)t;
	   crc           = fo433_crc8(fo433_data[3],crc);
	   fo433_data[4] = (HEAT_STATUS << 6) | (COOL_STATUS << 4);
	   crc           = fo433_crc8(fo433_data[4],crc);
	   fo433_data[5] = crc;
	   fo433_start   = true;	   
	} // if
} // fo433_fsm()


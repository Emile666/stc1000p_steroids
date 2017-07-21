/*==================================================================
  File Name    : adc.h
  Author       : Emile
  ------------------------------------------------------------------
  Purpose : This is the header-file for adc.c
  ------------------------------------------------------------------
  STC1000+ is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
 
  STC1000+ is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with STC1000+.  If not, see <http://www.gnu.org/licenses/>.
  ------------------------------------------------------------------
  $Log: adc.h,v $
  ==================================================================
*/ 
#ifndef STC1000P_ADC_H
#define STC1000P_ADC_H

#include "stdint.h"
#include <stdbool.h>
#include "stc1000p.h"

#define FILTER_SHIFT  (6)
#define ADC_AVG      (16)

// Defines for ADC_CRx registers
#define ADON    (0x01)
#define ALIGN   (0x08)
#define DBUF    (0x80)
#define CH_MASK (0x0F)
#define EOC     (0x80)

// Function prototypes
uint16_t read_adc(uint8_t ch);
int16_t  ad_to_temp(uint16_t adfilter, uint8_t *err);

#endif

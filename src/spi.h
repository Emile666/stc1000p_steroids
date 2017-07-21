/*==================================================================
   File Name    : spi.h
   Author       : Emile
  ------------------------------------------------------------------
  Purpose : This files contains the SPI related functions 
            for the STM8 uC.
            It is meant for the STC1000 thermostat hardware WR-032.
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
  ================================================================== */ 
#ifndef _SPI_H
#define _SPI_H   1

#include <iostm8s105.h>
#include <stdbool.h>
#include "stdint.h"

#define SPI_CR1_SPE (0x40)
#define SPI_SR_BSY  (0x80)
#define SPI_SR_TXE  (0x02)
#define SPI_SR_RXNE (0x01)

void    spi_init(void); // Initializes the SPI Interface. Needs to be called only once
void    spi_write(uint8_t data);
uint8_t spi_read(void);
#endif

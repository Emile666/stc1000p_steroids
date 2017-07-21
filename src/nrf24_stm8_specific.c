/*==================================================================
  File Name    : nrf24_stm8_specific.c
  Author       : Emile
  ------------------------------------------------------------------
  Purpose : This files contains the specific hardware related
            functions for the SPI interface for the STM8S105C6T6 uC.
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
  $Log: $
  ==================================================================
*/ 
#include "nrf24_stm8_specific.h"

/****************************************************************************/
void nrf24_csn(uint8_t mode)
{
  // Minimum ideal SPI bus speed is 2x data rate
  // If we assume 2Mbps data rate and 16Mhz clock, a
  // divider of 4 is the minimum we want.
  // CLK:BUS 8Mhz:2Mhz, 16Mhz:4Mhz, or 20Mhz:5Mhz
  if (mode)
  {   // HIGH
      PE_ODR  |=  0x20;        // Disable Chip-Select (CSN == PE5)
      SPI_CR1 &= ~SPI_CR1_SPE; // Disable SPI-device
  } // if
  else 
  {   // LOW
      PE_ODR  &= ~0x20;        // Enable Chip-Select
      SPI_CR1 |= SPI_CR1_SPE;  // Enable SPI-device
  } // else
} // csn()

/****************************************************************************/
void nrf24_ce(uint8_t level)
{
  if (level)
       PE_ODR |=  0x08;  // CE == PE3
  else PE_ODR &= ~0x08;
} // ce()

/****************************************************************************/
void nrf24_init(void)
{
  // Setting up all GPIO ports for input/output is already done in main()	
  spi_init(); // Initialize SPI bus

  nrf24_ce(LOW);
  nrf24_csn(HIGH);
} // nrf24_init()

/****************************************************************************/
uint8_t spi_transfer(uint8_t tx)
{
	spi_write(tx);
	return spi_read();
} // spi_transfer()

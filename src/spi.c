/*==================================================================
  File Name    : spi.c
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
  ------------------------------------------------------------------
  $Log: $
  ==================================================================
*/ 
#include "spi.h"
#include "stc1000p.h"
#include "delay.h"

uint8_t spi_error;

//  SPI interrupt handler.
@interrupt void SPI_Handler(void)
{   // Useful for debugging purposes
    spi_error = SPI_DR;
    spi_error = SPI_SR; // This clears the OVR bit
} // I2C_IRQHandler()

/*-----------------------------------------------------------------------------
  Purpose  : This function initializes the SPI bus controller
  Variables: --
  Returns  : --
  ---------------------------------------------------------------------------*/
void spi_init(void)
{
	SPI_CR1   &= ~SPI_CR1_SPE; // Disable SPI
    SPI_CR1    = 0x0C;         // MSB first, disable SPI, 4 MHz clock, Master mode, SPI mode 0
	SPI_CR2    = 0x03;         // Select SW Slave Management and Master mode
	SPI_ICR    = 0x00;         // Disable SPI interrupt on error
	// SPI_NSS is already set to Output and a high-level by GPIO init.
} // spi_init()

/*-----------------------------------------------------------------------------
  Purpose  : This function sends one byte to the SPI device
  Variables: data: byte to be transferred
  Returns  : -
  ---------------------------------------------------------------------------*/
void spi_write(uint8_t data)
{
	while (!(SPI_SR & SPI_SR_TXE)) delay_usec(5); // wait until TX Register is empty
	SPI_DR = data; // send byte over SPI bus
	while (!(SPI_SR & SPI_SR_TXE)) delay_usec(5); // wait until TX Register is empty
	while (SPI_SR & SPI_SR_BSY)    delay_usec(5); // wait until SPI is not busy anymore
} // spi_write()

/*-----------------------------------------------------------------------------
  Purpose  : This function reads one byte from the SPI device
  Variables: --
  Returns  : byte read from the I2C device
  ---------------------------------------------------------------------------*/
uint8_t spi_read(void)
{   
    uint8_t x1, reg;
    
	while (!(SPI_SR & SPI_SR_RXNE)) delay_usec(5); // wait until RX Register is full
    reg = SPI_DR;
    x1  = SPI_SR; // clear possible OVR flag
    return reg;
} // spi_read()


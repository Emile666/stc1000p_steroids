/*==================================================================
  File Name    : uart.h
  Author       : Emile
  ------------------------------------------------------------------
  Purpose : This files contains the header file for uart.c.
	    Original source at: 
            https://lujji.github.io/blog/bare-metal-programming-stm8/
  ------------------------------------------------------------------
  UART is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
 
  UART is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with UART. If not, see <http://www.gnu.org/licenses/>.
  ==================================================================
*/ 
#ifndef _UART_H_
#define _UART_H_

#include "stdint.h"
#include <stdbool.h>

//-----------------------------------------------------------------------
// Hardware defines for register definitions
// These value were defined in IAR, but not in Cosmic STM8
//-----------------------------------------------------------------------
#define UART2_CR2_TIEN    (0x80) /* Transmitter Interrupt Enable */
#define UART2_CR2_RIEN    (0x20) /* Receiver Interrupt Enable */
#define UART2_CR2_TEN     (0x08) /* Transmitter Enable */
#define UART2_CR2_REN     (0x04) /* Receiver Enable */
#define UART2_SR_TC       (0x40) /* Transmission Complete */
#define UART2_SR_RXNE     (0x20) /* Read data register not empty */
#define UART2_CR3_CLKEN   (0x08) /* Uart clock enable */

#define F_CPU       (16000000L)
#define BAUDRATE      (115200L)
#define UART_BUFLEN        (10)

#define TX_BUF_SIZE (20)
#define RX_BUF_SIZE (10)

void    uart_init(void);
void    uart_write(uint8_t data);
uint8_t uart_read(void);
void    xputs(uint8_t *s);
bool    uart_kbhit(void); /* returns true if character in receive buffer */

#endif
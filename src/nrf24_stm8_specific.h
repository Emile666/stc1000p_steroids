#ifndef _NRF24_STM8_SPECIFIC_H
#define _NRF24_STM8_SPECIFIC_H

#include <iostm8s105.h>
#include "spi.h"
#include "stdint.h"

#define HIGH (0x01)
#define LOW  (0x00) 

void    nrf24_csn(uint8_t mode);
void    nrf24_ce(uint8_t level);
void    nrf24_init(void);
uint8_t spi_transfer(uint8_t tx);

#endif
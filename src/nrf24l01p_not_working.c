/*
 Copyright (C) 2011 J. Coliz <maniacbug@ymail.com>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 */
#include "nrf24l01p.h"
#include "spi.h"
#include "delay.h"

// Driver for nRF24L01(+) 2.4GHz Wireless Transceiver
uint8_t  csn_pin; /* PE5: SPI Chip select */
uint8_t  ce_pin;  /* PE3: "Chip Enable" pin, activates the RX or TX role */
bool     wide_band = true; /* 2Mbs data rate in use? */
bool     p_variant = true; /* False for RF24L01 and true for RF24L01P */
uint8_t  payload_size = 32; /* Fixed size of payloads */
bool     ack_payload_available = false; /* Whether there is an ack payload waiting */
bool     dynamic_payloads_enabled = false; /* Whether dynamic payloads are enabled. */ 
uint8_t  ack_payload_length; /* Dynamic size of pending ack payload. */
pipe_addr pipe0_reading_address = {0x00,0x00,0x00,0x00,0x00}; /* Last address set on pipe 0 for reading. */

/****************************************************************************/
void csn(uint8_t mode)
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
void ce(uint8_t level)
{
  if (level)
       PE_ODR |=  0x08;  // CE == PE3
  else PE_ODR &= ~0x08;
} // ce()

/****************************************************************************/
uint8_t read_register(uint8_t reg, uint8_t* buf, uint8_t len)
{
  uint8_t status;

  csn(LOW);
  spi_write(R_REGISTER | (REGISTER_MASK & reg));
  spi_read(); // 1st byte read is the status register
  while ( len-- )
  {
    spi_write(NOP);
    *buf++ = spi_read();
  } // while
  csn(HIGH);
  return status;
} // read_register()

/****************************************************************************/
uint8_t read_register1(uint8_t reg)
{
  uint8_t result;
  
  csn(LOW);
  spi_write(R_REGISTER | (REGISTER_MASK & reg));
  result = spi_read(); // 1st byte read is the status register
  if (reg != STATUS)
  { 
    spi_write(NOP);
    result = spi_read();
  } // if
  csn(HIGH);
  return result;
} // read_register1()

/****************************************************************************/
void write_register(uint8_t reg, uint8_t* buf, uint8_t len)
{
  csn(LOW);
  spi_write(W_REGISTER | (REGISTER_MASK & reg));
  while ( len-- )
    spi_write(*buf++);
  csn(HIGH);
} // write_register()

/****************************************************************************/
void write_register1(uint8_t reg, uint8_t value)
{
  //IF_SERIAL_DEBUG(printf_P(PSTR("write_register(%02x,%02x)\r\n"),reg,value));
  csn(LOW);
  spi_write(W_REGISTER | (REGISTER_MASK & reg));
  spi_write(value);
  csn(HIGH);
} // write_register1()

/****************************************************************************/
void write_payload(uint8_t *buf, uint8_t len)
{
  uint8_t data_len  = min(len,payload_size);
  uint8_t blank_len = dynamic_payloads_enabled ? 0 : payload_size - data_len;
  
  //printf("[Writing %u bytes %u blanks]",data_len,blank_len);
  
  csn(LOW);
  spi_write(W_TX_PAYLOAD);
  while ( data_len-- )
    spi_write(*buf++);
  while ( blank_len-- )
    spi_write(0);
  csn(HIGH);
} // write_payload()

/****************************************************************************/
void read_payload(uint8_t *buf, uint8_t len)
{
  uint8_t data_len = min(len,payload_size);
  uint8_t blank_len = dynamic_payloads_enabled ? 0 : payload_size - data_len;
  
  //printf("[Reading %u bytes %u blanks]",data_len,blank_len);
  
  csn(LOW);
  spi_write(R_RX_PAYLOAD);
  spi_read(); // 1st byte read is the status register
  while (data_len--)
  {
    spi_write(NOP);
    *buf++ = spi_read();
  } // while
  while (blank_len--)
  {
    spi_write(NOP);
    spi_read();
  } // while
  csn(HIGH);
} // read_payload()

/****************************************************************************/
void flush_rx(void)
{
  csn(LOW);
  spi_write(FLUSH_RX);
  csn(HIGH);
} // flush_rx()

/****************************************************************************/
void flush_tx(void)
{
  csn(LOW);
  spi_write(FLUSH_TX);
  csn(HIGH);
} // flush_tx()

/****************************************************************************/
uint8_t get_status(void)
{
  uint8_t status = 0;

  csn(LOW);
  spi_write(NOP);
  status = spi_read(); // 1st byte read is the status register
  csn(HIGH);
  return status;
} // get_status()

/****************************************************************************/
void print_status(uint8_t status)
{
} // print_status()

/****************************************************************************/
void print_observe_tx(uint8_t value)
{
} // print_observe_tx()

/****************************************************************************/
void print_byte_register(char* name, uint8_t reg, uint8_t qty)
{
} // print_byte_register()

/****************************************************************************/
void print_address_register(char* name, uint8_t reg, uint8_t qty)
{
  uint8_t buffer[PIPE_ADDR_LEN];
  uint8_t* bufptr;

  while (qty--)
  {
    read_register(reg++,buffer,PIPE_ADDR_LEN);
    //printf_P(PSTR(" 0x"));
    //bufptr = buffer + 5;
    //while( --bufptr >= buffer )
    //printf_P(PSTR("%02x"),*bufptr);
  } // while
  // printf_P(PSTR("\r\n"));
} // print_address_register()

/****************************************************************************/
void setChannel(uint8_t channel)
{
  // TODO: This method could take advantage of the 'wide_band' calculation
  // done in setChannel() to require certain channel spacing.

  const uint8_t max_channel = 127;
  write_register1(RF_CH,min(channel,max_channel));
} // setChannel()

/****************************************************************************/
void setPayloadSize(uint8_t size)
{
  const uint8_t max_payload_size = 32;
  payload_size = min(size,max_payload_size);
} // setPayloadSize()

/****************************************************************************/
uint8_t getPayloadSize(void)
{
  return payload_size;
} // getPayloadSize()

/****************************************************************************/
void printDetails(void)
{
} // printDetails()

/****************************************************************************/
void init_nrf24l01p(void)
{
  uint8_t x;
  
  spi_init(); // Initialize SPI bus

  ce(LOW);
  csn(HIGH);

  // Must allow the radio time to settle else configuration bits will not necessarily stick.
  // This is actually only required following power up but some settling time also appears to
  // be required after resets too. For full coverage, we'll always assume the worst.
  // Enabling 16b CRC is by far the most obvious case if the wrong timing is used - or skipped.
  // Technically we require 4.5ms + 14us as a worst case. We'll just call it 5ms for good measure.
  // WARNING: Delay is based on P-variant whereby non-P *may* require different timing.
  delay_msec(5);

  // Set 1500uS (minimum for 32B payload in ESB@250KBPS) timeouts, to make testing a little easier
  // WARNING: If this is ever lowered, either 250KBS mode with AA is broken or maximum packet
  // sizes must never be used. See documentation for a more complete explanation.
  write_register1(SETUP_RETR,(0x04 << ARD) | (0x0f << ARC));

  // Restore our default PA level
  setPALevel(RF24_PA_LOW);
  
  // Determine if this is a p or non-p RF24 module and then
  // reset our data rate back to default value. This works
  // because a non-P variant won't allow the data rate to
  // be set to 250Kbps.
  if (setDataRate( RF24_250KBPS))
  {
    p_variant = true ;
  } // if
  
  // Then set the data rate to the slowest (and most reliable) speed supported by all
  // hardware.
  setDataRate(RF24_1MBPS);

  // Initialize CRC and request 2-byte (16bit) CRC
  setCRCLength(RF24_CRC_16);
  
  // Disable dynamic payloads, to match dynamic_payloads_enabled setting
  write_register1(DYNPD,0);

  // Reset current status
  // Notice reset and flush is the last thing we do
  write_register1(STATUS,_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );

  // Set up default configuration.  Callers can always change it later.
  // This channel should be universally safe and not bleed over into adjacent
  // spectrum.
  setChannel(110);
  x = read_register1(RF_CH);
  x = x+1;

  // Flush buffers
  flush_rx();
  flush_tx();
} // init_nrf24l01p()

/****************************************************************************/
void startListening(void)
{
  write_register1(CONFIG, read_register1(CONFIG) | _BV(PWR_UP) | _BV(PRIM_RX));
  write_register1(STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );

  // Restore the pipe0 adddress, if exists
  if (pipe0_reading_address.a[0] | pipe0_reading_address.a[1] | 
      pipe0_reading_address.a[2] | pipe0_reading_address.a[3] | pipe0_reading_address.a[4])
    write_register(RX_ADDR_P0, (uint8_t *)&pipe0_reading_address, PIPE_ADDR_LEN);

  // Flush buffers
  flush_rx();
  flush_tx();

  // Go!
  ce(HIGH);

  // wait for the radio to come up (130us actually only needed)
  delay_usec(130);
} // startListening()

/****************************************************************************/
void stopListening(void)
{
  ce(LOW);
  flush_tx();
  flush_rx();
} // stopListening()

/****************************************************************************/
void powerDown(void)
{
  write_register1(CONFIG,read_register1(CONFIG) & ~_BV(PWR_UP));
} // powerDown()

/****************************************************************************/
void powerUp(void)
{
  write_register1(CONFIG,read_register1(CONFIG) | _BV(PWR_UP));
} // PowerUp()

/******************************************************************/
bool write( const void* buf, uint8_t len )
{
  bool     result = false;
  uint8_t  observe_tx;
  uint8_t  status;
  uint32_t sent_at;
  uint32_t timeout; //ms to wait for timeout
  bool     tx_ok, tx_fail;

  // Begin the write
  startWrite(buf,len);

  // ------------
  // At this point we could return from a non-blocking write, and then call
  // the rest after an interrupt

  // Instead, we are going to block here until we get TX_DS (transmission completed and ack'd)
  // or MAX_RT (maximum retries, transmission failed).  Also, we'll timeout in case the radio
  // is flaky and we get neither.

  // IN the end, the send should be blocking.  It comes back in 60ms worst case, or much faster
  // if I tighted up the retry logic.  (Default settings will be 1500us.
  // Monitor the send
  sent_at = millis();
  timeout = 500; //ms to wait for timeout

  do
  {
    status = read_register(OBSERVE_TX,&observe_tx,1);
    //IF_SERIAL_DEBUG(Serial.print(observe_tx,HEX));
  }
  while( ! ( status & ( _BV(TX_DS) | _BV(MAX_RT) ) ) && ( millis() - sent_at < timeout ) );

  // The part above is what you could recreate with your own interrupt handler,
  // and then call this when you got an interrupt
  // ------------

  // Call this when you get an interrupt
  // The status tells us three things
  // * The send was successful (TX_DS, bit 5)
  // * The send failed, too many retries (MAX_RT, bit 4)
  // * There is an ack packet waiting (RX_DR, bit 6)
  status = whatHappened();
  ack_payload_available = status & _BV(RX_DR);
  //printf("%u%u%u\r\n",tx_ok,tx_fail,ack_payload_available);

  result = tx_ok;
  //IF_SERIAL_DEBUG(Serial.print(result?"...OK.":"...Failed"));

  // Handle the ack packet
  if (ack_payload_available)
  {
    ack_payload_length = getDynamicPayloadSize();
    //IF_SERIAL_DEBUG(Serial.print("[AckPacket]/"));
    //IF_SERIAL_DEBUG(Serial.println(ack_payload_length,DEC));
  } // if
  // Yay, we are done.
  powerDown(); // Power down
  flush_tx(); // Flush buffers (Is this a relic of past experimentation, and not needed anymore??)
  return result;
} // write()

/****************************************************************************/
void startWrite( const void* buf, uint8_t len )
{
  uint8_t reg = (read_register1(CONFIG) | _BV(PWR_UP)) & ~_BV(PRIM_RX);
  write_register1(CONFIG,reg); // Transmitter power-up
  delay_usec(150);
  write_payload(buf,len);      // Send the payload
  // Allons!
  ce(HIGH);
  delay_usec(15);
  ce(LOW);
} // startWrite()

/****************************************************************************/
uint8_t getDynamicPayloadSize(void)
{
  uint8_t result = 0;

  csn(LOW);
  spi_write(R_RX_PL_WID);
  spi_read(); // 1st byte read is the status register
  spi_write(NOP);
  result = spi_read();
  csn(HIGH);
  return result;
} // getDynamicPayloadSize()

/****************************************************************************/
bool available1(uint8_t* pipe_num)
{
  uint8_t status = get_status();

  // Too noisy, enable if you really want lots o data!!
  //IF_SERIAL_DEBUG(print_status(status));

  bool result = (status & _BV(RX_DR));

  if (result)
  {
    // If the caller wants the pipe number, include that
    if ( pipe_num )
      *pipe_num = (status >> RX_P_NO) & 0x07;

    // Clear the status bit

    // ??? Should this REALLY be cleared now?  Or wait until we
    // actually READ the payload?
    write_register1(STATUS,_BV(RX_DR) );

    // Handle ack payload receipt
    if (status & _BV(TX_DS))
    {
      write_register1(STATUS,_BV(TX_DS));
    } // if
  } // if
  return result;
} // available()

/****************************************************************************/
bool read(void* buf, uint8_t len)
{
  // Fetch the payload
  read_payload(buf,len);

  // was this the last of the data available?
  return read_register1(FIFO_STATUS) & _BV(RX_EMPTY);
} // read()

/****************************************************************************/
uint8_t whatHappened(void)
{
  uint8_t status = read_register1(STATUS); // Read the status
  
  write_register1(STATUS,_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) ); // reset bits
  return status; // Report to the user what happened
} // whatHappened()

/****************************************************************************/
void openWritingPipe(pipe_addr *value)
{
  uint8_t max_payload_size = 32;
  // Note that AVR 8-bit uC's store this LSB first, and the NRF24L01(+)
  // expects it LSB first too, so we're good.

  write_register(RX_ADDR_P0, (uint8_t *)value, PIPE_ADDR_LEN);
  write_register(TX_ADDR   , (uint8_t *)value, PIPE_ADDR_LEN);

  write_register1(RX_PW_P0,min(payload_size,max_payload_size));
} // openWritingPipe()

/****************************************************************************/
const uint8_t child_pipe[] =
{
  RX_ADDR_P0, RX_ADDR_P1, RX_ADDR_P2, RX_ADDR_P3, RX_ADDR_P4, RX_ADDR_P5
};
const uint8_t child_payload_size[] =
{
  RX_PW_P0, RX_PW_P1, RX_PW_P2, RX_PW_P3, RX_PW_P4, RX_PW_P5
};
const uint8_t child_pipe_enable[] =
{
  ERX_P0, ERX_P1, ERX_P2, ERX_P3, ERX_P4, ERX_P5
};

/****************************************************************************/
void openReadingPipe(uint8_t child, pipe_addr *address)
{
  uint8_t i;  
  // If this is pipe 0, cache the address.  This is needed because
  // openWritingPipe() will overwrite the pipe 0 address, so
  // startListening() will have to restore it.
  if (child == 0)
  {
    for (i = 0; i < PIPE_ADDR_LEN; i++) pipe0_reading_address.a[i] = address->a[i];
  } // if
  
  if (child <= 6)
  {
    // For pipes 2-5, only write the LSB
    if ( child < 2 )
         write_register(child_pipe[child] ,(uint8_t *)address,PIPE_ADDR_LEN);
    else write_register1(child_pipe[child],address->a[0]);

    write_register1(child_payload_size[child],payload_size);

    // Note it would be more efficient to set all of the bits for all open
    // pipes at once.  However, I thought it would make the calling code
    // more simple to do it this way.
    write_register1(EN_RXADDR,read_register1(EN_RXADDR) | _BV(child_pipe_enable[child]));
  } // if
} // openRadingPipe()

/****************************************************************************/
void toggle_features(void)
{
  csn(LOW);
  spi_write(ACTIVATE);
  spi_write(0x73);
  csn(HIGH);
} // toggle_features()

/****************************************************************************/
void enableDynamicPayloads(void)
{
  // Enable dynamic payload throughout the system
  write_register1(FEATURE,read_register1(FEATURE) | _BV(EN_DPL) );

  // If it didn't work, the features are not enabled
  if (!read_register1(FEATURE) )
  {
    // So enable them and try again
    toggle_features();
    write_register1(FEATURE,read_register1(FEATURE) | _BV(EN_DPL) );
  } // if

  //IF_SERIAL_DEBUG(printf("FEATURE=%i\r\n",read_register(FEATURE)));
  // Enable dynamic payload on all pipes
  //
  // Not sure the use case of only having dynamic payload on certain
  // pipes, so the library does not support it.
  write_register1(DYNPD,read_register1(DYNPD) | _BV(DPL_P5) | _BV(DPL_P4) | _BV(DPL_P3) | _BV(DPL_P2) | _BV(DPL_P1) | _BV(DPL_P0));

  dynamic_payloads_enabled = true;
}

/****************************************************************************/
void enableAckPayload(void)
{
  //
  // enable ack payload and dynamic payload features
  //
  write_register1(FEATURE,read_register1(FEATURE) | _BV(EN_ACK_PAY) | _BV(EN_DPL) );

  // If it didn't work, the features are not enabled
  if (!read_register1(FEATURE) )
  {
    // So enable them and try again
    toggle_features();
    write_register1(FEATURE,read_register1(FEATURE) | _BV(EN_ACK_PAY) | _BV(EN_DPL) );
  }

  //IF_SERIAL_DEBUG(printf("FEATURE=%i\r\n",read_register(FEATURE)));

  //
  // Enable dynamic payload on pipes 0 & 1
  //
  write_register1(DYNPD,read_register1(DYNPD) | _BV(DPL_P1) | _BV(DPL_P0));
}

/****************************************************************************/
void writeAckPayload(uint8_t pipe, const void* buf, uint8_t len)
{
  const uint8_t* current = (const uint8_t*)(buf);
  uint8_t max_payload_size = 32;
  uint8_t data_len;

  csn(LOW);
  spi_write(W_ACK_PAYLOAD | (pipe & 0x07));
  data_len = min(len,max_payload_size);
  while ( data_len-- )
    spi_write(*current++);
  csn(HIGH);
}

/****************************************************************************/
bool isAckPayloadAvailable(void)
{
  bool result = ack_payload_available;
  ack_payload_available = false;
  return result;
}

/****************************************************************************/
bool isPVariant(void)
{
  return p_variant ;
}

/****************************************************************************/
void setAutoAck1(bool enable)
{
  if ( enable )
    write_register1(EN_AA, 0x3f); // B111111
  else
    write_register1(EN_AA, 0x00);
}

/****************************************************************************/
void setAutoAck( uint8_t pipe, bool enable )
{
  if (pipe <= 6 )
  {
    uint8_t en_aa = read_register1(EN_AA) ;
    if (enable)
    {
      en_aa |= _BV(pipe) ;
    }
    else
    {
      en_aa &= ~_BV(pipe) ;
    }
    write_register1( EN_AA, en_aa ) ;
  }
}

/****************************************************************************/
bool testCarrier(void)
{
  return (read_register1(CD) & 1);
}

/****************************************************************************/
bool testRPD(void)
{
  return (read_register1(RPD) & 1);
}

/****************************************************************************/
void setPALevel(rf24_pa_dbm_e level)
{
  uint8_t setup = read_register1(RF_SETUP) ;
  setup &= ~(_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) ;

  // switch uses RAM (evil!)
  if ( level == RF24_PA_MAX )
  {
    setup |= (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) ;
  }
  else if ( level == RF24_PA_HIGH )
  {
    setup |= _BV(RF_PWR_HIGH) ;
  }
  else if ( level == RF24_PA_LOW )
  {
    setup |= _BV(RF_PWR_LOW);
  }
  else if ( level == RF24_PA_MIN )
  {
    // nothing
  }
  else if ( level == RF24_PA_ERROR )
  {
    // On error, go to maximum PA
    setup |= (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) ;
  }

  write_register1( RF_SETUP, setup ) ;
}

/****************************************************************************/
rf24_pa_dbm_e getPALevel(void)
{
  rf24_pa_dbm_e result = RF24_PA_ERROR ;
  uint8_t power = read_register1(RF_SETUP) & (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) ;

  // switch uses RAM (evil!)
  if ( power == (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) )
  {
    result = RF24_PA_MAX ;
  }
  else if ( power == _BV(RF_PWR_HIGH) )
  {
    result = RF24_PA_HIGH ;
  }
  else if ( power == _BV(RF_PWR_LOW) )
  {
    result = RF24_PA_LOW ;
  }
  else
  {
    result = RF24_PA_MIN ;
  }

  return result ;
}

/****************************************************************************/
bool setDataRate(rf24_datarate_e speed)
{
  bool result = false;
  uint8_t reg;
  uint8_t setup = read_register1(RF_SETUP) ;

  // HIGH and LOW '00' is 1 Mbps - our default
  wide_band = false ;
  setup &= ~(_BV(RF_DR_LOW) | _BV(RF_DR_HIGH)) ;
  if( speed == RF24_250KBPS )
  {
    // Must set the RF_DR_LOW to 1; RF_DR_HIGH (used to be RF_DR) is already 0
    // Making it '10'.
    wide_band = false ;
    setup |= _BV( RF_DR_LOW ) ;
  }
  else
  {
    // Set 2Mbs, RF_DR (RF_DR_HIGH) is set 1
    // Making it '01'
    if ( speed == RF24_2MBPS )
    {
      wide_band = true ;
      setup |= _BV(RF_DR_HIGH);
    }
    else
    {
      // 1Mbs
      wide_band = false ;
    }
  }
  write_register1(RF_SETUP,setup);

  // Verify our result
  reg = read_register1(RF_SETUP);
  if (reg == setup)
  {
    result = true;
  }
  else
  {
    wide_band = false;
  }
  return result;
}

/****************************************************************************/
rf24_datarate_e getDataRate( void )
{
  rf24_datarate_e result ;
  uint8_t dr = read_register1(RF_SETUP) & (_BV(RF_DR_LOW) | _BV(RF_DR_HIGH));
  
  // switch uses RAM (evil!)
  // Order matters in our case below
  if ( dr == _BV(RF_DR_LOW) )
  {
    // '10' = 250KBPS
    result = RF24_250KBPS ;
  }
  else if ( dr == _BV(RF_DR_HIGH) )
  {
    // '01' = 2MBPS
    result = RF24_2MBPS ;
  }
  else
  {
    // '00' = 1MBPS
    result = RF24_1MBPS ;
  }
  return result ;
}

/****************************************************************************/
void setCRCLength(rf24_crclength_e length)
{
  uint8_t config = read_register1(CONFIG) & ~( _BV(CRCO) | _BV(EN_CRC)) ;
  
  // switch uses RAM (evil!)
  if ( length == RF24_CRC_DISABLED )
  {
    // Do nothing, we turned it off above. 
  }
  else if ( length == RF24_CRC_8 )
  {
    config |= _BV(EN_CRC);
  }
  else
  {
    config |= _BV(EN_CRC);
    config |= _BV( CRCO );
  }
  write_register1( CONFIG, config ) ;
}

/****************************************************************************/
rf24_crclength_e getCRCLength(void)
{
  rf24_crclength_e result = RF24_CRC_DISABLED;
  uint8_t config = read_register1(CONFIG) & ( _BV(CRCO) | _BV(EN_CRC)) ;

  if ( config & _BV(EN_CRC ) )
  {
    if ( config & _BV(CRCO) )
      result = RF24_CRC_16;
    else
      result = RF24_CRC_8;
  }

  return result;
}

/****************************************************************************/
void disableCRC( void )
{
  uint8_t disable = read_register1(CONFIG) & ~_BV(EN_CRC) ;
  write_register1( CONFIG, disable ) ;
}

/****************************************************************************/
void setRetries(uint8_t delay, uint8_t count)
{
 write_register1(SETUP_RETR,(delay&0xf)<<ARD | (count&0xf)<<ARC);
}



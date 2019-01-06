/*==================================================================
  File Name    : i2c.c
  Author       : Emile
  ------------------------------------------------------------------
  Purpose : This files contains the I2C related functions 
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
#include "i2c.h"
#include "stc1000p.h"
#include "delay.h"

void i2c_delay(void)
{   
    uint8_t i;
    
    for (i = 0; i < 100; i++);
	//delay_usec(10);
} // i2c_delay()

//  I2C interrupts all share the same handler.
@interrupt void I2C_IRQHandler(void)
{
} // I2C_IRQHandler()

/*-----------------------------------------------------------------------------
  Purpose  : This function initializes the I2C bus controller
  Variables: bb = true: use bit-banging instead of I2C device
  Returns  : --
  ---------------------------------------------------------------------------*/
void i2c_init(void)
{
	I2C_CR1    = 0;     // Disable I2C before configuration starts
    I2C_FREQR  = 16;    // Set the internal clock frequency to 16 MHz
    I2C_CCRH   = 0x00;  // I2C running is standard mode.
    I2C_CCRL   = 0x50;  // SCL clock speed is 100 kHz.
    I2C_OARH   = 0x40;  // 7 bit address mode.
	I2C_OARL   = 0x00;  // Clear the address registers
    I2C_TRISER = 17;    // Set SCL rise-time to 1000 nsec.
    I2C_ITR    = 0x00;  // Disable I2C interrupts
    I2C_CR1    = 1;     // Configuration complete so turn the peripheral on
} // i2c_init()

/*-----------------------------------------------------------------------------
  Purpose  : This function checks the ACK bit after an address has been sent.
             If a device is not present, a NACK (1) is returned.
  Variables: --
  Returns  : --
  ---------------------------------------------------------------------------*/
uint8_t recv_ack_bit(void)
{
	uint8_t reg, ack, nack;
    uint8_t try = 0;
    
  do {
	ack  = (I2C_SR1 & I2C_SR1_ADDR);
	nack = (I2C_SR2 & I2C_SR2_AF);
	i2c_delay();
  } while ((++try < 10) && (!ack && !nack));
  if (ack)
  {   // ACK bit received
      reg = I2C_SR1; // Clear ADDR bit by reading I2C_SR1 and I2C_SR3
      reg = I2C_SR3;
	  I2C_CR2 |= I2C_CR2_ACK; // Set ACK-bit for further I2C communications
	  return I2C_ACK;
  }
  else
  { // Acknowledge Failure or time-out
	I2C_SR2 &= ~I2C_SR2_AF; // clear error bit
	return I2C_NACK;
  } // else
} // recv_ack_bit()

/*-----------------------------------------------------------------------------
  Purpose  : This function issues a start condition and sends address and 
             transfer direction.
  Variables: --
  Returns  : I2C_ACK, I2C_NACK
  ---------------------------------------------------------------------------*/
uint8_t i2c_start(uint8_t address)
{
  uint8_t reg;
	
  I2C_CR2 |= I2C_CR2_START;    // Generate Start condition
  while (!(I2C_SR1 & I2C_SR1_SB)) i2c_delay(); // Wait until Start is sent
  I2C_DR   = address; // Send the slave address and the R/W bit
  return recv_ack_bit();
} // i2c_start()

/*-----------------------------------------------------------------------------
  Purpose  : This function issues a repeated-start condition and sends 
             address and transfer direction.
  Variables: --
  Returns  : I2C_ACK, I2C_NACK
  ---------------------------------------------------------------------------*/
uint8_t i2c_rep_start(uint8_t address)
{
    uint8_t reg;
    
    I2C_CR2 |= I2C_CR2_START; // Generate Repeated Start condition
    while (!(I2C_SR1 & I2C_SR1_SB)) i2c_delay(); // Wait until Start is sent
    I2C_DR = address;         // Send the slave address and the R/W bit
    while (!(I2C_SR1 & I2C_SR1_ADDR)) i2c_delay();
    reg = I2C_SR3; // clear ADDR bit
} // i2c_rep_start()

/*-----------------------------------------------------------------------------
  Purpose  : This function issues a stop condition
  Variables: --
  Returns  : --
  ---------------------------------------------------------------------------*/
void i2c_stop(void)
{
    disable_interrupts();       // Errata workaround (Disable interrupt)
    I2C_CR2 |= I2C_CR2_STOP;    // generate stop here (STOP=1)
    enable_interrupts();		// Errata workaround (Enable interrupt)
    while((I2C_SR3 & I2C_SR3_MSL)) i2c_delay(); // wait until stop is performed
} // i2c_stop()

/*-----------------------------------------------------------------------------
  Purpose  : This function sends one byte to I2C device
  Variables: data: byte to be transferred
  Returns  : I2C_ACK : write successful
             I2C_NACK: write failed
  ---------------------------------------------------------------------------*/
void i2c_write(uint8_t data)
{
	I2C_DR = data; // send byte over I2C bus
	while (!(I2C_SR1 & I2C_SR1_TXE)) i2c_delay(); // wait until Data Register is empty
} // i2c_write()

/*-----------------------------------------------------------------------------
  Purpose  : This function reads one byte from the I2C device.
             This is the Lujji version of i2c_read().
  Variables: -
  Returns  : byte read
  ---------------------------------------------------------------------------*/
//uint8_t i2c_read1(void)
//{
//    I2C_CR2 &= ~I2C_CR2_ACK;
//    i2c_stop();
//    while (!(I2C_SR1 & I2C_SR1_RXNE));
//    return I2C_DR;
//} // i2c_read1()

/*-----------------------------------------------------------------------------
  Purpose  : This function reads one byte from the I2C device.
             This is the Lujji version of i2c_read().
  Variables: -
  Returns  : byte read
  ---------------------------------------------------------------------------*/
uint8_t i2c_read1(uint8_t ack)
{
    if (ack == I2C_NACK)
    {   // last byte
        I2C_CR2 &= ~I2C_CR2_ACK;
        i2c_stop();
    }
    else
    {   // more bytes to read
        I2C_CR2 |= I2C_CR2_ACK; 
    } // else
    while (!(I2C_SR1 & I2C_SR1_RXNE)) ;
    return I2C_DR;
} // i2c_read1()

/*-----------------------------------------------------------------------------
  Purpose  : This function reads two or more bytes from the I2C device
             This is the Lujji version of i2c_readN()
  Variables: *buf : pointer to array to store data in
	     len  : #bytes to read from the I2C device
  Returns  : byte read from the I2C device
  ---------------------------------------------------------------------------*/
void i2c_read_arr(uint8_t *buf, int len)
{
//    while (len-- > 1) 
//    {
//        I2C_CR2 |= I2C_CR2_ACK;
//        while (!(I2C_SR1 & I2C_SR1_RXNE));
//        *(buf++) = I2C_DR;
//    } // while
//    *buf = i2c_read1(I2C_NACK);
    while (len-- > 1) 
    {
        *(buf++) = i2c_read1(I2C_ACK);
    } // while
    *buf = i2c_read1(I2C_NACK);
} // i2c_read_arr()

//--------------------------------------------------------------------------
// Perform a device reset on the DS2482
//
// Device Reset
//   S AD,0 [A] DRST [A] Sr AD,1 [A] [SS] A\ P
//  [] indicates from slave
//  SS status byte to read to verify state
//
// Input: addr: the I2C address of the DS2482 to reset
// Returns: TRUE if device was reset
//          FALSE device not detected or failure to perform reset
//--------------------------------------------------------------------------
int8_t ds2482_reset(uint8_t addr)
{
   uint8_t err, ret;

	// generate I2C start + output address to I2C bus
	err = (i2c_start(addr | I2C_WRITE) == I2C_NACK);
	if (!err)
	{
		i2c_write(CMD_DRST); // write register address
		i2c_rep_start(addr | I2C_READ);
		ret = i2c_read1(I2C_NACK); // Read byte, generate I2C stop condition
   } // if
   // check for failure due to incorrect read back of status
   if (!err && ((ret & 0xF7) == 0x10))
   		  return true;
   else return false;	
} // ds2482_reset()
  
//--------------------------------------------------------------------------
// Write the configuration register in the DS2482. The configuration 
// options are provided in the lower nibble of the provided config byte. 
// The uppper nibble in bitwise inverted when written to the DS2482.
//  
// Write configuration (Case A)
//   S AD,0 [A] WCFG [A] CF [A] Sr AD,1 [A] [CF] A\ P
//  [] indicates from slave
//  CF configuration byte to write
//
// Input: addr: the I2C address of the DS2482 to reset
// Returns:  TRUE: config written and response correct
//           FALSE: response incorrect
//--------------------------------------------------------------------------
int8_t ds2482_write_config(uint8_t addr)
{
   uint8_t err, read_config;

	// generate I2C start + output address to I2C bus
	err = (i2c_start(addr | I2C_WRITE) == I2C_NACK);
	if (!err)
	{
		i2c_write(CMD_WCFG); // write register address
		i2c_write(DS2482_CONFIG); // write register address
        i2c_rep_start(addr | I2C_READ);
		read_config = i2c_read1(I2C_NACK); // Read byte, generate I2C stop condition
   } // if
   // check for failure due to incorrect read back
   if (err || (read_config != (DS2482_CONFIG & 0x0f)))
   {
      ds2482_reset(addr); // handle error
      return false;
   } // if
   return true;
} // ds2482_write_config()

//--------------------------------------------------------------------------
// DS2428 Detect routine that performs a device reset followed by writing 
// the default configuration settings (active pullup enabled)
//
// Input: addr: the I2C address of the DS2482 to reset
// Returns: TRUE if device was detected and written
//          FALSE device not detected or failure to write configuration byte
//--------------------------------------------------------------------------
int8_t ds2482_detect(uint8_t addr)
{
   if (!ds2482_reset(addr)) // reset the DS2482
      return false;

   if (!ds2482_write_config(addr)) // write default configuration settings
        return false;
   else return true;
} // ds2482_detect()

//--------------------------------------------------------------------------
// Use the DS2482 help command '1-Wire triplet' to perform one bit of a 1-Wire
// search. This command does two read bits and one write bit. The write bit
// is either the default direction (all device have same bit) or in case of 
// a discrepancy, the 'search_direction' parameter is used. 
//
// Returns: The DS2482 status byte result from the triplet command
//--------------------------------------------------------------------------
uint8_t ds2482_search_triplet(uint8_t search_direction, uint8_t addr)
{
   uint8_t err, status;
   int poll_count = 0;

   // 1-Wire Triplet (Case B)
   //   S AD,0 [A] 1WT [A] SS [A] Sr AD,1 [A] [Status] A [Status] A\ P
   //                                         \--------/        
   //                           Repeat until 1WB bit has changed to 0
   //  [] indicates from slave
   //  SS indicates byte containing search direction bit value in msbit
   // generate I2C start + output address to I2C bus
   err = (i2c_start(addr | I2C_WRITE) == I2C_NACK);
   if (!err)
   {
	   i2c_write(CMD_1WT); // write register address
   	   i2c_write(search_direction ? 0x80 : 0x00);
	   i2c_rep_start(addr | I2C_READ);
   	   // loop checking 1WB bit for completion of 1-Wire operation 
   	   // abort if poll limit reached
	   do
       {
	       I2C_CR2 |= I2C_CR2_ACK;
           while (!(I2C_SR1 & I2C_SR1_RXNE));
           status = I2C_DR;
	   } while ((status & STATUS_1WB) && (poll_count++ < DS2482_OW_POLL_LIMIT));	
       i2c_read1(I2C_NACK); // Read byte, generate I2C stop condition	   
	   //status = i2c_read(I2C_ACK); // Read byte
	   //do
	   //{
	   //  if (status & STATUS_1WB) status = i2c_read(I2C_ACK);
	   //}
	   //while ((status & STATUS_1WB) && (poll_count++ < DS2482_OW_POLL_LIMIT));
	   //i2c_read(I2C_NACK); // Read 1 byte and generate stop condition
   	   // check for failure due to poll limit reached
   	   if (poll_count >= DS2482_OW_POLL_LIMIT)
   	   {
      	  ds2482_reset(addr); // handle error
      	  return false;
   	   } // if
   	   return status;
   } // if
   else return false;
} // ds2482_search_triplet()
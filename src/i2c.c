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

#define SDA_in    (PE_DDR &= ~I2C_SDA)
#define SDA_out   (PE_DDR |=  I2C_SDA)
#define SDA1      (PE_ODR |=  I2C_SDA)
#define SDA0      (PE_ODR &= ~I2C_SDA)
#define SCL1      {PE_ODR |=  I2C_SCL; delay_usec(5);}
#define SCL0      {PE_ODR &= ~I2C_SCL; delay_usec(5);}

void i2c_delay(void)
{   
    uint8_t i;
    
    for (i = 0; i < 100; i++);
	//delay_usec(10);
} // i2c_delay()

uint8_t i2c_write_bb(uint8_t data)
{
    uint8_t i   = 0x80;
    uint8_t ack = I2C_ACK;
    
    SCL0; // clock low
    while (i > 0)
    {   // write bits to I2C bus
        if (data & i) SDA1;
        else          SDA0;
        SCL1;
        SCL0;
        i >>= 1;
    } // while
    SDA_in; // set as input
    delay_usec(5);
    SCL1;
    if (PE_IDR & I2C_SDA)
         ack = I2C_NACK; // nack (1) 
    SCL0;
    SDA_out; // set to output again
    SDA0;
    return ack;
} // i2c_write_bb()

uint8_t i2c_read_bb(uint8_t ack)
{
    uint8_t result = 0;
    uint8_t i      = 0x80;
    
    SCL0; // clock low
    SDA_in; // set as input
    while (i > 0)
    {   // read bits from I2C bus
        result <<= 1; // make room for new bit
        SCL1;
        if (PE_IDR & I2C_SDA)
             result |=  0x01;
        SCL0;
        i >>= 1;
    } // while
    SDA_out; // set to output again
    if (ack == I2C_ACK) 
         SDA0; // output ACK
    else SDA1; // output NACK
    SCL1;
    SCL0;
    SDA0;
    return result;
} // i2c_read_bb()

uint8_t i2c_start_bb(uint8_t address)
{   // Pre-condition : SDA = 1
    // Post-condition: SCL = 0, SDA = 0
    SCL1; 
    SDA0;
    delay_usec(5);
    SCL0;
    return i2c_write_bb(address);
} // i2c_start_bb;

uint8_t i2c_rep_start_bb(uint8_t address)
{   
    SDA1;
    delay_usec(5);
    return i2c_start_bb(address);
} // i2c_start_bb;

void i2c_stop_bb(void)
{   // Pre-condition : SDA = 0
    SCL1;
    SDA1;
    delay_usec(5);
} // i2c_stop_bb;

//  I2C interrupts all share the same handler.
@interrupt void I2C_IRQHandler(void)
{
} // I2C_IRQHandler()

/*-----------------------------------------------------------------------------
  Purpose  : This function initializes the I2C bus controller
  Variables: bb = true: use bit-banging instead of I2C device
  Returns  : --
  ---------------------------------------------------------------------------*/
void i2c_init(bool bb)
{
	I2C_CR1    = 0;     // Disable I2C before configuration starts
    I2C_FREQR  = 16;    // Set the internal clock frequency to 16 MHz
    I2C_CCRH   = 0x00;  // I2C running is standard mode.
    I2C_CCRL   = 0x50;  // SCL clock speed is 100 kHz.
    I2C_OARH   = 0x40;  // 7 bit address mode.
    I2C_TRISER = 17;    // Set SCL rise-time to 1000 nsec.
    I2C_ITR    = 0x00;  // Disable I2C interrupts
    if (bb)
    {
        SDA_out;
        SDA1; 
        SCL1;
    }
    else I2C_CR1 = 1;     // Configuration complete so turn the peripheral on
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

  do {
	ack  = (I2C_SR1 & I2C_SR1_ADDR);
	nack = (I2C_SR2 & I2C_SR2_AF);
	i2c_delay();
  } while (!ack && !nack);
  if (ack)
  {   // ACK bit received
      reg = I2C_SR1; // Clear ADDR bit by reading I2C_SR1 and I2C_SR3
      reg = I2C_SR3;
	  return I2C_ACK;
  }
  else
  { // Acknowledge Failure
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
	
  while((I2C_SR3 & I2C_SR3_BUSY)) i2c_stop();

  I2C_CR2 |= (I2C_CR2_ACK | I2C_CR2_START);    // Generate Start condition
  while (!(I2C_SR1 & I2C_SR1_SB)) i2c_delay(); // Wait until Start is sent
  reg      = I2C_SR1; // Reading SR1 clears the Start condition
  I2C_DR   = address; // Send the slave address and the R/W bit
  I2C_OARL = 0;       // Clear the address registers
  I2C_OARH = 0x40;
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
    i2c_delay();
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
	I2C_CR2 &= ~I2C_CR2_ACK;
    disable_interrupts();       // Errata workaround (Disable interrupt)
    I2C_CR2 |= I2C_CR2_STOP;    // generate stop here (STOP=1)
    enable_interrupts();		// Errata workaround (Enable interrupt)
    while((I2C_CR2 & I2C_CR2_STOP)) i2c_delay(); // wait until stop is performed
} // i2c_stop()

/*-----------------------------------------------------------------------------
  Purpose  : This function sends one byte to I2C device
  Variables: data: byte to be transferred
  Returns  : I2C_ACK : write successful
             I2C_NACK: write failed
  ---------------------------------------------------------------------------*/
void i2c_write(uint8_t data)
{
	while (!(I2C_SR1 & I2C_SR1_TXE)) i2c_delay(); // wait until Data Register is empty
	I2C_DR = data; // send byte over I2C bus
	while (!(I2C_SR1 & I2C_SR1_TXE)) i2c_delay(); // wait until Data Register is empty
	//while ((I2C_SR1 & (I2C_SR1_TXE | I2C_SR1_BTF)) != (I2C_SR1_TXE | I2C_SR1_BTF)) i2c_delay();
} // i2c_write()

/*-----------------------------------------------------------------------------
  Purpose  : This function reads one byte from the I2C device
  Variables: data: byte to be transferred
  Returns  : byte read
  ---------------------------------------------------------------------------*/
uint8_t i2c_read(uint8_t ack)
{
    uint8_t buf[3];
    
    if (ack == I2C_ACK)
    {
        while (!(I2C_SR1 & I2C_SR1_BTF)) i2c_delay(); // Wait for BTF
	    return I2C_DR;                 	              // Reading next data byte
    } // if
    else
    {
        i2c_readN(3,buf); // read 3 bytes, return 1st byte
        return buf[0];
    } // else
} // i2c_read()

/*-----------------------------------------------------------------------------
  Purpose  : This function reads one or more bytes from the I2C device
  Variables: num_bytes: #bytes to read from the I2C device
             *buf     : 
  Returns  : byte read from the I2C device
  ---------------------------------------------------------------------------*/
void i2c_readN(uint8_t num_bytes, uint8_t *buf)
{
    uint8_t reg;
    
    if (num_bytes > 2)
    {
        while (num_bytes > 3) // not last three bytes?
        {
            i2c_delay();
            while (!(I2C_SR1 & I2C_SR1_BTF)) i2c_delay(); // Wait for BTF
			*buf++ = I2C_DR;                 			  // Reading next data byte
            num_bytes--;								  // Decrease bytes to read
        } // while
        // 3 bytes remaining to read
        while (!(I2C_SR1 & I2C_SR1_BTF)) i2c_delay();  // Wait for BTF
        I2C_CR2 &= ~I2C_CR2_ACK;                       // Clear ACK (= send NACK)
        disable_interrupts();                          // Errata workaround (Disable interrupt)
        *buf++   = I2C_DR;                     		   // Read 1st byte
        I2C_CR2 |= I2C_CR2_STOP;                       // Generate stop here (STOP=1)
        *buf++   = I2C_DR;                     	       // Read 2nd byte
        enable_interrupts();						   // Errata workaround (Enable interrupt)
        while (!(I2C_SR1 & I2C_SR1_RXNE)) i2c_delay(); // Wait for RXNE
        *buf++   = I2C_DR;                   		   // Read 3rd Data byte
    } // if
    else
    {   // 1 or 2 bytes to read
        if (num_bytes == 2)
        {   // 2 bytes to read
            I2C_CR2 |= I2C_CR2_POS;                       // Set POS bit (NACK at next received byte)
            disable_interrupts();                         // Errata workaround (Disable interrupt)
            I2C_CR2 &= ~I2C_CR2_ACK;                      // Clear ACK 
            enable_interrupts();						  // Errata workaround (Enable interrupt)
            while (!(I2C_SR1 & I2C_SR1_BTF)) i2c_delay(); // Wait for BTF
            disable_interrupts();                         // Errata workaround (Disable interrupt)
            I2C_CR2 |= I2C_CR2_STOP;                      // Generate stop here (STOP=1)
            *buf++   = I2C_DR;                     	      // Read 1st Data byte
            enable_interrupts();						  // Errata workaround (Enable interrupt)
			*buf     = I2C_DR;							  // Read 2nd Data byte
        } // if
        else
        {   // only 1 byte to read
            I2C_CR2 &= ~I2C_CR2_ACK;         		       // Clear ACK 
            disable_interrupts();           		       // Errata workaround (Disable interrupt)
            I2C_CR2 |= I2C_CR2_STOP;            	       // generate stop here (STOP=1)
            enable_interrupts();					       // Errata workaround (Enable interrupt)
            while (!(I2C_SR1 & I2C_SR1_RXNE)) i2c_delay(); // test EV7, wait for RxNE
            *buf = I2C_DR;                             // Read Data byte
        } // else
    } // else
    while ((I2C_CR2 & I2C_CR2_STOP)) i2c_delay();  // Wait until stop is performed (STOPF = 1)
    I2C_CR2 &= ~I2C_CR2_POS;                       // return POS to default state (POS=0)
} // i2c_readN()

int16_t lm92_read(uint8_t addr, uint8_t *err)
/*------------------------------------------------------------------
  Purpose  : This function reads the LM92 13-bit Temp. Sensor and
             returns the temperature.
             Reading Register 0 of the LM92 results in the following bits:
              15   14  13 12      3   2    1   0
             Sign MSB B10 B9 ... B0 Crit High Low
  Variables:
      addr : I2C-address of LM92
  Returns  : The temperature from the LM92 in a signed Q8.7 format.
             Q8.7 is chosen here for accuracy reasons when filtering.
  ------------------------------------------------------------------*/
{
   uint8_t  buffer[2]; // array to store data from i2c_read()
   uint16_t temp_int;  // the temp. from the LM92 as an integer
   uint8_t  sign;      // sign of temperature
   int16_t  temp = 0;  // the temp. from the LM92 as an int
       
   // Start with selecting the proper channel on the PCA9544
   *err = (i2c_start(addr) == I2C_NACK); // generate I2C start + output address to I2C bus
   // adr contains address of LM92 found or *err is true (no LM92 present)     
   if (*err) i2c_stop();
   else
   {
      i2c_readN(2,buffer);
      temp_int = buffer[0] & 0x00FF;    // store {Sign, MSB, bit 10..5} at bits temp_int bits 7..0
      temp_int <<= 8;                   // SHL 8, Sign now at bit 15
      temp_int &= 0xFF00;               // Clear bits 7..0
      temp_int |= (buffer[1] & 0x00FF); // Add bits D4..D0 to temp_int bits 7..3
      temp_int &= 0xFFF8;               // Clear Crit High & Low bits
      sign = ((temp_int & LM92_SIGNb) == LM92_SIGNb);
      if (sign)
      {
         temp_int &= ~LM92_SIGNb;        // Clear sign bit
         temp_int  = LM92_FS - temp_int; // Convert two complement number
      } // if
      //temp = temp_int >> 3;
	  temp = temp_int; // without shifting! Returns Q8.7 format
      if (sign)
      {
         temp = -temp; // negate number
      } // if
	  //i2c_stop();
   } // else
   return temp;     // Return value now in °C << 7
} // lm92_read()

uint8_t mcp23008_init(void)
/*------------------------------------------------------------------
  Purpose  : This function inits the MCP23008 8-bit IO-expander
  Variables: -
  Returns  : 0: no error, 1: error
  ------------------------------------------------------------------*/
{
	uint8_t err;
	err = mcp230xx_write(IOCON, MCP23008_INIT);
	if (!err)
	{
		err = mcp230xx_write(IODIR, 0x00); // all PORT bits are output
		err = mcp230xx_write(GPPU,  0xFF); // Enable pull-ups (100k) on PORT
		err = mcp230xx_write(OLAT,  0x80); // HW-bug? Have to write this first
		err = mcp230xx_write(OLAT,  0x00); // All valves are OFF at power-up
	} // if
	return err;	
} // mcp23008_init()

uint8_t mcp230xx_read(uint8_t reg)
/*------------------------------------------------------------------
  Purpose  : This function reads one register from the MCP23008 or
             the MCP23017 IO-expander.
  Variables:
       reg : The register to read from
  Returns  : the value returned from the register
  ------------------------------------------------------------------*/
{
	uint8_t err, ret = 0;
	
	// generate I2C start + output address to I2C bus
	err = (i2c_start(MCP23008_ADDR | I2C_WRITE) == I2C_NACK); 
	if (err) i2c_stop();
    else 
	{
		i2c_write(reg); // write register address
		i2c_rep_start(MCP23008_ADDR | I2C_READ);
		i2c_readN(1,&ret); // Read byte, generate I2C stop condition
	} // if
	return ret;
} // mcp230xx_read()

uint8_t mcp230xx_write(uint8_t reg, uint8_t data)
/*------------------------------------------------------------------
  Purpose  : This function write one data byte to a specific register 
			 of the MCP23008 or MCP23017 IO-expander.
  Variables:
       reg : The register to write to
	   data: The data byte to write into the register
  Returns  : 0: no error, 1: error
  ------------------------------------------------------------------*/
{
	uint8_t err;
		
	// generate I2C start + output address to I2C bus
	err = (i2c_start(MCP23008_ADDR | I2C_WRITE) == I2C_NACK);
	if (err) i2c_stop();
    else
	{
		i2c_write(reg);  // write register address
		i2c_write(data); // write register value
		i2c_stop();
	} // if
	return err;
} // mcp230xx_write()

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
		ret = i2c_read(I2C_NACK);
        //i2c_readN(1, &ret); // Read byte, generate I2C stop condition
		//i2c_stop();
   } // if
   // check for failure due to incorrect read back of status
   if (!err && ((ret & 0xF7) == 0x10))
   		  return true;
   else return false;	
} // ds2482_reset()
  
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
int8_t ds2482_reset_bb(uint8_t addr)
{
   uint8_t err, ret;

	// generate I2C start + output address to I2C bus
	err = (i2c_start_bb(addr | I2C_WRITE) == I2C_NACK);
	if (!err)
	{
		err  = i2c_write_bb(CMD_DRST); // write register address
		err |= i2c_rep_start_bb(addr | I2C_READ);
		ret  = i2c_read_bb(I2C_NACK);
        i2c_stop_bb();
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
int8_t ds2482_write_config_bb(uint8_t addr)
{
   uint8_t err, read_config;

	// generate I2C start + output address to I2C bus
	err = (i2c_start_bb(addr | I2C_WRITE) == I2C_NACK);
	if (!err)
	{
		err = i2c_write_bb(CMD_WCFG); // write register address
		err = i2c_write_bb(DS2482_CONFIG); // write register address
        err = i2c_rep_start_bb(addr | I2C_READ);
		read_config = i2c_read_bb(I2C_NACK);
        i2c_stop_bb();
   } // if
   // check for failure due to incorrect read back
   if (err || (read_config != (DS2482_CONFIG & 0x0f)))
   {
      ds2482_reset_bb(addr); // handle error
      return false;
   } // if
   return true;
} // ds2482_write_config_bb()

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
		read_config = i2c_read(I2C_NACK);
        //i2c_readN(1, &read_config); // Read byte, generate I2C stop condition
		//i2c_stop();
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
// DS2428 Detect routine that performs a device reset followed by writing 
// the default configuration settings (active pullup enabled)
//
// Input: addr: the I2C address of the DS2482 to reset
// Returns: TRUE if device was detected and written
//          FALSE device not detected or failure to write configuration byte
//--------------------------------------------------------------------------
int8_t ds2482_detect_bb(uint8_t addr)
{
   if (!ds2482_reset_bb(addr)) // reset the DS2482
      return false;

   if (!ds2482_write_config_bb(addr)) // write default configuration settings
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
	   status = i2c_read(I2C_ACK); // Read byte
	   do
	   {
	     if (status & STATUS_1WB) status = i2c_read(I2C_ACK);
	   }
	   while ((status & STATUS_1WB) && (poll_count++ < DS2482_OW_POLL_LIMIT));
	   i2c_read(I2C_NACK); // Read 1 byte and generate stop condition
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
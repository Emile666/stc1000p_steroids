/*==================================================================
  File Name    : comms.c
  Author       : Emile
  ------------------------------------------------------------------
  Purpose : This files contains functions for the uart commands.
  ------------------------------------------------------------------
  COMMS is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
 
  COMMS is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with COMMS. If not, see <http://www.gnu.org/licenses/>.
  ==================================================================
*/ 
#include <ctype.h>
#include <stdlib.h>
#include <stdio.h>
#include "uart.h"
#include "comms.h"
#include "i2c.h"
#include "scheduler.h"
#include "eep.h"
#include "stc1000p_lib.h"

extern int16_t temp1_ow_10;   // TMLT Temperature from DS18B20 in °C * 10
extern uint8_t temp1_ow_err;  // 1 = Read error from DS18B20
extern int16_t setpoint;      // local copy of SP variable
extern uint8_t use_one_wire;  // 1 = Use one-wire sensor instead of NTC probe 1

uint8_t rs232_inbuf[UART_BUFLEN]; // buffer for RS232 commands
uint8_t rs232_ptr = 0;            // index in RS232 buffer

/*-----------------------------------------------------------------------------
  Purpose  : This function prints the version number info to both the UART and
             the LCD I2C display.
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
void print_version_number(void)
{
    const char s[] = "stc1000p-stm8s105c6 V0.1\r\n";
    xputs(s);               // print to uart
} // print_version_number()

/*-----------------------------------------------------------------------------
  Purpose  : Scan all devices on the I2C bus
  Variables: -
 Returns  : -
  ---------------------------------------------------------------------------*/
void i2c_scan(void)
{
	char    s[50]; // needed for printing to serial terminal
	uint8_t x = 0;
	int     i;     // Leave this as an int!
	
	xputs("I2C: ");
    for (i = 0x00; i < 0xff; i+=2)
	{
		if (i2c_start(i) == I2C_ACK)
		{
			sprintf(s,"0x%0x ",i);
		    xputs(s);
			x++;
		} // if
		i2c_stop();
	} // for
	if (!x) xputs("-");
	xputs("\r\n");
} // i2c_scan()

/*-----------------------------------------------------------------------------
  Purpose  : Non-blocking RS232 command-handler via the UART
  Variables: -
  Returns  : [NO_ERR, ERR_CMD, ERR_NUM, ERR_I2C]
  ---------------------------------------------------------------------------*/
uint8_t rs232_command_handler(void)
{
  uint8_t ch;
  static bool cmd_rcvd = 0;
  
  if (!cmd_rcvd && uart_kbhit())
  { // A new character has been received
    ch = tolower(uart_read()); // get character as lowercase
    uart_write(ch);
	switch (ch)
	{
		case '\r': break;
		case '\n': cmd_rcvd  = true;
		           rs232_inbuf[rs232_ptr] = '\0';
		           rs232_ptr = false;
				   break;
		default  : rs232_inbuf[rs232_ptr++] = ch;
				   break;
	} // switch
  } // if
  if (cmd_rcvd)
  {
	  cmd_rcvd = false;
	  return execute_single_command(rs232_inbuf);
  } // if
  else return NO_ERR;
} // rs232_command_handler()

print_value10(int16_t x)
{
    char     s[20];
    uint16_t temp = divu10(x);
    
    sprintf(s,"%d.",temp);
    xputs(s);
    temp = x - 10 * temp;
    sprintf(s,"%d\r\n",temp);
    xputs(s);
} // print_value10()

/*-----------------------------------------------------------------------------
  Purpose: interpret commands which are received via the UART:
   - S0           : Chess version number
	 S1           : List all connected I2C devices  
	 S2           : List all tasks
 
  Variables: 
          s: the string that contains the command from UART
  Returns  : [NO_ERR, ERR_CMD, ERR_NUM, ERR_I2C] or ack. value for command
  ---------------------------------------------------------------------------*/
uint8_t execute_single_command(char *s)
{
   uint8_t  num  = atoi(&s[1]); // convert number in command (until space is found)
   uint8_t  rval = NO_ERR, err, i, num2;
   char     s2[30]; // Used for printing to RS232 port
   
   if (isalpha(s[1]))
   {
       num = atoi(&s[3]);
       if (!strncmp(s,"sp",2))
       {
           setpoint = num;
           eeprom_write_config(EEADR_MENU_ITEM(SP), setpoint);
           xputs("SP=");
           print_value10(setpoint);
       } // if
   } // else
   else switch (s[0])
   {
       case 'o': if (num > 1) rval = ERR_NUM;
				 else
			  	 {
					use_one_wire = num;
                    eeprom_write_config(EEADR_MENU_ITEM(One), use_one_wire);
				 } // else
				 break;

       case 's': // System commands
				 switch (num)
				 {
					 case 0: // Chess revision number
							 print_version_number();
							 break;
					 case 1: // List all I2C devices
					         i2c_scan();
							 break;
					 case 2: // List all tasks
							 list_all_tasks(); 
							 break;	
                     case 3: sprintf(s2,"ds18b20_read():%d, T=",(uint16_t)temp1_ow_err);
                             xputs(s2);
                             print_value10(temp1_ow_10);
                             break;
					 default: rval = ERR_NUM;
							  break;
				 } // switch
				 break;
	   default: rval = ERR_CMD;
				sprintf(s2,"ERR.CMD[%s]\r\n",s);
				xputs(s2);
	            break;
   } // switch
   return rval;	
} // execute_single_command()

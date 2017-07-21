/*==================================================================
  File Name    : stc1000p.c
  Author       : Emile
  ------------------------------------------------------------------
  Purpose : This files contains the main() function and all the 
            hardware related functions for the STM8S105C6T6 uC.
            It is meant for the STC1000 with newly developed HW.
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
#include "stc1000p.h"
#include "stc1000p_lib.h"
#include "scheduler.h"
#include "adc.h"
#include "eep.h"
#include "i2c.h"
#include "nrf24.h"

// Global variables
uint8_t   ad_err1 = false; // used for adc range checking
uint8_t   ad_err2 = false; // used for adc range checking
bool      probe2  = false; // cached flag indicating whether 2nd probe is active
bool      show_sa_alarm = false;
bool      ad_ch   = false; // used in adc_task()
uint16_t  ad_ntc1 = (512L << FILTER_SHIFT);
uint16_t  ad_ntc2 = (512L << FILTER_SHIFT);
int16_t   temp_ntc1;         // The temperature in E-1 �C from NTC probe 1
int16_t   temp_ntc2;         // The temperature in E-1 �C from NTC probe 2
uint8_t   mpx_nr = 0;        // Used in multiplexer() function
int16_t   pwr_on_tmr = 1000; // Needed for 7-segment display test

// Radio pipe addresses for the communication nodes
uint8_t tx_address[5] = {0xFF,0xFF,0xFF,0xFF,0xFF};
uint8_t rx_address[5] = {0xFF,0xFF,0xFF,0xFF,0xFF};

// External variables, defined in other files
extern uint8_t  led_e;                 // value of extra LEDs
extern uint8_t  led_10, led_1, led_01; // values of 10s, 1s and 0.1s
extern bool     pwr_on;           // True = power ON, False = power OFF
extern uint8_t  sensor2_selected; // DOWN button pressed < 3 sec. shows 2nd temperature / pid_output
extern bool     minutes;          // timing control: false = hours, true = minutes
extern bool     menu_is_idle;     // No menus in STD active
extern bool     fahrenheit;       // false = Celsius, true = Fahrenheit
extern uint16_t cooling_delay;    // Initial cooling delay
extern uint16_t heating_delay;    // Initial heating delay
extern int16_t  setpoint;         // local copy of SP variable
extern uint8_t  ts;               // Parameter value for sample time [sec.]
extern int16_t  pid_out;          // Output from PID controller in E-1 %
extern int16_t  @eeprom eedata[]; // Link to .eeprom section
extern uint32_t t2_millis;        // needed for delay_msec()

/*-----------------------------------------------------------------------------
  Purpose  : This routine multiplexes the 4 segments of the 7-segment displays.
             It runs at 1 kHz, so there's a full update after every 4 msec.
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
void multiplexer(void)
{
    // Disable all 7-segment LEDs and common-cathode pins
	PD_ODR    &= ~PD_SEG7; // Clear LEDs
    PE_ODR    &= ~S7_C;    // Clear LEDs
    PC_ODR    |=  CC_ALL;  // Disable common-cathodes
    
    switch (mpx_nr)
    {
        case 0: // output 10s digit
            PD_ODR |= (led_10 & PD_SEG7);     // Update 7-segments
            PE_ODR |= ((led_10 >> 1) & S7_C); // Update 7-segment C
            PC_ODR &= ~CC_10;    // Enable  common-cathode for 10s
            mpx_nr = 1;
            break;
        case 1: // output 1s digit
            PD_ODR |= (led_1 & PD_SEG7);     // Update 7-segments
            PE_ODR |= ((led_1 >> 1) & S7_C); // Update 7-segment C
            PC_ODR &= ~CC_1;     // Enable  common-cathode for 1s
            mpx_nr = 2;
            break;
        case 2: // output 01s digit
            PD_ODR |= (led_01 & PD_SEG7);     // Update 7-segments
            PE_ODR |= ((led_01 >> 1) & S7_C); // Update 7-segment C
            PC_ODR &= ~CC_01;    // Enable common-cathode for 0.1s
            mpx_nr = 3;
            break;
        case 3: // outputs special digits
            PD_ODR |= (led_e & PD_SEG7);     // Update 7-segments
            PE_ODR |= ((led_e >> 1) & S7_C); // Update 7-segment C
            PC_ODR &= ~CC_e;     // Enable common-cathode for extras
        default: // FALL-THROUGH (less code-size)
            mpx_nr = 0;
            break;
            //mpx_nr = 0;
            //break;
    } // switch            
} // multiplexer()

/*-----------------------------------------------------------------------------
  Purpose  : This is the interrupt routine for the Timer 2 Overflow handler.
             It runs at 1 kHz and drives the scheduler and the multiplexer.
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
//#pragma vector = TIM2_OVR_UIF_vector
@interrupt void TIM2_UPD_OVF_IRQHandler(void)
{
    t2_millis++;      // update millisecond counter
	scheduler_isr();  // Run scheduler interrupt function
    if (!pwr_on)
    {   // Display OFF on dispay
	    led_10     = LED_O;
	    led_1      = led_01 = LED_F;
        led_e      = LED_OFF;
        pwr_on_tmr = 1000; // 1 second
    } // if
    else if (pwr_on_tmr > 0)
    {	// 7-segment display test for 1 second
        pwr_on_tmr--;
        led_10 = led_1 = led_01 = led_e = LED_ON;
    } // else if
    multiplexer();    // Run multiplexer for Display and Keys
    TIM2_SR1 &= ~TIM2_SR1_UIF; // Reset interrupt (UIF bit) so it will not fire again straight away.
} // TIM2_UPD_OVF_IRQHandler()

/*-----------------------------------------------------------------------------
  Purpose  : This routine initialises the system clock to run at 16 MHz.
             It uses the internal HSI oscillator.
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
void initialise_system_clock(void)
{
    CLK_ICKCR  = 0;                //  Reset the Internal Clock Register.
    CLK_ICKCR |= CLK_ICKCR_HSIEN;  //  Enable the HSI.
    while ((CLK_ICKCR & CLK_ICKCR_HSIRDY) == 0); //  Wait for the HSI to be ready for use.
    CLK_CKDIVR     = 0;            //  Ensure the clocks are running at full speed.
 
    // The datasheet lists that the max. ADC clock is equal to 6 MHz (4 MHz when on 3.3V).
    // Because fMASTER is now at 16 MHz, we need to set the ADC-prescaler to 4.
    ADC_CR1     &= ~ADC_CR1_SPSEL_MSK;
    ADC_CR1     |= 0x20;          //  Set prescaler (SPSEL bits) to 4, fADC = 4 MHz
    CLK_SWIMCCR  = 0x00;          //  Set SWIM to run at clock / 2.
    CLK_SWR      = 0xE1;          //  Use HSI as the clock source.
    CLK_SWCR     = 0x00;          //  Reset the clock switch control register.
    CLK_SWCR    |= CLK_SWCR_SWEN; //  Enable switching.
    while ((CLK_SWCR & CLK_SWCR_SWBSY) != 0);  //  Pause while the clock switch is busy.
} // initialise_system_clock()

/*-----------------------------------------------------------------------------
  Purpose  : This routine initialises Timer 2 to generate a 1 kHz interrupt.
             16 MHz / (  16 *  1000) = 1000 Hz (1000 = 0x03E8)
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
void setup_timer2(void)
{
    TIM2_PSCR = 0x04;         //  Prescaler = 16
    TIM2_ARRH = 0x03;         //  High byte of 1000
    TIM2_ARRL = 0xE8;         //  Low  byte of 1000
    TIM2_IER  = TIM2_IER_UIE; //  Enable the update interrupts, disable all others
    TIM2_CR1  = TIM2_CR1_CEN; //  Finally enable the timer
} // setup_timer2()

/*-----------------------------------------------------------------------------
  Purpose  : This routine initialises all the GPIO pins of the STM8 uC.
             See stc1000p.h for a detailed description of all pin-functions.
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
void setup_gpio_ports(void)
{
	PA_DDR     |=  (S3 | COOL | HEAT | ALARM); // Set as output
	PA_DDR     &= ~PA_NC;                      // Set unused ports as input
	PA_CR1     |=  PA_NC;                      // Enable pull-up
    PA_CR1     |=  (S3 | COOL | HEAT | ALARM); // Set to Push-Pull
    PA_ODR     &= ~(S3 | COOL | HEAT | ALARM); // Disable PORTA outputs
		
    PB_DDR     |= (IO1 | IO2 | LED1 | LED2 | LED3);  // Set as output
    PB_DDR     &= ~(PB_NC | AD_CHANNELS);            // Set as input
    PB_CR1     |= PB_NC;                             // Enable pull-up
	PB_CR1     &= ~AD_CHANNELS; // Set to floating-inputs (required by ADC)
	PB_CR1     |=  (IO1 | IO2 | LED1 | LED2 | LED3); // Set to Push-Pull
    PB_ODR     &= ~(IO1 | IO2 | LED1 | LED2 | LED3); // Disable PORTB outputs
		
    PC_DDR     |=  (SPI_MOSI | SPI_CLK | CC_10 | CC_1 | CC_01 | CC_e); // Set as outputs
	PC_CR1     |=  (SPI_MOSI | SPI_CLK | CC_10 | CC_1 | CC_01 | CC_e); // Set to Push-Pull
	PC_CR2     |=  (SPI_MOSI | SPI_CLK);            // Set to 10 MHz
    PC_ODR     &= ~(SPI_MOSI | SPI_CLK);            // Disable PORTC outputs
    PC_ODR     |=  (CC_10 | CC_1 | CC_01 | CC_e);   // Disable Common-Cathodes
    PC_DDR     &= ~SPI_MISO;         // set as Input
	PC_CR1     &= ~SPI_MISO;         // set to Floating
		
    PD_DDR     |=  PD_SEG7;          // Set 7-segment/key pins as output
    PD_CR1     |=  PD_SEG7;          // Set 7-segment/key pins to Push-Pull
    PD_ODR     &= ~PD_SEG7;          // Turn off all 7-segment/key pins

    PE_ODR     |=  (I2C_SCL | I2C_SDA); // Must be set here, or I2C will not work
    PE_DDR     |=  (SPI_NSS | NRF24_CE | S7_C | I2C_SCL | I2C_SDA); // Set as outputs
	PE_DDR     &= ~PE_NC;               // Set unused ports as input
	PE_CR1     |=  PE_NC;               // Enable pull-ups
    PE_CR1     |=  (SPI_NSS | NRF24_CE | S7_C);    // Set to Push-Pull
	//PE_CR2     |=  (I2C_SCL | I2C_SDA); // Set speed to 10 MHz
    PE_ODR     &= ~S7_C;                // Turn off 7-segment C
    PE_ODR     |=  (SPI_NSS);           // Turn off SPI_NSS
    PE_ODR     &= ~NRF24_CE;            // Set NRF24_CE low
    
    PG_DDR     &= ~PG_NC;            // Set unused ports as input
	PG_CR1     |=  PG_NC;            // Enable pull-up
} // setup_output_ports()

/*-----------------------------------------------------------------------------
  Purpose  : This task is called every 500 msec. and processes the NTC 
             temperature probes from NTC1 (PB3/AIN3) and NTC2 (PB2/AIN2)
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
void adc_task(void)
{
  uint16_t temp;
  uint8_t  i;
  
  // Save registers that interferes with LED's and disable common-cathodes
  disable_interrupts();       // Disable interrups while reading buttons
  for (i = 0; i < 200; i++) ; // Disable to let input signal settle
  if (ad_ch)
  {  // Process NTC probe 1
     temp       = read_adc(AD_NTC1);
     ad_ntc1    = ((ad_ntc1 - (ad_ntc1 >> FILTER_SHIFT)) + temp);
     temp_ntc1  = ad_to_temp(ad_ntc1,&ad_err1);
     temp_ntc1 += eeprom_read_config(EEADR_MENU_ITEM(tc));
  } // if
  else
  {  // Process NTC probe 2
     temp       = read_adc(AD_NTC2);
     ad_ntc2    = ((ad_ntc2 - (ad_ntc2 >> FILTER_SHIFT)) + temp);
     temp_ntc2  = ad_to_temp(ad_ntc2,&ad_err2);
     temp_ntc2 += eeprom_read_config(EEADR_MENU_ITEM(tc2));
  } // else
  ad_ch = !ad_ch;
  enable_interrupts();     // Re-enable Interrupts
} // adc_task()

/*-----------------------------------------------------------------------------
  Purpose  : This task is called every 100 msec. and creates a slow PWM signal
             from pid_output: T = 12.5 seconds. This signal can be used to
             drive a Solid-State Relay (SSR).
  Variables: pid_out (global) is used
  Returns  : -
  ---------------------------------------------------------------------------*/
void pid_to_time(void)
{
    static uint8_t std_ptt = 1; // state [on, off]
    static uint8_t ltmr    = 0; // #times to set S3 to 0
    static uint8_t htmr    = 0; // #times to set S3 to 1
    uint16_t x;                 // temp. variable
     
    x   = (pid_out < 0) ? -pid_out : pid_out;
    x >>= 3; // divide by 8 to give 1.25 * pid_out
    
    switch (std_ptt)
    {
        case 0: // OFF
            if (ltmr == 0)
            {   // End of low-time
                htmr = (uint8_t)x; // htmr = 1.25 * pid_out
                if ((htmr > 0) && pwr_on) std_ptt = 1;
            } // if
            else ltmr--; // decrease timer
            S3_OFF;      // S3 output = 0
            break;
        case 1: // ON
            if (htmr == 0)
            {   // End of high-time
                ltmr = (uint8_t)(125 - x); // ltmr = 1.25 * (100 - pid_out)
                if ((ltmr > 0) || !pwr_on) std_ptt = 0;
            } // if
            else htmr--; // decrease timer
            S3_ON;       // S3 output = 1
            break;
    } // switch
} // pid_to_time()

/*-----------------------------------------------------------------------------
  Purpose  : This task is called every 100 msec. and reads the buttons, runs
             the STD and updates the 7-segment display.
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
void std_task(void)
{
    read_buttons(); // reads the buttons keys, result is stored in _buttons
    menu_fsm();     // Finite State Machine menu
    pid_to_time();  // Make Slow-PWM signal and send to S3 output-port
} // std_task()

/*-----------------------------------------------------------------------------
  Purpose  : This task is called every second and contains the main control
             task for the device. It also calls temperature_control().
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
void ctrl_task(void)
{
   int16_t sa, diff;
   
	  if (PB_IDR & LED2)
		     PB_ODR &= ~LED2;
	  else PB_ODR |=  LED2;
		
    if (eeprom_read_config(EEADR_MENU_ITEM(CF))) // true = Fahrenheit
         fahrenheit = true;
    else fahrenheit = false;
    if (eeprom_read_config(EEADR_MENU_ITEM(HrS))) // true = hours
         minutes = false; // control-timing is in hours 
    else minutes = true;  // control-timing is in minutes

   // Start with updating the alarm
   // cache whether the 2nd probe is enabled or not.
   if (eeprom_read_config(EEADR_MENU_ITEM(Pb2))) 
        probe2 = true;
   else probe2 = false;
   if (ad_err1 || (ad_err2 && probe2))
   {
       ALARM_ON;   // enable the piezo buzzer
       RELAYS_OFF; // disable the output relays
       if (menu_is_idle)
       {  // Make it less anoying to nagivate menu during alarm
          led_10 = LED_A;
	      led_1  = LED_L;
	      led_e  = led_01 = LED_OFF;
       } // if
       cooling_delay = heating_delay = 60;
   } else {
       ALARM_OFF; // reset the piezo buzzer
       if(((uint8_t)eeprom_read_config(EEADR_MENU_ITEM(rn))) < THERMOSTAT_MODE)
            led_e |=  LED_SET; // Indicate profile mode
       else led_e &= ~LED_SET;
 
       ts = eeprom_read_config(EEADR_MENU_ITEM(Ts)); // Read Ts [seconds]
       sa = eeprom_read_config(EEADR_MENU_ITEM(SA)); // Show Alarm parameter
       if (sa)
       {
           if (minutes) // is timing-control in minutes?
                diff = temp_ntc1 - setpoint;
           else diff = temp_ntc1 - eeprom_read_config(EEADR_MENU_ITEM(SP));

           if (diff < 0) diff = -diff;
	       if (sa < 0)
           {
  	          sa = -sa;
              if (diff <= sa)
                   ALARM_ON;  // enable the piezo buzzer
              else ALARM_OFF; // reset the piezo buzzer
           } else {
              if (diff >= sa)
                   ALARM_ON;  // enable the piezo buzzer
              else ALARM_OFF; // reset the piezo buzzer
	       } // if
       } // if
       if (ts == 0)                // PID Ts parameter is 0?
       {
           temperature_control();  // Run thermostat
           pid_out = 0;            // Disable PID-output
       } // if
       else pid_control();         // Run PID controller
       if (menu_is_idle)           // show temperature if menu is idle
       {
           if ((PD_IDR & ALARM) && show_sa_alarm)
           {
               led_10 = LED_S;
               led_1  = LED_A;
	           led_01 = LED_OFF;
           } else {
               led_e &= ~LED_POINT; // LED in middle, does not seem to work
               switch (sensor2_selected)
               {
                   case 0: value_to_led(temp_ntc1,LEDS_TEMP); 
                           break;
                   case 1: value_to_led(temp_ntc2,LEDS_TEMP); 
                           led_e |= LED_POINT;
                           break;
                   case 2: value_to_led(pid_out  ,LEDS_INT) ; 
                           break;
               } // switch
           } // else
           show_sa_alarm = !show_sa_alarm;
       } // if
   } // else
} // ctrl_task()

/*-----------------------------------------------------------------------------
  Purpose  : This task is called every minute or every hour and updates the
             current running temperature profile.
  Variables: minutes: timing control: false = hours, true = minutes
  Returns  : -
  ---------------------------------------------------------------------------*/
void prfl_task(void)
{
    static uint8_t min = 0;
    
    if (minutes)
    {   // call every minute
        update_profile();
        min = 0;
    } else {
        if (++min >= 60)
        {   // call every hour
            min = 0;
            update_profile(); 
        } // if
    } // else
} // prfl_task();

/*-----------------------------------------------------------------------------
  Purpose  : This task is called every 5 seconds and sends the current
             temperature with the nRF24l01+ device
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
void spi_task(void)
{
    uint8_t buf[8];
    bool    ok;
    uint8_t temp;
    
    // Take the time, and send it.  This will block until complete
    strcpy(buf,"emile!");
    nrf24_send(buf);           // Automatically goes to TX mode
    while(nrf24_isSending()) delay_msec(1); // Wait for transmission to end

    temp = nrf24_lastMessageStatus(); // Analyse last tranmission attempt
    /*if(temp == NRF24_TRANSMISSON_OK)
    {                    
        xprintf("> Tranmission went OK\r\n");
    }
    else if(temp == NRF24_MESSAGE_LOST)
    {                    
        xprintf("> Message is lost ...\r\n");    
    } 
    */
        
	/* Retranmission count indicates the tranmission quality */
	temp = nrf24_retransmissionCount();
	//xprintf("> Retranmission count: %d\r\n",temp);

	//nrf24_powerUpRx(); // Optionally, go back to RX mode ...
	
	nrf24_powerDown(); /* Power down after TX */
} // spi_task();

/*-----------------------------------------------------------------------------
  Purpose  : This is the main entry-point for the entire program.
             It initialises everything, starts the scheduler and dispatches
             all tasks.
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
int main(void)
{
	int ee = eedata[0]; // This is to prevent the linker from removing .eeprom section
	uint8_t ok, buf[5], bb = false;
	
    disable_interrupts();
    initialise_system_clock(); // Set system-clock to 16 MHz
    setup_gpio_ports();        // Init. needed output-ports for LED and keys
    setup_timer2();            // Set Timer 2 to 1 kHz
    pwr_on = eeprom_read_config(EEADR_POWER_ON); // check pwr_on flag
    i2c_init(bb);           // Init. I2C bus
    
    // Initialise all tasks for the scheduler
	scheduler_init();                    // clear task_list struct
    add_task(adc_task ,"ADC",  0,  500); // every 500 msec.
    add_task(std_task ,"STD", 50,  100); // every 100 msec.
    add_task(ctrl_task,"CTL",200, 1000); // every second
    add_task(prfl_task,"PRF",300,60000); // every minute / hour
    add_task(spi_task ,"SPI",400, 5000); // every 5 seconds
    disable_task("SPI");                 // Enable it after init is done
    enable_interrupts();

    nrf24_init();                 // Init. nRF24l01p module on SPI bus
    nrf24_config(110,6);          // Channel #110 , payload length: 6
    nrf24_tx_address(tx_address); // Set the device addresses
    nrf24_rx_address(rx_address);        
    enable_task("SPI");

    if (bb)
    {
        ok = ds2482_detect_bb(DS2482_ADDR);
    }
    else
    {
        ok = ds2482_detect(DS2482_ADDR);   // true
    };
    ok = ok + 1;
    
    while (1)
    {   // background-processes
        dispatch_tasks();     // Run task-scheduler()
        wait_for_interrupt(); // do nothing
    } // while
} // main()
/*==================================================================
  File Name    : stc1000p.h
  Author       : Emile
  ------------------------------------------------------------------
  This is the header file for stc1000p.c, which is the main-body of the
  STC1000+. This version is made for the STM8S105C6T6 uC.
 
  This file is part of STC1000+.
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
  Schematic of the connections to the MCU.
 
                        STM8S105C6T6 HW version 03
      MCU pin-name            Function |    MCU pin-name        Function
   ------------------------------------|--------------------------------
   01 NRST                             | 13 VDDA
   02 PA1/OSC                 HEAT     | 14 VSSA
   03 PA2/OSCOUT              COOL     | 15 PB7/AIN7            LED3
   04 VSSIO_1                          | 16 PB6/AIN6            LED2
   05 VSS                              | 17 PB5/AIN5[I2C_SDA]   LED1
   06 VCAP                             | 18 PB4/AIN4[I2C_SCL]   -
   07 VDD                              | 19 PB3/AIN3[TIM1_ETR]  NTC1 
   08 VDDIO_1                          | 20 PB2/AIN2[TIM1_CH3N] NTC2
   09 PA3/TIM2_CH3[TIME3_CH1] SSR      | 21 PB1/AIN1[TIM1_CH2N] -
   10 PA4                     BUZZER   | 22 PB0/AIN0[TIM1_CH1N] -
   11 PA5                     D433     | 23 PE7/AIN8            -
   12 PA6                     ISR      | 24 PE6/AIN9            -
   ------------------------------------|--------------------------------
   25 PE5/SPI_NSS             SPI_NSS  | 37 PE3/TIM1_BKIN       NRF24_CE
   26 PC1/TIM1_CH1/UART2_CK   CC1      | 38 PE2/I2C_SDA         I2C_SDA
   27 PC2/TIM1_CH2            CC2      | 39 PE1/I2C_SCL         I2C_SCL
   28 PC3/TIM1_CH3            CC3      | 40 PE0/CLK_CC0         C
   29 PC4/TIM1_CH4            CC4      | 41 PD0/TIM3_CH2...     B
   30 PC5/SPI_SCK             SPI_SCK  | 42 PD1/SWIM            SWIM
   31 VSSIO_2                          | 43 PD2/TIM3_CH1...     A 
   32 VDDIO_2                          | 44 PD3/TIM2_CH2...     DP_S
   33 PC6/SPI_MOSI            SPI_MOSI | 45 PD4/TIM2_CH1[BEEP]  G_PWR
   34 PC7/SPI_MISO            SPI_MISO | 46 PD5/UART2_TX        TX
   35 PG0                     F_DOWN   | 47 PD6/UART2_RX        RX
   36 PG1                     E_UP     | 48 PD7/TLI[TIM1_CH4]   D
   ---------------------------------------------------------------------

  Schematic of the bit numbers for the display LED's. Useful if custom characters are needed.
 
            * a * b   --------    *    --------       * C
                     /   a   /    g   /   a   /       e f
             d    f /       / b    f /       / b    ----
            ---     -------          -------     f / a / b
            *     /   g   /        /   g   /       ---
            c  e /       / c    e /       / c  e / g / c
                 --------    *    --------   *   ----  *
                   d         dp     d        dp   d    dp
  ------------------------------------------------------------------
  $Log: $
  ==================================================================
*/
#ifndef __STC1000P_H__
#define __STC1000P_H__

// #include <iostm8s003f3.h> for stock STC1000 PCB
#include <iostm8s105.h>
#include "stdint.h"
#include "delay.h"
     
/* Define STC-1000+ version number (XYY, X=major, YY=minor) */
/* Also, keep track of last version that has changes in EEPROM layout */
#define STC1000P_VERSION	(200)
#define STC1000P_EEPROM_VERSION	 (20)

// PORTG IO: 7 segment E + F
#define S7_E     (0x02)
#define S7_F     (0x01)
#define KEY_UP   (S7_E)
#define KEY_DOWN (S7_F)
#define PG_SEG7  (S7_E | S7_F)

// PORTE IO: SPI_NSS, I2C, 7-segment C
#define SPI_NSS  (0x20) /* CSN for nrf24l01 module */
#define NRF24_CE (0x08) /* CE for nrf24l01 module */
#define I2C_SDA  (0x04) /* Controlled by I2C peripheral */
#define I2C_SCL  (0x02) /* Controlled by I2C peripheral */
#define S7_C     (0x01)
#define PE_NC    (0xD0)
#define PE_SEG7  (S7_C)

// PORTD IO: 7-segments, keys and SWIM (PD1)
#define S7_D     (0x80)
#define UART_RX  (0x40) /* Controlled by UART 2 */
#define UART_TX  (0x20) /* Controlled by UART 2 */
#define S7_G     (0x10)
#define S7_DP    (0x08)
#define S7_A     (0x04)
#define SWIM     (0x02) /* Do not Initialize */
#define S7_B     (0x01)
#define KEY_PWR  (S7_G)
#define KEY_S    (S7_DP)
#define PD_SEG7  (S7_D | S7_G | S7_DP | S7_A | S7_B)

// PORTC IO: SPI and Common-Cathode pins
#define SPI_MISO   (0x80)
#define SPI_MOSI   (0x40)
#define SPI_CLK    (0x20)
#define CC_e       (0x10)
#define CC_01      (0x08)
#define CC_1       (0x04)
#define CC_10      (0x02)
#define CC_ALL     (CC_e | CC_01 | CC_1 | CC_10)

// PORTB IO: ADC-channels AIN3 (PB3) and AIN2 (PB2)
#define LED3        (0x80) /* Green LED, shows FO433 activity */
#define LED2        (0x40) /* Orange LED, Alive indicator */
#define LED1        (0x20)
#define AD_CHANNELS (0x0C)
#define PB_NC       (0x13)

#define AD_NTC1     (0x03) /* AIN3 */
#define AD_NTC2     (0x02) /* AIN2 */

// PORTA IO: outputs: D433 = PA5, Alarm (Buzzer) = PA4, SSR = PA3, Cool = PA2, Heat = PA1
#define ISR_OUT  (0x40)
#define D433_OUT (0x20)
#define ALARM    (0x10)
#define SSR      (0x08)
#define COOL     (0x04)
#define HEAT     (0x02)
#define PA_NC    (0x81)

#define ALARM_ON     (PA_ODR |=  ALARM)
#define ALARM_OFF    (PA_ODR &= ~ALARM)
#define ALARM_STATUS ((PA_IDR & ALARM) == ALARM)
#define SSR_ON       (PA_ODR |=  SSR)
#define SSR_OFF      (PA_ODR &= ~SSR)
#define COOL_ON      (PA_ODR |=  COOL)
#define COOL_OFF     (PA_ODR &= ~COOL)
#define COOL_STATUS  ((PA_IDR & COOL) == COOL)
#define HEAT_ON      (PA_ODR |=  HEAT)
#define HEAT_OFF     (PA_ODR &= ~HEAT)
#define HEAT_STATUS  ((PA_IDR & HEAT) == HEAT)
#define RELAYS_OFF   (PA_ODR &= ~(HEAT | COOL))
     
// PD7 PG1 PG0 PD4 PD3 PD2 PE0 PD0
//  D   E   F   G   dp  A   C   B 
#define LED_OFF	(0x00)
#define LED_ON  (0xFF)
#define LED_0	(0xE7)
#define LED_1	(0x03)
#define LED_2	(0xD5) 
#define LED_3  	(0x97)
#define LED_4  	(0x33)
#define LED_5  	(0xB6) 
#define LED_6  	(0xF6) 
#define LED_7  	(0x07)
#define LED_8  	(0xF7)
#define LED_9  	(0xB7)
#define LED_A  	(0x77)
#define LED_a	(0xD7)
#define LED_b	(0xF2) 
#define LED_C	(0xE4)
#define LED_c	(0xD0)
#define LED_d	(0xD3)
#define LED_e	(0xF5) 
#define LED_E	(0xF4)
#define LED_F	(0x74)
#define LED_H	(0x73)
#define LED_h	(0x72) 
#define LED_I	(0x03)
#define LED_J	(0xC3)
#define LED_L	(0xE0)
#define LED_n	(0x52) 
#define LED_O	(0xE7)
#define LED_o	(0xD2) 
#define LED_P	(0x75) 
#define LED_r	(0x50)	
#define LED_S	(0xB6) 
#define LED_t	(0xF0)
#define LED_U	(0xE3)
#define LED_u	(0xC2) 
#define LED_y	(0xB3)

#define LED_HEAT    (0x01)
#define LED_SET     (0x02)
#define LED_COOL    (0x04)
#define LED_DECIMAL (0x08)
#define LED_POINT   (0x10)
#define LED_CELS    (0x20)
#define LED_DEGR    (0x40)
#define LED_NEG     (0x80)

// Hardware defines for register definitions
// These value were defined in IAR, but not in Cosmic STM8
#define TIM2_SR1_UIF      (0x01)
#define CLK_ICKCR_HSIEN   (0x01)
#define CLK_ICKCR_HSIRDY  (0x02)
#define ADC_CR1_SPSEL_MSK (0x70)
#define CLK_SWCR_SWBSY    (0x01)
#define CLK_SWCR_SWEN     (0x02)
#define TIM2_IER_UIE      (0x01)
#define TIM2_CR1_CEN      (0x01)

// Function prototypes
void save_display_state(void);
void restore_display_state(void);
void multiplexer(void);
void initialise_system_clock(void);
void initialise_timer2(void);
void setup_timer2(void);
void setup_gpio_ports(void);
void adc_task(void);
void std_task(void);
void ctrl_task(void);
void prfl_task(void);
void one_wire_task(void);

#endif // __STC1000P_H__

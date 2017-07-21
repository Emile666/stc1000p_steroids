/*
 Copyright (C) 2011 J. Coliz <maniacbug@ymail.com>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 */

/**
 * @file RF24.h
 *
 * Class declaration for RF24 and helper enums
 */
#ifndef _nrf24l01p_h_
#define _nrf24l01p_h_

#include <stdbool.h>
#include "stdint.h"

 // Power Amplifier level: For use with setPALevel()
typedef enum { RF24_PA_MIN = 0,RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX, RF24_PA_ERROR } rf24_pa_dbm_e ;

 // Data rate.  How fast data moves through the air. For use with setDataRate()
typedef enum { RF24_1MBPS = 0, RF24_2MBPS, RF24_250KBPS } rf24_datarate_e;

 // CRC Length.  How big (if any) of a CRC is included. For use with setCRCLength()
typedef enum { RF24_CRC_DISABLED = 0, RF24_CRC_8, RF24_CRC_16 } rf24_crclength_e;

#define PIPE_ADDR_LEN (5)
typedef struct _pipe_addr { uint8_t a[PIPE_ADDR_LEN]; } pipe_addr;

/* Memory Map */
#define CONFIG      (0x00)
#define EN_AA       (0x01)
#define EN_RXADDR   (0x02)
#define SETUP_AW    (0x03)
#define SETUP_RETR  (0x04)
#define RF_CH       (0x05)
#define RF_SETUP    (0x06)
#define STATUS      (0x07)
#define OBSERVE_TX  (0x08)
#define CD          (0x09)
#define RX_ADDR_P0  (0x0A)
#define RX_ADDR_P1  (0x0B)
#define RX_ADDR_P2  (0x0C)
#define RX_ADDR_P3  (0x0D)
#define RX_ADDR_P4  (0x0E)
#define RX_ADDR_P5  (0x0F)
#define TX_ADDR     (0x10)
#define RX_PW_P0    (0x11)
#define RX_PW_P1    (0x12)
#define RX_PW_P2    (0x13)
#define RX_PW_P3    (0x14)
#define RX_PW_P4    (0x15)
#define RX_PW_P5    (0x16)
#define FIFO_STATUS (0x17)
#define DYNPD	    (0x1C)
#define FEATURE	    (0x1D)

/* Bit Mnemonics */
#define MASK_RX_DR  (6)
#define MASK_TX_DS  (5)
#define MASK_MAX_RT (4)
#define EN_CRC      (3)
#define CRCO        (2)
#define PWR_UP      (1)
#define PRIM_RX     (0)
#define ENAA_P5     (5)
#define ENAA_P4     (4)
#define ENAA_P3     (3)
#define ENAA_P2     (2)
#define ENAA_P1     (1)
#define ENAA_P0     (0)
#define ERX_P5      (5)
#define ERX_P4      (4)
#define ERX_P3      (3)
#define ERX_P2      (2)
#define ERX_P1      (1)
#define ERX_P0      (0)
#define AW          (0)
#define ARD         (4)
#define ARC         (0)
#define PLL_LOCK    (4)
#define RF_DR       (3)
#define RF_PWR      (6)
#define RX_DR       (6)
#define TX_DS       (5)
#define MAX_RT      (4)
#define RX_P_NO     (1)
#define TX_FULL     (0)
#define PLOS_CNT    (4)
#define ARC_CNT     (0)
#define TX_REUSE    (6)
#define FIFO_FULL   (5)
#define TX_EMPTY    (4)
#define RX_FULL     (1)
#define RX_EMPTY    (0)
#define DPL_P5	    (5)
#define DPL_P4	    (4)
#define DPL_P3	    (3)
#define DPL_P2	    (2)
#define DPL_P1	    (1)
#define DPL_P0	    (0)
#define EN_DPL	    (2)
#define EN_ACK_PAY  (1)
#define EN_DYN_ACK  (0)

// Copied from Arduino Library
#define HIGH (0x01)
#define LOW  (0x00)
#define _BV(x) (1<<(x)) 
#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
#define abs(x) ((x)>0?(x):-(x)) 

/* Instruction Mnemonics */
#define R_REGISTER    (0x00)
#define W_REGISTER    (0x20)
#define REGISTER_MASK (0x1F)
#define ACTIVATE      (0x50)
#define R_RX_PL_WID   (0x60)
#define R_RX_PAYLOAD  (0x61)
#define W_TX_PAYLOAD  (0xA0)
#define W_ACK_PAYLOAD (0xA8)
#define FLUSH_TX      (0xE1)
#define FLUSH_RX      (0xE2)
#define REUSE_TX_PL   (0xE3)
#define NOP           (0xFF)

/* Non-P omissions */
#define LNA_HCURR   (0)

/* P model memory Map */
#define RPD         (0x09)

/* P model bit Mnemonics */
#define RF_DR_LOW   (5)
#define RF_DR_HIGH  (3)
#define RF_PWR_LOW  (1)
#define RF_PWR_HIGH (2) 

//  Low-level routines that address the chip directly.  Regular users cannot
//  ever call these.  They are documented for completeness and for developers who
//  may want to extend this class.
void    csn(uint8_t mode); // Set Chip-select pin (active-low)
void    ce(uint8_t level); // Set chip-enable (active-high)
uint8_t read_register(uint8_t reg, uint8_t *buf, uint8_t len); // Read a chunk of data in from a register
uint8_t read_register1(uint8_t reg); // Read single byte from a register
void    write_register(uint8_t reg, uint8_t* buf, uint8_t len); // Write a chunk of data to a register
void    write_register1(uint8_t reg, uint8_t value); // Write a single byte to a register
void    write_payload(uint8_t *buf, uint8_t len); // Write the transmit payload
void    read_payload(void* buf, uint8_t len);    // Read the receive payload
void    flush_rx(void);   // Empty the receive buffer
void    flush_tx(void);   // Empty the transmit buffer
uint8_t get_status(void); // Retrieve the current status of the chip
void print_status(uint8_t status);    // Decode and print the given status to stdout
void print_observe_tx(uint8_t value);    // Decode and print the given 'observe_tx' value to stdout
void print_byte_register(char* name, uint8_t reg, uint8_t qty); // Print the name and value of an 8-bit register to stdout
void print_address_register(char* name, uint8_t reg, uint8_t qty);    // Print the name and value of a 40-bit address register to stdout
void toggle_features(void);    // Turn on or off the special features of the chip

//  These are the main methods you need to operate the chip
void init_nrf24l01p(void);
void startListening(void); // Start listening on the pipes opened for reading.
void stopListening(void);  // Stop listening for incoming messages
bool write( const void* buf, uint8_t len );    // Write to the open writing pipe
bool available(void);    // Test whether there are bytes available to be read
bool read(void *buf, uint8_t len); // Read the payload, return the last payload received
void openWritingPipe(pipe_addr *value);    // Open a pipe for writing
void openReadingPipe(uint8_t child, pipe_addr *address); // Open a pipe for reading

void    setRetries(uint8_t delay, uint8_t count);    // Set the number and delay of retries upon failed submit
void    setChannel(uint8_t channel);    // Set RF communication channel
void    setPayloadSize(uint8_t size);    // Set Static Payload Size
uint8_t getPayloadSize(void);    // Get Static Payload Size
uint8_t getDynamicPayloadSize(void);    // Get Dynamic Payload Size
void    enableAckPayload(void);    // Enable custom payloads on the acknowledge packets
void    enableDynamicPayloads(void);    // Enable dynamically-sized payloads
bool    isPVariant(void) ;    // Determine whether the hardware is an nRF24L01+ or not.
void    setAutoAck1(bool enable);    // Enable or disable auto-acknowlede packets
void    setAutoAck( uint8_t pipe, bool enable ) ;    // Enable or disable auto-acknowlede packets on a per pipeline basis.
void    setPALevel( rf24_pa_dbm_e level ) ;    // Set Power Amplifier (PA) level to one of four levels.
rf24_pa_dbm_e getPALevel( void ) ;    // Fetches the current PA level.
bool    setDataRate(rf24_datarate_e speed);    // Set the transmission data rate
rf24_datarate_e getDataRate( void ) ;    // Fetches the transmission data rate
void setCRCLength(rf24_crclength_e length);    // Set the CRC length
rf24_crclength_e getCRCLength(void);    // Get the CRC length
void disableCRC( void ) ;    // Disable CRC validation

void printDetails(void);    // Print a giant block of debugging information to stdout
void powerDown(void);    // Enter low-power mode
void powerUp(void) ;    // Leave low-power mode - making radio more responsive
bool available1(uint8_t* pipe_num);    // Test whether there are bytes available to be read
void startWrite( const void* buf, uint8_t len );    // Non-blocking write to the open writing pipe
void writeAckPayload(uint8_t pipe, const void* buf, uint8_t len);    // Write an ack payload for the specified pipe
bool isAckPayloadAvailable(void);    // Determine if an ack payload was received in the most recent call to write().
uint8_t whatHappened(void);    // Call this when you get an interrupt to find out why
bool testCarrier(void);    // Test whether there was a carrier on the line for the previous listening period.
bool testRPD(void) ;    // Test whether a signal (carrier or otherwise) >= -64dBm is present on the channel.

#endif // __RF24_H__


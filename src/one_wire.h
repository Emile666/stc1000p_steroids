#ifndef ONE_WIRE_H
#define ONE_WIRE_H
//-----------------------------------------------------------------------------
// Created: 20-4-2013 22:32:11
// Author : Emile
// File   : $Id$
//-----------------------------------------------------------------------------
#include <stdint.h>
#include <stdbool.h>

#define OW_SEARCH_ROM_CMD        (0xF0)
#define OW_READ_ROM_CMD			 (0x33)
#define OW_MATCH_ROM_CMD		 (0x55)
#define OW_SKIP_ROM_CMD			 (0xCC)
#define OW_ALARM_SEARCH_CMD		 (0xEC)

#define OW_CONVERT_T_FCMD		 (0x44)
#define OW_WRITE_SCRATCHPAD_FCMD (0x4E)
#define OW_READ_SCRATCHPAD_FCMD  (0xBE)
#define OW_COPY_SCRATCHPAD_FCMD  (0x48)
#define OW_RECALL_EE_FCMD        (0xB8)
#define OW_READ_PSUP_FCMD        (0xB4)

// 1-Wire API for DS2482 function prototypes
bool    OW_reset(uint8_t addr);
uint8_t OW_touch_bit(uint8_t sendbit, uint8_t addr);
void    OW_write_bit(uint8_t sendbit, uint8_t addr);
uint8_t OW_read_bit(uint8_t addr);
bool    OW_write_byte(uint8_t sendbyte, uint8_t addr);
uint8_t OW_read_byte(uint8_t addr);
uint8_t OW_touch_byte(uint8_t sendbyte, uint8_t addr);
void    OW_block(uint8_t *tran_buf, uint8_t tran_len, uint8_t addr);
uint8_t OW_first(uint8_t addr);
uint8_t OW_next(uint8_t addr);
bool    OW_verify(uint8_t addr);
void    OW_target_setup(uint8_t family_code);
void    OW_family_skip_setup(void);
uint8_t OW_search(uint8_t addr);
uint8_t ds18b20_start_conversion(uint8_t i2c_addr);
int16_t ds18b20_read(uint8_t i2c_addr, uint8_t *err, uint8_t s2);

// Helper functions
uint8_t calc_crc8(uint8_t data);

#endif

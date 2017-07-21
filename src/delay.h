#ifndef _DELAY_H
#define _DELAY_H

#include "stdint.h"

#define enable_interrupts()  {_asm("rim\n");} /* enable interrupts */
#define disable_interrupts() {_asm("sim\n");} /* disable interrupts */
#define wait_for_interrupt() {_asm("wfi\n");} /* Wait For Interrupt */

uint32_t millis(void);
void     delay_msec(uint16_t ms);
void     delay_usec(uint16_t us);

#endif
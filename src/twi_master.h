#ifndef __TWI_MASTER_H
#define __TWI_MASTER_H

#include "stdint.h"

bool i2c_init(void);
bool i2c_start(uint8_t addr);
void i2c_start_wait(uint8_t addr);
bool i2c_rep_start(uint8_t addr);
void __attribute__((noinline)) i2c_stop(void) asm("ass_i2c_stop")
    __attribute__((used));
bool __attribute__((noinline)) i2c_write(uint8_t value) asm("ass_i2c_write")
    __attribute__((used));
uint8_t i2c_read(bool last);

#endif
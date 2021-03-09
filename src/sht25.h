#ifndef __SHT25_H
#define __SHT25_H
#include "stdint.h"

void sht25_init();
int16_t sht25_read_temperature(void);
int16_t sht25_read_humidity(void);

#endif
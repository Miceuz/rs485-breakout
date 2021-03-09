#ifndef __DPS310_H
#define __DPS310_H
#include "stdbool.h"
#include "stdint.h"

bool dps310_init();
int16_t measureTempOnce(int16_t *result);
int16_t measurePressureOnce(int16_t *result);
#endif
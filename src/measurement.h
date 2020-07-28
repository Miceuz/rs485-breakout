#ifndef __MEASUREMENT_H
#define __MEASUREMENT_H
#include "model.h"

void adcSetup();
void performMeasurement(t_InputRegisters *);
void processMeasurements(t_InputRegisters *);
void timer1msStart(volatile uint16_t *ptrToTimeout);
void measurementReset();
#endif
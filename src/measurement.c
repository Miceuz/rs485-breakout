#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdbool.h>

#include "model.h"

#define CHANNEL_ADC7 7
#define CHANNEL_ADC8 8

#define STATE_MEASUREMENT_OFF 0
#define STATE_MEASUREMENT_STABILIZE 1
#define STATE_MEASUREMENT_IN_PROGRESS 2
uint8_t measurementState = STATE_MEASUREMENT_OFF;

volatile uint16_t milliseconds = 0;
volatile uint16_t *measurementTimeoutMs;
uint8_t temp;

inline static void adcEnable() { ADCSRA |= _BV(ADEN); }

inline static void adcDisable() { ADCSRA &= ~_BV(ADEN); }

void adcSetup() {
  ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS0);
  ADMUXB = 0;
  DIDR0 |= _BV(ADC7D) |
           _BV(ADC8D);  // | _BV(ADC0D) | _BV(ADC1D) | _BV(ADC2D) | _BV(ADC4D) |
  // _BV(ADC6D) | _BV(ADC8D);

  ACSR0A = _BV(ACD0);  // disable comparators
  ACSR1A = _BV(ACD1);

  adcDisable();
}

uint16_t adcReadChannel(uint8_t channel) {
  ADMUXA = channel;
  ADCSRA |= _BV(ADSC);
  loop_until_bit_is_clear(ADCSRA, ADSC);
  uint16_t ret = ADC;
  return ret;
}

inline static bool isTimeToMeasure() {
  return milliseconds > *measurementTimeoutMs;
}

inline static bool isTimeToStabilizeOver() {
  return milliseconds > *measurementTimeoutMs + 2;
}

inline static void startMeasurementTimer() {
  TIMSK1 &= ~_BV(OCIE1A);
  milliseconds = 0;
  TCNT1 = 0;
  TIMSK1 |= _BV(OCIE1A);
}

void timer1msStart(volatile uint16_t *ptrToTimeout) {
  measurementTimeoutMs = ptrToTimeout;
  OCR1A = (uint16_t)16000L;
  TIMSK1 |= _BV(OCIE1A);
  TCCR1B = _BV(CS10) | _BV(WGM12);
}

ISR(TIMER1_COMPA_vect) { milliseconds++; }

void measurementReset() {
  adcDisable();
  startMeasurementTimer();
  measurementState = STATE_MEASUREMENT_OFF;
}

bool isMeasurementInProgress() {
  return measurementState != STATE_MEASUREMENT_OFF;
}

inline static void measurementPeripheryOn() { adcEnable(); }

void processMeasurements(t_InputRegisters inputRegisters) {
  switch (measurementState) {
    case STATE_MEASUREMENT_OFF:
      if (isTimeToMeasure()) {
        measurementPeripheryOn();
        measurementState = STATE_MEASUREMENT_STABILIZE;
      }
      break;

    case STATE_MEASUREMENT_STABILIZE:
      if (isTimeToStabilizeOver()) {
        measurementState = STATE_MEASUREMENT_IN_PROGRESS;
      }
      break;

    case STATE_MEASUREMENT_IN_PROGRESS: {
      uint16_t adc7 = adcReadChannel(CHANNEL_ADC7);
      uint16_t adc8 = adcReadChannel(CHANNEL_ADC8);
      inputRegisters.asStruct.adc7 = adc7;
      inputRegisters.asStruct.adc8 = adc8;

      measurementReset();
    } break;
  }
}

inline static void forceStartMeasurement() {
  cli();
  milliseconds = *measurementTimeoutMs + 1;
  sei();
  measurementPeripheryOn();
  measurementState = STATE_MEASUREMENT_STABILIZE;
}

void performMeasurement(t_InputRegisters inputRegisters) {
  forceStartMeasurement();
  while (isMeasurementInProgress()) {
    processMeasurements(inputRegisters);
  }
}

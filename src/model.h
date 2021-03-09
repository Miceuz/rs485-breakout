#ifndef __MODEL_H
#define __MODEL_H

// Input registers are read only registers provided by the device.
// Put your sensor data here
typedef union {
  uint16_t asArray[4];
  struct {
    uint16_t temperature;
    uint16_t humidity;
    uint16_t pressure;
    uint16_t fwVersion;
  } asStruct;
} t_InputRegisters;

// Holding registers are read/write registers.
// Use these registers to configure the device and control actuators.
typedef union {
  uint16_t asArray[5];
  struct {
    uint16_t address;               // register number 0
    uint16_t baud;                  // register number 1
    uint16_t parity;                // register number 2
    uint16_t measurementIntervalMs; // register number 3
    uint16_t sleepTimeS;            // register number 4
  } asStruct;
} t_HoldingRegisters;

#endif
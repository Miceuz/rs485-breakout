#include "stdbool.h"
#include "stdint.h"
#include "twi_master.h"
#include "util/delay.h"

#define DPS310__OSR_SE 3U

#define DPS__RESULT_BLOCK_LENGTH 3
#define DPS310_NUM_OF_REGMASKS 16
#define DPS__STD_SLAVE_ADDRESS (0x77U << 1)

#define DPS__SUCCEEDED 0
#define DPS__FAIL_UNKNOWN -1
#define DPS__FAIL_INIT_FAILED -2
#define DPS__FAIL_TOOBUSY -3
#define DPS__FAIL_UNFINISHED -4

#define DPS__MEASUREMENT_RATE_1 0
#define DPS__MEASUREMENT_RATE_2 1
#define DPS__MEASUREMENT_RATE_4 2
#define DPS__MEASUREMENT_RATE_8 3
#define DPS__MEASUREMENT_RATE_16 4
#define DPS__MEASUREMENT_RATE_32 5
#define DPS__MEASUREMENT_RATE_64 6
#define DPS__MEASUREMENT_RATE_128 7

#define DPS__OVERSAMPLING_RATE_1 DPS__MEASUREMENT_RATE_1
#define DPS__OVERSAMPLING_RATE_2 DPS__MEASUREMENT_RATE_2
#define DPS__OVERSAMPLING_RATE_4 DPS__MEASUREMENT_RATE_4
#define DPS__OVERSAMPLING_RATE_8 DPS__MEASUREMENT_RATE_8
#define DPS__OVERSAMPLING_RATE_16 DPS__MEASUREMENT_RATE_16
#define DPS__OVERSAMPLING_RATE_32 DPS__MEASUREMENT_RATE_32
#define DPS__OVERSAMPLING_RATE_64 DPS__MEASUREMENT_RATE_64
#define DPS__OVERSAMPLING_RATE_128 DPS__MEASUREMENT_RATE_128

enum Registers_e {
  PROD_ID = 0,
  REV_ID,
  TEMP_SENSOR,    // internal vs external
  TEMP_SENSORREC, // temperature sensor recommendation
  TEMP_SE,        // temperature shift enable (if temp_osr>3)
  PRS_SE,         // pressure shift enable (if prs_osr>3)
  FIFO_FL,        // FIFO flush
  FIFO_EMPTY,     // FIFO empty
  FIFO_FULL,      // FIFO full
  INT_HL,
  INT_SEL, // interrupt select
};

typedef struct {
  uint8_t regAddress;
  uint8_t mask;
  uint8_t shift;
} RegMask_t;

const RegMask_t registers[DPS310_NUM_OF_REGMASKS] = {
    {0x0D, 0x0F, 0}, // PROD_ID
    {0x0D, 0xF0, 4}, // REV_ID
    {0x07, 0x80, 7}, // TEMP_SENSOR
    {0x28, 0x80, 7}, // TEMP_SENSORREC
    {0x09, 0x08, 3}, // TEMP_SE
    {0x09, 0x04, 2}, // PRS_SE
    {0x0C, 0x80, 7}, // FIFO_FL
    {0x0B, 0x01, 0}, // FIFO_EMPTY
    {0x0B, 0x02, 1}, // FIFO_FULL
    {0x09, 0x80, 7}, // INT_HL
    {0x09, 0x70, 4}, // INT_SEL
};

typedef struct {
  uint8_t regAddress;
  uint8_t length;
} RegBlock_t;

const RegBlock_t coeffBlock = {0x10, 18};

enum Config_Registers_e {
  TEMP_MR = 0, // temperature measure rate
  TEMP_OSR,    // temperature measurement resolution
  PRS_MR,      // pressure measure rate
  PRS_OSR,     // pressure measurement resolution
  MSR_CTRL,    // measurement control
  FIFO_EN,

  TEMP_RDY,
  PRS_RDY,
  INT_FLAG_FIFO,
  INT_FLAG_TEMP,
  INT_FLAG_PRS,
};

#define NUM_OF_COMMON_REGMASKS 16
const RegMask_t config_registers[NUM_OF_COMMON_REGMASKS] = {
    {0x07, 0x70, 4}, // TEMP_MR
    {0x07, 0x07, 0}, // TEMP_OSR
    {0x06, 0x70, 4}, // PRS_MR
    {0x06, 0x07, 0}, // PRS_OSR
    {0x08, 0x07, 0}, // MSR_CTRL
    {0x09, 0x02, 1}, // FIFO_EN

    {0x08, 0x20, 5}, // TEMP_RDY
    {0x08, 0x10, 4}, // PRS_RDY
    {0x0A, 0x04, 2}, // INT_FLAG_FIFO
    {0x0A, 0x02, 1}, // INT_FLAG_TEMP
    {0x0A, 0x01, 0}, // INT_FLAG_PRS
};

enum Mode {
  IDLE = 0x00,
  CMD_PRS = 0x01,
  CMD_TEMP = 0x02,
  CMD_BOTH = 0x03, // only for DPS422
  CONT_PRS = 0x05,
  CONT_TMP = 0x06,
  CONT_BOTH = 0x07
};

enum RegisterBlocks_e {
  PRS = 0, // pressure value
  TEMP,    // temperature value
};

const RegBlock_t registerBlocks[2] = {
    {0x00, 3},
    {0x03, 3},
};

#define DPS__NUM_OF_SCAL_FACTS 8
const int32_t scaling_facts[DPS__NUM_OF_SCAL_FACTS] = {
    524288, 1572864, 3670016, 7864320, 253952, 516096, 1040384, 2088960};

int16_t readByteBitfield(RegMask_t regMask);
int16_t writeByteBitfield(uint8_t data, RegMask_t regMask);
int16_t readcoeffs(void);
int16_t configTemp(uint8_t tempMr, uint8_t tempOsr);
int16_t configPressure(uint8_t prsMr, uint8_t prsOsr);
int16_t standby(void);
int16_t correctTemp(void);
int16_t measureTempOnce(int16_t *result);
int16_t startMeasureTempOnce();
int16_t startMeasurePressureOnce();

int16_t m_tempSensor;
uint8_t m_slaveAddress = DPS__STD_SLAVE_ADDRESS;

// Compensation coefficients
int32_t m_c00;
int32_t m_c10;
int32_t m_c01;
int32_t m_c11;
int32_t m_c20;
int32_t m_c21;
int32_t m_c30;
int32_t m_c0;
int32_t m_c1;

uint8_t m_tempOsr;
uint8_t m_prsOsr;

int32_t m_lastTempRaw;

bool dps310_init(void) {
  // int16_t prodId = readByteBitfield(registers[PROD_ID]);
  // if (prodId < 0) {
  //   // Connected device is not a Dps310
  //   m_initFail = 1U;
  //   return;
  // }
  // m_productID = prodId;

  // int16_t revId = readByteBitfield(registers[REV_ID]);
  // if (revId < 0) {
  //   m_initFail = 1U;
  //   return;
  // }
  // m_revisionID = revId;

  // find out which temperature sensor is calibrated with coefficients...
  int16_t sensor = readByteBitfield(registers[TEMP_SENSORREC]);
  if (sensor < 0) {
    return false;
  }

  //...and use this sensor for temperature measurement
  m_tempSensor = sensor;
  if (writeByteBitfield((uint8_t)sensor, registers[TEMP_SENSOR]) < 0) {
    return false;
  }

  // read coefficients
  if (readcoeffs() < 0) {
    return false;
  }

  // set to standby for further configuration
  standby();

  // set measurement precision and rate to standard values;
  configTemp(DPS__MEASUREMENT_RATE_1, DPS__OVERSAMPLING_RATE_1);
  configPressure(DPS__MEASUREMENT_RATE_1, DPS__OVERSAMPLING_RATE_1);

  // perform a first temperature measurement
  // the most recent temperature will be saved internally
  // and used for compensation when calculating pressure
  int16_t trash;
  measureTempOnce(&trash);

  // make sure the DPS310 is in standby after initialization
  standby();

  // Fix IC with a fuse bit problem, which lead to a wrong temperature
  // Should not affect ICs without this problem
  correctTemp();

  return true;
}

int16_t readByte(uint8_t regAddress) {
  if (!i2c_start(m_slaveAddress & ~0x01)) {
    i2c_stop();
    return DPS__FAIL_UNKNOWN;
  }
  if (!i2c_write(regAddress)) {
    i2c_stop();
    return DPS__FAIL_UNKNOWN;
  }
  if (!i2c_rep_start(m_slaveAddress | 0x01)) {
    i2c_stop();
    return DPS__FAIL_UNKNOWN;
  }

  uint8_t ret = i2c_read(true);
  i2c_stop();

  return ret;
}

int16_t readByteBitfield(RegMask_t regMask) {
  int16_t ret = readByte(regMask.regAddress);
  if (ret < 0) {
    return ret;
  }
  return (((uint8_t)ret) & regMask.mask) >> regMask.shift;
}

int16_t writeByte(uint8_t regAddress, uint8_t data, uint8_t check) {
  if (!i2c_start(m_slaveAddress & ~0x01)) {
    i2c_stop();
    return DPS__FAIL_UNKNOWN;
  }
  if (!i2c_write(regAddress)) {
    i2c_stop();
    return DPS__FAIL_UNKNOWN;
  }
  if (!i2c_write(data)) {
    i2c_stop();
    return DPS__FAIL_UNKNOWN;
  }
  i2c_stop();

  if (0 == check) {
    return DPS__SUCCEEDED;
  } else {
    i2c_start(m_slaveAddress & ~0x01);
    i2c_write(regAddress);
    i2c_rep_start(m_slaveAddress | 0x01);
    uint8_t d = i2c_read(true);
    i2c_stop();

    if (data == d) {
      return DPS__SUCCEEDED;
    } else {
      return DPS__FAIL_UNKNOWN;
    }
  }
}

int16_t writeByteBitfield_(uint8_t data, uint8_t regAddress, uint8_t mask,
                           uint8_t shift, uint8_t check) {
  int16_t old = readByte(regAddress);
  if (old < 0) {
    // fail while reading
    return old;
  }
  return writeByte(regAddress,
                   ((uint8_t)old & ~mask) | ((data << shift) & mask), check);
}

int16_t writeByteBitfield(uint8_t data, RegMask_t regMask) {
  return writeByteBitfield_(data, regMask.regAddress, regMask.mask,
                            regMask.shift, 0U);
}

void getTwosComplement(int32_t *raw, uint8_t length) {
  if (*raw & ((uint32_t)1 << (length - 1))) {
    *raw -= (uint32_t)1 << length;
  }
}

int16_t readBlock(RegBlock_t regBlock, uint8_t *buffer) {
  // do not read if there is no buffer
  if (buffer == 0) {
    return 0; // 0 bytes read successfully
  }

  if (!i2c_start(m_slaveAddress & ~0x01)) {
    i2c_stop();
    return DPS__FAIL_UNKNOWN;
  }
  if (!i2c_write(regBlock.regAddress)) {
    i2c_stop();
    return DPS__FAIL_UNKNOWN;
  }

  if (!i2c_rep_start(m_slaveAddress | 0x01)) {
    i2c_stop();
    return DPS__FAIL_UNKNOWN;
  }

  // m_i2cbus->beginTransmission(m_slaveAddress);
  // m_i2cbus->write(regBlock.regAddress);
  // m_i2cbus->endTransmission(false);

  // request length bytes from slave
  // int16_t ret = m_i2cbus->requestFrom(m_slaveAddress, regBlock.length, 1U);
  // read all received bytes to buffer
  int16_t i;
  for (i = 0; i < regBlock.length - 1; i++) {
    buffer[i] = i2c_read(false);
  }
  buffer[i] = i2c_read(true);
  i2c_stop();

  return regBlock.length;
}

int16_t readcoeffs(void) {
  // TODO: remove magic number
  uint8_t buffer[18];
  // read COEF registers to buffer
  int16_t ret = readBlock(coeffBlock, buffer);

  // compose coefficients from buffer content
  m_c0 = ((uint32_t)buffer[0] << 4) | (((uint32_t)buffer[1] >> 4) & 0x0F);
  getTwosComplement(&m_c0, 12);

  // now do the same thing for all other coefficients
  m_c1 = (((uint32_t)buffer[1] & 0x0F) << 8) | (uint32_t)buffer[2];
  getTwosComplement(&m_c1, 12);
  m_c00 = ((uint32_t)buffer[3] << 12) | ((uint32_t)buffer[4] << 4) |
          (((uint32_t)buffer[5] >> 4) & 0x0F);
  getTwosComplement(&m_c00, 20);
  m_c10 = (((uint32_t)buffer[5] & 0x0F) << 16) | ((uint32_t)buffer[6] << 8) |
          (uint32_t)buffer[7];
  getTwosComplement(&m_c10, 20);

  m_c01 = ((uint32_t)buffer[8] << 8) | (uint32_t)buffer[9];
  getTwosComplement(&m_c01, 16);

  m_c11 = ((uint32_t)buffer[10] << 8) | (uint32_t)buffer[11];
  getTwosComplement(&m_c11, 16);
  m_c20 = ((uint32_t)buffer[12] << 8) | (uint32_t)buffer[13];
  getTwosComplement(&m_c20, 16);
  m_c21 = ((uint32_t)buffer[14] << 8) | (uint32_t)buffer[15];
  getTwosComplement(&m_c21, 16);
  m_c30 = ((uint32_t)buffer[16] << 8) | (uint32_t)buffer[17];
  getTwosComplement(&m_c30, 16);
  return DPS__SUCCEEDED;
}

int16_t setOpMode(uint8_t opMode) {
  if (writeByteBitfield(opMode, config_registers[MSR_CTRL]) ==
      DPS__FAIL_UNKNOWN) {
    return DPS__FAIL_UNKNOWN;
  }
  // m_opMode = (Mode)opMode;
  return DPS__SUCCEEDED;
}

int16_t disableFIFO() {
  // flush fifo
  int16_t ret = writeByteBitfield(1U, registers[FIFO_FL]);
  // disable fifo
  ret = writeByteBitfield(0U, config_registers[FIFO_EN]);
  return ret;
}

int16_t standby(void) {
  // set device to idling mode
  int16_t ret = setOpMode(IDLE);
  if (ret != DPS__SUCCEEDED) {
    return ret;
  }
  ret = disableFIFO();
  return ret;
}

int16_t configTemp(uint8_t tempMr, uint8_t tempOsr) {
  tempMr &= 0x07;
  tempOsr &= 0x07;
  // two accesses to the same register; for readability
  int16_t ret = writeByteBitfield(tempMr, config_registers[TEMP_MR]);
  ret = writeByteBitfield(tempOsr, config_registers[TEMP_OSR]);

  writeByteBitfield(m_tempSensor, registers[TEMP_SENSOR]);

  if (tempOsr > DPS310__OSR_SE) {
    ret = writeByteBitfield(1U, registers[TEMP_SE]);
  } else {
    ret = writeByteBitfield(0U, registers[TEMP_SE]);
  }

  // abort immediately on fail
  if (ret != DPS__SUCCEEDED) {
    return DPS__FAIL_UNKNOWN;
  }
  m_tempOsr = tempOsr;
}

int16_t configPressure(uint8_t prsMr, uint8_t prsOsr) {
  prsMr &= 0x07;
  prsOsr &= 0x07;
  int16_t ret = writeByteBitfield(prsMr, config_registers[PRS_MR]);
  ret = writeByteBitfield(prsOsr, config_registers[PRS_OSR]);

  // set PM SHIFT ENABLE if oversampling rate higher than eight(2^3)
  if (prsOsr > DPS310__OSR_SE) {
    ret = writeByteBitfield(1U, registers[PRS_SE]);
  } else {
    ret = writeByteBitfield(0U, registers[PRS_SE]);
  }

  // abort immediately on fail
  if (ret != DPS__SUCCEEDED) {
    return DPS__FAIL_UNKNOWN;
  }
  m_prsOsr = prsOsr;
}

int16_t getRawResult(int32_t *raw, RegBlock_t reg) {
  uint8_t buffer[DPS__RESULT_BLOCK_LENGTH] = {0};
  if (readBlock(reg, buffer) != DPS__RESULT_BLOCK_LENGTH) {
    return DPS__FAIL_UNKNOWN;
  }

  *raw = (uint32_t)buffer[0] << 16 | (uint32_t)buffer[1] << 8 |
         (uint32_t)buffer[2];
  getTwosComplement(raw, 24);
  return DPS__SUCCEEDED;
}

int16_t calcTemp(int32_t raw) {
  int32_t temp = raw;

  // update last measured temperature
  // it will be used for pressure compensation
  m_lastTempRaw = temp;

  // Calculate compensated temperature
  temp = m_c0 * 10 / 2 + m_c1 * temp * 10 / scaling_facts[m_tempOsr];

  return temp;
}

int16_t calcPressure(int32_t raw) {
  int32_t prs = raw;

  // Calculate compensated pressure
  prs = m_c00 +
        prs *
            (m_c10 +
             prs * (m_c20 + (int64_t)prs * m_c30 / scaling_facts[m_prsOsr]) /
                 scaling_facts[m_prsOsr]) /
            scaling_facts[m_prsOsr] +
        m_lastTempRaw *
            (m_c01 +
             prs * (m_c11 + (int64_t)prs * m_c21 / scaling_facts[m_prsOsr]) /
                 scaling_facts[m_prsOsr]) /
            scaling_facts[m_tempOsr];
  prs = prs - 101325;
  return prs;
}

// according to datasheet, conversion time is
// 3.6 + 1.6 * (oversamplingRate - 1)
// we add 10ms at the end just in case
uint8_t calcBusyTime(uint8_t oversampllingRate) {
  return (36 + 16 * ((1 << oversampllingRate) - 1)) / 10 + 10;
}

int16_t measureTempOnce(int16_t *result) {
  // Start measurement
  int16_t ret = startMeasureTempOnce();
  if (ret != DPS__SUCCEEDED) {
    return ret;
  }

  uint8_t delay = calcBusyTime(m_tempOsr);
  for (int i = 0; i < delay; i++) {
    _delay_ms(1);
  }

  // int16_t rdy = 0;
  // while (rdy != 1) {
  //   rdy = readByteBitfield(config_registers[TEMP_RDY]);
  // }

  int32_t raw_val = 0;
  ret = getRawResult(&raw_val, registerBlocks[TEMP]);
  if (ret != DPS__SUCCEEDED) {
    return ret;
  }

  *result = calcTemp(raw_val);
  return DPS__SUCCEEDED;
}

int16_t startMeasureTempOnce() {
  // set device to temperature measuring mode
  return setOpMode(CMD_TEMP);
}

int16_t measurePressureOnce(uint16_t *result) {
  // start the measurement
  int16_t ret = startMeasurePressureOnce();
  if (ret != DPS__SUCCEEDED) {
    return ret;
  }

  uint8_t delay = calcBusyTime(m_prsOsr);
  for (int i = 0; i < delay; i++) {
    _delay_ms(1);
  }

  int16_t rdy = 0;
  while (rdy != 1) {
    rdy = readByteBitfield(config_registers[PRS_RDY]);
  }

  int32_t raw_val;
  ret = getRawResult(&raw_val, registerBlocks[PRS]);
  if (ret != DPS__SUCCEEDED) {
    return ret;
  }
  *result = calcPressure(raw_val);

  return DPS__SUCCEEDED;
}

int16_t startMeasurePressureOnce() {
  // set device to pressure measuring mode
  return setOpMode(CMD_PRS);
}

int16_t correctTemp(void) {
  writeByte(0x0E, 0xA5, false);
  writeByte(0x0F, 0x96, false);
  writeByte(0x62, 0x02, false);
  writeByte(0x0E, 0x00, false);
  writeByte(0x0F, 0x00, false);

  // perform a first temperature measurement (again)
  // the most recent temperature will be saved internally
  // and used for compensation when calculating pressure
  int16_t trash;
  measureTempOnce(&trash);

  return DPS__SUCCEEDED;
}
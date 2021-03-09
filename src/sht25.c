#include "stdbool.h"
#include "twi_master.h"
#include "util/delay.h"

#define SHT_I2C_ADDR 0x80       // Sensor I2C address
#define SHT_TRIG_TEMP_HOLD 0xE3 // Trigger Temp  with hold master
#define SHT_TRIG_RH_HOLD 0xE5   // Trigger RH    with hold master
#define SHT_TRIG_TEMP 0xF3      // Trigger Temp  with no hold master
#define SHT_TRIG_RH 0xF5        // Trigger RH    with no hold master
#define SHT_WRITE_REG 0xE6      // Write to user register
#define SHT_READ_REG 0xE7       // Read from user register
#define SHT_SOFT_RESET 0xFE     // Soft reset the sensor
#define SHT_SELF_HEATING 0x01   // Keep self heating
#define SHT_PREC_1214 0x00      // RH 12 T 14 - default
#define SHT_PREC_0812 0x01      // RH 8  T 10
#define SHT_PREC_1013 0x80      // RH 10 T 13
#define SHT_PREC_1111 0x81      // RH 10 T 13
#define SHT_WAIT_MS_TEMP 95     // Sensor Temp measure
#define SHT_WAIT_MS_RH 39       // Sensor RH measure
#define SHT_WAIT_MS_SET 15      // Sensor (re)set

bool softReset();
bool setPrecision(char precision);

void sht25_init() {
  _delay_ms(SHT_WAIT_MS_SET);
  softReset();
  setPrecision(SHT_PREC_1214);
}

int16_t sht25_read_temperature(void) {
  uint8_t rx[3] = {0xFF, 0xFF, 0xFF};

  if (!i2c_start(SHT_I2C_ADDR & ~0x01)) {
    i2c_stop();
    return 32767;
  }
  if (!i2c_write(SHT_TRIG_TEMP)) {
    i2c_stop();
    return 32767;
  }
  i2c_stop();

  _delay_ms(SHT_WAIT_MS_TEMP);

  if (!i2c_start(SHT_I2C_ADDR | 0x01)) {
    i2c_stop();
    return 32767;
  }
  rx[0] = i2c_read(false);
  rx[1] = i2c_read(false);
  rx[2] = i2c_read(true);
  i2c_stop();

  int32_t raw_val = ((rx[0] << 8) | rx[1]) & 0xFFFFFFFC;

  return (int16_t)(-4685 + 17572 * raw_val / 65536);
}

int16_t sht25_read_humidity(void) {
  uint8_t rx[3] = {0xFF, 0xFF, 0xFF};

  if (!i2c_start(SHT_I2C_ADDR & ~0x01)) {
    i2c_stop();
    return 32767;
  }
  if (!i2c_write(SHT_TRIG_RH)) {
    i2c_stop();
    return 32767;
  }
  i2c_stop();

  _delay_ms(SHT_WAIT_MS_RH);

  if (!i2c_start(SHT_I2C_ADDR | 0x01)) {
    i2c_stop();
    return 32767;
  }

  rx[0] = i2c_read(false);
  rx[1] = i2c_read(false);
  rx[2] = i2c_read(true);
  i2c_stop();

  int32_t raw_val = ((rx[0] << 8) | rx[1]) & 0xFFFC;
  return (int16_t)(-600 + 12500 * raw_val / 65536);
}

bool setPrecision(char precision) {
  if (!i2c_start(SHT_I2C_ADDR & ~0x01)) {
    i2c_stop();
    return false;
  }
  if (!i2c_write(SHT_WRITE_REG)) {
    i2c_stop();
    return false;
  }
  if (!i2c_write(precision)) {
    i2c_stop();
    return false;
  }
  i2c_stop();
  _delay_ms(SHT_WAIT_MS_SET);
  return true;
}

bool softReset() {
  if (!i2c_start(SHT_I2C_ADDR & ~0x01)) {
    i2c_stop();
    return false;
  }
  if (!i2c_write(SHT_SOFT_RESET)) {
    i2c_stop();
    return false;
  }

  i2c_stop();
  _delay_ms(SHT_WAIT_MS_SET);
  return true;
}

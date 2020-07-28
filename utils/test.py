#!/usr/bin/python

import minimalmodbus
import serial
from time import sleep

FUNC_READ_HOLDING_REG = 3
FUNC_READ_INPUT_REG = 4
FUNC_WRITE_INPUT_REG = 6

board = minimalmodbus.Instrument('/dev/ttyUSB0', slaveaddress=1)
# board.debug=True

print("FW version: " + str(hex(board.read_register(2, functioncode=FUNC_READ_INPUT_REG))))

print("GPIO 0 - SCL/SCK")
board.write_register(5, value=0x00000001, functioncode=FUNC_WRITE_INPUT_REG)
sleep(0.3)
print("GPIO 1 - MISO")
board.write_register(5, value=0x00000002, functioncode=FUNC_WRITE_INPUT_REG)
sleep(0.3)
print("GPIO 2 - MOSI/SDA")
board.write_register(5, value=0x00000004, functioncode=FUNC_WRITE_INPUT_REG)
sleep(0.3)
print("all gpio on")
board.write_register(5, value=0x00000007, functioncode=FUNC_WRITE_INPUT_REG)
sleep(0.3)
print("all gpio off")
board.write_register(5, value=0x00000000, functioncode=FUNC_WRITE_INPUT_REG)

while True:
  print("ADC7: " + str(board.read_register(0, functioncode=FUNC_READ_INPUT_REG)))
  print("ADC8: " + str(board.read_register(1, functioncode=FUNC_READ_INPUT_REG)))


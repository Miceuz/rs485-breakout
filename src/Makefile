# Name: Makefile
# Project: hid-mouse example
# Author: Christian Starkjohann
# Creation Date: 2008-04-07
# Tabsize: 4
# Copyright: (c) 2008 by OBJECTIVE DEVELOPMENT Software GmbH
# License: GNU GPL v2 (see License.txt), GNU GPL v3 or proprietary (CommercialLicense.txt)
# This Revision: $Id: Makefile 692 2008-11-07 15:07:40Z cs $

DEVICE  = attiny841
F_CPU   = 16000000	# in Hz
# 10101111
# 1 - CKDIV8 - no
# 1 - CKOUT - no
# 1 - 
# 0 - SUT
# 1111 - >8MHz
FUSE_L  = 0xEF

# 11011111
# 1 - rst disable - no
# 1 - debgwire - no
# 0 - SPI enabled - yes
# 1 - watchdog - off
# 1 - eesave - no
# 111 - BOD level - 2.7V
FUSE_H  = 0xDD

# 11101101
#111 - freq selection, default
#01 - BOD sleep mode: Sampled
#10 - BOD active mode: Enabled
#1 - Enables SPM instruction - no
FUSE_E  = 0xED
AVRDUDE = avrdude -c usbasp -P usb -p $(DEVICE) -e # edit this line for your programmer

CFLAGS  = -Iusbdrv -I. -DDEBUG_LEVEL=0  -Os -std=gnu99
CFLAGS += -ffunction-sections -fdata-sections -save-temps 
LDFLAGS += -Wl,--gc-sections,--relax

OBJECTS = main.o modbus.o measurement.o twi_master.o sht25.o dps310.o

COMPILE = avr-gcc -DF_CPU=$(F_CPU) $(CFLAGS) $(LDFLAGS) -mmcu=$(DEVICE)  

# symbolic targets:
help:
	@echo "This Makefile has no default rule. Use one of the following:"
	@echo "make hex ....... to build main.hex"
	@echo "make program ... to flash fuses and firmware"
	@echo "make fuse ...... to flash the fuses"
	@echo "make flash ..... to flash the firmware (use this on metaboard)"
	@echo "make clean ..... to delete objects and hex file"

hex: main.hex

program: flash fuse

# rule for programming fuse bits:
fuse:
	@[ "$(FUSE_H)" != "" -a "$(FUSE_L)" != "" ] || \
		{ echo "*** Edit Makefile and choose values for FUSE_L and FUSE_H!"; exit 1; }
	$(AVRDUDE) -U hfuse:w:$(FUSE_H):m -U lfuse:w:$(FUSE_L):m -U efuse:w:$(FUSE_E):m -B 20.96

# rule for uploading firmware:
flash: main.hex
	$(AVRDUDE) -U flash:w:main.hex:i -U eeprom:w:main.eep:i #-B 20.92 
sflash: main.hex
	$(AVRDUDE) -U flash:w:main.hex:i -B 20.92
# rule for deleting dependent files (those which can be built by Make):
clean:
	rm -f main.hex main.lst main.obj main.cof main.list main.map main.eep.hex main.elf *.o main.s *.i *.s

# Generic rule for compiling C files:
.c.o:
	$(COMPILE) -c $< -o $@

# Generic rule for assembling Assembler source files:
.S.o:
	$(COMPILE) -x assembler-with-cpp -c $< -o $@
# "-x assembler-with-cpp" should not be necessary since this is the default
# file type for the .S (with capital S) extension. However, upper case
# characters are not always preserved on Windows. To ensure WinAVR
# compatibility define the file type manually.

# Generic rule for compiling C to assembler, used for debugging only.
.c.s:
	$(COMPILE) -S $< -o $@

# file targets:

# Since we don't want to ship the driver multipe times, we copy it into this project:

main.elf: $(OBJECTS)	# usbdrv dependency only needed because we copy it
	$(COMPILE) -L /usr/lib/avr/lib/ldscripts/ -o main.elf $(OBJECTS)

main.hex: main.elf
	rm -f main.hex main.eep.hex
	avr-objcopy -j .text -j .data -O ihex main.elf main.hex
	avr-objcopy -j .eeprom --set-section-flags=.eeprom="alloc,load" --change-section-lma .eeprom=0 -O ihex main.elf main.eep
	avr-size main.hex

# debugging targets:

disasm:	main.elf
	avr-objdump -d main.elf

cpp:
	$(COMPILE) -E main.c

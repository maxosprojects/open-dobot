#!/bin/bash

# Replace with were Arduino is installed in your system.
AVR_DIR=/Applications/Arduino.app/Contents/Java/hardware/tools/avr
SERIAL_PORT=/dev/cu.usbmodem1421

$AVR_DIR/bin/avr-g++ -O -DF_CPU=16000000UL -mmcu=atmega2560 -c -o $1.o $1.cpp \
&& $AVR_DIR/bin/avr-g++ -mmcu=atmega2560 $1.o -o $1 \
&& $AVR_DIR/bin/avr-objcopy -O ihex -R .eeprom $1 $1.hex \
&& $AVR_DIR/bin/avrdude -C${AVR_DIR}/etc/avrdude.conf -patmega2560 -cwiring -P${SERIAL_PORT} -b115200 -D -Uflash:w:$1.hex:i

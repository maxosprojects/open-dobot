#!/bin/bash

# Replace with were Arduino is installed in your system.
AVR_DIR=/Applications/Arduino.app/Contents/Java/hardware/tools/avr
SERIAL_PORT=/dev/cu.usbmodem1421

SRC_MAIN=../src/main
SRC_MISC=$SRC_MAIN/misc
SRC_MPU=$SRC_MAIN/mpu6050
SRC_RAMPS=../src/ramps
BUILD=../build

$AVR_DIR/bin/avr-g++ -O -DF_CPU=16000000UL -DRAMPS -mmcu=atmega2560 -c -o $BUILD/dobot.o $SRC_MAIN/dobot.cpp \
&& $AVR_DIR/bin/avr-g++ -O -DF_CPU=16000000UL -DRAMPS -mmcu=atmega2560 -c -o $BUILD/calibrator.o $SRC_MISC/calibrator.cpp \
&& $AVR_DIR/bin/avr-g++ -O -DF_CPU=16000000UL -DRAMPS -mmcu=atmega2560 -c -o $BUILD/queue.o $SRC_MISC/queue.cpp \
&& $AVR_DIR/bin/avr-g++ -O -DF_CPU=16000000UL -DRAMPS -mmcu=atmega2560 -c -o $BUILD/ramps.o $SRC_RAMPS/ramps.cpp \
&& $AVR_DIR/bin/avr-g++ -O -DF_CPU=16000000UL -DRAMPS -mmcu=atmega2560 -c -o $BUILD/mpu6050.o $SRC_MPU/mpu6050.c \
&& $AVR_DIR/bin/avr-g++ -O -DF_CPU=16000000UL -DRAMPS -mmcu=atmega2560 -c -o $BUILD/twimastertimeout.o $SRC_MPU/twimastertimeout.c \
&& $AVR_DIR/bin/avr-g++ -mmcu=atmega2560 \
$BUILD/dobot.o \
$BUILD/mpu6050.o \
$BUILD/twimastertimeout.o \
$BUILD/ramps.o \
$BUILD/calibrator.o \
$BUILD/queue.o \
-o $BUILD/dobot \
&& $AVR_DIR/bin/avr-objcopy -O ihex -R .eeprom $BUILD/dobot $BUILD/dobot-ramps.hex \
&& $AVR_DIR/bin/avrdude -C${AVR_DIR}/etc/avrdude.conf -patmega2560 -cwiring -P${SERIAL_PORT} -b115200 -D -Uflash:w:$BUILD/dobot-ramps.hex:i

#!/bin/bash

# Replace with were Arduino is installed in your system.
AVR_DIR=/Applications/Arduino.app/Contents/Java/hardware/tools/avr

$AVR_DIR/bin/avr-g++ -O -DF_CPU=16000000UL -mmcu=atmega2560 -Wa,-adhlns=$1.lst -fverbose-asm -o $1.o $1.cpp

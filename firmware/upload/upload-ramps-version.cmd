@echo off
REM Replace with were Arduino is installed on your system.
SET AVR_DIR="C:\Program Files (x86)\Arduino\hardware\tools\avr"
REM Replace with serial port number on your system.
SET SERIAL_PORT=COM3

SET SRC_MAIN=..\src\main
SET SRC_MISC=%SRC_MAIN%\misc
SET SRC_MPU=%SRC_MAIN%\mpu6050
SET SRC_RAMPS=..\src\ramps
SET BUILD=..\build

%AVR_DIR%\bin\avr-g++ -DF_CPU=16000000UL -DRAMPS -mmcu=atmega2560 -c -g -O -std=gnu++11 -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -o %BUILD%\dobot.o %SRC_MAIN%\dobot.cpp ^
 && %AVR_DIR%\bin\avr-g++ -DF_CPU=16000000UL -DRAMPS -mmcu=atmega2560 -c -g -O -std=gnu++11 -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -o %BUILD%\calibrator.o %SRC_MISC%\calibrator.cpp ^
 && %AVR_DIR%\bin\avr-g++ -DF_CPU=16000000UL -DRAMPS -mmcu=atmega2560 -c -g -O -std=gnu++11 -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -o %BUILD%\queue.o %SRC_MISC%\queue.cpp ^
 && %AVR_DIR%\bin\avr-g++ -DF_CPU=16000000UL -DRAMPS -mmcu=atmega2560 -c -g -O -std=gnu++11 -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -o %BUILD%\ramps.o %SRC_RAMPS%\ramps.cpp ^
 && %AVR_DIR%\bin\avr-g++ -DF_CPU=16000000UL -DRAMPS -mmcu=atmega2560 -c -g -O -std=gnu++11 -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -o %BUILD%\mpu6050.o %SRC_MPU%\mpu6050.c ^
 && %AVR_DIR%\bin\avr-g++ -DF_CPU=16000000UL -DRAMPS -mmcu=atmega2560 -c -g -O -std=gnu++11 -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -o %BUILD%\twimastertimeout.o %SRC_MPU%\twimastertimeout.c ^
 && %AVR_DIR%\bin\avr-g++ -O -Wl,--gc-sections,--relax -mmcu=atmega2560 ^
 %BUILD%\dobot.o ^
 %BUILD%\mpu6050.o ^
 %BUILD%\twimastertimeout.o ^
 %BUILD%\ramps.o ^
 %BUILD%\calibrator.o ^
 %BUILD%\queue.o ^
 -o %BUILD%\dobot -lm ^
 && %AVR_DIR%\bin\avr-objcopy -O ihex -R .eeprom %BUILD%\dobot %BUILD%\dobot-ramps.hex ^
 && %AVR_DIR%\bin\avrdude -C%AVR_DIR%\etc\avrdude.conf -patmega2560 -cwiring -P%SERIAL_PORT% -b115200 -D -Uflash:w:%BUILD%\dobot-ramps.hex:i

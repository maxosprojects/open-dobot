/*
open-dobot firmware.

Find driver and SDK at https://github.com/maxosprojects/open-dobot

Author: maxosprojects (March 18 2016)
Additional Authors: <put your name here>

Version: 1.2.2

License: MIT
*/

#include "../main/dobot.h"
#include "../main/misc/calibrator.h"
#include "../main/misc/queue.h"

#define SPI_PORT PORTB
#define SPI_DDR  DDRB
#define SPI_MOSI PORTB2
#define SPI_MISO PORTB3
#define SPI_SCK PORTB1
#define SPI_SS PORTB0

#define FPGA_ENABLE_PORT PORTG
#define FPGA_ENABLE_DDR  DDRG
#define FPGA_ENABLE_PIN PORTG1
#define POWERON_PORT PINL
#define POWERON_PIN PORTL5
#define FPGA_COMMAND_PORT PORTL
#define FPGA_COMMAND_DDR DDRL
#define FPGA_COMMAND_PIN PORTL7
// INIT pin is normally low.
#define FPGA_COMMAND_ACCELS_INIT_PIN PORTL0
// SS pins are normally high.
#define FPGA_COMMAND_ACCEL_REAR_SS_PIN PORTL2
#define FPGA_COMMAND_ACCEL_FRONT_SS_PIN PORTL4

extern uint currGripper;
extern uint currToolRotation;

extern byte prevMotorDirections;
extern byte currMotorDirections;

extern CommandQueue cmdQueue;
extern Calibrator calibrator;

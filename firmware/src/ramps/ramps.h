/*
open-dobot firmware.

Find driver and SDK at https://github.com/maxosprojects/open-dobot

Author: maxosprojects (March 18 2016)
Additional Authors: <put your name here>

Version: 1.2.2

License: MIT
*/

/**
RAMPS version routines.
**/

#include <avr/interrupt.h>
#include "../main/dobot.h"
#include "../main/misc/calibrator.h"
#include "../main/misc/queue.h"

#define X_STEP_PIN         PORTF0
#define X_STEP_PORT        PORTF
#define X_STEP_DDR         DDRF
#define X_DIR_PIN          PORTF1
#define X_DIR_PORT         PORTF
#define X_DIR_DDR          DDRF
#define X_ENABLE_PIN       PORTD7
#define X_ENABLE_PORT      PORTD
#define X_ENABLE_DDR       DDRD

#define Y_STEP_PIN         PORTF6
#define Y_STEP_PORT        PORTF
#define Y_STEP_DDR         DDRF
#define Y_DIR_PIN          PORTF7
#define Y_DIR_PORT         PORTF
#define Y_DIR_DDR          DDRF
#define Y_ENABLE_PIN       PORTF2
#define Y_ENABLE_PORT      PORTF
#define Y_ENABLE_DDR       DDRF

// #define Z_STEP_PIN         PORTL3
// #define Z_STEP_PORT        PORTL
// #define Z_STEP_DDR         DDRL
// #define Z_DIR_PIN          PORTL1
// #define Z_ENABLE_PIN       PORTK0

// Use E0 instead of Z as it is not easy to connect dobot to Z.
// Left named as Z to avoif renaming.
#define Z_STEP_PIN         PORTA4
#define Z_STEP_PORT        PORTA
#define Z_STEP_DDR         DDRA
#define Z_DIR_PIN          PORTA6
#define Z_DIR_PORT         PORTA
#define Z_DIR_DDR          DDRA
#define Z_ENABLE_PIN       PORTA2
#define Z_ENABLE_PORT      PORTA
#define Z_ENABLE_DDR       DDRA

#define LASER_PIN PORTB4
#define LASER_PORT PORTB
#define LASER_DDR DDRB

#define PUMP_PIN PORTH5
#define PUMP_PORT PORTH
#define PUMP_DDR DDRH

#define VALVE_PIN PORTH6
#define VALVE_PORT PORTH
#define VALVE_DDR DDRH

#define TOOL_ROT_PIN PORTH3
#define TOOL_ROT_DDR DDRH
#define TOOL_ROT_PWM OCR4A

#define GRIPPER_PIN PORTE3
#define GRIPPER_DDR DDRE
#define GRIPPER_PWM OCR3A

// At 50kHz how many ticks pass between TIMER5_COMPA_vect ISR calls.
#define TICKS_PER_CALL 40
// Coefficient that is used in DobotDriver to calculate stepping
// period.
#define STEPS_COEFF 20000
// How long delay does the TIMER1_COMPA_vect ISR introduce.
// This is to be acoounted for make corresponding adjustments.
#define ISR_DELAY 80

extern CommandQueue cmdQueue;
extern Calibrator calibrator;

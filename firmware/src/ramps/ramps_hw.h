/*
open-dobot firmware.

Find driver and SDK at https://github.com/maxosprojects/open-dobot

Author: maxosprojects (March 18 2016)
Additional Authors: <put your name here>

Version: 1.2.2

License: MIT
*/

/**
This file is left as an example of how would one do HW stepping on RAMPS
using timer.
Unfortunaly, this approach requires a separate 16-bit timer for every stepper.
In order to save timers for other purposes ramps.h implements stepping using
one timer with interrupts. The approach is not as accurate as HW stepping,
but for dobot it is good enough as it does exactly the requested number of
steps. The jitter is negligible.
**/

#include <avr/interrupt.h>

#define STEP_PIN_X PORTH3
#define STEP_PIN_Y PORTH4
#define STEP_PIN_Z PORTH5
#define STEP_PORT PORTH
#define STEP_PORTIN PINH
#define STEP_DDR DDRH

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

volatile byte executed = 0;
volatile uint ticksLeftX = 0;
volatile uint ticksLeftY = 0;
volatile uint ticksLeftZ = 0;

void setupBoard() {
  X_DIR_DDR |= (1<< X_DIR_PIN);
  Y_DIR_DDR |= (1<< Y_DIR_PIN);
  Z_DIR_DDR |= (1<< Z_DIR_PIN);

  X_ENABLE_DDR |= (1<< X_ENABLE_PIN);
  Y_ENABLE_DDR |= (1<< Y_ENABLE_PIN);
  Z_ENABLE_DDR |= (1<< Z_ENABLE_PIN);

  // Enable pin is negative, so 0 means motors are enabled.
  X_ENABLE_PORT &= ~(1<< X_ENABLE_PIN);
  Y_ENABLE_PORT &= ~(1<< Y_ENABLE_PIN);
  Z_ENABLE_PORT &= ~(1<< Z_ENABLE_PIN);

  STEP_DDR |= (1<< STEP_PIN_X);
  // STEP_DDR |= (1<< STEP_PIN_Y);
  // STEP_DDR |= (1<< STEP_PIN_Z);

  // Turn on Timer1 with 1/256 prescaler (62.5kHz) for command interrupts.
  TCCR1B |= (1 << CS12);
  // Set compare match register on Timer1 to desired timer count to form 50Hz
  OCR1A = 1250;
  // Turn on CTC mode.
  TCCR1B |= (1 << WGM12);
  // Enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  /**
   * Set up pin toggling driven by timers in hardware.
   */
  // Configure Timer4 to CTC mode.
  TCCR4B |= (1 << WGM42);
  // Start Timer4 at Fcpu/8 (2MHz)
  TCCR4B |= (1 << CS41);

  // Inable interrupts for to execute commands at 50Hz.
  sei();
}

// Timer1 compare match Interrupt Service Routine.
// Executes commands at 50Hz.
ISR(TIMER1_COMPA_vect)
{
  // If it happened that a timer driven pin is HIGH at the
  // end of previous command then turn it LOW by toggling
  // the pin via timer's FOC bit. Changing pin state by accessing
  // the port directly has no effect when timer drives the pin.
  if (STEP_PORTIN & (1 << STEP_PIN_X)) {
    TCCR4C |= (1 << FOC4A);
  }
  // if (STEP_PORTIN & (1 << STEP_PIN_Y)) {
  //   TCCR4C |= (1 << FOC4A);
  // }
  // if (STEP_PORTIN & (1 << STEP_PIN_Z)) {
  //   TCCR4C |= (1 << FOC4A);
  // }
  // Disconnect timer channels A. B and C from the pins to avoid
  // pin state changes while loading compare register.
  TCCR4A &= ~(1 << COM4A0);
  // TCCR4A &= ~(1 << COM4B0);
  // TCCR4A &= ~(1 << COM4C0);

  // Calibration routine.
  if (calibrator.isRunning()) {
    if (calibrator.isHit()) {
      if (calibrator.isBacking()) {
        // writeSpi(calibrator.getBackoffCommand());
      } else {
        calibrator.startBacking();
        // writeSpi(calibrator.getBackoffCommand());
      }
    } else if (calibrator.isBacking()) {
      calibrator.stop();
      // writeSpiRest();
    } else {
      // writeSpi(calibrator.getForwardCommand());
    }
  // If nothing left in the queue then send STOP.
  } else if (cmdQueue.isEmpty()) {
      // Do nothing, just leave timer pin disabled.
  // Some command is waiting to be processed.
  } else {
    Command* command = cmdQueue.peekTail();
    // If movement command then execute.
    if (command->type == Move) {
      // Remove the command from the queue.
      cmdQueue.popTail();
      // Execute. Take lower two bytes and feed them to the Output Compare Register.
      // This sets on which timer count the pin will be toggled.
      // The counter is incremented at 2MHz. See setupBoard()
      OCR4A = (uint) command->j1;
      // OCR4B = (uint) command->j2;
      // OCR4C = (uint) command->j3;
      // Reset timer counter.
      TCNT4 = 0;
      // Re-enable timer pins.
      TCCR4A |= (1 << COM4A0);
      // TCCR4A |= (1 << COM4B0);
      // TCCR4A |= (1 << COM4C0);
    }
  }
  executed = 1;
}

int main() {
  setup();

  executed = 1;

  byte i = 0;
  byte data;
  while (1) {
    if (executed) {
      executed = 0;
      // After a command has been sent to FPGA or directly executed peek at the
      // queue to see if there is another non-movement command is waiting next
      // and execute it too. Do until a movement command is next or no commands
      // left in the queue.
      while (!cmdQueue.isEmpty()) {
        Command* command = cmdQueue.peekTail();
        if (command->type == Move) {
          break;
        } else {
          serialRead();
          cmdQueue.popTail();
          implementationFunctions[command->type]();
        }
      }
      processSerialBuffer();
    } else {
      serialRead();
    }
  }
  return 0;
}


/**
RAMPS version routines.
**/

#include <avr/interrupt.h>

#define X_STEP_PIN PORTF0
#define X_STEP_PORT PORTF
#define X_STEP_DDR DDRF
#define X_STEP_PIN1 PORTF1
#define X_STEP_PIN2 PORTF2
#define CLOCK_PIN PORTH3
#define CLOCK_PORT PORTH
#define CLOCK_PORTIN PINH
#define CLOCK_DDR DDRH

volatile byte executed = 0;

void setupBoard() {
  X_STEP_DDR |= (1 << X_STEP_PIN);
  X_STEP_DDR |= (1 << X_STEP_PIN1);
  X_STEP_DDR |= (1  << X_STEP_PIN2);

  CLOCK_DDR |= (1<< CLOCK_PIN);

  // Turn on timer with 1/8 prescaler (2MHz).
  // TCCR1B |= _BV(CS11);
  // Turn on timer with 1/256 prescaler (62.5kHz).
  TCCR1B |= (1 << CS12);
  // Enable timer overflow interrupt.
  // TIMSK1 |= _BV(TOIE1);
  // Set compare match register to desired timer count to form ~7kHz
  // OCR1A = 143;
  // Set compare match register to desired timer count to form 50Hz
  OCR1A = 1250;
  // Turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);




  // --== HW clock toggle ==--

  // Configure timer 4 for CTC mode
  TCCR4B |= (1 << WGM42);
  // Enable timer 1 Compare Output channel A in toggle mode
  TCCR4A |= (1 << COM4A0);

  // Set CTC compare value to 1Hz at 1MHz AVR clock, with a prescaler of 64
  // OCR4A = 15624;
  // Get 50Hz for test
  OCR4A = 20000;

  // Start timer at Fcpu/64
  // TCCR4B |= ((1 << CS40) | (1 << CS41));
  // Start timer at Fcpu/8 (2MHz)
  TCCR4B |= (1 << CS41);

  sei();
}

ISR(TIMER1_OVF_vect) {
  X_STEP_PORT ^= (1 << X_STEP_PIN);
}

ISR(TIMER1_COMPA_vect)
{
  X_STEP_PORT ^= (1 << X_STEP_PIN);

  if (CLOCK_PORTIN & (1 << CLOCK_PIN)) {
    TCCR4C |= (1 << FOC4A);
  }
  TCCR4A ^= (1 << COM4A0);
  TCNT4 = 0;
  TCCR4A ^= (1 << COM4A0);

  // CLOCK_PORT |= (1<< CLOCK_PIN);
  // TCCR4A ^= (1 << COM4A0);

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
      // writeSpiRest();
  // Some command is waiting to be processed.
  } else {
    Command* command = cmdQueue.peekTail();
    // If movement command then execute.
    if (command->type == Move) {
      cmdQueue.popTail();
      // writeSpi(command);
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

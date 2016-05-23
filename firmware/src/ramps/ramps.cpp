
/**
RAMPS version routines.
**/

#include "ramps.h"
#include "../main/inlines.h"
#include "../main/mpu6050/mpu6050.h"

volatile byte executed = 0;
volatile byte stepsX = 1;
volatile byte stepsY = 1;
volatile byte stepsZ = 1;
volatile uint ticksX;
volatile uint ticksY;
volatile uint ticksZ;
volatile int ticksLeftX;
volatile int ticksLeftY;
volatile int ticksLeftZ;

SwitchPort calibrationPins[] = {
};

byte calibrationPinsNumber() {
  return sizeof(calibrationPins);
}

// CMD: Returns 1 for RAMPS.
byte cmdBoardVersion() {
  // Check if not enough bytes yet.
  if (cmdInBuffIndex < 3) {
    return 1;
  }
  cmdInBuffIndex = 0;
  if (!checkCrc(cmd, 1)) {
    return 2;
  }
  // Return 1 for RAMPS.
  cmd[0] = 1;
  write1(cmd);
  return 3;
}

int accelRead(byte pin) {
  long result = 0;
  int ax, ay, az;
  byte i = 20;

  while (i--) {
    mpu6050_getRawAccels(&ax, &ay, &az);
    result += ax;
  }
  return result / 20;
}

void switchToAccelReportMode() {
  while (1) {
    accelRear = accelRead(0);
    processSerialBuffer();
  }
}

void setupBoard() {
  // Enable internal pullup on accelerometer reporting mode pin.
  ACCEL_SWITCH_PORT |= (1<<ACCEL_SWITCH_PIN);

  //init mpu6050
  mpu6050_init();
  _delay_ms(50);

  if (! (ACCEL_SWITCH_PORTIN & (1<<ACCEL_SWITCH_PIN))) {
    switchToAccelReportMode();
  }

  // accelRear = accelRead(0);

  X_STEP_DDR |= (1<< X_STEP_PIN);
  Y_STEP_DDR |= (1<< Y_STEP_PIN);
  Z_STEP_DDR |= (1<< Z_STEP_PIN);

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

  /**
   * Set up TIMER1_COMPA_vect ISR to execute commands.
   */
  // Turn on Timer1 with 1/256 prescaler (62.5kHz) for command interrupts.
  TCCR1B |= (1 << CS12);
  // Set compare match register on Timer1 to form 50Hz
  OCR1A = 1250;
  // Turn on CTC mode.
  TCCR1B |= (1 << WGM12);
  // Enable timer compare interrupt.
  TIMSK1 |= (1 << OCIE1A);

  /**
   * Set up TIMER5_COMPA_vect ISR to toggle pins.
   */
  // Configure Timer5 to CTC mode.
  TCCR5B |= (1 << WGM52);
  // Start Timer4 at Fcpu/1 (16MHz)
  TCCR5B |= (1 << CS50);
  // Set compare match register on Timer1 to form 50kHz
  OCR5A = 320;
  // Enable timer compare interrupt (for stepping).
  TIMSK5 |= (1 << OCIE5A);

  // Inable interrupts for to execute commands at 50Hz.
  sei();
}

// Timer1 compare match Interrupt Service Routine.
// Executes commands at 50Hz.
ISR(TIMER1_COMPA_vect)
{
  // Reset pins.
  X_STEP_PORT &= ~(1 << X_STEP_PIN);
  Y_STEP_PORT &= ~(1 << Y_STEP_PIN);
  Z_STEP_PORT &= ~(1 << Z_STEP_PIN);

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
    stepsX = 0;
    stepsY = 0;
    stepsZ = 0;
  // Some command is waiting to be processed.
  } else {
    Command* command = cmdQueue.peekTail();
    // If movement command then execute.
    if (command->type == Move) {
      // Remove the command from the queue.
      cmdQueue.popTail();
      // Execute.
      // Set current ticks. The rest is taken care of by TIMER5_COMPA_vect.
      ticksX = (uint) command->j1;
      ticksY = (uint) command->j2;
      ticksZ = (uint) command->j3;
      if (ticksX == 0) {
        stepsX = 0;
      } else {
        X_DIR_PORT &= ~(1 << X_DIR_PIN);
        X_DIR_PORT |= ((command->control & 0x01) << X_DIR_PIN);
        ticksLeftX = ticksX - ISR_DELAY;
        stepsX = STEPS_COEFF / ticksX;
      }
      if (ticksY == 0) {
        stepsY = 0;
      } else {
        Y_DIR_PORT &= ~(1 << Y_DIR_PIN);
        Y_DIR_PORT |= (((command->control >> 1) & 0x01) << Y_DIR_PIN);
        ticksLeftY = ticksY - ISR_DELAY;
        stepsY = STEPS_COEFF / ticksY;
      }
      if (ticksZ == 0) {
        stepsZ = 1;
      } else {
        Z_DIR_PORT &= ~(1 << Z_DIR_PIN);
        Z_DIR_PORT |= (((command->control >> 2) & 0x01) << Z_DIR_PIN);
        ticksLeftZ = ticksZ - ISR_DELAY;
        stepsZ = STEPS_COEFF / ticksZ;
      }
    }
  }
  executed = 1;
}

// Timer5 compare match Interrupt Service Routine.
// Toggles pins at maximum 20kHz.
ISR(TIMER5_COMPA_vect)
{
  if (stepsX) {
    ticksLeftX -= TICKS_PER_CALL;
    if (ticksLeftX < 0) {
      X_STEP_PORT ^= (1 << X_STEP_PIN);
      if (X_STEP_PORT & (1 << X_STEP_PIN)) {
        stepsX--;
      }
      ticksLeftX += ticksX;
    }
  }
  if (stepsY) {
    ticksLeftY -= TICKS_PER_CALL;
    if (ticksLeftY < 0) {
      Y_STEP_PORT ^= (1 << Y_STEP_PIN);
      if (Y_STEP_PORT & (1 << Y_STEP_PIN)) {
        stepsY--;
      }
      ticksLeftY += ticksY;
    }
  }
  if (stepsZ) {
    ticksLeftZ -= TICKS_PER_CALL;
    if (ticksLeftZ < 0) {
      Z_STEP_PORT ^= (1 << Z_STEP_PIN);
      if (Z_STEP_PORT & (1 << Z_STEP_PIN)) {
        stepsZ--;
      }
      ticksLeftZ += ticksZ;
    }
  }
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

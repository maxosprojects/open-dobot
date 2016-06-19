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
  {
    // D16 - reference 0
    &PINH,
    PH1,
    &DDRH
  },
  {
    // D17 - reference 1
    &PINH,
    PH0,
    &DDRH
  },
  {
    // D25 - reference 2
    &PINA,
    PA3,
    &DDRA
  },
  {
    // D27 - reference 3
    &PINA,
    PA5,
    &DDRA
  },
  {
    // D29 - reference 4
    &PINA,
    PA7,
    &DDRA
  },
  {
    // D31 - reference 5
    &PINC,
    PC6,
    &DDRC
  },
  {
    // D33 - reference 6
    &PINC,
    PC4,
    &DDRC
  },
  {
    // D35 - reference 7
    &PINC,
    PC2,
    &DDRC
  },
  {
    // D37 - reference 8
    &PINC,
    PC0,
    &DDRC
  },
  {
    // D39 - reference 9
    &PING,
    PG2,
    &DDRG
  },
  {
    // D41 - reference 10
    &PING,
    PG0,
    &DDRG
  },
  {
    // D43 - reference 11
    &PINL,
    PL6,
    &DDRL
  },
  {
    // D45 - reference 12
    &PINL,
    PL4,
    &DDRL
  },
  {
    // D47 - reference 13
    &PINL,
    PL2,
    &DDRL
  },
  {
    // D32 - reference 14
    &PINC,
    PC5,
    &DDRC
  }
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

void laserOn() {
  LASER_PORT |= 1<<LASER_PIN;
}

void laserOff() {
  LASER_PORT &= ~(1<<LASER_PIN);
}

void pumpOn() {
  PUMP_PORT |= 1<<PUMP_PIN;
}

void pumpOff() {
  PUMP_PORT &= ~(1<<PUMP_PIN);
}

void valveOn() {
  VALVE_PORT |= 1<<VALVE_PIN;
}

void valveOff() {
  VALVE_PORT &= ~(1<<VALVE_PIN);
}

void setupAccels() {
  // The following initializes I2C and MPU-6050 units.
  // The approach seems to be working reliably when there is no interference from motors.
  // See wiki on how to suppress interference.
  byte i = 5;
  while (i--) {
    mpu6050_deinit();
    _delay_ms(5);
    mpu6050_init(MPU6050_ADDR0);
    _delay_ms(5);
    mpu6050_init(MPU6050_ADDR1);
    _delay_ms(5);
    if (mpu6050_testConnection(MPU6050_ADDR0) && mpu6050_testConnection(MPU6050_ADDR1)) {
      break;
    }
  }
}

void accelRead(byte unit, int *x, int *y, int *z) {
  int ax, ay, az;
  byte ret = 1;
  while (ret) {
    ret = mpu6050_getRawAccels(unit, &ax, &ay, &az);
    if (ret) {
      setupAccels();
    }
  }
  *x = ax;
  *y = ay;
  *z = az;
}

void updateAccels() {
  accelRead(MPU6050_ADDR0, &accelRearX, &accelRearY, &accelRearZ);
  accelRead(MPU6050_ADDR1, &accelFrontX, &accelFrontY, &accelFrontZ);
}

inline void setupPwm() {
  // Setup Gripper servo.
  // Set scaler to 1/8 FCPU_FREQ.
  TCCR3B |= (1<< CS31);
  // Set mode 14 (ICR3 defines TOP).
  TCCR3A |= (1<< WGM31);
  TCCR3B |= (1<< WGM32) | (1<< WGM33);
  // Set TOP to 50Hz at 1/8 FCPU_FREQ.
  ICR3 = 40000;
  // Fast PWM. Clear OCnA/OCnB/OCnC on compare match, clear OCnA/OCnB/OCnC at BOTTOM (inverting mode).
  TCCR3A |= (1<< COM3A1);
  // Set initial gripper state to 480 (480 * 2 + 2000).
  GRIPPER_PWM = 2960;
  // Enable pin.
  GRIPPER_DDR |= (1<< GRIPPER_PIN);

  // Setup ToolRotation servo.
  // Set scaler to 1/8 FCPU_FREQ.
  TCCR4B |= (1<< CS41);
  // Set mode 14 (ICR3 defines TOP).
  TCCR4A |= (1<< WGM41);
  TCCR4B |= (1<< WGM42) | (1<< WGM43);
  // Set TOP to 50Hz at 1/8 FCPU_FREQ.
  ICR4 = 40000;
  // Fast PWM. Clear OCnA/OCnB/OCnC on compare match, clear OCnA/OCnB/OCnC at BOTTOM (inverting mode).
  TCCR4A |= (1<< COM4A1);
  // Set initial tool state to 500 (500 * 2 + 2000).
  TOOL_ROT_PWM = 3000;
  // Enable pin.
  TOOL_ROT_DDR |= (1<< TOOL_ROT_PIN);
}

void setupBoard() {
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

  LASER_DDR |= (1<< LASER_PIN);
  PUMP_DDR |= (1<< PUMP_PIN);
  VALVE_DDR |= (1<< VALVE_PIN);

  setupPwm();

  setupAccels();

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
  // Start Timer5 at Fcpu/1 (16MHz)
  TCCR5B |= (1 << CS50);
  // Set compare match register on Timer1 to form 50kHz
  OCR5A = 320;
  // Enable timer compare interrupt (for stepping).
  TIMSK5 |= (1 << OCIE5A);

  // Inable interrupts to execute timer ISRs.
  sei();
}

inline void move(Command* command) {
  byte mask;

  // Set current ticks. The rest is taken care of by TIMER5_COMPA_vect.
  ticksX = (uint) command->j1;
  ticksY = (uint) command->j2;
  ticksZ = (uint) command->j3;

  if (ticksX == 0) {
    stepsX = 0;
  } else {
    mask = ((command->control & 0x01) << X_DIR_PIN);
    X_DIR_PORT = (X_DIR_PORT & ~(1 << X_DIR_PIN)) | mask;
    ticksLeftX = ticksX - ISR_DELAY;
    stepsX = STEPS_COEFF / ticksX;
    if (mask) {
      motorPositionBase += stepsX;
    } else {
      motorPositionBase -= stepsX;
    }
  }
  if (ticksY == 0) {
    stepsY = 0;
  } else {
    mask = (((command->control >> 1) & 0x01) << Y_DIR_PIN);
    Y_DIR_PORT = (Y_DIR_PORT & ~(1 << Y_DIR_PIN)) | mask;
    ticksLeftY = ticksY - ISR_DELAY;
    stepsY = STEPS_COEFF / ticksY;
    if (mask) {
      motorPositionRear += stepsY;
    } else {
      motorPositionRear -= stepsY;
    }
  }
  if (ticksZ == 0) {
    stepsZ = 0;
  } else {
    mask = (((command->control >> 2) & 0x01) << Z_DIR_PIN);
    Z_DIR_PORT = (Z_DIR_PORT & ~(1 << Z_DIR_PIN)) | mask;
    ticksLeftZ = ticksZ - ISR_DELAY;
    stepsZ = STEPS_COEFF / ticksZ;
    if (mask) {
      motorPositionFore -= stepsZ;
    } else {
      motorPositionFore += stepsZ;
    }
  }
  GRIPPER_PWM = command->servoGrab;
  TOOL_ROT_PWM = command->servoRot;
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
      if (!calibrator.isBacking()) {
        calibrator.startBacking();
      }
      move(calibrator.getBackoffCommand());
    } else if (calibrator.isBacking()) {
      calibrator.stop();
      stepsX = 0;
      stepsY = 0;
      stepsZ = 0;
    } else {
      move(calibrator.getForwardCommand());
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
      move(command);
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
      // After 50Hz has been executed peek at the queue to see if there is another
      // non-movement command is waiting next and execute it. Do until a
      // movement command is next or no commands left in the queue.
      while (1) {
        if (cmdQueue.isEmpty()) {
          // A safety net - the laser should not be left on when there are no commands
          // in the queue. This would be an indication that the user forgot to turn
          // it off or something happened with the comms or with the program.
          laserOff();
          break;
        }
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

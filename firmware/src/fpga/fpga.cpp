/*
open-dobot firmware.

Find driver and SDK at https://github.com/maxosprojects/open-dobot

Author: maxosprojects (March 18 2016)
Additional Authors: <put your name here>

Version: 1.2.2

License: MIT
*/

#include "fpga.h"
#include "../main/inlines.h"

// Sequence to set arm to rest.
volatile byte sequenceRest[19] = {
0xa5, // start signature
0x2,  // motor1 block start (middle driver - Stepper_Rot)
0x42,
0xf0,
0x0,
0x2,  // motor2 block start (left driver - Stepper_R)
0x42,
0xf0,
0x0,
0x2,  // motor3 block start (right driver - Stepper_L)
0x42,
0xf0,
0x0,
0xe0, // motors direction ([bit7-bit0]: bit7 - motor1, bit6 - motor2, bit5 - motor3)
0x2f, // servoGrab
0x80,
0x2f, // servoRot
0x80,
0x5a // end signature
};

/*
 * Refer to open-dobot/docs/images/arduino.jpg
 * 
 * Communication and control pins in use are:
 * D0-1 (USB serial), D12 (laser), D18-19 (bluetooth serial),
 * D40-53 (FPGA control, D48 - bluetooth enable, D41 - pump, D43 - valve)
 *
 * Unused pins:
 * D59-D61 (unknown, but soldered), D20-21 (I2C)
 * 
 * Some of the unused pins are not soldered through to the FPGA board
 * and can be used here:
 * D54-58, D62-69, D22-39
 *
 * This is the list of pins unused by the top board (FPGA board).
 * !!!!! BEFORE CHANGING ANY OF THESE:  !!!!!!
 * 1. Check you're not targeting USART pins
 * 2. Check you're not targeting any communication and control pins
 */
SwitchPort calibrationPins[] = {
  {
    // D3 - reference 0
    &PINE,
    PE5,
    &DDRE
  },
  {
    // D4 - reference 1
    &PING,
    PG5,
    &DDRG
  },
  {
    // D5 - reference 2
    &PINE,
    PE3,
    &DDRE
  },
  {
    // D6 - reference 3
    &PINH,
    PH3,
    &DDRH
  },
  {
    // D7 - reference 4
    &PINH,
    PH4,
    &DDRH
  },
  {
    // D8 - reference 5
    &PINH,
    PH5,
    &DDRH
  },
  {
    // D9 - reference 6
    &PINH,
    PH6,
    &DDRH
  },
  {
    // D10 - reference 7
    &PINB,
    PB4,
    &DDRB
  }
};

inline void writeSpiByte(byte data) {
  byte junk;

  loop_until_bit_is_set(SPSR, SPIF);
  junk = SPDR;
  SPDR = data;
  serialRead();
}

void writeSpi(Command* cmd) {
  byte* data = (byte*)cmd;

  FPGA_COMMAND_PORT |= (1<<FPGA_COMMAND_PIN);
  SPDR = sequenceRest[0];

  for (byte i = 0; i < 17; i++) {
    writeSpiByte(data[i]);
  }
  // writeSpiByte(sequenceRest[14]);
  // writeSpiByte(sequenceRest[15]);
  // writeSpiByte(sequenceRest[16]);
  // writeSpiByte(sequenceRest[17]);
  writeSpiByte(sequenceRest[18]);
  loop_until_bit_is_set(SPSR, SPIF);
  FPGA_COMMAND_PORT &= ~(1<<FPGA_COMMAND_PIN);
  prevMotorDirections = currMotorDirections;
  currMotorDirections = data[12];
}

void writeSpiRest() {
  FPGA_COMMAND_PORT |= (1<<FPGA_COMMAND_PIN);
  SPDR = sequenceRest[0];

  for (byte i = 1; i < 14; i++) {
    writeSpiByte(sequenceRest[i]);
  }
  writeSpiByte((currGripper >> 8) & 0xFF) ;
  writeSpiByte(currGripper & 0xFF) ;
  writeSpiByte((currToolRotation >> 8) & 0xFF) ;
  writeSpiByte(currToolRotation & 0xFF) ;

  writeSpiByte(sequenceRest[18]);
  loop_until_bit_is_set(SPSR, SPIF);
  FPGA_COMMAND_PORT &= ~(1<<FPGA_COMMAND_PIN);
}

inline byte readSpiByte() {
  loop_until_bit_is_set(SPSR, SPIF);
  return SPDR;
}

inline uint readSpiWord() {
  uint data;
  loop_until_bit_is_set(SPSR, SPIF);
  data = SPDR;
  serialRead();
  loop_until_bit_is_set(SPSR, SPIF);
  data |= (SPDR << 8);
  serialRead();
  return data;
}

// Reads a word of two bytes per joint.
// Each word is the number of steps joint just finished traveling (issued a command before).
inline void readSpi() {
  uint data;

  data = SPSR;
  data = SPDR;

  data = (readSpiWord() + 1) / 2;
  if (GET_MOTOR_DIRECTION(0)) {
    motorPositionBase += data;
  } else {
    motorPositionBase -= data;
  }

  data = (readSpiWord() + 1) / 2;
  if (GET_MOTOR_DIRECTION(1)) {
    motorPositionRear += data;
  } else {
    motorPositionRear -= data;
  }

  data = (readSpiWord() + 1) / 2;
  if (GET_MOTOR_DIRECTION(2)) {
    motorPositionFore -= data;
  } else {
    motorPositionFore += data;
  }

  data = readSpiByte();
}

byte calibrationPinsNumber() {
  return sizeof(calibrationPins);
}

int accelRead(byte pin) {
  byte junk;
  int result = 0;
  uint data;
  _delay_us(1);
  // // Clear SPIF bit.
  junk = SPSR;
  junk = SPDR;
  for (byte i = 0; i < 17; i++) {
    serialRead();
    _delay_us(770);
    serialRead();
    FPGA_COMMAND_PORT &= ~(1<<pin);
    SPDR = 0x10;
    serialRead();
    loop_until_bit_is_set(SPSR, SPIF);
    SPDR = 0x00;
    serialRead();
    loop_until_bit_is_set(SPSR, SPIF);
    data = SPDR << 8;
    SPDR = 0x00;
    serialRead();
    loop_until_bit_is_set(SPSR, SPIF);
    data |= SPDR;
    serialRead();
    // There are only 11 bits accelerometer returns,
    // so take advantage by shifting to prevent sum overflow.
    result += (data >> 5);
    FPGA_COMMAND_PORT |= (1<<pin);
  }
  data = result / 17;
  // Average
  return result / 17;
}

// Switches controller to accelerometers report mode.
byte switchToAccelReportMode() {
  /* Apparently there is a problem with the way dobot was designed.
   * It is not possible to switch back from SPI Slave to Master.
   * So this code is left in case a proper solution comes up.
   */
  // serialWrite(1);
  // if (checkCrc(cmd, 1)) {
  //   resetCrc();
  //   cmd[0] = 1;
  //   write1(cmd);

    accelReportMode = 1;

    // Disable FPGA.
    FPGA_ENABLE_PORT &= ~(1<<FPGA_ENABLE_PIN);

    // Enable accelerometers reading.
    FPGA_COMMAND_PORT |= (1<<FPGA_COMMAND_ACCELS_INIT_PIN);

    _delay_us(35);
    serialRead();

    // Set SPI as Master.
    SPI_DDR = (1<<SPI_MOSI)|(1<<SPI_SCK)|(1<<SPI_SS);
    SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);

    _delay_us(35);
    serialRead();

    // Never return from this function. Only update accelerometers
    // and return their values to the driver.
    // Process next waiting commands in between or else they will time out.
    while (1) {
      accelRearX = accelRead(FPGA_COMMAND_ACCEL_REAR_SS_PIN);
      processSerialBuffer();
      accelFrontX = accelRead(FPGA_COMMAND_ACCEL_FRONT_SS_PIN);
      processSerialBuffer();
    }
  // }
  return 1;
}

// CMD: Returns 0 for FPGA.
byte cmdBoardVersion() {
  // Check if not enough bytes yet.
  if (cmdInBuffIndex < 3) {
    return 1;
  }
  cmdInBuffIndex = 0;
  if (!checkCrc(cmd, 1)) {
    return 2;
  }
  // Return 0 for FPGA.
  cmd[0] = 0;
  write1(cmd);
  return 3;
}

void laserOn() {
  PORTB |= 1<<PB6;
}

void laserOff() {
  PORTB &= ~(1<<PB6);
}

void pumpOn() {
  PORTG |= 1<<PG0;
}

void pumpOff() {
  PORTG &= ~(1<<PG0);
}

void valveOn() {
  PORTL |= 1<<PL6;
}

void valveOff() {
  PORTL &= ~(1<<PL6);
}

void updateAccels() {
  // Do nothing. It is not possible to read original accelerometers on FPGA board after
  // Arduino finished booting.
}

void setupBoard() {
  //---=== Power-on sequence ===---
  // 1. FPGA_ENABLE_PIN = LOW, FPGA_COMMAND_PIN = LOW
  // 2. Check if FPGA_POWERON_PIN == HIGH. If not, then switch to accelerometer reporting mode.
  // 3. Arduino initializes - delay 900ms. Don't need to do anything.
  // 4. FPGA_COMMAND_ACCELS_INIT_PIN = HIGH
  // 5. Delay 35us
  // 6. Set SPI as Master
  // 7. Delay 35us
  // 8. Read accelerometers
  // 9. FPGA_COMMAND_ACCELS_INIT_PIN = LOW
  // 10. Delay 200ms
  // 11. Set SPI as Slave
  // 12. Enable FPGA: FPGA_ENABLE_PIN = HIGH

  //---=== Accelerometer reading sequence ===---
  // 1. Delay 800ns
  // 2. Loop 17 times:
  // 3.1. Delay 770us for accelerometer to prepare data
  // 3.2. FPGA_COMMAND_ACCEL_REAR_SS_PIN = LOW
  // 3.3. Send "RDAX / 00010000 / Read X-channel acceleration through SPI" command = 0x10
  // 3.4. Read two bytes
  // 3.5. FPGA_COMMAND_ACCEL_REAR_SS_PIN = LOW
  // 4. Average the result
  // 5. Repeat 1-3 for another accelerometer - FPGA_COMMAND_ACCEL_FRONT_SS_PIN
  
  // Power-on step 1
  FPGA_ENABLE_DDR = (1<<FPGA_ENABLE_PIN);
  FPGA_COMMAND_DDR = (1<<FPGA_COMMAND_PIN) | (1<<FPGA_COMMAND_ACCELS_INIT_PIN)
                  | (1<<FPGA_COMMAND_ACCEL_REAR_SS_PIN) | (1<<FPGA_COMMAND_ACCEL_FRONT_SS_PIN);
  FPGA_COMMAND_PORT = (1<<FPGA_COMMAND_ACCEL_REAR_SS_PIN) | (1<<FPGA_COMMAND_ACCEL_FRONT_SS_PIN);

  // Power-on step 2
  if (! (POWERON_PORT & (1<<POWERON_PIN))) {
    switchToAccelReportMode();
  }

  // Power-on step 3
  // Do nothing.

  // Power-on step 4
  FPGA_COMMAND_PORT |= (1<<FPGA_COMMAND_ACCELS_INIT_PIN);

  // Power-on step 5
  _delay_us(35);

  // Power-on step 6
  // Set MOSI, SCK and SS as output, all others input. SS must be output for SPI remain Master. See docs.
  SPI_DDR = (1<<SPI_MOSI)|(1<<SPI_SCK)|(1<<SPI_SS);
  // Enable SPI, Master, set clock rate fck/16
  SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);

  // Power-on step 7
  _delay_us(35);

  // Power-on step 8
  accelRearX = accelRead(FPGA_COMMAND_ACCEL_REAR_SS_PIN);
  accelFrontX = accelRead(FPGA_COMMAND_ACCEL_FRONT_SS_PIN);

  // Power-on step 9
  FPGA_COMMAND_PORT &= ~(1<<FPGA_COMMAND_ACCELS_INIT_PIN);

  // Power-on step 10
  _delay_ms(200);

  // Power-on step 11
  SPI_DDR = (1<<SPI_MISO);
  // CPOL=0, CPHA=1 - Trailing (Falling) Edge
  // DORD = 1 - LSB first
  SPCR = _BV(SPE) | _BV(DORD) | _BV(CPHA);

  // Power-on step 12
  FPGA_ENABLE_PORT |= (1<<FPGA_ENABLE_PIN);

  // Set up laser port.
  DDRB |= 1<<PB6;
  
  // set up pump port
  DDRG |= 1<<PG0;
  
  // set up valve port
  DDRL |= 1<<PL6;
}

int main() {
  setup();

  byte i = 0;
  byte data;
  while (1) {
    if (SPDR == 0xa5) {
      SPDR = 0x00;
      readSpi();
      // Calibration routine.
      if (calibrator.isRunning()) {
        if (calibrator.isHit()) {
          if (calibrator.isBacking()) {
            writeSpi(calibrator.getBackoffCommand());
          } else {
            calibrator.startBacking();
            writeSpi(calibrator.getBackoffCommand());
          }
        } else if (calibrator.isBacking()) {
          calibrator.stop();
          // writeSpi((Command*) &sequenceRest[1]);
          writeSpiRest();
        } else {
          writeSpi(calibrator.getForwardCommand());
        }
      // If nothing left in the queue then send STOP.
      } else if (cmdQueue.isEmpty()) {
          // A safety net - the laser should not be left on when there are no commands
          // in the queue. This would be an indication that the user forgot to turn
          // it off or something happened with the comms or with the program.
          laserOff();
          // writeSpi((Command*) &sequenceRest[1]);
          writeSpiRest();
      // Some command is waiting to be processed.
      } else {
        Command* command = cmdQueue.popTail();
        // If movement command then send it to FPGA.
        if (command->type == Move) {
          writeSpi(command);
        // If not a movement command then execute it.
        } else {
          serialRead();
          implementationFunctions[command->type]();
        }
        // After a command has been sent to FPGA or directly executed peek at the
        // queue to see if there is another non-movement command is waiting next
        // and execute it too. Do until a movement command is next or no commands
        // left in the queue.
        while (!cmdQueue.isEmpty()) {
          command = cmdQueue.peekTail();
          if (command->type == Move) {
            break;
          } else {
            serialRead();
            cmdQueue.popTail();
            implementationFunctions[command->type]();
          }
        }
      }
      processSerialBuffer();
    } else {
      serialRead();
    }
  }
  return 0;
}

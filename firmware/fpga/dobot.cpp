/*
open-dobot firmware that controls Dobot FPGA.

Find driver and SDK at https://github.com/maxosprojects/open-dobot

Author: maxosprojects (March 18 2016)
Additional Authors: <put your name here>

Version: 0.5.0

License: MIT
*/

// #define DEBUG

#define F_CPU 16000000UL
#define BAUD 115200
#include <util/setbaud.h>

#include "dobot.h"

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

#define CMD_QUEUE_SIZE 200

#define GET_MOTOR_DIRECTION(X) ((prevMotorDirections >> X) & 0x01)

#define BLUETOOTH_ENABLED (PINL & (1<<PORTL1))

#define CMD_READY 0
#define CMD_STEPS 1
#define CMD_EXEC_QUEUE 2
#define CMD_GET_ACCELS 3
#define CMD_SWITCH_TO_ACCEL_REPORT_MODE 4
#define CMD_CALIBRATE_JOINT 5
#define CMD_EMERGENCY_STOP 6
#define CMD_SET_COUNTERS 7
#define CMD_GET_COUNTERS 8
#define CMD_LASER_ON 9
#define CMD_PUMP_ON 10
#define CMD_VALVE_ON 11
// DO NOT FORGET TO UPDATE cmdArray SIZE!
funcPtrs cmdArray[12];
// Last index in the commands pointers array.
int cmdPtrArrayLastIndex;

CommandQueue cmdQueue(CMD_QUEUE_SIZE);
Calibrator calibrator;

// Buffer to read command into.
byte cmd[24];
// Current index in command buffer.
byte cmdInBuffIndex = 0;
byte crc[2];

// ulong lastTimeExecuted = 0;
byte defer = 1;

uint currGripper = 0xe001;
uint currToolRotation = 0;

byte prevMotorDirections = 0;
byte currMotorDirections = 0;
long motorPositionBase = 0;
long motorPositionRear = 0;
long motorPositionFore = 0;

uint accelRear;
uint accelFront;
byte accelReportMode = 0;

void setup() {
  cmdArray[CMD_READY] = cmdReady;
  cmdArray[CMD_STEPS] = cmdSteps;
  cmdArray[CMD_EXEC_QUEUE] = cmdExecQueue;
  cmdArray[CMD_GET_ACCELS] = cmdGetAccels;
  cmdArray[CMD_SWITCH_TO_ACCEL_REPORT_MODE] = cmdSwitchToAccelReportMode;
  cmdArray[CMD_CALIBRATE_JOINT] = cmdCalibrateJoint;
  cmdArray[CMD_EMERGENCY_STOP] = cmdEmergencyStop;
  cmdArray[CMD_SET_COUNTERS] = cmdSetCounters;
  cmdArray[CMD_GET_COUNTERS] = cmdGetCounters;
  cmdArray[CMD_LASER_ON] = cmdLaserOn;
  cmdArray[CMD_PUMP_ON] = cmdPumpOn;
  cmdArray[CMD_VALVE_ON] = cmdValveOn;
  cmdPtrArrayLastIndex = sizeof(cmdArray) / sizeof(cmdArray[0]) - 1;

  implementationFunctions[LaserOn] = laserOn;
  implementationFunctions[LaserOff] = laserOff;
  implementationFunctions[PumpOn] = pumpOn;
  implementationFunctions[PumpOff] = pumpOff;
  implementationFunctions[ValveOn] = valveOn;
  implementationFunctions[ValveOff] = valveOff;

  serialInit();

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
    cmdSwitchToAccelReportMode();
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
  accelRear = accelRead(FPGA_COMMAND_ACCEL_REAR_SS_PIN);
  accelFront = accelRead(FPGA_COMMAND_ACCEL_FRONT_SS_PIN);

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

#ifdef DEBUG
  initDebug();
#endif

}

inline byte processCommand() {
  if (cmdInBuffIndex > 0) {
    if ((accelReportMode && cmd[0] != CMD_GET_ACCELS) || cmd[0] > cmdPtrArrayLastIndex) {
      cmdInBuffIndex = 0;
      return 0;
    }
    return cmdArray[cmd[0]]();
  }
  return 0;
}

/*
 * Processes whatever we have received in the cmd buffer so far.
 * Allows to pick up and process more than one command in one FPGA window (~20ms).
 * The number of iterations experimantally (with oscilloscope) selected to fit into
 * FPGA window taking into account the number of serial reads and an average time one
 * command takes to process.
 * It is done this way to avoid interrupts, which would make it impossible to talk to
 * FPGA as we would lose SPI clock ticks and there is no SPI transmit buffer (only receive).
 */
inline void processSerialBuffer() {
  int iterationsLeft = 10000;
  while (iterationsLeft > 0) {
    byte result = processCommand();
    if (result == 0) {
      iterationsLeft -= 1;
    } else if (result == 1) {
      iterationsLeft -= 5;
    } else {
      iterationsLeft -= 300;
    }
    serialRead();
  }
}

// CMD: Returns magic number to indicate that the controller is alive.
byte cmdReady() {
  // Check if not enough bytes yet.
  if (cmdInBuffIndex < 3) {
    return 1;
  }
  cmdInBuffIndex = 0;
  if (!checkCrc(cmd, 1)) {
    return 2;
  }
  // Return magic number.
  cmd[0] = 0x40;
  write1(cmd);
  return 3;
}

// CMD: Adds a command to the queue.
byte cmdSteps() {
  // Check if not enough bytes yet.
  if (cmdInBuffIndex < 20) {
    return 1;
  }
  cmdInBuffIndex = 0;
  if (!checkCrc(cmd, 18)) {
    return 2;
  }
  currGripper = cmd[14] | cmd[15] << 8;
  currToolRotation = cmd[16] | cmd[17] << 8;
  cmd[0] = cmdQueue.appendHead((ulong*) &cmd[1], (ulong*) &cmd[5], (ulong*) &cmd[9], &cmd[13], currGripper, currToolRotation, Move);
  write1(cmd);
  return 3;
}

// CMD: Starts calibration routine.
byte cmdCalibrateJoint() {
  // Check if not enough bytes yet.
  if (cmdInBuffIndex < 13) {
    return 1;
  }
  cmdInBuffIndex = 0;
  if (!checkCrc(cmd, 11)) {
    return 2;
  }
  // The received command has the following structure:
  // long - forward speed
  // long - back speed
  // byte - pin number
  // byte - control structure: [1-0] - joint (#3 - illegal)
  byte pin = cmd[9];
  byte ctrl = cmd[10];
  byte joint = ctrl & 0x03;
  if (joint == 3) {
    return 2;
  }
  cmdQueue.clear();
  calibrator.start(pin, ctrl, (ulong*) &cmd[1], (ulong*) &cmd[5], currGripper, currToolRotation);
  write0();
  return 3;
}

// CMD: Returns data read from accelerometers.
byte cmdGetAccels() {
  // Check if not enough bytes yet.
  if (cmdInBuffIndex < 3) {
    return 1;
  }
  cmdInBuffIndex = 0;
  if (!checkCrc(cmd, 1)) {
    return 2;
  }
  write22(cmd, &accelRear, &accelFront);
  return 3;
}

// CMD: Stops the arm by clearing command buffer and stopping calibrator
// Use in emergency
byte cmdEmergencyStop() {
  // Check if not enough bytes yet.
  if (cmdInBuffIndex < 3) {
    return 1;
  }
  cmdInBuffIndex = 0;
  if (!checkCrc(cmd, 1)) {
    return 2;
  }
  cmdQueue.clear();
  calibrator.stop();
  write0();
  return 3;
}

// CMD: Sets joint counters to the received values.
byte cmdSetCounters() {
  // Check if not enough bytes yet.
  if (cmdInBuffIndex < 15) {
    return 1;
  }
  cmdInBuffIndex = 0;
  if (!checkCrc(cmd, 13)) {
    return 2;
  }
  motorPositionBase = ((long)cmd[1] << 24) | ((long)cmd[2] << 16) | ((long)cmd[3] << 8) | (long)cmd[4];
  motorPositionRear = ((long)cmd[5] << 24) | ((long)cmd[6] << 16) | ((long)cmd[7] << 8) | (long)cmd[8];
  motorPositionFore = ((long)cmd[9] << 24) | ((long)cmd[10] << 16) | ((long)cmd[11] << 8) | (long)cmd[12];
  write0();
  return 3;
}

// CMD: Returns current counters for all joints.
byte cmdGetCounters() {
  // Check if not enough bytes yet.
  if (cmdInBuffIndex < 3) {
    return 1;
  }
  cmdInBuffIndex = 0;
  if (!checkCrc(cmd, 1)) {
    return 2;
  }
  write444(cmd, (ulong*)&motorPositionBase, (ulong*)&motorPositionRear, (ulong*)&motorPositionFore);
  return 3;
}

// CMD: Enables/Disables Laser
byte cmdLaserOn() {
  // Check if not enough bytes yet.
  if (cmdInBuffIndex < 4) {
    return 1;
  }
  cmdInBuffIndex = 0;
  if (!checkCrc(cmd, 2)) {
    return 2;
  }
  byte on = cmd[1];
  if (on) {
    cmd[0] = cmdQueue.appendHead(0, 0, 0, 0, 0, 0, LaserOn);
  } else {
    cmd[0] = cmdQueue.appendHead(0, 0, 0, 0, 0, 0, LaserOff);
  }
  write1(cmd);
  return 3;
}

// CMD: Enables/Disables Pump
byte cmdPumpOn() {
  // Check if not enough bytes yet.
  if (cmdInBuffIndex < 4) {
    return 1;
  }
  cmdInBuffIndex = 0;
  if (!checkCrc(cmd, 2)) {
    return 2;
  }
  byte on = cmd[1];
  if (on) {
    cmd[0] = cmdQueue.appendHead(0, 0, 0, 0, 0, 0, PumpOn);
  } else {
    cmd[0] = cmdQueue.appendHead(0, 0, 0, 0, 0, 0, PumpOff);
  }
  write1(cmd);
  return 3;
}

// CMD: Enables/Disables Valve
byte cmdValveOn() {
  // Check if not enough bytes yet.
  if (cmdInBuffIndex < 4) {
    return 1;
  }
  cmdInBuffIndex = 0;
  if (!checkCrc(cmd, 2)) {
    return 2;
  }
  byte on = cmd[1];
  if (on) {
    cmd[0] = cmdQueue.appendHead(0, 0, 0, 0, 0, 0, ValveOn);
  } else {
    cmd[0] = cmdQueue.appendHead(0, 0, 0, 0, 0, 0, ValveOff);
  }
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

// CMD: Executes deferred commands in the queue.
byte cmdExecQueue() {
  // Check if not enough bytes yet.
  if (cmdInBuffIndex < 3) {
    return 0;
  }
  cmdInBuffIndex = 0;
  crcCcitt(cmd, 1);
  defer = 0;
  return 1;
}

// CMD: Switches controller to accelerometers report mode.
byte cmdSwitchToAccelReportMode() {
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
      accelRear = accelRead(FPGA_COMMAND_ACCEL_REAR_SS_PIN);
      processSerialBuffer();
      accelFront = accelRead(FPGA_COMMAND_ACCEL_FRONT_SS_PIN);
      processSerialBuffer();
    }
  // }
  return 1;
}

void serialInit(void) {
  // Configure serial0
  UBRR0H = UBRRH_VALUE;
  UBRR0L = UBRRL_VALUE;

#if USE_2X
  UCSR0A |= _BV(U2X0);
#else
  UCSR0A &= ~(_BV(U2X0));
#endif
  UCSR0C = _BV(UCSZ01) | _BV(UCSZ00); /* 8-bit data */
  UCSR0B = _BV(RXEN0) | _BV(TXEN0);   /* Enable RX and TX */

  // Configure serial1
  UBRR1H = UBRRH_VALUE;
  UBRR1L = UBRRL_VALUE;
#if USE_2X
  UCSR1A |= _BV(U2X1);
#else
  UCSR1A &= ~(_BV(U2X1));
#endif
  UCSR1C = _BV(UCSZ11) | _BV(UCSZ10); /* 8-bit data */
  UCSR1B = _BV(RXEN1) | _BV(TXEN1);   /* Enable RX and TX */
}

void initDebug() {
#ifdef DEBUG
  DDRH |= (1<<PORTH5);
#endif
}

inline void debugOn() {
#ifdef DEBUG
  PORTH |= (1<<PORTH5);
#endif
}

inline void debugOff() {
#ifdef DEBUG
  PORTH &= ~(1<<PORTH5);
#endif
}

inline void serialWrite(byte c) {
  if (BLUETOOTH_ENABLED) {
    loop_until_bit_is_set(UCSR1A, UDRE1);
    UDR1 = c;
  } else {
    loop_until_bit_is_set(UCSR0A, UDRE0);
    UDR0 = c;
  }
}

inline void serialWrite(byte data[], byte num) {
  if (BLUETOOTH_ENABLED) {
    for (byte i = 0; i < num; i++) {
      loop_until_bit_is_set(UCSR1A, UDRE1);
      UDR1 = data[i];
    }
  } else {
    for (byte i = 0; i < num; i++) {
      loop_until_bit_is_set(UCSR0A, UDRE0);
      UDR0 = data[i];
    }
  }
}

// Returns number of bytes read or 0 if timeout occurred.
// Timeout: 1000 increments ~ 600us, so allow about 9ms interbyte and 18ms overall.
inline byte serialReadNum(byte data[], byte num) {
  unsigned int interByteTimeout = 0;
  unsigned int transactionTimeout = 0;
  byte cnt = 0;
  byte tmp;
  if (BLUETOOTH_ENABLED) {
    while (cnt < num) {
      // Wait until data exists.
      while (!(UCSR1A & (1<<RXC1))) {
        interByteTimeout++;
        transactionTimeout++;
        if (interByteTimeout > 15000 || transactionTimeout > 30000) {
          return 0;
        }
      }
      interByteTimeout = 0;
      tmp = UDR1;
      data[cnt++] = tmp;
    }
  } else {
    while (cnt < num) {
      // Wait until data exists.
      while (!(UCSR0A & (1<<RXC0))) {
        interByteTimeout++;
        transactionTimeout++;
        if (interByteTimeout > 15000 || transactionTimeout > 30000) {
          return 0;
        }
      }
      interByteTimeout = 0;
      tmp = UDR0;
      data[cnt++] = tmp;
    }
  }
  return cnt;
}

inline void serialRead() {
  // This is done this way for code speed optimization purposes.
  // Don't change, otherwise SPI bytes are skipped and commands lost.
  if (BLUETOOTH_ENABLED) {
    if (UCSR1A & (1<<RXC1)) {
      cmd[cmdInBuffIndex] = UDR1;
      if (cmdInBuffIndex < 23) {
        cmdInBuffIndex++;
      }
    }
  } else {
    if (UCSR0A & (1<<RXC0)) {
      cmd[cmdInBuffIndex] = UDR0;
      if (cmdInBuffIndex < 23) {
        cmdInBuffIndex++;
      }
    }
  }
}

uint accelRead(unsigned char pin) {
  byte junk;
  uint result = 0, data;
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

byte read2(byte data[]) {
  return serialReadNum(data, 2);
}

byte read13(byte data[]) {
  return serialReadNum(data, 13);
}

void write0() {
  serialWrite(crc, 2);
}

void write1(byte data[]) {
  crcCcitt(data, 1, 1);
  data[1] = crc[0];
  data[2] = crc[1];
  serialWrite(data, 3);
}

byte write22(byte data[], uint* val1, uint* val2) {
  data[0] = (byte) (*val1 >> 8) & 0xff;
  data[1] = (byte) *val1 & 0xff;
  data[2] = (byte) (*val2 >> 8) & 0xff;
  data[3] = (byte) *val2 & 0xff;
  write4(data);
}

byte write4(byte data[]) {
  crcCcitt(data, 4, 1);
  data[4] = crc[0];
  data[5] = crc[1];
  serialWrite(data, 6);
}

byte write4Continue(byte data[], ulong* val) {
  data[0] = (byte) (*val >> 24) & 0xff;
  data[1] = (byte) (*val >> 16) & 0xff;
  data[2] = (byte) (*val >> 8) & 0xff;
  data[3] = (byte) *val & 0xff;
}

byte write444(byte data[], ulong* val1, ulong* val2, ulong* val3) {
  write4Continue(&data[0], val1);
  write4Continue(&data[4], val2);
  write4Continue(&data[8], val3);
  crcCcitt(data, 12, 1);
  data[12] = crc[0];
  data[13] = crc[1];
  serialWrite(data, 14);
}

int dataToInt(byte data[]) {
  int result = ((((uint16_t) data[0]) << 8) & 0xFF00) | (((uint16_t) data[1]) & 0x00FF);
  return result;
}

uint16_t dataToUint(byte data[]) {
  return (uint16_t) dataToInt(data);
}

byte checkCrc(byte data[], int len) {
  crcCcitt(cmd, len);
  if (data[len] == crc[0] && data[len + 1] == crc[1]) {
    return 1;
  }
  return 0;
}

byte confirmCrc(byte data[], int len) {
  if (checkCrc(data, len)) {
    serialWrite(1);
    return 1;
  }
  return 0;
}

void resetCrc() {
  crc[0] = 0xff;
  crc[1] = 0xff;
}

void crcCcitt(byte data[], int len) {
  crcCcitt(data, len, 0);
}

void crcCcitt(byte data[], int len, byte keepSeed) {
  uint16_t localCrc;
  if (keepSeed) {
    localCrc = ((((uint16_t) crc[0]) << 8) & 0xFF00) | (((uint16_t) crc[1]) & 0x00FF);
  } else {
    localCrc = 0xffff;
  }
  for (int i = 0; i < len; i++) {
    localCrc = localCrc ^ ( (((uint16_t) data[i]) << 8) & 0xff00);
    for (int n = 0; n < 8; n++) {
      if ((localCrc & 0x8000) == 0x8000) {
        localCrc = ((localCrc << 1) ^ 0x1021);
      } else {
        localCrc = localCrc << 1;
      }
    }
  }
  crc[0] = (byte) (localCrc >> 8);
  crc[1] = (byte) localCrc;
}

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

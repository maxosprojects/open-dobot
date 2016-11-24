/*
open-dobot firmware.

Find driver and SDK at https://github.com/maxosprojects/open-dobot

Author: maxosprojects (March 18 2016)
Additional Authors: <put your name here>

Version: 1.2.2

License: MIT
*/

#define F_CPU 16000000UL
#define BAUD 115200
#include <util/setbaud.h>

#include "dobot.h"
#include "inlines.h"
#include "misc/calibrator.h"
#include "misc/queue.h"

// #ifndef RAMPS
// #include "../fpga/fpga.h"
// #else
// #include "../ramps/ramps.h"
// #endif

funcPtrs cmdArray[14];
// Last index in the commands pointers array.
int cmdPtrArrayLastIndex;

implFuncPtrs implementationFunctions[7];

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

int accelRearX;
int accelRearY;
int accelRearZ;
int accelFrontX;
int accelFrontY;
int accelFrontZ;
byte accelReportMode = 0;

void setup() {
  cmdArray[CMD_READY] = cmdReady;
  cmdArray[CMD_STEPS] = cmdSteps;
  cmdArray[CMD_EXEC_QUEUE] = cmdExecQueue;
  cmdArray[CMD_GET_ACCELS] = cmdGetAccels;
  cmdArray[CMD_SWITCH_TO_ACCEL_REPORT_MODE] = cmdGetAccels;
  cmdArray[CMD_CALIBRATE_JOINT] = cmdCalibrateJoint;
  cmdArray[CMD_EMERGENCY_STOP] = cmdEmergencyStop;
  cmdArray[CMD_SET_COUNTERS] = cmdSetCounters;
  cmdArray[CMD_GET_COUNTERS] = cmdGetCounters;
  cmdArray[CMD_LASER_ON] = cmdLaserOn;
  cmdArray[CMD_PUMP_ON] = cmdPumpOn;
  cmdArray[CMD_VALVE_ON] = cmdValveOn;
  cmdArray[CMD_BOARD_VERSION] = cmdBoardVersion;
  cmdArray[CMD_CALIBRATOR_STATUS] = cmdCalibratorStatus;
  cmdPtrArrayLastIndex = sizeof(cmdArray) / sizeof(cmdArray[0]) - 1;

  implementationFunctions[LaserOn] = laserOn;
  implementationFunctions[LaserOff] = laserOff;
  implementationFunctions[PumpOn] = pumpOn;
  implementationFunctions[PumpOff] = pumpOff;
  implementationFunctions[ValveOn] = valveOn;
  implementationFunctions[ValveOff] = valveOff;

  serialInit();

  setupBoard();

#ifdef DEBUG
  initDebug();
#endif
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
void processSerialBuffer() {
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
  updateAccels();
  writes222222(cmd, &accelRearX, &accelRearY, &accelRearZ, &accelFrontX, &accelFrontY, &accelFrontZ);
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

// CMD: Return calibrator status 0:stopped, 1:unning, 2:backing
byte cmdCalibratorStatus() {
  // Check if not enough bytes yet.
  if (cmdInBuffIndex < 3) {
    return 1;
  }
  cmdInBuffIndex = 0;
  if (!checkCrc(cmd, 1)) {
    return 2;
  }
  
  if(calibrator.isRunning())
	cmd[0] = 1;
  else if(calibrator.isBacking())
	cmd[0] = 2;
  else
	cmd[0] = 0;
  
  // Return calibrator status.
  write1(cmd);
  return 3;
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

byte writes22(byte data[], int* val1, int* val2) {
  return write22(data, (uint*)val1, (uint*)val2);
}

byte writes222222(byte data[], int* val1, int* val2, int* val3, int* val4, int* val5, int* val6) {
  data[0] = (byte) (*val1 >> 8) & 0xff;
  data[1] = (byte) *val1 & 0xff;
  data[2] = (byte) (*val2 >> 8) & 0xff;
  data[3] = (byte) *val2 & 0xff;
  data[4] = (byte) (*val3 >> 8) & 0xff;
  data[5] = (byte) *val3 & 0xff;
  data[6] = (byte) (*val4 >> 8) & 0xff;
  data[7] = (byte) *val4 & 0xff;
  data[8] = (byte) (*val5 >> 8) & 0xff;
  data[9] = (byte) *val5 & 0xff;
  data[10] = (byte) (*val6 >> 8) & 0xff;
  data[11] = (byte) *val6 & 0xff;
  write12(data);
}

byte write4(byte data[]) {
  crcCcitt(data, 4, 1);
  data[4] = crc[0];
  data[5] = crc[1];
  serialWrite(data, 6);
}

byte write12(byte data[]) {
  crcCcitt(data, 12, 1);
  data[12] = crc[0];
  data[13] = crc[1];
  serialWrite(data, 14);
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

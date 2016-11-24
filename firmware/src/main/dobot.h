/*
open-dobot firmware that controls Dobot FPGA.

Find driver and SDK at https://github.com/maxosprojects/open-dobot

Author: maxosprojects (March 18 2016)
Additional Authors: <put your name here>

Version: 1.2.2

License: MIT
*/

#ifndef DOBOT_H_INCLUDED
#define DOBOT_H_INCLUDED

#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <string.h>

#define NOP __asm__ __volatile__ ("nop\n\t")

#define CMD_QUEUE_SIZE 100

#define BLUETOOTH_ENABLED (PINL & (1<<PORTL1))

// DO NOT FORGET TO UPDATE cmdArray SIZE!
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
#define CMD_BOARD_VERSION 12
#define CMD_CALIBRATOR_STATUS 13
// DO NOT FORGET TO UPDATE cmdArray SIZE!

#define GET_MOTOR_DIRECTION(X) ((prevMotorDirections >> X) & 0x01)

typedef unsigned char byte;
typedef unsigned int uint;
typedef unsigned long ulong;

typedef byte(*funcPtrs)(void);
typedef void(*implFuncPtrs)(void);

// DO NOT FORGET TO UPDATE implementationFunctions SIZE if changing these!
enum CommandType {
  Move = 0,
  LaserOn,
  LaserOff,
  PumpOn,
  PumpOff,
  ValveOn,
  ValveOff
};

typedef struct {
  ulong j1;
  ulong j2;
  ulong j3;
  // ControlByte control;
  byte control;
  uint servoGrab;
  uint servoRot;
  CommandType type;
} Command;

typedef struct {
  volatile byte* portin;
  byte pin;
  volatile byte* ddr;
} SwitchPort;

extern byte cmd[];
extern byte cmdInBuffIndex;
extern implFuncPtrs implementationFunctions[];
extern SwitchPort calibrationPins[];

extern byte calibrationPinsNumber();

extern long motorPositionBase;
extern long motorPositionRear;
extern long motorPositionFore;

extern int accelRearX;
extern int accelRearY;
extern int accelRearZ;
extern int accelFrontX;
extern int accelFrontY;
extern int accelFrontZ;

void setup();
void setupBoard();
void processSerialBuffer();
int accelRead(byte pin);
byte cmdReady();
byte cmdSteps();
byte cmdExecQueue();
byte cmdGetAccels();
byte cmdCalibrateJoint();
byte cmdEmergencyStop();
byte cmdSetCounters();
byte cmdGetCounters();
byte cmdLaserOn();
byte cmdPumpOn();
byte cmdValveOn();
byte cmdBoardVersion();
byte cmdCalibratorStatus();

void crcCcitt(byte data[], int len);
void crcCcitt(byte data[], int len, byte keepSeed);
byte read13(byte data[]);
void write0();
void write1(byte data[]);
byte write22(byte data[], uint* val1, uint* val2);
byte writes22(byte data[], int* val1, int* val2);
byte writes222222(byte data[], int* val1, int* val2, int* val3, int* val4, int* val5, int* val6);
byte write4(byte data[]);
byte write12(byte data[]);
byte write4Continue(byte data[], ulong* val);
byte write444(byte data[], ulong* val1, ulong* val2, ulong* val3);
byte checkCrc(byte data[], int len);
byte confirmCrc(byte data[], int len);
void resetCrc();
void serialInit(void);
void initDebug();
void laserOn();
void laserOff();
void pumpOn();
void pumpOff();
void valveOn();
void valveOff();
void updateAccels();

#endif // DOBOT_H_INCLUDED

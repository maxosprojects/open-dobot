/*
open-dobot firmware that controls Dobot FPGA.

Find driver and SDK at https://github.com/maxosprojects/open-dobot

Author: maxosprojects (March 18 2016)
Additional Authors: <put your name here>

Version: 0.5.0

License: MIT
*/

#ifndef DOBOT_H_INCLUDED
#define DOBOT_H_INCLUDED

#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <string.h>

#define NOP __asm__ __volatile__ ("nop\n\t")

typedef unsigned char byte;
typedef unsigned int uint;
typedef unsigned long ulong;

typedef byte(*funcPtrs)(void);
typedef void(*implFuncPtrs)(void);

// typedef struct {
//   byte j1dir : 1;
//   byte j2dir : 1;
//   byte j3dir : 1;
//   // byte deferred : 1;
// } ControlByte;

// DO NOT FORGET TO UPDATE implementationFunctions SIZE!
enum CommandType {
  Move = 0,
  LaserOn,
  LaserOff,
  PumpOn,
  PumpOff,
  ValveOn,
  ValveOff
};
implFuncPtrs implementationFunctions[7];

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

byte cmdReady();
byte cmdSteps();
byte cmdExecQueue();
byte cmdGetAccels();
byte cmdSwitchToAccelReportMode();
byte cmdCalibrateJoint();
byte cmdEmergencyStop();
byte cmdSetCounters();
byte cmdGetCounters();
byte cmdLaserOn();
byte cmdPumpOn();
byte cmdValveOn();
void crcCcitt(byte data[], int len);
void crcCcitt(byte data[], int len, byte keepSeed);
byte read13(byte data[]);
void write0();
void write1(byte data[]);
byte write22(byte data[], uint* val1, uint* val2);
byte write4(byte data[]);
byte write4Continue(byte data[], ulong* val);
byte write444(byte data[], ulong* val1, ulong* val2, ulong* val3);
byte checkCrc(byte data[], int len);
byte confirmCrc(byte data[], int len);
void resetCrc();
void serialInit(void);
void serialWrite(byte c);
void serialWrite(byte data[], byte num);
void serialRead();
byte serialReadNum(byte data[], byte num);
uint accelRead(unsigned char pin);
void initDebug();
void debugOn();
void debugOff();
void laserOn();
void laserOff();
void pumpOn();
void pumpOff();
void valveOn();
void valveOff();

// Rest
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

class Calibrator {
  public:
    Calibrator() {
      running = 0;
      backing = 0;
      // No pin currently selected.
      pin = 255;
      // Pull-up is disabled.
      pullup = 0;
    };

    // control: [7-4] unused, [4] - pinMode, [3] - pullup enable, [2] - direction, [1-0] - joint
    void start(byte newPin, byte control, ulong *fwdSpeed, ulong *backSpeed, uint gripper, uint toolRotation) {
      // Disable pull-up resistor if it was enabled.
      if (pullup == 1) {
        *(calibrationPins[pin].portin) &= ~(1 << calibrationPins[pin].pin);
      }
      // Check the boundaries.
      if (newPin < sizeof(calibrationPins) / 3) {
        pin = newPin;
        running = 1;
        byte joint = control & 0x03;
        pullup = (control >> 3) & 0x01;
        pinMode = (control >> 4) & 0x01;
        // direction
        fwdCmd.control = (control & 0x04) >> (2 - joint);
        fwdCmd.servoGrab= gripper;
        fwdCmd.servoRot = toolRotation;
        backCmd.control = (~control & 0x04) >> (5 - joint);
        backCmd.servoGrab= gripper;
        backCmd.servoRot = toolRotation;
        memset(&fwdCmd, 0, 12);
        memset(&backCmd, 0, 12);
        ulong *ptr;
        ptr = (ulong*) &fwdCmd;
        ptr[joint] = *fwdSpeed;
        ptr = (ulong*) &backCmd;
        ptr[joint] = *backSpeed;
        // Enable pull-up resistor.
        if (pullup) {
          *(calibrationPins[pin].portin) |= (1 << calibrationPins[pin].pin);
        }
      }
    };

    void startBacking() {
      backing = 1;
    }

    void stop() {
      running = 0;
      backing = 0;
      // Disable pull-up resistor if it was enabled.
      if (pullup == 1) {
        *(calibrationPins[pin].portin) &= ~(1 << calibrationPins[pin].pin);
      }
    };

    Command* getForwardCommand() {
      return &fwdCmd;
    };

    Command* getBackoffCommand() {
      return &backCmd;
    };

    byte isRunning() {
      return running;
    };

    byte isBacking() {
      return backing;
    }

    // Returns 0 if not hit and 1 if hit, based on the mode requested in start()
    byte isHit() {
      if (pinMode) {
        return ~(*(calibrationPins[pin].portin) >> calibrationPins[pin].pin) & 0x01;
      } else {
        return (*(calibrationPins[pin].portin) >> calibrationPins[pin].pin) & 0x01;
      }
    };

  private:
    // 0 - no, 1 - yes
    byte running;
    // 0 - not backing, 1 - backing
    byte backing;
    // Pin mode: 0 - normal LOW, 1 - normal HIGH
    byte pinMode;
    // Pin to poll state of.
    byte pin;
    // Enabling pullup.
    byte pullup;
    // Command to send when moving forward - towards the endstop/interrupter.
    Command fwdCmd;
    // Command to send when moving back - from endstop/interrupter to get accurate location.
    Command backCmd;
};

class CommandQueue {
  public:
    CommandQueue(unsigned int newSize) {
      // queue = new Command[newSize];
      queue = (Command*) malloc(sizeof(Command) * newSize);
      head = 0;
      tail = 0;
      size = newSize;
    };

    byte appendHead(ulong *newJ1, ulong *newJ2, ulong *newJ3, byte *control, uint servoGrab, uint servoRot, CommandType type) {
      if (!isFull()) {
        queue[head].j1 = *newJ1;
        queue[head].j2 = *newJ2;
        queue[head].j3 = *newJ3;
        queue[head].control = *control;
        queue[head].servoGrab = servoGrab;
        queue[head].servoRot = servoRot;
        queue[head].type = type;
        // queue[head].control.j1dir = (control >> 1) & 0x01;
        // queue[head].control.j2dir = (control >> 2) & 0x01;
        // queue[head].control.j3dir = (control >> 3) & 0x01;
        head++;
        if (head >= size) {
          head = 0;
        }
        return 1;
      }
      return 0;
    };

    Command* appendHead() {
      if (isFull()) {
        return 0;
      }
      Command* nextPlace = &queue[head];
      head++;
      if (head >= size) {
        head = 0;
      }
      return nextPlace;
    };

    Command* peekTail() {
      return &queue[tail];
    };

    Command* popTail() {
      Command* ptr = &queue[tail];
      tail++;
      if (tail >= size) {
        tail = 0;
      }
      return ptr;
    };

    void clear() {
      head = 0;
      tail = 0;
    };

    byte isEmpty() {
      return head == tail;
    };

    byte isFull() {
      int diff = head - tail;
      if (diff == -1 || diff == (size - 1)) {
        return 1;
      }
      return 0;
    };

  private:
    unsigned int size;
    unsigned int head;
    unsigned int tail;
    Command *queue;
};

#endif // DOBOT_H_INCLUDED

/*
open-dobot firmware for RAMPS version.

Find improved FPGA version, driver and SDK at https://github.com/maxosprojects/open-dobot

Author: maxosprojects (March 18 2016)
Additional Authors: <put your name here>

License: MIT
*/

#include "dobot.h"

#define J1_STEP_PIN         60
#define J1_DIR_PIN          61
#define J1_ENABLE_PIN       56

#define J2_STEP_PIN         54
#define J2_DIR_PIN          55
#define J2_ENABLE_PIN       38

#define J3_STEP_PIN         26
#define J3_DIR_PIN          28
#define J3_ENABLE_PIN       24

#define CMD_QUEUE_SIZE     500

volatile CommandQueue cmdQueue(CMD_QUEUE_SIZE);

volatile unsigned int ticks = 0;

#define CMD_READY 0
#define CMD_STEPS 1
#define CMD_EXEC_QUEUE 2
// DO NOT FORGET TO UPDATE cmdArray SIZE!
funcPtrs cmdArray[3];
// Last index in the commands array.
int cmdLastIndex;

byte cmd[20];
byte crc[2];

unsigned long lastTimeExecuted = 0;
unsigned char defer = 1;

void setup() {
  pinMode(J1_STEP_PIN, OUTPUT);
  pinMode(J1_DIR_PIN, OUTPUT);
  pinMode(J1_ENABLE_PIN, OUTPUT);

  pinMode(J2_STEP_PIN, OUTPUT);
  pinMode(J2_DIR_PIN, OUTPUT);
  pinMode(J2_ENABLE_PIN, OUTPUT);

  pinMode(J3_STEP_PIN, OUTPUT);
  pinMode(J3_DIR_PIN, OUTPUT);
  pinMode(J3_ENABLE_PIN, OUTPUT);

  digitalWrite(J1_ENABLE_PIN, LOW);
  digitalWrite(J2_ENABLE_PIN, LOW);
  digitalWrite(J3_ENABLE_PIN, LOW);

  cmdArray[CMD_READY] = cmdReady;
  cmdArray[CMD_STEPS] = cmdSteps;
  cmdArray[CMD_EXEC_QUEUE] = cmdExecQueue;
  cmdLastIndex = sizeof(cmdArray) / sizeof(cmdArray[0]) - 1;

  // initialize timer3
  noInterrupts();           // disable all interrupts
  TCCR3A = 0;
  TCCR3B = 0;
  TCNT3  = 0;

  OCR3A = 8;                // compare match register 16MHz/256/(frequency) - around 3.47kHz
  TCCR3B |= (1 << WGM32);   // CTC mode
  TCCR3B |= (1 << CS32);    // 256 prescaler 
  TIMSK3 |= (1 << OCIE3A);  // enable timer compare interrupt
  interrupts();             // enable all interrupts

  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect.
  }
  // Set timeout for readBytes() to 100 ms.
  Serial.setTimeout(100);
}

void loop () {
  processCommand();
}

void processCommand() {
  if (Serial.available() > 0) {
    // get incoming byte
    cmd[0] = (byte)Serial.read();
    if (cmd[0] > cmdLastIndex) {
      return;
    }
    cmdArray[cmd[0]]();
  }
}

// CMD: returns magic number to indicate that the controller is alive.
void cmdReady() {
  crcCcitt(cmd, 1);
  // Return magic number.
  cmd[0] = 0x40;
  write1(cmd);
}

/* CMD: adds a command to the queue.
* The format is the following:
* |----------------------------------------------------|
* | Byte |                Description                  |
* |number|                                             |
* |----------------------------------------------------|
* | 0    | Command number                              |
* | 1    | Joint 1 scaler                              |
* | 2    | Joint 2 scaler                              |
* | 3    | Joint 3 scaler                              |
* | 4-5  | Ticks number command will be executing for  |
* | 6    | Control byte: (bits 0-3)                    |
* |      |   0 - deferred command (waits for ExecQueue)|
* |      |   1 - Joint1 direction                      |
* |      |   2 - Joint2 direction                      |
* |      |   3 - Joint3 direction                      |
* | 7-8  | Checksum                                    |
* |----------------------------------------------------|
*/
void cmdSteps() {
  if (!read6(&cmd[1])) {
    return;
  }
  if (checkCrc(cmd, 7)) {
    resetCrc();
    if (cmdQueue.appendHead(cmd[1], cmd[2], cmd[3], dataToUint(&cmd[4]), cmd[6])) {
      cmd[0] = 1;
      write1(cmd);
    } else {
      cmd[0] = 0;
      write1(cmd);
    }
  }
}

// CMD: Execute deferred commands in the queue.
void cmdExecQueue() {
  defer = 0;
}

// Returns True if the requested number of bytes was read.
boolean readNum(byte data[], int num) {
  return Serial.readBytes(data, num) == num;
}

boolean read2(byte data[]) {
  return readNum(data, 2);
}

boolean read6(byte data[]) {
  return readNum(data, 6);
}

boolean write1(byte data[]) {
  crcCcitt(data, 1, true);
  data[1] = crc[0];
  data[2] = crc[1];
  Serial.write(data, 3);
}

int dataToInt(byte data[]) {
  int result = ((((uint16_t) data[0]) << 8) & 0xFF00) | (((uint16_t) data[1]) & 0x00FF);
  return result;
}

uint16_t dataToUint(byte data[]) {
  return (uint16_t) dataToInt(data);
}

boolean checkCrc(byte data[], int len) {
  if (!read2(&cmd[len])) {
    return false;
  }
  crcCcitt(cmd, len);
  if (data[len] == crc[0] && data[len + 1] == crc[1]) {
    return true;
  }
  return false;
}

boolean confirmCrc(byte data[], int len) {
  if (checkCrc(data, len)) {
    Serial.write((byte)1);
    return true;
  }
}

void resetCrc() {
  crc[0] = 0xff;
  crc[1] = 0xff;
}

void crcCcitt(byte data[], int len) {
  crcCcitt(data, len, false);
}

void crcCcitt(byte data[], int len, boolean keepSeed) {
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

// Timer3 ISR. Executes commands tail to head to avoid jerking.
// If a deferred command is found and it has more than 30ms passed since the last command executed
// then it will wait for ExecQueue command.
ISR(TIMER3_COMPA_vect) {
  if (cmdQueue.isEmptyIsr()) {
    ticks = 0;
    defer = 1;
    return;
  }
  unsigned long currentTime = millis();
  volatile Command* tail = cmdQueue.peekTailIsr();
  // If last executed command finished more than 30 ms ago and starting (ticks==0) a defered 
  // command (tail->defer==1) and global defer==1, then wait for global defer==0.
  if (ticks == 0 && tail->control.deferred && defer && (currentTime - lastTimeExecuted) > 30) {
    return;
  }
  lastTimeExecuted = currentTime;
  if (ticks % tail->j1 == 0) {
    // It is faster to update variable and read from variable than to read current port state.
    tail->control.currJ1 = !tail->control.currJ1;
    digitalWrite(J1_STEP_PIN, tail->control.currJ1);
  }
  if (ticks % tail->j2 == 0) {
    tail->control.currJ2 = !tail->control.currJ2;
    digitalWrite(J2_STEP_PIN, tail->control.currJ2);
  }
  if (ticks % tail->j3 == 0) {
    tail->control.currJ3 = !tail->control.currJ3;
    digitalWrite(J3_STEP_PIN, tail->control.currJ3);
  }
  // Change direction only at the beginning of the command to save cycles.
  if (ticks == 0) {
    digitalWrite(J1_DIR_PIN, tail->control.j1dir);
    digitalWrite(J2_DIR_PIN, tail->control.j2dir);
    digitalWrite(J3_DIR_PIN, tail->control.j3dir);
  }
  if (ticks >= tail->ticks && cmdQueue.isProcessing() == 0) {
    ticks = 0;
    cmdQueue.popTailIsr();
  }

  // Should never happen as any command should finish within the maximum tick range and 
  // then tick counter is reset.
  if (ticks == 0xFFFF) {
    ticks = 0;
  } else {
    ticks++;
  }
}

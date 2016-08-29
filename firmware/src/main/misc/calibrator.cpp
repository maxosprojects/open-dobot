/*
open-dobot firmware.

Find driver and SDK at https://github.com/maxosprojects/open-dobot

Author: maxosprojects (March 18 2016)
Additional Authors: <put your name here>

Version: 1.2.2

License: MIT
*/

#include "../dobot.h"
#include "calibrator.h"

Calibrator::Calibrator() {
  running = 0;
  backing = 0;
  // No pin currently selected.
  pin = 255;
  // Pull-up is disabled.
  pullup = 0;
};

// control: [7-4] unused, [4] - pinMode, [3] - pullup enable, [2] - direction, [1-0] - joint
void Calibrator::start(byte newPin, byte control, ulong *fwdSpeed, ulong *backSpeed, uint gripper, uint toolRotation) {
  // Disable pull-up resistor if it was enabled.
  if (pullup == 1) {
    *(calibrationPins[pin].portin) &= ~(1 << calibrationPins[pin].pin);
  }
  // Check the boundaries.
  if (newPin < calibrationPinsNumber() / 3) {
    pin = newPin;
    running = 1;
    byte joint = control & 0x03;
    pullup = (control >> 3) & 0x01;
    pinMode = (control >> 4) & 0x01;
    // direction
    fwdCmd.control = (control & 0x04) >> (2 - joint);
    fwdCmd.servoGrab= gripper;
    fwdCmd.servoRot = toolRotation;
    backCmd.control = (~control & 0x04) >> (2 - joint);
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

void Calibrator::startBacking() {
  backing = 1;
}

void Calibrator::stop() {
  running = 0;
  backing = 0;
  // Disable pull-up resistor if it was enabled.
  if (pullup == 1) {
    *(calibrationPins[pin].portin) &= ~(1 << calibrationPins[pin].pin);
  }
};

Command* Calibrator::getForwardCommand() {
  return &fwdCmd;
};

Command* Calibrator::getBackoffCommand() {
  return &backCmd;
};

byte Calibrator::isRunning() {
  return running;
};

byte Calibrator::isBacking() {
  return backing;
}

// Returns 0 if not hit and 1 if hit, based on the mode requested in start()
byte Calibrator::isHit() {
  if (pinMode) {
    return ~(*(calibrationPins[pin].portin) >> calibrationPins[pin].pin) & 0x01;
  } else {
    return (*(calibrationPins[pin].portin) >> calibrationPins[pin].pin) & 0x01;
  }
};

/*
open-dobot firmware.

Find driver and SDK at https://github.com/maxosprojects/open-dobot

Author: maxosprojects (March 18 2016)
Additional Authors: <put your name here>

Version: 1.2.2

License: MIT
*/

class Calibrator {
  public:
    Calibrator();

    // control: [7-4] unused, [4] - pinMode, [3] - pullup enable, [2] - direction, [1-0] - joint
    void start(byte newPin, byte control, ulong *fwdSpeed, ulong *backSpeed, uint gripper, uint toolRotation);
    void startBacking();
    void stop();

    Command* getForwardCommand();
    Command* getBackoffCommand();

    byte isRunning();
    byte isBacking();

    // Returns 0 if not hit and 1 if hit, based on the mode requested in start()
    byte isHit();

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

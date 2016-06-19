# New [open-dobot forum](http://open-dobot.boards.net)

# [RAMPS is back](https://github.com/maxosprojects/open-dobot/releases/) and is to stay!

# Table of Contents
1. [What is open-dobot](#what-is-open-dobot)
2. [Why? ...or the original firmware problems](#why)
   1. [Protocol](#protocol)
   2. [The "Arduino-based" joke](#arduino-joke)
3. [Supported platforms](supported-platforms)
   1. [Hardware Platforms](#hardware-platforms)
   2. [Software Platforms](#software-platforms)
4. [Features implemented so far](#features-implemented-so-far)
5. [Planned features](http://open-dobot.boards.net)
6. [Prerequisites](#prerequisites)
7. [Disclaimer](#disclaimer)
8. [Installation](#installation)
9. [Details](#details)

# DISCLAIMER

The information provided here is a collective effort of enthusiasts who wanted to make the Dobot arm useful for themselves and others. The instructions are provided without pursuing any personal or material benefit, AS IS and to the best of the knowledge of the people involved in this project as to not cause any damage or harm with hardware and software manipulations, but to only bring the arm to life, make it useful and reliable, squeezing out everything from every dollar that has been paid for the arm. With that said, following these instructions strictly or (especially) not strictly you take any risk and full responsibility for any damage or harm that it might lead to.

THE AUTHORS OF THIS PROJECT ACCEPT ABSOLUTELY NO LIABILITY FOR ANY HARM OR LOSS RESULTING FROM ITS USE. IT IS EXTREMELY UNWISE TO RELY ON SOFTWARE ALONE FOR SAFETY. Any machinery capable of harming persons must have provisions for completely removing power from all motors, etc, before persons enter any danger area. All machinery must be designed to comply with local and national safety codes, and the authors of this software can not, and do not, take any responsibility for such compliance.

---

# What is open-dobot
This project is intended as a completely open and free (MIT License) alternative to proprietary and limited (no features other than basic ones) original firmware for Dobot robotic arm (original Kickstarter campaign https://www.kickstarter.com/projects/dobot/dobot-robotic-arm-for-everyone-arduino-and-open-so/description).

---

# <a name="why"></a> Why? ...or the original firmware problems
The need for this project arisen due to the original claim on Kickstarter to make the arm "open-source" and "Arduino-based", which has been mostly unsatisfied.
The "openness" ends with the protocol that is used to send commands to the arm (and recently the same protocol in form of some questionable API). The "Arduino-based" claim brings absolutely no value to the owner as it does not drive the arm but sends commands further to the top board (kind of Arduino shield) via SPI. The top board has FPGA that controls the three stepper drivers and two servos (PWM signals). No information on FPGA and how it functions has been released. The SPI protocol to control FPGA board has been reverse-engineered and open-dobot supports that board, however, the board is not extendable and RAMPS is preferred for many reasons.

### Protocol
The issues with the original protocol the Dobot team has released are all in the proprietary firmware, which renders the protocol useless:
- the firmware drops commands randomly (seems to be fixed at expense of command rate - 256kbaud down to 9.6kbaud)
- provides no acknowledgement of commands being received or feedback on the reasons why they are dropped
- no command buffer and no indication of command being completed, which introduces unnecessary complexity to the application software
- too high-level with too little features - the firmware implements forward and inverse kinematics, but, of course, no trajectory planning, which puts it into the "toy" category
- absurd - 3d printing (one of the claims on the kickstarter campaign)... sigh, relative accuracy (not 0.2mm for sure and only under specific conditions) can be achieved only at extremely low velocities... it is a torture to look at... a cheap 3d printer would do much better job and much quicker

### <a name="arduino-joke"></a> The "Arduino-based" joke
The fact that the Arduino board does not actually control the arm but delegates the control function to the FPGA using a proprietary protocol, brings no value of having Arduino at all. It could in fact be as well any other hardware - we don't get full control over the robot either way.

---

# <a name="supported-platforms"></a> Supported platforms

### <a name="hardware-platforms"></a> Hardware
There are two parts:
1. The one that drives the arm (stepper motors, servos, pump, valve, laser) - Control Box
2. The one that drives Control Box - a computer with a USB port

Original _Control Box_ is composed of two boards: Arduino and a board hosting FPGA chip (the one that actually drives everything).
Given the hard limitations to extend any functionality with FPGA board open-dobot since version 1.2 also supports **[RAMPS v1.4](http://reprap.org/wiki/RAMPS_1.4)** as a replacement for FPGA board. **RAMPS** is a cheap board (around $10) and removes any dependencies on and limitations of the FPGA board.

To drive RAMPS or FPGA board almost any computer can be used, including **Rapberry Pi**.

### <a name="software-platforms"></a> Software Platforms
Any platform where Python can run is supported. Tested on the following:
- Linux
- Windows
- Mac
- Rapsberry Pi (well, also Linux)

---

# Features implemented so far
- direct control over the FPGA/RAMPS board from application level (DobotDriver.py, DobotSDK.py)
- reliable and fast communication to the host system via USB
- command buffering (queueing) in firmware (in Arduino) for smooth, non-jerky moves
- move each joint stepper motor by specified number of steps, in specified direction and at specified speed
- 100% accurate moves in steps
- limit switch/photointerrupter support with calibration routine implemented in firmware and controlled from application level (DobotDriver.py, DobotSDK.py) by selecting any of the unused Arduino pins dynamically
- accurate and fast Inverse Kinematics
- 100% accurate step number tracking in SDK and in firmware
- accurate (to the best of Dobot's mechanical design) moves in cartesian coordinates (x,y,z) in a straight line from current location to the specified location. There is a flaw in mechanical design - terrible backlash in joints and motor reduction gears
- laser on/off with correct queueing
- pump and valve on/off with correct queueing
- smooth moves with acceleration/deceleration
- gripper control
- ```wait``` command to introduce a delay in the movement/manipulation sequence if needed
- support for MPU-6050 accelerometer units (GY-521 module) in RAMPS version

---

# Prerequisites
For FPGA v1.0 board you will only need:
- [DobotTools](http://dobot.cc/download.php) or avrdude directly (included in Arduino IDE) to flash _open-dobot_ firmware
- [Python](https://www.python.org) to run application software and examples (see [Installation](#installation))

FPGA v1.1 board is based on unknown crap accelerometers (MP65) that sit on SPI bus. Those are not supported.
Preferred alternative is to switch to RAMPS v1.4 board and use GY-521 modules.

Some other solutions to the accelerometers problem:
- use limit switches/photointerrupters to calibrate all joints
- request support for GY-521 modules in FPGA version and solder those modules to the Arduino pins exposed on the FPGA board

---

# Installation

For **RAMPS** wiring refer to [wiki](https://github.com/maxosprojects/open-dobot/wiki/).

### Python
You will need Python 2.7 or Python 3.x. Download [here](https://www.python.org) and install.

You will also need ```pyserial``` module.

If pyserial is not installed (exceptions are thrown when executing examples referring to not being able to find ```serial``` module) install it using ```pip``` (```pip``` should be installed with Python in your system):

On Windows:

```pip install pyserial```

On *nix you may need to do that with sudo (depends on how Python/pip was installed):

```sudo pip install pyserial```

#### Permanent Device Name on Linux
On Linux, you can use udev-rules to create a permanent device name for the dobot: [udev-rules](https://github.com/maxosprojects/open-dobot/wiki/)

### Firmware
Always prefer latest [Release](https://github.com/maxosprojects/open-dobot/releases) version, for everything - the compiled firmware, source code and examples. 
Unless you know what you are doing and want to try out experimental features DON'T use the code in any of the branches, including *master*.

Using original [DobotTools](http://dobot.cc/download.php) or [avrdude](http://www.nongnu.org/avrdude) directly (included in Arduino IDE distribution) flash the firmware dobot-x.x.x.hex to Dobot's Arduino board via the USB cable. If using DobotTools then the *.hex file must be copied into the "hex" folder inside DobotTools.
Alternatively, you can compile and flash the firmware yourself on a \*nix machine using a simple build script firmware/upload/upload*.sh that refers to avrdude already installed with Arduino IDE.

# Usage
In application/python/ folder you may find the calibrate-accelerometers.py to find the offsets of the accelerometers installed on your dobot. Every accelerometer is soldered at a slight angle, which needs to be accounted for when performing calculations. Although that angle is very small, at longer distances (when the arm is stretched) is might come in the way of holding XY plane.
However, as it is explained in [issue #19](https://github.com/maxosprojects/open-dobot/issues/19), the accelerometers themselves have a significant sensing error and the calibration may be skipped altogether given the accelerometers on your Dobot are not terribly poorly soldered (about one degree inclination relative to the PCB the accelerometer is soldered to will be fine).

In the same folder you may find some examples that use the driver directly, SDK and more. Read the descriptions in those examples before executing.

Have fun!

PS: Don't forget to leave comments, suggestions, etc., and check updates on the [forum](http://open-dobot.boards.net).

---

# Details
### Hardware

As mentioned above, there are two options: use original FPGA board or replace it with **RAMPS**.

For instructions on how to set up and wire RAMPS refer to [wiki](https://github.com/maxosprojects/open-dobot/wiki/).

### Firmware
1. Design:
    - [Protocol](https://github.com/maxosprojects/open-dobot/wiki/)
    - Firmware (follow up on [wiki](https://github.com/maxosprojects/open-dobot/wiki) and [forum](http://open-dobot.boards.net))
2. Implementation

### Application software
There are plenty of open-source projects implementing forward and inverse kinematics, trajectory planning and execution that could be used with the arm. One example would be http://moveit.ros.org . A simple camera would bring perception, which increases the number of arm applications by orders of magnitude.

Official open-dobot forum is at http://open-dobot.no-ip.org

# Table of Contents
1. [What is open-dobot](#what-is-open-dobot)
2. [Why?](#why)
   1. [Protocol](#protocol)
   2. [The "Arduino-based" joke](#arduino-joke)
3. [Features implemented so far](#features-implemented-so-far)
4. [Planned features](http://open-dobot.no-ip.org)
5. [Prerequisites](#prerequisites)
6. [Disclaimer](#disclaimer)
7. [Installation](#installation)
8. [Details](#details)

# What is open-dobot
This project is intended as a completely open and free (MIT License) alternative to proprietary, unreliable and inflexible firmware for Dobot robotic arm (original Kickstarter campaign https://www.kickstarter.com/projects/dobot/dobot-robotic-arm-for-everyone-arduino-and-open-so/description).

---

# <a name="why"></a> Why? ...or the original firmware problems
The need for this project arisen due to the original claim on Kickstarter to make the arm "Open-source" and "Arduino-based", which has been mostly unsatisfied.
The "openness" ends with the protocol that is used to send commands to the arm. The "Arduino-based" claim brings absolutely no benefit to the owner as it does not drive the arm but sends commands further to the top board via SPI. The top board has FPGA that controls the three stepper drivers (PWM signals).
### Protocol
The issues with the original protocol the Dobot team has released are all in the proprietary firmware, which renders the protocol useless:
- the firmware drops commands randomly
- provides no acknowledgement of commands being received or feedback on the reasons why they are dropped
- no command buffer and no indication of command being completed, which introduces unnecessary complexity to the application software
- too high-level with too little features - the firmware implements forward and inverse kinematics, but, of course, no trajectory planning, which puts it into the "toy" category
- absurd - 3d printing (one of the claims on the kickstarter campaign) with delays between commands and commands dropping... no comments

### <a name="arduino-joke"></a> The "Arduino-based" joke
The fact that the Arduino board does not actually control the arm but delegates the control function to the FPGA using a proprietary protocol, brings no benefit of having Arduino at all. It could in fact be as well any other hardware don't have control over the arm either way.

---

# Features implemented so far
- direct control over the FPGA board from application level (DobotDriver.py)
- reliable and fast communication to the host system via USB
- command buffering (queueing) in firmware (in Arduino) for smooth, non-jerky moves
- move each joint stepper motor by specified number of steps, in specified direction and at specified speed
- 100% accurate moves in steps
- limit switch/photointerrupter support with calibration routing implemented in firmware and controlled from application level (DobotDriver.py, DobotSDK.py) by selecting any of the unused Arduino pins dynamically
- accurate and fast Inverse Kinematics
- 100% accurate step number tracking in SDK and in firmware (coming in 0.6.0)
- accurate (to the best of Dobot's mechanical design) moves in cartesian coordinates (x,y,z) in a straight line from current location to the specified location (coming in 0.6.0)
- laser on/off (coming in 0.6.0)

---

# Prerequisites
~~The simplest way to overcome the above problems and bring full control over the arm seems to be by replacing the top board (that contains FPGA) with a cheap (around $10) RAMPS1.4 board. By doing so we get direct control of the stepper drivers from Arduino and the rest becomes a matter of creating a simple firmware.~~

**With the latest advancements in figuring out the FPGA on the top board firmware/fpga version 0.2.0 requires NO changes to hardware and is ready to replace original firmware by simply uploading it with dobot-tools https://github.com/maxosprojects/open-dobot/releases/tag/0.2.0**

You will only need:
- [DobotTools](http://dobot.cc/download.php) or avrdude directly (included in Arduino IDE) to flash the firmware
- [Python](https://www.python.org) to run application software and examples

---

# Disclaimer
The information provided here is a collective effort of enthusiasts who wanted to make the Dobot arm useful for themselves and others. The instructions are provided without pursuing any personal or material benefit, AS IS and to the best of the knowledge of the people involved in this project as to not cause any damage or harm with hardware and software manipulations, but to only bring the arm to life, make it useful and reliable, squeezing out everything from every dollar that has been paid for the arm. That being said, following these instructions strictly or (especially) not strictly you take any risk and full responsibility for any damage or harm that it might lead to.

---

# Installation
Always prefer latest [Release](https://github.com/maxosprojects/open-dobot/releases) version, for everything - the compiled firmware, source code and examples. 
Unless you know what you are doing and want to try out experimental features DON'T use the code in any of the branches, including *master*.

Using original [DobotTools](http://dobot.cc/download.php) or [avrdude](http://www.nongnu.org/avrdude) directly (included in Arduino IDE distribution) flash the firmware dobot-x.x.x.hex to Dobot's Arduino board via the USB cable. If using DobotTools then the .hex file must be copied into the "hex" folder inside DobotTools.
Alternatively, you can compile and flash the firmware yourself using a simple build script firmware/fpga/upload.sh that refers to avrdude already installed with Arduino IDE.
In application/fpga/python/ folder you may find examples that use the driver directly, SDK and more. Read the descriptions in those examples before executing.

That is all to installation. Have fun!

Don't forget to leave comments, suggestions, etc., and check updates on the [forum](http://open-dobot.no-ip.org).

---

# Details
### Hardware
~~First, an overview of a quick and dirty solution proving the approach for this project feasible:
https://www.reddit.com/r/Dobot/comments/45ilan/controlling_dobot_with_ramps_14/
http://imgur.com/a/J7eaU
https://gist.github.com/erwincoumans/7433a9ea7951932b9d79
Don't mind the soldering, it may not be required at all.~~

~~Hardware changes (immediate and planned):
1. As stated above, the top board (the one with the Actel FPGA chip) is to be replaced with a cheap (around $10) RAMPS1.4 board. Step-by-step instructions with pictures will be provided starting from what board to buy (with examples on amazon/ebay)
2. The stepper drivers from the FPGA board are to be moved to RAMPS. There are 3 stepper drivers sitting on the FPGA board - you won't miss them. They are fairly easily taken out from the FPGA board and put on RAMPS.
3. Planned - connect end effectors (mechanical gripper, vacuum gripper, laser, etc.)
4. Planned - connect the accelerometers. This will enable firmware to read the current (initial or at any time at rest) arm configuration~~

**firmware/fpga version 0.2.0 requires NO changes to hardware and is ready to replace original firmware by simply uploading it with dobot-tools https://github.com/maxosprojects/open-dobot/releases/tag/0.2.0**

### Firmware
1. Design:
    - [Protocol](https://github.com/maxosprojects/open-dobot/wiki/Protocol-Design)
    - Firmware (follow up on [wiki](https://github.com/maxosprojects/open-dobot/wiki) and [forum](http://open-dobot.no-ip.org))
2. Implementation

### Application software
There are plenty of open-source projects implementing forward and inverse kinematics, trajectory planning and execution that could be used with the arm. One example would be http://moveit.ros.org . A simple camera would bring perception, which increases the number of arm applications by orders of magnitude.

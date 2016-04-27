#! /usr/bin/env python
'''
Simple demostration of open-dobot. High-level control via SDK.

It is assumed that no end effectors are installed on the arm (no gripper or sucker, etc.)
as end effector would crash into the desk Dobot stands on.

Refer to SDK to find the expected initial arm configuration.

Each move is split into a series of commands to send via driver to FPGA. Each command is
executed for 20ms. Dobot, thus, executes 50 commands per second.

The commands are queued in Arduino, hence when you stop this example dobot will continue
to execute until the queue (200 commands) is empty.

'''

from DobotDriver import DobotDriver as Dobot
import time

# The top Z to go to.
up = 180
# The bottom Z to go to.
down = 150
# Maximum speed in mm/s
speed = 700
# Acceleration in mm/s^2
acceleration = 400

# dobot = Dobot('/dev/tty.usbmodem1421', debug=True, fake=True)
# dobot = Dobot('/dev/tty.usbmodem1421', debug=True)

dobot = Dobot('/dev/tty.usbmodem1421')
dobot.Open()

# Enable calibration routine if you have a limit switch/photointerrupter installed on the arm.
# See example-switch.py for details.
# Move both arms to approximately 45 degrees.
# dobot.moveWithSpeed(260.0, 0.0, 145, 700, acceleration)
# time.sleep(2)
# dobot.CalibrateJoint(1, dobot.freqToCmdVal(2000), dobot.freqToCmdVal(50), 1, 5, 1, 0)

for i in range(0xffff):
	rev = dobot.reverseBits16(i)
	print dobot.Steps(0, 0, 0, 0, 0, 0, 0, rev)

# Rectangle with zig-zag inside
# dobot.moveWithSpeed(200.0, -90.0, up, speed, acceleration)
# dobot.moveWithSpeed(200.0, -90.0, down, speed, acceleration)
# dobot.moveWithSpeed(200.0, 80.0, down, speed, acceleration)
# dobot.moveWithSpeed(260.0, 80.0, down, speed, acceleration)
# dobot.moveWithSpeed(260.0, -90.0, down, speed, acceleration)
# dobot.moveWithSpeed(200.0, -90.0, down, speed, acceleration)
# dobot.moveWithSpeed(260.0, 0.0, 145, 700, acceleration)

